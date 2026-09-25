/*
 * WATERSTAMP v1 video emitter -- invisible spread-spectrum time stamp.
 * Copyright (c) 2026 nxtedition
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Add the waterstamp absolute-time stamp to the luma of every frame.
 *
 * The same 36-bit frame, ids, codes and key as the audio filter. The code
 * is laid on a 33 x 31 cell grid over the frame in normalised coordinates,
 * one chip per cell, multiplied within each cell by a balanced 4 x 4 Walsh
 * carrier. Layer A cycles through 32 cyclic shifts per 500 ms slot (one
 * per 15.625 ms chip, read from the frame's time), layer B holds the
 * slot's symbol shift plus the same per-chip advance. The amplitude follows the local texture of each
 * cell, clamped in LSB, and sub-LSB amplitudes are applied by a fixed
 * spatial dither. Absolute time is t0 plus the frame timestamp, where t0
 * (the time of media timestamp 0) comes from the option or from a
 * wall-clock anchor shared with the other emitters of the process.
 */

#include <math.h>

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/time.h"
#include "avfilter.h"
#include "filters.h"
#include "video.h"
#include "waterstamp.h"

typedef struct VWaterStampContext {
    const AVClass *class;

    double  level;          /* dB relative to the cell's luma std dev */
    double  floor_lsb;      /* minimum amplitude, LSB                */
    double  ceil_lsb;       /* maximum amplitude, LSB                */
    int     id;
    int64_t opt_t0;
    char   *group;
    char   *key;
    size_t  keylen;
    int     mode;
    double  test_level;     /* LSB, fixed amplitude in test mode      */

    float   lin_level;
    int8_t  seqA[WS_SEQ_LEN];
    int8_t  seqB[WS_SEQ_LEN];
    uint8_t bits[WS_BITS];
    int64_t cur_frame;
    int64_t cur_slot;
    int     shiftB;

    int     anchored;
    int64_t t0_us;          /* absolute time of media timestamp 0     */

    int     w, h;
    int     depth;              /* luma bit depth; amplitudes are in 8-bit LSB */
    int     full_range;         /* else stay inside the 16..235 video range   */
    int    *colsub, *rowsub;    /* pixel -> sub-cell column / row         */
    double *sum, *sumsq;        /* per cell                               */
    int    *cnt;
    float  *amp;                /* per cell: 0.5 * chip * amplitude, LSB  */
    int8_t *carrier;            /* [cell][sy*4+sx]                        */
    float  *dither;             /* 256 x 256 tile                         */
} VWaterStampContext;

enum { MODE_NORMAL, MODE_TEST, NB_MODES };

#define OFFSET(x) offsetof(VWaterStampContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM

static const AVOption vwaterstamp_options[] = {
    { "level", "amplitude in dB relative to the cell's luma standard deviation", OFFSET(level),     AV_OPT_TYPE_DOUBLE, {.dbl = -26},  -80,  0, FLAGS },
    { "floor", "minimum amplitude in LSB",                                       OFFSET(floor_lsb), AV_OPT_TYPE_DOUBLE, {.dbl = 0.1},    0,  8, FLAGS },
    { "ceil",  "maximum amplitude in LSB",                                       OFFSET(ceil_lsb),  AV_OPT_TYPE_DOUBLE, {.dbl = 1.5},    0, 16, FLAGS },
    { "id",    "source id (0-63), carried in the frame and selecting the code pair", OFFSET(id),   AV_OPT_TYPE_INT,    {.i64 = 0},      0, WS_MAX_IDS - 1, FLAGS },
    { "t0",    "absolute time of media timestamp 0 in microseconds, 0 = wall clock", OFFSET(opt_t0), AV_OPT_TYPE_INT64,  {.i64 = 0},      0, INT64_MAX, FLAGS },
    { "group", "emitters of one group share their wall-clock anchor",           OFFSET(group),     AV_OPT_TYPE_STRING, {.str = "default"}, 0, 0, FLAGS },
    { "key",   "secret key: derives the code pair and the frame check field",   OFFSET(key),       AV_OPT_TYPE_STRING, {.str = NULL},   0,  0, FLAGS },
    { "mode",  "normal: invisible, amplitude follows texture; test: fixed visible amplitude for QC feeds", OFFSET(mode), AV_OPT_TYPE_INT, {.i64 = MODE_NORMAL}, 0, NB_MODES - 1, FLAGS, .unit = "mode" },
    { "normal", "invisible, amplitude relative to local texture",             0,                 AV_OPT_TYPE_CONST,  {.i64 = MODE_NORMAL}, 0, 0, FLAGS, .unit = "mode" },
    { "test",  "fixed amplitude, visible, survives hard compression",         0,                 AV_OPT_TYPE_CONST,  {.i64 = MODE_TEST},   0, 0, FLAGS, .unit = "mode" },
    { "test_level", "fixed amplitude in LSB used by mode=test",               OFFSET(test_level), AV_OPT_TYPE_DOUBLE, {.dbl = 8},     0.5, 64, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(vwaterstamp);

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ444P,
    AV_PIX_FMT_NV12, AV_PIX_FMT_NV21, AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10,
    AV_PIX_FMT_YUV420P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV444P12,
    AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12,
    AV_PIX_FMT_NONE
};

static av_cold int init(AVFilterContext *ctx)
{
    VWaterStampContext *s = ctx->priv;

    s->keylen = s->key ? strlen(s->key) : 0;
    if (ws_sequences((const uint8_t *)s->key, s->keylen, s->id, s->seqA, s->seqB) < 0) {
        av_log(ctx, AV_LOG_ERROR, "could not build the spreading codes\n");
        return AVERROR_BUG;
    }
    if (s->floor_lsb > s->ceil_lsb) {
        av_log(ctx, AV_LOG_ERROR, "floor must not exceed ceil\n");
        return AVERROR(EINVAL);
    }
    s->lin_level = powf(10.f, s->level / 20.f);
    s->cur_frame = INT64_MIN;
    s->cur_slot  = INT64_MIN;

    s->sum   = av_calloc(WSV_CELLS, sizeof(*s->sum));
    s->sumsq = av_calloc(WSV_CELLS, sizeof(*s->sumsq));
    s->cnt   = av_calloc(WSV_CELLS, sizeof(*s->cnt));
    s->amp   = av_calloc(WSV_CELLS, sizeof(*s->amp));
    s->carrier = av_malloc(WSV_CELLS * WSV_SUB * WSV_SUB);
    s->dither  = av_malloc_array(256 * 256, sizeof(*s->dither));
    if (!s->sum || !s->sumsq || !s->cnt || !s->amp || !s->carrier || !s->dither)
        return AVERROR(ENOMEM);
    for (int c = 0; c < WSV_CELLS; c++)
        for (int sy = 0; sy < WSV_SUB; sy++)
            for (int sx = 0; sx < WSV_SUB; sx++)
                s->carrier[c * WSV_SUB * WSV_SUB + sy * WSV_SUB + sx] = ws_carrier(c, sx, sy);
    for (int y = 0; y < 256; y++)
        for (int x = 0; x < 256; x++)
            s->dither[y * 256 + x] = ws_dither(x, y);
    return 0;
}

static int config_input(AVFilterLink *inlink)
{
    VWaterStampContext *s = inlink->dst->priv;

    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(inlink->format);

    s->w = inlink->w;
    s->h = inlink->h;
    s->depth      = desc->comp[0].depth;
    s->full_range = inlink->color_range == AVCOL_RANGE_JPEG ||
                    inlink->format == AV_PIX_FMT_YUVJ420P ||
                    inlink->format == AV_PIX_FMT_YUVJ422P ||
                    inlink->format == AV_PIX_FMT_YUVJ444P;
    av_freep(&s->colsub);
    av_freep(&s->rowsub);
    s->colsub = av_malloc_array(s->w, sizeof(*s->colsub));
    s->rowsub = av_malloc_array(s->h, sizeof(*s->rowsub));
    if (!s->colsub || !s->rowsub)
        return AVERROR(ENOMEM);
    for (int x = 0; x < s->w; x++)
        s->colsub[x] = (int)((int64_t)x * WSV_SUBCOLS / s->w);
    for (int y = 0; y < s->h; y++)
        s->rowsub[y] = (int)((int64_t)y * WSV_SUBROWS / s->h);
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    VWaterStampContext *s = ctx->priv;
    av_freep(&s->colsub);
    av_freep(&s->rowsub);
    av_freep(&s->sum);
    av_freep(&s->sumsq);
    av_freep(&s->cnt);
    av_freep(&s->amp);
    av_freep(&s->carrier);
    av_freep(&s->dither);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx  = inlink->dst;
    VWaterStampContext *s = ctx->priv;
    const int   hbd   = s->depth > 8;
    const float scale = 1 << (s->depth - 8);           /* native units per 8-bit LSB */
    const int   lo    = s->full_range ? 0 : 16 << (s->depth - 8);
    const int   hi    = s->full_range ? (1 << s->depth) - 1 : 235 << (s->depth - 8);
    int64_t t_us, pts_us, slot, frm;
    int si, kA, shiftA, ret;
    uint8_t *y;
    int ls;

    if (frame->pts == AV_NOPTS_VALUE) {
        av_log(ctx, AV_LOG_WARNING, "frame without timestamp, passed through unstamped\n");
        return ff_filter_frame(ctx->outputs[0], frame);
    }
    pts_us = av_rescale_q(frame->pts, inlink->time_base, AV_TIME_BASE_Q);
    if (!s->anchored) {
        s->t0_us     = s->opt_t0 ? s->opt_t0 : ff_waterstamp_wall_anchor(s->group, pts_us);
        s->anchored  = 1;
        av_log(ctx, AV_LOG_INFO,
               "vwaterstamp anchored t0:%"PRId64".%06d id:%d keyed:%d mode:%s level:%g floor:%g ceil:%g\n",
               s->t0_us / 1000000, (int)(s->t0_us % 1000000), s->id, s->keylen > 0,
               s->mode == MODE_TEST ? "test" : "normal",
               s->mode == MODE_TEST ? s->test_level : s->level, s->floor_lsb, s->ceil_lsb);
    }
    t_us = s->t0_us + pts_us;
    if (t_us < 0)
        return ff_filter_frame(ctx->outputs[0], frame);

    slot = t_us / WS_SLOT_US;
    frm  = slot / WS_SLOTS;
    si   = (int)(slot - frm * WS_SLOTS);
    if (frm != s->cur_frame) {
        ws_frame_bits(frm * WS_FRAME_S, s->id, s->bits, (const uint8_t *)s->key, s->keylen);
        s->cur_frame = frm;
    }
    if (slot != s->cur_slot) {
        s->shiftB   = ws_symbol(s->bits, si) * WS_CSK_STEP;
        s->cur_slot = slot;
    }
    kA     = (int)((t_us - slot * WS_SLOT_US) / WSV_CHIP_US);
    shiftA = kA * WSV_A_STEP;

    if ((ret = ff_inlink_make_frame_writable(inlink, &frame)) < 0)
        return ret;
    y  = frame->data[0];
    ls = frame->linesize[0];

    /* 1. local texture per cell */
    memset(s->sum,   0, WSV_CELLS * sizeof(*s->sum));
    memset(s->sumsq, 0, WSV_CELLS * sizeof(*s->sumsq));
    memset(s->cnt,   0, WSV_CELLS * sizeof(*s->cnt));
    for (int j = 0; j < s->h; j++) {
        const uint8_t  *row   = y + (ptrdiff_t)j * ls;
        const uint16_t *row16 = (const uint16_t *)row;
        int r = s->rowsub[j] / WSV_SUB;
        int64_t sum = 0, sumsq = 0;
        int prev = -1, c = 0;
        for (int i = 0; i < s->w; i++) {
            int cc = r * WSV_COLS + s->colsub[i] / WSV_SUB;
            int v = hbd ? row16[i] : row[i];
            if (cc != prev) {
                if (prev >= 0) { s->sum[prev] += sum; s->sumsq[prev] += sumsq; }
                sum = sumsq = 0;
                prev = cc;
            }
            sum   += v;
            sumsq += v * v;
            s->cnt[cc]++;
            c = cc;
        }
        s->sum[c] += sum;
        s->sumsq[c] += sumsq;
    }
    for (int c = 0; c < WSV_CELLS; c++) {
        double m  = s->cnt[c] ? s->sum[c] / s->cnt[c] : 0;
        double v  = s->cnt[c] ? s->sumsq[c] / s->cnt[c] - m * m : 0;
        int chip  = s->seqA[(c + shiftA) % WS_SEQ_LEN] + s->seqB[(c + s->shiftB + shiftA) % WS_SEQ_LEN];
        s->amp[c] = 0.5f * chip * scale *
                    (s->mode == MODE_TEST ? (float)s->test_level
                     : av_clipf(sqrt(FFMAX(v, 0)) / scale * s->lin_level, s->floor_lsb, s->ceil_lsb));
    }

    /* 2. add chip * carrier * amplitude, dithered below one LSB, never
     * pushing a sample out of the legal range it was in */
    for (int j = 0; j < s->h; j++) {
        uint8_t  *row   = y + (ptrdiff_t)j * ls;
        uint16_t *row16 = (uint16_t *)row;
        const float *dit = s->dither + (j & 255) * 256;
        int sr = s->rowsub[j], r = sr / WSV_SUB, sy = sr % WSV_SUB;
        for (int i = 0; i < s->w; i++) {
            int sc = s->colsub[i], c = r * WSV_COLS + sc / WSV_SUB, sx = sc % WSV_SUB;
            float d, f;
            int   n;
            if (s->amp[c] == 0.f)
                continue;
            d = s->carrier[c * WSV_SUB * WSV_SUB + sy * WSV_SUB + sx] * s->amp[c];
            n = (int)d;
            f = d - n;
            if (dit[i & 255] < (f < 0 ? -f : f))
                n += d > 0 ? 1 : -1;
            if (n) {
                const int v = hbd ? row16[i] : row[i];
                const int o = av_clip(v + n, FFMIN(v, lo), FFMAX(v, hi));
                if (hbd)
                    row16[i] = o;
                else
                    row[i] = o;
            }
        }
    }
    return ff_filter_frame(ctx->outputs[0], frame);
}

static const AVFilterPad vwaterstamp_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_vf_vwaterstamp = {
    .p.name        = "vwaterstamp",
    .p.description = NULL_IF_CONFIG_SMALL("Add an invisible spread-spectrum time stamp to video."),
    .p.priv_class  = &vwaterstamp_class,
    .priv_size     = sizeof(VWaterStampContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(vwaterstamp_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
};

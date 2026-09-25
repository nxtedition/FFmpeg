/*
 * WATERSTAMP v1 emitter -- inaudible spread-spectrum time stamp.
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
 * Mix an inaudible spread-spectrum absolute-time stamp into audio.
 *
 * Absolute time is anchored once, at the first frame, to that frame's
 * timestamp (t0 is the time of media timestamp 0, from the option or a
 * wall-clock anchor shared with the other emitters of the process), and
 * then advanced by the running output sample count, never by PTS, so the
 * stamp measures the audio's own timeline: any downstream sample-rate error then
 * shows up at the detector as a slope, which is how fixed delay and drift
 * are told apart.
 *
 * The same waveform is added to every channel at identical phase, so a
 * downstream downmix sums it coherently. Its level follows the programme
 * band of the mix, but never exceeds a channel's own band level, so a
 * quiet channel is stamped relative to itself; a channel below the gate
 * (digital silence, an unused track) and LFE are left untouched.
 */

#include <math.h>

#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "waterstamp.h"

typedef struct WaterStampContext {
    const AVClass *class;

    /* options */
    double level;       /* dB relative to programme band RMS   */
    double floor_db;    /* dBFS absolute minimum               */
    double ceil_db;     /* dBFS absolute maximum               */
    int    id;          /* source id, carried in the frame     */
    int64_t opt_t0;     /* absolute time of media timestamp 0, us; 0 = wall clock */
    char   *group;      /* emitters sharing one wall-clock anchor */
    double attack;      /* level detector attack, s            */
    double release;     /* level detector release, s           */
    double gate_db;     /* dBFS: quieter channels get no stamp  */
    char  *key;         /* secret: keys codes and check field   */
    size_t keylen;
    int    mode;        /* MODE_NORMAL or MODE_TEST              */
    double test_level;  /* dBFS, fixed amplitude in test mode    */
    float  lin_test;

    /* derived */
    float lin_level, lin_floor, lin_ceil, gate_pow;
    float a_att, a_rel, a_gate;
    float b0, b2, a1, a2;   /* RBJ band-pass, DF2T; b1 == 0     */
    float z1, z2;
    float env;

    /* per channel: band-pass state, band and broadband envelopes, gate */
    int    nb_ch;
    float *cz1, *cz2, *cenv, *cbb, *cgain;
    uint8_t *lfe;

    int8_t  seqA[WS_SEQ_LEN];
    int8_t  seqB[WS_SEQ_LEN];
    uint8_t bits[WS_BITS];
    int64_t cur_frame;
    int64_t cur_slot;
    int     shift;

    int     anchored;
    int64_t t0_us;      /* absolute time of output sample 0    */
    int64_t n;          /* output samples emitted so far       */

    float  *tmp;        /* per-sample work buffer: unit waveform */
    float  *mixenv;     /* per-sample band power of the mix      */
    float  *gains;      /* per-sample, per-channel gate gain     */
    int     tmp_size;
} WaterStampContext;

enum { MODE_NORMAL, MODE_TEST, NB_MODES };

#define OFFSET(x) offsetof(WaterStampContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_AUDIO_PARAM

static const AVOption waterstamp_options[] = {
    { "level",   "watermark level in dB relative to programme band RMS", OFFSET(level),    AV_OPT_TYPE_DOUBLE, {.dbl = -23},   -80,   0, FLAGS },
    { "floor",   "absolute minimum level in dBFS",                       OFFSET(floor_db), AV_OPT_TYPE_DOUBLE, {.dbl = -65},  -120,   0, FLAGS },
    { "ceil",    "absolute maximum level in dBFS",                       OFFSET(ceil_db),  AV_OPT_TYPE_DOUBLE, {.dbl = -35},  -120,   0, FLAGS },
    { "id",      "source id (0-63), carried in the frame and selecting the code pair", OFFSET(id), AV_OPT_TYPE_INT, {.i64 = 0}, 0, WS_MAX_IDS - 1, FLAGS },
    { "t0",      "absolute time of media timestamp 0 in microseconds, 0 = wall clock", OFFSET(opt_t0), AV_OPT_TYPE_INT64, {.i64 = 0}, 0, INT64_MAX, FLAGS },
    { "group",   "emitters of one group share their wall-clock anchor", OFFSET(group), AV_OPT_TYPE_STRING, {.str = "default"}, 0, 0, FLAGS },
    { "attack",  "level detector attack in seconds",                     OFFSET(attack),   AV_OPT_TYPE_DOUBLE, {.dbl = 0.005}, 0.0001, 5, FLAGS },
    { "release", "level detector release in seconds, keep >= one 0.5 s slot", OFFSET(release), AV_OPT_TYPE_DOUBLE, {.dbl = 0.2}, 0.01, 30, FLAGS },
    { "gate",    "channels whose level stays below this (dBFS) are not stamped", OFFSET(gate_db), AV_OPT_TYPE_DOUBLE, {.dbl = -80}, -200, 0, FLAGS },
    { "key",     "secret key: derives the code pair and the frame check field, so only a detector with the same key reads the stamp", OFFSET(key), AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS },
    { "mode",    "normal: inaudible, level follows the programme; test: fixed audible level for QC feeds", OFFSET(mode), AV_OPT_TYPE_INT, {.i64 = MODE_NORMAL}, 0, NB_MODES - 1, FLAGS, .unit = "mode" },
    { "normal",  "inaudible, level relative to the programme band",     0,                AV_OPT_TYPE_CONST,  {.i64 = MODE_NORMAL}, 0, 0, FLAGS, .unit = "mode" },
    { "test",    "fixed level, audible, survives hard compression",     0,                AV_OPT_TYPE_CONST,  {.i64 = MODE_TEST},   0, 0, FLAGS, .unit = "mode" },
    { "test_level", "fixed amplitude in dBFS used by mode=test",        OFFSET(test_level), AV_OPT_TYPE_DOUBLE, {.dbl = -20}, -60, 0, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(waterstamp);

static av_cold int init(AVFilterContext *ctx)
{
    WaterStampContext *s = ctx->priv;

    s->keylen = s->key ? strlen(s->key) : 0;
    if (ws_sequences((const uint8_t *)s->key, s->keylen, s->id, s->seqA, s->seqB) < 0) {
        av_log(ctx, AV_LOG_ERROR, "could not build the spreading codes\n");
        return AVERROR_BUG;
    }
    if (s->floor_db > s->ceil_db) {
        av_log(ctx, AV_LOG_ERROR, "floor (%g dBFS) must not exceed ceil (%g dBFS)\n",
               s->floor_db, s->ceil_db);
        return AVERROR(EINVAL);
    }
    s->lin_level = powf(10.f, s->level    / 20.f);
    s->lin_test  = powf(10.f, s->test_level / 20.f);
    s->lin_floor = powf(10.f, s->floor_db / 20.f);
    s->lin_ceil  = powf(10.f, s->ceil_db  / 20.f);
    s->gate_pow  = powf(10.f, s->gate_db  / 10.f);
    s->cur_frame = INT64_MIN;
    s->cur_slot  = INT64_MIN;
    return 0;
}

static void free_channels(WaterStampContext *s)
{
    av_freep(&s->cz1);
    av_freep(&s->cz2);
    av_freep(&s->cenv);
    av_freep(&s->cbb);
    av_freep(&s->cgain);
    av_freep(&s->lfe);
}

static int config_input(AVFilterLink *inlink)
{
    WaterStampContext *s = inlink->dst->priv;
    const double sr = inlink->sample_rate;
    /* RBJ band-pass, constant 0 dB peak gain, f0 = carrier, Q = 1 */
    const double w0    = 2.0 * M_PI * WS_CARRIER / sr;
    const double alpha = sin(w0) / 2.0;
    const double a0    = 1.0 + alpha;

    if (sr < 2.0 * (WS_CARRIER + WS_CHIP_RATE / 2.0)) {
        av_log(inlink->dst, AV_LOG_ERROR,
               "sample rate %d Hz cannot carry the %g Hz band\n",
               inlink->sample_rate, WS_CARRIER + WS_CHIP_RATE / 2.0);
        return AVERROR(EINVAL);
    }

    s->b0 = alpha / a0;
    s->b2 = -alpha / a0;
    s->a1 = -2.0 * cos(w0) / a0;
    s->a2 = (1.0 - alpha) / a0;
    s->z1 = s->z2 = 0.f;
    s->env = 0.f;

    s->a_att  = 1.f - expf(-1.f / (float)(s->attack  * sr));
    s->a_rel  = 1.f - expf(-1.f / (float)(s->release * sr));
    s->a_gate = 1.f - expf(-1.f / (float)(0.01 * sr));   /* 10 ms gate ramp */

    s->nb_ch = inlink->ch_layout.nb_channels;
    free_channels(s);
    s->cz1   = av_calloc(s->nb_ch, sizeof(*s->cz1));
    s->cz2   = av_calloc(s->nb_ch, sizeof(*s->cz2));
    s->cenv  = av_calloc(s->nb_ch, sizeof(*s->cenv));
    s->cbb   = av_calloc(s->nb_ch, sizeof(*s->cbb));
    s->cgain = av_calloc(s->nb_ch, sizeof(*s->cgain));
    s->lfe   = av_calloc(s->nb_ch, sizeof(*s->lfe));
    if (!s->cz1 || !s->cz2 || !s->cenv || !s->cbb || !s->cgain || !s->lfe)
        return AVERROR(ENOMEM);
    for (int ch = 0; ch < s->nb_ch; ch++) {
        enum AVChannel c = av_channel_layout_channel_from_index(&inlink->ch_layout, ch);
        s->lfe[ch] = c == AV_CHAN_LOW_FREQUENCY || c == AV_CHAN_LOW_FREQUENCY_2;
    }
    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx  = inlink->dst;
    WaterStampContext *s    = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];
    const int nb_ch  = in->ch_layout.nb_channels;
    const int nb     = in->nb_samples;
    const int planar = av_sample_fmt_is_planar(in->format);
    const int stride = planar ? 1 : nb_ch;
    const double sr  = inlink->sample_rate;
    const double dt  = 1.0 / sr;
    AVFrame *out;
    float *w;
    int64_t t_us, slot;
    double tp;

    if (!s->anchored) {
        const int64_t pts_us = in->pts == AV_NOPTS_VALUE ? 0 :
                               av_rescale_q(in->pts, inlink->time_base, AV_TIME_BASE_Q);
        s->t0_us    = (s->opt_t0 ? s->opt_t0 : ff_waterstamp_wall_anchor(s->group, pts_us)) + pts_us;
        s->n        = 0;
        s->anchored = 1;
        av_log(ctx, AV_LOG_INFO,
               "waterstamp anchored t0:%"PRId64".%06d id:%d keyed:%d mode:%s level:%g floor:%g ceil:%g\n",
               s->t0_us / 1000000, (int)(s->t0_us % 1000000), s->id, s->keylen > 0,
               s->mode == MODE_TEST ? "test" : "normal",
               s->mode == MODE_TEST ? s->test_level : s->level, s->floor_db, s->ceil_db);
    }

    if (av_frame_is_writable(in)) {
        out = in;
    } else {
        int ret;
        out = ff_get_audio_buffer(outlink, nb);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        ret = av_frame_copy_props(out, in);
        if (ret < 0) {
            av_frame_free(&out);
            av_frame_free(&in);
            return ret;
        }
    }

    if (s->tmp_size < nb) {
        av_freep(&s->tmp);
        av_freep(&s->mixenv);
        av_freep(&s->gains);
        s->tmp    = av_malloc_array(nb, sizeof(*s->tmp));
        s->mixenv = av_malloc_array(nb, sizeof(*s->mixenv));
        s->gains  = av_malloc_array((size_t)nb * nb_ch, sizeof(*s->gains));
        if (!s->tmp || !s->mixenv || !s->gains) {
            if (out != in)
                av_frame_free(&out);
            av_frame_free(&in);
            s->tmp_size = 0;
            return AVERROR(ENOMEM);
        }
        s->tmp_size = nb;
    }
    w = s->tmp;

    /* 1. per-channel gate on the broadband level, then the band power of
     * the mix of the open non-LFE channels (a silent one must not dilute) */
    for (int i = 0; i < nb; i++) {
        float x = 0.f, n = 0.f, bp, e;
        for (int ch = 0; ch < nb_ch; ch++) {
            const float *src = planar ? (const float *)in->extended_data[ch]
                                      : (const float *)in->data[0] + ch;
            const float v = src[i * stride];
            float *bb = &s->cbb[ch], *g = &s->cgain[ch];
            if (s->lfe[ch])
                continue;
            e    = v * v;
            *bb += (e > *bb ? s->a_att : s->a_rel) * (e - *bb);
            *g  += s->a_gate * ((*bb > s->gate_pow ? 1.f : 0.f) - *g);
            x += *g * v;
            n += *g;
        }
        x = n > 1e-3f ? x / n : 0.f;
        bp    = s->b0 * x + s->z1;
        s->z1 = -s->a1 * bp + s->z2;
        s->z2 = s->b2 * x - s->a2 * bp;
        e = bp * bp;
        s->env += (e > s->env ? s->a_att : s->a_rel) * (e - s->env);
        s->mixenv[i] = s->env;
        for (int ch = 0; ch < nb_ch; ch++)
            s->gains[(size_t)i * nb_ch + ch] = s->cgain[ch];
    }

    /* 2. waveform, a pure function of absolute time. Absolute time of the
     * first sample is recomputed from integers every frame; within the
     * frame we step in double, which is exact enough over one frame and
     * cannot accumulate drift across frames. */
    t_us = s->t0_us + av_rescale(s->n, 1000000, inlink->sample_rate);
    if (t_us < 0) {                            /* before the time origin */
        s->n += nb;
        if (out != in)
            av_frame_free(&out);
        return ff_filter_frame(outlink, in);
    }
    slot = t_us / WS_SLOT_US;                  /* t0 >= 0, so this floors */
    tp   = (t_us - slot * WS_SLOT_US) * 1e-6;  /* time within slot, 0..0.5 */

    for (int i = 0; i < nb; i++, tp += dt) {
        int64_t frm;
        int si, ci;
        double cf;
        float u, a, b;

        if (tp >= WS_SLOT_US * 1e-6) {
            tp -= WS_SLOT_US * 1e-6;
            slot++;
        }
        frm = slot / WS_SLOTS;
        si  = (int)(slot - frm * WS_SLOTS);

        if (frm != s->cur_frame) {                      /* once per 6 s */
            ws_frame_bits(frm * WS_FRAME_S, s->id, s->bits, (const uint8_t *)s->key, s->keylen);
            s->cur_frame = frm;
        }
        if (slot != s->cur_slot) {                      /* once per 500 ms */
            s->shift    = ws_symbol(s->bits, si) * WS_CSK_STEP;
            s->cur_slot = slot;
        }

        cf = tp * WS_CHIP_RATE;
        ci = (int)cf;
        if (ci >= WS_SEQ_LEN)                          /* fp guard at slot end */
            ci = WS_SEQ_LEN - 1;
        u  = (float)(cf - ci);

        a = s->seqA[ci];
        b = s->seqB[(ci + s->shift) % WS_SEQ_LEN];
        /* carrier phase is periodic in the slot: 2000 Hz * 0.5 s = 1000 cycles */
        w[i]  = (a + b) * 0.70710678f
              * sinf((float)M_PI * u)
              * cosf((float)(2.0 * M_PI * WS_CARRIER * tp))
              * WS_NORM;
    }
    s->n += nb;

    /* 3. identical waveform on every channel; amplitude from the mix band
     * level, capped by the channel's own, gated on its broadband level */
    for (int ch = 0; ch < nb_ch; ch++) {
        const float *src = planar ? (const float *)in->extended_data[ch]
                                  : (const float *)in->data[0] + ch;
        float *dst = planar ? (float *)out->extended_data[ch]
                            : (float *)out->data[0] + ch;
        float z1 = s->cz1[ch], z2 = s->cz2[ch], env = s->cenv[ch];
        if (s->lfe[ch]) {
            if (dst != src)
                for (int i = 0; i < nb; i++)
                    dst[i * stride] = src[i * stride];
            continue;
        }
        for (int i = 0; i < nb; i++) {
            const float x = src[i * stride];
            float bp = s->b0 * x + z1, e, amp;
            z1  = -s->a1 * bp + z2;
            z2  = s->b2 * x - s->a2 * bp;
            e   = bp * bp;
            env += (e > env ? s->a_att : s->a_rel) * (e - env);
            if (s->mode == MODE_TEST)
                amp = s->lin_test;
            else
                amp = s->gains[(size_t)i * nb_ch + ch] * av_clipf(sqrtf(FFMIN(s->mixenv[i], env)) * s->lin_level,
                                   s->lin_floor, s->lin_ceil);
            dst[i * stride] = x + w[i] * amp;
        }
        s->cz1[ch] = z1; s->cz2[ch] = z2; s->cenv[ch] = env;
    }

    if (out != in)
        av_frame_free(&in);
    return ff_filter_frame(outlink, out);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    WaterStampContext *s = ctx->priv;
    av_freep(&s->tmp);
    av_freep(&s->mixenv);
    av_freep(&s->gains);
    free_channels(s);
}

static const AVFilterPad waterstamp_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_waterstamp = {
    .p.name        = "waterstamp",
    .p.description = NULL_IF_CONFIG_SMALL("Mix an inaudible spread-spectrum time stamp into audio."),
    .p.priv_class  = &waterstamp_class,
    .priv_size     = sizeof(WaterStampContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(waterstamp_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_FLTP),
};

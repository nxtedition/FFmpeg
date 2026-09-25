/*
 * WATERSTAMP v1 video detector -- recover the time stamp from pictures.
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
 * Recover the vwaterstamp time stamp(s) from decoded video.
 *
 * Per frame:
 *   1. find the active picture (letterbox and pillarbox bars are skipped),
 *      average the luma over the 132 x 124 sub-cells of the normalised
 *      grid, and demodulate each cell with its Walsh carrier -> 1023 cell
 *      values in which the picture's own content has largely cancelled;
 *   2. for every source id not yet tracked, correlate the cell values with
 *      the 32 layer-A shifts of that id's code, take z-scores across the
 *      shifts, and accumulate the z-score of the shift that a slot phase
 *      hypothesis predicts for this frame's timestamp, over 32 phase
 *      hypotheses. Frames at different chip phases therefore vote for the
 *      one phase that fits them all; the best hypothesis above threshold
 *      spawns a tracker;
 *   3. per tracker: keep the phase accumulator running (parabolic
 *      interpolation gives sub-chip phase), group frames into slots, sum
 *      the z-scores of the 8 layer-B shifts per slot -> symbol, and assemble
 *      frames exactly as the audio detector: sync word, check field, id
 *      match, confirmation by an earlier valid frame.
 *
 * Reports the same metadata and log line as waterdetect, with the filter
 * name vwaterdetect, so one parser covers both.
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

#define MAX_TRACKERS 8
#define COAST_US     10000000           /* hold a tracker through 10 s of flat picture */
#define UNLOCKED_SLOTS 120               /* 60 s without a frame lock: release */
#define MIN_FRAMES   4
#define WIDE_PCT     5                  /* wide search: +-5 % in 0.5 % steps    */
#define WIDE_STEPS   (2 * WIDE_PCT * 2 + 1)
#define PH_SUB       4                  /* phase hypotheses per chip           */
#define NPH          (WSV_CHIPS * PH_SUB) /* 128 slot phase hypotheses, 3.9 ms  */
#define PH_US        ((double)WSV_CHIP_US / PH_SUB)

enum ClockMode { CLOCK_WALL, CLOCK_PTS, NB_CLOCK };
enum AreaMode  { AREA_AUTO, AREA_FULL, NB_AREA };

typedef struct VTracker {
    int      used;
    int      code;
    int      geom;                      /* G_FULL or G_AUTO                     */
    int      low;                       /* score below release: coasting        */
    int64_t  low_since;
    double   rscale;                    /* pts scale hypothesis: tau = pts * rscale */
    float    acc[NPH];
    double   phase_us;                  /* slot boundary: pts = phase (mod 500 ms) */
    float    score;
    int64_t  cur_slot;
    int      have_slot;
    float    zB[WS_CSK_M];
    float    zaS;                       /* layer A at the chip, same units      */
    int      nzb;                       /* frames summed into zB                */
    double   slot_tau[WS_SLOTS];        /* slot boundary in tau when it was decoded */
    int64_t  last_pushed;               /* last slot given to the decoder       */
    int      have_pushed;
    int      unlocked_slots;
    WSFrameDec fd;

    int      lock;
    unsigned id;
    int64_t  ref_t_us;                  /* source time at ref_pts_us            */
    int64_t  ref_pts_us;
    double   rate;                      /* pts seconds per source second - 1    */
    int      rate_valid;                /* measured over a long enough baseline  */
    int      frames;
    int64_t  base_slot;                 /* first confirmed frame: rate baseline  */
    int64_t  base_tau;
    int      have_base;
    int64_t  settle_slot;               /* boundaries before this are transient  */
} VTracker;

enum { G_FULL, G_AUTO, NB_GEOM };
#define AREA_STABLE 25                  /* frames an active area must hold      */

typedef struct Geom {
    int     active;
    int     x0, y0, aw, ah;
    int    *colsub, *rowsub;
    float   r[WSV_CELLS];
    float   r_avg[WSV_CELLS];           /* running mean per cell: static content */
    float   r_pow[WSV_CELLS];           /* running |r| per cell: whitening        */
    int     have_avg;
} Geom;

typedef struct VWaterDetectContext {
    const AVClass *class;

    int     clock_mode;
    int64_t epoch;
    double  thresh;
    int     area_mode;
    int     max_sources;
    int     only_id;
    char   *key;
    size_t  keylen;
    int     wide;
    int     nvh;                        /* speed hypotheses                     */
    double  vh_scale[WIDE_STEPS];

    int8_t  codeA[WS_MAX_IDS][WS_SEQ_LEN];
    int8_t  codeB[WS_MAX_IDS][WS_SEQ_LEN];
    int     tracked[WS_MAX_IDS];
    int     hits[NB_GEOM][WS_MAX_IDS];  /* consecutive frames above threshold  */
    float  *cand;                       /* [NB_GEOM][WS_MAX_IDS][nvh][NPH]      */
    int     cand_frames;
    VTracker trk[MAX_TRACKERS];

    /* geometry: the stamp may cover the whole frame (stamped with its bars)
     * or only the picture inside bars added later; both are demodulated */
    Geom    g[NB_GEOM];
    int     pend[4];                    /* candidate active area, x0 y0 w h     */
    int     pend_n;                     /* frames it has been stable            */
    double *subsum;
    int    *subcnt;
    int     depth;                      /* luma bit depth                       */

    int64_t epoch_ref_us;               /* reference clock at the first frame  */
    int     have_first_pts;
    int64_t now_us;
} VWaterDetectContext;

#define OFFSET(x) offsetof(VWaterDetectContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_VIDEO_PARAM

static const AVOption vwaterdetect_options[] = {
    { "clock",     "reference clock for the offset measurement", OFFSET(clock_mode), AV_OPT_TYPE_INT,   {.i64 = CLOCK_WALL}, 0, NB_CLOCK - 1, FLAGS, .unit = "clock" },
    { "wall",      "detector wall clock",                        0,                  AV_OPT_TYPE_CONST, {.i64 = CLOCK_WALL}, 0, 0, FLAGS, .unit = "clock" },
    { "pts",       "frame timestamps",                           0,                  AV_OPT_TYPE_CONST, {.i64 = CLOCK_PTS},  0, 0, FLAGS, .unit = "clock" },
    { "epoch",     "hint for the absolute time in seconds, -1 = use the reference clock", OFFSET(epoch), AV_OPT_TYPE_INT64, {.i64 = -1}, -1, INT64_MAX, FLAGS },
    { "threshold", "acquisition threshold, peak-to-sidelobe ratio of the phase accumulator, -1 = auto", OFFSET(thresh), AV_OPT_TYPE_DOUBLE, {.dbl = -1}, -1, 50, FLAGS },
    { "area",      "active picture area",                        OFFSET(area_mode),  AV_OPT_TYPE_INT,   {.i64 = AREA_AUTO}, 0, NB_AREA - 1, FLAGS, .unit = "area" },
    { "auto",      "skip black letterbox and pillarbox bars",    0,                  AV_OPT_TYPE_CONST, {.i64 = AREA_AUTO}, 0, 0, FLAGS, .unit = "area" },
    { "full",      "use the full frame",                         0,                  AV_OPT_TYPE_CONST, {.i64 = AREA_FULL}, 0, 0, FLAGS, .unit = "area" },
    { "sources",   "maximum number of stamps tracked at once",   OFFSET(max_sources), AV_OPT_TYPE_INT,  {.i64 = 4}, 1, MAX_TRACKERS, FLAGS },
    { "id",        "only search for this source id, -1 = all",   OFFSET(only_id),    AV_OPT_TYPE_INT,   {.i64 = -1}, -1, WS_MAX_IDS - 1, FLAGS },
    { "key",       "secret key the stamps were made with",       OFFSET(key),        AV_OPT_TYPE_STRING, {.str = NULL}, 0, 0, FLAGS },
    { "wide",      "search speed changes up to +-5 % (for test-mode feeds)", OFFSET(wide), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(vwaterdetect);

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ444P,
    AV_PIX_FMT_NV12, AV_PIX_FMT_NV21, AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10,
    AV_PIX_FMT_YUV420P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV444P12,
    AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12,
    AV_PIX_FMT_NONE
};

/* Luma sample x of a row, in 8-bit units. */
static inline float luma8(const uint8_t *row, int x, int depth)
{
    return depth > 8 ? ((const uint16_t *)row)[x] * (1.f / (1 << (depth - 8))) : row[x];
}

static av_cold int init(AVFilterContext *ctx)
{
    VWaterDetectContext *s = ctx->priv;

    s->keylen = s->key ? strlen(s->key) : 0;
    for (int c = 0; c < WS_MAX_IDS; c++)
        if (ws_sequences((const uint8_t *)s->key, s->keylen, c, s->codeA[c], s->codeB[c]) < 0) {
            av_log(ctx, AV_LOG_ERROR, "could not build the spreading codes\n");
            return AVERROR_BUG;
        }
    s->nvh = s->wide ? WIDE_STEPS : 1;
    /* a wide search tries 21 speeds per code and geometry: a strong stamp's
     * cross-talk then clears a low threshold somewhere */
    if (s->thresh < 0)
        s->thresh = s->wide ? 7 : 5;
    for (int h = 0; h < s->nvh; h++)
        s->vh_scale[h] = s->wide ? 1.0 + (h - WIDE_PCT * 2) * 0.005 : 1.0;
    s->cand   = av_calloc((size_t)NB_GEOM * WS_MAX_IDS * s->nvh * NPH, sizeof(*s->cand));
    s->subsum = av_calloc((size_t)WSV_SUBCOLS * WSV_SUBROWS, sizeof(*s->subsum));
    s->subcnt = av_calloc((size_t)WSV_SUBCOLS * WSV_SUBROWS, sizeof(*s->subcnt));
    if (!s->cand || !s->subsum || !s->subcnt)
        return AVERROR(ENOMEM);
    for (int i = 0; i < NB_GEOM; i++)
        s->g[i].x0 = s->g[i].y0 = -1;
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    VWaterDetectContext *s = ctx->priv;
    for (int i = 0; i < MAX_TRACKERS; i++)
        ff_ws_framedec_uninit(&s->trk[i].fd);
    av_freep(&s->cand);
    av_freep(&s->subsum);
    av_freep(&s->subcnt);
    for (int i = 0; i < NB_GEOM; i++) {
        av_freep(&s->g[i].colsub);
        av_freep(&s->g[i].rowsub);
    }
}

static int config_input(AVFilterLink *inlink)
{
    VWaterDetectContext *s = inlink->dst->priv;
    s->depth = av_pix_fmt_desc_get(inlink->format)->comp[0].depth;
    return 0;
}

/* ---- geometry -------------------------------------------------------- */

static int set_area(Geom *g, int x0, int y0, int aw, int ah)
{
    g->active = 1;
    if (x0 == g->x0 && y0 == g->y0 && aw == g->aw && ah == g->ah)
        return 0;
    av_freep(&g->colsub);
    av_freep(&g->rowsub);
    g->colsub = av_malloc_array(aw, sizeof(*g->colsub));
    g->rowsub = av_malloc_array(ah, sizeof(*g->rowsub));
    if (!g->colsub || !g->rowsub)
        return AVERROR(ENOMEM);
    for (int x = 0; x < aw; x++)
        g->colsub[x] = (int)((int64_t)x * WSV_SUBCOLS / aw);
    for (int y = 0; y < ah; y++)
        g->rowsub[y] = (int)((int64_t)y * WSV_SUBROWS / ah);
    g->x0 = x0; g->y0 = y0; g->aw = aw; g->ah = ah;
    g->have_avg = 0;                    /* the cells moved: restart the means */
    return 0;
}

static int near(const int *a, int x0, int y0, int w, int h)
{
    return FFABS(a[0] - x0) <= 2 && FFABS(a[1] - y0) <= 2 &&
           FFABS(a[2] - w)  <= 2 && FFABS(a[3] - h)  <= 2;
}

/* Black bars: rows / columns whose subsampled mean luma stays below 24.
 * The detected area must hold for AREA_STABLE frames before it is used,
 * so a dark scene does not move the grid. */
static int find_area(VWaterDetectContext *s, const uint8_t *y, int ls, int w, int h)
{
    int top = 0, bot = h - 1, left = 0, right = w - 1, ret;
    const int step = 4, lim = 24;
    Geom *ga = &s->g[G_AUTO];

    if ((ret = set_area(&s->g[G_FULL], 0, 0, w, h)) < 0)
        return ret;
    if (s->area_mode == AREA_FULL) {
        ga->active = 0;
        return 0;
    }

    for (; top < h / 4; top++) {
        float sum = 0; int n = 0;
        const uint8_t *row = y + (ptrdiff_t)top * ls;
        for (int x = 0; x < w; x += step) { sum += luma8(row, x, s->depth); n++; }
        if (sum > lim * n)
            break;
    }
    for (; bot > h - h / 4; bot--) {
        float sum = 0; int n = 0;
        const uint8_t *row = y + (ptrdiff_t)bot * ls;
        for (int x = 0; x < w; x += step) { sum += luma8(row, x, s->depth); n++; }
        if (sum > lim * n)
            break;
    }
    for (; left < w / 4; left++) {
        float sum = 0; int n = 0;
        for (int yy = top; yy <= bot; yy += step) { sum += luma8(y + (ptrdiff_t)yy * ls, left, s->depth); n++; }
        if (sum > lim * n)
            break;
    }
    for (; right > w - w / 4; right--) {
        float sum = 0; int n = 0;
        for (int yy = top; yy <= bot; yy += step) { sum += luma8(y + (ptrdiff_t)yy * ls, right, s->depth); n++; }
        if (sum > lim * n)
            break;
    }

    if (s->pend_n && near(s->pend, left, top, right - left + 1, bot - top + 1)) {
        s->pend_n++;
    } else {
        s->pend[0] = left; s->pend[1] = top;
        s->pend[2] = right - left + 1; s->pend[3] = bot - top + 1;
        s->pend_n  = 1;
    }
    if (s->pend_n == AREA_STABLE || (ga->x0 < 0 && s->pend_n == 1)) {
        if (near(s->pend, 0, 0, w, h))
            ga->active = 0;             /* no bars: the full frame covers it */
        else if ((ret = set_area(ga, s->pend[0], s->pend[1], s->pend[2], s->pend[3])) < 0)
            return ret;
    }
    return 0;
}

/* Sub-cell means, then Walsh demodulation -> g->r[cell]. */
static void demodulate(VWaterDetectContext *s, Geom *g, const uint8_t *y, int ls)
{
    const int nsub = WSV_SUBCOLS * WSV_SUBROWS;
    memset(s->subsum, 0, nsub * sizeof(*s->subsum));
    memset(s->subcnt, 0, nsub * sizeof(*s->subcnt));
    for (int j = 0; j < g->ah; j++) {
        const uint8_t *row = y + (ptrdiff_t)(g->y0 + j) * ls;
        int base = g->rowsub[j] * WSV_SUBCOLS;
        if (s->depth > 8) {
            const uint16_t *r16 = (const uint16_t *)row + g->x0;
            const float sc = 1.f / (1 << (s->depth - 8));
            for (int i = 0; i < g->aw; i++) {
                int k = base + g->colsub[i];
                s->subsum[k] += r16[i] * sc;
                s->subcnt[k]++;
            }
        } else {
            const uint8_t *r8 = row + g->x0;
            for (int i = 0; i < g->aw; i++) {
                int k = base + g->colsub[i];
                s->subsum[k] += r8[i];
                s->subcnt[k]++;
            }
        }
    }
    for (int c = 0; c < WSV_CELLS; c++) {
        int col = c % WSV_COLS, rw = c / WSV_COLS;
        float v = 0.f;
        for (int sy = 0; sy < WSV_SUB; sy++)
            for (int sx = 0; sx < WSV_SUB; sx++) {
                int k = (rw * WSV_SUB + sy) * WSV_SUBCOLS + col * WSV_SUB + sx;
                float m = s->subcnt[k] ? (float)(s->subsum[k] / s->subcnt[k]) : 0.f;
                v += ws_carrier(c, sx, sy) * m;
            }
        /* static content survives the Walsh demodulation as a fixed
         * residual per cell; the stamp changes every chip. Subtract a
         * running mean so that only what moves remains. */
        if (g->have_avg) {
            float d = v - g->r_avg[c];
            g->r_avg[c] += 0.25f * d;
            /* whiten: a cell with a busy edge would otherwise outvote a
             * hundred quiet ones; give every cell unit weight */
            g->r_pow[c] += 0.1f * (fabsf(d) - g->r_pow[c]);
            g->r[c]      = d / (g->r_pow[c] + 0.5f);
        } else {
            g->r[c]     = 0.f;
            g->r_avg[c] = v;
            g->r_pow[c] = 1.f;
        }
    }
    g->have_avg = 1;
}

/* z-scores of the correlation of r with n cyclic shifts of code, step
 * cells apart, all offset by base cells; *sd (if set) gets their spread */
static void correlate_shifts(const float *r, const int8_t *code, int n, int step,
                             int base, float *z, float *sd)
{
    float mean = 0.f, var = 0.f;
    for (int k = 0; k < n; k++) {
        int sh = (k * step + base) % WS_SEQ_LEN;
        float c = 0.f;
        for (int cell = 0; cell < WSV_CELLS; cell++)
            c += r[cell] * code[(cell + sh) % WS_SEQ_LEN];
        z[k] = c;
        mean += c;
    }
    mean /= n;
    for (int k = 0; k < n; k++)
        var += (z[k] - mean) * (z[k] - mean);
    var = sqrtf(var / n) + 1e-6f;
    for (int k = 0; k < n; k++)
        z[k] = (z[k] - mean) / var;
    if (sd)
        *sd = var;
}

/* ---- time ------------------------------------------------------------ */

static inline int chip_for(int64_t pts_us, double phase_us)
{
    double tp = fmod(pts_us - phase_us, (double)WS_SLOT_US);
    if (tp < 0)
        tp += WS_SLOT_US;
    return FFMIN((int)(tp / WSV_CHIP_US), WSV_CHIPS - 1);
}

static int64_t measured_us(const VWaterDetectContext *s, int64_t pts_us)
{
    return s->clock_mode == CLOCK_PTS ? pts_us : s->now_us;
}

/* Tracker time runs on tau = pts * rscale; ref_pts_us and phase are in tau. */
static int64_t recovered_us(const VTracker *t, int64_t pts_us)
{
    double r = t->rate_valid ? t->rate : 0.0;
    return t->ref_t_us + llrint((pts_us * t->rscale - t->ref_pts_us) / (1.0 + r));
}

static double drift_ppm(const VTracker *t)
{
    double r = t->rate_valid ? t->rate : 0.0;
    return (t->rscale / (1.0 + r) - 1.0) * 1e6;
}

static void log_state(AVFilterContext *ctx, const VTracker *t, int64_t pts_us)
{
    const VWaterDetectContext *s = ctx->priv;
    int64_t tt  = t->lock ? recovered_us(t, pts_us) : 0;
    int64_t off = t->lock ? measured_us(s, pts_us) - tt : 0;
    av_log(ctx, AV_LOG_INFO,
           "vwaterdetect lock:%d id:%u t:%"PRId64".%03d offset:%s%"PRId64".%03d drift:%.1f snr:%.1f\n",
           t->lock, t->id, tt / 1000000, (int)(tt % 1000000 / 1000),
           off < 0 ? "-" : "", FFABS(off) / 1000, (int)(FFABS(off) % 1000),
           drift_ppm(t), 20.0 * log10(FFMAX(t->score, 1e-3)));
}

static void set_lock(AVFilterContext *ctx, VTracker *t, int lock, int64_t pts_us)
{
    if (t->lock == lock)
        return;
    t->lock = lock;
    log_state(ctx, t, pts_us);
}

/* ---- frames ---------------------------------------------------------- */

/* Take a frame from the decoder: re-anchor, and measure the rate from the
 * slot boundaries of decoded frames. */
static void take_frame(AVFilterContext *ctx, VTracker *t, int ev, const WSFrameResult *r,
                       int64_t pts_us)
{
    VWaterDetectContext *s = ctx->priv;
    const int64_t slot0 = (int64_t)r->pos;
    const int64_t pts0  = llrint(t->slot_tau[slot0 % WS_SLOTS]);
    int64_t ref_s, k;

    if (s->epoch >= 0)   /* the hint advances with the reference clock */
        ref_s = s->epoch + (measured_us(s, pts_us) - s->epoch_ref_us) / 1000000;
    else if (s->clock_mode == CLOCK_WALL)
        ref_s = measured_us(s, pts_us) / 1000000;
    else
        ref_s = r->payload;
    k = ref_s - (int64_t)r->payload + WS_PAYLOAD_MOD / 2;
    k = k >= 0 ? k / WS_PAYLOAD_MOD : -((-k + WS_PAYLOAD_MOD - 1) / WS_PAYLOAD_MOD);

    if (ev == WS_FD_JUMP && t->lock)
        av_log(ctx, AV_LOG_WARNING, "id %d: source time jumped\n", t->code);
    /* after a fold the phase accumulator needs a few seconds to settle;
     * boundaries recorded meanwhile are off by many ms */
    if (slot0 >= t->settle_slot) {
        if (!t->have_base) {
            t->base_slot = slot0;
            t->base_tau  = pts0;
            t->have_base = 1;
        } else if (slot0 - t->base_slot >= 2 * WS_SLOTS) {
            /* boundaries are known to a millisecond; over >= 12 s that
             * is better than 100 ppm and improves as the lock lasts */
            double elapsed_src = (double)(slot0 - t->base_slot) * WS_SLOT_US;
            t->rate       = av_clipd((pts0 - t->base_tau) / elapsed_src - 1.0, -0.01, 0.01);
            t->rate_valid = 1;
        }
        t->ref_t_us   = ((int64_t)r->payload + k * WS_PAYLOAD_MOD) * 1000000;
        t->ref_pts_us = pts0;
    } else if (!t->lock) {
        t->ref_t_us   = ((int64_t)r->payload + k * WS_PAYLOAD_MOD) * 1000000;
        t->ref_pts_us = pts0;
    }
    t->id = t->code;
    av_log(ctx, AV_LOG_DEBUG, "frame id:%d payload:%u slot0:%"PRId64" score:%.1f pts0:%.3fs rate:%.0fppm scale:%.5f\n",
           t->code, r->payload, slot0, r->stat, pts0 / 1e6, t->rate * 1e6, t->rscale);
    if (!t->lock)
        set_lock(ctx, t, 1, pts_us);
    else
        log_state(ctx, t, pts_us);
}

/* Give the finished slot to the frame decoder. */
static void push_slot(AVFilterContext *ctx, VTracker *t, int64_t pts_us)
{
    WSFrameResult res;
    float z[WS_CSK_M];
    const float nrm = 1.f / sqrtf(FFMAX(t->nzb, 1));
    int ev;

    /* slots without a frame are erasures, so the frame count holds */
    if (t->have_pushed)
        for (int64_t g = t->last_pushed + 1; g < t->cur_slot && g < t->last_pushed + 1 + WS_SLOTS; g++) {
            t->slot_tau[g % WS_SLOTS] = t->phase_us + (double)g * WS_SLOT_US;
            if (ff_ws_framedec_push(&t->fd, NULL, 0.f, (double)g, &res) == WS_FD_LOCK_OFF)
                set_lock(ctx, t, 0, pts_us);
        }
    for (int m = 0; m < WS_CSK_M; m++)
        z[m] = t->zB[m] * nrm;
    t->slot_tau[t->cur_slot % WS_SLOTS] = t->phase_us + (double)t->cur_slot * WS_SLOT_US;
    ev = ff_ws_framedec_push(&t->fd, t->nzb ? z : NULL, t->zaS * nrm, (double)t->cur_slot, &res);
    t->last_pushed = t->cur_slot;
    t->have_pushed = 1;
    if (t->fd.lock)
        av_log(ctx, AV_LOG_DEBUG, "code %d: layer ratio %.2f\n", t->code, t->fd.ratio);
    if (ev == WS_FD_LOCK_OFF)
        set_lock(ctx, t, 0, pts_us);
    else if (ev != WS_FD_NONE)
        take_frame(ctx, t, ev, &res, pts_us);
    t->unlocked_slots = t->lock ? 0 : t->unlocked_slots + 1;
}

/* ---- trackers -------------------------------------------------------- */

static void release_tracker(AVFilterContext *ctx, VTracker *t, int64_t pts_us)
{
    VWaterDetectContext *s = ctx->priv;
    set_lock(ctx, t, 0, pts_us);
    s->tracked[t->code] = 0;
    for (int g = 0; g < NB_GEOM; g++)
        memset(s->cand + ((size_t)g * WS_MAX_IDS + t->code) * s->nvh * NPH, 0,
               (size_t)s->nvh * NPH * sizeof(float));
    ff_ws_framedec_uninit(&t->fd);
    memset(t, 0, sizeof(*t));
}

static void accumulate_phase(float *acc, const float *zA, int64_t pts_us, float alpha)
{
    for (int ph = 0; ph < NPH; ph++) {
        int k = chip_for(pts_us, ph * PH_US);
        acc[ph] += alpha * (zA[k] - acc[ph]);
    }
}

/* Peak of acc with parabolic interpolation. Returns the peak-to-sidelobe
 * ratio: peak over the standard deviation of the bins more than one chip
 * away, which is what the accumulated noise actually looks like. */
static float peak_phase(const float *acc, double *phase_chips)
{
    int pk = 0, n = 0;
    float y1, mean = 0.f, var = 0.f;
    double d;
    for (int ph = 1; ph < NPH; ph++)
        if (acc[ph] > acc[pk])
            pk = ph;
    for (int ph = 0; ph < NPH; ph++) {
        int dd = FFABS(ph - pk);
        dd = FFMIN(dd, NPH - dd);
        if (dd > PH_SUB) { mean += acc[ph]; n++; }
    }
    mean /= FFMAX(n, 1);
    for (int ph = 0; ph < NPH; ph++) {
        int dd = FFABS(ph - pk);
        dd = FFMIN(dd, NPH - dd);
        if (dd > PH_SUB) var += (acc[ph] - mean) * (acc[ph] - mean);
    }
    var = sqrtf(var / FFMAX(n, 1)) + 1e-3f;
    /* One chip covers PH_SUB bins, so the peak is a plateau and a parabola
     * through its top says nothing. The plateau's edges are shaped by the
     * frames that fall near chip boundaries and move continuously with the
     * true phase, so its centroid resolves well below a bin. */
    {
        double num = 0, den = 0;
        for (int dd = -(PH_SUB + 1); dd <= PH_SUB + 1; dd++) {
            float w = acc[(pk + dd + NPH) % NPH] - mean;
            if (w > 0) { num += dd * w; den += w; }
        }
        d = den > 0 ? num / den : 0;
    }
    y1 = acc[pk];
    *phase_chips = pk + d;
    return (y1 - mean) / var;
}

static void track_frame(AVFilterContext *ctx, VTracker *t, int64_t pts_us)
{
    VWaterDetectContext *s = ctx->priv;
    float zA[WSV_CHIPS], zB[WS_CSK_M], sdA, sdB;
    double ph_chips, ph_us, d;
    int64_t tau = llrint(pts_us * t->rscale);         /* speed-corrected time */
    int64_t slot;

    const float *r = s->g[t->geom].r;

    if (!s->g[t->geom].active) {
        release_tracker(ctx, t, pts_us);
        return;
    }
    correlate_shifts(r, s->codeA[t->code], WSV_CHIPS, WSV_A_STEP, 0, zA, &sdA);
    accumulate_phase(t->acc, zA, tau, 1.f / 64);
    t->score = peak_phase(t->acc, &ph_chips);
    if (t->score < s->thresh * 0.6f) {
        /* a flat or black stretch carries (almost) no stamp: coast on the
         * last phase for a while instead of starting over */
        if (!t->low) {
            t->low       = 1;
            t->low_since = pts_us;
        } else if (pts_us - t->low_since > COAST_US) {
            av_log(ctx, AV_LOG_VERBOSE, "code %d: lost, score %.2f\n", t->code, t->score);
            release_tracker(ctx, t, pts_us);
            return;
        }
    } else {
        t->low = 0;
        /* keep the phase continuous across the 500 ms wrap */
        ph_us = ph_chips * PH_US;
        d = fmod(ph_us - t->phase_us, (double)WS_SLOT_US);
        if (d >  WS_SLOT_US / 2) d -= WS_SLOT_US;
        if (d < -WS_SLOT_US / 2) d += WS_SLOT_US;
        t->phase_us += d;
    }

    /* The slot boundary drifts through the timestamps at the sample-rate
     * error: measure its slope from an anchor set once the accumulator has
     * settled. Positive slope = the source runs slow against the clock. */
    t->frames++;
    if (t->frames % 25 == 0)
        av_log(ctx, AV_LOG_DEBUG, "track code %d frame %d pts %.3f phase %.2f ms score %.1f\n",
               t->code, t->frames, pts_us / 1e6, t->phase_us / 1000.0, t->score);
    /* Fold a confirmed residual above 300 ppm into the time scale, so that
     * the chip prediction stops lagging; the rate is then re-measured from
     * a fresh baseline. */
    if (t->rate_valid && fabs(t->rate) > 300e-6) {
        double f = 1.0 / (1.0 + t->rate);
        /* keep the current slot's index: boundaries are counted from
         * tau = 0, so the phase absorbs the scale change at cur_slot */
        t->phase_us        = t->phase_us * f + (double)t->cur_slot * WS_SLOT_US * (f - 1.0);
        t->rscale         *= f;
        t->ref_pts_us      = llrint(t->ref_pts_us * f);
        for (int i = 0; i < WS_SLOTS; i++)
            t->slot_tau[i] *= f;
        tau                = llrint(pts_us * t->rscale);
        t->rate            = 0;
        t->rate_valid      = 0;
        t->have_base       = 0;
        t->settle_slot     = t->cur_slot + 2 * WS_SLOTS;
        av_log(ctx, AV_LOG_VERBOSE, "code %d: speed now %+.2f %%\n", t->code, (t->rscale - 1.0) * 100.0);
    }

    slot = (int64_t)floor((tau - t->phase_us) / WS_SLOT_US);
    if (!t->have_slot || slot != t->cur_slot) {
        if (t->have_slot) {
            push_slot(ctx, t, pts_us);
            if (t->unlocked_slots > UNLOCKED_SLOTS) {
                /* a timing lock that never yields a frame was noise */
                av_log(ctx, AV_LOG_VERBOSE, "code %d: no frame lock in %d s, released\n",
                       t->code, UNLOCKED_SLOTS / 2);
                release_tracker(ctx, t, pts_us);
                return;
            }
        }
        memset(t->zB, 0, sizeof(t->zB));
        t->zaS = 0.f;
        t->nzb = 0;
        t->cur_slot  = slot;
        t->have_slot = 1;
    }
    /* layer B carries the symbol shift plus this chip's advance. The frame's
     * own layer A result names the chip directly when it is unambiguous,
     * which does not lag behind a speed residual the way the phase does. */
    {
        int k = 0, kp = chip_for(tau, t->phase_us);
        for (int i = 1; i < WSV_CHIPS; i++)
            if (zA[i] > zA[k])
                k = i;
        if (zA[k] < 3.f)
            k = kp;
        correlate_shifts(r, s->codeB[t->code], WS_CSK_M, WS_CSK_STEP, k * WSV_A_STEP, zB, &sdB);
        /* layer A at the predicted chip (not the argmax, which would be
         * biased up at low SNR), in layer B's units: the layer check */
        t->zaS += zA[kp] * sdA / sdB;
    }
    for (int m = 0; m < WS_CSK_M; m++)
        t->zB[m] += zB[m];
    t->nzb++;
}

static void acquire_frame(AVFilterContext *ctx, int64_t pts_us)
{
    VWaterDetectContext *s = ctx->priv;
    float zA[WSV_CHIPS];
    int best_c = -1, best_g = 0, best_h = 0, best_ph = 0, ntr = 0, any = 0;
    float best = 0.f;
    VTracker *t = NULL;

    for (int i = 0; i < MAX_TRACKERS; i++)
        ntr += s->trk[i].used;
    for (int g = 0; g < NB_GEOM; g++) {
        if (!s->g[g].active)
            continue;
        for (int c = 0; c < WS_MAX_IDS; c++) {
            float cbest = 0.f;
            int ch = 0, cph = 0;
            if (s->tracked[c] || (s->only_id >= 0 && s->only_id != c))
                continue;
            any = 1;
            correlate_shifts(s->g[g].r, s->codeA[c], WSV_CHIPS, WSV_A_STEP, 0, zA, NULL);
            for (int h = 0; h < s->nvh; h++) {
                float *acc = s->cand + (((size_t)g * WS_MAX_IDS + c) * s->nvh + h) * NPH;
                double ph;
                float psr;
                /* about five seconds: long enough for a heavily compressed
                 * stamp to rise out of the noise, and for wide-search
                 * hypotheses 0.5 % apart to separate by a chip */
                accumulate_phase(acc, zA, llrint(pts_us * s->vh_scale[h]), 1.f / 128);
                psr = peak_phase(acc, &ph);
                if (psr > cbest) {
                    cbest = psr; ch = h; cph = (int)floor(ph + 0.5) % NPH;
                }
            }
            s->hits[g][c] = cbest >= s->thresh ? s->hits[g][c] + 1 : 0;
            /* Speed hypotheses only separate once a wrong one has had time to
             * smear: two seconds at 1.5 % apart. Residuals below that are the
             * tracker's job. */
            if (s->hits[g][c] >= 3 && cbest >= s->thresh && cbest > best &&
                (s->nvh == 1 || s->cand_frames >= 128)) {
                best = cbest; best_c = c; best_g = g; best_h = ch; best_ph = cph;
            }
        }
    }
    if (!any)
        return;
    if (++s->cand_frames % 25 == 0)
        av_log(ctx, AV_LOG_DEBUG, "acquire: frame %d best code %d area %s hyp %d phase %d score %.2f\n",
               s->cand_frames, best_c, best_g == G_FULL ? "full" : "auto", best_h, best_ph, best);
    if (s->cand_frames < MIN_FRAMES || best_c < 0 || best < s->thresh || ntr >= s->max_sources)
        return;
    for (int i = 0; i < MAX_TRACKERS; i++)
        if (!s->trk[i].used) {
            t = &s->trk[i];
            s->tracked[best_c] = i + 1;
            break;
        }
    if (!t)
        return;
    memset(t, 0, sizeof(*t));
    if (ff_ws_framedec_init(&t->fd, best_c, (const uint8_t *)s->key, s->keylen) < 0) {
        s->tracked[best_c] = 0;
        return;
    }
    t->used   = 1;
    t->code   = best_c;
    t->geom   = best_g;
    t->rscale = s->vh_scale[best_h];
    memcpy(t->acc, s->cand + (((size_t)best_g * WS_MAX_IDS + best_c) * s->nvh + best_h) * NPH,
           NPH * sizeof(float));
    t->score = best;
    t->phase_us = best_ph * PH_US;
    for (int g = 0; g < NB_GEOM; g++) {
        memset(s->cand + ((size_t)g * WS_MAX_IDS + best_c) * s->nvh * NPH, 0,
               (size_t)s->nvh * NPH * sizeof(float));
        s->hits[g][best_c] = 0;
    }
    av_log(ctx, AV_LOG_VERBOSE, "code %d: layer A lock score %.2f phase %.1f ms speed %+.1f %% area %s\n",
           best_c, best, t->phase_us / 1000.0, (t->rscale - 1.0) * 100.0,
           best_g == G_FULL ? "full" : "active picture");
}

/* ---- output ---------------------------------------------------------- */

static void set_tracker_metadata(const VWaterDetectContext *s, const VTracker *t, AVFrame *f,
                                 const char *prefix, int64_t pts_us)
{
    char key[64], buf[64];
    int64_t tt = recovered_us(t, pts_us), off = measured_us(s, pts_us) - tt;
    snprintf(key, sizeof(key), "%sid", prefix);     av_dict_set_int(&f->metadata, key, t->id, 0);
    snprintf(key, sizeof(key), "%st", prefix);      av_dict_set_int(&f->metadata, key, tt, 0);
    snprintf(key, sizeof(key), "%soffset", prefix);
    snprintf(buf, sizeof(buf), "%s%"PRId64".%03d", off < 0 ? "-" : "", FFABS(off) / 1000, (int)(FFABS(off) % 1000));
    av_dict_set(&f->metadata, key, buf, 0);
    snprintf(key, sizeof(key), "%sdrift", prefix);  snprintf(buf, sizeof(buf), "%.1f", drift_ppm(t));
    av_dict_set(&f->metadata, key, buf, 0);
    snprintf(key, sizeof(key), "%ssnr", prefix);    snprintf(buf, sizeof(buf), "%.1f", 20.0 * log10(FFMAX(t->score, 1e-3)));
    av_dict_set(&f->metadata, key, buf, 0);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    VWaterDetectContext *s = ctx->priv;
    const VTracker *primary = NULL;
    char prefix[48];
    int64_t pts_us;
    int ret, n = 0;

    if (frame->pts == AV_NOPTS_VALUE)
        return ff_filter_frame(ctx->outputs[0], frame);
    s->now_us = av_gettime();
    pts_us = av_rescale_q(frame->pts, inlink->time_base, AV_TIME_BASE_Q);
    if (!s->have_first_pts) {
        s->epoch_ref_us   = measured_us(s, pts_us);
        s->have_first_pts = 1;
    }

    if ((ret = find_area(s, frame->data[0], frame->linesize[0], frame->width, frame->height)) < 0) {
        av_frame_free(&frame);
        return ret;
    }
    for (int g = 0; g < NB_GEOM; g++)
        if (s->g[g].active)
            demodulate(s, &s->g[g], frame->data[0], frame->linesize[0]);

    for (int i = 0; i < MAX_TRACKERS; i++)
        if (s->trk[i].used)
            track_frame(ctx, &s->trk[i], pts_us);
    acquire_frame(ctx, pts_us);

    for (int i = 0; i < MAX_TRACKERS; i++) {
        const VTracker *t = &s->trk[i];
        if (!t->used || !t->lock)
            continue;
        if (!primary)
            primary = t;
        snprintf(prefix, sizeof(prefix), "lavfi.vwaterdetect.%d.", n++);
        set_tracker_metadata(s, t, frame, prefix, pts_us);
    }
    av_dict_set_int(&frame->metadata, "lavfi.vwaterdetect.lock", primary != NULL, 0);
    av_dict_set_int(&frame->metadata, "lavfi.vwaterdetect.sources", n, 0);
    if (primary)
        set_tracker_metadata(s, primary, frame, "lavfi.vwaterdetect.", pts_us);
    return ff_filter_frame(ctx->outputs[0], frame);
}

static const AVFilterPad vwaterdetect_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_vf_vwaterdetect = {
    .p.name        = "vwaterdetect",
    .p.description = NULL_IF_CONFIG_SMALL("Recover the vwaterstamp time stamp and report the offset."),
    .p.priv_class  = &vwaterdetect_class,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    .priv_size     = sizeof(VWaterDetectContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(vwaterdetect_inputs),
    FILTER_OUTPUTS(ff_video_default_filterpad),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
};

/*
 * WATERSTAMP v1 detector -- recover an inaudible spread-spectrum time stamp.
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
 * Recover the waterstamp time stamp(s) from decoded audio and report offsets.
 *
 * Processing chain, all at a forced 8184 Hz mono float input (4 samples per
 * chip; FFmpeg inserts the conversion and the downmix sums the identical
 * per-channel watermarks coherently):
 *
 *   1. quadrature demodulation at the 2000 Hz carrier, low-pass, decimate
 *      by 2 -> complex baseband at 4092 Hz, two samples per chip;
 *   2. acquisition, per 500 ms period: one forward FFT of a two-period
 *      window per rate hypothesis, then one inverse FFT per untracked code
 *      against its layer A reference (two periods of data make the lags
 *      0..2045 an exact circular correlation). |corr|^2 accumulates
 *      non-coherently per (code, rate). The sharpest peak above threshold
 *      spawns a tracker for that code;
 *   3. per tracker, per period: correlate against its own layer A, keep the
 *      peak (parabolic interpolation -> sub-chip timing), feed the peak's
 *      motion into a rate loop that resamples and derotates the window;
 *   4. per tracker, per slot: correlate one period (duplicated) against its
 *      layer B, read the eight candidate cyclic shifts -> one 3-bit symbol;
 *   5. every 12 symbols: sync word + CRC-8 -> frame; a second frame whose
 *      payload matches the elapsed slots declares lock.
 *
 * Every source id has its own Gold code pair, so several stamps in one
 * signal (a mix, or a re-stamped feed) are tracked and reported
 * independently.
 *
 * Decisions are made on correlation peaks, never on amplitude, so gain
 * changes, normalisation and slow compression cost nothing.
 */

#include <math.h>

#include "libavutil/channel_layout.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/tx.h"
#include "audio.h"
#include "avfilter.h"
#include "filters.h"
#include "formats.h"
#include "waterstamp.h"

#define FFT_N       4096                    /* >= 2 * WS_BB_PERIOD          */
#define WIN_A       (2 * WS_BB_PERIOD)     /* 4092: two periods, layer A    */
#define FIR_TAPS    47
#define FIR_HALF    ((FIR_TAPS - 1) / 2)
#define FIR_CUTOFF  1150.0                  /* Hz; band edge is 1023 Hz      */
#define BB_RING     16384                   /* baseband samples kept, 4 s    */
#define LO_PERIOD   WS_SEQ_LEN             /* 2000/8184 = 250/1023          */
#define MIN_PERIODS 4                       /* before layer A may lock       */
#define MAX_RATE    5000e-6                 /* clamp on the rate estimate    */
#define FRAME_BB    (WS_SLOTS * WS_BB_PERIOD)
#define MAX_TRACKERS 8

/* Acquisition rate hypotheses in ppm, all evaluated in parallel while a
 * code is not tracked. A 1000 ppm error (25 <-> 23.98 speed change) moves
 * the peak two bins per period, which smears an accumulator advanced at
 * the wrong rate before the loop could see it; the hypothesis nearest the
 * truth stays sharp and wins. */
static const double rate_hyp[] = { 0, 500, -500, 1000, -1000, 1500, -1500, 2000, -2000 };
#define NHYP FF_ARRAY_ELEMS(rate_hyp)

enum ClockMode { CLOCK_WALL, CLOCK_PTS, NB_CLOCK };

typedef struct PeakStats {
    double phase;                       /* interpolated peak, 0..2046         */
    float  psr, snr_db;                 /* peak-to-sidelobe, peak-to-mean     */
} PeakStats;

typedef struct Candidate {
    float    *acc;                      /* WS_BB_PERIOD accumulated |corr|^2 */
    PeakStats st;
} Candidate;

typedef struct Tracker {
    int      used;
    int      code;                      /* source id whose codes are tracked  */
    float   *acc;
    double   blk_pos;                   /* baseband position of next block    */
    double   rate;                      /* detector/emitter sample ratio - 1  */
    double   phase;
    float    psr, snr_db;

    double   slot_pos;                  /* baseband position of next slot     */
    int      slot_valid;
    double   last_slot_pos;
    int      sym[WS_SLOTS];
    double   sym_pos[WS_SLOTS];
    int64_t  nsym;

    int      lock;
    unsigned id;
    double   ref_pos;                   /* baseband position of a frame start */
    int64_t  ref_t_us;                  /* recovered source time at ref_pos   */
    int      have_valid;
    unsigned last_payload;
    double   last_valid_pos;
    int64_t  slots_since_valid;
} Tracker;

typedef struct WaterDetectContext {
    const AVClass *class;

    /* options */
    int     clock_mode;
    int64_t epoch;
    double  rate_ppm;
    double  thresh_db;
    double  limit_db;
    int     max_sources;
    int     only_id;

    /* demodulator + decimator */
    AVComplexFloat lo[LO_PERIOD];
    float          fir[FIR_TAPS];
    AVComplexFloat dline[FIR_TAPS];
    int            dl_pos;
    int64_t        in_count;            /* input samples consumed (8184 Hz) */

    /* complex baseband ring */
    AVComplexFloat *bb;
    int64_t         bb_count;           /* baseband samples produced        */

    /* correlator */
    AVTXContext    *tx, *itx;
    av_tx_fn        tx_fn, itx_fn;
    AVComplexFloat *RefA, *RefB;        /* WS_MAX_IDS conjugated spectra each */
    AVComplexFloat *win, *spec, *prod, *corr;

    /* acquisition: candidates[code][hyp], blocks shared per hypothesis */
    Candidate *cand;
    float     *cand_mem;
    double     hyp_blk[NHYP];
    double     hyp_rate[NHYP];
    int        hyp_periods[NHYP];
    int        tracked[WS_MAX_IDS];    /* code -> tracker index + 1, or 0    */

    Tracker    trk[MAX_TRACKERS];
    float     *trk_mem;

    /* measured clock */
    int64_t  first_pts_us;
    int      have_first_pts;
    int64_t  now_us;                    /* wall clock at the current frame    */
} WaterDetectContext;

#define OFFSET(x) offsetof(WaterDetectContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM | AV_OPT_FLAG_AUDIO_PARAM

static const AVOption waterdetect_options[] = {
    { "clock",     "reference clock for the offset measurement", OFFSET(clock_mode), AV_OPT_TYPE_INT,    {.i64 = CLOCK_WALL}, 0, NB_CLOCK - 1, FLAGS, .unit = "clock" },
    { "wall",      "detector wall clock",                        0,                  AV_OPT_TYPE_CONST,  {.i64 = CLOCK_WALL}, 0, 0, FLAGS, .unit = "clock" },
    { "pts",       "media timestamps",                           0,                  AV_OPT_TYPE_CONST,  {.i64 = CLOCK_PTS},  0, 0, FLAGS, .unit = "clock" },
    { "epoch",     "hint for the absolute time in seconds, resolves the 9.1 h payload ambiguity; -1 = use the reference clock", OFFSET(epoch), AV_OPT_TYPE_INT64, {.i64 = -1}, -1, INT64_MAX, FLAGS },
    { "rate",      "centre of the sample-rate error search in ppm", OFFSET(rate_ppm), AV_OPT_TYPE_DOUBLE, {.dbl = 0}, -5000, 5000, FLAGS },
    { "threshold", "layer A lock threshold, peak-to-sidelobe ratio in dB", OFFSET(thresh_db), AV_OPT_TYPE_DOUBLE, {.dbl = 4}, 0, 30, FLAGS },
    { "limit",     "clip spectral lines this many dB above the mean magnitude before correlating, 0 = off", OFFSET(limit_db), AV_OPT_TYPE_DOUBLE, {.dbl = 20}, 0, 60, FLAGS },
    { "sources",   "maximum number of stamps tracked at once",        OFFSET(max_sources), AV_OPT_TYPE_INT, {.i64 = 4}, 1, MAX_TRACKERS, FLAGS },
    { "id",        "only search for this source id, -1 = all",        OFFSET(only_id), AV_OPT_TYPE_INT,  {.i64 = -1}, -1, WS_MAX_IDS - 1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(waterdetect);

static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out)
{
    static const enum AVSampleFormat fmts[] = { AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_NONE };
    static const int rates[] = { WS_ANALYSIS_RATE, -1 };
    static const AVChannelLayout layouts[] = { AV_CHANNEL_LAYOUT_MONO, { 0 } };
    int ret;

    if ((ret = ff_set_sample_formats_from_list2(ctx, cfg_in, cfg_out, fmts)) < 0 ||
        (ret = ff_set_common_samplerates_from_list2(ctx, cfg_in, cfg_out, rates)) < 0)
        return ret;
    return ff_set_common_channel_layouts_from_list2(ctx, cfg_in, cfg_out, layouts);
}

/* Reference at complex baseband: the sequence at two samples per chip.
 * The carrier is gone after demodulation and the half-sine chip pulse
 * samples to a constant at u = 0.25 and 0.75, so only the chips remain. */
static void build_reference(WaterDetectContext *s, AVComplexFloat *Ref, const int8_t *seq)
{
    memset(s->win, 0, FFT_N * sizeof(*s->win));
    for (int m = 0; m < WS_BB_PERIOD; m++)
        s->win[m].re = seq[m >> 1];
    s->tx_fn(s->tx, Ref, s->win, sizeof(AVComplexFloat));
    for (int k = 0; k < FFT_N; k++)
        Ref[k].im = -Ref[k].im;
}

static void reset_acquisition(WaterDetectContext *s, double pos)
{
    for (int h = 0; h < NHYP; h++) {
        s->hyp_blk[h]     = pos;
        s->hyp_rate[h]    = av_clipd((s->rate_ppm + rate_hyp[h]) * 1e-6, -MAX_RATE, MAX_RATE);
        s->hyp_periods[h] = 0;
    }
    memset(s->cand_mem, 0, (size_t)WS_MAX_IDS * NHYP * WS_BB_PERIOD * sizeof(*s->cand_mem));
}

static av_cold int init(AVFilterContext *ctx)
{
    WaterDetectContext *s = ctx->priv;
    float scale = 1.f / FFT_N, sum = 0.f;
    int8_t seqA[WS_SEQ_LEN], seqB[WS_SEQ_LEN];
    int ret;

    s->bb   = av_calloc(BB_RING, sizeof(*s->bb));
    s->RefA = av_calloc((size_t)WS_MAX_IDS * FFT_N, sizeof(*s->RefA));
    s->RefB = av_calloc((size_t)WS_MAX_IDS * FFT_N, sizeof(*s->RefB));
    s->win  = av_calloc(FFT_N, sizeof(*s->win));
    s->spec = av_calloc(FFT_N, sizeof(*s->spec));
    s->prod = av_calloc(FFT_N, sizeof(*s->prod));
    s->corr = av_calloc(FFT_N, sizeof(*s->corr));
    s->cand     = av_calloc((size_t)WS_MAX_IDS * NHYP, sizeof(*s->cand));
    s->cand_mem = av_calloc((size_t)WS_MAX_IDS * NHYP * WS_BB_PERIOD, sizeof(*s->cand_mem));
    s->trk_mem  = av_calloc((size_t)MAX_TRACKERS * WS_BB_PERIOD, sizeof(*s->trk_mem));
    if (!s->bb || !s->RefA || !s->RefB || !s->win || !s->spec || !s->prod || !s->corr ||
        !s->cand || !s->cand_mem || !s->trk_mem)
        return AVERROR(ENOMEM);
    for (int i = 0; i < WS_MAX_IDS * NHYP; i++)
        s->cand[i].acc = s->cand_mem + (size_t)i * WS_BB_PERIOD;
    for (int i = 0; i < MAX_TRACKERS; i++)
        s->trk[i].acc = s->trk_mem + (size_t)i * WS_BB_PERIOD;

    if ((ret = av_tx_init(&s->tx,  &s->tx_fn,  AV_TX_FLOAT_FFT, 0, FFT_N, &scale, 0)) < 0 ||
        (ret = av_tx_init(&s->itx, &s->itx_fn, AV_TX_FLOAT_FFT, 1, FFT_N, &scale, 0)) < 0)
        return ret;

    for (int c = 0; c < WS_MAX_IDS; c++) {
        if (ws_sequences(c, seqA, seqB) < 0) {
            av_log(ctx, AV_LOG_ERROR, "LFSR mask is not primitive\n");
            return AVERROR_BUG;
        }
        build_reference(s, s->RefA + (size_t)c * FFT_N, seqA);
        build_reference(s, s->RefB + (size_t)c * FFT_N, seqB);
    }

    /* local oscillator, exactly periodic in 1023 samples */
    for (int n = 0; n < LO_PERIOD; n++) {
        double ph = -2.0 * M_PI * (double)(250 * n % LO_PERIOD) / LO_PERIOD;
        s->lo[n].re = cos(ph);
        s->lo[n].im = sin(ph);
    }

    /* windowed-sinc low-pass for the decimator, linear phase */
    for (int k = 0; k < FIR_TAPS; k++) {
        double x  = k - FIR_HALF;
        double wc = 2.0 * M_PI * FIR_CUTOFF / WS_ANALYSIS_RATE;
        double h  = x == 0 ? wc / M_PI : sin(wc * x) / (M_PI * x);
        h *= 0.54 - 0.46 * cos(2.0 * M_PI * k / (FIR_TAPS - 1));
        s->fir[k] = h;
        sum += h;
    }
    for (int k = 0; k < FIR_TAPS; k++)
        s->fir[k] /= sum;

    reset_acquisition(s, 0);
    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    WaterDetectContext *s = ctx->priv;
    av_tx_uninit(&s->tx);
    av_tx_uninit(&s->itx);
    av_freep(&s->bb);
    av_freep(&s->RefA);
    av_freep(&s->RefB);
    av_freep(&s->win);
    av_freep(&s->spec);
    av_freep(&s->prod);
    av_freep(&s->corr);
    av_freep(&s->cand);
    av_freep(&s->cand_mem);
    av_freep(&s->trk_mem);
}

/* ---- time bookkeeping ------------------------------------------------ */

/* Measured (reference clock) time of the input sample at baseband position x. */
static int64_t measured_us_at(const WaterDetectContext *s, double bb_pos)
{
    double n = 2.0 * bb_pos;
    if (s->clock_mode == CLOCK_PTS)
        return s->first_pts_us + llrint(n * 1e6 / WS_ANALYSIS_RATE);
    return s->now_us - llrint((s->in_count - n) * 1e6 / WS_ANALYSIS_RATE);
}

/* Recovered source time of the input sample at baseband position x. */
static int64_t recovered_us_at(const Tracker *t, double bb_pos)
{
    double dn = 2.0 * (bb_pos - t->ref_pos);
    return t->ref_t_us + llrint(dn * 1e6 / (WS_ANALYSIS_RATE * (1.0 + t->rate)));
}

static double drift_ppm(const Tracker *t)
{
    return (1.0 / (1.0 + t->rate) - 1.0) * 1e6;
}

/* ---- correlator ------------------------------------------------------ */

/* Read n baseband samples from fractional position x0 with step (1+rate),
 * linear interpolation. Caller guarantees the range is inside the ring. */
static void read_window(const WaterDetectContext *s, double x0, int n, AVComplexFloat *dst, double rate)
{
    const double step = 1.0 + rate;
    /* A speed change of (1+rate) also moves the carrier: the emitter period
     * is (1+rate) detector periods, so the carrier sits at CARRIER/(1+rate),
     * i.e. -CARRIER*rate Hz off the local oscillator, a full cycle per
     * period at 1000 ppm. Derotate by that much. A pure time-stretch leaves
     * the carrier alone, but it also splices the chip sequence and is not
     * recoverable anyway. */
    const double dth = 2.0 * M_PI * WS_CARRIER * rate * step / WS_BB_RATE;
    const float cr = cos(dth), ci = sin(dth);
    float rr = 1.f, ri = 0.f;
    for (int i = 0; i < n; i++) {
        double x  = x0 + i * step;
        int64_t k = (int64_t)floor(x);
        float f   = x - k;
        const AVComplexFloat *a = &s->bb[k & (BB_RING - 1)];
        const AVComplexFloat *b = &s->bb[(k + 1) & (BB_RING - 1)];
        float re = a->re + f * (b->re - a->re);
        float im = a->im + f * (b->im - a->im);
        float t;
        dst[i].re = re * rr - im * ri;
        dst[i].im = re * ri + im * rr;
        t  = rr * cr - ri * ci;
        ri = rr * ci + ri * cr;
        rr = t;
    }
}

static inline float mag2(AVComplexFloat c)
{
    return c.re * c.re + c.im * c.im;
}

/* Transform s->win into s->spec and limit strong spectral lines.
 *
 * A stationary in-band tone correlates with a code into a term of constant
 * magnitude and rotating phase at every lag, so once it exceeds the peak the
 * max-magnitude symbol decision fails. The watermark is spread flat, so
 * clipping bins far above the mean magnitude removes the tone and barely
 * touches the signal; for noise-like programme it is a no-op. */
static void fwd_spec(WaterDetectContext *s)
{
    float mean = 0.f, lim;

    s->tx_fn(s->tx, s->spec, s->win, sizeof(AVComplexFloat));
    if (s->limit_db <= 0)
        return;
    for (int k = 0; k < FFT_N; k++)
        mean += sqrtf(mag2(s->spec[k]));
    lim = mean / FFT_N * powf(10.f, s->limit_db / 20.f);
    for (int k = 0; k < FFT_N; k++) {
        float m = sqrtf(mag2(s->spec[k]));
        if (m > lim) {
            float g = lim / m;
            s->spec[k].re *= g;
            s->spec[k].im *= g;
        }
    }
}

/* Forward transform of a two-period window into s->spec. */
static void fwd_window(WaterDetectContext *s, double pos, double rate)
{
    read_window(s, pos, WIN_A, s->win, rate);
    memset(s->win + WIN_A, 0, (FFT_N - WIN_A) * sizeof(*s->win));
    fwd_spec(s);
}

/* corr[L] = sum_m win[m+L] * conj(ref[m]) from s->spec; exact for lags
 * 0..2045 as long as the window holds at least 4091 samples. */
static void corr_from_spec(WaterDetectContext *s, const AVComplexFloat *Ref)
{
    for (int k = 0; k < FFT_N; k++) {
        s->prod[k].re = s->spec[k].re * Ref[k].re - s->spec[k].im * Ref[k].im;
        s->prod[k].im = s->spec[k].re * Ref[k].im + s->spec[k].im * Ref[k].re;
    }
    s->itx_fn(s->itx, s->corr, s->prod, sizeof(AVComplexFloat));
}

/* Accumulate |s->corr|^2 into acc and measure the peak. */
static void accumulate(const WaterDetectContext *s, float *acc, float alpha, PeakStats *st)
{
    float peak = 0.f, mean = 0.f, msl = 0.f, y0, y1, y2;
    double delta;
    int pk = 0;

    for (int L = 0; L < WS_BB_PERIOD; L++) {
        acc[L] += alpha * (mag2(s->corr[L]) - acc[L]);
        if (acc[L] > peak) {
            peak = acc[L];
            pk   = L;
        }
    }
    for (int L = 0; L < WS_BB_PERIOD; L++) {
        int d = FFABS(L - pk);
        d = FFMIN(d, WS_BB_PERIOD - d);
        if (d > 8) {
            mean += acc[L];
            msl   = FFMAX(msl, acc[L]);
        }
    }
    mean /= WS_BB_PERIOD - 17;
    st->psr    = msl  > 0 ? peak / msl : 0;
    st->snr_db = mean > 0 ? 10.0 * log10(peak / mean) : 0;

    /* parabolic interpolation on magnitude */
    y0 = sqrtf(acc[(pk - 1 + WS_BB_PERIOD) % WS_BB_PERIOD]);
    y1 = sqrtf(peak);
    y2 = sqrtf(acc[(pk + 1) % WS_BB_PERIOD]);
    delta = (y0 - 2 * y1 + y2) != 0 ? 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2) : 0;
    st->phase = pk + av_clipd(delta, -0.5, 0.5);
}

/* ---- frames ---------------------------------------------------------- */

static void log_state(AVFilterContext *ctx, const Tracker *t, double at_pos)
{
    const WaterDetectContext *s = ctx->priv;
    int64_t tt  = t->lock ? recovered_us_at(t, at_pos) : 0;
    int64_t off = t->lock ? measured_us_at(s, at_pos) - tt : 0;

    av_log(ctx, AV_LOG_INFO,
           "waterdetect lock:%d id:%u t:%"PRId64".%03d offset:%s%"PRId64".%03d drift:%.1f snr:%.1f\n",
           t->lock, t->id, tt / 1000000, (int)(tt % 1000000 / 1000),
           off < 0 ? "-" : "", FFABS(off) / 1000, (int)(FFABS(off) % 1000),
           drift_ppm(t), t->snr_db);
}

static void set_lock(AVFilterContext *ctx, Tracker *t, int lock, double at_pos)
{
    if (t->lock == lock)
        return;
    t->lock = lock;
    log_state(ctx, t, at_pos);
}

static void try_frame(AVFilterContext *ctx, Tracker *t)
{
    WaterDetectContext *s = ctx->priv;
    uint8_t bits[WS_BITS];
    unsigned id, payload;
    double frame_pos;
    int64_t k;

    if (t->nsym < WS_SLOTS)
        return;

    for (int i = 0; i < WS_SLOTS; i++) {
        int sym = t->sym[(t->nsym - WS_SLOTS + i) % WS_SLOTS];
        bits[3 * i]     = (sym >> 2) & 1;
        bits[3 * i + 1] = (sym >> 1) & 1;
        bits[3 * i + 2] =  sym       & 1;
    }
    if (!ws_frame_parse(bits, &id, &payload))
        return;
    if (id != t->code) {
        av_log(ctx, AV_LOG_VERBOSE, "code %d decoded id %u, rejected\n", t->code, id);
        return;
    }

    frame_pos = t->sym_pos[(t->nsym - WS_SLOTS) % WS_SLOTS];

    /* A frame is confirmed by a previous valid frame whose payload matches
     * the number of slots elapsed since it. */
    if (t->have_valid) {
        double  dpos  = frame_pos - t->last_valid_pos;
        int64_t nfrm  = llrint(dpos / (FRAME_BB * (1.0 + t->rate)));
        double  resid = dpos - nfrm * FRAME_BB * (1.0 + t->rate);
        unsigned want = (t->last_payload + WS_FRAME_S * nfrm) & (WS_PAYLOAD_MOD - 1);

        if (nfrm >= 1 && fabs(resid) < 8.0 && want == payload) {
            /* coarse time: resolve the 9.1 h ambiguity */
            int64_t ref_s;
            if (s->epoch >= 0)
                ref_s = s->epoch;
            else if (s->clock_mode == CLOCK_WALL)
                ref_s = measured_us_at(s, frame_pos) / 1000000;
            else
                ref_s = payload;
            k = (ref_s - (int64_t)payload + WS_PAYLOAD_MOD / 2);
            k = k >= 0 ? k / WS_PAYLOAD_MOD : -((-k + WS_PAYLOAD_MOD - 1) / WS_PAYLOAD_MOD);

            if (t->lock && id != t->id)
                av_log(ctx, AV_LOG_WARNING, "source id changed %u -> %u\n", t->id, id);
            t->id       = id;
            t->ref_pos  = frame_pos;
            t->ref_t_us = ((int64_t)payload + k * WS_PAYLOAD_MOD) * 1000000;
            t->slots_since_valid = 0;
            if (!t->lock)
                set_lock(ctx, t, 1, frame_pos);
            else
                log_state(ctx, t, frame_pos);
        } else {
            av_log(ctx, AV_LOG_VERBOSE,
                   "frame id:%u payload:%u not consistent with previous (want %u, resid %.1f)\n",
                   id, payload, want, resid);
        }
    } else {
        av_log(ctx, AV_LOG_VERBOSE, "first CRC-valid frame id:%u payload:%u\n", id, payload);
    }
    t->have_valid     = 1;
    t->last_payload   = payload;
    t->last_valid_pos = frame_pos;
}

/* ---- trackers -------------------------------------------------------- */

static inline double period_bb(double rate)
{
    return WS_BB_PERIOD * (1.0 + rate);
}

static void release_tracker(AVFilterContext *ctx, Tracker *t, double at_pos)
{
    WaterDetectContext *s = ctx->priv;
    set_lock(ctx, t, 0, at_pos);
    s->tracked[t->code] = 0;
    for (int h = 0; h < NHYP; h++)
        memset(s->cand[t->code * NHYP + h].acc, 0, WS_BB_PERIOD * sizeof(float));
    memset(t, 0, offsetof(Tracker, acc));
    t->used = 0;
    memset(t->acc, 0, WS_BB_PERIOD * sizeof(float));
}

static void schedule_slot(Tracker *t, double phase)
{
    /* slot boundary: the period boundary nearest to the scheduled one */
    double cand = t->blk_pos - period_bb(t->rate) + phase * (1.0 + t->rate);
    if (!t->slot_valid) {
        t->slot_pos   = cand;
        t->slot_valid = 1;
        t->nsym       = 0;
    } else {
        double p = period_bb(t->rate);
        double d = cand - t->slot_pos;
        d -= p * floor(d / p + 0.5);
        t->slot_pos += d;
        while (t->slot_pos < t->last_slot_pos + p / 2)
            t->slot_pos += p;
    }
}

static void decode_slot(AVFilterContext *ctx, Tracker *t)
{
    WaterDetectContext *s = ctx->priv;
    float e[WS_CSK_M];
    int best = 0;

    read_window(s, t->slot_pos, WS_BB_PERIOD, s->win, t->rate);
    memcpy(s->win + WS_BB_PERIOD, s->win, WS_BB_PERIOD * sizeof(*s->win));
    memset(s->win + WIN_A, 0, (FFT_N - WIN_A) * sizeof(*s->win));
    fwd_spec(s);
    corr_from_spec(s, s->RefB + (size_t)t->code * FFT_N);

    /* symbol sym shifts the sequence by sym*128 chips; the peak then sits at
     * lag 2046 - 2*128*sym. Take the best of three bins to absorb the
     * half-sample uncertainty of the slot boundary. */
    for (int sym = 0; sym < WS_CSK_M; sym++) {
        int L = (WS_BB_PERIOD - 2 * WS_CSK_STEP * sym) % WS_BB_PERIOD;
        float m = 0.f;
        for (int d = -1; d <= 1; d++)
            m = FFMAX(m, mag2(s->corr[(L + d + WS_BB_PERIOD) % WS_BB_PERIOD]));
        e[sym] = m;
        if (m > e[best])
            best = sym;
    }

    t->sym[t->nsym % WS_SLOTS]     = best;
    t->sym_pos[t->nsym % WS_SLOTS] = t->slot_pos;
    t->nsym++;
    t->last_slot_pos = t->slot_pos;
    t->slot_pos     += period_bb(t->rate);

    if (t->lock && ++t->slots_since_valid > 3 * WS_SLOTS) {
        av_log(ctx, AV_LOG_VERBOSE, "id %u: no valid frame for %d slots\n",
               t->id, (int)t->slots_since_valid);
        set_lock(ctx, t, 0, t->slot_pos);
        t->have_valid = 0;
    }
    try_frame(ctx, t);
}

static void track_block(AVFilterContext *ctx, Tracker *t)
{
    WaterDetectContext *s = ctx->priv;
    const double thr = pow(10.0, s->thresh_db / 10.0);
    PeakStats st;
    double dp;

    fwd_window(s, t->blk_pos, t->rate);
    corr_from_spec(s, s->RefA + (size_t)t->code * FFT_N);
    accumulate(s, t->acc, 1.f / 16, &st);
    t->blk_pos += period_bb(t->rate);
    t->psr    = st.psr;
    t->snr_db = st.snr_db;

    if (st.psr < thr / 1.585) {                         /* 2 dB hysteresis */
        av_log(ctx, AV_LOG_VERBOSE, "code %d: layer A lost psr:%.1f dB\n",
               t->code, 10 * log10(st.psr));
        release_tracker(ctx, t, t->blk_pos);
        return;
    }
    /* rate loop: the peak moves by the residual sample-rate error */
    if (st.psr >= 1.3) {
        dp = st.phase - t->phase;
        if (dp >  WS_BB_PERIOD / 2) dp -= WS_BB_PERIOD;
        if (dp < -WS_BB_PERIOD / 2) dp += WS_BB_PERIOD;
        t->rate = av_clipd(t->rate + 0.1 * dp / WS_BB_PERIOD, -MAX_RATE, MAX_RATE);
    }
    t->phase = st.phase;
    schedule_slot(t, st.phase);
}

/* ---- acquisition ----------------------------------------------------- */

static int n_tracked(const WaterDetectContext *s)
{
    int n = 0;
    for (int i = 0; i < MAX_TRACKERS; i++)
        n += s->trk[i].used;
    return n;
}

static int code_wanted(const WaterDetectContext *s, int c)
{
    return !s->tracked[c] && (s->only_id < 0 || s->only_id == c);
}

static void acquire_block(AVFilterContext *ctx)
{
    WaterDetectContext *s = ctx->priv;
    const double thr = pow(10.0, s->thresh_db / 10.0);
    int any = 0, best_c = -1, best_h = -1, min_periods = INT_MAX;
    Tracker *t = NULL;

    for (int c = 0; c < WS_MAX_IDS; c++)
        any |= code_wanted(s, c);

    for (int h = 0; h < NHYP; h++) {
        if (any) {
            fwd_window(s, s->hyp_blk[h], s->hyp_rate[h]);
            for (int c = 0; c < WS_MAX_IDS; c++) {
                Candidate *cd = &s->cand[c * NHYP + h];
                if (!code_wanted(s, c))
                    continue;
                corr_from_spec(s, s->RefA + (size_t)c * FFT_N);
                accumulate(s, cd->acc, 1.f / 4, &cd->st);
            }
        }
        s->hyp_blk[h] += period_bb(s->hyp_rate[h]);
        s->hyp_periods[h]++;
        min_periods = FFMIN(min_periods, s->hyp_periods[h]);
    }
    if (!any || min_periods < MIN_PERIODS || n_tracked(s) >= s->max_sources)
        return;

    for (int c = 0; c < WS_MAX_IDS; c++) {
        if (!code_wanted(s, c))
            continue;
        for (int h = 0; h < NHYP; h++) {
            const Candidate *cd = &s->cand[c * NHYP + h];
            if (cd->st.psr >= thr && (best_c < 0 || cd->st.psr > s->cand[best_c * NHYP + best_h].st.psr)) {
                best_c = c;
                best_h = h;
            }
        }
    }
    if (best_c < 0)
        return;

    for (int i = 0; i < MAX_TRACKERS; i++)
        if (!s->trk[i].used) {
            t = &s->trk[i];
            s->tracked[best_c] = i + 1;
            break;
        }
    if (!t)
        return;

    {
        Candidate *cd = &s->cand[best_c * NHYP + best_h];
        memcpy(t->acc, cd->acc, WS_BB_PERIOD * sizeof(float));
        t->used    = 1;
        t->code    = best_c;
        t->blk_pos = s->hyp_blk[best_h];
        t->rate    = s->hyp_rate[best_h];
        t->phase   = cd->st.phase;
        t->psr     = cd->st.psr;
        t->snr_db  = cd->st.snr_db;
        av_log(ctx, AV_LOG_VERBOSE, "code %d: layer A lock psr:%.1f dB snr:%.1f dB rate:%.0f ppm\n",
               best_c, 10 * log10(cd->st.psr), cd->st.snr_db, t->rate * 1e6);
        schedule_slot(t, t->phase);
        for (int h = 0; h < NHYP; h++)
            memset(s->cand[best_c * NHYP + h].acc, 0, WS_BB_PERIOD * sizeof(float));
    }
}

static int acquire_ready(const WaterDetectContext *s)
{
    for (int h = 0; h < NHYP; h++)
        if (s->hyp_blk[h] + WIN_A * (1.0 + s->hyp_rate[h]) + 2 > s->bb_count)
            return 0;
    return 1;
}

static void run_detector(AVFilterContext *ctx)
{
    WaterDetectContext *s = ctx->priv;

    for (;;) {
        int did = 0;
        if (acquire_ready(s)) {
            acquire_block(ctx);
            did = 1;
        }
        for (int i = 0; i < MAX_TRACKERS; i++) {
            Tracker *t = &s->trk[i];
            double need_b;
            if (!t->used)
                continue;
            if (t->blk_pos + WIN_A * (1.0 + t->rate) + 2 <= s->bb_count) {
                track_block(ctx, t);
                did = 1;
                if (!t->used)
                    continue;
            }
            need_b = period_bb(t->rate) + 2;
            while (t->slot_valid && t->slot_pos + need_b <= s->bb_count) {
                if (t->slot_pos < s->bb_count - BB_RING + need_b) {
                    /* fell out of the ring; resynchronise on the next block */
                    t->slot_valid = 0;
                    t->nsym = 0;
                    break;
                }
                decode_slot(ctx, t);
                did = 1;
            }
        }
        if (!did)
            break;
    }
}

/* ---- input ----------------------------------------------------------- */

static void set_tracker_metadata(const Tracker *t, AVFrame *frame, const char *prefix,
                                 double bb_pos, int64_t measured_us)
{
    char key[64], buf[64];
    int64_t tt  = recovered_us_at(t, bb_pos);
    int64_t off = measured_us - tt;

    snprintf(key, sizeof(key), "%sid", prefix);
    av_dict_set_int(&frame->metadata, key, t->id, 0);
    snprintf(key, sizeof(key), "%st", prefix);
    av_dict_set_int(&frame->metadata, key, tt, 0);
    snprintf(key, sizeof(key), "%soffset", prefix);
    snprintf(buf, sizeof(buf), "%s%"PRId64".%03d", off < 0 ? "-" : "",
             FFABS(off) / 1000, (int)(FFABS(off) % 1000));
    av_dict_set(&frame->metadata, key, buf, 0);
    snprintf(key, sizeof(key), "%sdrift", prefix);
    snprintf(buf, sizeof(buf), "%.1f", drift_ppm(t));
    av_dict_set(&frame->metadata, key, buf, 0);
    snprintf(key, sizeof(key), "%ssnr", prefix);
    snprintf(buf, sizeof(buf), "%.1f", t->snr_db);
    av_dict_set(&frame->metadata, key, buf, 0);
}

static void set_metadata(WaterDetectContext *s, AVFrame *frame, double bb_pos, int64_t measured_us)
{
    const Tracker *primary = NULL;
    char prefix[48], buf[32];
    int n = 0;

    for (int i = 0; i < MAX_TRACKERS; i++) {
        const Tracker *t = &s->trk[i];
        if (!t->used || !t->lock)
            continue;
        if (!primary)
            primary = t;
        snprintf(prefix, sizeof(prefix), "lavfi.waterdetect.%d.", n++);
        set_tracker_metadata(t, frame, prefix, bb_pos, measured_us);
    }
    av_dict_set_int(&frame->metadata, "lavfi.waterdetect.lock", primary != NULL, 0);
    av_dict_set_int(&frame->metadata, "lavfi.waterdetect.sources", n, 0);
    if (primary) {
        set_tracker_metadata(primary, frame, "lavfi.waterdetect.", bb_pos, measured_us);
    } else {
        /* still report layer A quality of the best tracker while acquiring frames */
        for (int i = 0; i < MAX_TRACKERS; i++)
            if (s->trk[i].used) {
                snprintf(buf, sizeof(buf), "%.1f", s->trk[i].snr_db);
                av_dict_set(&frame->metadata, "lavfi.waterdetect.snr", buf, 0);
                snprintf(buf, sizeof(buf), "%.1f", drift_ppm(&s->trk[i]));
                av_dict_set(&frame->metadata, "lavfi.waterdetect.drift", buf, 0);
                break;
            }
    }
}

static int filter_frame(AVFilterLink *inlink, AVFrame *frame)
{
    AVFilterContext *ctx = inlink->dst;
    WaterDetectContext *s  = ctx->priv;
    const float *x = (const float *)frame->data[0];
    const double frame_bb = s->in_count / 2.0;
    int64_t measured_us;

    s->now_us = av_gettime();
    if (!s->have_first_pts) {
        s->first_pts_us = frame->pts == AV_NOPTS_VALUE ? 0 :
                          av_rescale_q(frame->pts, inlink->time_base, AV_TIME_BASE_Q);
        s->have_first_pts = 1;
    }
    measured_us = s->clock_mode == CLOCK_PTS
                ? (frame->pts == AV_NOPTS_VALUE ? measured_us_at(s, frame_bb)
                                                : av_rescale_q(frame->pts, inlink->time_base, AV_TIME_BASE_Q))
                : s->now_us;

    /* demodulate, low-pass, decimate by two into the baseband ring */
    for (int i = 0; i < frame->nb_samples; i++) {
        const AVComplexFloat lo = s->lo[s->in_count % LO_PERIOD];
        int64_t n = s->in_count++;
        s->dline[s->dl_pos].re = x[i] * lo.re;
        s->dline[s->dl_pos].im = x[i] * lo.im;
        s->dl_pos = (s->dl_pos + 1) % FIR_TAPS;
        /* output m is centred on input sample 2m */
        if (n >= FIR_HALF && !((n - FIR_HALF) & 1)) {
            AVComplexFloat acc = { 0, 0 };
            int p = s->dl_pos;                     /* oldest sample */
            for (int k = FIR_TAPS - 1; k >= 0; k--) {
                acc.re += s->fir[k] * s->dline[p].re;
                acc.im += s->fir[k] * s->dline[p].im;
                p = (p + 1) % FIR_TAPS;
            }
            s->bb[s->bb_count & (BB_RING - 1)] = acc;
            s->bb_count++;
        }
    }

    run_detector(ctx);
    set_metadata(s, frame, frame_bb, measured_us);
    return ff_filter_frame(ctx->outputs[0], frame);
}

static const AVFilterPad waterdetect_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_waterdetect = {
    .p.name        = "waterdetect",
    .p.description = NULL_IF_CONFIG_SMALL("Recover the waterstamp time stamp and report the offset."),
    .p.priv_class  = &waterdetect_class,
    .priv_size     = sizeof(WaterDetectContext),
    .init          = init,
    .uninit        = uninit,
    FILTER_INPUTS(waterdetect_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_QUERY_FUNC2(query_formats),
};

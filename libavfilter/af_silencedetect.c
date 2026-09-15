/*
 * Copyright (c) 2012 Clément Bœsch <u pkh me>
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
 * Audio silence detector
 */

#include <float.h> /* DBL_MAX */
#include <math.h>

#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/timestamp.h"
#include "audio.h"
#include "avfilter.h"
#include "ebur128.h"
#include "filters.h"

enum SilenceDetectMode {
    MODE_PEAK,
    MODE_LOUDNESS,
    MODE_NB,
};

typedef struct SilenceDetectContext {
    const AVClass *class;
    FFEBUR128State **r128;

    double noise;               ///< noise amplitude ratio
    double relative;            ///< relative amplitude ratio
    int64_t duration;           ///< minimum duration of silence until notification
    int mode;                   ///< enum SilenceDetectMode
    int mono;                   ///< mono mode : check each channel separately (default = check when ALL channels are silent)
    int channels;               ///< number of channels
    int independent_channels;   ///< number of entries in following arrays (always 1 in mono mode)
    int64_t *nb_null_samples;   ///< (array) current number of continuous zero samples
    int64_t *start;             ///< (array) if silence is detected, this value contains the time of the first zero sample (default/unset = INT64_MIN)
    int64_t frame_end;          ///< pts of the end of the current frame (used to compute duration of silence at EOS)
    int last_sample_rate;       ///< last sample rate to check for sample rate changes
    AVRational time_base;       ///< time_base
    int nb_pending_samples;     ///< number of samples until loudness is measured
    int *is_silence;            ///< (array) result of previous loudness measurement

    void (*silencedetect)(AVFilterContext *ctx, AVFrame *insamples,
                          int nb_samples, int64_t nb_samples_notify,
                          AVRational time_base);
} SilenceDetectContext;

#define MAX_DURATION (24*3600*1000000LL)
#define LOUDNESS_STEP(sample_rate)   (((sample_rate) + 5) / 10)       /* 100 ms */
#define LOUDNESS_WINDOW(sample_rate) (4 * LOUDNESS_STEP(sample_rate)) /* 400 ms */
#define LOUDNESS_TOLERANCE 0.5 /* relative loudness tolerance (LU) */
#define OFFSET(x) offsetof(SilenceDetectContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_AUDIO_PARAM
static const AVOption silencedetect_options[] = {
    { "mode",      "set how to detect silence",        OFFSET(mode),      AV_OPT_TYPE_INT,    {.i64=MODE_PEAK},      0, MODE_NB-1,FLAGS, .unit = "mode" },
    {   "peak",     "detect each sample peak",         0,                 AV_OPT_TYPE_CONST,  {.i64=MODE_PEAK},      0, 0,        FLAGS, .unit = "mode" },
    {   "loudness", "detect momentary loudness",       0,                 AV_OPT_TYPE_CONST,  {.i64=MODE_LOUDNESS},  0, 0,        FLAGS, .unit = "mode" },
    { "n",         "set noise tolerance",              OFFSET(noise),     AV_OPT_TYPE_DOUBLE, {.dbl=0.001},          0, DBL_MAX,  FLAGS },
    { "noise",     "set noise tolerance",              OFFSET(noise),     AV_OPT_TYPE_DOUBLE, {.dbl=0.001},          0, DBL_MAX,  FLAGS },
    { "relative",  "set relative noise threshold",     OFFSET(relative),  AV_OPT_TYPE_DOUBLE, {.dbl=0},              0, 1,        FLAGS },
    { "d",         "set minimum duration in seconds",  OFFSET(duration),  AV_OPT_TYPE_DURATION, {.i64=2000000},      0, MAX_DURATION,FLAGS },
    { "duration",  "set minimum duration in seconds",  OFFSET(duration),  AV_OPT_TYPE_DURATION, {.i64=2000000},      0, MAX_DURATION,FLAGS },
    { "mono",      "check each channel separately",    OFFSET(mono),      AV_OPT_TYPE_BOOL,   {.i64=0},              0, 1,        FLAGS },
    { "m",         "check each channel separately",    OFFSET(mono),      AV_OPT_TYPE_BOOL,   {.i64=0},              0, 1,        FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(silencedetect);

static void set_meta(AVFrame *insamples, int channel, const char *key, char *value)
{
    char key2[128];

    if (channel)
        snprintf(key2, sizeof(key2), "lavfi.%s.%d", key, channel);
    else
        snprintf(key2, sizeof(key2), "lavfi.%s", key);
    av_dict_set(&insamples->metadata, key2, value, 0);
}
static av_always_inline void update(AVFilterContext *ctx, AVFrame *insamples,
                                    int is_silence, int current_sample, int64_t nb_samples_notify,
                                    AVRational time_base)
{
    SilenceDetectContext *s = ctx->priv;
    int channel = current_sample % s->independent_channels;
    if (is_silence) {
        if (s->start[channel] == INT64_MIN) {
            s->nb_null_samples[channel]++;
            if (s->nb_null_samples[channel] >= nb_samples_notify) {
                s->start[channel] = insamples->pts + av_rescale_q(current_sample / s->channels + 1 - nb_samples_notify * s->independent_channels / s->channels,
                        (AVRational){ 1, s->last_sample_rate }, time_base);
                set_meta(insamples, s->mono ? channel + 1 : 0, "silence_start",
                        av_ts2timestr(s->start[channel], &time_base));
                if (s->mono)
                    av_log(ctx, AV_LOG_INFO, "channel: %d | ", channel);
                av_log(ctx, AV_LOG_INFO, "silence_start: %s\n",
                        av_ts2timestr(s->start[channel], &time_base));
            }
        }
    } else {
        if (s->start[channel] > INT64_MIN) {
            int64_t end_pts = insamples ? insamples->pts + av_rescale_q(current_sample / s->channels,
                    (AVRational){ 1, s->last_sample_rate }, time_base)
                    : s->frame_end;

            /* momentary loudness lags the actual signal by up to the window
             * size when it rises, so report the end of silence earlier to
             * compensate, except at the end of the stream */
            if (insamples && s->mode == MODE_LOUDNESS)
                end_pts -= av_rescale_q(LOUDNESS_WINDOW(s->last_sample_rate),
                                        (AVRational){ 1, s->last_sample_rate }, time_base);

            int64_t duration_ts = end_pts - s->start[channel];
            if (insamples) {
                set_meta(insamples, s->mono ? channel + 1 : 0, "silence_end",
                        av_ts2timestr(end_pts, &time_base));
                set_meta(insamples, s->mono ? channel + 1 : 0, "silence_duration",
                        av_ts2timestr(duration_ts, &time_base));
            }
            if (s->mono)
                av_log(ctx, AV_LOG_INFO, "channel: %d | ", channel);
            av_log(ctx, AV_LOG_INFO, "silence_end: %s | silence_duration: %s\n",
                    av_ts2timestr(end_pts, &time_base),
                    av_ts2timestr(duration_ts, &time_base));
        }
        s->nb_null_samples[channel] = 0;
        s->start[channel] = INT64_MIN;
    }
}

#define SILENCE_DETECT(name, type)                                               \
static void silencedetect_##name(AVFilterContext *ctx, AVFrame *insamples,       \
                                 int nb_samples, int64_t nb_samples_notify,      \
                                 AVRational time_base)                           \
{                                                                                \
    SilenceDetectContext *s         = ctx->priv;                                 \
    const type *p = (const type *)insamples->data[0];                            \
    const type noise = s->noise;                                                 \
    int i;                                                                       \
                                                                                 \
    for (i = 0; i < nb_samples; i++, p++)                                        \
        update(ctx, insamples, *p < noise && *p > -noise, i,                     \
               nb_samples_notify, time_base);                                    \
}

#define SILENCE_DETECT_PLANAR(name, type)                                        \
static void silencedetect_##name(AVFilterContext *ctx, AVFrame *insamples,       \
                                 int nb_samples, int64_t nb_samples_notify,      \
                                 AVRational time_base)                           \
{                                                                                \
    SilenceDetectContext *s         = ctx->priv;                                 \
    const int channels = insamples->ch_layout.nb_channels;                       \
    const type noise = s->noise;                                                 \
                                                                                 \
    nb_samples /= channels;                                                      \
    for (int i = 0; i < nb_samples; i++) {                                       \
        for (int ch = 0; ch < insamples->ch_layout.nb_channels; ch++) {          \
            const type *p = (const type *)insamples->extended_data[ch];          \
            update(ctx, insamples, p[i] < noise && p[i] > -noise,                \
                   channels * i + ch,                                            \
                   nb_samples_notify, time_base);                                \
        }                                                                        \
    }                                                                            \
}

SILENCE_DETECT(dbl, double)
SILENCE_DETECT(flt, float)
SILENCE_DETECT(s32, int32_t)
SILENCE_DETECT(s16, int16_t)

SILENCE_DETECT_PLANAR(dblp, double)
SILENCE_DETECT_PLANAR(fltp, float)
SILENCE_DETECT_PLANAR(s32p, int32_t)
SILENCE_DETECT_PLANAR(s16p, int16_t)

static void update_loudness(SilenceDetectContext *s)
{
    /* measure the loudness every 100ms, per ITU-R BS.1770 */
    if (--s->nb_pending_samples)
        return;

    s->nb_pending_samples = LOUDNESS_STEP(s->last_sample_rate);
    for (int ch = 0; ch < s->channels; ch++) {
        double loudness, shortterm;
        int below, above;

        ff_ebur128_loudness_momentary(s->r128[ch], &loudness);
        below = loudness <  s->noise;
        above = loudness >= s->noise;
        if (isfinite(s->relative)) {
            /* we use an asymmetric tolerance; silence starts when the
             * content dips by the configured threshold, and stops again when
             * it reaches the short-term average minus a small constant */
            ff_ebur128_loudness_shortterm(s->r128[ch], &shortterm);
            below |= loudness < shortterm + s->relative;
            above &= loudness >= shortterm - LOUDNESS_TOLERANCE;
        }
        s->is_silence[ch] = s->is_silence[ch] ? !above : below;
    }
}

#define SILENCE_DETECT_LOUDNESS(name, type)                                      \
static void silencedetect_##name(AVFilterContext *ctx, AVFrame *insamples,       \
                                 int nb_samples, int64_t nb_samples_notify,      \
                                 AVRational time_base)                           \
{                                                                                \
    SilenceDetectContext *s = ctx->priv;                                         \
    const int channels = insamples->ch_layout.nb_channels;                       \
    const type *p = (const type *) insamples->data[0];                           \
                                                                                 \
    nb_samples /= channels;                                                      \
    for (int i = 0; i < nb_samples; i++) {                                       \
        for (int ch = 0; ch < channels; ch++, p++) {                             \
            update(ctx, insamples, s->is_silence[ch], channels * i + ch,         \
                   nb_samples_notify, time_base);                                \
            ff_ebur128_add_frames_##type(s->r128[ch], p, 1);                     \
        }                                                                        \
        update_loudness(s);                                                      \
    }                                                                            \
}

#define SILENCE_DETECT_LOUDNESS_PLANAR(name, type)                               \
static void silencedetect_##name(AVFilterContext *ctx, AVFrame *insamples,       \
                                 int nb_samples, int64_t nb_samples_notify,      \
                                 AVRational time_base)                           \
{                                                                                \
    SilenceDetectContext *s = ctx->priv;                                         \
    const int channels = insamples->ch_layout.nb_channels;                       \
                                                                                 \
    nb_samples /= channels;                                                      \
    for (int i = 0; i < nb_samples; i++) {                                       \
        for (int ch = 0; ch < channels; ch++) {                                  \
            const type *p = (const type *) insamples->extended_data[ch];         \
            update(ctx, insamples, s->is_silence[ch], channels * i + ch,         \
                   nb_samples_notify, time_base);                                \
            ff_ebur128_add_frames_##type(s->r128[ch], &p[i], 1);                 \
        }                                                                        \
        update_loudness(s);                                                      \
    }                                                                            \
}

SILENCE_DETECT_LOUDNESS(dbl_loudness, double)
SILENCE_DETECT_LOUDNESS(flt_loudness, float)
SILENCE_DETECT_LOUDNESS(s32_loudness, int)
SILENCE_DETECT_LOUDNESS(s16_loudness, short)

SILENCE_DETECT_LOUDNESS_PLANAR(dblp_loudness, double)
SILENCE_DETECT_LOUDNESS_PLANAR(fltp_loudness, float)
SILENCE_DETECT_LOUDNESS_PLANAR(s32p_loudness, int)
SILENCE_DETECT_LOUDNESS_PLANAR(s16p_loudness, short)

static int config_input(AVFilterLink *inlink)
{
    AVFilterContext *ctx = inlink->dst;
    SilenceDetectContext *s = ctx->priv;
    int c;

    s->channels = inlink->ch_layout.nb_channels;
    s->duration = av_rescale(s->duration, inlink->sample_rate, AV_TIME_BASE);
    s->independent_channels = s->mono ? s->channels : 1;
    s->nb_null_samples = av_calloc(s->independent_channels,
                                   sizeof(*s->nb_null_samples));
    if (!s->nb_null_samples)
        return AVERROR(ENOMEM);
    s->start = av_malloc_array(s->independent_channels, sizeof(*s->start));
    if (!s->start)
        return AVERROR(ENOMEM);
    for (c = 0; c < s->independent_channels; c++)
        s->start[c] = INT64_MIN;

    switch (s->mode) {
    case MODE_PEAK:
        switch (inlink->format) {
        case AV_SAMPLE_FMT_DBL: s->silencedetect = silencedetect_dbl; break;
        case AV_SAMPLE_FMT_FLT: s->silencedetect = silencedetect_flt; break;
        case AV_SAMPLE_FMT_S32:
            s->noise *= INT32_MAX;
            s->silencedetect = silencedetect_s32;
            break;
        case AV_SAMPLE_FMT_S16:
            s->noise *= INT16_MAX;
            s->silencedetect = silencedetect_s16;
            break;
        case AV_SAMPLE_FMT_DBLP: s->silencedetect = silencedetect_dblp; break;
        case AV_SAMPLE_FMT_FLTP: s->silencedetect = silencedetect_fltp; break;
        case AV_SAMPLE_FMT_S32P:
            s->noise *= INT32_MAX;
            s->silencedetect = silencedetect_s32p;
            break;
        case AV_SAMPLE_FMT_S16P:
            s->noise *= INT16_MAX;
            s->silencedetect = silencedetect_s16p;
            break;
        default:
            return AVERROR_BUG;
        }
        break;

    case MODE_LOUDNESS:
        s->r128       = av_calloc(s->channels, sizeof(*s->r128));
        s->is_silence = av_calloc(s->channels, sizeof(*s->is_silence));
        if (!s->r128 || !s->is_silence)
            return AVERROR(ENOMEM);
        for (c = 0; c < s->channels; c++) {
            s->r128[c] = ff_ebur128_init(1, inlink->sample_rate, 0,
                                         s->relative ? FF_EBUR128_MODE_S : FF_EBUR128_MODE_M);
            if (!s->r128[c])
                return AVERROR(ENOMEM);
        }
        s->nb_pending_samples = LOUDNESS_STEP(inlink->sample_rate);
        s->noise = 20 * log10(s->noise); /* convert to LUFS */
        s->relative = 20 * log10(s->relative); /* or -infinity if disabled */
        /* silence must outlast the window its end is moved back by, see update() */
        s->duration += LOUDNESS_WINDOW(inlink->sample_rate);

        switch (inlink->format) {
        case AV_SAMPLE_FMT_DBL:  s->silencedetect = silencedetect_dbl_loudness;  break;
        case AV_SAMPLE_FMT_FLT:  s->silencedetect = silencedetect_flt_loudness;  break;
        case AV_SAMPLE_FMT_S32:  s->silencedetect = silencedetect_s32_loudness;  break;
        case AV_SAMPLE_FMT_S16:  s->silencedetect = silencedetect_s16_loudness;  break;
        case AV_SAMPLE_FMT_DBLP: s->silencedetect = silencedetect_dblp_loudness; break;
        case AV_SAMPLE_FMT_FLTP: s->silencedetect = silencedetect_fltp_loudness; break;
        case AV_SAMPLE_FMT_S32P: s->silencedetect = silencedetect_s32p_loudness; break;
        case AV_SAMPLE_FMT_S16P: s->silencedetect = silencedetect_s16p_loudness; break;
        default: return AVERROR_BUG;
        }
        break;
    }

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *insamples)
{
    AVFilterContext *ctx            = inlink->dst;
    SilenceDetectContext *s         = ctx->priv;
    const int nb_channels           = inlink->ch_layout.nb_channels;
    const int srate                 = inlink->sample_rate;
    const int nb_samples            = insamples->nb_samples     * nb_channels;
    const int64_t nb_samples_notify = s->duration * (s->mono ? 1 : nb_channels);
    int c;

    // scale number of null samples to the new sample rate
    if (s->last_sample_rate && s->last_sample_rate != srate) {
        for (c = 0; c < s->independent_channels; c++)
            s->nb_null_samples[c] = srate * s->nb_null_samples[c] / s->last_sample_rate;
        s->nb_pending_samples = srate * s->nb_pending_samples / s->last_sample_rate;
    }
    s->last_sample_rate = srate;
    s->time_base = inlink->time_base;
    s->frame_end = insamples->pts + av_rescale_q(insamples->nb_samples,
            (AVRational){ 1, s->last_sample_rate }, inlink->time_base);

    s->silencedetect(ctx, insamples, nb_samples, nb_samples_notify,
                     inlink->time_base);

    return ff_filter_frame(inlink->dst->outputs[0], insamples);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    SilenceDetectContext *s = ctx->priv;
    int c;

    for (c = 0; c < s->independent_channels; c++)
        if (s->start[c] > INT64_MIN)
            update(ctx, NULL, 0, c, 0, s->time_base);
    for (c = 0; s->r128 && c < s->channels; c++)
        ff_ebur128_destroy(&s->r128[c]);
    av_freep(&s->r128);
    av_freep(&s->is_silence);
    av_freep(&s->nb_null_samples);
    av_freep(&s->start);
}

static const AVFilterPad silencedetect_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_AUDIO,
        .config_props = config_input,
        .filter_frame = filter_frame,
    },
};

const FFFilter ff_af_silencedetect = {
    .p.name        = "silencedetect",
    .p.description = NULL_IF_CONFIG_SMALL("Detect silence."),
    .p.priv_class  = &silencedetect_class,
    .p.flags       = AVFILTER_FLAG_METADATA_ONLY,
    .priv_size     = sizeof(SilenceDetectContext),
    .uninit        = uninit,
    FILTER_INPUTS(silencedetect_inputs),
    FILTER_OUTPUTS(ff_audio_default_filterpad),
    FILTER_SAMPLEFMTS(AV_SAMPLE_FMT_DBL, AV_SAMPLE_FMT_DBLP,
                      AV_SAMPLE_FMT_FLT, AV_SAMPLE_FMT_FLTP,
                      AV_SAMPLE_FMT_S32, AV_SAMPLE_FMT_S32P,
                      AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_S16P),
};

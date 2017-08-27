#include "avformat.h"
#include "internal.h"
#include "avio_internal.h"
#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "libavutil/timecode.h"
#include "nxt.h"

static int nxt_probe(AVProbeData *p)
{
    NXTHeader *nxt = (NXTHeader*)p->buf;
    return (nxt->tag & NXT_TAG_MASK) == NXT_TAG ? AVPROBE_SCORE_MAX : 0;
}

static int64_t nxt_read_timestamp(AVFormatContext *s, int stream_index, int64_t *ppos, int64_t pos_limit)
{
    char buf[NXT_ALIGN];
    NXTHeader *nxt = (NXTHeader*)buf;
    int64_t pos = *ppos;

    if (stream_index > 0) {
        return AV_NOPTS_VALUE;
    }

    if (pos % NXT_ALIGN > 0) {
        pos += NXT_ALIGN - (pos % NXT_ALIGN);
    }

    if (pos >= avio_size(s->pb)) {
        return AV_NOPTS_VALUE;
    }

    if (avio_seek(s->pb, pos, SEEK_SET) < 0) {
        return AV_NOPTS_VALUE;
    }

    pos_limit = FFMIN(pos_limit, pos + NXT_MAX_FRAME_SIZE);

    while (pos < pos_limit) {
        if (avio_read(s->pb, (char*)nxt, NXT_ALIGN) != NXT_ALIGN) {
            return AV_NOPTS_VALUE;
        } else if ((nxt->tag & NXT_TAG_MASK) == NXT_TAG) {
            *ppos = pos;
            return nxt->pts;
        } else {
            pos += NXT_ALIGN;
        }
    }

    return AV_NOPTS_VALUE;
}

static int nxt_read_timecode (AVStream *st, NXTHeader *nxt)
{
    char buf[AV_TIMECODE_STR_SIZE];
    int tc_flags = 0;
    AVTimecode tc;
    AVRational tc_rate = { 0 };
    
    switch (nxt->ltc_format) 
    {
    case NXT_LTC_50:
        tc_rate.num = 50;
        tc_rate.den = 1;
        av_log(NULL, AV_LOG_VERBOSE, "[nxt] LTC_50");
    break;
    case NXT_LTC_25:
        tc_rate.num = 25;
        tc_rate.den = 1;
        av_log(NULL, AV_LOG_VERBOSE, "[nxt] LTC_25");
    break;
    default:
        return -1;
    }
    
    if (av_timecode_init(&tc, tc_rate, tc_flags, nxt->ltc, NULL) == 0) {
        av_dict_set(&st->metadata, "timecode", av_timecode_make_string(&tc, buf, 0), 0);
    }

    return 0;
}

static int nxt_read_stream(AVStream *st, NXTHeader *nxt)
{
    st->start_time = nxt->pts;

    switch (nxt->format)
    {
    case NXT_DNXHD_120_1080i50:
        st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id = AV_CODEC_ID_DNXHD;
        st->codecpar->codec_tag = MKTAG('A','V','d','h');
        st->codecpar->format = AV_PIX_FMT_YUV422P;
        st->codecpar->field_order = AV_FIELD_TT;
        st->codecpar->sample_aspect_ratio.num = 1;
        st->codecpar->sample_aspect_ratio.den = 1;
        st->codecpar->bit_rate = 120000000;
        st->codecpar->width = 1920;
        st->codecpar->height = 1080;

        st->time_base.num = 1;
        st->time_base.den = 2500;

        st->avg_frame_rate.num = 25;
        st->avg_frame_rate.den = 1;

        av_log(NULL, AV_LOG_VERBOSE, "[nxt] DNXHD_120_1080i50");

        return 0;
    case NXT_PCM_S32LE_48000c8:
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S32LE;
        st->codecpar->codec_tag = 0;
        st->codecpar->format = AV_SAMPLE_FMT_S32;
        st->codecpar->block_align = 8 * 4;
        st->codecpar->channels = 8;
        st->codecpar->sample_rate = 48000;
        st->codecpar->bits_per_coded_sample = 32;
        st->codecpar->bits_per_raw_sample = 32;

        st->time_base.num = 1;
        st->time_base.den = 48000;

        av_log(NULL, AV_LOG_VERBOSE, "[nxt] PCM_S32LE_48000c8");

        return 0;
    case NXT_DNXHD_115_720p50:
        st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id = AV_CODEC_ID_DNXHD;
        st->codecpar->codec_tag = MKTAG('A','V','d','h');
        st->codecpar->format = AV_PIX_FMT_YUV422P;
        st->codecpar->field_order = AV_FIELD_PROGRESSIVE;
        st->codecpar->sample_aspect_ratio.num = 1;
        st->codecpar->sample_aspect_ratio.den = 1;
        st->codecpar->bit_rate = 115000000;
        st->codecpar->width = 1280;
        st->codecpar->height = 720;

        st->time_base.num = 1;
        st->time_base.den = 5000;

        st->avg_frame_rate.num = 50;
        st->avg_frame_rate.den = 1;

        av_log(NULL, AV_LOG_VERBOSE, "[nxt] DNXHD_115_720p50");

        return 0;
      default:
        av_log(NULL, AV_LOG_ERROR, "[nxt] invalid format %d\n", nxt->format);
        return -1;
    }
}

static int nxt_read_header(AVFormatContext *s)
{
    int ret;
    // int64_t last_ts = 0, 
    int64_t pos, lastpos = -1;
    char buf[NXT_ALIGN];
    NXTHeader *nxt = (NXTHeader*)buf;
    AVStream *st = NULL;

    av_log(NULL, AV_LOG_VERBOSE, "[nxt] read_header\n");  

    ret = avio_read(s->pb, buf, NXT_ALIGN);
    if (ret != NXT_ALIGN) {
        av_log(NULL, AV_LOG_ERROR, "[nxt] avio_read header failed %d eof %d\n", ret, avio_feof(s->pb));
        if (ret >= 0) {
            ret = -1;
        }
        return ret;
    }

    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG) {
        av_log(NULL, AV_LOG_ERROR, "[nxt] invalid tag\n");
        return -1;
    }

    av_log(NULL, AV_LOG_INFO, "[nxt] tag: %" PRId64 " index: %" PRId64 " position %" PRId64 " format: %d pts: %" PRId64 " ltc: %d ltc_format: %d", 
        nxt->tag, nxt->index, nxt->position, nxt->format, nxt->pts, nxt->ltc, nxt->ltc_format);

    ret = avio_tell(s->pb);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "[nxt] avio_tell failed %d\n", ret);
        return ret;
    }
    pos = ret - NXT_ALIGN;

    st = avformat_new_stream(s, NULL);
    if (!st) {
        ret = AVERROR(ENOMEM);
        av_log(NULL, AV_LOG_ERROR, "[nxt] avformat_new_stream failed %d\n", ret);
        return ret;
    }

    // ff_find_last_ts(s, -1, &last_ts, &lastpos, nxt_read_timestamp);
    // if (last_ts > 0) {
    //     s->duration_estimation_method = AVFMT_DURATION_FROM_PTS;
    //     st->duration = last_ts - nxt->pts;
    // }

    ret = avio_seek(s->pb, pos, SEEK_SET);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "[nxt] avio_seek failed %d\n", ret);
        goto fail;
    }

    ret = nxt_read_stream(st, nxt);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "[nxt] nxt_read_stream failed %d\n", ret);
        goto fail;
    }

    nxt_read_timecode(st, nxt);

    av_log(NULL, AV_LOG_INFO, "[nxt] start_time: %" PRId64 " time_base: %d/%d\n", st->start_time, st->time_base.num, st->time_base.den);

    return 0;
fail:
    av_free(st);
    return ret;
}

static int nxt_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret;
    char buf[NXT_ALIGN];
    NXTHeader *nxt = (NXTHeader*)buf;

    ret = avio_read(s->pb, buf, NXT_ALIGN);
    if (ret == AVERROR_EOF) {
        return ret;
    } else if (ret != NXT_ALIGN) {
        av_log(NULL, AV_LOG_ERROR, "[nxt] avio_read packet header failed %d\n", ret);
        return ret;
    }

    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG) {
        av_log(NULL, AV_LOG_ERROR, "[nxt] invalid tag\n");
        return -1;
    }

    ret = av_new_packet(pkt, nxt->next - NXT_ALIGN);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "[nxt] av_new_packet failed %d\n", ret);
        goto fail;
    }

    ret = avio_read(s->pb, pkt->data, pkt->size);
    if (ret < nxt->size) {
        av_log(NULL, AV_LOG_ERROR, "[nxt] avio_read packet data failed %d eof %d\n", ret, avio_feof(s->pb));
        if (ret >= 0) {
            ret = -1;
        }
        goto fail;
    }

    av_shrink_packet(pkt, nxt->size);

    pkt->pos = avio_tell(s->pb) - nxt->next;
    pkt->stream_index = 0;
    pkt->flags = AV_PKT_FLAG_KEY;
    pkt->duration = nxt->duration;
    pkt->pts = nxt->pts;

    // av_log(NULL, AV_LOG_INFO, "[nxt] pts: %" PRId64 " ltc: %d #%d\n", pkt->pts, nxt->ltc, nxt->format);

    return 0;
fail:
    av_packet_unref(pkt);
    return ret;
}

AVInputFormat ff_nxt_demuxer = {
    .name           = "nxt",
    .long_name      = NULL_IF_CONFIG_SMALL("NXT"),
    .read_probe     = nxt_probe,
    .read_header    = nxt_read_header,
    .read_packet    = nxt_read_packet,
    .read_timestamp = nxt_read_timestamp,
    .extensions     = "nxt"
};

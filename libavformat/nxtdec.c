#include "avformat.h"
#include "internal.h"
#include "avio_internal.h"
#include "libavutil/avassert.h"
#include "libavutil/common.h"
#include "nxt.h"

static int nxt_probe(AVProbeData *p)
{
    int i;
    NXTHeader *nxt;

    for (i = 0; i < p->buf_size - sizeof(nxt->tag); i++) {
      nxt = (NXTHeader*)(p->buf + i);
      if ((nxt->tag & NXT_TAG_MASK) == NXT_TAG)
          return AVPROBE_SCORE_MAX;
    }

    return 0;
}

static int nxt_floor(int val)
{
    return (val / NXT_ALIGN) * NXT_ALIGN;
}

static int nxt_seek_fwd(AVFormatContext *s, NXTHeader* nxt)
{
    int ret, i;
    AVIOContext *bc = s->pb;

    for (i = 0; i < NXT_MAX_FRAME_SIZE; i += NXT_ALIGN) {
        ret = avio_read(bc, (char*)nxt, NXT_ALIGN);

        if (ret < 0)
            return ret;

        if (ret < NXT_ALIGN)
            return -1;

        if ((nxt->tag & NXT_TAG_MASK) == NXT_TAG)
            return 0;
    }

    return -1;
}

static int nxt_read_header(AVFormatContext *s)
{
    int ret;
    NXTHeader *nxt = (NXTHeader*)s->priv_data;
    AVStream *st = NULL;
    AVIOContext *bc = s->pb;

    av_log(NULL, AV_LOG_VERBOSE, "nxt: read_header \n");
    
    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG) {
        av_log(NULL, AV_LOG_ERROR, "nxt: invalid tag %" PRId64 "\n", nxt->tag);
        return -1;        
    }

    ret = avio_read(bc, (char*)nxt, NXT_ALIGN);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "nxt: avio_read failed %d\n", ret);
        return ret;        
    }

    st = avformat_new_stream(s, NULL);
    if (!st) {
        av_log(NULL, AV_LOG_ERROR, "nxt: avformat_new_stream failed");
        return AVERROR(ENOMEM);
    }

    // TODO
    // st->duration = ???;
    st->start_time = nxt->pts;

    av_log(NULL, AV_LOG_VERBOSE, "nxt: start_time %" PRId64 "\n", st->start_time);

    switch (nxt->format)
    {
    case DNXHD_120_1080i50:
        st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id = AV_CODEC_ID_DNXHD;
        st->codecpar->format = AV_PIX_FMT_YUV422P;
        st->codecpar->field_order = AV_FIELD_TB;
        st->codecpar->sample_aspect_ratio.num = 1;
        st->codecpar->sample_aspect_ratio.den = 1;
        st->codecpar->bit_rate = 120000000;
        st->codecpar->width = 1920;
        st->codecpar->height = 1080;

        st->time_base.num = 1;
        st->time_base.den = 2500;

        st->avg_frame_rate.num = 25;
        st->avg_frame_rate.den = 1;

        return 0;
    case PCM_S32LE_48000c8:
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S32LE;
        st->codecpar->codec_tag = 0;
        st->codecpar->format = AV_SAMPLE_FMT_S32;
        st->codecpar->block_align = 32;
        st->codecpar->channels = 8;
        st->codecpar->sample_rate = 48000;
        st->codecpar->bits_per_coded_sample = 32;

        st->time_base.num = 1;
        st->time_base.den = 48000;

        return 0;
    case DNXHD_115_720p50:
        st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        st->codecpar->codec_id = AV_CODEC_ID_DNXHD;
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

        return 0;
      default:
        av_log(NULL, AV_LOG_ERROR, "nxt: invalid format %d\n", nxt->format);
        ret = -1;
        goto fail;
    }
fail:
    av_free(st);
    return ret;
}

static int nxt_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, size;
    NXTHeader *nxt = (NXTHeader*)s->priv_data;
    AVIOContext *bc = s->pb;
    size = nxt->size;

    if (avio_feof(bc)) {
        av_log(NULL, AV_LOG_VERBOSE , "nxt: eof");
        return AVERROR_EOF;
    }

    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG) {
        av_log(NULL, AV_LOG_ERROR, "nxt: invalid tag %" PRId64 "\n", nxt->tag);
        return -1;
    }

    ret = av_new_packet(pkt, nxt->next);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "nxt: av_new_packet failed %d\n", ret);
        goto fail;
    }

    ret = avio_read(bc, pkt->data, pkt->size);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "nxt: avio_read failed %d\n", ret);
        goto fail;
    }

    pkt->pos = avio_tell(bc);
    pkt->stream_index = 0;
    pkt->flags |= (nxt->flags & NXT_FLAG_KEY) != 0 ? AV_PKT_FLAG_KEY : 0;
    pkt->duration = nxt->duration;
    pkt->pts = nxt->pts;
    pkt->dts = pkt->pts;

    if (ret == pkt->size) {
        memcpy(nxt, pkt->data + nxt->next - NXT_ALIGN, NXT_ALIGN);
    } else if (ret >= size) {
        memset(nxt, 0, NXT_ALIGN);
    } else {
        av_log(NULL, AV_LOG_WARNING, "nxt: avio_read returned unexpected size %d\n", ret);
        ret = -1;
        goto fail;
    }

    av_shrink_packet(pkt, size);

    return 0;
fail:
    av_packet_unref(pkt);
    return ret;
}

static int nxt_read_seek_binary(AVFormatContext *s, int stream_index, int pts, int flags)
{
    int ret, step, offset, pos, size = 0;
    NXTHeader *nxt = (NXTHeader*)s->priv_data;
    NXTHeader nxt2;
    AVIOContext *bc = s->pb;

    ret = avio_tell(bc);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "nxt: avio_tell failed %d\n", ret);
        return ret;
    }
    pos = ret - NXT_ALIGN;

    ret = avio_size(bc);
    if (ret < 0) {
        av_log(NULL, AV_LOG_VERBOSE, "nxt: avio_size failed %d\n", ret);
    } else {
        size = ret;
    }

    step = size - NXT_MAX_FRAME_SIZE - NXT_ALIGN;
    offset = nxt->position - pos;

    if (nxt->pts > pts) {
        // TODO
        av_log(NULL, AV_LOG_VERBOSE, "nxt: seek backwards is not implemented");
        return -1;
    }

    while (step > NXT_ALIGN) {
        ret = avio_seek(bc, (nxt->position - offset) + nxt_floor(step), SEEK_SET);
        if (ret < 0) {
            step /= 2;
            continue;
        }

        ret = nxt_seek_fwd(s, &nxt2);
        if (ret < 0) {
            step /= 2;
            continue;
        }
        
        if (nxt2.pts > pts) {
            step /= 2;
        } else if (nxt2.index == nxt->index) {
            return 0;
        } else {
            memcpy(nxt, &nxt2, NXT_ALIGN);
            step = FFMIN(step, (size - (nxt->position - offset)) / 2);
        }
    }

    return -1;
}

static int nxt_read_seek(AVFormatContext *s, int stream_index, int64_t pts, int flags)
{
    int ret;
    NXTHeader *nxt = (NXTHeader*)s->priv_data;
    AVIOContext *bc = s->pb;

    av_log(NULL, AV_LOG_VERBOSE, "nxt: read_seek %" PRId64 "\n", pts);

    if (bc->seekable & AVIO_SEEKABLE_NORMAL) {
        ret = nxt_read_seek_binary(s, stream_index, pts, flags);
        if (ret < 0) {
            av_log(NULL, AV_LOG_VERBOSE, "nxt: nxt_read_seek_binary failed %d\n", ret);
        }
    }

    while (nxt->pts < pts) {
        ret = nxt_seek_fwd(s, nxt);
        if (ret < 0) {
            return -1;
        }
    }

    return 0;
}

AVInputFormat ff_nxt_demuxer = {
    .name           = "nxt",
    .long_name      = NULL_IF_CONFIG_SMALL("NXT"),
    .read_probe     = nxt_probe,
    .read_header    = nxt_read_header,
    .read_packet    = nxt_read_packet,
    .read_seek      = nxt_read_seek,
    .extensions     = "nxt",
    .flags          = AVFMT_SEEK_TO_PTS,
    .priv_data_size = sizeof(NXTHeader),
};

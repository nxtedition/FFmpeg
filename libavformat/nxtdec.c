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

static int64_t nxt_floor(int64_t val)
{
    return (val / NXT_ALIGN) * NXT_ALIGN;
}

static int64_t nxt_seek_fwd(AVFormatContext *s, NXTHeader* nxt)
{
    int i;
    int64_t ret;
    AVIOContext *bc = s->pb;

    for (i = 0; i < NXT_MAX_FRAME_SIZE; i += NXT_ALIGN) {
        ret = avio_read(bc, (char*)nxt, NXT_ALIGN);

        if (ret < 0)
            return ret;

        if (ret < sizeof(NXTHeader))
            return -1;

        if ((nxt->tag & NXT_TAG_MASK) == NXT_TAG)
            return 0;
    }

    return -1;
}

static int nxt_read_duration(AVFormatContext *s)
{
    int64_t ret, step, pos, size, offset;

    ret = avio_size(bc);
    if (ret < 0) {
        av_log(NULL, AV_LOG_VERBOSE, "nxt: avio_size failed %" PRId64 "\n", ret);
        return ret;
    }

    size = ret;

    step = size - NXT_MAX_FRAME_SIZE - NXT_ALIGN;
    
    ret = avio_tell(bc);
    if (ret < 0) {
        av_log(NULL, AV_LOG_VERBOSE, "nxt: avio_tell failed %" PRId64 "\n", ret);
        return ret;
    }
    pos = ret - NXT_ALIGN;

    memcpy(&nxt1, nxt, sizeof(NXTHeader));

    offset = nxt1.position - pos;

    while (step >= NXT_ALIGN) {
        ret = avio_seek(bc, (nxt1.position - offset) + nxt_floor(step), SEEK_SET);
        if (ret < 0) {
            av_log(NULL, AV_LOG_VERBOSE, "nxt: avio_seek failed %" PRId64 "\n", ret);
            return ret;
        }

        ret = nxt_seek_fwd(s, &nxt2);
        if (ret < 0) {
            step /= 2;
            continue;
        } 
        
        if (nxt2.index == nxt1.index) {
            break;
        } else {
            memcpy(&nxt1, &nxt2, sizeof(NXTHeader));
            step = FFMIN(step, (size - (nxt1.position - offset)) / 2);
        }
    }

    ret = avio_seek(bc, pos + NXT_ALIGN, SEEK_SET);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "nxt: avio_seek failed %" PRId64 "\n", ret);
        return ret;
    }

    return nxt1.pts - nxt->pts;
}

static int nxt_read_header(AVFormatContext *s)
{
    av_log(NULL, AV_LOG_VERBOSE, "nxt: read_header \n");

    int64_t ret;
    NXTHeader *nxt = (NXTHeader*)s->priv_data;
    NXTHeader nxt1, nxt2;
    AVStream *st = NULL;
    AVIOContext *bc = s->pb;

    ret = avio_read(bc, (char*)nxt, NXT_ALIGN);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "nxt: avio_read failed %" PRId64 "\n", ret);
        return ret;        
    }

    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG) {
        av_log(NULL, AV_LOG_ERROR, "nxt: invalid tag %" PRId64 "\n", nxt->tag);
        return -1;        
    }

    st = avformat_new_stream(s, NULL);
    if (!st) {
        av_log(NULL, AV_LOG_ERROR, "nxt: avformat_new_stream failed");
        return AVERROR(ENOMEM);
    }

    ret = nxt_read_duration(s);
    if (ret < 0) {
        av_log(NULL, AV_LOG_WARN, "nxt: nxt_read_duration failed %" PRId64 "\n", ret);
    } else {
        st->duration = ret;
    }

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
    int64_t ret, size;
    NXTHeader *nxt = (NXTHeader*)s->priv_data;
    AVIOContext *bc = s->pb;

    if (nxt->next < nxt->size) {
        av_log(NULL, AV_LOG_WARN , "nxt: next < size");
    }

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
        av_log(NULL, AV_LOG_ERROR, "nxt: av_new_packet failed %" PRId64 "\n", ret);
        goto fail;
    }

    ret = avio_read(bc, pkt->data, pkt->size);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "nxt: avio_read failed %" PRId64 "\n", ret);
        goto fail;
    }

    pkt->pos = avio_tell(bc);
    pkt->stream_index = 0;
    pkt->flags |= (nxt->flags & NXT_FLAG_KEY) != 0 ? AV_PKT_FLAG_KEY : 0;
    pkt->duration = nxt->duration;
    pkt->pts = nxt->pts;

    size = nxt->size;

    if (ret == pkt->size) {
        memcpy(nxt, pkt->data + nxt->next - NXT_ALIGN, NXT_ALIGN);
    } else if (ret == nxt->size) {
        memset(nxt, 0, NXT_ALIGN);
    } else {
        av_log(NULL, AV_LOG_ERROR, "nxt: avio_read returned unexpected size %" PRId64 "\n", ret);
        ret = -1;
        goto fail;
    }

    av_shrink_packet(pkt, size);

    return pkt->size;
fail:
    av_packet_unref(pkt);
    return ret;
}

static int nxt_read_seek(AVFormatContext *s, int stream_index, int64_t pts, int flags)
{
    av_log(NULL, AV_LOG_VERBOSE, "nxt: read_seek %" PRId64 "\n", pts);

    int64_t ret, step, offset, pos, pos2, size = INT64_MAX;
    NXTHeader *nxt = (NXTHeader*)s->priv_data;
    NXTHeader nxt2;
    AVIOContext *bc = s->pb;

    ret = avio_tell(bc);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "nxt: avio_tell failed %" PRId64 "\n", ret);
    }
    pos = ret - NXT_ALIGN;

    ret = avio_size(bc);
    if (ret < 0) {
        av_log(NULL, AV_LOG_VERBOSE, "nxt: avio_size failed %" PRId64 "\n", ret);
    } else {
        size = ret;        
    }

    step = size - NXT_MAX_FRAME_SIZE - NXT_ALIGN;
    offset = nxt->position - pos;

    // TODO seek backwards

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
            memcpy(nxt, &nxt2, sizeof(NXTHeader));
            step = FFMIN(step, (size - (nxt->position - offset)) / 2);
        }
    }

    while (true) {
        ret = nxt_seek_fwd(s, &nxt2);
        if (ret < 0) {
            break;
        }

        // TODO: Should it read past?
        if (nxt2.pts < pts) {
            continue;
        } else if (nxt2.index == nxt->index) {
            return 0;
        } else {
            memcpy(nxt, &nxt2, sizeof(NXTHeader));
        }
    }

    return -1;
}

AVInputFormat ff_nxt_demuxer = {
    .name           = "nxt",
    .long_name      = NULL_IF_CONFIG_SMALL("NXT"),
    .read_probe     = nxt_probe,
    .read_header    = nxt_read_header,
    .read_packet    = nxt_read_packet,
    .read_seek      = nxt_read_seek,
    .extensions     = "nxt",
    .flags          = AVFMT_GENERIC_INDEX,
    .priv_data_size = sizeof(NXTHeader),
};

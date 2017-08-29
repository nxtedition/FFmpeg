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

static int nxt_read_header(AVFormatContext *s)
{
    int64_t ret, step, pos, size, offset;
    NXTHeader *nxt = (NXTHeader*)s->priv_data;
    NXTHeader nxt1, nxt2;
    AVStream *st = NULL;
    AVIOContext *bc = s->pb;

    ret = avio_read(bc, (char*)nxt, NXT_ALIGN);

    if (ret < 0)
        return ret;

    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG)
        return -1;

    st = avformat_new_stream(s, NULL);

    if (!st) {
      return AVERROR(ENOMEM);
    }

    size = avio_size(bc);

    if (size > 0) {
        step = size - NXT_MAX_FRAME_SIZE - NXT_ALIGN;
        pos = avio_tell(bc) - NXT_ALIGN;

        memcpy(&nxt1, nxt, sizeof(NXTHeader));

        offset = nxt1.position - pos;

        while (step >= NXT_ALIGN) {
            ret = avio_seek(bc, (nxt1.position - offset) + nxt_floor(step), SEEK_SET);

            if (ret < 0)
                return ret;

            ret = nxt_seek_fwd(s, &nxt2);

            if (ret < 0) {
                step /= 2;
            } else if (nxt2.index == nxt1.index) {
                return 0;
            } else {
                memcpy(&nxt1, &nxt2, sizeof(NXTHeader));
                step = FFMIN(step, size - (nxt1.position - offset) - NXT_ALIGN);
            }
        }

        st->duration = nxt1.pts - nxt->pts;

        ret = avio_seek(bc, pos + NXT_ALIGN, SEEK_SET);

        if (ret < 0)
            return ret;
    }

    st->start_time = nxt->pts;

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

        st->avg_frame_rate.num = 25;
        st->avg_frame_rate.den = 1;

        st->time_base.num = 1;
        st->time_base.den = 25;

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
        st->time_base.den = 50;

        st->avg_frame_rate.num = 50;
        st->avg_frame_rate.den = 1;

        return 0;
      default:
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
    size = nxt->size;

    if (avio_feof(bc)) {
      return AVERROR_EOF;
    }

    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG) {
        return -1;
    }

    ret = av_new_packet(pkt, nxt->next);

    if (ret < 0)
        goto fail;

    ret = avio_read(bc, pkt->data, pkt->size);

    if (ret < 0)
        goto fail;

    if (ret < size) {
        ret = -1;
        goto fail;
    }

    pkt->stream_index = 0;
    pkt->flags |= (nxt->flags & NXT_FLAG_KEY) != 0 ? AV_PKT_FLAG_KEY : 0;
    pkt->duration = nxt->duration;
    pkt->pts = nxt->pts;

    if (ret == pkt->size) {
        memcpy(nxt, pkt->data + nxt->next - NXT_ALIGN, NXT_ALIGN);
    } else if (ret == size) {
        memset(nxt, 0, NXT_ALIGN);
    } else {
        ret = -1;
        goto fail;
    }

    av_shrink_packet(pkt, size);

    return pkt->size;
fail:
    av_packet_unref(pkt);
    return ret;
}

static int64_t nxt_read_seek(AVFormatContext *s, int stream_index, int64_t timestamp, int flags)
{
    int64_t ret;
    int64_t step, offset, pos, size;
    NXTHeader *nxt = (NXTHeader*)s->priv_data;
    NXTHeader nxt2;
    AVIOContext *bc = s->pb;

    pos = avio_tell(bc) - NXT_ALIGN;
    size = avio_size(bc);
    step = size - NXT_MAX_FRAME_SIZE - NXT_ALIGN;
    offset = nxt->position - pos;

    while (step >= NXT_ALIGN) {
        ret = avio_seek(bc, (nxt->position - offset) + nxt_floor(step), SEEK_SET);

        if (ret < 0)
            return ret;

        ret = nxt_seek_fwd(s, &nxt2);

        if (ret < 0 || nxt2.pts > timestamp) {
            step /= 2;
        } else if (nxt2.index == nxt->index) {
            return 0;
        } else {
            memcpy(nxt, &nxt2, sizeof(NXTHeader));
            step = FFMIN(step, size - (nxt1.position - offset) - NXT_ALIGN);
        }
    }

    ret = avio_seek(bc, pos + NXT_ALIGN, SEEK_SET);

    if (ret < 0)
        return ret;

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

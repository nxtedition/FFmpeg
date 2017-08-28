#include "avformat.h"
#include "internal.h"
#include "avio_internal.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/avassert.h"

#define NXT_TAG           0xf07563b4c0000000LL
#define NXT_TAG_MASK      0xfffffffff0000000LL
#define NXT_FLAG_KEY      1

#define DNXHD_120_1080i50 1
#define PCM_S32LE_48000c8 2
#define DNXHD_115_720p50  3
#define YUV422P_1080i50   4
#define YUV422P_720p50    5

typedef struct NXTContext {
  int64_t     tag;
  int32_t     crc;
  int64_t     index;
  int64_t     position;
  int32_t     next;
  int32_t     prev;

  int32_t     size;
  int32_t     flags;
  int32_t     format;
  int64_t     pts;
  int64_t     dts;
  int64_t     ltc;
  int32_t     duration;
} NXTContext;

static int64_t nxt_floor(int64_t val)
{
    return (val / 4096) * 4096;
}

static int64_t nxt_seek_fwd(AVFormatContext *s, NXTContext* nxt, int64_t pos)
{
    int64_t ret, end;
    AVIOContext *bc = s->pb;

    end = pos + 4096 * 256;

    for (; pos < end; pos += 4096) {
        ret = avio_seek(bc, pos, SEEK_SET);

        if (ret < 0)
            return ret;

        ret = avio_read(bc, (char*)nxt, sizeof(NXTContext));

        if (ret < 0)
            return ret;

        if (ret < sizeof(NXTContext))
            return -1;

        if ((nxt->tag & NXT_TAG_MASK) == NXT_TAG)
          return 0;
    }

    return -1;
}

static int nxt_probe(AVProbeData *p)
{
    int i;
    NXTContext *nxt;

    for (i = 0; i < p->buf_size - sizeof(nxt->tag); i++) {
      nxt = (NXTContext*)(p->buf + i);
      if ((nxt->tag & NXT_TAG_MASK) == NXT_TAG)
          return AVPROBE_SCORE_MAX;
    }

    return 0;
}

static int nxt_read_header(AVFormatContext *s)
{
    int64_t ret, step;
    NXTContext *nxt = (NXTContext*)s->priv_data;
    NXTContext nxt1, nxt2;
    AVStream *st = NULL;
    AVIOContext *bc = s->pb;

    ret = avio_read(bc, (char*)nxt, 4096);

    if (ret < 0)
        return ret;

    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG)
        return -1;

    st = avformat_new_stream(s, NULL);

    if (!st) {
      return AVERROR(ENOMEM);
    }

    step = avio_size(bc);

    if (step > 0) {
        step -= 4096 * 256;

        memcpy(&nxt1, nxt, sizeof(NXTContext));

        while (step >= 4096) {
            ret = nxt_seek_fwd(s, &nxt2, nxt1.position + nxt_floor(step));
            if (ret < 0 || nxt2.index < nxt1.index) {
                step /= 2;
            } else if (nxt2.index == nxt1.index) {
                break;
            } else {
                memcpy(&nxt1, &nxt2, sizeof(NXTContext));
            }
        }

        st->duration = nxt1.pts - nxt->pts;

        avio_seek(bc, nxt->position + 4096, SEEK_SET);
    }

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
        st->start_time = nxt->pts;

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
        st->start_time = nxt->pts;
        // st->duration =

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

        st->avg_frame_rate.num = 50;
        st->avg_frame_rate.den = 1;

        st->time_base.num = 1;
        st->time_base.den = 50;
        st->start_time = nxt->pts;
        // st->duration =

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
    NXTContext *nxt = (NXTContext*)s->priv_data;
    AVIOContext *bc = s->pb;
    size = nxt->size;

    if (avio_feof(bc)) {
      return AVERROR_EOF;
    }

    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG) {
        return -1;
    }

    ret = avio_seek(bc, nxt->position + 4096, SEEK_SET);

    if (ret < 0)
        return ret;

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
        memcpy(nxt, pkt->data + nxt->next - 4096, sizeof(NXTContext));
    } else if (ret == size && avio_feof(bc)) {
        memset(nxt, 0, sizeof(NXTContext));
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
    int64_t step;
    NXTContext *nxt = (NXTContext*)s->priv_data;
    NXTContext nxt1, nxt2;
    AVIOContext *bc = s->pb;

    step = avio_size(bc);

    if (step < 0) {
      return step;
    }

    step /= 2;

    nxt1.position = 0;
    nxt1.index = -1;

    while (step >= 4096) {
        ret = nxt_seek_fwd(s, &nxt2, nxt1.position + nxt_floor(step));

        if (ret < 0 || nxt2.index < nxt1.index || nxt2.pts > timestamp) {
            step /= 2;
        } else if (nxt2.index == nxt1.index) {
            break;
        } else {
            memcpy(&nxt1, &nxt2, sizeof(NXTContext));
        }
    }

    if (nxt1.index == -1)
        return -1;

    ret = avio_seek(bc, nxt1.position + 4096, SEEK_SET);

    if (ret < 0)
        return ret;

    memcpy(nxt, &nxt1, sizeof(NXTContext));

    return ret;
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
    .priv_data_size = 4096,
};

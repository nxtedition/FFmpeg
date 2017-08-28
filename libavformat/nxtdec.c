#include "avformat.h"
#include "internal.h"
#include "avio_internal.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/avassert.h"

#define NXT_TAG         0xf07563b4c0000000LL
#define NXT_TAG_MASK    0xfffffffff0000000LL
#define NXT_FLAG_KEY    1

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
    int ret, i;
    NXTContext *nxt = (NXTContext*)s->priv_data;
    AVStream *st = NULL;
    AVIOContext *bc = s->pb;

    ret = avio_read(bc, (char*)nxt, 4096);

    if (ret < 0)
      goto fail;

    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG) {
        ret = -1
        goto fail;
    }

    st = avformat_new_stream(s, NULL);

    if (!st) {
      ret = AVERROR(ENOMEM);
      goto fail;
    }

    ret = -1;

    switch (nxt->format)
    {
    case 1:
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
    case 2:
        st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        st->codecpar->codec_id = AV_CODEC_ID_PCM_S32LE;
        st->codecpar->codec_tag = 0;
        st->codecpar->format = AV_SAMPLE_FMT_S32;
        st->codecpar->block_align = 32;
        st->codecpar->channels = 8;
        st->codecpar->bits_per_coded_sample = 32;

        st->time_base.num = 1;
        st->time_base.den = 48000;
        st->start_time = nxt->pts;

        return 0;
    }
fail:
    av_free(st);
    return ret;
}

static int nxt_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, size;
    NXTContext *nxt = (NXTContext*)s->priv_data;
    AVIOContext *bc = s->pb;
    size = nxt->size;

    if (avio_feof(bc)) {
      return AVERROR_EOF;
    }

    if ((nxt->tag & NXT_TAG_MASK) != NXT_TAG) {
        ret = -1;
        goto fail;
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
        memcpy(nxt, pkt->data + nxt->next - 4096, sizeof(NXTContext));
    } else if (ret == size) {
        memset(nxt, 0, sizeof(NXTContext))
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

static int64_t nxt_read_timestamp(AVFormatContext *s, int stream_index, int64_t *ppos, int64_t pos_limit)
{
    int64_t pos;
    NXTContext nxt;
    AVIOContext *bc = s->pb;

    for (pos = ((*ppos + 4095) / 4096) * 4096; pos < pos_limit || avio_read(bc, (char*)&nxt, 4096) == 4096; pos += 4096) {
        if ((nxt.tag & NXT_TAG_MASK) == NXT_TAG) {
          *ppos = nxt.position;
          return nxt.pts;
        }
    }

    return AV_NOPTS_VALUE;
}

AVInputFormat ff_nxt_demuxer = {
    .name           = "nxt",
    .long_name      = NULL_IF_CONFIG_SMALL("NXT"),
    .read_probe     = nxt_probe,
    .read_header    = nxt_read_header,
    .read_packet    = nxt_read_packet,
    .read_timestamp = nxt_read_timestamp,
    .extensions     = "nxt",
    .flags          = AVFMT_GENERIC_INDEX,
    .priv_data_size = 4096,
};

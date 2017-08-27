#include "avformat.h"
#include "internal.h"
#include "avio_internal.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/avassert.h"

#define NXT_TAG         0xf07563b4c0000000LL
#define NXT_TAG_MASK    0xfffffffff0000000LL
#define NXT_FLAG_KEY    1

typedef struct NXTHeader {
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
} NXTHeader;

static int __attribute__((optimize("O0"))) nxt_probe(AVProbeData *p)
{
    int i;
    NXTHeader *header;

    for (i = 0; i < p->buf_size - sizeof(header->tag); i++) {
      header = (NXTHeader*)(p->buf + i);
      if ((header->tag & NXT_TAG_MASK) == NXT_TAG)
          return AVPROBE_SCORE_MAX;
    }

    return 0;
}

static int nxt_read_header(AVFormatContext *s)
{
    int ret;
    NXTHeader *header = (NXTHeader*)s->priv_data;
    AVStream *st = NULL;
    AVIOContext *bc = s->pb;

    ret = avio_read(bc, (char*)header, 4096);

    if (ret < 0)
      goto fail;

    st = avformat_new_stream(s, NULL);

    if (!st) {
      ret = AVERROR(ENOMEM);
      goto fail;
    }

    ret = -1;

    st->start_time = header->pts;
    // TODO st->duration

    switch (header->format) {
      case 1: {
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
      }
      case 2: {
        // TODO
        // st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
        // st->codecpar->codec_id = AV_CODEC_ID_PCM_S32LE;
        // st->codecpar->codec_tag = 0;
        // st->codecpar->format = AV_SAMPLE_FMT_S32;
        // // st->codecpar->block_align = 24;
        // st->codecpar->channel_layout = 1599;
        // st->codecpar->channels = 8;
        // st->codecpar->bits_per_coded_sample = 24;
        //
        // st->time_base.num = 1;
        // st->time_base.den = 48000;

        return 0;
      }
    }
fail:
    av_free(st);
    return ret;
}

static int nxt_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    int ret, size;
    NXTHeader *header = (NXTHeader*)s->priv_data;
    AVIOContext *bc = s->pb;

    if ((header->tag & NXT_TAG_MASK) != NXT_TAG){
        ret = -1;
        goto fail;
    }

    ret = av_new_packet(pkt, header->next);

    if (ret < 0)
        goto fail;

    av_assert0(avio_tell(bc) == header->position + 4096);

    ret = avio_read(bc, pkt->data, pkt->size);

    if (ret < 0)
        goto fail;

    if (ret < size) {
        ret = -1;
        goto fail;
    }

    pkt->stream_index = 0;
    pkt->flags |= (header->flags & NXT_FLAG_KEY) != 0 ? AV_PKT_FLAG_KEY : 0;
    pkt->duration = header->duration;
    pkt->pts = header->pts;

    size = header->size;
    if (ret == pkt->size) {
      memcpy(header, pkt->data + header->next - 4096, sizeof(NXTHeader));
    } else {
      memset(header, 0, sizeof(NXTHeader));
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
    NXTHeader header;
    AVIOContext *bc = s->pb;

    for (pos = (*ppos + 4095) / 4096; pos < pos_limit || avio_read(bc, (char*)&header, 4096) == 4096; pos += 4096) {
        if ((header.tag & NXT_TAG_MASK) == NXT_TAG) {
          *ppos = header.position;
          return header.pts;
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

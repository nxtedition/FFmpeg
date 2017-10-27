#ifndef AVFORMAT_NXT_H
#define AVFORMAT_NXT_H

#define NXT_TAG             0xf07563b4c0000000LL
#define NXT_TAG_MASK        0xfffffffff0000000LL
#define NXT_FLAG_KEY        1
#define NXT_MAX_FRAME_SIZE  (4096 * 256)
#define NXT_ALIGN           4096

#define DNXHD_120_1080i50   1
#define PCM_S32LE_48000c8   2
#define DNXHD_115_720p50    3
#define YUV422P_1080i50     4
#define YUV422P_720p50      5

typedef struct NXTHeader {
  int64_t tag;
  int32_t crc;
  int64_t index;
  int64_t position;
  int32_t next;
  int32_t prev;
  int32_t size;
  int32_t flags;
  int32_t format;
  int64_t pts;
  int64_t dts;
  int64_t ltc;
  int32_t duration;
  char    pad[4005];
} NXTHeader;

_Static_assert(sizeof(NXTHeader) == NXT_ALIGN, "sizeof NXTHeader != NXT_ALIGN");

#endif

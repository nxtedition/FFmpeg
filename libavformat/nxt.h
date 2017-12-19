#ifndef AVFORMAT_NXT_H
#define AVFORMAT_NXT_H

#define NXT_TAG             0xf07563b4c0000000LL
#define NXT_TAG_MASK        0xfffffffff0000000LL
#define NXT_FLAG_KEY        1
#define NXT_MAX_FRAME_SIZE  (4096 * 256)
#define NXT_ALIGN           4096

#define NXT_DNXHD_120_1080i50   1
#define NXT_PCM_S32LE_48000c8   2
#define NXT_DNXHD_115_720p50    3

#define NXT_LTC_50 1
#define NXT_LTC_25 2

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
  int32_t ltc;
  int32_t ltc_format;
  int32_t duration;
} NXTHeader;

#endif

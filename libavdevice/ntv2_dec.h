#ifndef AVDEVICE_NTV2_DEC_H
#define AVDEVICE_NTV2_DEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "libavutil/thread.h"

typedef enum NTV2PtsSource {
    PTS_SRC_WALLCLOCK = 1,
    PTS_SRC_ABS_WALLCLOCK = 2,
    PTS_SRC_NB
} NTV2PtsSource;

typedef struct AVPacketQueue {
    struct AVPacketList *first_pkt, *last_pkt;
    int size;
    int eof;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
} AVPacketQueue;

typedef struct NTV2Context {
    const struct AVClass *cclass;

    int device_index;
    int input_source;
    int video_format;
    int audio_source;
    int queue_size;

    int quit;

    void *device;
    void *thread;

    int dropped;
    int draw_bars;

    struct AVStream *video_st;
    struct AVStream *audio_st;

    AVPacketQueue queue;
} NTV2Context;

int ff_ntv2_read_header(struct AVFormatContext *avctx);
int ff_ntv2_read_packet(struct AVFormatContext *avctx, struct AVPacket *pkt);
int ff_ntv2_read_close(struct AVFormatContext *avctx);

#ifdef __cplusplus
}
#endif

#endif /* AVDEVICE_NTV2_DEC_H */

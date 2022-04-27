#ifndef AVDEVICE_NTV2_DEC_H
#define AVDEVICE_NTV2_DEC_H

#ifdef __cplusplus
#include <atomic>
using namespace std;
#else
#include <stdatomic.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "libavutil/thread.h"
#include "libavutil/threadmessage.h"

typedef enum NTV2PtsSource {
    PTS_SRC_WALLCLOCK = 1,
    PTS_SRC_ABS_WALLCLOCK = 2,
    PTS_SRC_NB
} NTV2PtsSource;

typedef struct NTV2Context {
    const struct AVClass *cclass;

    int device_index;
    int input_source;
    int raw_format;
    int video_format;
    int audio_source;
    int queue_size;

    atomic_int exit;

    void *device;
    void *thread;

    int dropped;
    int draw_bars;

    struct AVStream *video_st;
    struct AVStream *audio_st;

    AVThreadMessageQueue *queue;
} NTV2Context;

int ff_ntv2_read_header(struct AVFormatContext *avctx);
int ff_ntv2_read_packet(struct AVFormatContext *avctx, struct AVPacket *pkt);
int ff_ntv2_read_close(struct AVFormatContext *avctx);

#ifdef __cplusplus
}
#endif

#endif /* AVDEVICE_NTV2_DEC_H */

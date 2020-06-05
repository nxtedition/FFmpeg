#include "libavformat/avformat.h"
#include "libavutil/opt.h"

#include "ntv2_dec.h"

#define OFFSET(x) offsetof(struct NTV2Context, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM

static const AVOption options[] = {
    { "input_source",   "video input",              OFFSET(input_source),       AV_OPT_TYPE_INT,        { .i64 = 5 },                       0, 12,             DEC, "input_source" },
    { "sdi1",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 5 },                       0, 0,              DEC, "input_source" },
    { "sdi2",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 6 },                       0, 0,              DEC, "input_source" },
    { "sdi3",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 7 },                       0, 0,              DEC, "input_source" },
    { "sdi4",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 8 },                       0, 0,              DEC, "input_source" },
    { "sdi5",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 9 },                       0, 0,              DEC, "input_source" },
    { "sdi6",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 10 },                      0, 0,              DEC, "input_source" },
    { "sdi7",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 11 },                      0, 0,              DEC, "input_source" },
    { "sdi8",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 12 },                      0, 0,              DEC, "input_source" },
    { "video_format",   "video format",             OFFSET(video_format),       AV_OPT_TYPE_INT,        { .i64 = 1 },                       0, 414,            DEC, "video_format" },
    { "1080i5000",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 1 },                       0, 0,              DEC, "video_format" },
    { "1080p2500",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 10 },                      0, 0,              DEC, "video_format" },
    { "720p5000",       NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 17 },                      0, 0,              DEC, "video_format" },
    { "1080p5000",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 23 },                      0, 0,              DEC, "video_format" },
    { "audio_source",   "audio input",              OFFSET(audio_source),       AV_OPT_TYPE_INT,        { .i64 = 0 },                       0, 1,              DEC, "audio_source" },
    { "embedded",       NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 0 },                       0, 0,              DEC, "audio_source" },
    { "aes",            NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 1 },                       0, 0,              DEC, "audio_source" },
    { "draw_bars",      "draw bars on signal loss", OFFSET(draw_bars),          AV_OPT_TYPE_BOOL,       { .i64 = 1 },                       0, 1,              DEC },
    { "queue_size",     "input queue buffer size",  OFFSET(queue_size),         AV_OPT_TYPE_INT,        { .i64 = 32 },                      0, INT_MAX,        DEC },
    { NULL },
};

static const AVClass ntv2_demuxer_class = {
    .class_name = "AJA NTV2 demuxer",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
    .category   = AV_CLASS_CATEGORY_DEVICE_VIDEO_INPUT,
};

AVInputFormat ff_ntv2_demuxer = {
    .name           = "ntv2",
    .long_name      = "AJA NTV2 input",
    .flags          = AVFMT_NOFILE,
    .priv_class     = &ntv2_demuxer_class,
    .priv_data_size = sizeof(struct NTV2Context),
    .read_header    = ff_ntv2_read_header,
    .read_packet    = ff_ntv2_read_packet,
    .read_close     = ff_ntv2_read_close,
};

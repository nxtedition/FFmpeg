#include "libavformat/avformat.h"
#include "libavutil/opt.h"

#include "ntv2_dec.h"

#define OFFSET(x) offsetof(struct NTV2Context, x)
#define DEC AV_OPT_FLAG_DECODING_PARAM

typedef enum
{
    NTV2_FBF_FIRST                  = 0
    ,NTV2_FBF_10BIT_YCBCR           = NTV2_FBF_FIRST
    ,NTV2_FBF_8BIT_YCBCR
    ,NTV2_FBF_ARGB
    ,NTV2_FBF_RGBA
    ,NTV2_FBF_10BIT_RGB
    ,NTV2_FBF_8BIT_YCBCR_YUY2
    ,NTV2_FBF_ABGR
    ,NTV2_FBF_LAST_SD_FBF = NTV2_FBF_ABGR
    ,NTV2_FBF_10BIT_DPX
    ,NTV2_FBF_10BIT_YCBCR_DPX
    ,NTV2_FBF_8BIT_DVCPRO
    ,NTV2_FBF_8BIT_YCBCR_420PL3
    ,NTV2_FBF_8BIT_HDV
    ,NTV2_FBF_24BIT_RGB
    ,NTV2_FBF_24BIT_BGR
    ,NTV2_FBF_10BIT_YCBCRA
    ,NTV2_FBF_10BIT_DPX_LE
    ,NTV2_FBF_48BIT_RGB
    ,NTV2_FBF_12BIT_RGB_PACKED
    ,NTV2_FBF_PRORES_DVCPRO
    ,NTV2_FBF_PRORES_HDV
    ,NTV2_FBF_10BIT_RGB_PACKED
    ,NTV2_FBF_10BIT_ARGB
    ,NTV2_FBF_16BIT_ARGB
    ,NTV2_FBF_8BIT_YCBCR_422PL3
    ,NTV2_FBF_10BIT_RAW_RGB
    ,NTV2_FBF_10BIT_RAW_YCBCR
    ,NTV2_FBF_10BIT_YCBCR_420PL3_LE
    ,NTV2_FBF_10BIT_YCBCR_422PL3_LE
    ,NTV2_FBF_10BIT_YCBCR_420PL2
    ,NTV2_FBF_10BIT_YCBCR_422PL2
    ,NTV2_FBF_8BIT_YCBCR_420PL2
    ,NTV2_FBF_8BIT_YCBCR_422PL2
    ,NTV2_FBF_LAST
    ,NTV2_FBF_NUMFRAMEBUFFERFORMATS = NTV2_FBF_LAST
    ,NTV2_FBF_INVALID               = NTV2_FBF_NUMFRAMEBUFFERFORMATS
} NTV2FrameBufferFormat;

typedef enum _NTV2VideoFormat
{
     NTV2_FORMAT_UNKNOWN

    ,NTV2_FORMAT_FIRST_HIGH_DEF_FORMAT			= 1
    ,NTV2_FORMAT_FIRST_STANDARD_DEF_FORMAT		= 32
    ,NTV2_FORMAT_FIRST_2K_DEF_FORMAT			= 64
    ,NTV2_FORMAT_FIRST_4K_DEF_FORMAT			= 80
    ,NTV2_FORMAT_FIRST_HIGH_DEF_FORMAT2			= 110
    ,NTV2_FORMAT_FIRST_UHD_TSI_DEF_FORMAT       = 200
    ,NTV2_FORMAT_FIRST_4K_TSI_DEF_FORMAT        = 250
    ,NTV2_FORMAT_FIRST_4K_DEF_FORMAT2			= 300
    ,NTV2_FORMAT_FIRST_UHD2_DEF_FORMAT			= 350
    ,NTV2_FORMAT_FIRST_UHD2_FULL_DEF_FORMAT		= 400

    ,NTV2_FORMAT_1080i_5000					= NTV2_FORMAT_FIRST_HIGH_DEF_FORMAT
    ,NTV2_FORMAT_1080i_5994
    ,NTV2_FORMAT_1080i_6000
    ,NTV2_FORMAT_720p_5994
    ,NTV2_FORMAT_720p_6000
    ,NTV2_FORMAT_1080psf_2398
    ,NTV2_FORMAT_1080psf_2400
    ,NTV2_FORMAT_1080p_2997
    ,NTV2_FORMAT_1080p_3000
    ,NTV2_FORMAT_1080p_2500
    ,NTV2_FORMAT_1080p_2398
    ,NTV2_FORMAT_1080p_2400
    ,NTV2_FORMAT_1080p_2K_2398
    ,NTV2_FORMAT_1080p_2K_2400
    ,NTV2_FORMAT_1080psf_2K_2398
    ,NTV2_FORMAT_1080psf_2K_2400
    ,NTV2_FORMAT_720p_5000
    ,NTV2_FORMAT_1080p_5000_B
    ,NTV2_FORMAT_1080p_5994_B
    ,NTV2_FORMAT_1080p_6000_B
    ,NTV2_FORMAT_720p_2398
    ,NTV2_FORMAT_720p_2500
    ,NTV2_FORMAT_1080p_5000_A
    ,NTV2_FORMAT_1080p_5994_A
    ,NTV2_FORMAT_1080p_6000_A
    ,NTV2_FORMAT_1080p_2K_2500
    ,NTV2_FORMAT_1080psf_2K_2500
    ,NTV2_FORMAT_1080psf_2500_2
    ,NTV2_FORMAT_1080psf_2997_2
    ,NTV2_FORMAT_1080psf_3000_2
    ,NTV2_FORMAT_END_HIGH_DEF_FORMATS

    ,NTV2_FORMAT_525_5994					= NTV2_FORMAT_FIRST_STANDARD_DEF_FORMAT
    ,NTV2_FORMAT_625_5000
    ,NTV2_FORMAT_525_2398
    ,NTV2_FORMAT_525_2400
    ,NTV2_FORMAT_525psf_2997
    ,NTV2_FORMAT_625psf_2500
    ,NTV2_FORMAT_END_STANDARD_DEF_FORMATS

    ,NTV2_FORMAT_2K_1498					= NTV2_FORMAT_FIRST_2K_DEF_FORMAT
    ,NTV2_FORMAT_2K_1500
    ,NTV2_FORMAT_2K_2398
    ,NTV2_FORMAT_2K_2400
    ,NTV2_FORMAT_2K_2500
    ,NTV2_FORMAT_END_2K_DEF_FORMATS

    ,NTV2_FORMAT_4x1920x1080psf_2398		= NTV2_FORMAT_FIRST_4K_DEF_FORMAT
    ,NTV2_FORMAT_4x1920x1080psf_2400
    ,NTV2_FORMAT_4x1920x1080psf_2500
    ,NTV2_FORMAT_4x1920x1080p_2398
    ,NTV2_FORMAT_4x1920x1080p_2400
    ,NTV2_FORMAT_4x1920x1080p_2500
    ,NTV2_FORMAT_4x2048x1080psf_2398
    ,NTV2_FORMAT_4x2048x1080psf_2400
    ,NTV2_FORMAT_4x2048x1080psf_2500
    ,NTV2_FORMAT_4x2048x1080p_2398
    ,NTV2_FORMAT_4x2048x1080p_2400
    ,NTV2_FORMAT_4x2048x1080p_2500
    ,NTV2_FORMAT_4x1920x1080p_2997
    ,NTV2_FORMAT_4x1920x1080p_3000
    ,NTV2_FORMAT_4x1920x1080psf_2997
    ,NTV2_FORMAT_4x1920x1080psf_3000
    ,NTV2_FORMAT_4x2048x1080p_2997
    ,NTV2_FORMAT_4x2048x1080p_3000
    ,NTV2_FORMAT_4x2048x1080psf_2997
    ,NTV2_FORMAT_4x2048x1080psf_3000
    ,NTV2_FORMAT_4x1920x1080p_5000
    ,NTV2_FORMAT_4x1920x1080p_5994
    ,NTV2_FORMAT_4x1920x1080p_6000
    ,NTV2_FORMAT_4x2048x1080p_5000
    ,NTV2_FORMAT_4x2048x1080p_5994
    ,NTV2_FORMAT_4x2048x1080p_6000
    ,NTV2_FORMAT_4x2048x1080p_4795
    ,NTV2_FORMAT_4x2048x1080p_4800
    ,NTV2_FORMAT_4x2048x1080p_11988
    ,NTV2_FORMAT_4x2048x1080p_12000
    ,NTV2_FORMAT_END_4K_DEF_FORMATS

    ,NTV2_FORMAT_1080p_2K_6000_A			= NTV2_FORMAT_FIRST_HIGH_DEF_FORMAT2
    ,NTV2_FORMAT_1080p_2K_5994_A
    ,NTV2_FORMAT_1080p_2K_2997
    ,NTV2_FORMAT_1080p_2K_3000
    ,NTV2_FORMAT_1080p_2K_5000_A
    ,NTV2_FORMAT_1080p_2K_4795_A
    ,NTV2_FORMAT_1080p_2K_4800_A
    ,NTV2_FORMAT_1080p_2K_4795_B
    ,NTV2_FORMAT_1080p_2K_4800_B
    ,NTV2_FORMAT_1080p_2K_5000_B
    ,NTV2_FORMAT_1080p_2K_5994_B
    ,NTV2_FORMAT_1080p_2K_6000_B
    ,NTV2_FORMAT_END_HIGH_DEF_FORMATS2

    ,NTV2_FORMAT_3840x2160psf_2398		= NTV2_FORMAT_FIRST_UHD_TSI_DEF_FORMAT
    ,NTV2_FORMAT_3840x2160psf_2400
    ,NTV2_FORMAT_3840x2160psf_2500
    ,NTV2_FORMAT_3840x2160p_2398
    ,NTV2_FORMAT_3840x2160p_2400
    ,NTV2_FORMAT_3840x2160p_2500
    ,NTV2_FORMAT_3840x2160p_2997
    ,NTV2_FORMAT_3840x2160p_3000
    ,NTV2_FORMAT_3840x2160psf_2997
    ,NTV2_FORMAT_3840x2160psf_3000
    ,NTV2_FORMAT_3840x2160p_5000
    ,NTV2_FORMAT_3840x2160p_5994
    ,NTV2_FORMAT_3840x2160p_6000
    ,NTV2_FORMAT_3840x2160p_5000_B
    ,NTV2_FORMAT_3840x2160p_5994_B
    ,NTV2_FORMAT_3840x2160p_6000_B

    ,NTV2_FORMAT_4096x2160psf_2398		= NTV2_FORMAT_FIRST_4K_TSI_DEF_FORMAT
    ,NTV2_FORMAT_4096x2160psf_2400
    ,NTV2_FORMAT_4096x2160psf_2500
    ,NTV2_FORMAT_4096x2160p_2398
    ,NTV2_FORMAT_4096x2160p_2400
    ,NTV2_FORMAT_4096x2160p_2500
    ,NTV2_FORMAT_4096x2160p_2997
    ,NTV2_FORMAT_4096x2160p_3000
    ,NTV2_FORMAT_4096x2160psf_2997
    ,NTV2_FORMAT_4096x2160psf_3000
    ,NTV2_FORMAT_4096x2160p_4795
    ,NTV2_FORMAT_4096x2160p_4800
    ,NTV2_FORMAT_4096x2160p_5000
    ,NTV2_FORMAT_4096x2160p_5994
    ,NTV2_FORMAT_4096x2160p_6000
    ,NTV2_FORMAT_4096x2160p_11988
    ,NTV2_FORMAT_4096x2160p_12000
    ,NTV2_FORMAT_4096x2160p_4795_B
    ,NTV2_FORMAT_4096x2160p_4800_B
    ,NTV2_FORMAT_4096x2160p_5000_B
    ,NTV2_FORMAT_4096x2160p_5994_B
    ,NTV2_FORMAT_4096x2160p_6000_B
    ,NTV2_FORMAT_END_4K_TSI_DEF_FORMATS

    ,NTV2_FORMAT_4x1920x1080p_5000_B	= NTV2_FORMAT_FIRST_4K_DEF_FORMAT2
    ,NTV2_FORMAT_4x1920x1080p_5994_B
    ,NTV2_FORMAT_4x1920x1080p_6000_B
    ,NTV2_FORMAT_4x2048x1080p_5000_B
    ,NTV2_FORMAT_4x2048x1080p_5994_B
    ,NTV2_FORMAT_4x2048x1080p_6000_B
    ,NTV2_FORMAT_4x2048x1080p_4795_B
    ,NTV2_FORMAT_4x2048x1080p_4800_B
    ,NTV2_FORMAT_END_4K_DEF_FORMATS2

    ,NTV2_FORMAT_4x3840x2160p_2398		= NTV2_FORMAT_FIRST_UHD2_DEF_FORMAT
    ,NTV2_FORMAT_4x3840x2160p_2400
    ,NTV2_FORMAT_4x3840x2160p_2500
    ,NTV2_FORMAT_4x3840x2160p_2997
    ,NTV2_FORMAT_4x3840x2160p_3000
    ,NTV2_FORMAT_4x3840x2160p_5000
    ,NTV2_FORMAT_4x3840x2160p_5994
    ,NTV2_FORMAT_4x3840x2160p_6000
    ,NTV2_FORMAT_4x3840x2160p_5000_B
    ,NTV2_FORMAT_4x3840x2160p_5994_B
    ,NTV2_FORMAT_4x3840x2160p_6000_B
    ,NTV2_FORMAT_END_UHD2_DEF_FORMATS

    ,NTV2_FORMAT_4x4096x2160p_2398		= NTV2_FORMAT_FIRST_UHD2_FULL_DEF_FORMAT
    ,NTV2_FORMAT_4x4096x2160p_2400
    ,NTV2_FORMAT_4x4096x2160p_2500
    ,NTV2_FORMAT_4x4096x2160p_2997
    ,NTV2_FORMAT_4x4096x2160p_3000
    ,NTV2_FORMAT_4x4096x2160p_4795
    ,NTV2_FORMAT_4x4096x2160p_4800
    ,NTV2_FORMAT_4x4096x2160p_5000
    ,NTV2_FORMAT_4x4096x2160p_5994
    ,NTV2_FORMAT_4x4096x2160p_6000
    ,NTV2_FORMAT_4x4096x2160p_4795_B
    ,NTV2_FORMAT_4x4096x2160p_4800_B
    ,NTV2_FORMAT_4x4096x2160p_5000_B
    ,NTV2_FORMAT_4x4096x2160p_5994_B
    ,NTV2_FORMAT_4x4096x2160p_6000_B
    ,NTV2_FORMAT_END_UHD2_FULL_DEF_FORMATS

    ,NTV2_MAX_NUM_VIDEO_FORMATS = NTV2_FORMAT_END_UHD2_FULL_DEF_FORMATS
} NTV2VideoFormat;

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
    { "raw_format",     "frame buffer format",      OFFSET(raw_format),         AV_OPT_TYPE_INT,        { .i64 = NTV2_FBF_8BIT_YCBCR },     NTV2_FBF_FIRST, NTV2_FBF_LAST, DEC, "raw_format" },
    { "8BIT_YCBCR",     NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FBF_8BIT_YCBCR },     0, 0,              DEC, "raw_format"},
    { "10BIT_YCBCR",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FBF_10BIT_YCBCR },    0, 0,              DEC, "raw_format"},
    { "video_format",   "video format",             OFFSET(video_format),       AV_OPT_TYPE_INT,        { .i64 = 1 },                       0, 414,            DEC, "video_format" },
    // Compat
    { "1080i5000",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 1 },                       0, 0,              DEC, "video_format" },
    { "1080p2500",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 10 },                      0, 0,              DEC, "video_format" },
    { "720p5000",       NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 17 },                      0, 0,              DEC, "video_format" },
    { "1080p5000",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = 23 },                      0, 0,              DEC, "video_format" },
    // HD
    { "1080i_5000",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080i_5000 },           0, 0,              DEC, "video_format" },
    { "1080i_5994",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080i_5994 },           0, 0,              DEC, "video_format" },
    { "1080i_6000",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080i_6000 },           0, 0,              DEC, "video_format" },
    { "720p_5994",            NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_720p_5994 },            0, 0,              DEC, "video_format" },
    { "720p_6000",            NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_720p_6000 },            0, 0,              DEC, "video_format" },
    { "1080psf_2398",         NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080psf_2398 },         0, 0,              DEC, "video_format" },
    { "1080psf_2400",         NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080psf_2400 },         0, 0,              DEC, "video_format" },
    { "1080p_2997",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2997 },           0, 0,              DEC, "video_format" },
    { "1080p_3000",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_3000 },           0, 0,              DEC, "video_format" },
    { "1080p_2500",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2500 },           0, 0,              DEC, "video_format" },
    { "1080p_2398",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2398 },           0, 0,              DEC, "video_format" },
    { "1080p_2400",           NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2400 },           0, 0,              DEC, "video_format" },
    { "1080p_2K_2398",        NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_2398 },        0, 0,              DEC, "video_format" },
    { "1080p_2K_2400",        NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_2400 },        0, 0,              DEC, "video_format" },
    { "1080psf_2K_2398",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080psf_2K_2398 },      0, 0,              DEC, "video_format" },
    { "1080psf_2K_2400",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080psf_2K_2400 },      0, 0,              DEC, "video_format" },
    { "720p_5000",            NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_720p_5000 },            0, 0,              DEC, "video_format" },
    { "1080p_5000_B",         NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_5000_B },         0, 0,              DEC, "video_format" },
    { "1080p_5994_B",         NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_5994_B },         0, 0,              DEC, "video_format" },
    { "1080p_6000_B",         NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_6000_B },         0, 0,              DEC, "video_format" },
    { "720p_2398",            NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_720p_2398 },            0, 0,              DEC, "video_format" },
    { "720p_2500",            NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_720p_2500 },            0, 0,              DEC, "video_format" },
    { "1080p_5000_A",         NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_5000_A },         0, 0,              DEC, "video_format" },
    { "1080p_5994_A",         NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_5994_A },         0, 0,              DEC, "video_format" },
    { "1080p_6000_A",         NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_6000_A },         0, 0,              DEC, "video_format" },
    { "1080p_2K_2500",        NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_2500 },        0, 0,              DEC, "video_format" },
    { "1080psf_2K_2500",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080psf_2K_2500 },      0, 0,              DEC, "video_format" },
    { "1080psf_2500_2",       NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080psf_2500_2 },       0, 0,              DEC, "video_format" },
    { "1080psf_2997_2",       NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080psf_2997_2 },       0, 0,              DEC, "video_format" },
    { "1080psf_3000_2",       NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080psf_3000_2 },       0, 0,              DEC, "video_format" },
    // SD
    { "525_5994",             NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_525_5994 },             0, 0,              DEC, "video_format" },
    { "625_5000",             NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_625_5000 },             0, 0,              DEC, "video_format" },
    { "525_2398",             NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_525_2398 },             0, 0,              DEC, "video_format" },
    { "525_2400",             NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_525_2400 },             0, 0,              DEC, "video_format" },
    { "525psf_2997",          NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_525psf_2997 },          0, 0,              DEC, "video_format" },
    { "625psf_2500",          NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_625psf_2500 },          0, 0,              DEC, "video_format" },
    // 2K
    { "2K_1498",              NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_2K_1498 },              0, 0,              DEC, "video_format" },
    { "2K_1500",              NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_2K_1500 },              0, 0,              DEC, "video_format" },
    { "2K_2398",              NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_2K_2398 },              0, 0,              DEC, "video_format" },
    { "2K_2400",              NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_2K_2400 },              0, 0,              DEC, "video_format" },
    { "2K_2500",              NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_2K_2500 },              0, 0,              DEC, "video_format" },
    // 4K
    { "4x1920x1080psf_2398",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080psf_2398 },  0, 0,              DEC, "video_format" },
    { "4x1920x1080psf_2400",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080psf_2400 },  0, 0,              DEC, "video_format" },
    { "4x1920x1080psf_2500",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080psf_2500 },  0, 0,              DEC, "video_format" },
    { "4x1920x1080p_2398",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_2398 },    0, 0,              DEC, "video_format" },
    { "4x1920x1080p_2400",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_2400 },    0, 0,              DEC, "video_format" },
    { "4x1920x1080p_2500",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_2500 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080psf_2398",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080psf_2398 },  0, 0,              DEC, "video_format" },
    { "4x2048x1080psf_2400",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080psf_2400 },  0, 0,              DEC, "video_format" },
    { "4x2048x1080psf_2500",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080psf_2500 },  0, 0,              DEC, "video_format" },
    { "4x2048x1080p_2398",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_2398 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080p_2400",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_2400 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080p_2500",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_2500 },    0, 0,              DEC, "video_format" },
    { "4x1920x1080p_2997",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_2997 },    0, 0,              DEC, "video_format" },
    { "4x1920x1080p_3000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_3000 },    0, 0,              DEC, "video_format" },
    { "4x1920x1080psf_2997",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080psf_2997 },  0, 0,              DEC, "video_format" },
    { "4x1920x1080psf_3000",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080psf_3000 },  0, 0,              DEC, "video_format" },
    { "4x2048x1080p_2997",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_2997 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080p_3000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_3000 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080psf_2997",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080psf_2997 },  0, 0,              DEC, "video_format" },
    { "4x2048x1080psf_3000",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080psf_3000 },  0, 0,              DEC, "video_format" },
    { "4x1920x1080p_5000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_5000 },    0, 0,              DEC, "video_format" },
    { "4x1920x1080p_5994",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_5994 },    0, 0,              DEC, "video_format" },
    { "4x1920x1080p_6000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_6000 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080p_5000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_5000 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080p_5994",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_5994 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080p_6000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_6000 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080p_4795",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_4795 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080p_4800",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_4800 },    0, 0,              DEC, "video_format" },
    { "4x2048x1080p_11988",   NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_11988 },   0, 0,              DEC, "video_format" },
    { "4x2048x1080p_12000",   NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_12000 },   0, 0,              DEC, "video_format" },
    // HD 2
    { "1080p_2K_6000_A",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_6000_A },      0, 0,              DEC, "video_format" },
    { "1080p_2K_5994_A",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_5994_A },      0, 0,              DEC, "video_format" },
    { "1080p_2K_2997",        NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_2997 },        0, 0,              DEC, "video_format" },
    { "1080p_2K_3000",        NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_3000 },        0, 0,              DEC, "video_format" },
    { "1080p_2K_5000_A",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_5000_A },      0, 0,              DEC, "video_format" },
    { "1080p_2K_4795_A",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_4795_A },      0, 0,              DEC, "video_format" },
    { "1080p_2K_4800_A",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_4800_A },      0, 0,              DEC, "video_format" },
    { "1080p_2K_4795_B",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_4795_B },      0, 0,              DEC, "video_format" },
    { "1080p_2K_4800_B",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_4800_B },      0, 0,              DEC, "video_format" },
    { "1080p_2K_5000_B",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_5000_B },      0, 0,              DEC, "video_format" },
    { "1080p_2K_5994_B",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_5994_B },      0, 0,              DEC, "video_format" },
    { "1080p_2K_6000_B",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_1080p_2K_6000_B },      0, 0,              DEC, "video_format" },
    // UHD TSI
    { "3840x2160psf_2398",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160psf_2398 },    0, 0,              DEC, "video_format" },
    { "3840x2160psf_2400",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160psf_2400 },    0, 0,              DEC, "video_format" },
    { "3840x2160psf_2500",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160psf_2500 },    0, 0,              DEC, "video_format" },
    { "3840x2160p_2398",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_2398 },      0, 0,              DEC, "video_format" },
    { "3840x2160p_2400",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_2400 },      0, 0,              DEC, "video_format" },
    { "3840x2160p_2500",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_2500 },      0, 0,              DEC, "video_format" },
    { "3840x2160p_2997",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_2997 },      0, 0,              DEC, "video_format" },
    { "3840x2160p_3000",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_3000 },      0, 0,              DEC, "video_format" },
    { "3840x2160psf_2997",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160psf_2997 },    0, 0,              DEC, "video_format" },
    { "3840x2160psf_3000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160psf_3000 },    0, 0,              DEC, "video_format" },
    { "3840x2160p_5000",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_5000 },      0, 0,              DEC, "video_format" },
    { "3840x2160p_5994",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_5994 },      0, 0,              DEC, "video_format" },
    { "3840x2160p_6000",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_6000 },      0, 0,              DEC, "video_format" },
    { "3840x2160p_5000_B",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_5000_B },    0, 0,              DEC, "video_format" },
    { "3840x2160p_5994_B",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_5994_B },    0, 0,              DEC, "video_format" },
    { "3840x2160p_6000_B",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_3840x2160p_6000_B },    0, 0,              DEC, "video_format" },
    // 4K TSI
    { "4096x2160psf_2398",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160psf_2398 },    0, 0,              DEC, "video_format" },
    { "4096x2160psf_2400",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160psf_2400 },    0, 0,              DEC, "video_format" },
    { "4096x2160psf_2500",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160psf_2500 },    0, 0,              DEC, "video_format" },
    { "4096x2160p_2398",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_2398 },      0, 0,              DEC, "video_format" },
    { "4096x2160p_2400",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_2400 },      0, 0,              DEC, "video_format" },
    { "4096x2160p_2500",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_2500 },      0, 0,              DEC, "video_format" },
    { "4096x2160p_2997",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_2997 },      0, 0,              DEC, "video_format" },
    { "4096x2160p_3000",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_3000 },      0, 0,              DEC, "video_format" },
    { "4096x2160psf_2997",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160psf_2997 },    0, 0,              DEC, "video_format" },
    { "4096x2160psf_3000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160psf_3000 },    0, 0,              DEC, "video_format" },
    { "4096x2160p_4795",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_4795 },      0, 0,              DEC, "video_format" },
    { "4096x2160p_4800",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_4800 },      0, 0,              DEC, "video_format" },
    { "4096x2160p_5000",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_5000 },      0, 0,              DEC, "video_format" },
    { "4096x2160p_5994",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_5994 },      0, 0,              DEC, "video_format" },
    { "4096x2160p_6000",      NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_6000 },      0, 0,              DEC, "video_format" },
    { "4096x2160p_11988",     NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_11988 },     0, 0,              DEC, "video_format" },
    { "4096x2160p_12000",     NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_12000 },     0, 0,              DEC, "video_format" },
    { "4096x2160p_4795_B",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_4795_B },    0, 0,              DEC, "video_format" },
    { "4096x2160p_4800_B",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_4800_B },    0, 0,              DEC, "video_format" },
    { "4096x2160p_5000_B",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_5000_B },    0, 0,              DEC, "video_format" },
    { "4096x2160p_5994_B",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_5994_B },    0, 0,              DEC, "video_format" },
    { "4096x2160p_6000_B",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4096x2160p_6000_B },    0, 0,              DEC, "video_format" },
    // 4K 2
    { "4x1920x1080p_5000_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_5000_B },  0, 0,              DEC, "video_format" },
    { "4x1920x1080p_5994_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_5994_B },  0, 0,              DEC, "video_format" },
    { "4x1920x1080p_6000_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x1920x1080p_6000_B },  0, 0,              DEC, "video_format" },
    { "4x2048x1080p_5000_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_5000_B },  0, 0,              DEC, "video_format" },
    { "4x2048x1080p_5994_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_5994_B },  0, 0,              DEC, "video_format" },
    { "4x2048x1080p_6000_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_6000_B },  0, 0,              DEC, "video_format" },
    { "4x2048x1080p_4795_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_4795_B },  0, 0,              DEC, "video_format" },
    { "4x2048x1080p_4800_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x2048x1080p_4800_B },  0, 0,              DEC, "video_format" },
    // UHD 2
    { "4x3840x2160p_2398",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_2398 },    0, 0,              DEC, "video_format" },
    { "4x3840x2160p_2400",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_2400 },    0, 0,              DEC, "video_format" },
    { "4x3840x2160p_2500",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_2500 },    0, 0,              DEC, "video_format" },
    { "4x3840x2160p_2997",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_2997 },    0, 0,              DEC, "video_format" },
    { "4x3840x2160p_3000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_3000 },    0, 0,              DEC, "video_format" },
    { "4x3840x2160p_5000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_5000 },    0, 0,              DEC, "video_format" },
    { "4x3840x2160p_5994",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_5994 },    0, 0,              DEC, "video_format" },
    { "4x3840x2160p_6000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_6000 },    0, 0,              DEC, "video_format" },
    { "4x3840x2160p_5000_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_5000_B },  0, 0,              DEC, "video_format" },
    { "4x3840x2160p_5994_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_5994_B },  0, 0,              DEC, "video_format" },
    { "4x3840x2160p_6000_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x3840x2160p_6000_B },  0, 0,              DEC, "video_format" },
    // UHD 2 FULL
    { "4x4096x2160p_2398",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_2398 },    0, 0,              DEC, "video_format" },
    { "4x4096x2160p_2400",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_2400 },    0, 0,              DEC, "video_format" },
    { "4x4096x2160p_2500",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_2500 },    0, 0,              DEC, "video_format" },
    { "4x4096x2160p_2997",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_2997 },    0, 0,              DEC, "video_format" },
    { "4x4096x2160p_3000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_3000 },    0, 0,              DEC, "video_format" },
    { "4x4096x2160p_4795",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_4795 },    0, 0,              DEC, "video_format" },
    { "4x4096x2160p_4800",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_4800 },    0, 0,              DEC, "video_format" },
    { "4x4096x2160p_5000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_5000 },    0, 0,              DEC, "video_format" },
    { "4x4096x2160p_5994",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_5994 },    0, 0,              DEC, "video_format" },
    { "4x4096x2160p_6000",    NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_6000 },    0, 0,              DEC, "video_format" },
    { "4x4096x2160p_4795_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_4795_B },  0, 0,              DEC, "video_format" },
    { "4x4096x2160p_4800_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_4800_B },  0, 0,              DEC, "video_format" },
    { "4x4096x2160p_5000_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_5000_B },  0, 0,              DEC, "video_format" },
    { "4x4096x2160p_5994_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_5994_B },  0, 0,              DEC, "video_format" },
    { "4x4096x2160p_6000_B",  NULL,                       0,                          AV_OPT_TYPE_CONST,      { .i64 = NTV2_FORMAT_4x4096x2160p_6000_B },  0, 0,              DEC, "video_format" },

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

const AVInputFormat ff_ntv2_demuxer = {
    .name           = "ntv2",
    .long_name      = NULL_IF_CONFIG_SMALL("AJA NTV2 input"),
    .flags          = AVFMT_NOFILE,
    .priv_class     = &ntv2_demuxer_class,
    .priv_data_size = sizeof(struct NTV2Context),
    .read_header    = ff_ntv2_read_header,
    .read_packet    = ff_ntv2_read_packet,
    .read_close     = ff_ntv2_read_close,
};

/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Video black detector, loosely based on blackframe with extended
 * syntax and features
 */

#include <float.h>
#include <stdbool.h>
#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_cuda_internal.h"
#include "libavutil/cuda_check.h"
#include "libavutil/internal.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/timestamp.h"

#include "avfilter.h"
#include "filters.h"
#include "video.h"

#include "cuda/load_helper.h"

typedef struct CUDABlackDetectContext {
    const AVClass *class;
    AVCUDADeviceContext *hwctx;
    AVBufferRef *frames_ctx;

    /**
     * Output sw format. AV_PIX_FMT_NONE for no conversion.
     */
    enum AVPixelFormat format;

    double  black_min_duration_time; ///< minimum duration of detected black, in seconds
    int64_t black_min_duration;      ///< minimum duration of detected black, expressed in timebase units
    int64_t black_start;             ///< pts start time of the first black picture
    int64_t black_end;               ///< pts end time of the last black picture
    int64_t last_picref_pts;         ///< pts of the last input picture
    int black_started;

    double       picture_black_ratio_th;
    double       pixel_black_th;
    unsigned int pixel_black_th_i;

    unsigned int nb_black_pixels;   ///< number of black pixels counted so far
    AVRational   time_base;
    int          depth;

    int block_count;
    int pixels_per_iteration;
    unsigned int* h_sums;
    CUdeviceptr d_sums;

    CUmodule    cu_module;
    CUfunction  cu_kernel_8bit;
    CUfunction  cu_kernel_16bit;
    CUstream    cu_stream;
} CUDABlackDetectContext;

#define OFFSET(x) offsetof(CUDABlackDetectContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption cudablackdetect_options[] = {
    { "d",                  "set minimum detected black duration in seconds", OFFSET(black_min_duration_time), AV_OPT_TYPE_DOUBLE, {.dbl=2}, 0, DBL_MAX, FLAGS },
    { "black_min_duration", "set minimum detected black duration in seconds", OFFSET(black_min_duration_time), AV_OPT_TYPE_DOUBLE, {.dbl=2}, 0, DBL_MAX, FLAGS },
    { "picture_black_ratio_th", "set the picture black ratio threshold", OFFSET(picture_black_ratio_th), AV_OPT_TYPE_DOUBLE, {.dbl=.98}, 0, 1, FLAGS },
    { "pic_th",                 "set the picture black ratio threshold", OFFSET(picture_black_ratio_th), AV_OPT_TYPE_DOUBLE, {.dbl=.98}, 0, 1, FLAGS },
    { "pixel_black_th", "set the pixel black threshold", OFFSET(pixel_black_th), AV_OPT_TYPE_DOUBLE, {.dbl=.10}, 0, 1, FLAGS },
    { "pix_th",         "set the pixel black threshold", OFFSET(pixel_black_th), AV_OPT_TYPE_DOUBLE, {.dbl=.10}, 0, 1, FLAGS },
    { NULL }
};

AVFILTER_DEFINE_CLASS(cudablackdetect);

#define YUVJ_FORMATS \
    AV_PIX_FMT_YUVJ411P, AV_PIX_FMT_YUVJ420P, AV_PIX_FMT_YUVJ422P, AV_PIX_FMT_YUVJ444P, AV_PIX_FMT_YUVJ440P

static const enum AVPixelFormat yuvj_formats[] = {
    YUVJ_FORMATS, AV_PIX_FMT_NONE
};

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV411P,
    AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_NV12, AV_PIX_FMT_NV21,
    YUVJ_FORMATS,
    AV_PIX_FMT_GRAY10, AV_PIX_FMT_GRAY12, AV_PIX_FMT_GRAY14,
    AV_PIX_FMT_GRAY16,
    AV_PIX_FMT_YUV420P9, AV_PIX_FMT_YUV422P9, AV_PIX_FMT_YUV444P9,
    AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10,
    AV_PIX_FMT_YUV440P10,
    AV_PIX_FMT_YUV444P12, AV_PIX_FMT_YUV422P12, AV_PIX_FMT_YUV420P12,
    AV_PIX_FMT_YUV440P12,
    AV_PIX_FMT_YUV444P14, AV_PIX_FMT_YUV422P14, AV_PIX_FMT_YUV420P14,
    AV_PIX_FMT_YUV420P16, AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV444P16,
    AV_PIX_FMT_YUVA420P,  AV_PIX_FMT_YUVA422P,   AV_PIX_FMT_YUVA444P,
    AV_PIX_FMT_YUVA444P9, AV_PIX_FMT_YUVA444P10, AV_PIX_FMT_YUVA444P12, AV_PIX_FMT_YUVA444P16,
    AV_PIX_FMT_YUVA422P9, AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_YUVA422P12, AV_PIX_FMT_YUVA422P16,
    AV_PIX_FMT_YUVA420P9, AV_PIX_FMT_YUVA420P10, AV_PIX_FMT_YUVA420P16,
    AV_PIX_FMT_NONE
};

#define BLOCKX 32
#define BLOCKY 8
#define DIV_UP(a, b) ( ((a) + (b) - 1) / (b) )
#define CHECK_CU(x) FF_CUDA_CHECK_DL(ctx, s->hwctx->internal->cuda_dl, x)

static int format_is_supported(enum AVPixelFormat fmt)
{
    int i;

    for (i = 0; i < FF_ARRAY_ELEMS(pix_fmts); i++)
        if (pix_fmts[i] == fmt)
            return 1;
    return 0;
}

static av_cold int init_processing_chain(AVFilterContext *ctx)
{
    CUDABlackDetectContext *s = ctx->priv;
    AVHWFramesContext *in_frames_ctx;
    FilterLink         *inl = ff_filter_link(ctx->inputs[0]);
    FilterLink        *outl = ff_filter_link(ctx->outputs[0]);

    enum AVPixelFormat in_format;
    enum AVPixelFormat out_format;

    /* check that we have a hw context */
    if (!inl->hw_frames_ctx) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext*)inl->hw_frames_ctx->data;
    in_format     = in_frames_ctx->sw_format;
    out_format    = (s->format == AV_PIX_FMT_NONE) ? in_format : s->format;


    if (!format_is_supported(in_frames_ctx->sw_format)) {
        av_log(ctx, AV_LOG_ERROR, "Unsupported format: %s\n", av_get_pix_fmt_name(in_frames_ctx->sw_format));
        return AVERROR(ENOSYS);
    }

    s->frames_ctx = av_buffer_ref(inl->hw_frames_ctx);
    if (!s->frames_ctx)
        return AVERROR(ENOMEM);

    outl->hw_frames_ctx = av_buffer_ref(s->frames_ctx);
    if (!outl->hw_frames_ctx)
        return AVERROR(ENOMEM);
    return 0;
}

static av_cold int cuda_blackdetect_load_functions(AVFilterContext *ctx)
{
    CUDABlackDetectContext *s = ctx->priv;
    CUcontext blackdetect, cuda_ctx = s->hwctx->cuda_ctx;
    CudaFunctions *cu = s->hwctx->internal->cuda_dl;
    int ret;

    extern const unsigned char ff_vf_blackdetect_cuda_ptx_data[];
    extern const unsigned int ff_vf_blackdetect_cuda_ptx_len;

    ret = CHECK_CU(cu->cuCtxPushCurrent(cuda_ctx));
    if (ret < 0)
        return ret;

    ret = ff_cuda_load_module(ctx, s->hwctx, &s->cu_module,
                              ff_vf_blackdetect_cuda_ptx_data, ff_vf_blackdetect_cuda_ptx_len);
    if (ret < 0)
        goto fail;

    ret = CHECK_CU(cu->cuModuleGetFunction(&s->cu_kernel_8bit, s->cu_module, "blackdetect_8"));
    if (ret < 0) {
        av_log(ctx, AV_LOG_FATAL, "Failed loading blackdetect_8\n");
        goto fail;
    }

    ret = CHECK_CU(cu->cuModuleGetFunction(&s->cu_kernel_16bit, s->cu_module, "blackdetect_16"));
    if (ret < 0) {
        av_log(ctx, AV_LOG_FATAL, "Failed loading blackdetect_16\n");
        goto fail;
    }

fail:
    CHECK_CU(cu->cuCtxPopCurrent(&blackdetect));

    return ret;
}

static av_cold int cuda_blackdetect_config_props(AVFilterLink *outlink)
{
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = outlink->src->inputs[0];
    FilterLink      *inl = ff_filter_link(inlink);
    CUDABlackDetectContext *s = ctx->priv;
    CudaFunctions *cu;
    AVHWFramesContext     *frames_ctx = (AVHWFramesContext*)inl->hw_frames_ctx->data;
    AVCUDADeviceContext *device_hwctx = frames_ctx->device_ctx->hwctx;
    CUcontext dummy;
    
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(frames_ctx->sw_format);
    const int depth = desc->comp[0].depth;
    bool is_16bit = depth > 8;

    int w = ctx->inputs[0]->w;
    int h = ctx->inputs[0]->h;
    outlink->w = w;
    outlink->h = h;

    int ret = 0;

    int pixels_per_iteration = is_16bit ? 4 : 8;

    int grid_x = DIV_UP(w / pixels_per_iteration, BLOCKX);
    int grid_y = DIV_UP(h, BLOCKY);

    int block_count = grid_x * grid_y;

    s->hwctx = device_hwctx;
    s->cu_stream = s->hwctx->stream;

    ret = init_processing_chain(ctx);
    if (ret < 0)
        return ret;

    s->depth = depth;
    s->pixels_per_iteration = pixels_per_iteration;
    s->block_count = block_count;

    s->time_base = inlink->time_base;
    s->black_min_duration = s->black_min_duration_time / av_q2d(s->time_base);

    ret = cuda_blackdetect_load_functions(ctx);
    if (ret < 0)
        return ret;

    cu = s->hwctx->internal->cuda_dl;
    ret = CHECK_CU(cu->cuCtxPushCurrent(s->hwctx->cuda_ctx));
    if (ret < 0)
        return ret;

    s->h_sums = malloc(sizeof(int) * block_count);
    if(!s->h_sums) {
        ret = AVERROR(ENOMEM);
        goto exit;
    }

    ret = CHECK_CU(cu->cuMemAlloc(&s->d_sums, s->block_count * sizeof(int)));
    if (ret < 0)
        goto exit;

exit:
    CHECK_CU(cu->cuCtxPopCurrent(&dummy));
    return ret;
}

static void check_black_end(AVFilterContext *ctx)
{
    CUDABlackDetectContext *s = ctx->priv;

    if ((s->black_end - s->black_start) >= s->black_min_duration) {
        av_log(s, AV_LOG_INFO,
               "black_start:%s black_end:%s black_duration:%s\n",
               av_ts2timestr(s->black_start, &s->time_base),
               av_ts2timestr(s->black_end,   &s->time_base),
               av_ts2timestr(s->black_end - s->black_start, &s->time_base));
    }
}

static int cuda_blackdetect_process(AVFilterContext *ctx, AVFrame *in)
{
    CUDABlackDetectContext *s = ctx->priv;
    CudaFunctions *cu = s->hwctx->internal->cuda_dl;
    CUcontext dummy, cuda_ctx = s->hwctx->cuda_ctx;
    int ret;

    int nb_black_pixels = 0;
    bool is_16bit = s->depth > 8;
    int linesize = in->linesize[0];
    int w = in->width;
    int h = in->height;
    const int lim = s->pixel_black_th_i;

    int grid_x = DIV_UP(w / s->pixels_per_iteration, BLOCKX);
    int grid_y = DIV_UP(h, BLOCKY);

    CUfunction kernel = is_16bit ? s->cu_kernel_16bit : s->cu_kernel_8bit;
    CUdeviceptr data = (CUdeviceptr)in->data[0];
    unsigned int packed_limits = is_16bit ? (lim << 16) | lim : (lim << 24) | (lim << 16) | (lim << 8) | lim;

    CUDA_TEXTURE_DESC tex_desc = {
        .filterMode = CU_TR_FILTER_MODE_POINT,
        .flags = CU_TRSF_READ_AS_INTEGER,
        .addressMode = {CU_TR_ADDRESS_MODE_BORDER, CU_TR_ADDRESS_MODE_BORDER, CU_TR_ADDRESS_MODE_BORDER},
    };
    unsigned int *borderColor = (unsigned int *)(tex_desc.borderColor);

    CUDA_RESOURCE_DESC res_desc = {
        .resType = CU_RESOURCE_TYPE_PITCH2D,
        .res.pitch2D.format = CU_AD_FORMAT_UNSIGNED_INT16,
        .res.pitch2D.numChannels = 4,
        .res.pitch2D.width = w / s->pixels_per_iteration,
        .res.pitch2D.height = h,
        .res.pitch2D.pitchInBytes = linesize,
        .res.pitch2D.devPtr = data
    };
    CUtexObject tex;

    void *kernel_args[] = {
        &(s->d_sums), &packed_limits, &grid_x, &tex
    };

    borderColor[0] = 0xffffffff;
    borderColor[1] = 0xffffffff;
    borderColor[2] = 0xffffffff;
    borderColor[3] = 0xffffffff;

    ret = CHECK_CU(cu->cuCtxPushCurrent(cuda_ctx));
    if (ret < 0)
        return ret;

    ret = CHECK_CU(cu->cuTexObjectCreate(&tex, &res_desc, &tex_desc, NULL));
    if (ret < 0)
        goto exit;


    ret = CHECK_CU(cu->cuLaunchKernel(kernel,
                                      grid_x, grid_y, 1,
                                      BLOCKX, BLOCKY, 1, 32 * sizeof(int), s->cu_stream, kernel_args, NULL));
    if (ret < 0)
        goto exit;

    // copy result back to host
    cu->cuMemcpyDtoH(s->h_sums, s->d_sums, s->block_count * sizeof(int));

    // sum the sums
    for (int i = 0; i < s->block_count; i++)
    {
        nb_black_pixels += s->h_sums[i];
    }

    s->nb_black_pixels = nb_black_pixels;

exit:
    if (tex)
        CHECK_CU(cu->cuTexObjectDestroy(tex));

    CHECK_CU(cu->cuCtxPopCurrent(&dummy));
    return ret;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *picref)
{
    AVFilterContext *ctx = inlink->dst;
    CUDABlackDetectContext *s = ctx->priv;
    FilterLink      *inl = ff_filter_link(inlink);
    int ret = 0;

    double picture_black_ratio = 0;
    const int max = (1 << s->depth) - 1;
    const int factor = (1 << (s->depth - 8));
    const int full = picref->color_range == AVCOL_RANGE_JPEG ||
                     ff_fmt_is_in(picref->format, yuvj_formats);

    s->pixel_black_th_i = full ? s->pixel_black_th * max :
        // luminance_minimum_value + pixel_black_th * luminance_range_size
        16 * factor + s->pixel_black_th * (235 - 16) * factor;

    ret = cuda_blackdetect_process(ctx, picref);

    picture_black_ratio = (double)s->nb_black_pixels / (inlink->w * inlink->h);

    av_log(ctx, AV_LOG_DEBUG,
           "frame:%"PRId64" picture_black_ratio:%f pts:%s t:%s type:%c\n",
           inl->frame_count_out, picture_black_ratio,
           av_ts2str(picref->pts), av_ts2timestr(picref->pts, &s->time_base),
           av_get_picture_type_char(picref->pict_type));

    if (picture_black_ratio >= s->picture_black_ratio_th) {
        if (!s->black_started) {
            /* black starts here */
            s->black_started = 1;
            s->black_start = picref->pts;
            av_dict_set(&picref->metadata, "lavfi.black_start",
                av_ts2timestr(s->black_start, &s->time_base), 0);
        }
    } else if (s->black_started) {
        /* black ends here */
        s->black_started = 0;
        s->black_end = picref->pts;
        check_black_end(ctx);
        av_dict_set(&picref->metadata, "lavfi.black_end",
            av_ts2timestr(s->black_end, &s->time_base), 0);
    }

    s->last_picref_pts = picref->pts;
    s->nb_black_pixels = 0;

    if(ret != 0)
        return ret;

    return ff_filter_frame(inlink->dst->outputs[0], picref);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    CUDABlackDetectContext *s = ctx->priv;

    if(s->h_sums) {
        free(s->h_sums);
        s->h_sums = NULL;
    }

    if (s->hwctx && s->cu_module) {
        CudaFunctions *cu = s->hwctx->internal->cuda_dl;
        CUcontext dummy;

        if(s->d_sums)
            CHECK_CU(cu->cuMemFree(s->d_sums));

        CHECK_CU(cu->cuCtxPushCurrent(s->hwctx->cuda_ctx));
        CHECK_CU(cu->cuModuleUnload(s->cu_module));
        s->cu_module = NULL;
        CHECK_CU(cu->cuCtxPopCurrent(&dummy));
    }

    av_buffer_unref(&s->frames_ctx);

    if (s->black_started) {
        // FIXME: black_end should be set to last_picref_pts + last_picref_duration
        s->black_end = s->last_picref_pts;
        check_black_end(ctx);
    }
}

static const AVFilterPad cudablackdetect_inputs[] = {
    {
        .name          = "default",
        .type          = AVMEDIA_TYPE_VIDEO,
        .filter_frame  = filter_frame,
    },
};

static const AVFilterPad cudablackdetect_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props  = cuda_blackdetect_config_props,
    },
};


const FFFilter ff_vf_blackdetect_cuda = {
    .p.name          = "blackdetect_cuda",
    .p.description   = NULL_IF_CONFIG_SMALL("GPU accelerated blackdetect filter."),
    .priv_size     = sizeof(CUDABlackDetectContext),
    FILTER_INPUTS(cudablackdetect_inputs),
    FILTER_OUTPUTS(cudablackdetect_outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_CUDA),

    .uninit        = uninit,
    .p.flags = AVFILTER_FLAG_METADATA_ONLY,
    .p.priv_class    = &cudablackdetect_class,
    .flags_internal         = FF_FILTER_FLAG_HWFRAME_AWARE,
};

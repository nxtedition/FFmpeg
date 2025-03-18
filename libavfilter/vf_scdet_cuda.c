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
 * video scene change detection filter
 */

#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_cuda_internal.h"
#include "libavutil/cuda_check.h"
#include "libavutil/imgutils.h"
#include "libavutil/internal.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/timestamp.h"

#include "avfilter.h"
#include "filters.h"
#include "video.h"

#include "cuda/load_helper.h"

typedef struct CUDASCDetContext {
    const AVClass *class;
    AVCUDADeviceContext *hwctx;
    AVBufferRef *frames_ctx;

    ptrdiff_t width[4];
    ptrdiff_t height[4];
    int nb_planes;
    int bitdepth;
    double prev_mafd;
    double scene_score;
    AVFrame *prev_picref;
    double threshold;
    int sc_pass;

    CUmodule    cu_module;
    CUfunction  cu_kernel;
    CUstream    cu_stream;
} CUDASCDetContext;

#define OFFSET(x) offsetof(CUDASCDetContext, x)
#define V AV_OPT_FLAG_VIDEO_PARAM
#define F AV_OPT_FLAG_FILTERING_PARAM

static const AVOption cudascdet_options[] = {
    { "threshold",   "set scene change detect threshold",        OFFSET(threshold),  AV_OPT_TYPE_DOUBLE,   {.dbl = 10.},     0,  100., V|F },
    { "t",           "set scene change detect threshold",        OFFSET(threshold),  AV_OPT_TYPE_DOUBLE,   {.dbl = 10.},     0,  100., V|F },
    { "sc_pass",     "Set the flag to pass scene change frames", OFFSET(sc_pass),    AV_OPT_TYPE_BOOL,     {.dbl =  0  },    0,    1,  V|F },
    { "s",           "Set the flag to pass scene change frames", OFFSET(sc_pass),    AV_OPT_TYPE_BOOL,     {.dbl =  0  },    0,    1,  V|F },
    {NULL}
};

AVFILTER_DEFINE_CLASS(cudascdet);

static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_RGB24, AV_PIX_FMT_BGR24, AV_PIX_FMT_RGBA,
        AV_PIX_FMT_ABGR, AV_PIX_FMT_BGRA, AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUVJ422P,
        AV_PIX_FMT_YUV440P, AV_PIX_FMT_YUVJ440P,
        AV_PIX_FMT_YUV444P, AV_PIX_FMT_YUVJ444P,
        AV_PIX_FMT_YUV420P9, AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV420P12,
        AV_PIX_FMT_YUV422P9, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV422P12,
        AV_PIX_FMT_YUV444P9, AV_PIX_FMT_YUV444P10, AV_PIX_FMT_YUV444P12,
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
    AVHWFramesContext *in_frames_ctx;
    CUDASCDetContext *s = ctx->priv;
    FilterLink     *inl = ff_filter_link(ctx->inputs[0]);
    FilterLink    *outl = ff_filter_link(ctx->outputs[0]);

    /* check that we have a hw context */
    if (!inl->hw_frames_ctx) {
        av_log(ctx, AV_LOG_ERROR, "No hw context provided on input\n");
        return AVERROR(EINVAL);
    }
    in_frames_ctx = (AVHWFramesContext*)inl->hw_frames_ctx->data;

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

static av_cold int cuda_scdet_load_kernel(AVFilterContext *ctx)
{
    CUDASCDetContext *s = ctx->priv;
    CUcontext scdet, cuda_ctx = s->hwctx->cuda_ctx;
    CudaFunctions *cu = s->hwctx->internal->cuda_dl;
    int ret;

    extern const unsigned char ff_vf_scdet_cuda_ptx_data[];
    extern const unsigned int ff_vf_scdet_cuda_ptx_len;

    ret = CHECK_CU(cu->cuCtxPushCurrent(cuda_ctx));
    if (ret < 0)
        return ret;

    ret = ff_cuda_load_module(ctx, s->hwctx, &s->cu_module,
                              ff_vf_scdet_cuda_ptx_data, ff_vf_scdet_cuda_ptx_len);
    if (ret < 0)
        goto fail;

    if(s->bitdepth == 8) {
        ret = CHECK_CU(cu->cuModuleGetFunction(&s->cu_kernel, s->cu_module, "scdet_8"));
        if (ret < 0) {
            av_log(ctx, AV_LOG_FATAL, "Failed loading scdet_8\n");
            goto fail;
        }
    }
    else {
        ret = CHECK_CU(cu->cuModuleGetFunction(&s->cu_kernel, s->cu_module, "scdet_16"));
        if (ret < 0) {
            av_log(ctx, AV_LOG_FATAL, "Failed loading scdet_16\n");
            goto fail;
        }
    }

fail:
    CHECK_CU(cu->cuCtxPopCurrent(&scdet));

    return ret;
}

static int config_output(AVFilterLink *outlink)
{
    int ret = 0;
    AVFilterContext *ctx = outlink->src;
    AVFilterLink *inlink = outlink->src->inputs[0];
    FilterLink      *inl = ff_filter_link(inlink);
    CUDASCDetContext *s = ctx->priv;
    AVHWFramesContext     *frames_ctx = (AVHWFramesContext*)inl->hw_frames_ctx->data;
    int w,h;
    AVCUDADeviceContext *device_hwctx = frames_ctx->device_ctx->hwctx;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(frames_ctx->sw_format);

    int is_yuv = !(desc->flags & AV_PIX_FMT_FLAG_RGB) &&
        (desc->flags & AV_PIX_FMT_FLAG_PLANAR) &&
        desc->nb_components >= 3;

    s->bitdepth = desc->comp[0].depth;
    s->nb_planes = is_yuv ? 1 : av_pix_fmt_count_planes(frames_ctx->sw_format);

    for (int plane = 0; plane < 4; plane++) {
        ptrdiff_t line_size = av_image_get_linesize(frames_ctx->sw_format, inlink->w, plane);
        s->width[plane] = line_size >> (s->bitdepth > 8);
        s->height[plane] = inlink->h >> ((plane == 1 || plane == 2) ? desc->log2_chroma_h : 0);
    }

    s->hwctx = device_hwctx;
    s->cu_stream = s->hwctx->stream;

    w = ctx->inputs[0]->w;
    h = ctx->inputs[0]->h;
    outlink->w = w;
    outlink->h = h;

    ret = init_processing_chain(ctx);
    if (ret < 0)
        return ret;

    ret = cuda_scdet_load_kernel(ctx);
    if (ret < 0)
        return ret;

    return ret;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    CUDASCDetContext *s = ctx->priv;

    if (s->hwctx && s->cu_module) {
        CudaFunctions *cu = s->hwctx->internal->cuda_dl;
        CUcontext dummy;

        CHECK_CU(cu->cuCtxPushCurrent(s->hwctx->cuda_ctx));
        CHECK_CU(cu->cuModuleUnload(s->cu_module));
        s->cu_module = NULL;
        CHECK_CU(cu->cuCtxPopCurrent(&dummy));
    }

    av_buffer_unref(&s->frames_ctx);

    av_frame_free(&s->prev_picref);
}

static int create_texture_object(CudaFunctions *cu, CUtexObject* tex, unsigned int w, unsigned int h, unsigned int linesize, unsigned char elements_per_iteration, void* ptr) {
    CUDA_TEXTURE_DESC tex_desc = {
        .filterMode = CU_TR_FILTER_MODE_POINT,
        .flags = CU_TRSF_READ_AS_INTEGER,
        .addressMode = {CU_TR_ADDRESS_MODE_BORDER, CU_TR_ADDRESS_MODE_BORDER, CU_TR_ADDRESS_MODE_BORDER},
    };
    unsigned int *borderColor = (unsigned int *)(tex_desc.borderColor);
    CUdeviceptr data = (CUdeviceptr)ptr;

    CUDA_RESOURCE_DESC res_desc = {
        .resType = CU_RESOURCE_TYPE_PITCH2D,
        .res.pitch2D.format = CU_AD_FORMAT_UNSIGNED_INT16,
        .res.pitch2D.numChannels = 4,
        .res.pitch2D.width = w / elements_per_iteration,
        .res.pitch2D.height = h,
        .res.pitch2D.pitchInBytes = linesize,
        .res.pitch2D.devPtr = data
    };

    borderColor[0] = 0xffffffff;
    borderColor[1] = 0xffffffff;
    borderColor[2] = 0xffffffff;
    borderColor[3] = 0xffffffff;

    return cu->cuTexObjectCreate(tex, &res_desc, &tex_desc, NULL);
}

static int cuda_sad(AVFilterContext *ctx, void *data1, unsigned int linesize1, void *data2, unsigned int linesize2, unsigned int w, unsigned int h, uint64_t *plane_sad) {
    int ret;
    CUDASCDetContext *s = ctx->priv;
    CudaFunctions *cu = s->hwctx->internal->cuda_dl;
    CUcontext dummy, cuda_ctx = s->hwctx->cuda_ctx;
    CUtexObject tex1, tex2;
    int elements_per_iteration = s->bitdepth == 16 ? 4 : 8;

    int grid_x = DIV_UP(w / elements_per_iteration, BLOCKX);
    int grid_y = DIV_UP(h, BLOCKY);

    int blockCount = grid_x * grid_y;
    unsigned int* h_sums = NULL;
    CUdeviceptr d_sums;

    void *kernel_args[] = {
        &d_sums, &tex1, &tex2
    };
    uint64_t sad = 0;

    ret = CHECK_CU(cu->cuCtxPushCurrent(cuda_ctx));
    if (ret < 0)
        return ret;

    ret = CHECK_CU(cu->cuMemAlloc(&d_sums, blockCount * sizeof(unsigned int)));
    if (ret < 0)
        goto exit;

    ret = CHECK_CU(create_texture_object(cu, &tex1, w, h, linesize1, elements_per_iteration, data1));
    if (ret < 0)
        goto exit;

    ret = CHECK_CU(create_texture_object(cu, &tex2, w, h, linesize2, elements_per_iteration, data2));
    if (ret < 0)
        goto exit;
    
    ret = CHECK_CU(cu->cuLaunchKernel(s->cu_kernel,
                                      grid_x, grid_y, 1,
                                      BLOCKX, BLOCKY, 1, 32 * sizeof(int), s->cu_stream, kernel_args, NULL));
    if (ret < 0)
        goto exit;

    // copy result back to host
    h_sums = malloc(sizeof(int) * blockCount);
    if(!h_sums) {
        ret = AVERROR(ENOMEM);
        goto exit;
    }
    cu->cuStreamSynchronize(s->cu_stream);

    cu->cuMemcpyDtoH(h_sums, d_sums, blockCount * sizeof(int));

    // sum the sums
    for (int i = 0; i < blockCount; i++)
    {
        sad += h_sums[i];
    }
    *plane_sad = sad;

exit:
    if (tex1)
        CHECK_CU(cu->cuTexObjectDestroy(tex1));
    if (tex2)
        CHECK_CU(cu->cuTexObjectDestroy(tex2));

    if(d_sums)
        CHECK_CU(cu->cuMemFree(d_sums));

    if(h_sums)
        free(h_sums);

    CHECK_CU(cu->cuCtxPopCurrent(&dummy));
    return ret;
}

static double get_scene_score(AVFilterContext *ctx, AVFrame *frame)
{
    double ret = 0;
    CUDASCDetContext *s = ctx->priv;
    AVFrame *prev_picref = s->prev_picref;

    if (prev_picref && frame->height == prev_picref->height
                    && frame->width  == prev_picref->width) {
        uint64_t sad = 0;
        double mafd, diff;
        uint64_t count = 0;

        for (int plane = 0; plane < s->nb_planes; plane++) {
            uint64_t plane_sad;
            cuda_sad(ctx, prev_picref->data[plane], prev_picref->linesize[plane],
                    frame->data[plane], frame->linesize[plane],
                    s->width[plane], s->height[plane], &plane_sad);
            sad += plane_sad;
            count += s->width[plane] * s->height[plane];
        }

        mafd = (double)sad * 100. / count / (1ULL << s->bitdepth);
        diff = fabs(mafd - s->prev_mafd);
        ret  = av_clipf(FFMIN(mafd, diff), 0, 100.);
        s->prev_mafd = mafd;
        av_frame_free(&prev_picref);
    }
    s->prev_picref = av_frame_clone(frame);
    return ret;
}

static int set_meta(CUDASCDetContext *s, AVFrame *frame, const char *key, const char *value)
{
    return av_dict_set(&frame->metadata, key, value, 0);
}

static int activate(AVFilterContext *ctx)
{
    int ret;
    AVFilterLink *inlink = ctx->inputs[0];
    AVFilterLink *outlink = ctx->outputs[0];
    CUDASCDetContext *s = ctx->priv;
    AVFrame *frame;

    FF_FILTER_FORWARD_STATUS_BACK(outlink, inlink);

    ret = ff_inlink_consume_frame(inlink, &frame);
    if (ret < 0)
        return ret;

    if (frame) {
        char buf[64];
        s->scene_score = get_scene_score(ctx, frame);
        snprintf(buf, sizeof(buf), "%0.3f", s->prev_mafd);
        set_meta(s, frame, "lavfi.scd.mafd", buf);
        snprintf(buf, sizeof(buf), "%0.3f", s->scene_score);
        set_meta(s, frame, "lavfi.scd.score", buf);

        if (s->scene_score >= s->threshold) {
            av_log(s, AV_LOG_INFO, "lavfi.scd.score: %.3f, lavfi.scd.time: %s\n",
                    s->scene_score, av_ts2timestr(frame->pts, &inlink->time_base));
            set_meta(s, frame, "lavfi.scd.time",
                    av_ts2timestr(frame->pts, &inlink->time_base));
        }
        if (s->sc_pass) {
            if (s->scene_score >= s->threshold)
                return ff_filter_frame(outlink, frame);
            else {
                av_frame_free(&frame);
            }
        } else
            return ff_filter_frame(outlink, frame);
    }

    FF_FILTER_FORWARD_STATUS(inlink, outlink);
    FF_FILTER_FORWARD_WANTED(outlink, inlink);

    return FFERROR_NOT_READY;
}


static const AVFilterPad scdet_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .config_props = config_output,
    },
};

const FFFilter ff_vf_scdet_cuda = {
    .p.name          = "scdet_cuda",
    .p.description   = NULL_IF_CONFIG_SMALL("Detect video scene change (CUDA)."),
    .priv_size     = sizeof(CUDASCDetContext),
    .p.priv_class    = &cudascdet_class,
    .uninit        = uninit,
    .flags_internal         = FF_FILTER_FLAG_HWFRAME_AWARE,
    .p.flags = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(ff_video_default_filterpad),
    FILTER_OUTPUTS(scdet_outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_CUDA),
    .activate      = activate,
};

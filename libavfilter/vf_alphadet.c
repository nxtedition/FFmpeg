#include "libavutil/opt.h"
#include "libavutil/internal.h"
#include "filters.h"
#include "vf_alphadet.h"

static const AVOption alphadet_options[] = {
    { NULL }
};

AVFILTER_DEFINE_CLASS(alphadet);

static const char *type2str(Type type)
{
    switch(type) {
        case NONE         : return "none";
        case STRAIGHT     : return "straight";
        case UNDETERMINED : return "undetermined";
    }
    return NULL;
}

static int filter_frame(AVFilterLink *link, AVFrame *picref)
{
    AVFilterContext *ctx = link->dst;
    ALPHADETContext *alphadet = ctx->priv;

    if (!alphadet->csp) {
        alphadet->csp = av_pix_fmt_desc_get(link->format);
        if (alphadet->csp->nb_components < 4) {
            alphadet->type = NONE;
        }
    }

    if (alphadet->type == UNDETERMINED) {
        int l_stride = picref->linesize[0];
        int a_stride = picref->linesize[3];

        for (int y = 0; y < picref->height; y++) {
            for (int x = 0; x < picref->width; x++) {
                int l = picref->data[0][y * l_stride + x];
                int a = picref->data[3][y * a_stride + x];
                if (l > a) {
                    av_log(ctx, AV_LOG_INFO, "L: %d A: %d\n", l, a);
                    alphadet->type = STRAIGHT;
                    goto end;
                }
            }
        }
    }

end:
    return ff_filter_frame(ctx->outputs[0], picref);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ALPHADETContext *alphadet = ctx->priv;

    av_log(ctx, AV_LOG_INFO, "Alpha detection: %s\n", type2str(alphadet->type));
}

static const enum AVPixelFormat pix_fmts[] = {
    AV_PIX_FMT_YUV420P,
    AV_PIX_FMT_YUV422P,
    AV_PIX_FMT_YUV444P,
    AV_PIX_FMT_YUV410P,
    AV_PIX_FMT_YUV411P,
    AV_PIX_FMT_GRAY8,
    AV_PIX_FMT_YUVJ420P,
    AV_PIX_FMT_YUVJ422P,
    AV_PIX_FMT_YUVJ444P,
    AV_PIX_FMT_GRAY16,
    AV_PIX_FMT_YUV440P,
    AV_PIX_FMT_YUVJ440P,
    AV_PIX_FMT_YUV420P9,
    AV_PIX_FMT_YUV422P9,
    AV_PIX_FMT_YUV444P9,
    AV_PIX_FMT_YUV420P10,
    AV_PIX_FMT_YUV422P10,
    AV_PIX_FMT_YUV444P10,
    AV_PIX_FMT_YUV420P12,
    AV_PIX_FMT_YUV422P12,
    AV_PIX_FMT_YUV444P12,
    AV_PIX_FMT_YUV420P14,
    AV_PIX_FMT_YUV422P14,
    AV_PIX_FMT_YUV444P14,
    AV_PIX_FMT_YUV420P16,
    AV_PIX_FMT_YUV422P16,
    AV_PIX_FMT_YUV444P16,
    AV_PIX_FMT_YUVA420P,
    AV_PIX_FMT_YUVA422P,
    AV_PIX_FMT_YUVA444P,
    AV_PIX_FMT_NONE
};

static av_cold int init(AVFilterContext *ctx)
{
    ALPHADETContext *alphadet = ctx->priv;

    alphadet->type = UNDETERMINED;
    alphadet->csp = NULL;

    return 0;
}

static const AVFilterPad alphadet_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
    },
};

static const AVFilterPad alphadet_outputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
    },
};

const FFFilter ff_vf_alphadet = {
    .p.name          = "alphadet",
    .p.description   = NULL_IF_CONFIG_SMALL("Alpha detect Filter."),
    .priv_size     = sizeof(ALPHADETContext),
    .init          = init,
    .uninit        = uninit,
    .p.flags         = AVFILTER_FLAG_METADATA_ONLY,
    FILTER_INPUTS(alphadet_inputs),
    FILTER_OUTPUTS(alphadet_outputs),
    FILTER_PIXFMTS_ARRAY(pix_fmts),
    .p.priv_class    = &alphadet_class,
};

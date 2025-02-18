/*
 * Copyright 2025 (c) Niklas Haas
 *
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

#include <float.h>
#include "libavutil/vulkan_spirv.h"
#include "libavutil/opt.h"
#include "libavutil/timestamp.h"
#include "vulkan_filter.h"

#include "drawutils.h"
#include "filters.h"
#include "video.h"

enum AlphaType {
    UNDETERMINED = 0,
    STRAIGHT,
    NONE,
};

static const char *type2str(enum AlphaType type)
{
    switch(type) {
        case NONE         : return "none";
        case STRAIGHT     : return "straight";
        case UNDETERMINED : return "undetermined";
    }
    return NULL;
}

typedef struct AlphaDetectVulkanContext {
    FFVulkanContext vkctx;

    int initialized;
    FFVkExecPool e;
    AVVulkanDeviceQueueFamily *qf;
    FFVulkanShader shd;
    AVBufferPool *det_buf_pool;

    enum AlphaType type;
} AlphaDetectVulkanContext;

typedef struct AlphaDetectBuf {
    uint32_t frame_straight;
} AlphaDetectBuf;

static void fmt_swizzle(FFVulkanShader *shd, enum AVPixelFormat fmt)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(fmt);
    uint8_t map[4];
    char swiz[5] = {0};

    if (desc->flags & AV_PIX_FMT_FLAG_RGB) {
        ff_fill_rgba_map(map, fmt);
    } else if (desc->nb_components == 2) { /* ya */
        GLSLC(1, color.a = color.y;);
        return;
    } else {
        ff_fill_ayuv_map(map, fmt);
    }

    for (int i = 0; i < 4; i++)
        swiz[i] = "rgba"[map[i]];

    GLSLF(1, color = color.%s; ,swiz);
}

static av_cold int init_filter(AVFilterContext *ctx)
{
    int err;
    uint8_t *spv_data;
    size_t spv_len;
    void *spv_opaque = NULL;
    AlphaDetectVulkanContext *s = ctx->priv;
    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanShader *shd;
    FFVkSPIRVCompiler *spv;
    FFVulkanDescriptorSetBinding *desc;

    const int planes = av_pix_fmt_count_planes(s->vkctx.input_format);
    const AVPixFmtDescriptor *pixdesc = av_pix_fmt_desc_get(s->vkctx.input_format);
    if (!(pixdesc->flags & AV_PIX_FMT_FLAG_ALPHA)) { /* nothing to do */
        s->initialized = 1;
        s->type = NONE;
        return 0;
    }

    spv = ff_vk_spirv_init();
    if (!spv) {
        av_log(ctx, AV_LOG_ERROR, "Unable to initialize SPIR-V compiler!\n");
        return AVERROR_EXTERNAL;
    }

    s->qf = ff_vk_qf_find(vkctx, VK_QUEUE_COMPUTE_BIT, 0);
    if (!s->qf) {
        av_log(ctx, AV_LOG_ERROR, "Device has no compute queues\n");
        err = AVERROR(ENOTSUP);
        goto fail;
    }

    RET(ff_vk_exec_pool_init(vkctx, s->qf, &s->e, s->qf->num*4, 0, 0, 0, NULL));
    RET(ff_vk_shader_init(vkctx, &s->shd, "alphadetect",
                          VK_SHADER_STAGE_COMPUTE_BIT,
                          (const char *[]) { "GL_KHR_shader_subgroup_vote" }, 1,
                          32, 32, 1,
                          0));
    shd = &s->shd;

    desc = (FFVulkanDescriptorSetBinding []) {
        {
            .name       = "input_img",
            .type       = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
            .mem_layout = ff_vk_shader_rep_fmt(s->vkctx.input_format, FF_VK_REP_FLOAT),
            .mem_quali  = "readonly",
            .dimensions = 2,
            .elems      = av_pix_fmt_count_planes(s->vkctx.input_format),
            .stages     = VK_SHADER_STAGE_COMPUTE_BIT,
        }, {
            .name        = "det_buffer",
            .type        = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
            .stages      = VK_SHADER_STAGE_COMPUTE_BIT,
            .buf_content = "uint frame_straight;",
        }
    };

    RET(ff_vk_shader_add_descriptor_set(vkctx, &s->shd, desc, 2, 0, 0));

    GLSLC(0, void main()                                                      );
    GLSLC(0, {                                                                );
    GLSLC(1,     const ivec2 pos = ivec2(gl_GlobalInvocationID.xy);           );
    GLSLC(1,     if (!IS_WITHIN(pos, imageSize(input_img[0])))                );
    GLSLC(2,         return;                                                  );

    GLSLC(1, vec4 color = imageLoad(input_img[0],  pos);                      );
    for (int i = 1; i < planes; i++) {
        const int idx = planes == 2 ? 3 : i;
        if (!(pixdesc->flags & AV_PIX_FMT_FLAG_RGB) && idx != 3)
            continue; /* skip loading chroma */
        GLSLF(1, color[%d] = imageLoad(input_img[%d], pos).x;           ,idx,i);
    }
    fmt_swizzle(shd, s->vkctx.input_format);
    if (pixdesc->flags & AV_PIX_FMT_FLAG_RGB)
        GLSLC(1, bool straight = any(greaterThan(color.rgb, color.a));        );
    else
        GLSLC(1, bool straight = color.x > color.a;                           );
    GLSLC(1,     if (subgroupAny(straight) && subgroupElect())                );
    GLSLC(2,         frame_straight = 1u;                                     );
    GLSLC(0, }                                                                );

    RET(spv->compile_shader(vkctx, spv, &s->shd, &spv_data, &spv_len, "main",
                            &spv_opaque));
    RET(ff_vk_shader_link(vkctx, &s->shd, spv_data, spv_len, "main"));

    RET(ff_vk_shader_register_exec(vkctx, &s->e, &s->shd));

    s->initialized = 1;

fail:
    if (spv_opaque)
        spv->free_shader(spv, &spv_opaque);
    if (spv)
        spv->uninit(&spv);

    return err;
}

static int alphadetect_vulkan_filter_frame(AVFilterLink *link, AVFrame *in)
{
    int err;
    AVFilterContext *ctx = link->dst;
    AlphaDetectVulkanContext *s = ctx->priv;
    AVFilterLink *outlink = ctx->outputs[0];

    VkImageView in_views[AV_NUM_DATA_POINTERS];
    VkImageMemoryBarrier2 img_bar[4];
    int nb_img_bar = 0;

    FFVulkanContext *vkctx = &s->vkctx;
    FFVulkanFunctions *vk = &vkctx->vkfn;
    FFVkExecContext *exec = NULL;
    AVBufferRef *sum_buf = NULL;
    FFVkBuffer *sum_vk;

    AlphaDetectBuf *det;

    if (!s->initialized)
        RET(init_filter(ctx));

    if (s->type != UNDETERMINED)
        return ff_filter_frame(outlink, in);

    err = ff_vk_get_pooled_buffer(vkctx, &s->det_buf_pool, &sum_buf,
                                  VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                  NULL,
                                  sizeof(AlphaDetectBuf),
                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT |
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                  VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (err < 0)
        return err;
    sum_vk = (FFVkBuffer *)sum_buf->data;
    det = (AlphaDetectBuf *) sum_vk->mapped_mem;

    exec = ff_vk_exec_get(vkctx, &s->e);
    ff_vk_exec_start(vkctx, exec);

    RET(ff_vk_exec_add_dep_frame(vkctx, exec, in,
                                 VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                                 VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT));
    RET(ff_vk_create_imageviews(vkctx, exec, in_views, in, FF_VK_REP_FLOAT));

    ff_vk_shader_update_img_array(vkctx, exec, &s->shd, in, in_views, 0, 0,
                                  VK_IMAGE_LAYOUT_GENERAL, VK_NULL_HANDLE);

    ff_vk_frame_barrier(vkctx, exec, in, img_bar, &nb_img_bar,
                        VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
                        VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                        VK_ACCESS_SHADER_READ_BIT,
                        VK_IMAGE_LAYOUT_GENERAL,
                        VK_QUEUE_FAMILY_IGNORED);

    /* zero det buffer */
    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
            .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .pBufferMemoryBarriers = &(VkBufferMemoryBarrier2) {
                .sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2,
                .srcStageMask = VK_PIPELINE_STAGE_2_NONE,
                .dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                .dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT,
                .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .buffer = sum_vk->buf,
                .size = sum_vk->size,
                .offset = 0,
            },
            .bufferMemoryBarrierCount = 1,
        });

    vk->CmdFillBuffer(exec->buf, sum_vk->buf, 0, sum_vk->size, 0x0);

    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
            .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .pImageMemoryBarriers = img_bar,
            .imageMemoryBarrierCount = nb_img_bar,
            .pBufferMemoryBarriers = &(VkBufferMemoryBarrier2) {
                .sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2,
                .srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                .dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                .srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT,
                .dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                 VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .buffer = sum_vk->buf,
                .size = sum_vk->size,
                .offset = 0,
            },
            .bufferMemoryBarrierCount = 1,
        });

    RET(ff_vk_shader_update_desc_buffer(&s->vkctx, exec, &s->shd, 0, 1, 0,
                                        sum_vk, 0, sum_vk->size,
                                        VK_FORMAT_UNDEFINED));

    ff_vk_exec_bind_shader(vkctx, exec, &s->shd);

    vk->CmdDispatch(exec->buf,
                    FFALIGN(in->width,  s->shd.lg_size[0]) / s->shd.lg_size[0],
                    FFALIGN(in->height, s->shd.lg_size[1]) / s->shd.lg_size[1],
                    s->shd.lg_size[2]);

    vk->CmdPipelineBarrier2(exec->buf, &(VkDependencyInfo) {
            .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .pBufferMemoryBarriers = &(VkBufferMemoryBarrier2) {
                .sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2,
                .srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                .dstStageMask = VK_PIPELINE_STAGE_2_HOST_BIT,
                .srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                 VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                .dstAccessMask = VK_ACCESS_HOST_READ_BIT,
                .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
                .buffer = sum_vk->buf,
                .size = sum_vk->size,
                .offset = 0,
            },
            .bufferMemoryBarrierCount = 1,
        });

    RET(ff_vk_exec_submit(vkctx, exec));
    ff_vk_exec_wait(vkctx, exec);
    if (s->type == UNDETERMINED && det->frame_straight)
        s->type = STRAIGHT;

    av_buffer_unref(&sum_buf);
    return ff_filter_frame(outlink, in);

fail:
    if (exec)
        ff_vk_exec_discard_deps(&s->vkctx, exec);
    av_frame_free(&in);
    av_buffer_unref(&sum_buf);
    return err;
}

static void alphadetect_vulkan_uninit(AVFilterContext *avctx)
{
    AlphaDetectVulkanContext *s = avctx->priv;
    FFVulkanContext *vkctx = &s->vkctx;

    if (s->initialized)
        av_log(avctx, AV_LOG_INFO, "Alpha detection: %s\n", type2str(s->type));

    ff_vk_exec_pool_free(vkctx, &s->e);
    ff_vk_shader_free(vkctx, &s->shd);

    av_buffer_pool_uninit(&s->det_buf_pool);

    ff_vk_uninit(&s->vkctx);

    s->initialized = 0;
}

static const AVOption alphadetect_vulkan_options[] = {
    { NULL }
};

AVFILTER_DEFINE_CLASS(alphadetect_vulkan);

static const AVFilterPad alphadetect_vulkan_inputs[] = {
    {
        .name         = "default",
        .type         = AVMEDIA_TYPE_VIDEO,
        .filter_frame = &alphadetect_vulkan_filter_frame,
        .config_props = &ff_vk_filter_config_input,
    },
};

static const AVFilterPad alphadetect_vulkan_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
        .config_props = &ff_vk_filter_config_output,
    },
};

const FFFilter ff_vf_alphadetect_vulkan = {
    .p.name         = "alphadetect_vulkan",
    .p.description  = NULL_IF_CONFIG_SMALL("Detects if input is premultiplied or straight alpha."),
    .p.priv_class   = &alphadetect_vulkan_class,
    .p.flags        = AVFILTER_FLAG_HWDEVICE,
    .priv_size      = sizeof(AlphaDetectVulkanContext),
    .init           = &ff_vk_filter_init,
    .uninit         = &alphadetect_vulkan_uninit,
    FILTER_INPUTS(alphadetect_vulkan_inputs),
    FILTER_OUTPUTS(alphadetect_vulkan_outputs),
    FILTER_SINGLE_PIXFMT(AV_PIX_FMT_VULKAN),
    .flags_internal = FF_FILTER_FLAG_HWFRAME_AWARE,
};

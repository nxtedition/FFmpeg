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

#include "libavcodec/apv.h"
#include "libavcodec/bytestream.h"

#include "avformat.h"
#include "avio_internal.h"
#include "demux.h"
#include "internal.h"


typedef struct APVHeaderInfo {
    uint8_t  pbu_type;
    uint16_t group_id;

    uint8_t  profile_idc;
    uint8_t  level_idc;
    uint8_t  band_idc;

    int      frame_width;
    int      frame_height;

    uint8_t  chroma_format_idc;
    uint8_t  bit_depth_minus8;

    enum AVPixelFormat pixel_format;
} APVHeaderInfo;

static const enum AVPixelFormat apv_format_table[5][5] = {
    { AV_PIX_FMT_GRAY8,    AV_PIX_FMT_GRAY10,     AV_PIX_FMT_GRAY12,     AV_PIX_FMT_GRAY14, AV_PIX_FMT_GRAY16 },
    { 0 }, // 4:2:0 is not valid.
    { AV_PIX_FMT_YUV422P,  AV_PIX_FMT_YUV422P10,  AV_PIX_FMT_YUV422P12,  AV_PIX_FMT_GRAY14, AV_PIX_FMT_YUV422P16 },
    { AV_PIX_FMT_YUV444P,  AV_PIX_FMT_YUV444P10,  AV_PIX_FMT_YUV444P12,  AV_PIX_FMT_GRAY14, AV_PIX_FMT_YUV444P16 },
    { AV_PIX_FMT_YUVA444P, AV_PIX_FMT_YUVA444P10, AV_PIX_FMT_YUVA444P12, AV_PIX_FMT_GRAY14, AV_PIX_FMT_YUVA444P16 },
};

static int apv_extract_header_info(APVHeaderInfo *info,
                                   GetByteContext *gbc)
{
    int zero, byte, bit_depth_index;

    info->pbu_type = bytestream2_get_byte(gbc);
    info->group_id = bytestream2_get_be16(gbc);

    zero = bytestream2_get_byte(gbc);
    if (zero != 0)
        return AVERROR_INVALIDDATA;

    if (info->pbu_type != APV_PBU_PRIMARY_FRAME)
        return AVERROR_INVALIDDATA;

    info->profile_idc = bytestream2_get_byte(gbc);
    info->level_idc   = bytestream2_get_byte(gbc);

    byte = bytestream2_get_byte(gbc);
    info->band_idc = byte >> 3;
    zero = byte & 7;
    if (zero != 0)
        return AVERROR_INVALIDDATA;

    info->frame_width  = bytestream2_get_be24(gbc);
    info->frame_height = bytestream2_get_be24(gbc);
    if (info->frame_width  < 1 || info->frame_width  > 65536 ||
        info->frame_height < 1 || info->frame_height > 65536)
        return AVERROR_INVALIDDATA;

    byte = bytestream2_get_byte(gbc);
    info->chroma_format_idc = byte >> 4;
    info->bit_depth_minus8  = byte & 0xf;

    if (info->bit_depth_minus8 > 8) {
        return AVERROR_INVALIDDATA;
    }
    if (info->bit_depth_minus8 % 2) {
        // Odd bit depths are technically valid but not useful here.
        return AVERROR_INVALIDDATA;
    }
    bit_depth_index = info->bit_depth_minus8 / 2;

    switch (info->chroma_format_idc) {
    case APV_CHROMA_FORMAT_400:
    case APV_CHROMA_FORMAT_422:
    case APV_CHROMA_FORMAT_444:
    case APV_CHROMA_FORMAT_4444:
        info->pixel_format = apv_format_table[info->chroma_format_idc][bit_depth_index];
        break;
    default:
        return AVERROR_INVALIDDATA;
    }

    // Ignore capture_time_distance.
    bytestream2_skip(gbc, 1);

    zero = bytestream2_get_byte(gbc);
    if (zero != 0)
        return AVERROR_INVALIDDATA;

    return 1;
}

static int apv_probe(const AVProbeData *p)
{
    GetByteContext gbc;
    APVHeaderInfo header;
    uint32_t au_size, signature, pbu_size;
    int err;

    if (p->buf_size < 28) {
        // Too small to fit an APV header.
        return 0;
    }

    bytestream2_init(&gbc, p->buf, p->buf_size);

    au_size = bytestream2_get_be32(&gbc);
    if (au_size < 24) {
        // Too small.
        return 0;
    }
    signature = bytestream2_get_be32(&gbc);
    if (signature != APV_SIGNATURE) {
        // Signature is mandatory.
        return 0;
    }
    pbu_size = bytestream2_get_be32(&gbc);
    if (pbu_size < 16) {
        // Too small.
        return 0;
    }

    err = apv_extract_header_info(&header, &gbc);
    if (err < 0) {
        // Header does not look like APV.
        return 0;
    }
    return AVPROBE_SCORE_MAX;
}

static int apv_read_header(AVFormatContext *s)
{
    AVStream *st;
    GetByteContext gbc;
    APVHeaderInfo header;
    uint8_t buffer[28];
    uint32_t au_size, signature, pbu_size;
    int err, size;

    err = ffio_ensure_seekback(s->pb, sizeof(buffer));
    if (err < 0)
        return err;
    size = avio_read(s->pb, buffer, sizeof(buffer));
    if (size < 0)
        return size;

    bytestream2_init(&gbc, buffer, sizeof(buffer));

    au_size = bytestream2_get_be32(&gbc);
    if (au_size < 24) {
        // Too small.
        return AVERROR_INVALIDDATA;
    }
    signature = bytestream2_get_be32(&gbc);
    if (signature != APV_SIGNATURE) {
        // Signature is mandatory.
        return AVERROR_INVALIDDATA;
    }
    pbu_size = bytestream2_get_be32(&gbc);
    if (pbu_size < 16) {
        // Too small.
        return AVERROR_INVALIDDATA;
    }

    err = apv_extract_header_info(&header, &gbc);
    if (err < 0)
        return err;

    st = avformat_new_stream(s, NULL);
    if (!st)
        return AVERROR(ENOMEM);

    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->codec_id   = AV_CODEC_ID_APV;
    st->codecpar->format     = header.pixel_format;
    st->codecpar->profile    = header.profile_idc;
    st->codecpar->level      = header.level_idc;
    st->codecpar->width      = header.frame_width;
    st->codecpar->height     = header.frame_height;

    st->avg_frame_rate = (AVRational){ 30, 1 };
    avpriv_set_pts_info(st, 64, 1, 30);

    avio_seek(s->pb, -size, SEEK_CUR);

    return 0;
}

static int apv_read_packet(AVFormatContext *s, AVPacket *pkt)
{
    uint32_t au_size, signature;
    int ret;

    au_size = avio_rb32(s->pb);
    if (au_size == 0 && avio_feof(s->pb))
        return AVERROR_EOF;
    if (au_size < 24 || au_size > 1 << 24) {
        av_log(s, AV_LOG_ERROR,
               "APV AU has invalid size: %"PRIu32"\n", au_size);
        return AVERROR_INVALIDDATA;
    }

    ret = av_get_packet(s->pb, pkt, au_size);
    pkt->flags        = AV_PKT_FLAG_KEY;

    signature = AV_RB32(pkt->data);
    if (signature != APV_SIGNATURE) {
        av_log(s, AV_LOG_ERROR, "APV AU has invalid signature.\n");
        return AVERROR_INVALIDDATA;
    }

    return ret;
}

const FFInputFormat ff_apv_demuxer = {
    .p.name         = "apv",
    .p.long_name    = NULL_IF_CONFIG_SMALL("APV raw bitstream"),
    .p.extensions   = "apv",
    .p.flags        = AVFMT_GENERIC_INDEX | AVFMT_NOTIMESTAMPS,
    .flags_internal = FF_INFMT_FLAG_INIT_CLEANUP,
    .read_probe     = apv_probe,
    .read_header    = apv_read_header,
    .read_packet    = apv_read_packet,
};

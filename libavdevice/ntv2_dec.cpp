#include "ntv2_dec.h"

extern "C" {
#include "libavformat/avformat.h"
#include "libavformat/internal.h"
#include "libavutil/avassert.h"
#include "avdevice.h"
}

#include <stdlib.h>

#define AJALinux
#define AJA_LINUX
#include <ajabase/system/thread.h>
#include <ajabase/system/memory.h>
#include <ajabase/system/systemtime.h>
#include <ajabase/common/types.h>
#include <ntv2card.h>
#include <ntv2utils.h>
#include <ntv2devicefeatures.h>
#include <ntv2publicinterface.h>

#define AJA_AUDIO_TIME_BASE_Q {1,10000000}

// from libavcodec/avpacket.c
int ff_side_data_set_prft(AVPacket *pkt, int64_t timestamp)
{
    AVProducerReferenceTime *prft;
    uint8_t *side_data;
    int side_data_size;

    side_data = av_packet_get_side_data(pkt, AV_PKT_DATA_PRFT, &side_data_size);
    if (!side_data) {
        side_data_size = sizeof(AVProducerReferenceTime);
        side_data = av_packet_new_side_data(pkt, AV_PKT_DATA_PRFT, side_data_size);
    }

    if (!side_data || side_data_size < sizeof(AVProducerReferenceTime))
        return AVERROR(ENOMEM);

    prft = (AVProducerReferenceTime *)side_data;
    prft->wallclock = timestamp;
    prft->flags = 0;

    return 0;
}

static void avpacket_queue_init(AVPacketQueue *q)
{
    memset(q, 0, sizeof(AVPacketQueue));
    av_assert0(pthread_mutex_init(&q->mutex, NULL) == 0);
    av_assert0(pthread_cond_init(&q->cond, NULL) == 0);
}

static void avpacket_queue_flush(AVPacketQueue *q)
{
    AVPacketList *pkt, *pkt1;

    av_assert0(pthread_mutex_lock(&q->mutex) == 0);
    for (pkt = q->first_pkt; pkt != NULL; pkt = pkt1) {
        pkt1 = pkt->next;
        av_packet_unref(&pkt->pkt);
        av_freep(&pkt);
    }
    q->last_pkt   = NULL;
    q->first_pkt  = NULL;
    q->size       = 0;
    q->eof        = 1;

    av_assert0(pthread_cond_signal(&q->cond) == 0);

    av_assert0(pthread_mutex_unlock(&q->mutex) == 0);
}

static void avpacket_queue_close(AVPacketQueue *q)
{
    avpacket_queue_flush(q);
    av_assert0(pthread_mutex_destroy(&q->mutex) == 0);
    av_assert0(pthread_cond_destroy(&q->cond) == 0);
}

static int avpacket_queue_size(AVPacketQueue *q)
{
    int size;
    av_assert0(pthread_mutex_lock(&q->mutex) == 0);
    size = q->size;
    av_assert0(pthread_mutex_unlock(&q->mutex) == 0);
    return size;
}

static int avpacket_queue_put(AVPacketQueue *q, AVPacket *pkt)
{
    if (!pkt->buf || pkt->size < 64 || !pkt->data) {
        return AVERROR(EINVAL);
    }

    if (av_packet_make_refcounted(pkt) < 0) {
        av_packet_unref(pkt);
        return AVERROR(EINVAL);
    }

    auto pkt1 = (AVPacketList *)av_malloc(sizeof(AVPacketList));
    if (!pkt1) {
        av_packet_unref(pkt);
        return AVERROR(ENOMEM);
    }
    av_packet_move_ref(&pkt1->pkt, pkt);
    pkt1->next = NULL;

    av_assert0(pthread_mutex_lock(&q->mutex) == 0);

    if (!q->last_pkt) {
        q->first_pkt = pkt1;
    } else {
        q->last_pkt->next = pkt1;
    }

    q->last_pkt = pkt1;
    q->size++;

    av_assert0(pthread_cond_signal(&q->cond) == 0);

    av_assert0(pthread_mutex_unlock(&q->mutex) == 0);
    return 0;
}

static int avpacket_queue_get(AVPacketQueue *q, AVPacket *pkt)
{
    AVPacketList *pkt1;
    int ret;

    if (q->eof) {
        return AVERROR_EOF;
    }

    av_assert0(pthread_mutex_lock(&q->mutex) == 0);

    for (;; ) {
        pkt1 = q->first_pkt;
        if (pkt1) {
            q->first_pkt = pkt1->next;
            if (!q->first_pkt) {
                q->last_pkt = NULL;
            }
            q->size--;
            *pkt = pkt1->pkt;
            av_free(pkt1);
            ret = 1;
            break;
        } else if (q->eof) {
            ret = -1;
            break;
        } else {
            av_assert0(pthread_cond_wait(&q->cond, &q->mutex) == 0);
        }
    }
    av_assert0(pthread_mutex_unlock(&q->mutex) == 0);
    return ret;
}

static int setup_video(AVFormatContext *avctx, NTV2Context *ctx)
{
    const auto device = reinterpret_cast<CNTV2Card*>(ctx->device);
    const auto channel = ::NTV2InputSourceToChannel(static_cast<NTV2InputSource>(ctx->input_source));
    const auto input_source = static_cast<NTV2InputSource>(ctx->input_source);
    const auto video_format = static_cast<NTV2VideoFormat>(ctx->video_format);
    const auto pixel_format = NTV2_FBF_8BIT_YCBCR;
    const auto frame_rate = ::GetNTV2FrameRateFromVideoFormat(video_format);

    av_assert0(channel < NTV2_CHANNEL_INVALID);
    av_assert0(input_source < NTV2_INPUTSOURCE_INVALID);
    av_assert0(video_format > NTV2_FORMAT_UNKNOWN && video_format < NTV2_MAX_NUM_VIDEO_FORMATS);
    av_assert0(pixel_format > NTV2_FBF_FIRST && pixel_format < NTV2_FBF_INVALID);
    av_assert0(frame_rate > NTV2_FRAMERATE_UNKNOWN);

    av_assert0(NTV2_IS_HD_VIDEO_FORMAT(video_format));
    av_assert0(NTV2_INPUT_SOURCE_IS_SDI(input_source));

    if (device->IsDeviceReady(true)) {
        av_log(avctx, AV_LOG_DEBUG, "IsDeviceReady %i\n", 1);
    } else {
        av_log(avctx, AV_LOG_ERROR, "IsDeviceReady %i\n", 1);
        return AJA_STATUS_FAIL;
    }

    if (device->EnableChannel(channel)) {
        av_log(avctx, AV_LOG_DEBUG, "EnableChannel %i\n", channel);
    } else {
        av_log(avctx, AV_LOG_ERROR, "EnableChannel %i\n", channel);
        return AJA_STATUS_FAIL;
    }

	if (::NTV2DeviceCanDoMultiFormat(device->GetDeviceID())) {
        if (device->SetMultiFormatMode(true)) {
            av_log(avctx, AV_LOG_DEBUG, "SetMultiFormatMode %i\n", 0);
        } else {
            av_log(avctx, AV_LOG_ERROR, "SetMultiFormatMode %i\n", 0);
            return AJA_STATUS_FAIL;
        }
    }

    if (device->EnableInputInterrupt(channel)) {
        av_log(avctx, AV_LOG_DEBUG, "EnableInputInterrupt %i\n", channel);
    } else {
        av_log(avctx, AV_LOG_ERROR, "EnableInputInterrupt %i\n", channel);
        return AJA_STATUS_FAIL;
    }

    if (device->SubscribeInputVerticalEvent(channel)) {
        av_log(avctx, AV_LOG_DEBUG, "SubscribeInputVerticalEvent %i\n", channel);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SubscribeInputVerticalEvent %i\n", channel);
        return AJA_STATUS_FAIL;
    }

	// The input vertical is not always available so we like to use the output for timing.
    if (device->SubscribeOutputVerticalEvent(channel)) {
        av_log(avctx, AV_LOG_DEBUG, "SubscribeOutputVerticalEvent %i\n", channel);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SubscribeOutputVerticalEvent %i\n", channel);
        return AJA_STATUS_FAIL;
    }

    if (device->SetSDITransmitEnable(channel, false)) {
        av_log(avctx, AV_LOG_DEBUG, "SetSDITransmitEnable %i\n", channel);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SetSDITransmitEnable %i\n", channel);
        return AJA_STATUS_FAIL;
    }

	// Wait to let the reciever lock...
    if (device->WaitForOutputVerticalInterrupt(channel, 10)) {
        av_log(avctx, AV_LOG_DEBUG, "WaitForOutputVerticalInterrupt %i\n", channel);
    } else {
        av_log(avctx, AV_LOG_ERROR, "WaitForOutputVerticalInterrupt %i\n", channel);
        return AJA_STATUS_FAIL;
    }

    if (!::NTV2DeviceCanDoFrameBufferFormat(device->GetDeviceID(), pixel_format)) {
        av_log(avctx, AV_LOG_ERROR, "NTV2DeviceCanDoFrameBufferFormat %i %i\n", device->GetDeviceID(), pixel_format);
        return AJA_STATUS_FAIL;
    }

    if (device->SetVideoFormat(video_format, false, false, channel)) {
        av_log(avctx, AV_LOG_DEBUG, "SetVideoFormat %i %i\n", channel, video_format);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SetVideoFormat %i %i\n", channel, video_format);
        return AJA_STATUS_FAIL;
    }

    // TODO: IsInput3Gb?
    if (device->SetSDIInLevelBtoLevelAConversion(channel, video_format == NTV2_FORMAT_1080p_5000_A)) {
        av_log(avctx, AV_LOG_DEBUG, "SetSDIInLevelBtoLevelAConversion %i %i\n", channel, video_format == NTV2_FORMAT_1080p_5000_A);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SetSDIInLevelBtoLevelAConversion %i %i\n", channel, video_format == NTV2_FORMAT_1080p_5000_A);
        return AJA_STATUS_FAIL;
    }

    if (device->SetFrameBufferFormat(channel, pixel_format)) {
        av_log(avctx, AV_LOG_DEBUG, "SetFrameBufferFormat %i %i\n", channel, pixel_format);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SetFrameBufferFormat %i %i\n", channel, pixel_format);
        return AJA_STATUS_FAIL;
    }

    if (device->Connect(::GetFrameBufferInputXptFromChannel(channel), ::GetSDIInputOutputXptFromChannel(channel))) {
        av_log(avctx, AV_LOG_DEBUG, "Connect %i\n", channel);
    } else {
        av_log(avctx, AV_LOG_ERROR, "Connect %i\n", channel);
        return AJA_STATUS_FAIL;
    }

    const auto width = ::GetDisplayWidth(video_format);
    const auto height = ::GetDisplayHeight(video_format);

    av_assert0(width > 1);
    av_assert0(height > 1);

    ULWord tb_num;
    ULWord tb_den;

    if (!::GetFramesPerSecond(frame_rate, tb_den, tb_num)) {
        av_log(avctx, AV_LOG_ERROR, "GetFramesPerSecond\n");
        return AJA_STATUS_FAIL;
    }

    av_assert0(tb_num > 0);
    av_assert0(tb_den > 0);

    av_log(avctx, AV_LOG_DEBUG, "NTV2 Video:\n  input_source=%s (%i)\n  video_format=%s (%i)\n  pixel_format=%s (%i)\n  frame_rate=%s %i/%i (%i)\n  width=%i \n  height=%i\n",
        ::NTV2InputSourceToString(input_source).c_str(), input_source,
        ::NTV2VideoFormatToString(video_format).c_str(), video_format,
        ::NTV2FrameBufferFormatToString(pixel_format).c_str(), pixel_format,
        ::NTV2FrameRateToString(frame_rate).c_str(), tb_den, tb_num, frame_rate,
        width,
        height);

    auto st = avformat_new_stream(avctx, nullptr);
    if (!st) {
        av_log(avctx, AV_LOG_ERROR, "avformat_new_stream\n");
        return AVERROR(ENOMEM);
    }

    st->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
    st->codecpar->sample_aspect_ratio = { 1, 1 };
    st->codecpar->color_range = AVCOL_RANGE_MPEG;
    st->codecpar->color_primaries = AVCOL_PRI_BT709;
    st->codecpar->color_trc = AVCOL_TRC_BT709;
    st->codecpar->color_space = AVCOL_SPC_BT709;
    st->codecpar->chroma_location = AVCHROMA_LOC_UNSPECIFIED; // TODO
    st->codecpar->width = width;
    st->codecpar->height = height;
    st->codecpar->codec_id = AV_CODEC_ID_RAWVIDEO;
    st->codecpar->codec_tag = MKTAG('U', 'Y', 'V', 'Y');
    st->codecpar->format = AV_PIX_FMT_UYVY422;
    st->codecpar->bit_rate = av_rescale(width * height * 16, tb_den, tb_num);
    st->codecpar->field_order = ::IsProgressivePicture(video_format) ? AV_FIELD_PROGRESSIVE : AV_FIELD_TT;

    st->r_frame_rate = av_make_q(tb_den, tb_num);
    avpriv_set_pts_info(st, 64, 1, 48000);

    av_assert0(av_q2d(st->r_frame_rate) >= 24 && av_q2d(st->r_frame_rate) < 120);
    av_assert0(st->codecpar->width > 0);
    av_assert0(st->codecpar->height > 0);
    av_assert0(st->codecpar->bit_rate > 0);

    ctx->video_st = st;

    return 0;
}

static int setup_audio(AVFormatContext *avctx, NTV2Context *ctx)
{
    const auto device = reinterpret_cast<CNTV2Card*>(ctx->device);
    const auto channel = ::NTV2InputSourceToChannel(static_cast<NTV2InputSource>(ctx->input_source));
    const auto audio_source = static_cast<NTV2AudioSource>(ctx->audio_source);

    const auto audio_system = ::NTV2ChannelToAudioSystem(channel);
    const auto audio_input = ::NTV2ChannelToEmbeddedAudioInput(channel);
    const auto audio_channels = FFMIN(8, ::NTV2DeviceGetMaxAudioChannels(device->GetDeviceID()));

    av_assert0(channel < NTV2_CHANNEL_INVALID);
    av_assert0(audio_source < NTV2_AUDIO_SOURCE_INVALID);
    av_assert0(audio_input < NTV2_EMBEDDED_AUDIO_INPUT_INVALID);
    av_assert0(audio_system < NTV2_AUDIOSYSTEM_INVALID);
    av_assert0(audio_channels > 0 && audio_channels <= 16);

    if (device->SetAudioSystemInputSource(audio_system, audio_source, audio_input)) {
        av_log(avctx, AV_LOG_DEBUG, "SetAudioSystemInputSource %i %i %i\n", audio_system, audio_source, audio_input);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SetAudioSystemInputSource %i %i %i\n", audio_system, audio_source, audio_input);
        return AJA_STATUS_FAIL;
    }

    if (device->SetNumberAudioChannels(audio_channels, audio_system)) {
        av_log(avctx, AV_LOG_DEBUG, "SetNumberAudioChannels %i %i\n", audio_channels, audio_system);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SetNumberAudioChannels %i %i\n", audio_channels, audio_system);
        return AJA_STATUS_FAIL;
    }

    if (device->SetAudioRate(NTV2_AUDIO_48K, audio_system)) {
        av_log(avctx, AV_LOG_DEBUG, "SetAudioRate %i %i\n", NTV2_AUDIO_48K, audio_system);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SetAudioRate %i %i\n", NTV2_AUDIO_48K, audio_system);
        return AJA_STATUS_FAIL;
    }

    if (device->SetAudioBufferSize(NTV2_AUDIO_BUFFER_BIG, audio_system)) {
        av_log(avctx, AV_LOG_DEBUG, "SetAudioBufferSize %i %i\n", NTV2_AUDIO_BUFFER_BIG, audio_system);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SetAudioBufferSize %i %i\n", NTV2_AUDIO_BUFFER_BIG, audio_system);
        return AJA_STATUS_FAIL;
    }

    if (device->SetAudioLoopBack(NTV2_AUDIO_LOOPBACK_OFF, audio_system)) {
        av_log(avctx, AV_LOG_DEBUG, "SetAudioLoopBack %i %i\n", NTV2_AUDIO_LOOPBACK_OFF, audio_system);
    } else {
        av_log(avctx, AV_LOG_ERROR, "SetAudioLoopBack %i %i\n", NTV2_AUDIO_LOOPBACK_OFF, audio_system);
        return AJA_STATUS_FAIL;
    }

    av_log(avctx, AV_LOG_DEBUG, "NTV2 Audio:\n  audio_source=%s (%i)\n  audio_system=%s (%i)\n  audio_input=%s (%i)\n",
        ::NTV2AudioSourceToString(audio_source).c_str(), audio_source,
        ::NTV2AudioSystemToString(audio_system).c_str(), audio_system,
        ::NTV2EmbeddedAudioInputToString(audio_input).c_str(), audio_input);

    auto st = avformat_new_stream(avctx, nullptr);
    if (!st) {
        av_log(avctx, AV_LOG_ERROR, "avformat_new_stream\n");
        return AVERROR(ENOMEM);
    }

    st->codecpar->codec_type = AVMEDIA_TYPE_AUDIO;
    st->codecpar->codec_id = AV_CODEC_ID_PCM_S32LE;
    st->codecpar->format = AV_SAMPLE_FMT_S32;
    st->codecpar->channels = audio_channels;
    st->codecpar->channel_layout = av_get_default_channel_layout(st->codecpar->channels);
    st->codecpar->sample_rate = 48000;
    avpriv_set_pts_info(st, 64, 1, 48000);

    ctx->audio_st = st;

    return 0;
}

static void aja_free(void *opaque, uint8_t *data)
{
    AJAMemory::Free(data);
}

static AVBufferRef* aja_pool_alloc(void *opaque, int size)
{
    const auto device = reinterpret_cast<CNTV2Card*>(opaque);

    auto data = AJAMemory::AllocateAligned(size, 4096);

    if (!data) {
        return NULL;
    }

    device->DMABufferLock(reinterpret_cast<ULWord*>(data), size);

    return av_buffer_create(reinterpret_cast<uint8_t*>(data), size, aja_free, NULL, 0);
}

static void capture_thread(AJAThread *thread, void *opaque)
{
    // TODO: try/catch
    // TODO: remove runtime asserts.
    // TODO: auto restart on failure

    const auto avctx = reinterpret_cast<AVFormatContext*>(opaque);
    const auto ctx = reinterpret_cast<NTV2Context*>(avctx->priv_data);
    const auto device = reinterpret_cast<CNTV2Card*>(ctx->device);
    const auto channel = ::NTV2InputSourceToChannel(static_cast<NTV2InputSource>(ctx->input_source));
    const auto video_format = static_cast<NTV2VideoFormat>(ctx->video_format);

    int ret;

    av_assert0(channel < NTV2_CHANNEL_INVALID);
    av_assert0(video_format > NTV2_FORMAT_UNKNOWN && video_format < NTV2_MAX_NUM_VIDEO_FORMATS);

    // TODO: Assumes resolution <= 1920x1080 && format <= 8bit ycbcr.
    const auto num_frame_buffers = ::NTV2DeviceGetNumberFrameBuffers(device->GetDeviceID(), NTV2_FG_4x1920x1080, NTV2_FBF_8BIT_YCBCR);
    const auto num_frames_stores = ::NTV2DeviceGetNumFrameStores(device->GetDeviceID());
    const auto buffer_count =  num_frame_buffers / num_frames_stores;
    av_assert0(buffer_count > 1);

    // Setup auto circulate.
    {
        av_log(avctx, AV_LOG_DEBUG, "AutoCirculateInitForInput channel=%i audio_system=%i buffer_count=%i [%i-%i]\n",
            channel,
            ::NTV2ChannelToAudioSystem(channel),
            buffer_count,
            buffer_count * channel,
            buffer_count * (channel + 1) - 1);

        av_assert0(device->AutoCirculateStop(channel));
        av_assert0(device->AutoCirculateInitForInput(
            channel,
            0,
            ::NTV2ChannelToAudioSystem(channel),
            AUTOCIRCULATE_WITH_RP188,
            1,
            buffer_count * channel,
            buffer_count * (channel + 1) - 1
        ));
        av_assert0(device->AutoCirculateStart(channel));
    }

    AUTOCIRCULATE_TRANSFER transfer;
    av_assert0(device->GetFrameBufferFormat(channel, transfer.acFrameBufferFormat));

    const auto video_size = ::GetVideoActiveSize(video_format, transfer.acFrameBufferFormat, NTV2_VANCMODE_OFF);
    const auto video_codec = ctx->video_st->codecpar;
    const auto pixel_format = static_cast<AVPixelFormat>(video_codec->format);
    auto video_pool = av_buffer_pool_init2(video_size, device, aja_pool_alloc, NULL);

    const auto audio_size = 8192 * 16 * 4;
    const auto audio_codec = ctx->audio_st->codecpar;
    const auto sample_format = static_cast<AVSampleFormat>(audio_codec->format);
    auto audio_pool = av_buffer_pool_init2(audio_size, device, aja_pool_alloc, NULL);

    auto video_pts = AV_NOPTS_VALUE;
    auto audio_pts = AV_NOPTS_VALUE;

    auto signal_debounce = buffer_count;
    ULWord last_frames_dropped = 0;

    while (!ctx->quit) {
        AUTOCIRCULATE_STATUS status;
        av_assert0(device->AutoCirculateGetStatus(channel, status));

        if (last_frames_dropped != status.acFramesDropped) {
            av_log(avctx, AV_LOG_ERROR, "ac frames dropped: %u %u\n", status.acFramesDropped - last_frames_dropped, status.acFramesDropped);
            last_frames_dropped = status.acFramesDropped;
        }

        // TODO (fix): Is this the correct way to detect signal?
        const auto has_video_signal = device->GetInputVideoFormat(static_cast<NTV2InputSource>(ctx->input_source)) == video_format;
        {
            if (has_video_signal) {
                if (signal_debounce == 1) {
                    av_log(avctx, AV_LOG_INFO, "signal connected\n");
                }
                if (signal_debounce > 0) {
                    signal_debounce--;
                }
            } else {
                if (signal_debounce == 0) {
                    av_log(avctx, AV_LOG_WARNING, "signal disconnected\n");
                }
                signal_debounce = buffer_count;
            }
        }

        AVPacket video_pkt;
        av_init_packet(&video_pkt);
        video_pkt.flags |= AV_PKT_FLAG_KEY;
        video_pkt.stream_index = ctx->video_st->index;

        AVPacket audio_pkt;
        av_init_packet(&audio_pkt);
        audio_pkt.flags |= AV_PKT_FLAG_KEY;
        audio_pkt.stream_index = ctx->audio_st->index;

        if (status.IsRunning() && status.HasAvailableInputFrame()) {
            video_pkt.buf = av_buffer_pool_get(video_pool);
            video_pkt.data = video_pkt.buf->data;
            video_pkt.size = video_pkt.buf->size;

            audio_pkt.buf = av_buffer_pool_get(audio_pool);
            audio_pkt.data = audio_pkt.buf->data;
            audio_pkt.size = audio_pkt.buf->size;

            av_assert0(transfer.SetVideoBuffer(reinterpret_cast<ULWord*>(video_pkt.buf->data), static_cast<ULWord>(video_pkt.buf->size)));
            av_assert0(transfer.SetAudioBuffer(reinterpret_cast<ULWord*>(audio_pkt.buf->data), static_cast<ULWord>(audio_pkt.buf->size)));
            av_assert0(device->AutoCirculateTransfer(channel, transfer));

            const auto frameInfo = transfer.GetFrameInfo();
            const auto audioTime = frameInfo.acAudioClockCurrentTime - status.acAudioClockStartTime;

            video_pkt.pts = av_rescale_q(audioTime, AJA_AUDIO_TIME_BASE_Q, ctx->video_st->time_base);
            video_pkt.dts = video_pkt.pts;
            video_pkt.duration = ctx->video_st->time_base.num;

            video_pts = av_rescale_q(video_pkt.pts + video_pkt.duration, ctx->video_st->time_base, AJA_AUDIO_TIME_BASE_Q);

            audio_pkt.pts = av_rescale_q(audioTime, AJA_AUDIO_TIME_BASE_Q, ctx->audio_st->time_base);
            audio_pkt.dts = audio_pkt.pts;
            av_shrink_packet(&audio_pkt, transfer.GetCapturedAudioByteCount());
            audio_pkt.duration = audio_pkt.size / (audio_codec->channels * av_get_bytes_per_sample(sample_format));
            av_assert0(ctx->audio_st->time_base.num == 1 && ctx->audio_st->time_base.den == audio_codec->sample_rate);

            audio_pts = av_rescale_q(audio_pkt.pts + audio_pkt.duration, ctx->audio_st->time_base, AJA_AUDIO_TIME_BASE_Q);

            // set producer reference time side data
            ff_side_data_set_prft(&video_pkt, frameInfo.acFrameTime / 10);

            av_log(avctx, AV_LOG_TRACE, "video_pts=%li video_dur=%li audio_pts=%li audio_dur=%li\n",
                video_pkt.pts,
                video_pkt.duration,
                audio_pkt.pts,
                audio_pkt.duration);

            if (avpacket_queue_size(&ctx->queue) > ctx->queue_size) {
                ctx->dropped += 1;
                av_log(avctx, AV_LOG_WARNING, "input frame dropped: dropped=%i video_pts=%li video_dur=%li audio_pts=%li audio_dur=%li\n",
                    ctx->dropped,
                    video_pkt.pts,
                    video_pkt.duration,
                    audio_pkt.pts,
                    audio_pkt.duration);
            } else {
                ret = avpacket_queue_put(&ctx->queue, &video_pkt);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_ERROR, "invalid video packet pkt_pts=%li pkt_size=%i\n",
                        video_pkt.pts,
                        video_pkt.size);
                }

                ret = avpacket_queue_put(&ctx->queue, &audio_pkt);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_ERROR, "invalid audio packet pkt_pts=%li pkt_size=%i\n",
                        audio_pkt.pts,
                        audio_pkt.size);
                }
            }
        } else {
            const auto audioTime = status.acAudioClockCurrentTime - status.acAudioClockStartTime;

            const auto diff = video_pts == AV_NOPTS_VALUE || audio_pts == AV_NOPTS_VALUE
                ? 0
                : av_rescale_q(FFMAX(audioTime - video_pts, audioTime - audio_pts), AJA_AUDIO_TIME_BASE_Q, {1,1000});

            if (diff > 100 && avpacket_queue_size(&ctx->queue) < ctx->queue_size) {
                // Video

                video_pkt.buf = av_buffer_pool_get(video_pool);
                video_pkt.data = video_pkt.buf->data;
                video_pkt.size = video_pkt.buf->size;

                if (ctx->draw_bars && pixel_format == AV_PIX_FMT_UYVY422) {
                    const unsigned bars[8] = {
                        0xEA80EA80, 0xD292D210, 0xA910A9A5, 0x90229035,
                        0x6ADD6ACA, 0x51EF515A, 0x286D28EF, 0x10801080 };

                    const auto width  = video_codec->width;
                    const auto height = video_codec->height;
                    auto p = reinterpret_cast<unsigned*>(video_pkt.data);

                    for (int y = 0; y < height; y++) {
                        for (int x = 0; x < width; x += 2)
                            *p++ = bars[(x * 8) / width];
                    }
                }

                video_pkt.pts = av_rescale_q(video_pts, AJA_AUDIO_TIME_BASE_Q, ctx->video_st->time_base);
                video_pkt.dts = video_pkt.pts;
                video_pkt.duration = av_rescale_q(audioTime - video_pts, AJA_AUDIO_TIME_BASE_Q, ctx->video_st->time_base);

                video_pts = av_rescale_q(video_pkt.pts + video_pkt.duration, ctx->video_st->time_base, AJA_AUDIO_TIME_BASE_Q);

                // Audio

                audio_pkt.pts = av_rescale_q(audio_pts, AJA_AUDIO_TIME_BASE_Q, ctx->audio_st->time_base);
                audio_pkt.dts = audio_pkt.pts;
                audio_pkt.duration = av_rescale_q(audioTime - audio_pts, AJA_AUDIO_TIME_BASE_Q, ctx->audio_st->time_base);
                audio_pkt.size = av_rescale_q(audio_pkt.duration, ctx->audio_st->time_base, {1, audio_codec->sample_rate}) * audio_codec->channels * av_get_bytes_per_sample(sample_format);
                audio_pkt.buf = av_buffer_allocz(audio_pkt.size);
                audio_pkt.data = audio_pkt.buf->data;
                audio_pkt.size = audio_pkt.buf->size;

                audio_pts = av_rescale_q(audio_pkt.pts + audio_pkt.duration, ctx->audio_st->time_base, AJA_AUDIO_TIME_BASE_Q);

                // Send

                av_log(avctx, AV_LOG_TRACE, "no signal: %0.6f video_pts=%li video_dur=%li audio_pts=%li audio_dur=%li\n",
                    diff / 1000.0,
                    video_pkt.pts,
                    video_pkt.duration,
                    audio_pkt.pts,
                    audio_pkt.duration);

                ret = avpacket_queue_put(&ctx->queue, &video_pkt);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_ERROR, "invalid video packet\n");
                }

                ret = avpacket_queue_put(&ctx->queue, &audio_pkt);
                if (ret < 0) {
                    av_log(avctx, AV_LOG_ERROR, "invalid audio packet\n");
                }
            }

            device->WaitForInputVerticalInterrupt(channel);
        }

        av_packet_unref(&video_pkt);
        av_packet_unref(&audio_pkt);
    }

    device->AutoCirculateStop(channel);

    av_buffer_pool_uninit(&video_pool);
    av_buffer_pool_uninit(&audio_pool);
}

extern "C" {

av_cold int ff_ntv2_read_header(AVFormatContext *avctx)
{
    const auto ctx = reinterpret_cast<NTV2Context*>(avctx->priv_data);
    int ret;

    avpacket_queue_init(&ctx->queue);

    ctx->device_index = strtol(avctx->url, NULL, 10) - 1;
    if (ctx->device_index < 0 || ctx->device_index > 16) {
        return AVERROR(EINVAL);
    }

    try {
        auto device = new CNTV2Card();

        if (device->Open(ctx->device_index)) {
            av_log(avctx, AV_LOG_DEBUG, "Open %i %i\n", ctx->device_index, device->GetDeviceID());
        } else {
            av_log(avctx, AV_LOG_ERROR, "Open %i\n", ctx->device_index);
            goto error;
        }

        if (!::NTV2DeviceCanDoCapture(device->GetDeviceID())) {
            av_log(avctx, AV_LOG_ERROR, "Cannot do capture\n");
            goto error;
        }

        if (device->SetEveryFrameServices(NTV2_OEM_TASKS)) {
            av_log(avctx, AV_LOG_DEBUG, "SetEveryFrameServices\n");
        } else {
            av_log(avctx, AV_LOG_ERROR, "SetEveryFrameServices\n");
            goto error;
        }

        ctx->device = device;

        av_assert0(ctx->input_source != NTV2_INPUTSOURCE_INVALID);

        ret = setup_video(avctx, ctx);
        if (ret < 0) {
            goto error;
        }

        ret = setup_audio(avctx, ctx);
        if (ret < 0) {
            goto error;
        }

        const auto thread = new AJAThread();
        ctx->thread = thread;

        thread->Attach(capture_thread, avctx);
        thread->SetPriority(AJA_ThreadPriority_High);
        thread->Start();

        return 0;
    } catch (...) {
        av_log(avctx, AV_LOG_ERROR, "unexpected exception");
    }
error:
    ff_ntv2_read_close(avctx);
    return ret < 0 ? ret : -1;
}

av_cold int ff_ntv2_read_close(AVFormatContext *avctx)
{
    const auto ctx = reinterpret_cast<NTV2Context*>(avctx->priv_data);
    const auto device = reinterpret_cast<CNTV2Card*>(ctx->device);
    const auto thread = reinterpret_cast<AJAThread*>(ctx->thread);
    const auto channel = ::NTV2InputSourceToChannel(static_cast<NTV2InputSource>(ctx->input_source));

    ctx->quit = 1;

    if (thread != NULL) {
        while (thread->Active()) {
            AJATime::Sleep(10);
        }
        delete thread;
        ctx->thread = NULL;
    }

    avpacket_queue_close(&ctx->queue);

    if (device) {
        if (device->UnsubscribeInputVerticalEvent(channel)) {
            av_log(avctx, AV_LOG_DEBUG, "UnsubscribeInputVerticalEvent %i\n", channel);
        } else {
            av_log(avctx, AV_LOG_ERROR, "UnsubscribeInputVerticalEvent %i\n", channel);
        }

        if (device->UnsubscribeOutputVerticalEvent(channel)) {
            av_log(avctx, AV_LOG_DEBUG, "UnsubscribeOutputVerticalEvent %i\n", channel);
        } else {
            av_log(avctx, AV_LOG_ERROR, "UnsubscribeOutputVerticalEvent %i\n", channel);
        }

        if (device->DisableInputInterrupt(channel)) {
            av_log(avctx, AV_LOG_DEBUG, "DisableInputInterrupt %i\n", channel);
        } else {
            av_log(avctx, AV_LOG_ERROR, "DisableInputInterrupt %i\n", channel);
        }

	    device->DMABufferUnlockAll();

        delete device;
        ctx->device = NULL;
    }

    return 0;
}

int ff_ntv2_read_packet(AVFormatContext *avctx, AVPacket *pkt)
{
    const auto ctx = reinterpret_cast<NTV2Context*>(avctx->priv_data);

    return avpacket_queue_get(&ctx->queue, pkt);
}

} // extern "C"

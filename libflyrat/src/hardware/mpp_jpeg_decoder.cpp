// SPDX-License-Identifier: MIT
#include "flyrat/hardware/mpp_jpeg_decoder.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>

#include <dlfcn.h>

namespace flyrat::hardware {
namespace {

using MppCtx = void*;
using MppApi = void*;
using MppPacket = void*;
using MppFrame = void*;
using MppBuffer = void*;
using MppBufferGroup = void*;
using MppMeta = void*;

// Values from the Rockchip MPP headers (rk_type.h, rk_mpi_cmd.h, mpp_frame.h,
// mpp_meta.h, mpp_buffer.h). They are stable ABI constants rather than local
// configuration, so they are repeated here instead of pulling in the headers.
constexpr int kMppCtxDec = 0;
constexpr int kMppCodingMjpeg = 8;
constexpr int kMppOk = 0;
constexpr int kMppErrTimeout = -8;

constexpr int kMppBufferTypeIon = 1;
constexpr int kMppBufferTypeDrm = 3;
constexpr int kMppBufferInternal = 0;

constexpr int kMppDecSetExtBufGroup = 3211266;
constexpr int kMppDecSetInfoChangeReady = 3211267;
constexpr int kMppDecSetOutputFormat = 3211274;
constexpr int kMppSetOutputTimeout = 2097159;

// FOURCC_META('o', 'f', 'r', 'm'): the frame the decoder writes into.
constexpr std::uint32_t kKeyOutputFrame = 1868984941U;

constexpr std::uint32_t kMppFmtNv12 = 0;
constexpr std::uint32_t kMppFmtNv16 = 2;
constexpr std::uint32_t kMppFmtMask = 0x0000000fU;

// decode_get_frame blocks for up to 20 ms once the output timeout is set, so a
// handful of retries is enough to cover a lost wakeup.
constexpr int kDecodeRetries = 10;

// MppApi starts with a size/version header followed by the codec entry points.
// Only the prefix needed here is declared; `size` is checked at runtime before
// any pointer is called.
struct MppApiPrefix {
    std::uint32_t size;
    std::uint32_t version;
    int (*decode)(MppCtx, MppPacket, MppFrame*);
    int (*decode_put_packet)(MppCtx, MppPacket);
    int (*decode_get_frame)(MppCtx, MppFrame*);
    int (*encode)(MppCtx, MppFrame, MppPacket*);
    int (*encode_put_frame)(MppCtx, MppFrame);
    int (*encode_get_packet)(MppCtx, MppPacket*);
    int (*isp)(MppCtx, MppFrame, MppFrame);
    int (*isp_put_frame)(MppCtx, MppFrame);
    int (*isp_get_frame)(MppCtx, MppFrame*);
    int (*poll)(MppCtx, int, int);
    int (*dequeue)(MppCtx, int, void*);
    int (*enqueue)(MppCtx, int, void*);
    int (*reset)(MppCtx);
    int (*control)(MppCtx, int, void*);
};

static_assert(offsetof(MppApiPrefix, decode_put_packet) == 16,
    "MppApi decode_put_packet offset changed");
static_assert(offsetof(MppApiPrefix, decode_get_frame) == 24,
    "MppApi decode_get_frame offset changed");
static_assert(offsetof(MppApiPrefix, reset) == 104,
    "MppApi reset offset changed");
static_assert(offsetof(MppApiPrefix, control) == 112,
    "MppApi control offset changed");

using MppCreateFn = int (*)(MppCtx*, MppApi*);
using MppInitFn = int (*)(MppCtx, int, int);
using MppDestroyFn = int (*)(MppCtx);
using MppPacketInitWithBufferFn = int (*)(MppPacket*, MppBuffer);
using MppPacketDeinitFn = int (*)(MppPacket*);
using MppPacketSetLengthFn = int (*)(MppPacket, std::size_t);
using MppPacketGetMetaFn = MppMeta (*)(MppPacket);
using MppMetaSetFrameFn = int (*)(MppMeta, std::uint32_t, MppFrame);
using MppFrameInitFn = int (*)(MppFrame*);
using MppFrameDeinitFn = int (*)(MppFrame*);
using MppFrameSetU32Fn = void (*)(MppFrame, std::uint32_t);
using MppFrameSetFmtFn = void (*)(MppFrame, std::uint32_t);
using MppFrameSetBufferFn = void (*)(MppFrame, MppBuffer);
using MppFrameGetU32Fn = std::uint32_t (*)(MppFrame);
using MppFrameGetBufferFn = MppBuffer (*)(MppFrame);
using MppBufferGroupGetFn = int (*)(MppBufferGroup*, int, int, const char*,
    const char*);
using MppBufferGroupPutFn = int (*)(MppBufferGroup);
using MppBufferGroupLimitFn = int (*)(MppBufferGroup, std::size_t, int);
using MppBufferGetFn = int (*)(MppBufferGroup, MppBuffer*, std::size_t,
    const char*, const char*);
using MppBufferPutFn = int (*)(MppBuffer, const char*);
using MppBufferPtrFn = void* (*)(MppBuffer, const char*);
using MppBufferSizeFn = std::size_t (*)(MppBuffer, const char*);
using MppBufferWriteFn = int (*)(MppBuffer, std::size_t, const void*,
    std::size_t, const char*);

std::string dl_error()
{
    const char* error = dlerror();
    return error != nullptr ? error : "unknown dynamic loader error";
}

template <typename T>
T symbol(void* handle, const char* name)
{
    dlerror();
    void* value = dlsym(handle, name);
    const char* error = dlerror();
    if (value == nullptr || error != nullptr) {
        throw std::runtime_error(std::string("missing MPP symbol ") + name + ": "
            + (error != nullptr ? error : "not found"));
    }
    return reinterpret_cast<T>(value);
}

// Reads the frame dimensions from a baseline/progressive JPEG SOF marker.
bool jpeg_size(const std::uint8_t* data, std::size_t size, int& width,
    int& height)
{
    if (data == nullptr || size < 4 || data[0] != 0xff || data[1] != 0xd8) {
        return false;
    }

    std::size_t pos = 2;
    while (pos + 4 <= size) {
        if (data[pos] != 0xff) {
            // Some encoders pad between segments; resynchronise on the marker.
            ++pos;
            continue;
        }
        const std::uint8_t marker = data[pos + 1];
        pos += 2;
        if (marker == 0xd8 || marker == 0xd9) {
            continue;
        }
        if (marker == 0xda) {
            // Start of scan: no more header segments to inspect.
            return false;
        }
        if (pos + 2 > size) {
            return false;
        }
        const std::size_t length =
            (static_cast<std::size_t>(data[pos]) << 8) | data[pos + 1];
        if (length < 2 || pos + length > size) {
            return false;
        }

        const bool is_sof = marker >= 0xc0 && marker <= 0xcf
            && marker != 0xc4 && marker != 0xc8 && marker != 0xcc;
        if (is_sof) {
            if (length < 7) {
                return false;
            }
            height = (static_cast<int>(data[pos + 3]) << 8) | data[pos + 4];
            width = (static_cast<int>(data[pos + 5]) << 8) | data[pos + 6];
            return width > 0 && height > 0;
        }
        pos += length;
    }
    return false;
}

int align16(int value)
{
    return (value + 15) & ~15;
}

} // namespace

struct MppJpegDecoder::Impl {
    ~Impl()
    {
        stop();
        if (library != nullptr) {
            dlclose(library);
        }
    }

    bool start()
    {
        if (running) {
            return true;
        }
        stop();

        if (!load_symbols()) {
            return false;
        }

        if (create(&ctx, reinterpret_cast<MppApi*>(&api)) != kMppOk
            || ctx == nullptr || api == nullptr) {
            last_error = "mpp_create failed";
            stop();
            return false;
        }
        if (api->size < sizeof(MppApiPrefix) || api->decode_get_frame == nullptr
            || api->decode_put_packet == nullptr) {
            last_error = "librockchip_mpp has an incompatible MppApi";
            stop();
            return false;
        }
        if (init(ctx, kMppCtxDec, kMppCodingMjpeg) != kMppOk) {
            last_error = "mpp_init(MJPEG decoder) failed";
            stop();
            return false;
        }

        // The UVC camera sends 4:2:2 JPEGs. Asking for NV12 keeps the output
        // layout predictable; the luma plane is identical either way, so a
        // rejected format is not fatal.
        std::uint32_t format = kMppFmtNv12;
        if (api->control != nullptr) {
            api->control(ctx, kMppDecSetOutputFormat, &format);
            // Block inside decode_get_frame instead of polling from here.
            std::int64_t timeout_ms = 20;
            api->control(ctx, kMppSetOutputTimeout, &timeout_ms);
        }

        frames = 0;
        errors = 0;
        last_error.clear();
        running = true;
        return true;
    }

    void stop()
    {
        running = false;
        release_frame();
        if (group != nullptr && buffer_group_put != nullptr) {
            buffer_group_put(group);
            group = nullptr;
        }
        if (info_group != nullptr && buffer_group_put != nullptr) {
            buffer_group_put(info_group);
            info_group = nullptr;
        }
        if (input_buffer != nullptr && buffer_put != nullptr) {
            buffer_put(input_buffer, "flyrat_mpp");
            input_buffer = nullptr;
            input_size = 0;
        }
        if (input_group != nullptr && buffer_group_put != nullptr) {
            buffer_group_put(input_group);
            input_group = nullptr;
        }
        if (ctx != nullptr && destroy != nullptr) {
            destroy(ctx);
            ctx = nullptr;
        }
        api = nullptr;
    }

    bool decode(const std::uint8_t* jpeg, std::size_t size, DecodedJpeg& out)
    {
        if (!running) {
            return fail("MPP JPEG decoder is not running");
        }
        int width = 0;
        int height = 0;
        if (!jpeg_size(jpeg, size, width, height)) {
            return fail("JPEG header has no frame size");
        }
        if (!ensure_frame(width, height)) {
            return false;
        }

        // MJPEG has no inter-frame references, so every packet carries a full
        // image. MPP still requires the async put/get pair; the simple
        // MppApi::decode path is only valid for non-MJPEG codecs. The decoder
        // drops packets without an attached MppBuffer, so the JPEG is copied
        // into a reusable ION/DRM buffer instead of passing user memory.
        if (!ensure_input_buffer(size)) {
            return false;
        }
        if (buffer_write(input_buffer, 0, jpeg, size, "flyrat_mpp") != kMppOk) {
            return fail("failed to copy the JPEG into the MPP input buffer");
        }

        MppPacket packet = nullptr;
        if (packet_init_with_buffer(&packet, input_buffer) != kMppOk
            || packet == nullptr) {
            return fail("mpp_packet_init_with_buffer failed");
        }
        packet_set_length(packet, size);

        MppMeta meta = packet_get_meta(packet);
        if (meta != nullptr) {
            meta_set_frame(meta, kKeyOutputFrame, frame);
        }

        if (api->decode_put_packet(ctx, packet) != kMppOk) {
            packet_deinit(&packet);
            return fail("mpp decode_put_packet failed");
        }

        MppFrame decoded = nullptr;
        bool complete = false;
        for (int attempt = 0; attempt < kDecodeRetries; ++attempt) {
            const int ret = api->decode_get_frame(ctx, &decoded);
            if (ret == kMppErrTimeout) {
                continue;
            }
            if (ret != kMppOk) {
                packet_deinit(&packet);
                return fail("mpp decode_get_frame failed");
            }
            if (decoded == nullptr) {
                continue;
            }
            if (frame_info_change(decoded)) {
                const bool ready = setup_info_change(decoded);
                if (decoded != frame) {
                    frame_deinit(&decoded);
                }
                if (!ready) {
                    packet_deinit(&packet);
                    return false;
                }
                continue;
            }
            complete = true;
            break;
        }
        packet_deinit(&packet);

        if (!complete || decoded == nullptr) {
            return fail("timed out waiting for a decoded frame");
        }
        if ((frame_err(decoded) & 0x0fU) != 0) {
            char message[160];
            std::snprintf(message, sizeof(message),
                "mpp frame error 0x%x fmt=%u %ux%u own=%d",
                frame_err(decoded), frame_fmt(decoded), frame_width(decoded),
                frame_height(decoded), decoded == frame ? 1 : 0);
            release_extra(decoded);
            return fail(message);
        }

        const std::uint32_t fmt = frame_fmt(decoded);
        const std::uint32_t decoded_width = frame_width(decoded);
        const std::uint32_t decoded_height = frame_height(decoded);
        const std::uint32_t hor_stride = frame_hor_stride(decoded);
        if ((fmt & kMppFmtMask) != kMppFmtNv12
            && (fmt & kMppFmtMask) != kMppFmtNv16) {
            release_extra(decoded);
            return fail("unexpected MPP output format");
        }
        if (decoded_width == 0 || decoded_height == 0
            || hor_stride < decoded_width) {
            release_extra(decoded);
            return fail("invalid MPP frame geometry");
        }

        MppBuffer buffer = frame_buffer(decoded);
        if (buffer == nullptr) {
            release_extra(decoded);
            return fail("MPP frame has no buffer");
        }
        const std::uint8_t* pixels = static_cast<const std::uint8_t*>(
            buffer_ptr(buffer, "flyrat_mpp"));
        const std::size_t bytes = static_cast<std::size_t>(hor_stride)
            * static_cast<std::size_t>(decoded_height);
        if (pixels == nullptr || buffer_size(buffer, "flyrat_mpp") < bytes) {
            release_extra(decoded);
            return fail("MPP frame buffer is too small");
        }

        out.width = static_cast<int>(decoded_width);
        out.height = static_cast<int>(decoded_height);
        out.stride = static_cast<int>(hor_stride);
        out.luma.resize(bytes);
        std::memcpy(out.luma.data(), pixels, bytes);

        release_extra(decoded);
        last_format = static_cast<int>(fmt & kMppFmtMask);
        ++frames;
        last_error.clear();
        return true;
    }

    bool available() const
    {
        return running;
    }

    bool fail(const std::string& message)
    {
        last_error = message;
        ++errors;
        return false;
    }

    bool load_symbols()
    {
        if (library == nullptr) {
            library = dlopen("librockchip_mpp.so.1", RTLD_NOW | RTLD_LOCAL);
            if (library == nullptr) {
                library = dlopen("librockchip_mpp.so.0", RTLD_NOW | RTLD_LOCAL);
            }
            if (library == nullptr) {
                last_error = "load librockchip_mpp: " + dl_error();
                return false;
            }
        }

        try {
            create = symbol<MppCreateFn>(library, "mpp_create");
            init = symbol<MppInitFn>(library, "mpp_init");
            destroy = symbol<MppDestroyFn>(library, "mpp_destroy");
            packet_init_with_buffer = symbol<MppPacketInitWithBufferFn>(
                library, "mpp_packet_init_with_buffer");
            packet_deinit =
                symbol<MppPacketDeinitFn>(library, "mpp_packet_deinit");
            packet_set_length = symbol<MppPacketSetLengthFn>(library,
                "mpp_packet_set_length");
            packet_get_meta =
                symbol<MppPacketGetMetaFn>(library, "mpp_packet_get_meta");
            meta_set_frame =
                symbol<MppMetaSetFrameFn>(library, "mpp_meta_set_frame");
            frame_init = symbol<MppFrameInitFn>(library, "mpp_frame_init");
            frame_deinit =
                symbol<MppFrameDeinitFn>(library, "mpp_frame_deinit");
            frame_set_width =
                symbol<MppFrameSetU32Fn>(library, "mpp_frame_set_width");
            frame_set_height =
                symbol<MppFrameSetU32Fn>(library, "mpp_frame_set_height");
            frame_set_hor_stride = symbol<MppFrameSetU32Fn>(
                library, "mpp_frame_set_hor_stride");
            frame_set_ver_stride = symbol<MppFrameSetU32Fn>(
                library, "mpp_frame_set_ver_stride");
            frame_set_fmt =
                symbol<MppFrameSetFmtFn>(library, "mpp_frame_set_fmt");
            frame_set_buffer =
                symbol<MppFrameSetBufferFn>(library, "mpp_frame_set_buffer");
            frame_width = symbol<MppFrameGetU32Fn>(library,
                "mpp_frame_get_width");
            frame_height = symbol<MppFrameGetU32Fn>(library,
                "mpp_frame_get_height");
            frame_hor_stride = symbol<MppFrameGetU32Fn>(library,
                "mpp_frame_get_hor_stride");
            frame_ver_stride = symbol<MppFrameGetU32Fn>(library,
                "mpp_frame_get_ver_stride");
            frame_fmt = symbol<MppFrameGetU32Fn>(library, "mpp_frame_get_fmt");
            frame_buffer = symbol<MppFrameGetBufferFn>(library,
                "mpp_frame_get_buffer");
            frame_err = symbol<MppFrameGetU32Fn>(library,
                "mpp_frame_get_errinfo");
            frame_info_change = symbol<MppFrameGetU32Fn>(library,
                "mpp_frame_get_info_change");
            buffer_group_get = symbol<MppBufferGroupGetFn>(library,
                "mpp_buffer_group_get");
            buffer_group_put = symbol<MppBufferGroupPutFn>(library,
                "mpp_buffer_group_put");
            buffer_group_limit = symbol<MppBufferGroupLimitFn>(library,
                "mpp_buffer_group_limit_config");
            buffer_get = symbol<MppBufferGetFn>(library,
                "mpp_buffer_get_with_tag");
            buffer_put = symbol<MppBufferPutFn>(library,
                "mpp_buffer_put_with_caller");
            buffer_ptr = symbol<MppBufferPtrFn>(library,
                "mpp_buffer_get_ptr_with_caller");
            buffer_size = symbol<MppBufferSizeFn>(library,
                "mpp_buffer_get_size_with_caller");
            buffer_write = symbol<MppBufferWriteFn>(library,
                "mpp_buffer_write_with_caller");
        } catch (const std::exception& error) {
            last_error = error.what();
            dlclose(library);
            library = nullptr;
            return false;
        }
        return true;
    }

    bool ensure_input_buffer(std::size_t size)
    {
        if (input_buffer != nullptr && input_size >= size) {
            return true;
        }
        if (input_buffer != nullptr && buffer_put != nullptr) {
            buffer_put(input_buffer, "flyrat_mpp");
            input_buffer = nullptr;
            input_size = 0;
        }
        if (input_group == nullptr) {
            if (buffer_group_get(&input_group, kMppBufferTypeIon,
                    kMppBufferInternal, "flyrat_mpp", "input")
                != kMppOk) {
                input_group = nullptr;
                if (buffer_group_get(&input_group, kMppBufferTypeDrm,
                        kMppBufferInternal, "flyrat_mpp", "input")
                    != kMppOk) {
                    input_group = nullptr;
                    return fail("failed to create an MPP input group");
                }
            }
            buffer_group_limit(input_group, 4U * 1024U * 1024U, 2);
        }
        if (buffer_get(input_group, &input_buffer, size, "flyrat_mpp", "input")
            != kMppOk || input_buffer == nullptr) {
            input_buffer = nullptr;
            return fail("failed to allocate the MPP input buffer");
        }
        input_size = size;
        return true;
    }

    bool ensure_frame(int width, int height)
    {
        if (frame != nullptr && width == image_width && height == image_height) {
            return true;
        }
        release_frame();

        const int hor_stride = align16(width);
        const int ver_stride = align16(height);
        // NV16 needs two bytes per pixel; reserve that much so the decoder can
        // keep 4:2:2 output if NV12 conversion is unavailable.
        const std::size_t size = static_cast<std::size_t>(hor_stride)
            * static_cast<std::size_t>(ver_stride) * 2;

        if (group == nullptr) {
            if (buffer_group_get(&group, kMppBufferTypeIon,
                    kMppBufferInternal, "flyrat_mpp", "ensure_frame")
                != kMppOk) {
                group = nullptr;
                if (buffer_group_get(&group, kMppBufferTypeDrm,
                        kMppBufferInternal, "flyrat_mpp", "ensure_frame")
                    != kMppOk) {
                    group = nullptr;
                    return fail("failed to create an MPP buffer group");
                }
            }
            buffer_group_limit(group, size, 4);
        }

        if (buffer_get(group, &buffer, size, "flyrat_mpp", "ensure_frame")
            != kMppOk || buffer == nullptr) {
            buffer = nullptr;
            return fail("failed to allocate the MPP output buffer");
        }
        if (frame_init(&frame) != kMppOk || frame == nullptr) {
            frame = nullptr;
            buffer_put(buffer, "flyrat_mpp");
            buffer = nullptr;
            return fail("mpp_frame_init failed");
        }
        frame_set_width(frame, static_cast<std::uint32_t>(width));
        frame_set_height(frame, static_cast<std::uint32_t>(height));
        frame_set_hor_stride(frame, static_cast<std::uint32_t>(hor_stride));
        frame_set_ver_stride(frame, static_cast<std::uint32_t>(ver_stride));
        frame_set_fmt(frame, kMppFmtNv12);
        frame_set_buffer(frame, buffer);

        image_width = width;
        image_height = height;
        return true;
    }

    void release_frame()
    {
        if (frame != nullptr && frame_deinit != nullptr) {
            frame_deinit(&frame);
            frame = nullptr;
        }
        if (buffer != nullptr && buffer_put != nullptr) {
            buffer_put(buffer, "flyrat_mpp");
            buffer = nullptr;
        }
        image_width = 0;
        image_height = 0;
    }

    // MPP can still decide to allocate its own output buffers through the
    // info-change handshake. Hand it a group sized for the advertised frame
    // and let it continue; the preallocated frame remains allocated but idle.
    bool setup_info_change(MppFrame changed)
    {
        const std::uint32_t hor_stride = frame_hor_stride(changed);
        const std::uint32_t ver_stride = frame_ver_stride(changed);
        const std::size_t size = static_cast<std::size_t>(hor_stride)
            * static_cast<std::size_t>(ver_stride) * 2;
        if (size == 0) {
            return fail("MPP info change reported an empty frame");
        }

        if (info_group == nullptr) {
            if (buffer_group_get(&info_group, kMppBufferTypeIon,
                    kMppBufferInternal, "flyrat_mpp", "info_change")
                != kMppOk) {
                info_group = nullptr;
                if (buffer_group_get(&info_group, kMppBufferTypeDrm,
                        kMppBufferInternal, "flyrat_mpp", "info_change")
                    != kMppOk) {
                    info_group = nullptr;
                    return fail("failed to create an MPP info-change group");
                }
            }
        }
        ++info_changes;
        buffer_group_limit(info_group, size, 6);
        if (api->control == nullptr
            || api->control(ctx, kMppDecSetExtBufGroup, info_group) != kMppOk
            || api->control(ctx, kMppDecSetInfoChangeReady, nullptr)
                != kMppOk) {
            return fail("MPP info change handshake failed");
        }
        return true;
    }

    void release_extra(MppFrame decoded)
    {
        if (decoded != nullptr && decoded != frame && frame_deinit != nullptr) {
            frame_deinit(&decoded);
        }
    }

    void* library = nullptr;
    MppCtx ctx = nullptr;
    MppApiPrefix* api = nullptr;
    MppFrame frame = nullptr;
    MppBuffer buffer = nullptr;
    MppBufferGroup group = nullptr;
    MppBufferGroup info_group = nullptr;
    MppBuffer input_buffer = nullptr;
    MppBufferGroup input_group = nullptr;
    std::size_t input_size = 0;
    int image_width = 0;
    int image_height = 0;
    bool running = false;

    MppCreateFn create = nullptr;
    MppInitFn init = nullptr;
    MppDestroyFn destroy = nullptr;
    MppPacketInitWithBufferFn packet_init_with_buffer = nullptr;
    MppPacketDeinitFn packet_deinit = nullptr;
    MppPacketSetLengthFn packet_set_length = nullptr;
    MppPacketGetMetaFn packet_get_meta = nullptr;
    MppMetaSetFrameFn meta_set_frame = nullptr;
    MppFrameInitFn frame_init = nullptr;
    MppFrameDeinitFn frame_deinit = nullptr;
    MppFrameSetU32Fn frame_set_width = nullptr;
    MppFrameSetU32Fn frame_set_height = nullptr;
    MppFrameSetU32Fn frame_set_hor_stride = nullptr;
    MppFrameSetU32Fn frame_set_ver_stride = nullptr;
    MppFrameSetFmtFn frame_set_fmt = nullptr;
    MppFrameSetBufferFn frame_set_buffer = nullptr;
    MppFrameGetU32Fn frame_width = nullptr;
    MppFrameGetU32Fn frame_height = nullptr;
    MppFrameGetU32Fn frame_hor_stride = nullptr;
    MppFrameGetU32Fn frame_ver_stride = nullptr;
    MppFrameGetU32Fn frame_fmt = nullptr;
    MppFrameGetBufferFn frame_buffer = nullptr;
    MppFrameGetU32Fn frame_err = nullptr;
    MppFrameGetU32Fn frame_info_change = nullptr;
    MppBufferGroupGetFn buffer_group_get = nullptr;
    MppBufferGroupPutFn buffer_group_put = nullptr;
    MppBufferGroupLimitFn buffer_group_limit = nullptr;
    MppBufferGetFn buffer_get = nullptr;
    MppBufferPutFn buffer_put = nullptr;
    MppBufferPtrFn buffer_ptr = nullptr;
    MppBufferSizeFn buffer_size = nullptr;
    MppBufferWriteFn buffer_write = nullptr;

    std::uint64_t frames = 0;
    std::uint64_t errors = 0;
    std::uint64_t info_changes = 0;
    int last_format = 0;
    std::string last_error;
};

MppJpegDecoder::MppJpegDecoder()
    : impl_(std::make_unique<Impl>())
{
}

MppJpegDecoder::~MppJpegDecoder() = default;

bool MppJpegDecoder::start()
{
    return impl_->start();
}

void MppJpegDecoder::stop()
{
    impl_->stop();
}

bool MppJpegDecoder::decode(const std::uint8_t* jpeg, std::size_t size,
    DecodedJpeg& out)
{
    return impl_->decode(jpeg, size, out);
}

bool MppJpegDecoder::available() const
{
    return impl_->available();
}

const std::string& MppJpegDecoder::lastError() const
{
    return impl_->last_error;
}

std::uint64_t MppJpegDecoder::frames() const
{
    return impl_->frames;
}

std::uint64_t MppJpegDecoder::errors() const
{
    return impl_->errors;
}

int MppJpegDecoder::lastFormat() const
{
    return impl_->last_format;
}

std::uint64_t MppJpegDecoder::infoChanges() const
{
    return impl_->info_changes;
}

} // namespace flyrat::hardware

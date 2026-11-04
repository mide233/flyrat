// SPDX-License-Identifier: MIT
#include "mjpeg_decoder.hpp"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <dlfcn.h>

namespace flyrat::hardware::detail {
namespace {

// Opaque GStreamer types. We deliberately avoid the development headers,
// because the Buildroot target ships the runtime libraries only.
struct GstElement;
struct GstBin;
struct GstSample;
struct GstBuffer;
struct GstCaps;
struct GstStructure;
struct GstAppSink;

using GstInitFn = void (*)(int*, char***);
using GstParseLaunchFn = GstElement* (*)(const char*, void**);
using GstElementSetStateFn = int (*)(GstElement*, int);
using GstBinGetByNameFn = GstElement* (*)(GstBin*, const char*);
using GstObjectUnrefFn = void (*)(void*);
using GstAppSinkTryPullSampleFn = GstSample* (*)(GstAppSink*, std::uint64_t);
using GstAppSinkIsEosFn = int (*)(GstAppSink*);
using GstSampleGetBufferFn = GstBuffer* (*)(GstSample*);
using GstSampleGetCapsFn = GstCaps* (*)(GstSample*);
using GstCapsGetStructureFn = GstStructure* (*)(const GstCaps*, unsigned int);
using GstStructureGetIntFn = int (*)(const GstStructure*, const char*, int*);
using GstBufferGetSizeFn = std::size_t (*)(GstBuffer*);
using GstBufferExtractFn = std::size_t (*)(GstBuffer*, std::size_t, void*, std::size_t);
using GstSampleUnrefFn = void (*)(GstSample*);

constexpr int kGstStateNull = 1;
constexpr int kGstStatePlaying = 4;
constexpr std::uint64_t kReadTimeoutNs = 200000000ULL;

std::string dl_error()
{
    const char* error = dlerror();
    return error != nullptr ? error : "unknown dynamic loader error";
}

template <typename T>
T load_symbol(void* handle, const char* name)
{
    dlerror();
    void* symbol = dlsym(handle, name);
    const char* error = dlerror();
    if (symbol == nullptr || error != nullptr) {
        throw StereoError(std::string("missing GStreamer symbol ") + name + ": " +
            (error != nullptr ? error : "not found"));
    }
    return reinterpret_cast<T>(symbol);
}

bool valid_pipeline_atom(const std::string& value)
{
    return value.find_first_of(" \t\r\n!\"'") == std::string::npos;
}

} // namespace

struct MjpegDecoder::Impl {
    explicit Impl(MjpegConfig cfg)
        : config(std::move(cfg))
    {
        if (!valid_pipeline_atom(config.device)) {
            throw StereoError("stereo device path contains characters unsafe for a GStreamer pipeline");
        }
        if (config.width <= 0 || config.height <= 0 || config.fps <= 0) {
            throw StereoError("MJPEG width, height and fps must be positive");
        }
        // GStreamer's NV12 buffer is usually tightly packed for widths that
        // are a multiple of 16. We only read the Y plane and require this
        // alignment rather than guessing a hidden stride.
        if ((config.width % 16) != 0 || (config.height % 2) != 0) {
            throw StereoError("MJPEG mode requires a width divisible by 16 and an even height");
        }

        gst_library = dlopen("libgstreamer-1.0.so.0", RTLD_NOW | RTLD_GLOBAL);
        if (gst_library == nullptr) {
            throw StereoError("load libgstreamer-1.0.so.0: " + dl_error());
        }
        app_library = dlopen("libgstapp-1.0.so.0", RTLD_NOW | RTLD_LOCAL);
        if (app_library == nullptr) {
            const std::string error = dl_error();
            dlclose(gst_library);
            gst_library = nullptr;
            throw StereoError("load libgstapp-1.0.so.0: " + error);
        }

        try {
            gst_init = load_symbol<GstInitFn>(gst_library, "gst_init");
            parse_launch = load_symbol<GstParseLaunchFn>(gst_library, "gst_parse_launch");
            element_set_state = load_symbol<GstElementSetStateFn>(
                gst_library, "gst_element_set_state");
            bin_get_by_name = load_symbol<GstBinGetByNameFn>(
                gst_library, "gst_bin_get_by_name");
            object_unref = load_symbol<GstObjectUnrefFn>(gst_library, "gst_object_unref");
            sample_get_buffer = load_symbol<GstSampleGetBufferFn>(
                gst_library, "gst_sample_get_buffer");
            sample_get_caps = load_symbol<GstSampleGetCapsFn>(
                gst_library, "gst_sample_get_caps");
            caps_get_structure = load_symbol<GstCapsGetStructureFn>(
                gst_library, "gst_caps_get_structure");
            structure_get_int = load_symbol<GstStructureGetIntFn>(
                gst_library, "gst_structure_get_int");
            buffer_get_size = load_symbol<GstBufferGetSizeFn>(
                gst_library, "gst_buffer_get_size");
            buffer_extract = load_symbol<GstBufferExtractFn>(
                gst_library, "gst_buffer_extract");
            sample_unref = load_symbol<GstSampleUnrefFn>(gst_library, "gst_sample_unref");
            app_sink_try_pull_sample = load_symbol<GstAppSinkTryPullSampleFn>(
                app_library, "gst_app_sink_try_pull_sample");
            app_sink_is_eos = load_symbol<GstAppSinkIsEosFn>(
                app_library, "gst_app_sink_is_eos");
        } catch (...) {
            dlclose(app_library);
            dlclose(gst_library);
            app_library = nullptr;
            gst_library = nullptr;
            throw;
        }
    }

    ~Impl()
    {
        stop();
        if (app_library != nullptr) {
            dlclose(app_library);
        }
        if (gst_library != nullptr) {
            dlclose(gst_library);
        }
    }

    void start()
    {
        if (running) {
            return;
        }

        std::ostringstream pipeline_text;
        pipeline_text << "v4l2src device=" << config.device << " io-mode=4"
                      << " ! image/jpeg,width=" << config.width
                      << ",height=" << config.height
                      << ",framerate=" << config.fps << "/1"
                      << " ! mppjpegdec"
                      << " ! video/x-raw,format=NV12"
                      << " ! appsink name=flyrat_sink max-buffers="
                      << std::max(config.max_buffers, 2)
                      << " drop=true sync=false";

        const std::string text = pipeline_text.str();
        gst_init(nullptr, nullptr);
        pipeline = parse_launch(text.c_str(), nullptr);
        if (pipeline == nullptr) {
            throw StereoError("failed to create the MJPEG GStreamer pipeline");
        }

        sink_element = bin_get_by_name(reinterpret_cast<GstBin*>(pipeline), "flyrat_sink");
        if (sink_element == nullptr) {
            const std::string error = "appsink was not created by the MJPEG pipeline";
            stop();
            throw StereoError(error);
        }
        sink = reinterpret_cast<GstAppSink*>(sink_element);

        const int state = element_set_state(pipeline, kGstStatePlaying);
        if (state == 0) {
            const std::string error = "GStreamer could not start the MJPEG pipeline";
            stop();
            throw StereoError(error);
        }

        sequence = 0;
        running = true;
    }

    void stop()
    {
        running = false;
        if (pipeline != nullptr) {
            element_set_state(pipeline, kGstStateNull);
        }
        if (sink_element != nullptr) {
            object_unref(sink_element);
            sink_element = nullptr;
            sink = nullptr;
        }
        if (pipeline != nullptr) {
            object_unref(pipeline);
            pipeline = nullptr;
        }
    }

    MjpegReadStatus read(MjpegFrame& frame)
    {
        if (!running || sink == nullptr) {
            return MjpegReadStatus::kError;
        }

        GstSample* sample = app_sink_try_pull_sample(sink, kReadTimeoutNs);
        if (sample == nullptr) {
            return app_sink_is_eos(sink) != 0 ? MjpegReadStatus::kEos
                                                : MjpegReadStatus::kTimeout;
        }

        GstBuffer* buffer = sample_get_buffer(sample);
        GstCaps* caps = sample_get_caps(sample);
        int width = 0;
        int height = 0;
        GstStructure* structure = caps != nullptr ? caps_get_structure(caps, 0) : nullptr;
        const bool valid_caps = structure != nullptr &&
            structure_get_int(structure, "width", &width) != 0 &&
            structure_get_int(structure, "height", &height) != 0;
        if (buffer == nullptr || !valid_caps || width != config.width ||
            height != config.height) {
            sample_unref(sample);
            return MjpegReadStatus::kError;
        }

        const std::size_t luma_size = static_cast<std::size_t>(width) *
            static_cast<std::size_t>(height);
        const std::size_t buffer_size = buffer_get_size(buffer);
        std::size_t bytes = luma_size;
        int aligned_height = height;
        if (config.color) {
            const std::size_t nv12_frame = static_cast<std::size_t>(width) * 3U / 2U;
            if (nv12_frame == 0 || (buffer_size % nv12_frame) != 0) {
                sample_unref(sample);
                return MjpegReadStatus::kError;
            }
            bytes = buffer_size;
            aligned_height = static_cast<int>(buffer_size / nv12_frame);
            if (aligned_height < height || (aligned_height % 2) != 0) {
                sample_unref(sample);
                return MjpegReadStatus::kError;
            }
        } else if (buffer_size < luma_size) {
            sample_unref(sample);
            return MjpegReadStatus::kError;
        }

        frame.width = width;
        frame.height = height;
        frame.aligned_height = aligned_height;
        frame.color = config.color;
        frame.data.resize(bytes);
        const std::size_t copied = buffer_extract(
            buffer, 0, frame.data.data(), bytes);
        sample_unref(sample);
        if (copied != bytes) {
            return MjpegReadStatus::kError;
        }

        frame.sequence = ++sequence;
        frame.stamp = std::chrono::steady_clock::now();
        return MjpegReadStatus::kFrame;
    }

    MjpegConfig config;
    void* gst_library = nullptr;
    void* app_library = nullptr;

    GstInitFn gst_init = nullptr;
    GstParseLaunchFn parse_launch = nullptr;
    GstElementSetStateFn element_set_state = nullptr;
    GstBinGetByNameFn bin_get_by_name = nullptr;
    GstObjectUnrefFn object_unref = nullptr;
    GstAppSinkTryPullSampleFn app_sink_try_pull_sample = nullptr;
    GstAppSinkIsEosFn app_sink_is_eos = nullptr;
    GstSampleGetBufferFn sample_get_buffer = nullptr;
    GstSampleGetCapsFn sample_get_caps = nullptr;
    GstCapsGetStructureFn caps_get_structure = nullptr;
    GstStructureGetIntFn structure_get_int = nullptr;
    GstBufferGetSizeFn buffer_get_size = nullptr;
    GstBufferExtractFn buffer_extract = nullptr;
    GstSampleUnrefFn sample_unref = nullptr;

    GstElement* pipeline = nullptr;
    GstElement* sink_element = nullptr;
    GstAppSink* sink = nullptr;
    std::atomic<bool> running { false };
    std::uint64_t sequence = 0;
};

MjpegDecoder::MjpegDecoder(MjpegConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

MjpegDecoder::~MjpegDecoder() = default;

void MjpegDecoder::start()
{
    impl_->start();
}

void MjpegDecoder::stop()
{
    impl_->stop();
}

bool MjpegDecoder::running() const
{
    return impl_->running;
}

MjpegReadStatus MjpegDecoder::read(MjpegFrame& frame)
{
    return impl_->read(frame);
}

} // namespace flyrat::hardware::detail

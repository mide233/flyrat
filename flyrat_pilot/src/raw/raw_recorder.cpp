// SPDX-License-Identifier: MIT

#include "raw/raw_recorder.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "flyrat/hardware/serial_transport.hpp"
#include "flyrat/msp_interact.hpp"
#include "flyrat/time.hpp"
#include "raw/raw_bag.hpp"
#include "sensors/sensor_stream.hpp"

namespace flyrat::pilot {

namespace {

constexpr char kDefaultMspDevice[] = "/dev/ttyS3";

using flyrat::steady_now_ns;
using flyrat::system_now_ns;
using flyrat::to_ns;

std::string camera_stats_json(const hardware::StereoJpegStats& stats) {
    std::ostringstream out;
    out << "{\"frames\":" << stats.frames << ",\"driver_dropped\":" << stats.dropped
        << ",\"errors\":" << stats.errors << ",\"throttled\":" << stats.throttled
        << ",\"fps\":" << stats.fps << ",\"capture_fps\":" << stats.capture_fps
        << ",\"width\":" << stats.width << ",\"height\":" << stats.height << '}';
    return out.str();
}

} // namespace

struct RawRecorder::Impl {
    explicit Impl(RawRecorderConfig value)
        : config(std::move(value)) {}

    RawRecorderConfig config;
    std::unique_ptr<MspInteract> msp;
    std::unique_ptr<hardware::StereoJpegSource> camera;
    std::unique_ptr<RawBagWriter> bag;
    std::unique_ptr<SensorStreamServer> preview;
    RawBagMetadata metadata;
    std::atomic<std::uint64_t> imu_samples{0};
    std::atomic<std::uint64_t> images{0};
    std::chrono::steady_clock::time_point start_time{};
    bool started = false;
    bool last_link_state = false;
    bool have_link_state = false;
    hardware::StereoJpegStats last_camera_stats{};
    bool have_last_camera_stats = false;
    LinkStats last_link_stats{};
    bool have_last_link_stats = false;
    SensorStreamStats last_preview_stats{};
    bool have_last_preview_stats = false;

    void on_imu(const Sample<ImuSample>& sample) {
        if (!bag) {
            return;
        }
        std::array<std::int16_t, 9> values{};
        for (std::size_t i = 0; i < 3; ++i) {
            values[i] = sample.value.acc[i];
            values[3 + i] = sample.value.gyro[i];
            values[6 + i] = sample.value.mag[i];
        }
        const std::int64_t stamp_ns = to_ns(sample.stamp);
        bag->writeImu(values, stamp_ns);
        imu_samples.fetch_add(1, std::memory_order_relaxed);

        if (preview) {
            sensors::ImuPayload payload;
            payload.acc = sample.value.acc;
            payload.gyro = sample.value.gyro;
            payload.mag = sample.value.mag;
            preview->send_imu(payload, stamp_ns);
        }
    }

    void on_image(const hardware::StereoJpegFrame& frame) {
        if (!bag || frame.jpeg.empty()) {
            return;
        }
        const std::int64_t stamp_ns = to_ns(frame.stamp);
        bag->writeImage(frame.jpeg.data(), frame.jpeg.size(), frame.sequence, stamp_ns);
        images.fetch_add(1, std::memory_order_relaxed);

        if (preview) {
            preview->send_stereo(static_cast<std::uint32_t>(frame.width),
                static_cast<std::uint32_t>(frame.height), frame.jpeg.data(),
                frame.jpeg.size(), stamp_ns);
        }
    }

    void on_link(bool connected, const std::string& message) {
        if (have_link_state && connected == last_link_state && message.empty()) {
            return;
        }
        have_link_state = true;
        last_link_state = connected;
        std::fprintf(
            stderr, "raw recorder: MSP link %s%s%s\n", connected ? "up" : "down",
            message.empty() ? "" : ": ", message.c_str());
    }

    void check_capture_rate(const hardware::StereoJpegStats& stats) const {
        const int requested = config.camera.capture_fps;
        if (requested <= 0 || stats.capture_fps + 0.5 >= requested) {
            return;
        }
        char detail[128]{};
        std::snprintf(
            detail, sizeof(detail), "camera negotiated %.1f fps, requested %d fps",
            stats.capture_fps, requested);
        if (config.require_capture_fps) {
            throw std::runtime_error(detail);
        }
        std::fprintf(stderr, "raw recorder: warning: %s\n", detail);
    }

    RawRecorderStats snapshot() const {
        RawRecorderStats out;
        out.imu_samples = imu_samples.load(std::memory_order_relaxed);
        out.images = images.load(std::memory_order_relaxed);
        if (bag) {
            const RawBagStats bag_stats = bag->stats();
            out.dropped_imu = bag_stats.dropped_imu;
            out.dropped_images = bag_stats.dropped_images;
            out.bytes = bag_stats.bytes_written;
        }
        if (camera) {
            const hardware::StereoJpegStats camera_stats = camera->stats();
            out.throttled = camera_stats.throttled;
            out.driver_dropped = camera_stats.dropped;
            out.capture_fps = camera_stats.capture_fps;
        } else if (have_last_camera_stats) {
            out.throttled = last_camera_stats.throttled;
            out.driver_dropped = last_camera_stats.dropped;
            out.capture_fps = last_camera_stats.capture_fps;
        }
        if (msp) {
            out.imu_connected = msp->connected();
        } else if (have_last_link_stats) {
            out.imu_connected = last_link_stats.connected;
        }
        out.preview_enabled = config.preview_enabled;
        if (preview) {
            const SensorStreamStats stream_stats = preview->stats();
            out.preview_connected = stream_stats.connected;
            out.preview_priority_sent = stream_stats.priority_sent;
            out.preview_stereo_sent = stream_stats.stereo_sent;
            out.preview_dropped = stream_stats.priority_dropped + stream_stats.stereo_dropped;
        } else if (have_last_preview_stats) {
            out.preview_connected = last_preview_stats.connected;
            out.preview_priority_sent = last_preview_stats.priority_sent;
            out.preview_stereo_sent = last_preview_stats.stereo_sent;
            out.preview_dropped =
                last_preview_stats.priority_dropped + last_preview_stats.stereo_dropped;
        }
        if (start_time.time_since_epoch().count() != 0) {
            out.elapsed_s =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time)
                    .count();
        }
        return out;
    }

    void stop_all() {
        if (msp) {
            try {
                msp->stop();
            } catch (...) {}
        }
        if (camera) {
            try {
                camera->stop();
            } catch (...) {}
        }
        if (preview) {
            try {
                preview->stop();
            } catch (...) {}
        }
    }

    void close_bag() {
        if (!bag) {
            return;
        }

        const std::int64_t end_steady_ns = steady_now_ns();
        const std::int64_t end_system_ns = system_now_ns();
        {
            std::ostringstream clock;
            clock << "{\"event\":\"clock\",\"steady_ns\":" << end_steady_ns
                  << ",\"system_ns\":" << end_system_ns << '}';
            try {
                bag->writeEvent(clock.str(), end_steady_ns);
            } catch (...) {}
        }

        std::ostringstream event;
        event << "{\"event\":\"sensor_stats\"";
        hardware::StereoJpegStats camera_stats;
        bool have_camera_stats = false;
        if (camera) {
            camera_stats = camera->stats();
            have_camera_stats = true;
        }
        if (have_last_camera_stats) {
            camera_stats = last_camera_stats;
            have_camera_stats = true;
        }
        if (have_camera_stats) {
            event << ",\"camera\":" << camera_stats_json(camera_stats);
        }
        LinkStats link;
        bool have_link_stats = false;
        if (msp) {
            link = msp->stats();
            have_link_stats = true;
        }
        if (have_last_link_stats) {
            link = last_link_stats;
            have_link_stats = true;
        }
        if (have_link_stats) {
            event << ",\"msp\":{\"connected\":" << (link.connected ? "true" : "false")
                  << ",\"timeouts\":" << link.timeouts
                  << ",\"checksum_errors\":" << link.checksum_errors
                  << ",\"malformed_frames\":" << link.malformed_frames
                  << ",\"reconnects\":" << link.reconnects
                  << ",\"last_imu_latency_ms\":" << link.last_imu_latency_ms << '}';
        }
        SensorStreamStats stream;
        bool have_stream_stats = false;
        if (preview) {
            stream = preview->stats();
            have_stream_stats = true;
        }
        if (have_last_preview_stats) {
            stream = last_preview_stats;
            stream.connected = false;
            have_stream_stats = true;
        }
        if (have_stream_stats) {
            event << ",\"preview\":{\"connected\":"
                  << (stream.connected ? "true" : "false")
                  << ",\"priority_sent\":" << stream.priority_sent
                  << ",\"stereo_sent\":" << stream.stereo_sent
                  << ",\"dropped\":"
                  << (stream.priority_dropped + stream.stereo_dropped) << '}';
        }
        event << '}';
        try {
            bag->writeEvent(event.str(), steady_now_ns());
        } catch (...) {}

        const RawBagStats bag_stats = bag->close();
        std::printf(
            "raw recorder: wrote %llu IMU samples, %llu images, %llu bytes\n",
            static_cast<unsigned long long>(bag_stats.imu_records),
            static_cast<unsigned long long>(bag_stats.image_records),
            static_cast<unsigned long long>(bag_stats.bytes_written));
        if (bag_stats.dropped_imu != 0 || bag_stats.dropped_images != 0
            || bag_stats.write_errors != 0) {
            std::fprintf(
                stderr, "raw recorder: dropped IMU=%llu images=%llu write_errors=%llu\n",
                static_cast<unsigned long long>(bag_stats.dropped_imu),
                static_cast<unsigned long long>(bag_stats.dropped_images),
                static_cast<unsigned long long>(bag_stats.write_errors));
        }
        bag.reset();
    }
};

RawRecorder::RawRecorder(RawRecorderConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

RawRecorder::~RawRecorder() {
    try {
        stop();
    } catch (...) {}
}

void RawRecorder::start() {
    Impl& impl = *impl_;
    if (impl.started) {
        throw std::runtime_error("raw recorder is already running");
    }
    if (!impl.config.imu_enabled && !impl.config.camera_enabled) {
        throw std::runtime_error("both IMU and camera recording are disabled");
    }
    if (impl.config.camera_enabled && impl.config.camera.fps <= 0) {
        throw std::runtime_error("camera delivery fps must be positive");
    }
    if (impl.config.imu_enabled && impl.config.msp_device.empty()) {
        impl.config.msp_device = kDefaultMspDevice;
    }

    impl.metadata = RawBagMetadata{};
#ifdef FLYRAT_GIT_COMMIT
    impl.metadata.git_commit = FLYRAT_GIT_COMMIT;
#endif
    impl.metadata.imu_enabled = impl.config.imu_enabled;
    impl.metadata.msp_device = impl.config.msp_device;
    impl.metadata.msp_baud = impl.config.msp_baud;
    impl.metadata.imu_rate_hz = impl.config.imu_hz;
    impl.metadata.acc_lsb_per_g = impl.config.acc_lsb_per_g;
    impl.metadata.gyro_lsb_per_dps = impl.config.gyro_lsb_per_dps;
    impl.metadata.accel_axis = impl.config.accel_axis;
    impl.metadata.gyro_axis = impl.config.gyro_axis;
    impl.metadata.camera_enabled = impl.config.camera_enabled;
    impl.metadata.camera_device = impl.config.camera.device;
    impl.metadata.camera_layout = impl.config.camera_layout;
    impl.metadata.capture_fps_requested = impl.config.camera.capture_fps;
    impl.metadata.delivery_fps = impl.config.camera.fps;
    impl.metadata.preview_enabled = impl.config.preview_enabled;
    impl.metadata.preview_address = impl.config.preview_address;
    impl.metadata.preview_port = impl.config.preview_port;

    try {
        if (impl.config.preview_enabled) {
            impl.preview = std::make_unique<SensorStreamServer>("raw recorder preview");
            impl.preview->start(impl.config.preview_address, impl.config.preview_port);
            std::printf(
                "raw recorder: preview listening on %s:%d\n",
                impl.config.preview_address.c_str(), impl.config.preview_port);
        }

        if (impl.config.camera_enabled) {
            impl.camera = std::make_unique<hardware::StereoJpegSource>(impl.config.camera);
            impl.camera->start();
            const hardware::StereoJpegStats stats = impl.camera->stats();
            if (stats.width <= 0 || stats.height <= 0) {
                throw std::runtime_error("camera reported an invalid image size");
            }
            impl.metadata.width = stats.width;
            impl.metadata.height = stats.height;
            impl.metadata.capture_fps_negotiated = stats.capture_fps;
            impl.check_capture_rate(stats);
            std::printf(
                "raw recorder: camera %dx%d capture %.1f fps -> delivery %d fps\n", stats.width,
                stats.height, stats.capture_fps, impl.config.camera.fps);
        }

        impl.metadata.created_unix_ns = system_now_ns();
        impl.metadata.start_steady_ns = steady_now_ns();
        impl.metadata.start_system_ns = system_now_ns();
        impl.bag = std::make_unique<RawBagWriter>();
        impl.bag->open(impl.config.output, impl.metadata);

        if (impl.camera) {
            impl.camera->set_callback(
                [&impl](const hardware::StereoJpegFrame& frame) { impl.on_image(frame); });
            // The callback must be installed after the bag exists, but frames
            // captured before this point are still available through latest();
            // they are intentionally not back-filled.
        }

        if (impl.config.imu_enabled) {
            MspConfig msp_config;
            msp_config.poll = {};
            msp_config.poll.imu_hz = impl.config.imu_hz;
            auto transport = std::make_unique<hardware::SerialTransport>(
                impl.config.msp_device, impl.config.msp_baud);
            impl.msp = std::make_unique<MspInteract>(msp_config, std::move(transport));
            impl.msp->set_imu_callback(
                [&impl](const Sample<ImuSample>& sample) { impl.on_imu(sample); });
            impl.msp->set_link_callback([&impl](bool connected, const std::string& message) {
                impl.on_link(connected, message);
            });
            impl.msp->start();
        }

        impl.start_time = std::chrono::steady_clock::now();
        impl.started = true;
        std::printf("raw recorder: recording to %s\n", impl.bag->path().c_str());
    } catch (...) {
        impl.stop_all();
        impl.close_bag();
        impl.msp.reset();
        impl.camera.reset();
        impl.preview.reset();
        throw;
    }
}

void RawRecorder::stop() {
    Impl& impl = *impl_;
    if (!impl.started && !impl.bag) {
        return;
    }

    if (impl.camera) {
        impl.last_camera_stats = impl.camera->stats();
        impl.have_last_camera_stats = true;
    }
    if (impl.msp) {
        impl.last_link_stats = impl.msp->stats();
        impl.have_last_link_stats = true;
    }
    if (impl.preview) {
        impl.last_preview_stats = impl.preview->stats();
        impl.last_preview_stats.connected = false;
        impl.have_last_preview_stats = true;
    }
    impl.stop_all();
    impl.close_bag();
    impl.msp.reset();
    impl.camera.reset();
    impl.preview.reset();
    impl.started = false;
}

void RawRecorder::check() {
    Impl& impl = *impl_;
    if (!impl.config.imu_enabled && !impl.config.camera_enabled) {
        throw std::runtime_error("both IMU and camera checking are disabled");
    }

    if (impl.config.preview_enabled) {
        SensorStreamServer preview("raw recorder preview");
        preview.start(impl.config.preview_address, impl.config.preview_port);
        std::printf(
            "raw recorder check: preview can listen on %s:%d\n",
            impl.config.preview_address.c_str(), impl.config.preview_port);
        preview.stop();
    }

    if (impl.config.camera_enabled) {
        hardware::StereoJpegSource camera(impl.config.camera);
        camera.start();
        const auto stats = camera.stats();
        impl.check_capture_rate(stats);
        std::printf(
            "raw recorder check: camera %s %dx%d capture %.1f fps, "
            "delivery %d fps, throttled %llu\n",
            impl.config.camera.device.c_str(), stats.width, stats.height, stats.capture_fps,
            impl.config.camera.fps, static_cast<unsigned long long>(stats.throttled));
        camera.stop();
    }

    if (impl.config.imu_enabled) {
        MspConfig msp_config;
        msp_config.poll = {};
        msp_config.poll.imu_hz = impl.config.imu_hz;
        auto transport = std::make_unique<hardware::SerialTransport>(
            impl.config.msp_device, impl.config.msp_baud);
        MspInteract msp(msp_config, std::move(transport));
        std::atomic<int> samples{0};
        msp.set_imu_callback([&samples](const Sample<ImuSample>&) {
            samples.fetch_add(1, std::memory_order_relaxed);
        });
        msp.start();
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        const LinkStats link = msp.stats();
        msp.stop();
        if (samples.load() == 0) {
            throw std::runtime_error("MSP IMU check received no samples");
        }
        std::printf(
            "raw recorder check: MSP %s @ %d, IMU samples %d in 1.5 s, "
            "timeouts %llu, latency %.2f ms\n",
            impl.config.msp_device.c_str(), impl.config.msp_baud, samples.load(),
            static_cast<unsigned long long>(link.timeouts), link.last_imu_latency_ms);
    }
}

RawRecorderStats RawRecorder::stats() const { return impl_->snapshot(); }

std::string RawRecorder::path() const { return impl_->bag ? impl_->bag->path() : std::string(); }

} // namespace flyrat::pilot

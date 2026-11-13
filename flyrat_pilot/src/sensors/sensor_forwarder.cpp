// SPDX-License-Identifier: MIT
#include "sensor_forwarder.hpp"

#include <atomic>
#include <chrono>
#include <iostream>
#include <utility>

#include "flyrat/hardware/serial_transport.hpp"
#include "flyrat/msp_interact.hpp"
#include "flyrat/sensors/protocol.hpp"
#include "flyrat/time.hpp"
#include "sensors/sensor_stream.hpp"

namespace flyrat::pilot {
namespace {

using flyrat::to_ns;

} // namespace

struct SensorForwarder::Impl {
    explicit Impl(SensorForwarderConfig cfg)
        : config(std::move(cfg))
        , stream(std::make_unique<SensorStreamServer>("sensor forwarder"))
    {
    }

    ~Impl()
    {
        stop();
    }

    void start()
    {
        if (running) {
            return;
        }
        stream->start(config.listen_address, config.listen_port);
        try {
            start_sensors();
        } catch (...) {
            stream->stop();
            throw;
        }
        running = true;
    }

    void stop()
    {
        if (!running.exchange(false)) {
            return;
        }
        if (camera) {
            camera->stop();
        }
        if (tof) {
            tof->stop();
        }
        if (msp) {
            msp->stop();
        }
        stream->stop();
    }

    void start_sensors()
    {
        if (config.enable_imu) {
            try {
                MspConfig msp_config;
                msp_config.poll = {};
                msp_config.poll.imu_hz = 200;
                auto transport = std::make_unique<hardware::SerialTransport>(
                    config.msp_device, config.msp_baud);
                msp = std::make_unique<MspInteract>(msp_config, std::move(transport));
                msp->set_imu_callback([this](const Sample<ImuSample>& sample) {
                    on_imu(sample);
                });
                msp->start();
            } catch (const std::exception& error) {
                std::cerr << "sensor forwarder: IMU disabled: " << error.what() << '\n';
                msp.reset();
            }
        }

        if (config.enable_tof) {
            try {
                tof = std::make_unique<hardware::TofArray>(config.tof);
                tof->set_callback([this](const hardware::TofFrame& frame) {
                    on_tof(frame);
                });
                tof->start();
            } catch (const std::exception& error) {
                std::cerr << "sensor forwarder: ToF disabled: " << error.what() << '\n';
                tof.reset();
            }
        }

        if (config.enable_stereo) {
            try {
                camera = std::make_unique<hardware::StereoJpegSource>(config.camera);
                camera->set_callback([this](const hardware::StereoJpegFrame& frame) {
                    on_stereo(frame);
                });
                camera->start();
                const auto camera_stats = camera->stats();
                std::cerr << "sensor forwarder: stereo " << config.camera.width
                          << "x" << config.camera.height << " target "
                          << config.camera.fps << " fps, capture "
                          << camera_stats.capture_fps << " fps\n";
            } catch (const std::exception& error) {
                std::cerr << "sensor forwarder: stereo disabled: " << error.what() << '\n';
                camera.reset();
            }
        }
    }

    void on_imu(const Sample<ImuSample>& sample)
    {
        sensors::ImuPayload payload;
        payload.acc = sample.value.acc;
        payload.gyro = sample.value.gyro;
        payload.mag = sample.value.mag;
        stream->send_imu(payload, to_ns(sample.stamp));
    }

    void on_tof(const hardware::TofFrame& frame)
    {
        sensors::TofPayload payload;
        payload.sensor_id = static_cast<std::uint8_t>(frame.sensor);
        payload.resolution = static_cast<std::uint8_t>(frame.resolution);
        payload.temperature_c = frame.silicon_temp_degc;
        for (std::size_t i = 0; i < payload.zones.size(); ++i) {
            payload.zones[i].distance_mm = frame.zones[i].distance_mm;
            payload.zones[i].target_status = frame.zones[i].target_status;
            payload.zones[i].nb_target = frame.zones[i].nb_target;
        }
        stream->send_tof(payload, to_ns(frame.stamp));
    }

    void on_stereo(const hardware::StereoJpegFrame& frame)
    {
        stream->send_stereo(static_cast<std::uint32_t>(frame.width),
            static_cast<std::uint32_t>(frame.height),
            frame.jpeg.data(), frame.jpeg.size(), to_ns(frame.stamp));
    }

    SensorForwarderConfig config;
    std::atomic<bool> running { false };
    std::unique_ptr<SensorStreamServer> stream;
    std::unique_ptr<MspInteract> msp;
    std::unique_ptr<hardware::TofArray> tof;
    std::unique_ptr<hardware::StereoJpegSource> camera;
};

SensorForwarder::SensorForwarder(SensorForwarderConfig config)
    : impl_(std::make_unique<Impl>(std::move(config)))
{
}

SensorForwarder::~SensorForwarder() = default;

void SensorForwarder::start()
{
    impl_->start();
}

void SensorForwarder::stop()
{
    impl_->stop();
}

bool SensorForwarder::running() const
{
    return impl_->running;
}

} // namespace flyrat::pilot

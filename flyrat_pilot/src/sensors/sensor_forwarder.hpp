// SPDX-License-Identifier: MIT
//
// Forwards real vehicle sensors to a development machine over a TCP tunnel
// (normally `adb forward tcp:5000 tcp:5000`).
//
// The forwarder is deliberately a separate program from the flight runtime:
// IMU, four ToF sensors and the stereo camera can be streamed while the
// flight-control code is still being developed. Sensor callbacks only encode
// and enqueue; a dedicated writer thread performs socket I/O, so a slow host
// can drop stereo frames without stalling the camera or ToF readers.
#pragma once

#include <memory>
#include <string>

#include "flyrat/hardware/stereo_jpeg_source.hpp"
#include "flyrat/hardware/tof.hpp"

namespace flyrat::pilot {

struct SensorForwarderConfig {
    std::string listen_address = "127.0.0.1";
    int listen_port = 5000;

    bool enable_imu = true;
    std::string msp_device = "/dev/ttyS3";
    int msp_baud = 921600;

    bool enable_tof = true;
    hardware::TofConfig tof;

    bool enable_stereo = true;
    hardware::StereoJpegConfig camera;
};

class SensorForwarder {
public:
    explicit SensorForwarder(SensorForwarderConfig config);
    ~SensorForwarder();

    SensorForwarder(const SensorForwarder&) = delete;
    SensorForwarder& operator=(const SensorForwarder&) = delete;

    // Opens the listening socket and starts the enabled sensors. Sensor
    // failures are reported on stderr but do not prevent the other sensors
    // from being forwarded.
    void start();
    void stop();

    bool running() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::pilot

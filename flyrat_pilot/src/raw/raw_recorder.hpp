// SPDX-License-Identifier: MIT
//
// Raw sensor recorder: MSP IMU counts plus native MJPEG frames, written to a
// RawBag without applying any camera or IMU calibration.
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "flyrat/hardware/stereo_jpeg_source.hpp"

namespace flyrat::pilot {

struct RawRecorderConfig {
    std::string output;

    bool imu_enabled = true;
    std::string msp_device = "/dev/ttyS3";
    int msp_baud = 921600;
    int imu_hz = 200;
    double acc_lsb_per_g = 2048.0;
    double gyro_lsb_per_dps = 16.4;
    // Recorded as metadata only. The raw recorder never rotates or flips the
    // IMU; the offline export step can apply a sign convention explicitly.
    std::string accel_axis = "none";
    std::string gyro_axis = "none";

    bool camera_enabled = true;
    hardware::StereoJpegConfig camera;
    std::string camera_layout = "side_by_side";
    // Fail instead of continuing when the camera cannot negotiate the requested
    // capture rate.
    bool require_capture_fps = false;

    // Stream the same raw samples to a host while recording. The stream uses
    // the sensor forwarder protocol, so sensor_receiver_node and Foxglove can
    // display it without touching the bag.
    bool preview_enabled = false;
    std::string preview_address = "127.0.0.1";
    int preview_port = 5000;
};

struct RawRecorderStats {
    std::uint64_t imu_samples = 0;
    std::uint64_t images = 0;
    std::uint64_t dropped_imu = 0;
    std::uint64_t dropped_images = 0;
    std::uint64_t bytes = 0;
    std::uint64_t throttled = 0;
    std::uint64_t driver_dropped = 0;
    double elapsed_s = 0.0;
    double capture_fps = 0.0;
    bool imu_connected = false;

    bool preview_connected = false;
    std::uint64_t preview_priority_sent = 0;
    std::uint64_t preview_stereo_sent = 0;
    std::uint64_t preview_dropped = 0;
    bool preview_enabled = false;
};

class RawRecorder {
public:
    explicit RawRecorder(RawRecorderConfig config);
    ~RawRecorder();

    RawRecorder(const RawRecorder&) = delete;
    RawRecorder& operator=(const RawRecorder&) = delete;

    // Opens the camera and MSP link, then starts writing. Throws on failure.
    void start();
    // Stops capture, drains the writer and closes the bag. Safe to call twice.
    void stop();

    // Opens both devices briefly and reports what they negotiated. Does not
    // create a bag. Throws when a requested device is unavailable.
    void check();

    RawRecorderStats stats() const;
    std::string path() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::pilot

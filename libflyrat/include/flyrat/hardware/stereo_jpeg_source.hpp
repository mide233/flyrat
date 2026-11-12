// SPDX-License-Identifier: MIT
//
// Raw MJPEG capture for the side-by-side USB stereo camera.
//
// This source deliberately does not decode the JPEG. The sensor forwarder
// sends the camera's native MJPEG frames to the development machine, where
// OpenCV decodes and splits them. At 1280x480@60 this is about 3 MB/s instead
// of 55 MB/s for raw NV12, which matters because the adb tunnel tops out near
// 30 MB/s.
//
// The capture rate is always requested at the highest rate the camera supports
// (or `capture_fps` when set). `fps` is the delivery rate: excess frames are
// discarded before the JPEG is copied. Running the sensor at its maximum rate
// also caps the automatic exposure time to one capture interval, which keeps
// motion blur short in low light.
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace flyrat::hardware {

struct StereoJpegConfig {
    std::string device = "/dev/video10";
    int width = 1280; // full side-by-side width
    int height = 480;
    // Delivered frames per second. Captured frames above this rate are
    // dropped before they are copied; 0 would mean "deliver every frame", but
    // start() requires a positive value.
    int fps = 60;
    // Requested capture rate. 0 selects the highest rate the camera reports
    // for the configured mode.
    int capture_fps = 0;
    int buffer_count = 4;
};

struct StereoJpegFrame {
    int width = 0; // full side-by-side width
    int height = 0;
    std::vector<std::uint8_t> jpeg;
    std::uint64_t sequence = 0;
    std::chrono::steady_clock::time_point stamp;
};

struct StereoJpegStats {
    std::uint64_t frames = 0;
    std::uint64_t dropped = 0;
    std::uint64_t errors = 0;
    // Frames skipped by the delivery-rate throttle (not driver drops).
    std::uint64_t throttled = 0;
    double fps = 0.0;
    // Capture rate negotiated with V4L2 (may be the camera maximum).
    double capture_fps = 0.0;
    int width = 0;
    int height = 0;
    bool running = false;
};

class StereoJpegError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class StereoJpegSource {
public:
    explicit StereoJpegSource(StereoJpegConfig config);
    ~StereoJpegSource();

    StereoJpegSource(const StereoJpegSource&) = delete;
    StereoJpegSource& operator=(const StereoJpegSource&) = delete;

    void start();
    void stop();

    bool running() const;
    std::optional<StereoJpegFrame> latest() const;
    // Moves the newest frame out without copying the JPEG payload. Returns
    // nullopt when no new frame is available.
    std::optional<StereoJpegFrame> takeLatest();
    StereoJpegStats stats() const;

    // Runs on the capture thread. It must not block or call stop().
    void set_callback(std::function<void(const StereoJpegFrame&)> callback);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::hardware

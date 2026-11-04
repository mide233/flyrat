// SPDX-License-Identifier: MIT
//
// Raw MJPEG capture for the side-by-side USB stereo camera.
//
// This source deliberately does not decode the JPEG. The sensor forwarder
// sends the camera's native MJPEG frames to the development machine, where
// OpenCV decodes and splits them. At 1280x480@60 this is about 3 MB/s instead
// of 55 MB/s for raw NV12, which matters because the adb tunnel tops out near
// 30 MB/s.
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
    int fps = 60;
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
    double fps = 0.0;
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
    StereoJpegStats stats() const;

    // Runs on the capture thread. It must not block or call stop().
    void set_callback(std::function<void(const StereoJpegFrame&)> callback);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::hardware

// SPDX-License-Identifier: MIT
//
// Raw MJPEG capture for the USB monochrome camera.
//
// This source deliberately does not decode the JPEG. The sensor forwarder
// sends the camera's native MJPEG frames to the development machine, where
// OpenCV decodes them. QVGA (320x240) mono frames are small enough for the
// adb tunnel, and the capture loop throttles to the requested output rate
// because some UVC cameras only advertise high-speed modes (120/210 Hz).
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

struct MonoJpegConfig {
    std::string device = "/dev/video10";
    int width = 320;
    int height = 240;
    int fps = 15;
    int buffer_count = 4;
};

struct MonoJpegFrame {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> jpeg;
    std::uint64_t sequence = 0;
    std::chrono::steady_clock::time_point stamp;
};

struct MonoJpegStats {
    std::uint64_t frames = 0;
    std::uint64_t dropped = 0;
    std::uint64_t errors = 0;
    double fps = 0.0;
    int width = 0;
    int height = 0;
    bool running = false;
};

class MonoJpegError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class MonoJpegSource {
public:
    explicit MonoJpegSource(MonoJpegConfig config);
    ~MonoJpegSource();

    MonoJpegSource(const MonoJpegSource&) = delete;
    MonoJpegSource& operator=(const MonoJpegSource&) = delete;

    void start();
    void stop();

    bool running() const;
    std::optional<MonoJpegFrame> latest() const;
    MonoJpegStats stats() const;

    // Runs on the capture thread. It must not block or call stop().
    void set_callback(std::function<void(const MonoJpegFrame&)> callback);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::hardware

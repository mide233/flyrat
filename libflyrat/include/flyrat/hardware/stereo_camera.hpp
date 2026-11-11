// SPDX-License-Identifier: MIT
//
// Side-by-side USB stereo camera capture through V4L2.
//
// The DECXIN module used by the flight stack exposes one UVC video node whose
// frame contains the left and right images next to each other. A single frame
// therefore carries both eyes with one USB timestamp, so the two images are
// inherently synchronised even though the module has no trigger input.
//
// This class captures raw YUYV only. MJPEG decoding lives in
// `StereoJpegSource` (capture) and `MppJpegDecoder` (VPU decode) so that
// decoding can run on its own thread instead of the capture thread.
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

enum class StereoPixelFormat {
    kYuyv, // packed YUYV 4:2:2
    kGray8, // one byte per pixel
};

struct StereoConfig {
    std::string device = "/dev/video10";
    // Width of the full side-by-side frame, not of one eye. The driver is
    // asked for exactly this mode; width must be even.
    int width = 1280;
    int height = 480;
    int fps = 15;
    int buffer_count = 4;
    // Extracts luma into GRAY8 instead of returning packed YUYV.
    bool grayscale = false;
};

struct StereoFrame {
    int width = 0; // per eye
    int height = 0;
    // Bytes per row of each eye plane.
    int stride = 0;
    StereoPixelFormat format = StereoPixelFormat::kGray8;
    std::vector<std::uint8_t> left;
    std::vector<std::uint8_t> right;
    std::uint64_t sequence = 0;
    // V4L2 buffer timestamp.
    std::chrono::steady_clock::time_point stamp;
};

struct StereoStats {
    std::uint64_t frames = 0;
    std::uint64_t dropped = 0;
    std::uint64_t errors = 0;
    double fps = 0.0;
    int width = 0; // per eye
    int height = 0;
    bool running = false;
};

class StereoError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class StereoCamera {
public:
    explicit StereoCamera(StereoConfig config);
    ~StereoCamera();

    StereoCamera(const StereoCamera&) = delete;
    StereoCamera& operator=(const StereoCamera&) = delete;

    // Opens the device, negotiates the mode and starts streaming. Throws
    // StereoError if the device cannot be opened or the mode is unsupported.
    void start();
    void stop();

    bool running() const;
    std::optional<StereoFrame> latest() const;
    // Moves the newest decoded frame out without copying the two eye planes.
    std::optional<StereoFrame> takeLatest();
    StereoStats stats() const;

    // Runs on the capture thread. It must not block or call stop().
    void set_callback(std::function<void(const StereoFrame&)> callback);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::hardware

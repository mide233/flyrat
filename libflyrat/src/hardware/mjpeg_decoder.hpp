// SPDX-License-Identifier: MIT
//
// Private MJPEG decoder backend for StereoCamera.
//
// Rockchip ships a GStreamer plugin named mppjpegdec that decodes JPEG on the
// VPU. The target image has the runtime libraries but no GStreamer development
// headers, so the handful of stable C entry points we need are resolved with
// dlopen(). NV12 is requested from the decoder and only its Y plane is kept;
// the Y plane is exactly the GRAY8 image OpenCV/ORB-SLAM3 wants, without the
// expensive videoconvert -> GRAY8 step.
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "flyrat/hardware/stereo_camera.hpp"

namespace flyrat::hardware::detail {

struct MjpegConfig {
    std::string device;
    int width = 1280;
    int height = 480;
    int fps = 60;
    int max_buffers = 2;
};

struct MjpegFrame {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> luma;
    std::uint64_t sequence = 0;
    std::chrono::steady_clock::time_point stamp;
};

enum class MjpegReadStatus {
    kFrame,
    kTimeout,
    kEos,
    kError,
};

class MjpegDecoder {
public:
    explicit MjpegDecoder(MjpegConfig config);
    ~MjpegDecoder();

    MjpegDecoder(const MjpegDecoder&) = delete;
    MjpegDecoder& operator=(const MjpegDecoder&) = delete;

    // Starts the GStreamer pipeline. Throws StereoError when the runtime
    // libraries, the mppjpegdec plugin or the selected camera mode is missing.
    void start();
    void stop();
    bool running() const;

    // Blocks for at most a short timeout so the capture loop can observe stop().
    MjpegReadStatus read(MjpegFrame& frame);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::hardware::detail

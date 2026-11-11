// SPDX-License-Identifier: MIT
//
// Rockchip MPP JPEG decoder.
//
// The RK3566 VPU decodes MJPEG through librockchip_mpp. The Buildroot image
// ships the runtime library but not its headers, so the C API is resolved with
// dlopen() and only the small MppApi prefix that is needed is declared here.
// This keeps host builds free of an MPP dependency while allowing the pilot to
// decode on the VPU.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace flyrat::hardware {

// Luma plane of a decoded JPEG. The VPU may output NV12 or NV16; only the Y
// plane is kept because the stereo pipeline consumes grayscale images.
struct DecodedJpeg {
    int width = 0;
    int height = 0;
    // Bytes per luma row, at least width.
    int stride = 0;
    std::vector<std::uint8_t> luma;
};

class MppJpegDecoder {
public:
    MppJpegDecoder();
    ~MppJpegDecoder();

    MppJpegDecoder(const MppJpegDecoder&) = delete;
    MppJpegDecoder& operator=(const MppJpegDecoder&) = delete;

    // Loads librockchip_mpp and opens a JPEG decoder context. Returns false
    // when the library or the decoder is unavailable; lastError() then
    // describes the reason.
    bool start();
    void stop();

    // Decodes one complete JPEG synchronously. The output frame is reused
    // between calls, so the luma plane is reallocated only when the image size
    // changes. Returns false on malformed input or decoder failure.
    bool decode(const std::uint8_t* jpeg, std::size_t size, DecodedJpeg& out);

    bool available() const;
    const std::string& lastError() const;

    std::uint64_t frames() const;
    std::uint64_t errors() const;
    // MPP pixel format of the last decoded frame (0 = NV12, 2 = NV16).
    int lastFormat() const;
    // Number of times the decoder asked for new output buffers.
    std::uint64_t infoChanges() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::hardware

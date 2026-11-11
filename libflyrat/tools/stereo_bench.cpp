// SPDX-License-Identifier: MIT
//
// Acceptance tool for the side-by-side USB stereo camera.
//
//   stereo_bench --device /dev/video10 --width 1280 --height 480 --fps 15
//
// It captures for a few seconds, prints frame rate and drop counters, then
// writes the latest left and right images as PGM files for visual inspection.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "flyrat/hardware/stereo_camera.hpp"

namespace {

struct Options {
    std::string device = "/dev/video10";
    std::string save_prefix = "/tmp/stereo";
    int width = 1280;
    int height = 480;
    int fps = 15;
    int seconds = 5;
    bool grayscale = false;
};

void usage()
{
    std::printf("usage: stereo_bench [--device /dev/video10] [--width 1280] "
                "[--height 480] [--fps 15] [--seconds 5] [--gray] "
                "[--save /tmp/stereo]\n");
}

const char* format_name(flyrat::hardware::StereoPixelFormat format)
{
    switch (format) {
    case flyrat::hardware::StereoPixelFormat::kYuyv:
        return "yuyv";
    case flyrat::hardware::StereoPixelFormat::kGray8:
        return "gray8";
    }
    return "unknown";
}

std::vector<std::uint8_t> yuyv_to_gray(const flyrat::hardware::StereoFrame& frame)
{
    std::vector<std::uint8_t> gray(
        static_cast<std::size_t>(frame.width) * frame.height);
    for (int y = 0; y < frame.height; ++y) {
        const std::uint8_t* row = frame.left.data() +
            static_cast<std::size_t>(y) * frame.stride;
        std::uint8_t* out = gray.data() + static_cast<std::size_t>(y) * frame.width;
        for (int x = 0; x < frame.width; x += 2) {
            out[x] = row[x * 2];
            out[x + 1] = row[x * 2 + 2];
        }
    }
    return gray;
}

bool write_pgm(const std::string& path, const flyrat::hardware::StereoFrame& frame,
    bool left)
{
    const std::uint8_t* data = left ? frame.left.data() : frame.right.data();
    std::vector<std::uint8_t> gray;
    if (frame.format == flyrat::hardware::StereoPixelFormat::kYuyv) {
        flyrat::hardware::StereoFrame one_eye = frame;
        one_eye.left.assign(data, data + frame.stride * frame.height);
        gray = yuyv_to_gray(one_eye);
        data = gray.data();
    }

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }
    out << "P5\n" << frame.width << ' ' << frame.height << "\n255\n";
    out.write(reinterpret_cast<const char*>(data),
        static_cast<std::streamsize>(frame.width) * frame.height);
    return out.good();
}

} // namespace

int main(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--device") {
            options.device = next("--device");
        } else if (arg == "--width") {
            options.width = std::atoi(next("--width").c_str());
        } else if (arg == "--height") {
            options.height = std::atoi(next("--height").c_str());
        } else if (arg == "--fps") {
            options.fps = std::atoi(next("--fps").c_str());
        } else if (arg == "--seconds") {
            options.seconds = std::atoi(next("--seconds").c_str());
        } else if (arg == "--save") {
            options.save_prefix = next("--save");
        } else if (arg == "--gray") {
            options.grayscale = true;
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
            usage();
            return 2;
        }
    }

    if (options.seconds <= 0 || options.width <= 0 || options.height <= 0) {
        std::fprintf(stderr, "seconds, width and height must be positive\n");
        return 2;
    }

    flyrat::hardware::StereoConfig config;
    config.device = options.device;
    config.width = options.width;
    config.height = options.height;
    config.fps = options.fps;
    config.grayscale = options.grayscale;

    try {
        flyrat::hardware::StereoCamera camera(config);
        camera.start();
        std::this_thread::sleep_for(std::chrono::seconds(options.seconds));
        const auto stats = camera.stats();
        const auto frame = camera.latest();
        camera.stop();

        std::printf("device=%s source=yuyv mode=%dx%d@%d eye=%dx%d\n",
            options.device.c_str(), options.width, options.height, options.fps,
            stats.width, stats.height);
        std::printf("frames=%llu dropped=%llu errors=%llu fps=%.2f\n",
            static_cast<unsigned long long>(stats.frames),
            static_cast<unsigned long long>(stats.dropped),
            static_cast<unsigned long long>(stats.errors), stats.fps);

        if (!frame) {
            std::fprintf(stderr, "no stereo frame captured\n");
            return 1;
        }
        std::printf("latest: sequence=%llu eye=%dx%d stride=%d pixel=%s stamp_us=%lld\n",
            static_cast<unsigned long long>(frame->sequence), frame->width,
            frame->height, frame->stride, format_name(frame->format),
            static_cast<long long>(std::chrono::duration_cast<std::chrono::microseconds>(
                frame->stamp.time_since_epoch()).count()));

        const std::string left_path = options.save_prefix + "_left.pgm";
        const std::string right_path = options.save_prefix + "_right.pgm";
        if (!write_pgm(left_path, *frame, true) || !write_pgm(right_path, *frame, false)) {
            std::fprintf(stderr, "failed to write %s / %s\n",
                left_path.c_str(), right_path.c_str());
            return 1;
        }
        std::printf("saved=%s,%s\n", left_path.c_str(), right_path.c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "stereo capture failed: %s\n", e.what());
        return 1;
    }
    return 0;
}

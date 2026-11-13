// SPDX-License-Identifier: MIT
//
// Continuous hardware JPEG decode acceptance tool.
//
//   decode_bench --camera /dev/video10 --seconds 60 --mode both
//
// Captures MJPEG from the real camera and decodes every frame with the
// Rockchip VPU, optionally comparing against OpenCV's software decoder. It is
// the smallest end-to-end check that the VPU path can run continuously without
// stalling the UVC stream, and it reports the numbers used to decide whether
// the pilot should ever enable hardware decode.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <sys/resource.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "flyrat/hardware/mpp_jpeg_decoder.hpp"
#include "flyrat/hardware/stereo_jpeg_source.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct Options {
    std::string device = "/dev/video10";
    std::string mode = "both";
    int width = 1280;
    int height = 480;
    int fps = 25;
    int seconds = 60;
    int verify_every = 25;
};

void usage()
{
    std::printf(
        "usage: decode_bench [--camera /dev/video10] [--mode mpp|cpu|both]\n"
        "                    [--width 1280] [--height 480] [--fps 25]\n"
        "                    [--seconds 60] [--verify-every 25]\n"
        "  --fps is the delivered rate; capture always uses the sensor maximum.\n");
}

std::string format_name(int value)
{
    if (value == 0) {
        return "nv12";
    }
    if (value == 2) {
        return "nv16";
    }
    char buffer[16];
    std::snprintf(buffer, sizeof(buffer), "0x%x", value);
    return buffer;
}

double process_cpu_seconds()
{
    rusage usage {};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        return 0.0;
    }
    return static_cast<double>(usage.ru_utime.tv_sec + usage.ru_stime.tv_sec)
        + static_cast<double>(usage.ru_utime.tv_usec + usage.ru_stime.tv_usec)
            / 1e6;
}

double percentile(const std::vector<double>& values, double fraction)
{
    if (values.empty()) {
        return 0.0;
    }
    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());
    const std::size_t index = static_cast<std::size_t>(
        fraction * static_cast<double>(sorted.size() - 1));
    return sorted[index];
}

double average(const std::vector<double>& values)
{
    if (values.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}

double maximum(const std::vector<double>& values)
{
    if (values.empty()) {
        return 0.0;
    }
    return *std::max_element(values.begin(), values.end());
}

struct DecodeStats {
    std::uint64_t frames = 0;
    std::uint64_t errors = 0;
    std::vector<double> ms;
};

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
        if (arg == "--camera" || arg == "--device") {
            options.device = next(arg.c_str());
        } else if (arg == "--mode") {
            options.mode = next("--mode");
        } else if (arg == "--width") {
            options.width = std::atoi(next("--width").c_str());
        } else if (arg == "--height") {
            options.height = std::atoi(next("--height").c_str());
        } else if (arg == "--fps") {
            options.fps = std::atoi(next("--fps").c_str());
        } else if (arg == "--seconds") {
            options.seconds = std::atoi(next("--seconds").c_str());
        } else if (arg == "--verify-every") {
            options.verify_every = std::atoi(next("--verify-every").c_str());
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
            usage();
            return 2;
        }
    }

    if (options.mode != "mpp" && options.mode != "cpu" && options.mode != "both") {
        std::fprintf(stderr, "--mode must be mpp, cpu or both\n");
        return 2;
    }
    if (options.seconds <= 0 || options.width <= 0 || options.height <= 0
        || options.fps <= 0 || options.verify_every < 0) {
        std::fprintf(stderr, "invalid --seconds/--width/--height/--fps/--verify-every\n");
        return 2;
    }

    const bool want_mpp = options.mode != "cpu";
    const bool want_cpu = options.mode != "mpp";

    flyrat::hardware::MppJpegDecoder mpp;
    if (want_mpp && !mpp.start()) {
        std::fprintf(stderr, "MPP JPEG decoder unavailable: %s\n",
            mpp.lastError().c_str());
        return 2;
    }

    flyrat::hardware::StereoJpegConfig config;
    config.device = options.device;
    config.width = options.width;
    config.height = options.height;
    config.fps = options.fps;

    DecodeStats mpp_stats;
    DecodeStats cpu_stats;
    std::uint64_t frames = 0;
    std::uint64_t gaps = 0;
    std::uint64_t verified = 0;
    double verify_mean_sum = 0.0;
    double verify_mean_worst = 0.0;
    double verify_max = 0.0;
    std::string first_mpp_error;
    std::uint64_t source_drops = 0;
    std::uint64_t source_errors = 0;

    try {
        flyrat::hardware::StereoJpegSource source(config);
        source.start();

        const double cpu_start = process_cpu_seconds();
        const auto start = Clock::now();
        auto last_stamp = std::optional<Clock::time_point>();
        auto last_report = start;
        double decode_sum = 0.0;

        while (true) {
            const double elapsed = std::chrono::duration<double>(
                Clock::now() - start).count();
            if (elapsed >= static_cast<double>(options.seconds)) {
                break;
            }

            auto frame = source.takeLatest();
            if (!frame) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            if (last_stamp && elapsed > 2.0
                && frame->stamp - *last_stamp > std::chrono::milliseconds(100)) {
                ++gaps;
            }
            last_stamp = frame->stamp;
            ++frames;

            if (frame->width != options.width || frame->height != options.height) {
                std::fprintf(stderr, "camera returned %dx%d, expected %dx%d\n",
                    frame->width, frame->height, options.width, options.height);
                return 1;
            }

            flyrat::hardware::DecodedJpeg decoded;
            if (want_mpp) {
                const auto begin = Clock::now();
                const bool ok = mpp.decode(
                    frame->jpeg.data(), frame->jpeg.size(), decoded);
                const double ms = std::chrono::duration<double, std::milli>(
                    Clock::now() - begin).count();
                mpp_stats.ms.push_back(ms);
                decode_sum += ms;
                if (!ok) {
                    ++mpp_stats.errors;
                    if (first_mpp_error.empty()) {
                        first_mpp_error = mpp.lastError();
                    }
                } else {
                    ++mpp_stats.frames;
                    if (decoded.width != options.width
                        || decoded.height != options.height) {
                        std::fprintf(stderr,
                            "MPP returned %dx%d, expected %dx%d\n",
                            decoded.width, decoded.height, options.width,
                            options.height);
                        ++mpp_stats.errors;
                    }
                }
            }

            if (want_cpu) {
                cv::Mat encoded(1, static_cast<int>(frame->jpeg.size()), CV_8UC1,
                    const_cast<std::uint8_t*>(frame->jpeg.data()));
                const auto begin = Clock::now();
                cv::Mat image = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
                const double ms = std::chrono::duration<double, std::milli>(
                    Clock::now() - begin).count();
                cpu_stats.ms.push_back(ms);
                if (image.empty() || image.cols != options.width
                    || image.rows != options.height) {
                    ++cpu_stats.errors;
                } else {
                    ++cpu_stats.frames;
                    const bool verify = options.verify_every > 0
                        && mpp_stats.frames > 0
                        && (frames % static_cast<std::uint64_t>(
                                options.verify_every) == 0);
                    if (verify) {
                        cv::Mat luma(decoded.height, decoded.width, CV_8UC1,
                            decoded.luma.data(),
                            static_cast<std::size_t>(decoded.stride));
                        cv::Mat diff;
                        cv::absdiff(luma, image, diff);
                        const double mean = cv::mean(diff)[0];
                        double worst = 0.0;
                        cv::minMaxLoc(diff, nullptr, &worst);
                        ++verified;
                        verify_mean_sum += mean;
                        verify_mean_worst = std::max(verify_mean_worst, mean);
                        verify_max = std::max(verify_max, worst);
                    }
                }
            }

            const auto report_now = Clock::now();
            if (report_now - last_report >= std::chrono::seconds(5)) {
                last_report = report_now;
                std::printf("frames=%llu camera_fps=%.2f",
                    static_cast<unsigned long long>(frames),
                    static_cast<double>(frames) / elapsed);
                if (want_mpp) {
                    std::printf(" mpp_avg_ms=%.2f mpp_errors=%llu",
                        decode_sum / static_cast<double>(frames),
                        static_cast<unsigned long long>(mpp_stats.errors));
                }
                std::printf(" gaps=%llu\n",
                    static_cast<unsigned long long>(gaps));
                std::fflush(stdout);
            }
        }

        const double cpu_used = process_cpu_seconds() - cpu_start;
        const auto stats = source.stats();
        source.stop();

        const double wall = std::chrono::duration<double>(Clock::now() - start).count();
        source_drops = stats.dropped;
        source_errors = stats.errors;

        std::printf("\ndevice=%s mode=%s %dx%d@%d (capture %.0f) wall=%.1fs\n",
            options.device.c_str(), options.mode.c_str(), options.width,
            options.height, options.fps, stats.capture_fps, wall);
        std::printf("frames=%llu camera_fps=%.2f gaps_gt_100ms=%llu "
                    "source_drops=%llu source_errors=%llu throttled=%llu "
                    "cpu=%.1f%%\n",
            static_cast<unsigned long long>(frames),
            static_cast<double>(frames) / wall,
            static_cast<unsigned long long>(gaps),
            static_cast<unsigned long long>(source_drops),
            static_cast<unsigned long long>(source_errors),
            static_cast<unsigned long long>(stats.throttled),
            cpu_used / wall * 100.0);

        if (want_mpp) {
            std::printf("mpp: ok=%llu errors=%llu avg_ms=%.2f p99_ms=%.2f "
                        "max_ms=%.2f format=%s info_changes=%llu\n",
                static_cast<unsigned long long>(mpp_stats.frames),
                static_cast<unsigned long long>(mpp_stats.errors),
                average(mpp_stats.ms), percentile(mpp_stats.ms, 0.99),
                maximum(mpp_stats.ms), format_name(mpp.lastFormat()).c_str(),
                static_cast<unsigned long long>(mpp.infoChanges()));
            if (!first_mpp_error.empty()) {
                std::printf("mpp error: %s\n", first_mpp_error.c_str());
            }
        }
        if (want_cpu) {
            std::printf("cpu: ok=%llu errors=%llu avg_ms=%.2f p99_ms=%.2f "
                        "max_ms=%.2f\n",
                static_cast<unsigned long long>(cpu_stats.frames),
                static_cast<unsigned long long>(cpu_stats.errors),
                average(cpu_stats.ms), percentile(cpu_stats.ms, 0.99),
                maximum(cpu_stats.ms));
        }
        if (verified > 0) {
            std::printf("verify: checks=%llu mean_abs=%.2f worst_mean=%.2f "
                        "max_abs=%.0f\n",
                static_cast<unsigned long long>(verified),
                verify_mean_sum / static_cast<double>(verified),
                verify_mean_worst, verify_max);
        }

        const std::uint64_t expected = static_cast<std::uint64_t>(
            options.seconds) * static_cast<std::uint64_t>(options.fps);
        std::vector<std::string> failures;
        if (frames * 100 < expected * 98) {
            failures.push_back("captured fewer than 98% of the expected frames");
        }
        if (gaps != 0) {
            failures.push_back("camera stalled for more than 100 ms");
        }
        if (want_mpp && mpp_stats.errors != 0) {
            failures.push_back("MPP reported decode errors");
        }
        if (want_cpu && cpu_stats.errors != 0) {
            failures.push_back("OpenCV reported decode errors");
        }
        if (verified > 0
            && verify_mean_sum / static_cast<double>(verified) >= 2.0) {
            failures.push_back("MPP output differs from OpenCV by 2 LSB or more");
        }
        if (want_mpp) {
            const double mpp_p99 = percentile(mpp_stats.ms, 0.99);
            if (want_cpu && !cpu_stats.ms.empty()) {
                if (mpp_p99 >= percentile(cpu_stats.ms, 0.99)) {
                    failures.push_back(
                        "MPP p99 is not lower than the software decoder");
                }
            } else if (mpp_p99 >= 15.0) {
                failures.push_back("MPP p99 decode time is 15 ms or more");
            }
        }

        if (failures.empty()) {
            std::printf("result: PASS\n");
            return 0;
        }
        std::printf("result: FAIL\n");
        for (const std::string& failure : failures) {
            std::printf("  - %s\n", failure.c_str());
        }
        return 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "decode_bench failed: %s\n", e.what());
        return 1;
    }
}

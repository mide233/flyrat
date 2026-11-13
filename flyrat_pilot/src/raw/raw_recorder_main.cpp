// SPDX-License-Identifier: MIT
//
// flyrat_raw_recorder: capture MSP IMU counts and native MJPEG frames without
// applying the calibration used by the live forwarded sensor topics.
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <stdexcept>
#include <string>
#include <thread>

#include "raw/raw_bag.hpp"
#include "raw/raw_recorder.hpp"

namespace {

volatile std::sig_atomic_t g_stop = 0;

void handle_signal(int) { g_stop = 1; }

void usage(const char* program) {
    std::printf(
        "Usage: %s --output DIR [options]\n"
        "\n"
        "Records raw MSP IMU counts and native MJPEG stereo frames. No IMU\n"
        "intrinsics, axis mapping, time offset, undistortion or splitting is\n"
        "applied. Use tools/raw_export.py to produce Kalibr input.\n"
        "\n"
        "Output:\n"
        "  --output DIR                 directory for raw_sensors.bin\n"
        "  --duration SECONDS           stop after this many seconds (s/m/h suffix; 0 = until "
        "SIGINT)\n"
        "  --max-bytes BYTES            stop after this many bytes (K/M/G suffix)\n"
        "\n"
        "IMU:\n"
        "  --msp-device DEV             MSP serial device (default /dev/ttyS3)\n"
        "  --msp-baud BAUD              MSP baud rate (default 921600)\n"
        "  --imu-hz HZ                  MSP_RAW_IMU poll rate (default 200)\n"
        "  --no-imu                     do not record IMU\n"
        "  --acc-lsb-per-g VALUE        metadata only (default 2048)\n"
        "  --gyro-lsb-per-dps VALUE     metadata only (default 16.4)\n"
        "  --accel-axis NAME            metadata only: none|frd_to_flu\n"
        "  --gyro-axis NAME             metadata only: none|frd_to_flu\n"
        "\n"
        "Camera:\n"
        "  --camera-device DEV          V4L2 device (default /dev/video10)\n"
        "  --camera-width PX            full side-by-side width (default 1280)\n"
        "  --camera-height PX           height (default 480)\n"
        "  --camera-capture-fps FPS     requested capture rate (default 60)\n"
        "  --camera-fps FPS             delivered/kept rate; extra frames are dropped (default "
        "20)\n"
        "  --layout NAME                side_by_side (default)\n"
        "  --require-capture-fps        fail when the camera negotiates less than requested\n"
        "  --no-stereo                  do not record camera\n"
        "\n"
        "Preview:\n"
        "  --preview                    stream raw IMU and JPEG to a host while recording\n"
        "  --preview-address ADDR       listen address for the preview stream (default "
        "127.0.0.1)\n"
        "  --preview-port PORT          listen port for the preview stream (default 5000)\n"
        "\n"
        "Diagnostics:\n"
        "  --check                      open devices, print negotiated settings, exit\n"
        "  --self-test                  exercise the raw bag writer/reader, exit\n"
        "  --help                       show this help\n",
        program);
}

double parse_seconds(const std::string& text) {
    if (text.empty()) {
        throw std::invalid_argument("empty duration");
    }
    char suffix = text.back();
    double scale = 1.0;
    std::string number = text;
    if (suffix == 's' || suffix == 'S') {
        number.pop_back();
    } else if (suffix == 'm' || suffix == 'M') {
        number.pop_back();
        scale = 60.0;
    } else if (suffix == 'h' || suffix == 'H') {
        number.pop_back();
        scale = 3600.0;
    }
    const double value = std::stod(number);
    if (value < 0.0) {
        throw std::invalid_argument("duration must not be negative");
    }
    return value * scale;
}

std::uint64_t parse_bytes(const std::string& text) {
    if (text.empty()) {
        throw std::invalid_argument("empty byte count");
    }
    char suffix = text.back();
    std::uint64_t scale = 1;
    std::string number = text;
    if (suffix == 'k' || suffix == 'K') {
        number.pop_back();
        scale = 1024;
    } else if (suffix == 'm' || suffix == 'M') {
        number.pop_back();
        scale = 1024 * 1024;
    } else if (suffix == 'g' || suffix == 'G') {
        number.pop_back();
        scale = 1024 * 1024 * 1024;
    }
    return std::stoull(number) * scale;
}

std::string default_output() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char buffer[64]{};
    std::strftime(buffer, sizeof(buffer), "/tmp/flyrat_raw/%Y%m%d_%H%M%S", &local);
    return buffer;
}

std::string format_duration(double seconds) {
    char buffer[32]{};
    std::snprintf(buffer, sizeof(buffer), "%.1f", seconds);
    return buffer;
}

void print_stats(const flyrat::pilot::RawRecorderStats& stats) {
    const double imu_rate = stats.elapsed_s > 0.0 ? stats.imu_samples / stats.elapsed_s : 0.0;
    const double image_rate = stats.elapsed_s > 0.0 ? stats.images / stats.elapsed_s : 0.0;
    std::printf(
        "raw recorder: %s s  IMU %llu (%.1f Hz)  images %llu (%.1f Hz)  "
        "capture %.1f fps  throttled %llu  driver_dropped %llu  "
        "dropped_imu %llu  dropped_images %llu  %.1f MiB\n",
        format_duration(stats.elapsed_s).c_str(),
        static_cast<unsigned long long>(stats.imu_samples), imu_rate,
        static_cast<unsigned long long>(stats.images), image_rate, stats.capture_fps,
        static_cast<unsigned long long>(stats.throttled),
        static_cast<unsigned long long>(stats.driver_dropped),
        static_cast<unsigned long long>(stats.dropped_imu),
        static_cast<unsigned long long>(stats.dropped_images),
        static_cast<double>(stats.bytes) / (1024.0 * 1024.0));
    if (stats.preview_enabled) {
        std::printf(
            "raw recorder: preview %s  IMU frames %llu  stereo frames %llu  dropped %llu\n",
            stats.preview_connected ? "connected" : "waiting",
            static_cast<unsigned long long>(stats.preview_priority_sent),
            static_cast<unsigned long long>(stats.preview_stereo_sent),
            static_cast<unsigned long long>(stats.preview_dropped));
    }
}

} // namespace

int main(int argc, char** argv) {
    flyrat::pilot::RawRecorderConfig config;
    config.camera.device = "/dev/video10";
    config.camera.width = 1280;
    config.camera.height = 480;
    config.camera.capture_fps = 60;
    config.camera.fps = 20;

    double duration_s = 0.0;
    std::uint64_t max_bytes = 0;
    bool check = false;
    bool self_test = false;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto value = [&]() -> std::string {
                if (i + 1 >= argc) {
                    throw std::invalid_argument("missing value after " + arg);
                }
                return argv[++i];
            };

            if (arg == "--help" || arg == "-h") {
                usage(argv[0]);
                return 0;
            } else if (arg == "--output") {
                config.output = value();
            } else if (arg == "--duration") {
                duration_s = parse_seconds(value());
            } else if (arg == "--max-bytes") {
                max_bytes = parse_bytes(value());
            } else if (arg == "--msp-device") {
                config.msp_device = value();
            } else if (arg == "--msp-baud") {
                config.msp_baud = std::stoi(value());
            } else if (arg == "--imu-hz") {
                config.imu_hz = std::stoi(value());
            } else if (arg == "--no-imu") {
                config.imu_enabled = false;
            } else if (arg == "--acc-lsb-per-g") {
                config.acc_lsb_per_g = std::stod(value());
            } else if (arg == "--gyro-lsb-per-dps") {
                config.gyro_lsb_per_dps = std::stod(value());
            } else if (arg == "--accel-axis") {
                config.accel_axis = value();
            } else if (arg == "--gyro-axis") {
                config.gyro_axis = value();
            } else if (arg == "--camera-device") {
                config.camera.device = value();
            } else if (arg == "--camera-width") {
                config.camera.width = std::stoi(value());
            } else if (arg == "--camera-height") {
                config.camera.height = std::stoi(value());
            } else if (arg == "--camera-capture-fps") {
                config.camera.capture_fps = std::stoi(value());
            } else if (arg == "--camera-fps") {
                config.camera.fps = std::stoi(value());
            } else if (arg == "--layout") {
                config.camera_layout = value();
            } else if (arg == "--require-capture-fps") {
                config.require_capture_fps = true;
            } else if (arg == "--no-stereo") {
                config.camera_enabled = false;
            } else if (arg == "--preview") {
                config.preview_enabled = true;
            } else if (arg == "--preview-address") {
                config.preview_address = value();
            } else if (arg == "--preview-port") {
                config.preview_port = std::stoi(value());
            } else if (arg == "--check") {
                check = true;
            } else if (arg == "--self-test") {
                self_test = true;
            } else {
                std::fprintf(stderr, "unknown option: %s\n", arg.c_str());
                usage(argv[0]);
                return 2;
            }
        }

        if (self_test) {
            return flyrat::pilot::run_raw_bag_self_test();
        }
        if (config.camera_layout != "side_by_side") {
            throw std::invalid_argument("only --layout side_by_side is supported");
        }
        if (config.imu_hz <= 0) {
            throw std::invalid_argument("--imu-hz must be positive");
        }
        if (config.camera_enabled && config.camera.fps <= 0) {
            throw std::invalid_argument("--camera-fps must be positive");
        }
        if (config.camera_enabled && config.camera.capture_fps < 0) {
            throw std::invalid_argument("--camera-capture-fps must not be negative");
        }
        if (config.preview_enabled
            && (config.preview_port <= 0 || config.preview_port > 65535)) {
            throw std::invalid_argument("--preview-port must be between 1 and 65535");
        }
        if (config.preview_enabled && config.preview_address.empty()) {
            throw std::invalid_argument("--preview-address must not be empty");
        }
        if (!check && config.output.empty()) {
            config.output = default_output();
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "raw recorder: %s\n", error.what());
        return 2;
    }

    try {
        flyrat::pilot::RawRecorder recorder(config);
        if (check) {
            recorder.check();
            return 0;
        }

        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        recorder.start();
        double last_report = 0.0;
        while (g_stop == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            const auto stats = recorder.stats();
            const double elapsed = stats.elapsed_s;
            if (duration_s > 0.0 && elapsed >= duration_s) {
                break;
            }
            if (max_bytes > 0 && stats.bytes >= max_bytes) {
                break;
            }
            if (elapsed - last_report >= 5.0) {
                print_stats(stats);
                last_report = elapsed;
            }
        }

        const std::string bag_path = recorder.path();
        recorder.stop();
        const auto stats = recorder.stats();
        print_stats(stats);
        std::printf("raw recorder: bag %s\n", bag_path.c_str());
        if (config.imu_enabled && stats.imu_samples == 0) {
            std::fprintf(stderr, "raw recorder: no IMU samples were recorded\n");
            return 1;
        }
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "raw recorder: %s\n", error.what());
        return 1;
    }
}

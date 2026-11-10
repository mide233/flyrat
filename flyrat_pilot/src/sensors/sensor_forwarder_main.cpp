// SPDX-License-Identifier: MIT
//
// Standalone sensor forwarder for the real vehicle.
//
//   flyrat_sensor_forwarder --listen 127.0.0.1:5000
//
// On the development machine:
//   adb forward tcp:5000 tcp:5000
//   ros2 launch flyrat_sensors sensors.launch.py
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "sensor_forwarder.hpp"

namespace {

std::atomic<bool> g_stop { false };

void on_signal(int)
{
    g_stop = true;
}

void usage()
{
    std::cout
        << "usage: flyrat_sensor_forwarder [options]\n"
        << "  --listen ADDR:PORT   default 127.0.0.1:5000\n"
        << "  --msp-device PATH    default /dev/ttyS3\n"
        << "  --msp-baud N         default 921600\n"
        << "  --camera-device PATH default /dev/video10\n"
        << "  --camera-width N     default 320 (QVGA)\n"
        << "  --camera-height N    default 240 (QVGA)\n"
        << "  --camera-fps N       default 15\n"
        << "  --camera-exposure-us N default 5000 (5 ms, 0 = camera auto)\n"
        << "  --camera-gain N      default -1 (leave camera gain untouched)\n"
        << "  --tof-resolution N   default 8 (4 or 8)\n"
        << "  --tof-rate HZ        default 15 (4x4 up to 60, 8x8 up to 15)\n"
        << "  --no-imu --no-tof --no-camera\n"
        << "  --help\n";
}

std::pair<std::string, int> parse_listen(const std::string& value)
{
    const std::size_t colon = value.rfind(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 == value.size()) {
        throw std::runtime_error("listen must be ADDR:PORT");
    }
    const int port = std::atoi(value.c_str() + colon + 1);
    if (port <= 0 || port > 65535) {
        throw std::runtime_error("listen port is out of range");
    }
    return {value.substr(0, colon), port};
}

} // namespace

int main(int argc, char** argv)
{
    flyrat::pilot::SensorForwarderConfig config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "missing value for " << name << '\n';
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--listen") {
            const auto listen = parse_listen(next("--listen"));
            config.listen_address = listen.first;
            config.listen_port = listen.second;
        } else if (arg == "--msp-device") {
            config.msp_device = next("--msp-device");
        } else if (arg == "--msp-baud") {
            config.msp_baud = std::atoi(next("--msp-baud").c_str());
        } else if (arg == "--camera-device") {
            config.camera.device = next("--camera-device");
        } else if (arg == "--camera-width") {
            config.camera.width = std::atoi(next("--camera-width").c_str());
        } else if (arg == "--camera-height") {
            config.camera.height = std::atoi(next("--camera-height").c_str());
        } else if (arg == "--camera-fps") {
            config.camera.fps = std::atoi(next("--camera-fps").c_str());
        } else if (arg == "--camera-exposure-us") {
            config.camera.exposure_us = std::atoi(next("--camera-exposure-us").c_str());
        } else if (arg == "--camera-gain") {
            config.camera.gain = std::atoi(next("--camera-gain").c_str());
        } else if (arg == "--tof-resolution") {
            config.tof.resolution = std::atoi(next("--tof-resolution").c_str());
        } else if (arg == "--tof-rate") {
            config.tof.ranging_hz = std::atoi(next("--tof-rate").c_str());
        } else if (arg == "--no-imu") {
            config.enable_imu = false;
        } else if (arg == "--no-tof") {
            config.enable_tof = false;
        } else if (arg == "--no-camera" || arg == "--no-stereo") {
            config.enable_camera = false;
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
            std::cerr << "unknown option: " << arg << '\n';
            usage();
            return 2;
        }
    }

    if (config.tof.resolution != 4 && config.tof.resolution != 8) {
        std::cerr << "tof-resolution must be 4 or 8\n";
        return 2;
    }
    const int max_tof_hz = config.tof.resolution == 4 ? 60 : 15;
    if (config.tof.ranging_hz <= 0 || config.tof.ranging_hz > max_tof_hz) {
        std::cerr << "tof-rate must be 1.." << max_tof_hz
                  << " for resolution " << config.tof.resolution << '\n';
        return 2;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    try {
        flyrat::pilot::SensorForwarder forwarder(config);
        forwarder.start();
        std::cout << "sensor forwarder listening on " << config.listen_address << ':'
                  << config.listen_port << '\n';
        while (!g_stop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        forwarder.stop();
    } catch (const std::exception& error) {
        std::cerr << "sensor forwarder failed: " << error.what() << '\n';
        return 1;
    }
    return 0;
}

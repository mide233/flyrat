// SPDX-License-Identifier: MIT
//
// Smoke benchmark for the TorchScript TIO-Former backend.
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "flyrat/tio/tio_odometry.hpp"

int main(int argc, char** argv)
{
    flyrat::tio::Options options;
    options.model_path = argc > 1
        ? argv[1]
        : "models/tio_former_window64.pt";
    const int frames = argc > 2 ? std::atoi(argv[2]) : 100;
    const bool camera_enabled = argc > 3 && std::string(argv[3]) == "camera";
    options.camera = camera_enabled;

    try {
        flyrat::tio::TioOdometry odometry(options);
        std::vector<flyrat::tio::TofGrid> views(4);
        for (auto& view : views) {
            view.range.fill(1.0F);
            view.weight.fill(1.0F);
        }

        flyrat::tio::CameraFrame camera;
        if (camera_enabled) {
            camera.gray.resize(static_cast<std::size_t>(options.camera_width * options.camera_height));
            for (int y = 0; y < options.camera_height; ++y) {
                for (int x = 0; x < options.camera_width; ++x) {
                    camera.gray[static_cast<std::size_t>(y * options.camera_width + x)] =
                        static_cast<float>(x + y) / static_cast<float>(
                            options.camera_width + options.camera_height);
                }
            }
        }

        const auto started = std::chrono::steady_clock::now();
        int valid = 0;
        flyrat::odometry::Result last;
        for (int index = 0; index < frames; ++index) {
            const double timestamp = static_cast<double>(index) / 15.0;
            for (int sample = 0; sample < 14; ++sample) {
                flyrat::odometry::ImuSample imu;
                imu.timestamp = timestamp - 0.066 + 0.0047 * sample;
                imu.accel = Eigen::Vector3d(0.0, 0.0, 9.81);
                odometry.addImu(imu);
            }
            last = camera_enabled ? odometry.track(views, timestamp, &camera)
                                  : odometry.track(views, timestamp);
            if (last.pose.valid) {
                ++valid;
            }
        }
        const double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started)
                                   .count();
        std::cout << "frames=" << frames << " valid=" << valid
                  << " avg_ms=" << (elapsed / frames * 1000.0)
                  << " state=" << static_cast<int>(last.state)
                  << " position=" << last.pose.position.transpose() << '\n';
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}

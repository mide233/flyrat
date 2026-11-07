// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "flyrat/slam/stereo_slam.hpp"

namespace {

int failures = 0;

void check(bool condition, const char* message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        ++failures;
    }
}

std::string write_temp_config(const std::string& name, const std::string& body)
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / name;
    std::ofstream output(path);
    output << body;
    return path.string();
}

} // namespace

int main()
{
    using namespace flyrat::slam;

    check(static_cast<int>(State::kTracking) != static_cast<int>(State::kLost),
        "state values are distinct");

    Options options;
    bool threw = false;
    try {
        StereoSlam slam(options);
    } catch (const SlamError&) {
        threw = true;
    }
    check(threw, "empty config path is rejected");

    options.config_path = "/definitely/not/a/vins/config.yaml";
    threw = false;
    try {
        StereoSlam slam(options);
    } catch (const SlamError&) {
        threw = true;
    }
    check(threw, "missing config is rejected");

    const std::string dummy_cam = write_temp_config(
        "flyrat_dummy_cam.yaml", "%YAML:1.0\n---\n");
    const std::string invalid_config = write_temp_config(
        "flyrat_invalid_vins.yaml",
        "%YAML:1.0\n"
        "---\n"
        "imu: 0\n"
        "num_of_cam: 1\n"
        "cam0_calib: \"" + dummy_cam + "\"\n"
        "image_width: 640\n"
        "image_height: 480\n"
        "estimate_extrinsic: 0\n"
        "body_T_cam0: !!opencv-matrix\n"
        "   rows: 4\n"
        "   cols: 4\n"
        "   dt: d\n"
        "   data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]\n");
    options.config_path = invalid_config;
    threw = false;
    try {
        StereoSlam slam(options);
    } catch (const SlamError&) {
        threw = true;
    }
    check(threw, "stereo-inertial requirements are enforced");

#ifdef FLYRAT_TEST_VINS_CONFIG
    options.config_path = FLYRAT_TEST_VINS_CONFIG;
    try {
        StereoSlam slam(options);
        slam.reset();

        std::vector<ImuSample> imu(1);
        imu[0].timestamp = 1.0;
        imu[0].accel = Eigen::Vector3d(0.0, 0.0, 9.81);
        imu[0].gyro = Eigen::Vector3d::Zero();

        threw = false;
        try {
            slam.track(cv::Mat(), cv::Mat(), 1.0, imu);
        } catch (const SlamError&) {
            threw = true;
        }
        check(threw, "empty images are rejected");

        const cv::Mat left(240, 320, CV_8UC1, cv::Scalar(0));
        const cv::Mat right(240, 320, CV_8UC1, cv::Scalar(0));
        threw = false;
        try {
            slam.track(left, right, 1.0, imu);
        } catch (const SlamError&) {
            threw = true;
        }
        check(threw, "image/config size mismatch is rejected");

        const std::filesystem::path trajectory =
            std::filesystem::temp_directory_path() / "flyrat_empty_trajectory.txt";
        slam.saveTrajectory(trajectory.string());
        check(std::filesystem::is_regular_file(trajectory),
            "trajectory file is written");

        slam.shutdown();
        slam.shutdown();
        slam.saveTrajectory(trajectory.string());
        check(std::filesystem::is_regular_file(trajectory),
            "trajectory file is available after shutdown");
        std::filesystem::remove(trajectory);
        threw = false;
        try {
            slam.track(cv::Mat(), cv::Mat(), 1.0, imu);
        } catch (const SlamError&) {
            threw = true;
        }
        check(threw, "track after shutdown is rejected");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FAIL: VINS construction: %s\n", error.what());
        ++failures;
    }
#endif

    std::filesystem::remove(invalid_config);
    std::filesystem::remove(dummy_cam);

    if (failures == 0) {
        std::printf("SLAM API tests passed\n");
    }
    return failures == 0 ? 0 : 1;
}

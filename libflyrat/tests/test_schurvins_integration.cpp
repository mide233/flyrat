// SPDX-License-Identifier: MIT
//
// Smoke test for the SchurVINS adapter. It feeds a deterministic synthetic
// stereo pair with a close-range disparity and a static IMU stream, then checks
// that the frontend initializes, tracks, and writes a trajectory. The disparity
// corresponds to about 0.43 m, which guards the near-field depth range used by
// the real setup. It also guards the IMU lookahead requirement: without a
// sample after each frame the SVO frontend aborts with CHECK(0).
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "flyrat/odometry/odometry.hpp"

namespace {

int g_failures = 0;

void check(bool ok, const std::string& what)
{
    if (ok)
    {
        std::printf("  ok   %s\n", what.c_str());
    } else
    {
        std::printf("  FAIL %s\n", what.c_str());
        ++g_failures;
    }
}

} // namespace

int main()
{
    using namespace flyrat::odometry;

    Options options;
    options.mode = Mode::kImuStereo;
    options.camera_config_path = FLYRAT_TEST_CAMERA_CONFIG;

    Odometry odometry(options);

    cv::Mat left(480, 640, CV_8UC1);
    cv::Mat right(480, 640, CV_8UC1);
    cv::RNG rng(7);
    rng.fill(left, cv::RNG::UNIFORM, 0, 255);
    cv::GaussianBlur(left, left, cv::Size(5, 5), 1.5);
    const int disparity = 40;
    right.setTo(0);
    left(cv::Rect(0, 0, left.cols - disparity, left.rows))
        .copyTo(right(cv::Rect(disparity, 0, left.cols - disparity, left.rows)));

    Result result;
    int first_tracking_frame = -1;
    const double start = 100.0;
    for (int frame = 0; frame < 10; ++frame)
    {
        const double stamp = start + 0.1 * frame;
        std::vector<ImuSample> imu;
        for (int i = -20; i <= 4; ++i)
        {
            ImuSample sample;
            sample.timestamp = stamp + 0.005 * i;
            sample.accel = Eigen::Vector3d(0.0, 0.0, 9.81);
            sample.gyro = Eigen::Vector3d::Zero();
            imu.push_back(sample);
        }
        const ImageView left_view{left.cols, left.rows, static_cast<int>(left.step), left.data};
        const ImageView right_view{
            right.cols, right.rows, static_cast<int>(right.step), right.data};
        result = odometry.track(left_view, right_view, stamp, imu);
        if (result.state == State::kTracking && first_tracking_frame < 0)
        {
            first_tracking_frame = frame;
        }
    }

    check(
        first_tracking_frame >= 0 && first_tracking_frame <= 5,
        "frontend initializes within the first few frames");
    check(result.state == State::kTracking, "frontend reaches tracking");
    check(result.pose.valid, "tracking pose is valid");
    check(
        std::isfinite(result.pose.position.x()) && std::isfinite(result.pose.position.y())
            && std::isfinite(result.pose.position.z()),
        "tracking pose is finite");

    const std::string path = "/tmp/flyrat_schurvins_trajectory.tum";
    bool wrote = true;
    try
    {
        odometry.saveTrajectory(path);
    } catch (const OdometryError&)
    {
        wrote = false;
    }
    std::ifstream input(path);
    check(wrote && input.good(), "trajectory file is written");
    std::remove(path.c_str());

    odometry.shutdown();

    if (g_failures != 0)
    {
        std::printf("%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("all SchurVINS checks passed\n");
    return 0;
}

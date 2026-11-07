// SPDX-License-Identifier: MIT
//
// Contract test for the odometry scaffold. The backend is not implemented yet,
// so the important guarantees are: the calibration file is validated, malformed
// frames are rejected, and a valid call reports NOT_INITIALIZED instead of a
// bogus pose.
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "flyrat/odometry/odometry.hpp"

namespace {

int g_failures = 0;

void check(bool ok, const std::string& what)
{
    if (ok) {
        std::printf("  ok   %s\n", what.c_str());
    } else {
        std::printf("  FAIL %s\n", what.c_str());
        ++g_failures;
    }
}

} // namespace

int main()
{
    using namespace flyrat::odometry;

    std::printf("Odometry\n");

    check(State::kNotInitialized != State::kTracking, "states are distinct");
    check(Options {}.mode == Mode::kImuStereo, "default mode is IMU-stereo");

    {
        bool threw = false;
        try {
            Odometry odometry(Options {});
        } catch (const OdometryError&) {
            threw = true;
        }
        check(threw, "empty calibration path is rejected");
    }

    {
        Options options;
        options.camera_config_path = "/nonexistent/flyrat-camera.yaml";
        bool threw = false;
        try {
            Odometry odometry(options);
        } catch (const OdometryError&) {
            threw = true;
        }
        check(threw, "missing calibration file is rejected");
    }

    Options options;
    options.camera_config_path = FLYRAT_TEST_CAMERA_CONFIG;
    Odometry odometry(options);

    std::vector<std::uint8_t> left_pixels(16 * 16, 0);
    std::vector<std::uint8_t> right_pixels(16 * 16, 0);
    const ImageView left { 16, 16, 16, left_pixels.data() };
    const ImageView right { 16, 16, 16, right_pixels.data() };

    const Result result = odometry.track(left, right, 12.5);
    check(result.state == State::kNotInitialized, "stub reports NOT_INITIALIZED");
    check(!result.pose.valid, "stub pose is invalid");
    check(result.timestamp == 12.5, "result keeps the frame timestamp");

    {
        bool threw = false;
        try {
            odometry.track(ImageView {}, right, 13.0);
        } catch (const OdometryError&) {
            threw = true;
        }
        check(threw, "empty image is rejected");
    }

    {
        std::vector<std::uint8_t> small(8 * 8, 0);
        const ImageView smaller { 8, 8, 8, small.data() };
        bool threw = false;
        try {
            odometry.track(left, smaller, 14.0);
        } catch (const OdometryError&) {
            threw = true;
        }
        check(threw, "size mismatch is rejected");
    }

    {
        bool threw = false;
        try {
            odometry.saveTrajectory("/tmp/trajectory.tum");
        } catch (const OdometryError&) {
            threw = true;
        }
        check(threw, "trajectory output is explicitly unimplemented");
    }

    odometry.reset();
    odometry.shutdown();
    odometry.shutdown();
    check(true, "reset and repeated shutdown are safe");

    {
        bool threw = false;
        try {
            odometry.track(left, right, 15.0);
        } catch (const OdometryError&) {
            threw = true;
        }
        check(threw, "track after shutdown is rejected");
    }

    if (g_failures != 0) {
        std::printf("%d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("all odometry checks passed\n");
    return 0;
}

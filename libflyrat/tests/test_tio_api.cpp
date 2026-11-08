// SPDX-License-Identifier: MIT
//
// Smoke test for the TorchScript TIO-Former backend.  This only checks the
// interface, warm-up behavior and that inference produces a finite pose; the
// research evaluation in research/tio_former owns accuracy.

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "flyrat/tio/tio_odometry.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

std::vector<flyrat::tio::TofGrid> empty_views()
{
    return std::vector<flyrat::tio::TofGrid>(4);
}

} // namespace

int main()
{
    using flyrat::odometry::State;
    using flyrat::tio::Options;
    using flyrat::tio::TioOdometry;

    Options empty;
    bool threw = false;
    try {
        TioOdometry odometry(empty);
    } catch (const flyrat::odometry::OdometryError&) {
        threw = true;
    }
    check(threw, "empty model path is rejected");

    Options missing;
    missing.model_path = "/nonexistent/tio_model.pt";
    threw = false;
    try {
        TioOdometry odometry(missing);
    } catch (const flyrat::odometry::OdometryError&) {
        threw = true;
    }
    check(threw, "missing model file is rejected");

    Options options;
    options.model_path = FLYRAT_TEST_TIO_MODEL;
    options.warmup_frames = 64;

    Options invalid = options;
    invalid.warmup_frames = 0;
    threw = false;
    try {
        TioOdometry odometry(invalid);
    } catch (const flyrat::odometry::OdometryError&) {
        threw = true;
    }
    check(threw, "invalid warm-up is rejected");

    TioOdometry odometry(options);
    const auto views = empty_views();
    for (int frame = 0; frame < 64; ++frame) {
        const double stamp = static_cast<double>(frame) / 15.0;
        flyrat::odometry::ImuSample imu;
        imu.timestamp = stamp;
        odometry.addImu(imu);
        const auto result = odometry.track(views, stamp);
        if (frame < 63) {
            check(result.state == State::kNotInitialized,
                "state is NOT_INITIALIZED during warm-up");
        } else {
            check(result.state == State::kTracking, "state is TRACKING after warm-up");
            check(result.pose.valid, "first pose is valid");
            check(result.pose.position.allFinite(), "first position is finite");
            check(std::abs(result.pose.orientation.norm() - 1.0) < 1e-6,
                "first orientation is normalized");
        }
    }

    odometry.shutdown();
    threw = false;
    try {
        odometry.track(views, 4.3);
    } catch (const flyrat::odometry::OdometryError&) {
        threw = true;
    }
    check(threw, "track after shutdown is rejected");

    if (failures != 0) {
        std::cerr << failures << " checks failed\n";
        return 1;
    }
    std::cout << "test_tio_api: all checks passed\n";
    return 0;
}

// SPDX-License-Identifier: MIT
//
// Stub stereo / stereo-inertial odometry backend.
//
// TODO(odometry): replace the track() body with the real estimator. The node
// already supplies synchronised grayscale stereo pairs and the IMU samples
// between consecutive frames, so the backend only has to produce a
// T_world_camera pose and a State transition.
#include "flyrat/odometry/odometry.hpp"

#include <fstream>
#include <utility>

namespace flyrat::odometry {

struct Odometry::Impl {
    explicit Impl(Options value)
        : options(std::move(value))
    {
    }

    Options options;
    bool shutdown = false;
};

Odometry::Odometry(Options options)
    : impl_(std::make_unique<Impl>(std::move(options)))
{
    if (impl_->options.camera_config_path.empty()) {
        throw OdometryError("camera_config_path is empty");
    }
    std::ifstream config(impl_->options.camera_config_path);
    if (!config) {
        throw OdometryError(
            "cannot open camera configuration: " + impl_->options.camera_config_path);
    }
}

Odometry::~Odometry() = default;

Result Odometry::track(
    const ImageView& left, const ImageView& right, double timestamp)
{
    return track(left, right, timestamp, {});
}

Result Odometry::track(const ImageView& left, const ImageView& right,
    double timestamp, const std::vector<ImuSample>& imu)
{
    (void)imu;
    if (impl_->shutdown) {
        throw OdometryError("odometry has been shut down");
    }
    if (left.empty() || right.empty()) {
        throw OdometryError("left/right image is empty");
    }
    if (left.width != right.width || left.height != right.height) {
        throw OdometryError("left/right image size mismatch");
    }

    // The stub never initialises, so the ROS node publishes NOT_INITIALIZED
    // diagnostics instead of a bogus pose.
    Result result;
    result.state = State::kNotInitialized;
    result.timestamp = timestamp;
    return result;
}

void Odometry::reset()
{
    // TODO(odometry): clear the map, IMU state and trajectory here.
}

void Odometry::shutdown()
{
    impl_->shutdown = true;
}

void Odometry::saveTrajectory(const std::string& path) const
{
    (void)path;
    throw OdometryError("trajectory output is not implemented");
}

} // namespace flyrat::odometry

// SPDX-License-Identifier: MIT
//
// Stereo / stereo-inertial odometry backend.
//
// When built with FLYRAT_WITH_SCHURVINS this forwards to the vendored
// SchurVINS frontend (see schurvins_backend.cpp). Otherwise it keeps the
// original scaffold behaviour: validate the calibration, report
// NOT_INITIALIZED and reject trajectory output.
#include "flyrat/odometry/odometry.hpp"

#include <fstream>
#include <utility>

#ifdef FLYRAT_ODOMETRY_WITH_SCHURVINS
#include "schurvins_backend.hpp"
#endif

namespace flyrat::odometry {

struct Odometry::Impl {
    explicit Impl(Options value)
        : options(std::move(value))
    {
    }

    Options options;
    bool shutdown = false;
#ifdef FLYRAT_ODOMETRY_WITH_SCHURVINS
    std::unique_ptr<SchurvinsBackend> backend;
#endif
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

#ifdef FLYRAT_ODOMETRY_WITH_SCHURVINS
    if (impl_->options.mode != Mode::kImuStereo) {
        throw OdometryError("SchurVINS requires IMU-stereo mode");
    }
    impl_->backend =
        std::make_unique<SchurvinsBackend>(impl_->options.camera_config_path);
#endif
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
    if (impl_->shutdown) {
        throw OdometryError("odometry has been shut down");
    }
    if (left.empty() || right.empty()) {
        throw OdometryError("left/right image is empty");
    }
    if (left.width != right.width || left.height != right.height) {
        throw OdometryError("left/right image size mismatch");
    }

#ifdef FLYRAT_ODOMETRY_WITH_SCHURVINS
    return impl_->backend->track(left, right, timestamp, imu);
#else
    (void)imu;
    // Without a backend there is nothing to initialise, so the ROS node
    // publishes NOT_INITIALIZED diagnostics instead of a bogus pose.
    Result result;
    result.state = State::kNotInitialized;
    result.timestamp = timestamp;
    return result;
#endif
}

void Odometry::reset()
{
#ifdef FLYRAT_ODOMETRY_WITH_SCHURVINS
    impl_->backend->reset();
#endif
}

void Odometry::shutdown()
{
    impl_->shutdown = true;
}

void Odometry::saveTrajectory(const std::string& path) const
{
#ifdef FLYRAT_ODOMETRY_WITH_SCHURVINS
    impl_->backend->saveTrajectory(path);
#else
    (void)path;
    throw OdometryError("trajectory output requires a built odometry backend");
#endif
}

} // namespace flyrat::odometry

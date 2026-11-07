// SPDX-License-Identifier: MIT
//
// Stereo / stereo-inertial odometry interface.
//
// The implementation is intentionally a stub: it validates the calibration
// file and reports NOT_INITIALIZED so the ROS node, TF tree and sensor pipeline
// can be exercised before the algorithm exists. Replace the body of
// src/odometry/odometry.cpp with the real estimator; the interface below is
// what the rest of the flight stack consumes.
#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace flyrat::odometry {

class OdometryError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

enum class Mode {
    kStereo,
    kImuStereo,
};

enum class State {
    kNotInitialized,
    kTracking,
    kRecentlyLost,
    kLost,
};

// A non-owning 8-bit grayscale image. The data pointer must stay valid for the
// duration of track(); row stride is in bytes.
struct ImageView {
    int width = 0;
    int height = 0;
    int stride = 0;
    const std::uint8_t* data = nullptr;

    bool empty() const
    {
        return width <= 0 || height <= 0 || stride <= 0 || data == nullptr;
    }
    std::size_t size() const
    {
        return static_cast<std::size_t>(stride) * static_cast<std::size_t>(height);
    }
};

struct ImuSample {
    double timestamp = 0.0;
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Pose of the left camera in the world frame (T_world_camera). The world frame
// is defined by the estimator; the ROS node rebases it to the initial body
// frame when it needs a FLU map.
struct Pose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool valid = false;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct Result {
    State state = State::kNotInitialized;
    Pose pose;
    double timestamp = 0.0;
};

struct Options {
    Mode mode = Mode::kImuStereo;
    // Camera/IMU calibration in the neutral YAML schema stored under
    // libflyrat/configs/camera/.
    std::string camera_config_path;
};

class Odometry {
public:
    explicit Odometry(Options options);
    ~Odometry();

    Odometry(const Odometry&) = delete;
    Odometry& operator=(const Odometry&) = delete;

    Result track(const ImageView& left, const ImageView& right, double timestamp);
    Result track(const ImageView& left, const ImageView& right, double timestamp,
        const std::vector<ImuSample>& imu);

    void reset();
    void shutdown();

    // Writes a TUM trajectory. Not implemented until the backend produces
    // poses; currently throws OdometryError.
    void saveTrajectory(const std::string& path) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::odometry

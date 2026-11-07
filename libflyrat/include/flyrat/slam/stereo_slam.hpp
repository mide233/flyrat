// SPDX-License-Identifier: GPL-3.0-or-later
//
// VINS-Fusion stereo-inertial SLAM facade.
//
// The vendored estimator is GPL-3.0 and exposes a process-wide parameter set.
// Only one StereoSlam instance may be alive in a process.

#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core/mat.hpp>

namespace flyrat::slam {

class SlamError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

enum class State {
    kNotInitialized,
    kTracking,
    kLost,
};

struct ImuSample {
    double timestamp = 0.0;
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

// Pose of the IMU/body frame in the VINS world frame: T_world_body.
//
// VINS defines the world frame during initialization. It is gravity aligned,
// but its yaw is fixed by the first image/IMU window. The body frame is the
// IMU frame, not a camera frame.
struct Pose {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    bool valid = false;
};

struct Result {
    State state = State::kNotInitialized;
    Pose pose;
    double timestamp = 0.0;
};

struct Options {
    // VINS stereo-inertial YAML. Camera calibration YAML paths are resolved
    // relative to this file.
    std::string config_path;
};

class StereoSlam {
public:
    explicit StereoSlam(Options options);
    ~StereoSlam();

    StereoSlam(const StereoSlam&) = delete;
    StereoSlam& operator=(const StereoSlam&) = delete;
    StereoSlam(StereoSlam&&) = delete;
    StereoSlam& operator=(StereoSlam&&) = delete;

    // Feed one synchronized stereo pair and the IMU samples up to `timestamp`.
    // The images may be mono8, BGR8 or BGRA8; they are converted to grayscale
    // internally. Timestamps are seconds and must advance.
    Result track(const cv::Mat& left, const cv::Mat& right, double timestamp,
        const std::vector<ImuSample>& imu);

    // Reset the estimator and clear the in-memory trajectory. The configuration
    // is retained.
    void reset();

    // Release the estimator. Safe to call more than once. The trajectory is kept
    // until the object is destroyed, so saveTrajectory() may be called after it.
    void shutdown();

    // Write a TUM trajectory: timestamp tx ty tz qx qy qz qw.
    void saveTrajectory(const std::string& path) const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::slam

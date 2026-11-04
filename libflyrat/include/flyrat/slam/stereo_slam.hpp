// SPDX-License-Identifier: GPL-3.0-or-later
//
// Stereo / stereo-inertial SLAM facade over the vendored ORB-SLAM3 core.
//
// ORB-SLAM3 is GPLv3. Linking this facade pulls that license into the resulting
// binary; see third_party/orb_slam3/LICENSE.
#pragma once

#include <cstdint>
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

struct ImuSample {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Seconds, on the same monotonic clock as track().
    double timestamp = 0.0;
    // m/s^2 and rad/s, already converted from raw sensor counts.
    Eigen::Vector3d accel { Eigen::Vector3d::Zero() };
    Eigen::Vector3d gyro { Eigen::Vector3d::Zero() };
};

struct Pose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // T_world_camera: maps a point from the left camera frame to the world
    // frame. The world frame is the left camera pose of the first frame that
    // ORB-SLAM3 successfully initialised on.
    Eigen::Vector3d position { Eigen::Vector3d::Zero() };
    Eigen::Quaterniond orientation { Eigen::Quaterniond::Identity() };
    bool valid = false;
};

struct Result {
    State state = State::kNotInitialized;
    Pose pose;
    double timestamp = 0.0;
};

struct Options {
    Mode mode = Mode::kStereo;
    // Path to ORBvoc.txt. ORB-SLAM3 only supports its text vocabulary format;
    // the 139 MB file is intentionally not vendored.
    std::string vocabulary_path;
    // Path to the ORB-SLAM3 settings YAML (camera intrinsics, stereo extrinsics,
    // ORB extractor and, for kImuStereo, IMU noise and T_b_c1).
    std::string settings_path;
};

class StereoSlam {
public:
    explicit StereoSlam(const Options& options);
    ~StereoSlam();

    StereoSlam(const StereoSlam&) = delete;
    StereoSlam& operator=(const StereoSlam&) = delete;

    // Track one synchronized stereo pair. Images may be CV_8UC1, CV_8UC3
    // (BGR, as OpenCV decodes them) or CV_8UC4. Timestamps are in seconds and
    // must increase monotonically on the same clock as ImuSample::timestamp.
    //
    // Call from one thread only. imu is only used in kImuStereo mode and should
    // contain the samples since the previous track() call, oldest first.
    Result track(const cv::Mat& left, const cv::Mat& right, double timestamp_s);
    Result track(const cv::Mat& left, const cv::Mat& right, double timestamp_s,
        const std::vector<ImuSample>& imu);

    // Drops the current map. The next track() starts a new map.
    void reset();

    // Stops the mapping and loop-closing threads. Idempotent. track() must not
    // be called afterwards.
    void shutdown();

    // Writes the camera trajectory in TUM format. Call shutdown() first.
    void saveTrajectory(const std::string& path);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::slam

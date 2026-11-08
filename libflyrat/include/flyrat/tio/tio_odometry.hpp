// SPDX-License-Identifier: MIT
//
// ToF-inertial odometry backed by the TIO-Former research model.
//
// The research prototype lives in research/tio_former and is exported to a
// TorchScript file with a fixed window (default 64 frames).  This class owns a
// rolling window of the four range images and the matching IMU windows, runs
// the model on every new ToF frame and integrates the predicted body-frame
// increment into a world pose.
//
// Unlike flyrat::odometry::Odometry (stereo), the returned pose is
// T_world_body (not T_world_camera).  The ROS node publishes it directly.
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "flyrat/odometry/odometry.hpp"

namespace flyrat::tio {

// One directional 8x8 range image.  Ranges are metres, weights are in [0, 1].
struct TofGrid {
    std::array<float, 64> range {};
    std::array<float, 64> weight {};
};

struct Options {
    // TorchScript file produced by research/tio_former/scripts/export_tio.py.
    std::string model_path;
    int window_frames = 64;
    int views = 4;
    int grid_size = 8;
    int imu_samples = 16;
    double tof_rate_hz = 15.0;
    double imu_rate_hz = 200.0;
    // Number of ToF frames collected before publishing a pose.  The exported
    // model was trained on 64-frame windows, so shorter windows are zero-padded
    // and are out of distribution; keep this at window_frames for real use.
    int warmup_frames = 64;
    double imu_buffer_seconds = 2.0;
};

class TioOdometry {
public:
    explicit TioOdometry(Options options);
    ~TioOdometry();

    TioOdometry(const TioOdometry&) = delete;
    TioOdometry& operator=(const TioOdometry&) = delete;

    // Thread-safe relative to track() as long as the caller does not reset()
    // concurrently.  The ROS node calls these from the subscription and the
    // worker thread respectively.
    void addImu(const odometry::ImuSample& sample);

    // views must be ordered front, left, right, down (the training order).
    odometry::Result track(const std::vector<TofGrid>& views, double timestamp);

    void reset();
    void shutdown();

    // Writes a TUM trajectory (timestamp tx ty tz qx qy qz qw).
    void saveTrajectory(const std::string& path) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace flyrat::tio

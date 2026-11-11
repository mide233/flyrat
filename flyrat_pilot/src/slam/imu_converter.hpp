// SPDX-License-Identifier: MIT
//
// On-vehicle copy of the IMU calibration applied by the host sensor receiver.
// Keep this in sync with flyrat_sensors/config/imu_intrinsics.yaml.
#pragma once

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

#include "flyrat/odometry/odometry.hpp"
#include "flyrat/sensors/protocol.hpp"

namespace flyrat::pilot {

struct ImuIntrinsicsConfig {
    bool enable = false;
    double acc_lsb_per_g = 2048.0;
    double gyro_lsb_per_dps = 16.4;
    double time_offset_s = 0.0;
    cv::Matx33d acc_M = cv::Matx33d::eye();
    cv::Matx33d acc_M_inv = cv::Matx33d::eye();
    cv::Matx33d gyr_M = cv::Matx33d::eye();
    cv::Matx33d gyr_M_inv = cv::Matx33d::eye();
    cv::Matx33d gyr_A = cv::Matx33d::zeros();
    cv::Matx33d gyr_C = cv::Matx33d::eye();
};

ImuIntrinsicsConfig loadImuIntrinsics(const std::string& path);

// Converts raw MSP int16 counts to the calibrated FLU body frame. The returned
// timestamp is in seconds on the Unix epoch, shifted by time_offset_s so the
// IMU sample is aligned with the camera exposure.
odometry::ImuSample convertImuSample(const ImuIntrinsicsConfig& config,
    const sensors::ImuPayload& raw, double device_time_s);

} // namespace flyrat::pilot

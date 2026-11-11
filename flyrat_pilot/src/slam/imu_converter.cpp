// SPDX-License-Identifier: MIT
#include "imu_converter.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace flyrat::pilot {
namespace {

constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;

cv::Matx33d readMatrix(const YAML::Node& node, const char* name,
    const cv::Matx33d& fallback)
{
    if (!node) {
        return fallback;
    }
    const auto values = node.as<std::vector<double>>();
    if (values.size() != 9) {
        throw std::runtime_error(std::string(name) + " must contain 9 values");
    }
    return cv::Matx33d(values[0], values[1], values[2], values[3], values[4],
        values[5], values[6], values[7], values[8]);
}

} // namespace

ImuIntrinsicsConfig loadImuIntrinsics(const std::string& path)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& error) {
        throw std::runtime_error(
            "cannot open IMU intrinsics file " + path + ": " + error.what());
    }
    const YAML::Node params = root["sensor_receiver"]["ros__parameters"];
    if (!params) {
        throw std::runtime_error(
            "IMU intrinsics file has no sensor_receiver.ros__parameters: " + path);
    }

    ImuIntrinsicsConfig config;
    config.enable = params["imu_intrinsics_enable"].as<bool>(false);
    config.time_offset_s = params["imu_time_offset"].as<double>(0.0);
    config.acc_lsb_per_g = params["acc_lsb_per_g"].as<double>(config.acc_lsb_per_g);
    config.gyro_lsb_per_dps
        = params["gyro_lsb_per_dps"].as<double>(config.gyro_lsb_per_dps);
    if (config.acc_lsb_per_g <= 0.0 || config.gyro_lsb_per_dps <= 0.0) {
        throw std::runtime_error("IMU lsb scales must be positive");
    }

    config.acc_M = readMatrix(params["imu_acc_M"], "imu_acc_M", cv::Matx33d::eye());
    config.gyr_M = readMatrix(params["imu_gyr_M"], "imu_gyr_M", cv::Matx33d::eye());
    config.gyr_A = readMatrix(params["imu_gyr_A"], "imu_gyr_A", cv::Matx33d::zeros());
    config.gyr_C = readMatrix(params["imu_gyr_C_gyro_i"], "imu_gyr_C_gyro_i",
        cv::Matx33d::eye());
    bool acc_ok = false;
    bool gyr_ok = false;
    config.acc_M_inv = config.acc_M.inv(cv::DECOMP_LU, &acc_ok);
    config.gyr_M_inv = config.gyr_M.inv(cv::DECOMP_LU, &gyr_ok);
    if (!acc_ok || !gyr_ok) {
        throw std::runtime_error("IMU calibration matrices must be invertible");
    }
    return config;
}

odometry::ImuSample convertImuSample(const ImuIntrinsicsConfig& config,
    const sensors::ImuPayload& raw, double device_time_s)
{
    cv::Vec3d accel(
        static_cast<double>(raw.acc[0]) / config.acc_lsb_per_g * kGravity,
        static_cast<double>(raw.acc[1]) / config.acc_lsb_per_g * kGravity,
        static_cast<double>(raw.acc[2]) / config.acc_lsb_per_g * kGravity);
    cv::Vec3d gyro(
        static_cast<double>(raw.gyro[0]) / config.gyro_lsb_per_dps * kPi / 180.0,
        static_cast<double>(raw.gyro[1]) / config.gyro_lsb_per_dps * kPi / 180.0,
        static_cast<double>(raw.gyro[2]) / config.gyro_lsb_per_dps * kPi / 180.0);

    if (config.enable) {
        // a_body = inv(M_acc) * a_raw
        // w_body = C^T * inv(M_gyr) * (w_raw - A * C * a_body)
        const cv::Vec3d accel_body = config.acc_M_inv * accel;
        const cv::Vec3d gyro_in_accel_frame = config.gyr_C * accel_body;
        gyro = config.gyr_C.t()
            * (config.gyr_M_inv * (gyro - config.gyr_A * gyro_in_accel_frame));
        accel = accel_body;
    }

    odometry::ImuSample sample;
    sample.timestamp = device_time_s - config.time_offset_s;
    sample.accel = Eigen::Vector3d(accel[0], accel[1], accel[2]);
    sample.gyro = Eigen::Vector3d(gyro[0], gyro[1], gyro[2]);
    return sample;
}

} // namespace flyrat::pilot

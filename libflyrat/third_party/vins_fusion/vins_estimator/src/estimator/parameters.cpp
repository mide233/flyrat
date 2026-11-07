/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"

#include <cmath>
#include <filesystem>
#include <stdexcept>

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G { 0.0, 0.0, 9.8 };

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

Eigen::Matrix4d read_transform(cv::FileStorage& settings, const char* key)
{
    cv::Mat cv_T;
    settings[key] >> cv_T;
    require(cv_T.rows == 4 && cv_T.cols == 4,
        "VINS configuration needs a 4x4 body_T_cam matrix");
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_T, T);
    require(T.allFinite(), "VINS body_T_cam matrix contains non-finite values");
    const Eigen::Matrix3d rotation = T.block<3, 3>(0, 0);
    require((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() < 1e-3,
        "VINS body_T_cam rotation is not orthonormal");
    require(std::abs(rotation.determinant() - 1.0) < 1e-3,
        "VINS body_T_cam rotation determinant is not 1");
    return T;
}

} // namespace

void readParameters(std::string config_file)
{
    if (config_file.empty()) {
        throw std::runtime_error("VINS config path is empty");
    }
    if (!std::filesystem::is_regular_file(config_file)) {
        throw std::runtime_error("VINS config file does not exist: " + config_file);
    }

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened()) {
        throw std::runtime_error("cannot open VINS config: " + config_file);
    }

    // The upstream parser uses process-wide globals. Clear the vectors so the
    // same process can re-read a configuration (for example after a reset).
    RIC.clear();
    TIC.clear();
    CAM_NAMES.clear();
    G = Eigen::Vector3d(0.0, 0.0, 9.8);

    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];
    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_IMU = fsSettings["imu"];
    if (USE_IMU) {
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
        require(G.z() > 0.0, "VINS g_norm must be positive");
    }

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2) {
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
    } else {
        const Eigen::Matrix4d T = read_transform(fsSettings, "body_T_cam0");
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    NUM_OF_CAM = fsSettings["num_of_cam"];
    require(NUM_OF_CAM == 1 || NUM_OF_CAM == 2,
        "VINS num_of_cam must be 1 or 2");

    const std::filesystem::path config_path(config_file);
    const std::filesystem::path config_dir = config_path.parent_path();

    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    require(!cam0Calib.empty(), "VINS cam0_calib is missing");
    const std::filesystem::path cam0_path = config_dir / cam0Calib;
    if (!std::filesystem::is_regular_file(cam0_path)) {
        throw std::runtime_error("VINS cam0_calib file does not exist: " + cam0_path.string());
    }
    CAM_NAMES.push_back(cam0_path.string());

    if (NUM_OF_CAM == 2) {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        require(!cam1Calib.empty(), "VINS cam1_calib is missing");
        const std::filesystem::path cam1_path = config_dir / cam1Calib;
        if (!std::filesystem::is_regular_file(cam1_path)) {
            throw std::runtime_error("VINS cam1_calib file does not exist: " + cam1_path.string());
        }
        CAM_NAMES.push_back(cam1_path.string());

        const Eigen::Matrix4d T = read_transform(fsSettings, "body_T_cam1");
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } else {
        STEREO = 0;
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    require(ROW > 0 && COL > 0, "VINS image_width/image_height must be positive");

    if (!USE_IMU) {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
    }

    fsSettings.release();
}

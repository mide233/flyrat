// SPDX-License-Identifier: MIT
//
// SchurVINS (GPL-3.0) stereo-inertial backend adapter.
//
// The vendored frontend is SVO 2.0 with the SchurVINS EKF backend. This file
// only performs calibration parsing, camera/IMU setup and the impedance
// matching between flyrat's Odometry interface and svo::FrameHandlerStereo.
#include "schurvins_backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>
#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection_types.h>
#include <svo/frame_handler_stereo.h>
#include <svo/imu_handler.h>
#include <svo/initialization.h>
#include <svo/reprojector.h>
#include <svo/stereo_triangulation.h>
#include <svo/tracker/feature_tracking_types.h>
#include <vikit/cameras/camera_geometry.h>
#include <vikit/cameras/ncamera.h>
#include <vikit/cameras/pinhole_projection.h>
#include <vikit/cameras/radial_tangential_distortion.h>

namespace flyrat::odometry {

namespace {

using PinholeCamera = vk::cameras::CameraGeometry<
    vk::cameras::PinholeProjection<vk::cameras::RadialTangentialDistortion>>;

struct CameraCalibration {
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
};

struct Calibration {
    CameraCalibration left;
    CameraCalibration right;
    Eigen::Isometry3d T_left_right = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_body_camera = Eigen::Isometry3d::Identity();
    double imu_rate = 200.0;
    double gyro_noise = 0.0;
    double acc_noise = 0.0;
    double gyro_walk = 0.0;
    double acc_walk = 0.0;
};

double read_double(const cv::FileNode& node, const std::string& key)
{
    const cv::FileNode value = node[key];
    if (value.empty() || (!value.isReal() && !value.isInt()))
    {
        throw OdometryError("camera calibration is missing '" + key + "'");
    }
    return static_cast<double>(value);
}

Eigen::Isometry3d read_isometry(const cv::FileNode& node, const std::string& key)
{
    cv::Mat matrix;
    node >> matrix;
    if (node.empty() || matrix.rows != 4 || matrix.cols != 4)
    {
        throw OdometryError(key + " must be a 4x4 matrix");
    }
    cv::Mat matrix64;
    matrix.convertTo(matrix64, CV_64F);
    Eigen::Matrix4d raw = Eigen::Matrix4d::Identity();
    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            raw(row, col) = matrix64.at<double>(row, col);
        }
    }
    const Eigen::Matrix3d rotation = raw.block<3, 3>(0, 0);
    if (!raw.allFinite()
        || (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-3
        || std::abs(rotation.determinant() - 1.0) > 1e-3)
    {
        throw OdometryError(key + " rotation is not orthonormal");
    }
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.linear() = rotation;
    out.translation() = raw.block<3, 1>(0, 3);
    return out;
}

CameraCalibration read_camera(const cv::FileNode& camera, const std::string& name)
{
    CameraCalibration out;
    out.width = static_cast<int>(read_double(camera, "width"));
    out.height = static_cast<int>(read_double(camera, "height"));
    out.fx = read_double(camera[name], "fx");
    out.fy = read_double(camera[name], "fy");
    out.cx = read_double(camera[name], "cx");
    out.cy = read_double(camera[name], "cy");
    out.k1 = read_double(camera[name], "k1");
    out.k2 = read_double(camera[name], "k2");
    out.p1 = read_double(camera[name], "p1");
    out.p2 = read_double(camera[name], "p2");
    if (out.width <= 0 || out.height <= 0 || out.fx <= 0.0 || out.fy <= 0.0)
    {
        throw OdometryError("invalid " + name + " camera intrinsics");
    }
    return out;
}

Calibration load_calibration(const std::string& path)
{
    cv::FileStorage config(path, cv::FileStorage::READ);
    if (!config.isOpened())
    {
        throw OdometryError("cannot open camera configuration: " + path);
    }
    const cv::FileNode camera = config["camera"];
    const cv::FileNode imu = config["imu"];
    if (camera.empty() || imu.empty())
    {
        throw OdometryError("camera configuration needs 'camera' and 'imu' sections");
    }

    Calibration out;
    out.left = read_camera(camera, "left");
    out.right = read_camera(camera, "right");
    out.T_left_right = read_isometry(camera["T_left_right"], "camera.T_left_right");
    out.T_body_camera = read_isometry(imu["T_body_camera"], "imu.T_body_camera");
    out.imu_rate = read_double(imu, "frequency");
    out.gyro_noise = read_double(imu, "noise_gyro");
    out.acc_noise = read_double(imu, "noise_acc");
    out.gyro_walk = read_double(imu, "gyro_walk");
    out.acc_walk = read_double(imu, "acc_walk");
    if (out.imu_rate <= 0.0)
    {
        throw OdometryError("imu.frequency must be positive");
    }
    return out;
}

svo::Transformation to_svo(const Eigen::Isometry3d& transform)
{
    return svo::Transformation(transform.matrix());
}

std::shared_ptr<PinholeCamera> make_camera(const CameraCalibration& calibration)
{
    using Distortion = vk::cameras::RadialTangentialDistortion;
    using Projection = vk::cameras::PinholeProjection<Distortion>;
    return std::make_shared<PinholeCamera>(
        calibration.width, calibration.height,
        Projection(
            calibration.fx, calibration.fy, calibration.cx, calibration.cy,
            Distortion(calibration.k1, calibration.k2, calibration.p1, calibration.p2)));
}

struct TrajectorySample {
    double timestamp = 0.0;
    Eigen::Isometry3d T_world_camera = Eigen::Isometry3d::Identity();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace

struct SchurvinsBackend::Impl {
    explicit Impl(const std::string& path, const std::string& trace)
        : camera_config_path(path)
        , trace_dir(trace)
        , calibration(load_calibration(path))
    {
        static std::once_flag glog_once;
        std::call_once(glog_once, [] {
            google::InitGoogleLogging("flyrat_odometry");
            google::SetStderrLogging(google::GLOG_WARNING);
        });
        build();
    }

    void build()
    {
        std::vector<std::shared_ptr<vk::cameras::Camera>> cameras;
        cameras.push_back(make_camera(calibration.left));
        cameras.push_back(make_camera(calibration.right));

        // NCamera expects T_C_B (body -> camera). The YAML stores the inverse,
        // T_body_camera, for the left camera, and T_left_right for the rig.
        const Eigen::Isometry3d T_C_B_left = calibration.T_body_camera.inverse();
        const Eigen::Isometry3d T_C_B_right = calibration.T_left_right.inverse() * T_C_B_left;
        vk::cameras::TransformationVector T_C_B;
        T_C_B.push_back(to_svo(T_C_B_left));
        T_C_B.push_back(to_svo(T_C_B_right));

        const svo::CameraBundle::Ptr camera_bundle =
            std::make_shared<svo::CameraBundle>(T_C_B, cameras, "flyrat_stereo");

        svo::BaseOptions base;
        base.max_n_kfs = 10;
        base.kfselect_criterion = svo::KeyframeCriterion::FORWARD;
        base.kfselect_min_dist_metric = 0.1;
        base.kfselect_min_angle = 20.0;
        base.kfselect_min_disparity = 40.0;
        base.kfselect_numkfs_upper_thresh = 140;
        base.kfselect_numkfs_lower_thresh = 50;
        base.quality_min_fts = 30;
        base.quality_max_fts_drop = 60;
        base.poseoptim_thresh = 2.0;
        base.img_align_max_level = 4;
        base.img_align_min_level = 2;
        base.use_imu = true;
        // The stereo reprojector processes both cameras sequentially. Running
        // them concurrently uses the spare cores on the pilot SoC and cuts the
        // reprojection wall time from sum(cam) to max(cam).
        base.use_async_reprojectors = true;
        base.poseoptim_prior_lambda = 0.0;
        base.img_align_prior_lambda_rot = 0.5;
        base.img_align_prior_lambda_trans = 0.0;
        base.update_seeds_with_old_keyframes = true;
        if (!trace_dir.empty()) {
            std::error_code error;
            std::filesystem::create_directories(trace_dir, error);
            if (error) {
                std::fprintf(stderr,
                    "schurvins: cannot create trace directory %s: %s\n",
                    trace_dir.c_str(), error.message().c_str());
            }
            base.trace_statistics = true;
            base.trace_dir = trace_dir;
        }
        base.window_size_ = 4;
        base.obs_dev_ = 2.0;

        svo::DetectorOptions detector;
        detector.cell_size = 25;
        detector.max_level = 2;
        detector.threshold_primary = 10;
        detector.threshold_secondary = 200;
        detector.threshold_shitomasi = 50;
        detector.detector_type = svo::DetectorType::kFast;

        svo::DepthFilterOptions depth_filter;
        depth_filter.max_search_level = 2;
        depth_filter.use_threaded_depthfilter = true;
        depth_filter.seed_convergence_sigma2_thresh = 200.0;
        depth_filter.mappoint_convergence_sigma2_thresh = 500.0;
        depth_filter.affine_est_offset = true;
        depth_filter.affine_est_gain = false;
        depth_filter.max_n_seeds_per_frame = 540;
        depth_filter.max_map_seeds_per_frame = 300;

        svo::InitializationOptions initialization;
        initialization.init_type = svo::InitializerType::kStereo;
        initialization.init_min_features = 40;
        initialization.init_min_tracked = 40;
        initialization.init_min_inliers = 30;
        initialization.init_min_disparity = 10.0;
        initialization.init_min_features_factor = 1.5;
        initialization.reproj_error_thresh = 2.0;

        svo::StereoTriangulationOptions stereo;
        stereo.triangulate_n_features = 180;
        stereo.mean_depth_inv = 1.0 / 1.5;
        stereo.min_depth_inv = 1.0 / 20.0;
        stereo.max_depth_inv = 1.0 / 0.25;

        svo::ReprojectorOptions reprojector;
        reprojector.max_n_features_per_frame = 180;
        reprojector.max_n_kfs = 5;
        reprojector.cell_size = 25;
        reprojector.reproject_unconverged_seeds = true;
        reprojector.max_unconverged_seeds_ratio = 0.2;
        reprojector.min_required_features = 30;
        reprojector.seed_sigma2_thresh = 200.0;
        reprojector.affine_est_offset = true;
        reprojector.affine_est_gain = false;
        reprojector.remove_unconstrained_points = true;

        svo::FeatureTrackerOptions tracker;
        tracker.klt_max_level = 4;
        tracker.klt_min_level = 0;

        svo::ImuCalibration imu_calibration;
        imu_calibration.gyro_noise_density = calibration.gyro_noise;
        imu_calibration.acc_noise_density = calibration.acc_noise;
        imu_calibration.gyro_bias_random_walk_sigma = calibration.gyro_walk;
        imu_calibration.acc_bias_random_walk_sigma = calibration.acc_walk;
        imu_calibration.imu_rate = calibration.imu_rate;
        imu_calibration.max_imu_delta_t = 0.02;
        imu_calibration.delay_imu_cam = 0.0;
        svo::ImuInitialization imu_initialization;
        svo::IMUHandlerOptions imu_options;
        imu_handler =
            std::make_shared<svo::ImuHandler>(imu_calibration, imu_initialization, imu_options);

        frontend = std::make_shared<svo::FrameHandlerStereo>(
            base, depth_filter, detector, initialization, stereo, reprojector, tracker,
            camera_bundle);
        // setImuHandler() must run after construction: it also forwards the IMU
        // noise model to SchurVINS.
        frontend->setImuHandler(imu_handler);
        frontend->start();
    }

    Result result_for(double timestamp) const
    {
        Result result;
        result.timestamp = timestamp;
        switch (frontend->stage())
        {
        case svo::Stage::kTracking:
            result.state = frontend->trackingQuality() == svo::TrackingQuality::kInsufficient
                               ? State::kRecentlyLost
                               : State::kTracking;
            break;
        case svo::Stage::kRelocalization:
            result.state = State::kRecentlyLost;
            break;
        case svo::Stage::kInitializing:
            result.state = State::kNotInitialized;
            break;
        case svo::Stage::kPaused:
        default:
            result.state = had_tracking ? State::kLost : State::kNotInitialized;
            break;
        }

        if (result.state != State::kTracking)
        {
            return result;
        }

        const svo::FrameBundlePtr frames = frontend->getLastFrames();
        if (!frames)
        {
            result.state = State::kRecentlyLost;
            return result;
        }
        const Eigen::Matrix4d T_W_B = frames->get_T_W_B().getTransformationMatrix();
        if (!T_W_B.allFinite())
        {
            result.state = State::kRecentlyLost;
            return result;
        }
        Eigen::Isometry3d world_body = Eigen::Isometry3d::Identity();
        world_body.linear() = T_W_B.block<3, 3>(0, 0);
        world_body.translation() = T_W_B.block<3, 1>(0, 3);
        const Eigen::Isometry3d world_camera = world_body * calibration.T_body_camera;
        if (!world_camera.matrix().allFinite())
        {
            result.state = State::kRecentlyLost;
            return result;
        }
        result.pose.position = world_camera.translation();
        result.pose.orientation = Eigen::Quaterniond(world_camera.linear()).normalized();
        result.pose.valid = true;
        return result;
    }

    std::string camera_config_path;
    std::string trace_dir;
    Calibration calibration;
    svo::FrameHandlerStereo::Ptr frontend;
    svo::ImuHandlerPtr imu_handler;

    std::uint64_t last_timestamp_ns = 0;
    double last_imu_timestamp = -std::numeric_limits<double>::infinity();
    double first_imu_timestamp = std::numeric_limits<double>::infinity();
    std::size_t imu_sample_count = 0;
    bool had_tracking = false;

    mutable std::mutex trajectory_mutex;
    std::vector<TrajectorySample, Eigen::aligned_allocator<TrajectorySample>> trajectory;
};

SchurvinsBackend::SchurvinsBackend(const std::string& camera_config_path,
    const std::string& trace_dir)
    : impl_(std::make_unique<Impl>(camera_config_path, trace_dir))
{}

SchurvinsBackend::~SchurvinsBackend() = default;

Result SchurvinsBackend::track(
    const ImageView& left, const ImageView& right, double timestamp,
    const std::vector<ImuSample>& imu)
{
    if (!std::isfinite(timestamp))
    {
        throw OdometryError("frame timestamp is not finite");
    }
    if (left.width != impl_->calibration.left.width || left.height != impl_->calibration.left.height
        || right.width != impl_->calibration.right.width
        || right.height != impl_->calibration.right.height)
    {
        throw OdometryError("image size does not match the camera calibration");
    }

    for (const ImuSample& sample : imu)
    {
        if (!std::isfinite(sample.timestamp) || !sample.accel.allFinite()
            || !sample.gyro.allFinite())
        {
            continue;
        }
        if (sample.timestamp <= impl_->last_imu_timestamp)
        {
            continue;
        }
        impl_->imu_handler->addImuMeasurement(
            svo::ImuMeasurement(sample.timestamp, sample.gyro, sample.accel));
        impl_->last_imu_timestamp = sample.timestamp;
        impl_->first_imu_timestamp = std::min(impl_->first_imu_timestamp, sample.timestamp);
        ++impl_->imu_sample_count;
    }

    // SchurVINS interpolates IMU measurements across the image timestamp, so
    // the frontend aborts (CHECK) unless the buffer already contains a sample
    // after the frame. The ROS node supplies a short lookahead; skip frames
    // whose lookahead has not arrived yet instead of letting SVO abort.
    const bool imu_ready = impl_->imu_sample_count >= 2 && impl_->first_imu_timestamp <= timestamp
                        && impl_->last_imu_timestamp > timestamp;

    const std::uint64_t timestamp_ns = static_cast<std::uint64_t>(std::llround(timestamp * 1e9));
    if (timestamp_ns <= impl_->last_timestamp_ns)
    {
        Result result;
        result.timestamp = timestamp;
        result.state = impl_->had_tracking ? State::kRecentlyLost : State::kNotInitialized;
        return result;
    }

    if (!imu_ready)
    {
        Result result;
        result.timestamp = timestamp;
        result.state = impl_->had_tracking ? State::kRecentlyLost : State::kNotInitialized;
        return result;
    }

    const cv::Mat left_view(
        left.height, left.width, CV_8UC1, const_cast<std::uint8_t*>(left.data),
        static_cast<std::size_t>(left.stride));
    const cv::Mat right_view(
        right.height, right.width, CV_8UC1, const_cast<std::uint8_t*>(right.data),
        static_cast<std::size_t>(right.stride));

    const bool accepted = impl_->frontend->addImageBundle({left_view, right_view}, timestamp_ns);
    impl_->last_timestamp_ns = timestamp_ns;

    Result result = impl_->result_for(timestamp);
    if (!accepted && result.state == State::kTracking)
    {
        result.state = State::kRecentlyLost;
        result.pose.valid = false;
    }
    if (result.pose.valid)
    {
        impl_->had_tracking = true;
        TrajectorySample sample;
        sample.timestamp = timestamp;
        sample.T_world_camera.linear() = result.pose.orientation.toRotationMatrix();
        sample.T_world_camera.translation() = result.pose.position;
        std::lock_guard<std::mutex> lock(impl_->trajectory_mutex);
        impl_->trajectory.push_back(sample);
    }
    return result;
}

void SchurvinsBackend::reset()
{
    impl_->last_timestamp_ns = 0;
    impl_->last_imu_timestamp = -std::numeric_limits<double>::infinity();
    impl_->first_imu_timestamp = std::numeric_limits<double>::infinity();
    impl_->imu_sample_count = 0;
    impl_->had_tracking = false;
    {
        std::lock_guard<std::mutex> lock(impl_->trajectory_mutex);
        impl_->trajectory.clear();
    }
    // Rebuilding the frontend clears the SVO map, SchurVINS EKF state and the
    // IMU buffer; svo::FrameHandlerBase::reset() alone leaves the EKF alive.
    impl_->build();
}

void SchurvinsBackend::saveTrajectory(const std::string& path) const
{
    std::lock_guard<std::mutex> lock(impl_->trajectory_mutex);
    std::ofstream output(path);
    if (!output)
    {
        throw OdometryError("cannot open trajectory output: " + path);
    }
    output.precision(17);
    for (const TrajectorySample& sample : impl_->trajectory)
    {
        const Eigen::Quaterniond orientation(sample.T_world_camera.linear());
        const Eigen::Vector3d& position = sample.T_world_camera.translation();
        output << sample.timestamp << ' ' << position.x() << ' ' << position.y() << ' '
               << position.z() << ' ' << orientation.x() << ' ' << orientation.y() << ' '
               << orientation.z() << ' ' << orientation.w() << '\n';
    }
}

} // namespace flyrat::odometry

// SPDX-License-Identifier: MIT
//
// Odometry node for the real-vehicle sensor stream.
//
// The receiver publishes the camera images and the IMU asynchronously over ROS
// 2. This node synchronises the two images, keeps a short IMU history, and
// runs the estimator on a dedicated worker thread. Only the newest
// unsynchronised stereo pair is kept, so a slow estimator drops frames instead
// of building latency.
//
// The backend is still a stub (see flyrat::odometry::Odometry): it reports
// NOT_INITIALIZED until the estimator is implemented. Everything around it -
// synchronisation, TF, path, diagnostics and reset - is ready.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/imgproc.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "flyrat/odometry/odometry.hpp"

namespace flyrat::odometry {

namespace {

using Image = sensor_msgs::msg::Image;
using StereoPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

const char* state_name(State state)
{
    switch (state) {
    case State::kTracking:
        return "TRACKING";
    case State::kRecentlyLost:
        return "RECENTLY_LOST";
    case State::kLost:
        return "LOST";
    case State::kNotInitialized:
    default:
        return "NOT_INITIALIZED";
    }
}

std::uint8_t diagnostic_level(State state)
{
    switch (state) {
    case State::kTracking:
        return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case State::kRecentlyLost:
    case State::kNotInitialized:
        return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    case State::kLost:
    default:
        return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
}

diagnostic_msgs::msg::KeyValue value(const std::string& key, const std::string& text)
{
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = text;
    return item;
}

cv::Mat to_gray(const Image& msg)
{
    if (msg.height == 0 || msg.width == 0 || msg.step == 0) {
        return {};
    }
    if (msg.data.size() < static_cast<std::size_t>(msg.step) * msg.height) {
        return {};
    }

    int type = 0;
    int conversion = 0;
    if (msg.encoding == "mono8" || msg.encoding == "8UC1") {
        type = CV_8UC1;
    } else if (msg.encoding == "bgr8") {
        type = CV_8UC3;
        conversion = cv::COLOR_BGR2GRAY;
    } else if (msg.encoding == "rgb8") {
        type = CV_8UC3;
        conversion = cv::COLOR_RGB2GRAY;
    } else if (msg.encoding == "bgra8") {
        type = CV_8UC4;
        conversion = cv::COLOR_BGRA2GRAY;
    } else if (msg.encoding == "rgba8") {
        type = CV_8UC4;
        conversion = cv::COLOR_RGBA2GRAY;
    } else {
        return {};
    }

    cv::Mat view(msg.height, msg.width, type,
        const_cast<unsigned char*>(msg.data.data()), msg.step);
    if (type == CV_8UC1) {
        return view.clone();
    }
    cv::Mat gray;
    cv::cvtColor(view, gray, conversion);
    return gray;
}

} // namespace

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode()
        : Node("odometry_node")
    {
        camera_config_path_ = declare_parameter<std::string>("camera_config_path", "");
        left_topic_ = declare_parameter<std::string>(
            "left_topic", "/flyrat/sensors/stereo/left/image_raw");
        right_topic_ = declare_parameter<std::string>(
            "right_topic", "/flyrat/sensors/stereo/right/image_raw");
        imu_topic_ = declare_parameter<std::string>("imu_topic", "/flyrat/sensors/imu");
        mode_ = declare_parameter<std::string>("mode", "imu_stereo");
        world_frame_ = declare_parameter<std::string>("world_frame", "odometry_map");
        body_frame_ = declare_parameter<std::string>("body_frame", "imu_link");
        camera_frame_ = declare_parameter<std::string>(
            "camera_frame", "camera_left_optical_frame");
        right_camera_frame_ = declare_parameter<std::string>(
            "right_camera_frame", "camera_right_optical_frame");
        sync_slop_ = declare_parameter<double>("sync_slop", 0.010);
        imu_buffer_seconds_ = declare_parameter<double>("imu_buffer_seconds", 1.0);
        path_max_poses_ = declare_parameter<int>("path_max_poses", 1000);
        path_publish_period_ = declare_parameter<double>("path_publish_period", 0.2);
        status_period_ = declare_parameter<double>("status_period", 1.0);
        trajectory_path_ = declare_parameter<std::string>("trajectory_path", "");

        if (camera_config_path_.empty()) {
            throw std::runtime_error("camera_config_path is required");
        }
        if (mode_ != "stereo" && mode_ != "imu_stereo") {
            throw std::runtime_error("mode must be 'stereo' or 'imu_stereo'");
        }
        if (body_frame_.empty() || camera_frame_.empty() ||
            body_frame_ == camera_frame_) {
            throw std::runtime_error(
                "body_frame and camera_frame must be non-empty and different");
        }
        if (right_camera_frame_.empty() || right_camera_frame_ == camera_frame_ ||
            right_camera_frame_ == body_frame_) {
            throw std::runtime_error(
                "right_camera_frame must be non-empty and differ from body/camera frames");
        }
        load_extrinsics();
        if (sync_slop_ <= 0.0 || imu_buffer_seconds_ <= 0.0 ||
            path_publish_period_ <= 0.0 || status_period_ <= 0.0) {
            throw std::runtime_error("time parameters must be positive");
        }
        if (path_max_poses_ < 2) {
            throw std::runtime_error("path_max_poses must be at least 2");
        }

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/flyrat/odometry/odometry", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/flyrat/odometry/path", rclcpp::QoS(1).transient_local());
        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            "/flyrat/odometry/status", 10);
        reset_service_ = create_service<std_srvs::srv::Trigger>(
            "/flyrat/odometry/reset",
            std::bind(&OdometryNode::on_reset, this, std::placeholders::_1,
                std::placeholders::_2));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        publish_static_extrinsics();

        path_.header.frame_id = world_frame_;

        rclcpp::QoS image_qos = rclcpp::SensorDataQoS();
        image_qos.keep_last(2);
        const rmw_qos_profile_t qos = image_qos.get_rmw_qos_profile();
        left_sub_.subscribe(this, left_topic_, qos);
        right_sub_.subscribe(this, right_topic_, qos);
        sync_ = std::make_unique<message_filters::Synchronizer<StereoPolicy>>(
            StereoPolicy(10), left_sub_, right_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_));
        sync_->registerCallback(
            std::bind(&OdometryNode::on_stereo, this, std::placeholders::_1,
                std::placeholders::_2));

        use_imu_ = mode_ == "imu_stereo";
        const bool use_imu = use_imu_;
        if (use_imu) {
            imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
                imu_topic_, rclcpp::SensorDataQoS(),
                std::bind(&OdometryNode::on_imu, this, std::placeholders::_1));
        }

        Options options;
        options.mode = use_imu ? Mode::kImuStereo : Mode::kStereo;
        options.camera_config_path = camera_config_path_;
        RCLCPP_INFO(get_logger(), "loading %s odometry calibration %s",
            use_imu ? "IMU-stereo" : "stereo", camera_config_path_.c_str());
        odometry_ = std::make_unique<Odometry>(options);
        RCLCPP_WARN(get_logger(),
            "odometry backend is not implemented; reporting NOT_INITIALIZED");
        const std::string imu_suffix = use_imu ? ", " + imu_topic_ : "";
        RCLCPP_INFO(get_logger(),
            "subscribed to %s, %s%s; publishing /flyrat/odometry/{odometry,path,status}",
            left_topic_.c_str(), right_topic_.c_str(), imu_suffix.c_str());

        last_status_time_ = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
                                .count();
        worker_ = std::thread(&OdometryNode::worker_loop, this);
        status_timer_ = create_wall_timer(
            std::chrono::milliseconds(
                static_cast<int>(status_period_ * 1000.0)),
            std::bind(&OdometryNode::publish_status, this));
    }

    ~OdometryNode() override
    {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_ = true;
        }
        queue_cv_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }

        if (odometry_) {
            try {
                odometry_->shutdown();
            } catch (const std::exception& error) {
                RCLCPP_ERROR(get_logger(), "failed to shut down odometry: %s", error.what());
            }
            if (!trajectory_path_.empty()) {
                try {
                    odometry_->saveTrajectory(trajectory_path_);
                    RCLCPP_INFO(get_logger(), "saved trajectory to %s",
                        trajectory_path_.c_str());
                } catch (const std::exception& error) {
                    RCLCPP_ERROR(get_logger(), "failed to save trajectory: %s", error.what());
                }
            }
        }
    }

private:
    struct PendingFrame {
        cv::Mat left;
        cv::Mat right;
        rclcpp::Time stamp;
        double stamp_s = 0.0;
    };

    void load_extrinsics()
    {
        cv::FileStorage config(camera_config_path_, cv::FileStorage::READ);
        if (!config.isOpened()) {
            throw std::runtime_error(
                "cannot open camera configuration: " + camera_config_path_);
        }

        const auto read_isometry = [&](const cv::FileNode& node,
                                       const std::string& key) {
            cv::Mat node_mat;
            node >> node_mat;
            if (node.empty() || node_mat.rows != 4 || node_mat.cols != 4) {
                throw std::runtime_error(
                    key + " must be a 4x4 matrix in " + camera_config_path_);
            }
            cv::Mat mat64;
            node_mat.convertTo(mat64, CV_64F);
            Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    matrix(row, col) = mat64.at<double>(row, col);
                }
            }
            const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
            if (!rotation.allFinite() ||
                (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-3 ||
                std::abs(rotation.determinant() - 1.0) > 1e-3) {
                throw std::runtime_error(key + " rotation is not orthonormal");
            }
            Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
            out.linear() = rotation;
            out.translation() = matrix.block<3, 1>(0, 3);
            return out;
        };

        // imu.T_body_camera maps a point in the left optical frame into the
        // IMU/body frame. That is also the pose of the camera in the body
        // frame, exactly what the imu_link -> camera static TF must broadcast.
        T_body_camera_ = read_isometry(
            config["imu"]["T_body_camera"], "imu.T_body_camera");
        // Composing T_world_camera into T_world_body needs the inverse.
        T_camera_body_ = T_body_camera_.inverse();

        // camera.T_left_right maps right-camera coordinates into the left
        // camera frame (p_left = T * p_right), i.e. the pose of the right
        // camera expressed in the left camera frame.
        const bool has_stereo_extrinsic =
            !config["camera"]["T_left_right"].empty();
        if (has_stereo_extrinsic) {
            T_left_right_ = read_isometry(
                config["camera"]["T_left_right"], "camera.T_left_right");
            has_right_camera_extrinsic_ = true;
        } else {
            RCLCPP_WARN(get_logger(),
                "camera.T_left_right is missing; not publishing %s -> %s",
                camera_frame_.c_str(), right_camera_frame_.c_str());
        }
    }

    void publish_static_extrinsics()
    {
        const auto make_transform = [&](const std::string& parent,
                                        const std::string& child,
                                        const Eigen::Isometry3d& pose) {
            const Eigen::Quaterniond orientation(pose.linear());
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = now();
            transform.header.frame_id = parent;
            transform.child_frame_id = child;
            transform.transform.translation.x = pose.translation().x();
            transform.transform.translation.y = pose.translation().y();
            transform.transform.translation.z = pose.translation().z();
            transform.transform.rotation.x = orientation.x();
            transform.transform.rotation.y = orientation.y();
            transform.transform.rotation.z = orientation.z();
            transform.transform.rotation.w = orientation.w();
            return transform;
        };

        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        transforms.push_back(make_transform(
            body_frame_, camera_frame_, T_body_camera_));
        if (has_right_camera_extrinsic_) {
            transforms.push_back(make_transform(
                camera_frame_, right_camera_frame_, T_left_right_));
        }
        static_tf_broadcaster_->sendTransform(transforms);
    }

    void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
    {
        const double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        if (!std::isfinite(timestamp)) {
            return;
        }

        ImuSample sample;
        sample.timestamp = timestamp;
        sample.accel = Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z);
        sample.gyro = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z);

        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            imu_buffer_.push_back(sample);
            const double cutoff = timestamp - imu_buffer_seconds_;
            while (!imu_buffer_.empty() && imu_buffer_.front().timestamp < cutoff) {
                imu_buffer_.pop_front();
            }
            imu_buffer_size_.store(imu_buffer_.size());
        }
        last_imu_stamp_.store(timestamp);
    }

    void on_stereo(const Image::ConstSharedPtr& left_msg,
        const Image::ConstSharedPtr& right_msg)
    {
        if (left_msg->header.stamp.sec == 0 && left_msg->header.stamp.nanosec == 0) {
            return;
        }

        cv::Mat left = to_gray(*left_msg);
        cv::Mat right = to_gray(*right_msg);
        if (left.empty() || right.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "unsupported or malformed stereo image encoding: '%s' / '%s'",
                left_msg->encoding.c_str(), right_msg->encoding.c_str());
            return;
        }
        if (left.size() != right.size()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "left/right image size mismatch: %dx%d vs %dx%d",
                left.cols, left.rows, right.cols, right.rows);
            return;
        }

        PendingFrame frame;
        frame.left = std::move(left);
        frame.right = std::move(right);
        frame.stamp = rclcpp::Time(left_msg->header.stamp);
        frame.stamp_s = frame.stamp.seconds();

        input_pairs_.fetch_add(1);
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (pending_) {
                dropped_frames_.fetch_add(1);
            }
            pending_ = std::move(frame);
        }
        queue_cv_.notify_one();
    }

    void worker_loop()
    {
        while (true) {
            PendingFrame frame;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this] { return stop_ || pending_.has_value(); });
                if (stop_ && !pending_) {
                    return;
                }
                frame = std::move(*pending_);
                pending_.reset();
            }
            process(std::move(frame));
        }
    }

    void process(PendingFrame frame)
    {
        double previous_stamp = 0.0;
        std::uint64_t epoch = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            previous_stamp = last_frame_stamp_;
            epoch = reset_epoch_.load();
        }
        if (previous_stamp >= 0.0 && frame.stamp_s <= previous_stamp) {
            dropped_frames_.fetch_add(1);
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "stereo timestamp did not advance (%.9f <= %.9f)",
                frame.stamp_s, previous_stamp);
            return;
        }

        std::vector<ImuSample> imu;
        if (previous_stamp >= 0.0) {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            for (const ImuSample& sample : imu_buffer_) {
                if (sample.timestamp > previous_stamp &&
                    sample.timestamp <= frame.stamp_s) {
                    imu.push_back(sample);
                }
            }
            while (!imu_buffer_.empty() &&
                imu_buffer_.front().timestamp <= frame.stamp_s) {
                imu_buffer_.pop_front();
            }
            imu_buffer_size_.store(imu_buffer_.size());
        }

        Result result;
        try {
            const ImageView left_view { frame.left.cols, frame.left.rows,
                static_cast<int>(frame.left.step), frame.left.data };
            const ImageView right_view { frame.right.cols, frame.right.rows,
                static_cast<int>(frame.right.step), frame.right.data };
            std::lock_guard<std::mutex> lock(odometry_mutex_);
            result = odometry_->track(left_view, right_view, frame.stamp_s, imu);
        } catch (const std::exception& error) {
            failed_frames_.fetch_add(1);
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                "odometry tracking failed: %s", error.what());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // A reset may have happened while track() was running; drop this
            // result instead of repopulating the freshly cleared path.
            if (reset_epoch_.load() != epoch) {
                return;
            }
            last_frame_stamp_ = frame.stamp_s;
            if (result.pose.valid) {
                const Eigen::Isometry3d world_body = to_world_body(result);
                const Eigen::Quaterniond orientation(world_body.linear());
                geometry_msgs::msg::PoseStamped pose;
                pose.header.stamp = frame.stamp;
                pose.header.frame_id = world_frame_;
                pose.pose.position.x = world_body.translation().x();
                pose.pose.position.y = world_body.translation().y();
                pose.pose.position.z = world_body.translation().z();
                pose.pose.orientation.x = orientation.x();
                pose.pose.orientation.y = orientation.y();
                pose.pose.orientation.z = orientation.z();
                pose.pose.orientation.w = orientation.w();
                path_.poses.push_back(std::move(pose));
                while (static_cast<int>(path_.poses.size()) > path_max_poses_) {
                    path_.poses.erase(path_.poses.begin());
                }
                path_.header.stamp = frame.stamp;
            }
        }

        state_.store(result.state);
        processed_frames_.fetch_add(1);

        if (result.pose.valid) {
            publish_pose(result, frame.stamp);
            publish_path_if_due();
        }
    }

    Eigen::Isometry3d to_world_body(const Result& result)
    {
        Eigen::Isometry3d world_camera = Eigen::Isometry3d::Identity();
        world_camera.linear() = result.pose.orientation.toRotationMatrix();
        world_camera.translation() = result.pose.position;
        Eigen::Isometry3d world_body = world_camera * T_camera_body_;

        // A pure-stereo backend may define its world at the first left-camera
        // frame, whose axes are optical (x right, y down, z forward). Rebase
        // the world to the first tracked body pose so odometry_map is the
        // initial imu_link frame (FLU: x forward, y left, z up). Without this,
        // a forward motion shows up as +z and a climb as -y in Foxglove.
        if (!use_imu_) {
            std::lock_guard<std::mutex> lock(world_origin_mutex_);
            if (!have_body_world_origin_) {
                body_world_origin_ = world_body;
                have_body_world_origin_ = true;
                RCLCPP_INFO(get_logger(),
                    "stereo world frame rebased to the first body pose (FLU)");
            }
            return body_world_origin_.inverse() * world_body;
        }
        return world_body;
    }

    void publish_pose(const Result& result, const rclcpp::Time& stamp)
    {
        const Eigen::Isometry3d world_body = to_world_body(result);
        const Eigen::Vector3d& position = world_body.translation();
        const Eigen::Quaterniond orientation(world_body.linear());
        if (!position.allFinite() || !Eigen::Vector4d(orientation.x(),
                orientation.y(), orientation.z(), orientation.w()).allFinite()) {
            return;
        }

        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = stamp;
        odometry.header.frame_id = world_frame_;
        odometry.child_frame_id = body_frame_;
        odometry.pose.pose.position.x = position.x();
        odometry.pose.pose.position.y = position.y();
        odometry.pose.pose.position.z = position.z();
        odometry.pose.pose.orientation.x = orientation.x();
        odometry.pose.pose.orientation.y = orientation.y();
        odometry.pose.pose.orientation.z = orientation.z();
        odometry.pose.pose.orientation.w = orientation.w();
        odom_pub_->publish(odometry);

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = stamp;
        transform.header.frame_id = world_frame_;
        transform.child_frame_id = body_frame_;
        transform.transform.translation.x = position.x();
        transform.transform.translation.y = position.y();
        transform.transform.translation.z = position.z();
        transform.transform.rotation.x = orientation.x();
        transform.transform.rotation.y = orientation.y();
        transform.transform.rotation.z = orientation.z();
        transform.transform.rotation.w = orientation.w();
        tf_broadcaster_->sendTransform(transform);
    }

    void publish_path_if_due()
    {
        const double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
                               .count();
        nav_msgs::msg::Path path;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (last_path_publish_ >= 0.0 &&
                now - last_path_publish_ < path_publish_period_) {
                return;
            }
            last_path_publish_ = now;
            path = path_;
        }
        path_pub_->publish(path);
    }

    void on_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;
        try {
            {
                std::lock_guard<std::mutex> lock(odometry_mutex_);
                odometry_->reset();
            }
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                last_frame_stamp_ = -1.0;
                path_.poses.clear();
                path_.header.stamp = now();
                last_path_publish_ = -1.0;
                reset_epoch_.fetch_add(1);
            }
            {
                std::lock_guard<std::mutex> lock(world_origin_mutex_);
                have_body_world_origin_ = false;
                body_world_origin_.setIdentity();
            }
            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                imu_buffer_.clear();
                imu_buffer_size_.store(0);
            }
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                pending_.reset();
            }
            state_.store(State::kNotInitialized);
            reset_count_.fetch_add(1);
            response->success = true;
            response->message = "odometry reset";
            RCLCPP_INFO(get_logger(), "odometry reset requested");
        } catch (const std::exception& error) {
            response->success = false;
            response->message = error.what();
            RCLCPP_ERROR(get_logger(), "odometry reset failed: %s", error.what());
        }
    }

    void publish_status()
    {
        const double now_steady = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
                                      .count();
        const double elapsed = now_steady - last_status_time_;
        last_status_time_ = now_steady;
        if (elapsed <= 0.0) {
            return;
        }

        const std::uint64_t input = input_pairs_.load();
        const std::uint64_t processed = processed_frames_.load();
        const std::uint64_t dropped = dropped_frames_.load();
        const std::uint64_t failed = failed_frames_.load();
        const double input_fps = static_cast<double>(input - last_input_count_) / elapsed;
        const double processed_fps =
            static_cast<double>(processed - last_processed_count_) / elapsed;
        last_input_count_ = input;
        last_processed_count_ = processed;

        const State state = state_.load();
        const double last_imu = last_imu_stamp_.load();
        const double imu_age = last_imu >= 0.0 ? now().seconds() - last_imu : -1.0;
        const bool imu_missing = last_imu < 0.0;
        const bool imu_stale = !imu_missing && imu_age > 0.5;
        const bool pose_valid = state == State::kTracking;

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "flyrat_odometry";
        status.hardware_id = "flyrat_odometry";
        status.level = diagnostic_level(state);

        // Device and host clocks are only approximately aligned, so a small
        // negative age means "just arrived", not "no data"; only an empty
        // buffer counts as missing.
        if (imu_missing || imu_stale) {
            status.level = std::max<uint8_t>(
                status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
        }
        status.message = state_name(state);
        if (imu_missing) {
            status.message += "; no IMU data";
        } else if (imu_stale) {
            status.message += "; IMU data is stale";
        }

        status.values.push_back(value("state", state_name(state)));
        status.values.push_back(value("pose_valid", pose_valid ? "true" : "false"));
        status.values.push_back(value("input_fps", std::to_string(input_fps)));
        status.values.push_back(value("processed_fps", std::to_string(processed_fps)));
        status.values.push_back(value("input_pairs", std::to_string(input)));
        status.values.push_back(value("processed_frames", std::to_string(processed)));
        status.values.push_back(value("dropped_frames", std::to_string(dropped)));
        status.values.push_back(value("failed_frames", std::to_string(failed)));
        status.values.push_back(value("reset_count", std::to_string(reset_count_.load())));
        status.values.push_back(
            value("imu_buffer_samples", std::to_string(imu_buffer_size_.load())));
        status.values.push_back(value("imu_age_s", std::to_string(imu_age)));
        status.values.push_back(value("last_imu_stamp_s", std::to_string(last_imu)));

        status_pub_->publish(status);
    }

    std::string camera_config_path_;
    std::string left_topic_;
    std::string right_topic_;
    std::string imu_topic_;
    std::string mode_;
    std::string world_frame_;
    std::string body_frame_;
    std::string camera_frame_;
    std::string right_camera_frame_;
    // imu.T_body_camera: pose of the left camera in the body frame, used by
    // the imu_link -> camera static TF.
    Eigen::Isometry3d T_body_camera_ { Eigen::Isometry3d::Identity() };
    // Inverse of T_body_camera_: maps body coordinates into camera
    // coordinates and composes T_world_camera into T_world_body.
    Eigen::Isometry3d T_camera_body_ { Eigen::Isometry3d::Identity() };
    // camera.T_left_right: pose of the right camera in the left camera frame.
    Eigen::Isometry3d T_left_right_ { Eigen::Isometry3d::Identity() };
    bool has_right_camera_extrinsic_ = false;
    bool use_imu_ = false;
    // Pure stereo only: body pose of the first tracked frame, used to rebase
    // odometry_map from the camera optical frame to the initial body frame.
    bool have_body_world_origin_ = false;
    Eigen::Isometry3d body_world_origin_ { Eigen::Isometry3d::Identity() };
    std::mutex world_origin_mutex_;
    double sync_slop_ = 0.010;
    double imu_buffer_seconds_ = 1.0;
    int path_max_poses_ = 1000;
    double path_publish_period_ = 0.2;
    double status_period_ = 1.0;
    std::string trajectory_path_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    message_filters::Subscriber<Image> left_sub_;
    message_filters::Subscriber<Image> right_sub_;
    std::unique_ptr<message_filters::Synchronizer<StereoPolicy>> sync_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::unique_ptr<Odometry> odometry_;
    std::mutex odometry_mutex_;

    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::optional<PendingFrame> pending_;
    bool stop_ = false;
    std::thread worker_;

    std::mutex imu_mutex_;
    std::deque<ImuSample> imu_buffer_;
    std::atomic<double> last_imu_stamp_ { -1.0 };
    std::atomic<std::size_t> imu_buffer_size_ { 0 };

    std::mutex state_mutex_;
    double last_frame_stamp_ = -1.0;
    nav_msgs::msg::Path path_;
    double last_path_publish_ = -1.0;
    std::atomic<std::uint64_t> reset_epoch_ { 0 };

    std::atomic<State> state_ { State::kNotInitialized };
    std::atomic<std::uint64_t> input_pairs_ { 0 };
    std::atomic<std::uint64_t> processed_frames_ { 0 };
    std::atomic<std::uint64_t> dropped_frames_ { 0 };
    std::atomic<std::uint64_t> failed_frames_ { 0 };
    std::atomic<std::uint64_t> reset_count_ { 0 };

    double last_status_time_ = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch())
                                   .count();
    std::uint64_t last_input_count_ = 0;
    std::uint64_t last_processed_count_ = 0;
};

} // namespace flyrat::odometry

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<flyrat::odometry::OdometryNode>();
        rclcpp::spin(node);
        node.reset();
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("odometry_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

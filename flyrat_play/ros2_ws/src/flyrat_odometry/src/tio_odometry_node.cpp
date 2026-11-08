// SPDX-License-Identifier: MIT
//
// TIO-Former ToF-inertial odometry node.
//
// Subscribes to the four 8x8 ToF frames and the IMU published by
// flyrat_sensors, runs the TorchScript TIO-Former backend and publishes the
// same nav_msgs/Odometry, Path, DiagnosticStatus and TF contract as the stereo
// odometry node.  The model consumes four directional range images ordered
// front, left, right, down plus 16 IMU samples per ToF frame.

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "flyrat/tio/tio_odometry.hpp"
#include "flyrat_sensors_interfaces/msg/tof_frame.hpp"

namespace flyrat::tio {

namespace {

using Tof = flyrat_sensors_interfaces::msg::TofFrame;
using TofPolicy = message_filters::sync_policies::ApproximateTime<Tof, Tof, Tof, Tof>;

const char* state_name(odometry::State state)
{
    switch (state) {
    case odometry::State::kTracking:
        return "TRACKING";
    case odometry::State::kRecentlyLost:
        return "RECENTLY_LOST";
    case odometry::State::kLost:
        return "LOST";
    case odometry::State::kNotInitialized:
    default:
        return "NOT_INITIALIZED";
    }
}

std::uint8_t diagnostic_level(odometry::State state)
{
    switch (state) {
    case odometry::State::kTracking:
        return diagnostic_msgs::msg::DiagnosticStatus::OK;
    case odometry::State::kRecentlyLost:
    case odometry::State::kNotInitialized:
        return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    case odometry::State::kLost:
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

bool to_grid(const Tof& message, TofGrid& grid)
{
    const int resolution = static_cast<int>(message.resolution);
    if (resolution != 4 && resolution != 8) {
        return false;
    }
    for (int cell = 0; cell < 64; ++cell) {
        int source = cell;
        if (resolution == 4) {
            source = (cell / 8 / 2) * 4 + (cell % 8 / 2);
        }
        if (source < 0 || source >= static_cast<int>(message.zones.size())) {
            return false;
        }
        const auto& zone = message.zones[source];
        const float range = static_cast<float>(zone.distance_mm) * 1e-3F;
        const bool valid = zone.valid && range >= 0.015F && range <= 4.0F;
        grid.range[cell] = valid ? range : 0.0F;
        grid.weight[cell] = valid ? 1.0F : 0.0F;
    }
    return true;
}

} // namespace

class TioOdometryNode : public rclcpp::Node {
public:
    TioOdometryNode()
        : Node("tio_odometry_node")
    {
        model_path_ = declare_parameter<std::string>("model_path", "");
        world_frame_ = declare_parameter<std::string>("world_frame", "odometry_map");
        body_frame_ = declare_parameter<std::string>("body_frame", "imu_link");
        const int warmup_frames = declare_parameter<int>("warmup_frames", 64);
        path_max_poses_ = declare_parameter<int>("path_max_poses", 1000);
        path_publish_period_ = declare_parameter<double>("path_publish_period", 0.2);
        status_period_ = declare_parameter<double>("status_period", 1.0);
        trajectory_path_ = declare_parameter<std::string>("trajectory_path", "");
        tof_rate_hz_ = declare_parameter<double>("tof_rate_hz", 15.0);
        imu_buffer_seconds_ = declare_parameter<double>("imu_buffer_seconds", 2.0);

        if (model_path_.empty()) {
            throw std::runtime_error("model_path is required");
        }
        if (world_frame_.empty() || body_frame_.empty()) {
            throw std::runtime_error("world_frame and body_frame must be non-empty");
        }
        if (tof_rate_hz_ <= 0.0 || imu_buffer_seconds_ <= 0.0 ||
            path_publish_period_ <= 0.0 || status_period_ <= 0.0) {
            throw std::runtime_error("time parameters must be positive");
        }
        if (path_max_poses_ < 2) {
            throw std::runtime_error("path_max_poses must be at least 2");
        }

        Options options;
        options.model_path = model_path_;
        options.warmup_frames = warmup_frames;
        options.tof_rate_hz = tof_rate_hz_;
        options.imu_buffer_seconds = imu_buffer_seconds_;
        RCLCPP_INFO(get_logger(), "loading TIO-Former model %s", model_path_.c_str());
        odometry_ = std::make_unique<TioOdometry>(options);

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/flyrat/odometry/odometry", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/flyrat/odometry/path", rclcpp::QoS(1).transient_local());
        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            "/flyrat/odometry/status", 10);
        reset_service_ = create_service<std_srvs::srv::Trigger>(
            "/flyrat/odometry/reset",
            std::bind(&TioOdometryNode::on_reset, this, std::placeholders::_1,
                std::placeholders::_2));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        path_.header.frame_id = world_frame_;

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/flyrat/sensors/imu", rclcpp::SensorDataQoS(),
            std::bind(&TioOdometryNode::on_imu, this, std::placeholders::_1));

        const std::array<std::string, 4> defaults {
            "/flyrat/sensors/tof_0", "/flyrat/sensors/tof_1",
            "/flyrat/sensors/tof_2", "/flyrat/sensors/tof_3"
        };
        for (int index = 0; index < 4; ++index) {
            tof_topics_[index] = declare_parameter<std::string>(
                "tof_topic_" + std::to_string(index), defaults[index]);
        }
        const auto sensor_order = declare_parameter<std::vector<int64_t>>(
            "sensor_order", { 1, 3, 2, 0 });
        if (sensor_order.size() != 4) {
            throw std::runtime_error("sensor_order must contain four sensor ids");
        }
        std::array<bool, 4> seen {};
        for (int index = 0; index < 4; ++index) {
            const int id = static_cast<int>(sensor_order[index]);
            if (id < 0 || id >= 4 || seen[id]) {
                throw std::runtime_error(
                    "sensor_order must be a permutation of 0..3");
            }
            seen[id] = true;
            sensor_order_[index] = id;
        }
        RCLCPP_INFO(get_logger(),
            "model views front/left/right/down <- ToF sensors %d/%d/%d/%d",
            sensor_order_[0], sensor_order_[1], sensor_order_[2],
            sensor_order_[3]);

        rclcpp::QoS tof_qos = rclcpp::SensorDataQoS();
        tof_qos.keep_last(4);
        const rmw_qos_profile_t qos = tof_qos.get_rmw_qos_profile();
        for (int index = 0; index < 4; ++index) {
            tof_subs_[index].subscribe(this, tof_topics_[index], qos);
        }
        sync_ = std::make_unique<message_filters::Synchronizer<TofPolicy>>(
            TofPolicy(10), tof_subs_[0], tof_subs_[1], tof_subs_[2], tof_subs_[3]);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
        sync_->registerCallback(
            std::bind(&TioOdometryNode::on_tof, this, std::placeholders::_1,
                std::placeholders::_2, std::placeholders::_3,
                std::placeholders::_4));

        worker_ = std::thread(&TioOdometryNode::worker_loop, this);
        status_timer_ = create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(status_period_ * 1000.0)),
            std::bind(&TioOdometryNode::publish_status, this));

        RCLCPP_INFO(get_logger(),
            "TIO-Former ready: four ToF views, %s -> %s, publishing /flyrat/odometry/*",
            world_frame_.c_str(), body_frame_.c_str());
    }

    ~TioOdometryNode() override
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
            odometry_->shutdown();
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
        std::array<TofGrid, 4> views;
        rclcpp::Time stamp;
    };

    void on_imu(const sensor_msgs::msg::Imu::ConstSharedPtr& message)
    {
        odometry::ImuSample sample;
        sample.timestamp = rclcpp::Time(message->header.stamp).seconds();
        sample.accel = Eigen::Vector3d(message->linear_acceleration.x,
            message->linear_acceleration.y, message->linear_acceleration.z);
        sample.gyro = Eigen::Vector3d(message->angular_velocity.x,
            message->angular_velocity.y, message->angular_velocity.z);
        if (!sample.accel.allFinite() || !sample.gyro.allFinite()) {
            return;
        }
        odometry_->addImu(sample);
        imu_count_.fetch_add(1);
        last_imu_stamp_.store(sample.timestamp);
    }

    void on_tof(const Tof::ConstSharedPtr& first, const Tof::ConstSharedPtr& second,
        const Tof::ConstSharedPtr& third, const Tof::ConstSharedPtr& fourth)
    {
        const std::array<Tof::ConstSharedPtr, 4> messages {
            first, second, third, fourth
        };
        const double stamp = rclcpp::Time(first->header.stamp).seconds();
        if (!std::isfinite(stamp)) {
            dropped_frames_.fetch_add(1);
            return;
        }
        const double previous_stamp = last_tof_stamp_.load();
        if (previous_stamp >= 0.0) {
            const double minimum_period = 0.8 / tof_rate_hz_;
            const double delta = stamp - previous_stamp;
            if (delta <= 0.0 || delta < minimum_period) {
                dropped_frames_.fetch_add(1);
                return;
            }
        }
        PendingFrame frame;
        for (int index = 0; index < 4; ++index) {
            if (!to_grid(*messages[sensor_order_[index]], frame.views[index])) {
                dropped_frames_.fetch_add(1);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "dropping malformed ToF frame (resolution/zone count)");
                return;
            }
        }
        last_tof_stamp_.store(stamp);
        frame.stamp = rclcpp::Time(first->header.stamp);
        input_frames_.fetch_add(1);
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
            odometry::Result result;
            try {
                std::lock_guard<std::mutex> lock(odometry_mutex_);
                result = odometry_->track(std::vector<TofGrid>(
                                              frame.views.begin(), frame.views.end()),
                    frame.stamp.seconds());
            } catch (const std::exception& error) {
                failed_frames_.fetch_add(1);
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                    "TIO tracking failed: %s", error.what());
                continue;
            }

            state_.store(result.state);
            processed_frames_.fetch_add(1);
            if (result.pose.valid) {
                publish_pose(result, frame.stamp);
                publish_path_if_due();
            }
        }
    }

    void publish_pose(const odometry::Result& result, const rclcpp::Time& stamp)
    {
        const Eigen::Vector3d& position = result.pose.position;
        const Eigen::Quaterniond orientation = result.pose.orientation.normalized();
        if (!position.allFinite() ||
            !Eigen::Vector4d(orientation.x(), orientation.y(), orientation.z(),
                orientation.w())
                 .allFinite()) {
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
        if (pose_frames_.fetch_add(1) == 0) {
            RCLCPP_INFO(get_logger(),
                "first odometry pose: position=(%.3f, %.3f, %.3f)",
                position.x(), position.y(), position.z());
        }

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

        std::lock_guard<std::mutex> lock(state_mutex_);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = odometry.header;
        pose.pose = odometry.pose.pose;
        path_.poses.push_back(std::move(pose));
        while (static_cast<int>(path_.poses.size()) > path_max_poses_) {
            path_.poses.erase(path_.poses.begin());
        }
        path_.header.stamp = stamp;
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

    void publish_status()
    {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "flyrat_tio_odometry";
        status.hardware_id = "tio_former";
        const odometry::State state = state_.load();
        status.level = diagnostic_level(state);
        status.message = state_name(state);
        if (state != odometry::State::kTracking) {
            status.message += "; no valid pose";
        }
        const double now = get_clock()->now().seconds();
        const double last_imu = last_imu_stamp_.load();
        const double last_tof = last_tof_stamp_.load();
        const double imu_age = last_imu > 0.0 ? now - last_imu : -1.0;
        const double tof_age = last_tof >= 0.0 ? now - last_tof : -1.0;
        status.values.push_back(value("state", state_name(state)));
        status.values.push_back(value("input_frames", std::to_string(input_frames_.load())));
        status.values.push_back(value("processed_frames", std::to_string(processed_frames_.load())));
        status.values.push_back(value("pose_frames", std::to_string(pose_frames_.load())));
        status.values.push_back(value("dropped_frames", std::to_string(dropped_frames_.load())));
        status.values.push_back(value("failed_frames", std::to_string(failed_frames_.load())));
        status.values.push_back(value("imu_samples", std::to_string(imu_count_.load())));
        status.values.push_back(value("imu_age_s", std::to_string(imu_age)));
        status.values.push_back(value("tof_age_s", std::to_string(tof_age)));
        status.values.push_back(value("tof_rate_hz", std::to_string(tof_rate_hz_)));
        status.values.push_back(value("model", model_path_));
        status_pub_->publish(status);
    }

    void on_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        {
            std::lock_guard<std::mutex> lock(odometry_mutex_);
            odometry_->reset();
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            path_.poses.clear();
            last_path_publish_ = -1.0;
        }
        state_.store(odometry::State::kNotInitialized);
        last_tof_stamp_.store(-1.0);
        response->success = true;
        response->message = "TIO odometry reset";
    }

    std::string model_path_;
    std::string world_frame_;
    std::string body_frame_;
    std::string trajectory_path_;
    double tof_rate_hz_ = 15.0;
    double imu_buffer_seconds_ = 2.0;
    double path_publish_period_ = 0.2;
    double status_period_ = 1.0;
    int path_max_poses_ = 1000;
    std::array<std::string, 4> tof_topics_;
    std::array<int, 4> sensor_order_ { 1, 3, 2, 0 };
    std::atomic<double> last_tof_stamp_ { -1.0 };

    std::unique_ptr<TioOdometry> odometry_;
    std::mutex odometry_mutex_;
    std::mutex queue_mutex_;
    std::mutex state_mutex_;
    std::condition_variable queue_cv_;
    std::optional<PendingFrame> pending_;
    bool stop_ = false;
    std::thread worker_;

    nav_msgs::msg::Path path_;
    double last_path_publish_ = -1.0;
    std::atomic<odometry::State> state_ { odometry::State::kNotInitialized };
    std::atomic<std::uint64_t> input_frames_ { 0 };
    std::atomic<std::uint64_t> processed_frames_ { 0 };
    std::atomic<std::uint64_t> pose_frames_ { 0 };
    std::atomic<std::uint64_t> dropped_frames_ { 0 };
    std::atomic<std::uint64_t> failed_frames_ { 0 };
    std::atomic<std::uint64_t> imu_count_ { 0 };
    std::atomic<double> last_imu_stamp_ { 0.0 };

    message_filters::Subscriber<Tof> tof_subs_[4];
    std::unique_ptr<message_filters::Synchronizer<TofPolicy>> sync_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

} // namespace flyrat::tio

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<flyrat::tio::TioOdometryNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("tio_odometry_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

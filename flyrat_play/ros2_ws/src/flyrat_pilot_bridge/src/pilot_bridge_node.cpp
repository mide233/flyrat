// SPDX-License-Identifier: MIT
//
// Host-side bridge for the on-vehicle SLAM runtime.
//
// flyrat_pilot runs SchurVINS on the vehicle and streams body poses over TCP
// (usually through `adb forward tcp:5001 tcp:5001`). This node turns that
// stream into the same ROS 2 interface that the host-side odometry node uses,
// so Foxglove layouts keep working when SLAM moves onto the vehicle.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "flyrat/sensors/protocol.hpp"

namespace {

using namespace std::chrono_literals;

struct Calibration {
    cv::Matx44d T_body_camera = cv::Matx44d::eye();
    cv::Matx44d T_left_right = cv::Matx44d::eye();
};

cv::Matx44d readMatrix(const cv::FileNode& node, const char* name)
{
    cv::Mat matrix;
    node >> matrix;
    if (matrix.rows != 4 || matrix.cols != 4) {
        throw std::runtime_error(std::string(name) + " must be a 4x4 matrix");
    }
    cv::Matx44d result;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            result(row, col) = matrix.at<double>(row, col);
        }
    }
    return result;
}

Calibration loadCalibration(const std::string& path)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("cannot open camera calibration: " + path);
    }
    Calibration calibration;
    calibration.T_body_camera = readMatrix(
        fs["imu"]["T_body_camera"], "imu.T_body_camera");
    calibration.T_left_right = readMatrix(
        fs["camera"]["T_left_right"], "camera.T_left_right");
    return calibration;
}

geometry_msgs::msg::Transform matrixToTransform(const cv::Matx44d& matrix)
{
    tf2::Matrix3x3 rotation(
        matrix(0, 0), matrix(0, 1), matrix(0, 2),
        matrix(1, 0), matrix(1, 1), matrix(1, 2),
        matrix(2, 0), matrix(2, 1), matrix(2, 2));
    tf2::Quaternion quaternion;
    rotation.getRotation(quaternion);
    quaternion.normalize();

    geometry_msgs::msg::Transform transform;
    transform.translation.x = matrix(0, 3);
    transform.translation.y = matrix(1, 3);
    transform.translation.z = matrix(2, 3);
    transform.rotation.x = quaternion.x();
    transform.rotation.y = quaternion.y();
    transform.rotation.z = quaternion.z();
    transform.rotation.w = quaternion.w();
    return transform;
}

std::string stateName(std::uint8_t state)
{
    switch (state) {
    case 0:
        return "not_initialized";
    case 1:
        return "tracking";
    case 2:
        return "recently_lost";
    case 3:
        return "lost";
    default:
        return "unknown";
    }
}

bool recvAll(int fd, std::uint8_t* data, std::size_t size, std::atomic<bool>& stop)
{
    std::size_t offset = 0;
    while (offset < size && !stop) {
        pollfd pfd {};
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int ready = ::poll(&pfd, 1, 500);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (ready == 0) {
            continue;
        }
        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            return false;
        }
        const ssize_t received = ::recv(fd, data + offset, size - offset, 0);
        if (received < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if (received == 0) {
            return false;
        }
        offset += static_cast<std::size_t>(received);
    }
    return offset == size;
}

} // namespace

namespace flyrat::pilot_bridge {

class PilotBridgeNode : public rclcpp::Node {
public:
    PilotBridgeNode()
        : Node("pilot_bridge_node")
    {
        host_ = declare_parameter<std::string>("host", "127.0.0.1");
        port_ = declare_parameter<int>("port", 5001);
        reconnect_ms_ = declare_parameter<int>("reconnect_ms", 1000);
        world_frame_ = declare_parameter<std::string>("world_frame", "odometry_map");
        body_frame_ = declare_parameter<std::string>("body_frame", "imu_link");
        camera_frame_ = declare_parameter<std::string>(
            "camera_frame", "camera_left_optical_frame");
        right_camera_frame_ = declare_parameter<std::string>(
            "right_camera_frame", "camera_right_optical_frame");
        const std::string camera_config_path = declare_parameter<std::string>(
            "camera_config_path", "");
        path_max_poses_ = declare_parameter<int>("path_max_poses", 2000);
        publish_tf_ = declare_parameter<bool>("publish_tf", true);
        status_period_s_ = declare_parameter<double>("status_period", 1.0);
        if (port_ <= 0 || port_ > 65535 || reconnect_ms_ <= 0
            || path_max_poses_ <= 0) {
            throw std::runtime_error("invalid bridge parameters");
        }

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
            "/flyrat/odometry/odometry", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>(
            "/flyrat/odometry/path", rclcpp::QoS(1).transient_local());
        status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>(
            "/flyrat/odometry/status", 10);

        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            static_broadcaster_
                = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
            if (!camera_config_path.empty()) {
                const Calibration calibration = loadCalibration(camera_config_path);
                publishStaticTransforms(calibration);
            } else {
                RCLCPP_WARN(get_logger(),
                    "camera_config_path is empty; camera static TF is not published");
            }
        }

        reader_thread_ = std::thread([this] { readerLoop(); });
        timer_ = create_wall_timer(20ms, [this] { onTimer(); });
        status_timer_ = create_wall_timer(
            std::chrono::duration<double>(status_period_s_), [this] { publishStatus(); });
    }

    ~PilotBridgeNode() override
    {
        stop_ = true;
        if (reader_thread_.joinable()) {
            reader_thread_.join();
        }
    }

private:
    struct PosePacket {
        sensors::PosePayload pose;
        std::int64_t stamp_ns = 0;
        std::uint64_t sequence = 0;
    };

    void publishStaticTransforms(const Calibration& calibration)
    {
        const auto stamp = now();
        std::vector<geometry_msgs::msg::TransformStamped> transforms;

        geometry_msgs::msg::TransformStamped camera;
        camera.header.stamp = stamp;
        camera.header.frame_id = body_frame_;
        camera.child_frame_id = camera_frame_;
        camera.transform = matrixToTransform(calibration.T_body_camera);
        transforms.push_back(camera);

        geometry_msgs::msg::TransformStamped right;
        right.header.stamp = stamp;
        right.header.frame_id = camera_frame_;
        right.child_frame_id = right_camera_frame_;
        right.transform = matrixToTransform(calibration.T_left_right);
        transforms.push_back(right);

        static_broadcaster_->sendTransform(transforms);
    }

    void readerLoop()
    {
        while (!stop_) {
            const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
            if (fd < 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
                continue;
            }
            int one = 1;
            (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

            sockaddr_in address {};
            address.sin_family = AF_INET;
            address.sin_port = htons(static_cast<std::uint16_t>(port_));
            if (::inet_pton(AF_INET, host_.c_str(), &address.sin_addr) != 1
                || ::connect(fd, reinterpret_cast<sockaddr*>(&address),
                       sizeof(address)) < 0) {
                ::close(fd);
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "waiting for flyrat_pilot pose stream on %s:%d",
                    host_.c_str(), port_);
                std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
                continue;
            }
            connected_ = true;
            RCLCPP_INFO(get_logger(), "connected to flyrat_pilot pose stream on %s:%d",
                host_.c_str(), port_);

            bool healthy = true;
            while (!stop_ && healthy) {
                std::uint8_t header_bytes[sensors::kHeaderSize];
                if (!recvAll(fd, header_bytes, sizeof(header_bytes), stop_)) {
                    break;
                }
                sensors::Header header;
                if (!sensors::decode_header(header_bytes, sizeof(header_bytes), header)) {
                    RCLCPP_WARN(get_logger(), "invalid pose stream header");
                    break;
                }
                std::vector<std::uint8_t> payload(header.payload_size);
                if (!payload.empty()
                    && !recvAll(fd, payload.data(), payload.size(), stop_)) {
                    break;
                }
                if (header.type != sensors::MessageType::kPose) {
                    continue;
                }
                PosePacket packet;
                if (!sensors::decode_pose(payload.data(), payload.size(), packet.pose)) {
                    RCLCPP_WARN(get_logger(), "malformed pose payload");
                    continue;
                }
                packet.stamp_ns = header.device_time_ns;
                packet.sequence = header.sequence;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    latest_ = packet;
                    ++received_;
                }
            }

            connected_ = false;
            ::close(fd);
            if (!stop_) {
                RCLCPP_WARN(get_logger(), "pose stream disconnected; reconnecting");
                std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
            }
        }
    }

    void onTimer()
    {
        PosePacket packet;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!latest_ || latest_->sequence == last_sequence_) {
                return;
            }
            packet = *latest_;
            last_sequence_ = packet.sequence;
        }
        last_pose_time_ = std::chrono::steady_clock::now();
        last_state_ = packet.pose.state;

        if (!packet.pose.valid) {
            return;
        }

        const rclcpp::Time stamp = packet.stamp_ns > 0
            ? rclcpp::Time(packet.stamp_ns)
            : now();

        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = stamp;
        odometry.header.frame_id = world_frame_;
        odometry.child_frame_id = body_frame_;
        odometry.pose.pose.position.x = packet.pose.position[0];
        odometry.pose.pose.position.y = packet.pose.position[1];
        odometry.pose.pose.position.z = packet.pose.position[2];
        odometry.pose.pose.orientation.x = packet.pose.orientation[0];
        odometry.pose.pose.orientation.y = packet.pose.orientation[1];
        odometry.pose.pose.orientation.z = packet.pose.orientation[2];
        odometry.pose.pose.orientation.w = packet.pose.orientation[3];
        odom_pub_->publish(odometry);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = odometry.header;
        pose.pose = odometry.pose.pose;
        path_.poses.push_back(pose);
        while (static_cast<int>(path_.poses.size()) > path_max_poses_) {
            path_.poses.erase(path_.poses.begin());
        }
        path_.header = odometry.header;
        path_pub_->publish(path_);

        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped transform;
            transform.header = odometry.header;
            transform.child_frame_id = body_frame_;
            transform.transform.translation.x = odometry.pose.pose.position.x;
            transform.transform.translation.y = odometry.pose.pose.position.y;
            transform.transform.translation.z = odometry.pose.pose.position.z;
            transform.transform.rotation = odometry.pose.pose.orientation;
            tf_broadcaster_->sendTransform(transform);
        }

        ++published_;
    }

    void publishStatus()
    {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "flyrat_pilot/slam";
        status.hardware_id = "taishanpi";
        if (!connected_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "pose stream disconnected";
        } else if (last_state_ != 1) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "tracking state: " + stateName(last_state_);
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "tracking";
        }

        diagnostic_msgs::msg::KeyValue state;
        state.key = "state";
        state.value = stateName(last_state_);
        status.values.push_back(state);

        diagnostic_msgs::msg::KeyValue connected;
        connected.key = "connected";
        connected.value = connected_ ? "true" : "false";
        status.values.push_back(connected);

        diagnostic_msgs::msg::KeyValue received;
        received.key = "received_poses";
        received.value = std::to_string(received_);
        status.values.push_back(received);

        diagnostic_msgs::msg::KeyValue published;
        published.key = "published_poses";
        published.value = std::to_string(published_);
        status.values.push_back(published);

        diagnostic_msgs::msg::KeyValue age;
        age.key = "last_pose_age_s";
        if (last_pose_time_.time_since_epoch().count() == 0) {
            age.value = "never";
        } else {
            const double seconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_pose_time_)
                                       .count();
            age.value = std::to_string(seconds);
        }
        status.values.push_back(age);

        status_pub_->publish(status);
    }

    std::string host_;
    int port_ = 5001;
    int reconnect_ms_ = 1000;
    std::string world_frame_;
    std::string body_frame_;
    std::string camera_frame_;
    std::string right_camera_frame_;
    int path_max_poses_ = 2000;
    bool publish_tf_ = true;
    double status_period_s_ = 1.0;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr status_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    std::thread reader_thread_;
    std::atomic<bool> stop_ { false };
    std::atomic<bool> connected_ { false };
    std::atomic<std::uint64_t> received_ { 0 };
    std::atomic<std::uint64_t> published_ { 0 };

    std::mutex mutex_;
    std::optional<PosePacket> latest_;
    std::uint64_t last_sequence_ = 0;
    std::uint8_t last_state_ = 0;
    std::chrono::steady_clock::time_point last_pose_time_ {};

    nav_msgs::msg::Path path_;
};

} // namespace flyrat::pilot_bridge

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<flyrat::pilot_bridge::PilotBridgeNode>();
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("pilot_bridge_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

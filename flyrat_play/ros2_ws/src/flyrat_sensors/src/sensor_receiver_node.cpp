// SPDX-License-Identifier: MIT
//
// Receives the flyrat sensor stream from the real vehicle over an adb TCP
// tunnel and publishes ROS 2 topics. The wire format is defined in
// flyrat/sensors/protocol.hpp and is shared with flyrat_sensor_forwarder.
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "flyrat/sensors/protocol.hpp"
#include "flyrat_sensors_interfaces/msg/tof_frame.hpp"

namespace flyrat_sensors {

namespace {

constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;

std::int64_t system_now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

bool read_exact(int fd, std::uint8_t* data, std::size_t size, const std::atomic<bool>& stop)
{
    std::size_t offset = 0;
    while (offset < size && !stop) {
        const ssize_t n = ::recv(fd, data + offset, size - offset, 0);
        if (n > 0) {
            offset += static_cast<std::size_t>(n);
            continue;
        }
        if (n == 0) {
            return false;
        }
        if (errno == EINTR) {
            continue;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            continue;
        }
        return false;
    }
    return offset == size;
}

void fill_image(sensor_msgs::msg::Image& msg, const cv::Mat& image,
    const std::string& encoding, const rclcpp::Time& stamp,
    const std::string& frame_id)
{
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = static_cast<std::uint32_t>(image.rows);
    msg.width = static_cast<std::uint32_t>(image.cols);
    msg.encoding = encoding;
    msg.is_bigendian = 0;
    msg.step = static_cast<std::uint32_t>(image.cols * image.elemSize());
    msg.data.resize(static_cast<std::size_t>(msg.step) * msg.height);
    for (int y = 0; y < image.rows; ++y) {
        std::memcpy(msg.data.data() + static_cast<std::size_t>(y) * msg.step,
            image.ptr(y), msg.step);
    }
}

} // namespace

class SensorReceiverNode : public rclcpp::Node {
public:
    SensorReceiverNode()
        : rclcpp::Node("sensor_receiver")
    {
        host_ = declare_parameter<std::string>("host", "127.0.0.1");
        port_ = declare_parameter<int>("port", 5000);
        reconnect_ms_ = declare_parameter<int>("reconnect_ms", 1000);
        imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
        tof_frame_prefix_ = declare_parameter<std::string>("tof_frame_id", "tof_");
        left_frame_id_ = declare_parameter<std::string>(
            "left_frame_id", "camera_left_optical_frame");
        right_frame_id_ = declare_parameter<std::string>(
            "right_frame_id", "camera_right_optical_frame");
        acc_lsb_per_g_ = declare_parameter<double>("acc_lsb_per_g", 2048.0);
        gyro_lsb_per_dps_ = declare_parameter<double>("gyro_lsb_per_dps", 16.4);
        stereo_color_ = declare_parameter<bool>("stereo_color", false);
        const int tof_count = declare_parameter<int>("tof_count", 4);

        if (acc_lsb_per_g_ <= 0.0 || gyro_lsb_per_dps_ <= 0.0) {
            throw std::runtime_error("acc_lsb_per_g and gyro_lsb_per_dps must be positive");
        }
        if (port_ <= 0 || port_ > 65535) {
            throw std::runtime_error("port must be in 1..65535");
        }

        const auto qos = rclcpp::SensorDataQoS().keep_last(2);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
            "/flyrat/sensors/imu", qos);
        left_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/flyrat/sensors/stereo/left/image_raw", qos);
        right_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/flyrat/sensors/stereo/right/image_raw", qos);
        for (int i = 0; i < std::max(tof_count, 0); ++i) {
            tof_pubs_.push_back(create_publisher<flyrat_sensors_interfaces::msg::TofFrame>(
                "/flyrat/sensors/tof_" + std::to_string(i), qos));
        }
    }

    ~SensorReceiverNode() override
    {
        stop();
    }

    void start()
    {
        if (thread_.joinable()) {
            return;
        }
        stop_ = false;
        thread_ = std::thread([this] { run(); });
    }

    void stop()
    {
        stop_ = true;
        const int fd = fd_.exchange(-1);
        if (fd >= 0) {
            ::shutdown(fd, SHUT_RDWR);
        }
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    void run()
    {
        while (rclcpp::ok() && !stop_) {
            const int fd = connect_to_server();
            if (fd < 0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "waiting for sensor forwarder at %s:%d", host_.c_str(), port_);
                std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
                continue;
            }

            fd_ = fd;
            have_offset_ = false;
            RCLCPP_INFO(get_logger(), "connected to sensor forwarder at %s:%d",
                host_.c_str(), port_);

            while (rclcpp::ok() && !stop_ && read_one(fd)) {
            }

            fd_ = -1;
            ::close(fd);
            if (!stop_) {
                RCLCPP_WARN(get_logger(), "sensor link closed, reconnecting");
                std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
            }
        }
    }

    int connect_to_server() const
    {
        addrinfo hints {};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        addrinfo* result = nullptr;
        const std::string port = std::to_string(port_);
        if (::getaddrinfo(host_.c_str(), port.c_str(), &hints, &result) != 0) {
            return -1;
        }

        int fd = -1;
        for (addrinfo* entry = result; entry != nullptr; entry = entry->ai_next) {
            fd = ::socket(entry->ai_family, entry->ai_socktype, entry->ai_protocol);
            if (fd < 0) {
                continue;
            }
            timeval timeout {};
            timeout.tv_sec = 0;
            timeout.tv_usec = 500000;
            (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
            const int one = 1;
            (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
            if (::connect(fd, entry->ai_addr, entry->ai_addrlen) == 0) {
                break;
            }
            ::close(fd);
            fd = -1;
        }
        ::freeaddrinfo(result);
        return fd;
    }

    bool read_one(int fd)
    {
        std::uint8_t header_bytes[flyrat::sensors::kHeaderSize] {};
        if (!read_exact(fd, header_bytes, sizeof(header_bytes), stop_)) {
            return false;
        }

        flyrat::sensors::Header header;
        if (!flyrat::sensors::decode_header(header_bytes, sizeof(header_bytes), header)) {
            RCLCPP_WARN(get_logger(), "invalid sensor frame header");
            return false;
        }

        std::vector<std::uint8_t> payload(header.payload_size);
        if (!payload.empty() &&
            !read_exact(fd, payload.data(), payload.size(), stop_)) {
            return false;
        }

        dispatch(header, payload);
        return true;
    }

    rclcpp::Time stamp_for(std::int64_t device_ns)
    {
        const std::int64_t host_ns = system_now_ns();
        if (device_ns <= 0) {
            return now();
        }

        const std::int64_t sample = host_ns - device_ns;
        if (!have_offset_) {
            offset_ns_ = sample;
            have_offset_ = true;
        } else {
            // Light smoothing; a slow drift is fine for algorithm testing.
            offset_ns_ = static_cast<std::int64_t>(
                static_cast<double>(offset_ns_) * 0.99 +
                static_cast<double>(sample) * 0.01);
        }
        return rclcpp::Time(device_ns + offset_ns_);
    }

    void dispatch(const flyrat::sensors::Header& header,
        const std::vector<std::uint8_t>& payload)
    {
        switch (header.type) {
        case flyrat::sensors::MessageType::kTime:
            // Sampling the offset on TIME frames keeps the mapping fresh even
            // when no IMU is connected.
            (void)stamp_for(header.device_time_ns);
            break;
        case flyrat::sensors::MessageType::kImu:
            publish_imu(header, payload);
            break;
        case flyrat::sensors::MessageType::kTof:
            publish_tof(header, payload);
            break;
        case flyrat::sensors::MessageType::kStereoJpeg:
            publish_stereo(header, payload);
            break;
        case flyrat::sensors::MessageType::kStats:
            break;
        }
    }

    void publish_imu(const flyrat::sensors::Header& header,
        const std::vector<std::uint8_t>& payload)
    {
        flyrat::sensors::ImuPayload raw;
        if (!flyrat::sensors::decode_imu(payload.data(), payload.size(), raw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "malformed IMU payload");
            return;
        }

        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp_for(header.device_time_ns);
        msg.header.frame_id = imu_frame_id_;
        msg.linear_acceleration.x =
            static_cast<double>(raw.acc[0]) / acc_lsb_per_g_ * kGravity;
        msg.linear_acceleration.y =
            static_cast<double>(raw.acc[1]) / acc_lsb_per_g_ * kGravity;
        msg.linear_acceleration.z =
            static_cast<double>(raw.acc[2]) / acc_lsb_per_g_ * kGravity;
        msg.angular_velocity.x =
            static_cast<double>(raw.gyro[0]) / gyro_lsb_per_dps_ * kPi / 180.0;
        msg.angular_velocity.y =
            static_cast<double>(raw.gyro[1]) / gyro_lsb_per_dps_ * kPi / 180.0;
        msg.angular_velocity.z =
            static_cast<double>(raw.gyro[2]) / gyro_lsb_per_dps_ * kPi / 180.0;
        // No orientation estimate is available from MSP_RAW_IMU.
        msg.orientation_covariance[0] = -1.0;
        imu_pub_->publish(msg);
        ++imu_count_;
    }

    void publish_tof(const flyrat::sensors::Header& header,
        const std::vector<std::uint8_t>& payload)
    {
        flyrat::sensors::TofPayload raw;
        if (!flyrat::sensors::decode_tof(payload.data(), payload.size(), raw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "malformed ToF payload");
            return;
        }
        if (raw.sensor_id >= tof_pubs_.size()) {
            return;
        }

        flyrat_sensors_interfaces::msg::TofFrame msg;
        msg.header.stamp = stamp_for(header.device_time_ns);
        msg.header.frame_id = tof_frame_prefix_ + std::to_string(raw.sensor_id);
        msg.sensor_id = raw.sensor_id;
        msg.resolution = raw.resolution;
        msg.temperature_c = raw.temperature_c;
        for (std::size_t i = 0; i < raw.zones.size(); ++i) {
            msg.zones[i].distance_mm = raw.zones[i].distance_mm;
            msg.zones[i].target_status = raw.zones[i].target_status;
            msg.zones[i].nb_target = raw.zones[i].nb_target;
            msg.zones[i].valid = raw.zones[i].target_status == 5 ||
                raw.zones[i].target_status == 9;
        }
        tof_pubs_[raw.sensor_id]->publish(msg);
        ++tof_count_;
    }

    void publish_stereo(const flyrat::sensors::Header& header,
        const std::vector<std::uint8_t>& payload)
    {
        flyrat::sensors::StereoJpegView view;
        if (!flyrat::sensors::decode_stereo_jpeg(payload.data(), payload.size(), view)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "malformed stereo JPEG payload");
            return;
        }
        if ((view.width % 2U) != 0U) {
            return;
        }

        cv::Mat encoded(1, static_cast<int>(view.size), CV_8UC1,
            const_cast<std::uint8_t*>(view.jpeg));
        cv::Mat decoded = cv::imdecode(
            encoded, stereo_color_ ? cv::IMREAD_COLOR : cv::IMREAD_GRAYSCALE);
        if (decoded.empty() || decoded.cols != static_cast<int>(view.width) ||
            decoded.rows != static_cast<int>(view.height)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "JPEG decode failed or returned an unexpected size");
            return;
        }

        const int half = decoded.cols / 2;
        const cv::Mat left = decoded.colRange(0, half);
        const cv::Mat right = decoded.colRange(half, decoded.cols);
        const rclcpp::Time stamp = stamp_for(header.device_time_ns);
        const std::string encoding = stereo_color_ ? "bgr8" : "mono8";

        sensor_msgs::msg::Image left_msg;
        sensor_msgs::msg::Image right_msg;
        fill_image(left_msg, left, encoding, stamp, left_frame_id_);
        fill_image(right_msg, right, encoding, stamp, right_frame_id_);
        left_pub_->publish(left_msg);
        right_pub_->publish(right_msg);
        ++stereo_count_;
    }

    std::string host_;
    int port_ = 5000;
    int reconnect_ms_ = 1000;
    std::string imu_frame_id_;
    std::string tof_frame_prefix_;
    std::string left_frame_id_;
    std::string right_frame_id_;
    double acc_lsb_per_g_ = 2048.0;
    double gyro_lsb_per_dps_ = 16.4;
    bool stereo_color_ = false;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::vector<rclcpp::Publisher<flyrat_sensors_interfaces::msg::TofFrame>::SharedPtr>
        tof_pubs_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;

    std::thread thread_;
    std::atomic<bool> stop_ { false };
    std::atomic<int> fd_ { -1 };
    bool have_offset_ = false;
    std::int64_t offset_ns_ = 0;
    std::uint64_t imu_count_ = 0;
    std::uint64_t tof_count_ = 0;
    std::uint64_t stereo_count_ = 0;
};

} // namespace flyrat_sensors

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<flyrat_sensors::SensorReceiverNode>();
        node->start();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("sensor_receiver"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

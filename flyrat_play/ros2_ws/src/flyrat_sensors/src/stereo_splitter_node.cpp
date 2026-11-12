// SPDX-License-Identifier: MIT
//
// Splits the side-by-side stereo JPEG produced by sensor_receiver_node into
// the two raw image topics used by odometry and Foxglove. Both images of a
// pair are published from one input message with the same header stamp, so a
// bag recorded from these topics cannot contain orphan left or right frames.
//
// Run this only while replaying a bag or working offline: sensor_receiver_node
// already publishes the same two topics on the live vehicle path.
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace flyrat_sensors {

namespace {

void fill_image(sensor_msgs::msg::Image& msg, const cv::Mat& image,
    const std::string& encoding, const builtin_interfaces::msg::Time& stamp,
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

class StereoSplitterNode : public rclcpp::Node {
public:
    StereoSplitterNode()
        : rclcpp::Node("stereo_splitter")
    {
        input_topic_ = declare_parameter<std::string>(
            "input_topic", "/flyrat/sensors/stereo/compressed");
        left_topic_ = declare_parameter<std::string>(
            "left_topic", "/flyrat/sensors/stereo/left/image_raw");
        right_topic_ = declare_parameter<std::string>(
            "right_topic", "/flyrat/sensors/stereo/right/image_raw");
        left_frame_id_ = declare_parameter<std::string>(
            "left_frame_id", "camera_left_optical_frame");
        right_frame_id_ = declare_parameter<std::string>(
            "right_frame_id", "camera_right_optical_frame");
        color_ = declare_parameter<bool>("color", false);
        max_rate_ = declare_parameter<double>("max_rate", 0.0);
        const int qos_depth = declare_parameter<int>("qos_depth", 5);

        if (max_rate_ < 0.0) {
            throw std::runtime_error("max_rate must be >= 0");
        }
        if (qos_depth <= 0) {
            throw std::runtime_error("qos_depth must be positive");
        }

        const auto input_qos = rclcpp::SensorDataQoS().keep_last(qos_depth);
        // Reliable output keeps the two eyes together for offline consumers
        // such as rosbag2; best-effort subscribers (odometry) remain
        // compatible with a reliable publisher.
        rclcpp::QoS output_qos(static_cast<std::size_t>(qos_depth));
        output_qos.reliable();
        left_pub_ = create_publisher<sensor_msgs::msg::Image>(left_topic_, output_qos);
        right_pub_ = create_publisher<sensor_msgs::msg::Image>(right_topic_, output_qos);
        subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
            input_topic_, input_qos,
            [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
                on_image(std::move(msg));
            });

        RCLCPP_INFO(get_logger(), "splitting %s -> %s + %s (max_rate %.1f Hz)",
            input_topic_.c_str(), left_topic_.c_str(), right_topic_.c_str(),
            max_rate_);
    }

private:
    void on_image(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& msg)
    {
        if (msg->data.empty()) {
            return;
        }

        if (max_rate_ > 0.0) {
            const auto now = std::chrono::steady_clock::now();
            if (now < next_allowed_) {
                return;
            }
            next_allowed_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / max_rate_));
        }

        cv::Mat encoded(1, static_cast<int>(msg->data.size()), CV_8UC1,
            const_cast<std::uint8_t*>(msg->data.data()));
        cv::Mat decoded = cv::imdecode(
            encoded, color_ ? cv::IMREAD_COLOR : cv::IMREAD_GRAYSCALE);
        if (decoded.empty() || (decoded.cols % 2) != 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "JPEG decode failed or returned an odd width");
            return;
        }

        const int half = decoded.cols / 2;
        const cv::Mat left = decoded.colRange(0, half);
        const cv::Mat right = decoded.colRange(half, decoded.cols);
        const std::string encoding = color_ ? "bgr8" : "mono8";

        sensor_msgs::msg::Image left_msg;
        sensor_msgs::msg::Image right_msg;
        fill_image(left_msg, left, encoding, msg->header.stamp, left_frame_id_);
        fill_image(right_msg, right, encoding, msg->header.stamp, right_frame_id_);
        left_pub_->publish(left_msg);
        right_pub_->publish(right_msg);

        if (++count_ % 300 == 0) {
            RCLCPP_INFO(get_logger(), "published %llu stereo pairs",
                static_cast<unsigned long long>(count_));
        }
    }

    std::string input_topic_;
    std::string left_topic_;
    std::string right_topic_;
    std::string left_frame_id_;
    std::string right_frame_id_;
    bool color_ = false;
    double max_rate_ = 0.0;
    std::uint64_t count_ = 0;
    std::chrono::steady_clock::time_point next_allowed_ {};

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

} // namespace flyrat_sensors

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<flyrat_sensors::StereoSplitterNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("stereo_splitter"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

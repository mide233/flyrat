// SPDX-License-Identifier: MIT
//
// flyrat_msp node: owns the two links to the Betaflight SITL flight
// controller and exposes them as ROS 2 topics/services.
//
//   * a virtual RC receiver (UDP 9004) that carries the pilot's switches
//     (AUX1 = ARM, AUX2 = MSP OVERRIDE);
//   * an MSP client (TCP, the SITL UART) used to push MSP_SET_RAW_RC and to
//     read back the flight controller's estimated state.
//
// The autopilot never sees Gazebo ground truth: everything published here comes
// from Betaflight itself.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "flyrat_msp/msp_client.hpp"
#include "flyrat_msp/rc_receiver.hpp"

namespace flyrat_msp {
namespace {

// Raw Betaflight receiver channel order.
constexpr size_t kChRoll = 0;
constexpr size_t kChPitch = 1;
constexpr size_t kChThrottle = 2;
constexpr size_t kChYaw = 3;
constexpr size_t kChAux1 = 4;  // ARM
constexpr size_t kChAux2 = 5;  // MSP OVERRIDE
constexpr size_t kChannelCount = 16;

constexpr uint16_t kRcMin = 1000;
constexpr uint16_t kRcMid = 1500;
constexpr uint16_t kRcMax = 2000;
constexpr uint16_t kSwitchHigh = 1800;

// Raw IMU scaling used by the SITL simulator.
constexpr double kAccOneG = 2048.0;
constexpr double kGyroLsbPerDps = 16.4;
constexpr double kStandardGravity = 9.80665;

constexpr double kPi = 3.14159265358979323846;

double to_radians(double degrees) { return degrees * kPi / 180.0; }

}  // namespace

class MspNode : public rclcpp::Node {
 public:
  MspNode() : rclcpp::Node("flyrat_msp") {
    host_ = declare_parameter<std::string>("host", "127.0.0.1");
    msp_port_ = declare_parameter<int>("msp_port", 5761);
    rc_port_ = declare_parameter<int>("rc_port", 9004);
    rate_ = declare_parameter<double>("rate", 50.0);
    telemetry_rate_ = declare_parameter<double>("telemetry_rate", 10.0);
    rc_timeout_ = declare_parameter<double>("rc_timeout", 0.25);
    override_enabled_ = declare_parameter<bool>("msp_override", true);
    armed_ = declare_parameter<bool>("arm_on_start", false);

    if (rate_ <= 0.0) throw std::runtime_error("rate must be > 0");
    if (telemetry_rate_ <= 0.0) {
      throw std::runtime_error("telemetry_rate must be > 0");
    }

    auto qos = rclcpp::QoS(10);
    attitude_pub_ =
        create_publisher<geometry_msgs::msg::Vector3Stamped>("/flyrat/attitude",
                                                              qos);
    rc_pub_ =
        create_publisher<std_msgs::msg::UInt16MultiArray>("/flyrat/rc", qos);
    motor_pub_ =
        create_publisher<std_msgs::msg::UInt16MultiArray>("/flyrat/motor", qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/flyrat/imu", qos);
    status_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
        "/flyrat/status", qos);
    armed_pub_ = create_publisher<std_msgs::msg::Bool>("/flyrat/armed", qos);
    altitude_pub_ =
        create_publisher<std_msgs::msg::Float32>("/flyrat/altitude", qos);
    battery_pub_ =
        create_publisher<std_msgs::msg::Float32>("/flyrat/battery", qos);

    rc_override_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "/flyrat/rc_override", qos,
        [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (msg->data.size() < kChannelCount) {
            for (size_t i = 0; i < msg->data.size(); ++i) {
              override_[i] = static_cast<uint16_t>(
                  std::clamp(msg->data[i], 0.0,
                             static_cast<double>(kRcMax)));
            }
          }
          override_stamp_ = std::chrono::steady_clock::now();
          has_override_ = true;
        });

    set_arm_srv_ = create_service<std_srvs::srv::SetBool>(
        "/flyrat/set_arm",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr res) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            armed_ = req->data;
          }
          res->success = true;
          res->message = req->data ? "ARM switch high" : "ARM switch low";
          RCLCPP_INFO(get_logger(), "ARM switch -> %s",
                      req->data ? "high" : "low");
        });

    set_override_srv_ = create_service<std_srvs::srv::SetBool>(
        "/flyrat/set_msp_override",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr res) {
          {
            std::lock_guard<std::mutex> lock(mutex_);
            override_enabled_ = req->data;
          }
          res->success = true;
          res->message = req->data ? "MSP OVERRIDE switch high"
                                   : "MSP OVERRIDE switch low";
          RCLCPP_INFO(get_logger(), "MSP OVERRIDE switch -> %s",
                      req->data ? "high" : "low");
        });
  }

  ~MspNode() override {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (sender_thread_.joinable()) sender_thread_.join();
    receiver_.close();
    client_.close();
  }

  void start() {
    override_.assign(kChannelCount, kRcMid);
    override_[kChThrottle] = kRcMin;
    running_ = true;
    connect_links();
    sender_thread_ = std::thread([this] { sender_loop(); });
    thread_ = std::thread([this] { loop(); });
  }

 private:
  std::vector<uint16_t> build_channels() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<uint16_t> channels(kChannelCount, kRcMid);
    channels[kChThrottle] = kRcMin;

    const auto now = std::chrono::steady_clock::now();
    const double age =
        std::chrono::duration<double>(now - override_stamp_).count();
    if (has_override_ && age <= rc_timeout_) {
      for (size_t i = 0; i < 4; ++i) channels[i] = override_[i];
    }
    channels[kChAux1] = armed_ ? kSwitchHigh : kRcMin;
    channels[kChAux2] = override_enabled_ ? kSwitchHigh : kRcMin;
    return channels;
  }

  void connect_links() {
    std::lock_guard<std::mutex> lock(link_mutex_);
    if (!receiver_.is_open()) {
      receiver_.open(host_, rc_port_);
    }
    if (!client_.connected()) {
      client_.connect(host_, msp_port_);
      // The SITL can be slow to answer while the vehicle is flying (heavy
      // physics load), so allow a generous reply window; telemetry runs on its
      // own thread and can afford to block without stalling the RC stream.
      client_.set_timeout(0.5);
      RCLCPP_INFO(get_logger(), "connected to SITL MSP at %s:%d, RC -> %s:%d",
                  host_.c_str(), msp_port_, host_.c_str(), rc_port_);
    }
  }

  // Dedicated RC sender. It runs at the full control rate and never blocks on
  // a telemetry read, so the virtual receiver (UDP 9004) and the MSP override
  // stay fresh even while the CPU-starved SITL is slow to answer telemetry.
  // Concurrent send() from this thread and recv() from loop() are safe: every
  // MSP frame goes out in a single send() call and buf_/flush() are only
  // touched by the reader.
  void sender_loop() {
    const auto period = std::chrono::duration<double>(1.0 / rate_);
    auto next = std::chrono::steady_clock::now();
    while (running_ && rclcpp::ok()) {
      try {
        connect_links();
        const std::vector<uint16_t> channels = build_channels();
        const double ts = std::chrono::duration<double>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count();
        receiver_.send(channels, ts);
        client_.set_raw_rc(channels);
      } catch (const std::exception& exc) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "SITL link error: %s", exc.what());
        {
          std::lock_guard<std::mutex> lock(link_mutex_);
          client_.close();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          period);
      std::this_thread::sleep_until(next);
      if (std::chrono::steady_clock::now() > next + period * 5) {
        next = std::chrono::steady_clock::now();  // fell behind, resync
      }
    }
  }

  // Telemetry reader. It may block for a while when the flight controller is
  // busy (expected in SITL while the vehicle is flying). That must not disturb
  // the RC stream, which lives in sender_loop().
  void loop() {
    const double period = 1.0 / telemetry_rate_;
    int telemetry_ticks = 0;
    while (running_ && rclcpp::ok()) {
      try {
        connect_links();
        publish_telemetry((telemetry_ticks % 25) == 0);
        ++telemetry_ticks;
      } catch (const std::exception& exc) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "telemetry read failed: %s", exc.what());
      }
      std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              std::chrono::duration<double>(period)));
    }
  }

  void publish_telemetry(bool extras) {
    const auto stamp = now();
    const Attitude attitude = client_.attitude();
    geometry_msgs::msg::Vector3Stamped a;
    a.header.stamp = stamp;
    a.header.frame_id = "base_link";
    a.vector.x = to_radians(attitude.roll_deg);
    a.vector.y = to_radians(attitude.pitch_deg);
    a.vector.z = to_radians(attitude.yaw_deg);
    attitude_pub_->publish(a);

    const StatusFrame status = client_.status(true);
    const bool armed = status.armed;
    const bool msp_override = status.msp_override;
    std_msgs::msg::Int32MultiArray status_msg;
    status_msg.data = {armed ? 1 : 0, msp_override ? 1 : 0,
                       static_cast<int32_t>(status.cycletime),
                       static_cast<int32_t>(status.sensors)};
    status_pub_->publish(status_msg);

    std_msgs::msg::Bool armed_msg;
    armed_msg.data = armed;
    armed_pub_->publish(armed_msg);

    if (!extras) return;

    // MSP_RC returns internal order (roll, pitch, yaw, throttle, aux...);
    // republish in raw receiver order for consistency with the wire format.
    const RcFrame rc = client_.rc();
    std_msgs::msg::UInt16MultiArray rc_msg;
    rc_msg.layout.data_offset = 0;
    if (rc.channels.size() >= 4) {
      rc_msg.data = {rc.channels[0], rc.channels[1], rc.channels[3],
                     rc.channels[2]};
      for (size_t i = 4; i < rc.channels.size(); ++i) {
        rc_msg.data.push_back(rc.channels[i]);
      }
    } else {
      rc_msg.data = rc.channels;
    }
    rc_pub_->publish(rc_msg);

    const MotorFrame motors = client_.motor();
    std_msgs::msg::UInt16MultiArray motor_msg;
    motor_msg.data = motors.motors;
    motor_pub_->publish(motor_msg);

    const ImuSample imu = client_.raw_imu();
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = "base_link";
    imu_msg.orientation_covariance[0] = -1.0;  // no orientation estimate
    for (int i = 0; i < 3; ++i) {
      imu_msg.angular_velocity_covariance[i * 4] = -1.0;
      imu_msg.linear_acceleration_covariance[i * 4] = -1.0;
    }
    imu_msg.angular_velocity.x = to_radians(imu.gyro[0] / kGyroLsbPerDps);
    imu_msg.angular_velocity.y = to_radians(imu.gyro[1] / kGyroLsbPerDps);
    imu_msg.angular_velocity.z = to_radians(imu.gyro[2] / kGyroLsbPerDps);
    imu_msg.linear_acceleration.x = imu.acc[0] / kAccOneG * kStandardGravity;
    imu_msg.linear_acceleration.y = imu.acc[1] / kAccOneG * kStandardGravity;
    imu_msg.linear_acceleration.z = imu.acc[2] / kAccOneG * kStandardGravity;
    imu_pub_->publish(imu_msg);

    const AltitudeFrame altitude = client_.altitude();
    std_msgs::msg::Float32 altitude_msg;
    altitude_msg.data =
        altitude.valid ? static_cast<float>(altitude.altitude_m)
                       : std::numeric_limits<float>::quiet_NaN();
    altitude_pub_->publish(altitude_msg);

    const AnalogFrame analog = client_.analog();
    std_msgs::msg::Float32 battery_msg;
    battery_msg.data = static_cast<float>(analog.voltage_v);
    battery_pub_->publish(battery_msg);
  }

  std::string host_;
  int msp_port_ = 5761;
  int rc_port_ = 9004;
  double rate_ = 50.0;
  double telemetry_rate_ = 10.0;
  double rc_timeout_ = 0.25;

  std::mutex mutex_;
  std::mutex link_mutex_;
  bool armed_ = false;
  bool override_enabled_ = true;
  bool has_override_ = false;

  std::vector<uint16_t> override_;
  std::chrono::steady_clock::time_point override_stamp_{};

  std::atomic<bool> running_{false};
  std::thread thread_;
  std::thread sender_thread_;

  MspClient client_;
  RcReceiver receiver_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr rc_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      rc_override_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_arm_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_override_srv_;
};

}  // namespace flyrat_msp

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<flyrat_msp::MspNode>();
    node->start();
    rclcpp::spin(node);
  } catch (const std::exception& exc) {
    RCLCPP_FATAL(rclcpp::get_logger("flyrat_msp"), "fatal: %s", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

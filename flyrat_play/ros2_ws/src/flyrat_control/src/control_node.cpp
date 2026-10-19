// SPDX-License-Identifier: MIT
//
// flyrat_control node: runs the ROS-free autopilot and feeds its commands back
// to Betaflight through flyrat_msp.
//
//   flyrat_msp/attitude  ──►  autopilot (flyrat::Autopilot)  ──►  rc_override
//
// The estimated state is Betaflight's own; no Gazebo ground truth is used.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "flyrat/attitude_hold.hpp"

namespace {

constexpr double kRcMin = 1000.0;
constexpr double kRcMid = 1500.0;
constexpr double kRcMax = 2000.0;
constexpr double kStickRange = 500.0;

double clamp_channel(double value) { return std::max(kRcMin, std::min(kRcMax, value)); }

} // namespace

namespace flyrat_control {

using flyrat::AttitudeHold;
using flyrat::Command;
using flyrat::Setpoint;
using flyrat::State;

class ControlNode : public rclcpp::Node {
public:
    ControlNode()
        : rclcpp::Node("flyrat_control") {
        control_rate_ = declare_parameter<double>("control_rate", 50.0);
        state_timeout_ = declare_parameter<double>("state_timeout", 2.0);
        auto_takeoff_ = declare_parameter<bool>("auto_takeoff", false);
        takeoff_seconds_ = declare_parameter<double>("takeoff_seconds", 2.0);
        takeoff_throttle_ = declare_parameter<double>("takeoff_throttle", 0.7);
        flight_seconds_ = declare_parameter<double>("flight_seconds", 5.0);
        disarm_after_flight_ = declare_parameter<bool>("disarm_after_flight", true);
        target_roll_ = declare_parameter<double>("target_roll", 0.0);
        target_pitch_ = declare_parameter<double>("target_pitch", 0.0);

        rc_override_pub_ =
            create_publisher<std_msgs::msg::Float64MultiArray>("/flyrat/rc_override", 10);
        arm_client_ = create_client<std_srvs::srv::SetBool>("/flyrat/set_arm");

        attitude_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/flyrat/attitude", 10, [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
                state_.roll = msg->vector.x;
                state_.pitch = msg->vector.y;
                state_.yaw = msg->vector.z;
                last_state_ = now();
            });
        status_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            "/flyrat/status", 10, [this](std_msgs::msg::Int32MultiArray::ConstSharedPtr msg) {
                if (!msg->data.empty())
                    state_.armed = msg->data[0] != 0;
            });
        altitude_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/flyrat/altitude", 10, [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
                if (std::isfinite(msg->data))
                    state_.altitude = msg->data;
            });

        const auto period = std::chrono::duration<double>(1.0 / control_rate_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] { tick(); });

        RCLCPP_INFO(
            get_logger(), "flyrat_control up: rate=%.1fHz auto_takeoff=%s", control_rate_,
            auto_takeoff_ ? "true" : "false");
    }

private:
    enum class Phase { kIdle, kArming, kClimb, kHold, kDisarm, kDone };

    void request_arm(bool arm) {
        if (!arm_client_->service_is_ready()) {
            RCLCPP_WARN_ONCE(get_logger(), "waiting for /flyrat/set_arm service");
            return;
        }
        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = arm;
        arm_client_->async_send_request(req);
        RCLCPP_INFO(get_logger(), "requested ARM switch %s", arm ? "high" : "low");
    }

    void tick() {
        const rclcpp::Time stamp = now();
        const double age = last_state_.nanoseconds() == 0 ? 1e9 : (stamp - last_state_).seconds();

        Setpoint setpoint;
        setpoint.roll = target_roll_;
        setpoint.pitch = target_pitch_;

        switch (phase_) {
        case Phase::kIdle:
            if (auto_takeoff_) {
                request_arm(true);
                phase_ = Phase::kArming;
                phase_start_ = stamp;
            }
            break;
        case Phase::kArming:
            setpoint.throttle = 0.0;
            if (state_.armed) {
                phase_ = Phase::kClimb;
                phase_start_ = stamp;
                RCLCPP_INFO(get_logger(), "armed, climbing");
            } else if ((stamp - phase_start_).seconds() > 5.0) {
                request_arm(true); // keep trying
                phase_start_ = stamp;
            }
            break;
        case Phase::kClimb: {
            const double elapsed = (stamp - phase_start_).seconds();
            const double ramp = takeoff_seconds_ > 0.0 ? takeoff_seconds_ : 1.0;
            const double fraction = std::clamp(elapsed / ramp, 0.0, 1.0);
            setpoint.throttle = takeoff_throttle_ * fraction;
            if (fraction >= 1.0) {
                phase_ = Phase::kHold;
                phase_start_ = stamp;
                RCLCPP_INFO(get_logger(), "at takeoff throttle");
            }
            break;
        }
        case Phase::kHold:
            setpoint.throttle = takeoff_throttle_;
            if ((stamp - phase_start_).seconds() > flight_seconds_) {
                phase_ = Phase::kDisarm;
                phase_start_ = stamp;
            }
            break;
        case Phase::kDisarm:
            setpoint.throttle = 0.0;
            if ((stamp - phase_start_).seconds() > 1.0) {
                request_arm(false);
                phase_ = Phase::kDone;
            }
            break;
        case Phase::kDone: setpoint.throttle = 0.0; break;
        }

        // Fail safe: if telemetry stalls, command idle throttle and level attitude
        // (the RC override keeps the vehicle in our hands, not the stale one).
        if (age > state_timeout_) {
            setpoint.throttle = 0.0;
            setpoint.roll = 0.0;
            setpoint.pitch = 0.0;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "state telemetry stale (%.2fs), commanding idle",
                age);
        }

        const Command command = autopilot_.update(state_, setpoint, 1.0 / control_rate_);

        std_msgs::msg::Float64MultiArray msg;
        msg.data = {
            clamp_channel(kRcMid + command.roll * kStickRange),
            clamp_channel(kRcMid + command.pitch * kStickRange),
            clamp_channel(kRcMin + command.throttle * 1000.0),
            clamp_channel(kRcMid + command.yaw * kStickRange)};
        rc_override_pub_->publish(msg);
    }

    double control_rate_ = 50.0;
    double state_timeout_ = 2.0;
    bool auto_takeoff_ = false;
    double takeoff_seconds_ = 2.0;
    double takeoff_throttle_ = 0.7;
    double flight_seconds_ = 5.0;
    bool disarm_after_flight_ = true;
    double target_roll_ = 0.0;
    double target_pitch_ = 0.0;

    AttitudeHold autopilot_;
    State state_;
    rclcpp::Time last_state_{0, 0, RCL_ROS_TIME};

    Phase phase_ = Phase::kIdle;
    rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rc_override_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altitude_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace flyrat_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<flyrat_control::ControlNode>());
    rclcpp::shutdown();
    return 0;
}

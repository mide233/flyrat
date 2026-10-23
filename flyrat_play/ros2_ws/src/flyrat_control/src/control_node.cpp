// SPDX-License-Identifier: MIT
//
// flyrat_control node: runs the ROS-free autopilot and feeds its commands back
// to Betaflight through flyrat_msp.
//
//   flyrat_msp/attitude  ──►  autopilot (flyrat::Autopilot)  ──►  rc_override
//   flyrat_msp/position  ──►
//   flyrat_msp/imu       ──►
//
// The estimated state is Betaflight's own; no Gazebo ground truth is used.
//
// Two controllers are available, selected with the `controller` parameter:
//
//   attitude_hold - the minimal demo controller (default)
//   ommpc         - on-manifold MPC on (p, v, R), commanding body rates and
//                   thrust, the same interface the real machine will use
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <Eigen/Geometry>

#include "flyrat/attitude_hold.hpp"
#include "flyrat/ommpc.hpp"
#include "flyrat/rc_mapping.hpp"
#include "flyrat/so3.hpp"
#include "flyrat/thrust_model.hpp"

namespace {

constexpr double kRcMin = 1000.0;
constexpr double kRcMid = 1500.0;
constexpr double kRcMax = 2000.0;
constexpr double kStickRange = 500.0;

double clamp_channel(double value) { return std::max(kRcMin, std::min(kRcMax, value)); }

// Betaflight reports attitude in NED/FRD (x north, y east, z down) while the
// position estimate and the world frame are ENU (x east, y north, z up).
//   R_flu_enu = ENU<-NED * R_ned_frd * FRD<-FLU
// Body rates follow the same change of basis: (p, q, r) -> (p, -q, -r).
const Eigen::Matrix3d& enu_from_ned() {
    static const Eigen::Matrix3d m =
        (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
    return m;
}

const Eigen::Matrix3d& frd_from_flu() {
    static const Eigen::Matrix3d m = Eigen::Vector3d(1, -1, -1).asDiagonal();
    return m;
}

Eigen::Matrix3d attitude_flu_enu(double roll, double pitch, double yaw) {
    // The flight controller reports pitch as positive-forward in this
    // airframe: an open-loop +48 deg/s pitch stick both increased the reported
    // pitch and accelerated the vehicle north. The MPC's world frame therefore
    // needs the negated pitch, otherwise the attitude it believes the vehicle
    // has is mirrored in the pitch axis and every correction pushes the wrong
    // way (the exact runaway seen before this fix).
    const Eigen::Matrix3d r_ned_frd = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                       Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                                          .toRotationMatrix();
    return enu_from_ned() * r_ned_frd * frd_from_flu();
}

} // namespace

namespace flyrat_control {

using flyrat::AttitudeHold;
using flyrat::Command;
using flyrat::MpcCommand;
using flyrat::MpcParameters;
using flyrat::MpcReference;
using flyrat::MpcState;
using flyrat::Ommpc;
using flyrat::RcMapping;
using flyrat::Setpoint;
using flyrat::State;
using flyrat::ThrustModel;

class ControlNode : public rclcpp::Node {
public:
    ControlNode()
        : rclcpp::Node("flyrat_control") {
        control_rate_ = declare_parameter<double>("control_rate", 100.0);
        state_timeout_ = declare_parameter<double>("state_timeout", 2.0);
        controller_ = declare_parameter<std::string>("controller", "attitude_hold");
        auto_takeoff_ = declare_parameter<bool>("auto_takeoff", false);
        takeoff_seconds_ = declare_parameter<double>("takeoff_seconds", 2.0);
        takeoff_throttle_ = declare_parameter<double>("takeoff_throttle", 0.7);
        takeoff_height_ = declare_parameter<double>("takeoff_height", 1.0);
        flight_seconds_ = declare_parameter<double>("flight_seconds", 5.0);
        disarm_after_flight_ = declare_parameter<bool>("disarm_after_flight", true);
        target_roll_ = declare_parameter<double>("target_roll", 0.0);
        target_pitch_ = declare_parameter<double>("target_pitch", 0.0);

        if (controller_ != "attitude_hold" && controller_ != "ommpc") {
            throw std::runtime_error(
                "controller must be 'attitude_hold' or 'ommpc', got '" + controller_ + "'");
        }

        // The MPC weights and horizon are exposed so a tuning sweep does not
        // need a rebuild. The defaults are the ones inherited from the
        // reference on-manifold MPC.
        MpcParameters mpc_params;
        mpc_params.horizon = declare_parameter<int>("mpc_horizon", 20);
        mpc_params.step = declare_parameter<double>("mpc_step", 0.01);
        mpc_params.q_position_xy = declare_parameter<double>("mpc_q_position_xy", 1000.0);
        mpc_params.q_position_z = declare_parameter<double>("mpc_q_position_z", 800.0);
        mpc_params.q_velocity = declare_parameter<double>("mpc_q_velocity", 20.0);
        mpc_params.q_attitude_roll_pitch =
            declare_parameter<double>("mpc_q_attitude_roll_pitch", 40.0);
        mpc_params.q_attitude_yaw = declare_parameter<double>("mpc_q_attitude_yaw", 40.0);
        mpc_params.r_thrust = declare_parameter<double>("mpc_r_thrust", 0.5);
        mpc_params.r_body_rate_xy = declare_parameter<double>("mpc_r_body_rate_xy", 1.2);
        mpc_params.r_body_rate_z = declare_parameter<double>("mpc_r_body_rate_z", 0.6);
        mpc_params.weight_decay = declare_parameter<double>("mpc_weight_decay", 0.5);
        mpc_params.max_body_rate_xy = declare_parameter<double>("mpc_max_body_rate_xy", 6.0);
        mpc_params.max_body_rate_z = declare_parameter<double>("mpc_max_body_rate_z", 4.0);
        max_rate_slew_ = declare_parameter<double>("mpc_max_rate_slew", 0.0);
        yaw_offset_ = declare_parameter<double>("mpc_yaw_offset", 0.0);
        trajectory_ = declare_parameter<std::string>("trajectory", "none");
        trajectory_radius_ = declare_parameter<double>("trajectory_radius", 1.5);
        trajectory_period_ = declare_parameter<double>("trajectory_period", 20.0);
        // Throttle -> acceleration. The airframe hovers at about 0.69 throttle,
        // so one unit of throttle buys 9.81/0.69 m/s^2. Adapting this online
        // was tried first and drifted into the model's clamp; a fixed value
        // measured from a hover is more trustworthy in the simulator.
        thrust_to_accel_ = declare_parameter<double>("mpc_thrust_to_accel", 14.2);
        adapt_thrust_ = declare_parameter<bool>("mpc_adapt_thrust", false);
        mpc_ = std::make_unique<Ommpc>(mpc_params);

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
        position_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/flyrat/position", 10, [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
                state_.east = msg->vector.x;
                state_.north = msg->vector.y;
                state_.up = msg->vector.z;
                if (!state_.has_position) {
                    state_.has_position = true;
                    RCLCPP_INFO(get_logger(), "local position available (ENU origin locked)");
                }
            });
        velocity_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/flyrat/velocity", 10, [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
                state_.vel_east = msg->vector.x;
                state_.vel_north = msg->vector.y;
                state_.vel_up = msg->vector.z;
            });
        target_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/flyrat/target", 10, [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {
                target_east_ = msg->vector.x;
                target_north_ = msg->vector.y;
                target_up_ = msg->vector.z;
                RCLCPP_INFO(
                    get_logger(), "target offset set to (%.2f %.2f %.2f)", target_east_,
                    target_north_, target_up_);
            });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/flyrat/imu", 10, [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
                // Betaflight's gyro is FRD and its accelerometer is NWU (z up),
                // so the specific force along the body up axis is used as is.
                body_rates_frd_ = msg->angular_velocity.x;
                body_rates_frd_y_ = msg->angular_velocity.y;
                body_rates_frd_z_ = msg->angular_velocity.z;
                specific_force_z_ = msg->linear_acceleration.z;
                have_imu_ = true;
            });

        const auto period = std::chrono::duration<double>(1.0 / control_rate_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] { tick(); });

        RCLCPP_INFO(
            get_logger(), "flyrat_control up: controller=%s rate=%.1fHz auto_takeoff=%s",
            controller_.c_str(), control_rate_, auto_takeoff_ ? "true" : "false");
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

    // Body rates in the FLU body frame, which is what the MPC works in.
    Eigen::Vector3d body_rates_flu() const {
        return {body_rates_frd_, -body_rates_frd_y_, -body_rates_frd_z_};
    }

    MpcState mpc_state(const rclcpp::Time& stamp) const {
        MpcState s;
        s.position = {state_.east, state_.north, state_.up};
        s.velocity = {state_.vel_east, state_.vel_north, state_.vel_up};
        s.attitude = Eigen::Quaterniond(attitude_flu_enu(state_.roll, state_.pitch, state_.yaw));
        s.body_rates = body_rates_flu();

        // Position and attitude arrive at ~10 Hz while this loop runs at 100 Hz.
        // Extrapolate with the measured velocity and body rates so every solve
        // starts from a state that matches the current instant; reusing a stale
        // state ten times in a row makes the MPC chase its own tail.
        const double age = (stamp - last_state_).nanoseconds() == 0
                               ? 0.0
                               : std::clamp((stamp - last_state_).seconds(), 0.0, 0.1);
        s.position += s.velocity * age;
        // Do NOT propagate the attitude with the gyro here. MSP_RAW_IMU's rate
        // field is the firmware's gyroRateDps(), which divides the raw counts
        // by GYRO_SCALE_2000DPS (0.061 dps/lsb) instead of multiplying, so the
        // reported rate is ~269x the truth. Rotating the attitude by that value
        // injected radians of error every cycle and closed a positive feedback
        // loop. The attitude sample is at most one telemetry period old, which
        // the MPC handles fine without extrapolation.
        return s;
    }

    double takeoff_z() const {
        const double elapsed = (now() - phase_start_).seconds();
        const double ramp = takeoff_seconds_ > 0.0 ? takeoff_seconds_ : 1.0;
        return hold_up_ + takeoff_height_ * std::clamp(elapsed / ramp, 0.0, 1.0);
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
                hold_east_ = state_.east;
                hold_north_ = state_.north;
                hold_up_ = state_.up;
                trajectory_started_ = false;
                hold_yaw_ = std::atan2(
                    attitude_flu_enu(state_.roll, state_.pitch, state_.yaw)(1, 0),
                    attitude_flu_enu(state_.roll, state_.pitch, state_.yaw)(0, 0));
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
                RCLCPP_INFO(get_logger(), "at takeoff altitude");
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
        const bool stale = age > state_timeout_;
        if (stale) {
            setpoint.throttle = 0.0;
            setpoint.roll = 0.0;
            setpoint.pitch = 0.0;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000, "state telemetry stale (%.2fs), commanding idle",
                age);
        }

        std_msgs::msg::Float64MultiArray msg;
        if (controller_ == "ommpc") {
            msg.data = mpc_tick(stamp, stale);
        } else {
            const Command command = autopilot_.update(state_, setpoint, 1.0 / control_rate_);
            msg.data = {
                clamp_channel(kRcMid + command.roll * kStickRange),
                clamp_channel(kRcMid + command.pitch * kStickRange),
                clamp_channel(kRcMin + command.throttle * 1000.0),
                clamp_channel(kRcMid + command.yaw * kStickRange)};
        }
        rc_override_pub_->publish(msg);
    }

    std::vector<double> mpc_tick(const rclcpp::Time& stamp, bool stale) {
        // Nothing to fly on before the estimator has a fix, and no thrust while
        // disarmed: the FC would ignore it anyway, but zero is the safe command.
        const bool armed_and_flying =
            state_.armed && state_.has_position && !stale && phase_ != Phase::kIdle &&
            phase_ != Phase::kArming;

        if (adapt_thrust_ && have_imu_)
            thrust_model_.update(delayed_throttle(stamp), specific_force_z_);
        const double thrust_to_accel =
            adapt_thrust_ ? thrust_model_.thrust_to_accel() : thrust_to_accel_;
        mpc_->set_thrust_to_accel(thrust_to_accel);

        if (!armed_and_flying) {
            return {kRcMid, kRcMid, kRcMin, kRcMid};
        }

        const MpcState state = mpc_state(stamp);

        const int horizon = mpc_->parameters().horizon;
        const double step = mpc_->parameters().step;
        std::vector<MpcReference> references(static_cast<std::size_t>(horizon));
        MpcReference reference;

        const bool tracking = trajectory_ != "none" && phase_ == Phase::kHold;
        if (tracking) {
            if (!trajectory_started_) {
                trajectory_started_ = true;
                trajectory_start_time_ = stamp;
                trajectory_origin_ = state.position;
                RCLCPP_INFO(
                    get_logger(), "tracking a %.1f m circle, %.1f s per lap",
                    trajectory_radius_, trajectory_period_);
            }
            const double t = (stamp - trajectory_start_time_).seconds();
            const double omega = 2.0 * M_PI / std::max(trajectory_period_, 1.0);
            // Put the arming point on the circle so the path starts where the
            // vehicle already is instead of teleporting the reference.
            const bool figure8 = trajectory_ == "figure8";
            // A Gerono lemniscate starts at its own centre, so the arming point
            // is the centre; the circle is shifted by one radius so that it too
            // starts where the vehicle already is.
            const Eigen::Vector3d centre(
                trajectory_origin_.x() - (figure8 ? 0.0 : trajectory_radius_), trajectory_origin_.y(),
                trajectory_origin_.z());
            for (int i = 0; i < horizon; ++i) {
                const double theta = omega * (t + i * step);
                MpcReference& r = references[static_cast<std::size_t>(i)];
                const double a = trajectory_radius_;
                if (figure8) {
                    const double b = 0.5 * trajectory_radius_;
                    r.position = centre + Eigen::Vector3d(a * std::sin(theta),
                                                          b * std::sin(2.0 * theta), 0.0);
                    r.velocity = Eigen::Vector3d(a * omega * std::cos(theta),
                                                 2.0 * b * omega * std::cos(2.0 * theta), 0.0);
                    r.acceleration = Eigen::Vector3d(
                        -a * omega * omega * std::sin(theta),
                        -4.0 * b * omega * omega * std::sin(2.0 * theta), 0.0);
                } else {
                    r.position = centre + Eigen::Vector3d(a * std::cos(theta), a * std::sin(theta), 0.0);
                    r.velocity = Eigen::Vector3d(-a * omega * std::sin(theta), a * omega * std::cos(theta), 0.0);
                    r.acceleration = Eigen::Vector3d(
                        -a * omega * omega * std::cos(theta),
                        -a * omega * omega * std::sin(theta), 0.0);
                }
                // Keep the heading captured at arming. Pointing the nose
                // along the tangent is a step of up to 180 deg at t=0, which
                // saturated the yaw channel before the position loop engaged.
                r.yaw = hold_yaw_;
                r.yaw_rate = 0.0;
            }
            reference = references.front();
        } else {
            // The hold point is the arming position plus whatever offset the
            // mission layer asks for through /flyrat/target.
            reference.position = {hold_east_ + target_east_, hold_north_ + target_north_,
                                  (phase_ == Phase::kClimb ? takeoff_z() : hold_up_ + takeoff_height_) +
                                      target_up_};
            reference.velocity.setZero();
            reference.acceleration.setZero();
            // Hold the heading captured at arming. Following the measured yaw
            // (the previous behaviour) left the controller indifferent to a yaw
            // disturbance, so nothing opposed the spin that started at liftoff.
            reference.yaw = hold_yaw_ + yaw_offset_;
            for (auto& r : references)
                r = reference;
        }

        const MpcCommand command = mpc_->solve(state, references);
        if (!command.solved) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000, "MPC did not converge, holding level");
            return {kRcMid, kRcMid, kRcMid, kRcMid};
        }

        const double throttle = std::clamp(command.thrust_accel / thrust_to_accel, 0.0, 1.0);
        last_throttle_ = throttle;
        thrust_history_.emplace_back(stamp, throttle);

        const double g = mpc_->parameters().gravity;
        const Eigen::Quaterniond desired = Ommpc::attitude_from_acceleration(
            reference.acceleration + Eigen::Vector3d(0.0, 0.0, g), reference.yaw, g);
        const double attitude_error_deg =
            flyrat::so3::log(state.attitude.toRotationMatrix().transpose() *
                             desired.toRotationMatrix())
                .norm() *
            57.29577951308232;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 100,
            "mpc: cmd=(%.2f %.2f %.2f) att_err=%.0f vel=(%.2f %.2f %.2f) att_thr=%.1f "
            "thr2acc=%.1f thr=%.2f err=(%.2f %.2f %.2f) solve=%.2fms it=%d",
            command.body_rates.x(), command.body_rates.y(), command.body_rates.z(),
            attitude_error_deg, state.velocity.x(), state.velocity.y(), state.velocity.z(),
            command.thrust_accel, thrust_to_accel, throttle,
            reference.position.x() - state.position.x(),
            reference.position.y() - state.position.y(),
            reference.position.z() - state.position.z(), command.solve_time_ms,
            command.iterations);

        // A rate command that jumps between +/-6 rad/s every cycle is beyond
        // what the Betaflight rate loop can follow; the resulting phase lag
        // closes a positive feedback loop. Limiting the slew keeps the command
        // inside the actuator's bandwidth.
        Eigen::Vector3d rates = command.body_rates; // FLU
        if (max_rate_slew_ > 0.0) {
            const double step = max_rate_slew_ / control_rate_;
            for (int i = 0; i < 3; ++i)
                rates[i] = std::clamp(rates[i], last_rates_[i] - step, last_rates_[i] + step);
        }
        last_rates_ = rates;
        // The simulated airframe pitches nose-down for a positive stick and
        // therefore accelerates along +x_body (verified with an open-loop
        // step: +48 deg/s pitch -> 2.8 m/s north). The MPC's body rate is in
        // the same convention, so the sign goes through unchanged.
        return {
            clamp_channel(rc_.pulse_from_stick(rc_.stick_from_rate(rates.x()))),
            clamp_channel(rc_.pulse_from_stick(rc_.stick_from_rate(rates.y()))),
            clamp_channel(rc_.pulse_from_throttle(throttle)),
            clamp_channel(rc_.pulse_from_stick(rc_.stick_from_rate(-rates.z())))};
    }

    // Thrust -> acceleration lags the command by a few tens of milliseconds
    // (ESC ramp + rotor inertia). Correlating the specific force with the
    // throttle of ~40 ms ago keeps the recursive least squares unbiased.
    double delayed_throttle(const rclcpp::Time& stamp) {
        while (!thrust_history_.empty() &&
               (stamp - thrust_history_.front().first).seconds() > 0.035) {
            last_throttle_ = thrust_history_.front().second;
            thrust_history_.pop_front();
        }
        return last_throttle_;
    }

    double control_rate_ = 100.0;
    double state_timeout_ = 2.0;
    std::string controller_ = "attitude_hold";
    bool auto_takeoff_ = false;
    double takeoff_seconds_ = 2.0;
    double takeoff_throttle_ = 0.7;
    double takeoff_height_ = 1.0;
    double flight_seconds_ = 5.0;
    bool disarm_after_flight_ = true;
    double target_roll_ = 0.0;
    double target_pitch_ = 0.0;

    AttitudeHold autopilot_;
    std::unique_ptr<Ommpc> mpc_;
    double max_rate_slew_ = 0.0;
    double yaw_offset_ = 0.0;

    // Optional trajectory tracking (currently a horizontal circle).
    std::string trajectory_ = "none";
    double trajectory_radius_ = 1.5;
    double trajectory_period_ = 20.0;
    bool trajectory_started_ = false;
    rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
    Eigen::Vector3d trajectory_origin_ = Eigen::Vector3d::Zero();
    double thrust_to_accel_ = 14.2;
    bool adapt_thrust_ = false;
    Eigen::Vector3d last_rates_ = Eigen::Vector3d::Zero();
    ThrustModel thrust_model_;
    RcMapping rc_;
    State state_;
    rclcpp::Time last_state_{0, 0, RCL_ROS_TIME};

    // IMU, in the flight controller's own frames.
    double body_rates_frd_ = 0.0;
    double body_rates_frd_y_ = 0.0;
    double body_rates_frd_z_ = 0.0;
    double specific_force_z_ = 0.0;
    bool have_imu_ = false;

    // Hold point captured when the vehicle arms.
    double hold_east_ = 0.0;
    double hold_north_ = 0.0;
    double hold_up_ = 0.0;

    // Offset from the hold point commanded through /flyrat/target.
    double target_east_ = 0.0;
    double target_north_ = 0.0;
    double target_up_ = 0.0;
    double hold_yaw_ = 0.0;

    double last_throttle_ = 0.0;
    std::deque<std::pair<rclcpp::Time, double>> thrust_history_;

    Phase phase_ = Phase::kIdle;
    rclcpp::Time phase_start_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rc_override_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_client_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr target_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr status_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr altitude_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace flyrat_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<flyrat_control::ControlNode>());
    rclcpp::shutdown();
    return 0;
}

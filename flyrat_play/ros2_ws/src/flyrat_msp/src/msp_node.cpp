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
#include <array>
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
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "flyrat/hardware/serial_transport.hpp"
#include "flyrat/hardware/tcp_transport.hpp"
#include "flyrat/local_estimator.hpp"
#include "flyrat/msp_interact.hpp"
#include "flyrat/rc_receiver.hpp"

namespace flyrat_msp {

using flyrat::AltitudeFrame;
using flyrat::AnalogFrame;
using flyrat::Attitude;
using flyrat::GpsFrame;
using flyrat::ImuSample;
using flyrat::LocalEstimator;
using flyrat::LocalPosition;
using flyrat::MotorFrame;
using flyrat::MspInteract;
using flyrat::RcFrame;
using flyrat::RcReceiver;
using flyrat::StatusFrame;
namespace {

// Raw Betaflight receiver channel order.
constexpr size_t kChRoll = 0;
constexpr size_t kChPitch = 1;
constexpr size_t kChThrottle = 2;
constexpr size_t kChYaw = 3;
constexpr size_t kChAux1 = 4; // ARM
constexpr size_t kChAux2 = 5; // MSP OVERRIDE
constexpr size_t kChannelCount = 16;

constexpr uint16_t kRcMin = 1000;
constexpr uint16_t kRcMid = 1500;
constexpr uint16_t kRcMax = 2000;
constexpr uint16_t kSwitchHigh = 1800;
// Betaflight latches ARMING_DISABLED_NOT_DISARMED when the RX link recovers
// while the ARM switch is already high, and ARMING_DISABLED_ARM_SWITCH while
// any other block is active with the switch high. Both only clear once the
// switch has been low with a good RX link, so every arm request has to start
// with a short low pulse instead of holding the switch high forever.
constexpr auto kArmPulseLow = std::chrono::milliseconds(250);
// While an arm request is outstanding, repeat the pulse so a link glitch that
// re-latches the flags does not leave the vehicle disarmed until a restart.
constexpr auto kArmRetryPeriod = std::chrono::seconds(3);

// Raw IMU scaling used by the SITL simulator.
// MSP_RAW_IMU does not report raw sensor counts. The SITL fills the
// accelerometer with `accADC = acc_mps2 * 256 / 9.80665`
// (src/platform/SIMULATOR/sitl.c: ACC_SCALE), i.e. 256 counts per g, and the
// gyro with `gyroRateDps()`, which is already in degrees per second. Using the
// datasheet-style 2048 counts/g and 16.4 LSB/(deg/s) made the published
// acceleration 8x too small and the angular rate 16x too fast.
constexpr double kAccOneG = 256.0;
constexpr double kGyroLsbPerDps = 1.0;
constexpr double kStandardGravity = 9.80665;

constexpr double kPi = 3.14159265358979323846;

double to_radians(double degrees) { return degrees * kPi / 180.0; }

} // namespace

class MspNode : public rclcpp::Node {
public:
    MspNode()
        : rclcpp::Node("flyrat_msp") {
        host_ = declare_parameter<std::string>("host", "127.0.0.1");
        msp_port_ = declare_parameter<int>("msp_port", 5761);
        rc_port_ = declare_parameter<int>("rc_port", 9004);
        rate_ = declare_parameter<double>("rate", 50.0);
        // 50 Hz: the MPC runs at 100 Hz and needs a state sample every control
        // cycle; 10 Hz forced the controller to extrapolate over 100 ms.
        telemetry_rate_ = declare_parameter<double>("telemetry_rate", 50.0);
        rc_timeout_ = declare_parameter<double>("rc_timeout", 0.25);
        override_enabled_ = declare_parameter<bool>("msp_override", true);
        armed_ = declare_parameter<bool>("arm_on_start", false);
        // When the simulator is the position source, /flyrat/position and
        // /flyrat/velocity are republished from the ground-truth odometry at
        // 60 Hz instead of the 10 Hz GPS path.
        use_ground_truth_ = declare_parameter<bool>("use_ground_truth", false);
        // "tcp" for SITL, "serial" for the real vehicle (UART to the FC).
        transport_ = declare_parameter<std::string>("transport", "tcp");
        msp_device_ = declare_parameter<std::string>("msp_device", "/dev/ttyS3");
        msp_baud_ = declare_parameter<int>("msp_baud", 921600);
        imu_rate_ = declare_parameter<double>("imu_rate", telemetry_rate_);
        gps_rate_ = declare_parameter<double>("gps_rate", telemetry_rate_);
        rc_rate_ = declare_parameter<double>("rc_rate", 2.0);
        extras_rate_ = declare_parameter<double>("extras_rate", 2.0);

        if (rate_ <= 0.0)
            throw std::runtime_error("rate must be > 0");
        if (telemetry_rate_ <= 0.0) {
            throw std::runtime_error("telemetry_rate must be > 0");
        }
        if (imu_rate_ <= 0.0 || gps_rate_ <= 0.0 || rc_rate_ <= 0.0 || extras_rate_ <= 0.0)
            throw std::runtime_error("imu_rate, gps_rate, rc_rate and extras_rate must be > 0");
        if (transport_ != "tcp" && transport_ != "serial")
            throw std::runtime_error("transport must be 'tcp' or 'serial'");

        auto qos = rclcpp::QoS(10);
        attitude_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>("/flyrat/attitude", qos);
        rc_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>("/flyrat/rc", qos);
        motor_pub_ = create_publisher<std_msgs::msg::UInt16MultiArray>("/flyrat/motor", qos);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/flyrat/imu", qos);
        status_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("/flyrat/status", qos);
        armed_pub_ = create_publisher<std_msgs::msg::Bool>("/flyrat/armed", qos);
        altitude_pub_ = create_publisher<std_msgs::msg::Float32>("/flyrat/altitude", qos);
        position_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>("/flyrat/position", qos);
        velocity_pub_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>("/flyrat/velocity", qos);

        if (use_ground_truth_) {
            ground_truth_timer_ = create_wall_timer(std::chrono::microseconds(16667), [this] {
                std::array<double, 3> p, v;
                {
                    std::lock_guard<std::mutex> lock(ground_truth_mutex_);
                    if (!ground_truth_valid_)
                        return;
                    p = ground_truth_position_;
                    v = ground_truth_velocity_;
                }
                const rclcpp::Time stamp = now();
                geometry_msgs::msg::Vector3Stamped pos;
                pos.header.stamp = stamp;
                pos.header.frame_id = "map";
                pos.vector.x = p[0];
                pos.vector.y = p[1];
                pos.vector.z = p[2];
                position_pub_->publish(pos);

                geometry_msgs::msg::Vector3Stamped vel;
                vel.header = pos.header;
                vel.vector.x = v[0];
                vel.vector.y = v[1];
                vel.vector.z = v[2];
                velocity_pub_->publish(vel);
            });
        }

        ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/flyrat/ground_truth/odometry", rclcpp::SensorDataQoS(),
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
                const auto& p = msg->pose.pose.position;
                const auto& o = msg->pose.pose.orientation;
                const auto& v = msg->twist.twist.linear;
                // Gazebo reports the twist in the body frame; rotate it into
                // the world (ENU) frame with the published orientation.
                const double x = o.x, y = o.y, z = o.z, w = o.w;
                const double vx = v.x, vy = v.y, vz = v.z;
                std::lock_guard<std::mutex> lock(ground_truth_mutex_);
                ground_truth_position_ = {p.x, p.y, p.z};
                ground_truth_velocity_ = {
                    (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - w * z) * vy +
                        2 * (x * z + w * y) * vz,
                    2 * (x * y + w * z) * vx + (1 - 2 * (x * x + z * z)) * vy +
                        2 * (y * z - w * x) * vz,
                    2 * (x * z - w * y) * vx + 2 * (y * z + w * x) * vy +
                        (1 - 2 * (x * x + y * y)) * vz};
                ground_truth_valid_ = true;
            });
        battery_pub_ = create_publisher<std_msgs::msg::Float32>("/flyrat/battery", qos);

        rc_override_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/flyrat/rc_override", qos,
            [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (msg->data.size() < kChannelCount) {
                    for (size_t i = 0; i < msg->data.size(); ++i) {
                        override_[i] = static_cast<uint16_t>(
                            std::clamp(msg->data[i], 0.0, static_cast<double>(kRcMax)));
                    }
                }
                override_stamp_ = std::chrono::steady_clock::now();
                has_override_ = true;
            });

        set_arm_srv_ = create_service<std_srvs::srv::SetBool>(
            "/flyrat/set_arm", [this](
                                   const std_srvs::srv::SetBool::Request::SharedPtr req,
                                   std_srvs::srv::SetBool::Response::SharedPtr res) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    armed_ = req->data;
                    if (req->data) {
                        // Drop AUX1 before raising it: Betaflight latches
                        // NOT_DISARMED (and ARM_SWITCH) when the switch is
                        // already high, and only clears them on a low with a
                        // healthy RX link, so a retry without the pulse can
                        // never arm.
                        const auto now = std::chrono::steady_clock::now();
                        arm_release_ = now + kArmPulseLow;
                        last_arm_pulse_ = now;
                    }
                }
                res->success = true;
                res->message = req->data ? "ARM switch high (after a low pulse)" : "ARM switch low";
                RCLCPP_INFO(get_logger(), "ARM switch -> %s", req->data ? "high" : "low");
            });

        set_override_srv_ = create_service<std_srvs::srv::SetBool>(
            "/flyrat/set_msp_override", [this](
                                            const std_srvs::srv::SetBool::Request::SharedPtr req,
                                            std_srvs::srv::SetBool::Response::SharedPtr res) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    override_enabled_ = req->data;
                }
                res->success = true;
                res->message = req->data ? "MSP OVERRIDE switch high" : "MSP OVERRIDE switch low";
                RCLCPP_INFO(get_logger(), "MSP OVERRIDE switch -> %s", req->data ? "high" : "low");
            });
    }

    ~MspNode() override {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
        if (sender_thread_.joinable())
            sender_thread_.join();
        receiver_.close();
        if (msp_)
            msp_->stop();
    }

    // The UDP receiver is independent of MSP: open it once at startup and retry
    // from the telemetry thread only, so the RC stream never waits for the
    // handshake.
    void ensure_receiver() {
        if (receiver_.is_open()) {
            return;
        }
        std::lock_guard<std::mutex> lock(link_mutex_);
        if (!receiver_.is_open()) {
            receiver_.open(host_, rc_port_);
        }
    }

    void start() {
        override_.assign(kChannelCount, kRcMid);
        override_[kChThrottle] = kRcMin;
        running_ = true;
        ensure_receiver();
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
        const double age = std::chrono::duration<double>(now - override_stamp_).count();
        if (has_override_ && age <= rc_timeout_) {
            for (size_t i = 0; i < 4; ++i)
                channels[i] = override_[i];
        }
        // Re-pulse while the request is outstanding: an RX glitch re-latches
        // the arming flags, and the FC only clears them on a fresh edge.
        if (armed_ && !fc_armed_.load() && now - last_arm_pulse_ >= kArmRetryPeriod) {
            arm_release_ = now + kArmPulseLow;
            last_arm_pulse_ = now;
        }
        channels[kChAux1] = (armed_ && now >= arm_release_) ? kSwitchHigh : kRcMin;
        // Keep the MSP override low until the FC is armed: Betaflight sets
        // ARMING_DISABLED_MSP while an override is active, which blocked
        // every arming attempt. The same channels already reach the FC
        // through the UDP receiver, so nothing changes before arming.
        channels[kChAux2] = (override_enabled_ && fc_armed_) ? kSwitchHigh : kRcMin;
        return channels;
    }

    void connect_links() {
        std::lock_guard<std::mutex> lock(link_mutex_);
        // MspInteract owns reconnection after the first successful start.
        if (msp_) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now < next_connect_attempt_) {
            return;
        }

        std::unique_ptr<flyrat::hardware::Transport> transport;
        std::string endpoint;
        if (transport_ == "serial") {
            transport = flyrat::hardware::make_serial_transport(msp_device_, msp_baud_);
            endpoint = msp_device_ + " @" + std::to_string(msp_baud_);
        } else {
            transport = flyrat::hardware::make_tcp_transport(host_, msp_port_);
            endpoint = host_ + ":" + std::to_string(msp_port_);
        }

        flyrat::MspConfig config;
        config.poll.attitude_hz = static_cast<int>(std::lround(telemetry_rate_));
        config.poll.imu_hz = static_cast<int>(std::lround(imu_rate_));
        config.poll.status_hz = static_cast<int>(std::lround(telemetry_rate_));
        config.poll.altitude_hz = static_cast<int>(std::lround(telemetry_rate_));
        config.poll.rc_hz = static_cast<int>(std::lround(rc_rate_));
        config.poll.analog_hz = static_cast<int>(std::lround(extras_rate_));
        config.poll.motor_hz = static_cast<int>(std::lround(extras_rate_));
        config.poll.gps_hz = static_cast<int>(std::lround(gps_rate_));
        config.poll.debug_hz = 0;
        config.poll_timeout_ms = 20;
        config.request_timeout_ms = 500;
        config.reconnect_backoff_ms = 1000;
        config.status_extended = true;

        auto candidate = std::make_unique<MspInteract>(config, std::move(transport));
        install_callbacks(*candidate);
        try {
            candidate->start();
        } catch (const std::exception& exc) {
            next_connect_attempt_ = now + std::chrono::seconds(1);
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "MSP link to %s failed: %s. Check that the flight controller is "
                "processing MSP and that the UART/baud are correct.",
                endpoint.c_str(), exc.what());
            return;
        }

        msp_ptr_.store(candidate.get(), std::memory_order_release);
        msp_ = std::move(candidate);
        RCLCPP_INFO(
            get_logger(), "connected to MSP at %s, RC -> %s:%d", endpoint.c_str(),
            host_.c_str(), rc_port_);
    }

    // Dedicated RC sender. It runs at the full control rate and never blocks on
    // the MSP link: the handshake can take up to a second when the flight
    // controller is wedged, and starving the UDP receiver for that long makes
    // Betaflight raise RXLOSS (which in turn latches the arming flags).
    void sender_loop() {
        const auto period = std::chrono::duration<double>(1.0 / rate_);
        auto next = std::chrono::steady_clock::now();
        while (running_ && rclcpp::ok()) {
            try {
                const std::vector<uint16_t> channels = build_channels();
                const double ts = std::chrono::duration<double>(
                                      std::chrono::steady_clock::now().time_since_epoch())
                                      .count();
                receiver_.send(channels, ts);
                MspInteract* msp = msp_ptr_.load(std::memory_order_acquire);
                if (msp && msp->connected()) {
                    msp->send_rc(channels);
                }
            } catch (const std::exception& exc) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000, "RC link error: %s", exc.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
            std::this_thread::sleep_until(next);
            if (std::chrono::steady_clock::now() > next + period * 5) {
                next = std::chrono::steady_clock::now(); // fell behind, resync
            }
        }
    }

    void install_callbacks(MspInteract& msp) {
        msp.set_attitude_callback([this](const flyrat::Sample<Attitude>& sample) {
            publish_attitude(sample.value);
        });
        msp.set_imu_callback([this](const flyrat::Sample<ImuSample>& sample) {
            publish_imu(sample.value);
        });
        msp.set_status_callback([this](const flyrat::Sample<StatusFrame>& sample) {
            publish_status(sample.value);
        });
        msp.set_altitude_callback([this](const flyrat::Sample<AltitudeFrame>& sample) {
            publish_altitude(sample.value);
        });
        msp.set_rc_callback([this](const flyrat::Sample<RcFrame>& sample) {
            publish_rc(sample.value);
        });
        msp.set_motor_callback([this](const flyrat::Sample<MotorFrame>& sample) {
            publish_motor(sample.value);
        });
        msp.set_analog_callback([this](const flyrat::Sample<AnalogFrame>& sample) {
            publish_analog(sample.value);
        });
        msp.set_gps_callback([this](const flyrat::Sample<GpsFrame>& sample) {
            AltitudeFrame altitude;
            {
                std::lock_guard<std::mutex> lock(altitude_mutex_);
                if (!has_altitude_)
                    return;
                altitude = latest_altitude_;
            }
            publish_position(sample.value, altitude);
        });
        msp.set_link_callback([this](bool up, const std::string& why) {
            if (up) {
                RCLCPP_INFO(get_logger(), "MSP link up");
            } else {
                RCLCPP_WARN(get_logger(), "MSP link down: %s", why.c_str());
            }
        });
    }

    // All publish_* helpers run on MspInteract's dispatch thread. They only
    // copy the decoded structs into ROS messages; no blocking calls here.
    void publish_attitude(const Attitude& attitude) {
        geometry_msgs::msg::Vector3Stamped a;
        a.header.stamp = now();
        a.header.frame_id = "base_link";
        a.vector.x = to_radians(attitude.roll_deg);
        a.vector.y = to_radians(attitude.pitch_deg);
        a.vector.z = to_radians(attitude.yaw_deg);
        attitude_pub_->publish(a);
    }

    void publish_imu(const ImuSample& imu) {
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now();
        imu_msg.header.frame_id = "base_link";
        imu_msg.orientation_covariance[0] = -1.0; // no orientation estimate
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
    }

    void publish_status(const StatusFrame& status) {
        fc_armed_ = status.armed;
        const bool armed = status.armed;
        const bool msp_override = status.msp_override;
        std_msgs::msg::Int32MultiArray status_msg;
        status_msg.data = {
            armed ? 1 : 0, msp_override ? 1 : 0, static_cast<int32_t>(status.cycletime),
            static_cast<int32_t>(status.sensors)};
        status_pub_->publish(status_msg);

        std_msgs::msg::Bool armed_msg;
        armed_msg.data = armed;
        armed_pub_->publish(armed_msg);

        if (!status.armed) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 2000, "arming blocked: flags=0x%08x",
                status.arming_disable_flags);
        }
    }

    void publish_altitude(const AltitudeFrame& altitude) {
        {
            std::lock_guard<std::mutex> lock(altitude_mutex_);
            latest_altitude_ = altitude;
            has_altitude_ = true;
        }
        std_msgs::msg::Float32 altitude_msg;
        altitude_msg.data = altitude.valid ? static_cast<float>(altitude.altitude_m)
                                           : std::numeric_limits<float>::quiet_NaN();
        altitude_pub_->publish(altitude_msg);
    }

    void publish_rc(const RcFrame& rc) {
        // MSP_RC returns internal order (roll, pitch, yaw, throttle, aux...);
        // republish in raw receiver order for consistency with the wire format.
        std_msgs::msg::UInt16MultiArray rc_msg;
        rc_msg.layout.data_offset = 0;
        if (rc.channels.size() >= 4) {
            rc_msg.data = {rc.channels[0], rc.channels[1], rc.channels[3], rc.channels[2]};
            for (size_t i = 4; i < rc.channels.size(); ++i) {
                rc_msg.data.push_back(rc.channels[i]);
            }
        } else {
            rc_msg.data = rc.channels;
        }
        rc_pub_->publish(rc_msg);
    }

    void publish_motor(const MotorFrame& motors) {
        std_msgs::msg::UInt16MultiArray motor_msg;
        motor_msg.data = motors.motors;
        motor_pub_->publish(motor_msg);
    }

    void publish_analog(const AnalogFrame& analog) {
        std_msgs::msg::Float32 battery_msg;
        battery_msg.data = static_cast<float>(analog.voltage_v);
        battery_pub_->publish(battery_msg);
    }

    // GPS -> local ENU. Malformed frames are dropped by MspInteract, so this
    // only runs for a valid sample; a missing altitude sample just skips the
    // position update.
    void publish_position(const GpsFrame& gps, const AltitudeFrame& altitude) {
        if (use_ground_truth_)
            return;

        if (gps.fix != last_fix_) {
            last_fix_ = gps.fix;
            if (gps.fix) {
                RCLCPP_INFO(
                    get_logger(), "GPS fix acquired (%u sats): local origin locked",
                    static_cast<unsigned>(gps.satellites));
            } else {
                RCLCPP_WARN(
                    get_logger(), "GPS fix lost (%u sats)",
                    static_cast<unsigned>(gps.satellites));
            }
        }

        if (!estimator_.update(gps, altitude.valid, altitude.altitude_m, altitude.vario_m_s))
            return;

        const LocalPosition& p = estimator_.position();
        geometry_msgs::msg::Vector3Stamped pos;
        pos.header.stamp = now();
        pos.header.frame_id = "map";
        pos.vector.x = p.east_m;
        pos.vector.y = p.north_m;
        pos.vector.z = p.up_m;
        position_pub_->publish(pos);

        geometry_msgs::msg::Vector3Stamped vel;
        vel.header.stamp = pos.header.stamp;
        vel.header.frame_id = "map";
        vel.vector.x = p.vel_east_mps;
        vel.vector.y = p.vel_north_mps;
        vel.vector.z = p.vel_up_mps;
        velocity_pub_->publish(vel);
    }

    // Supervises the UDP receiver and the initial MSP connection. Once
    // MspInteract has started it reconnects on its own; this loop only creates
    // the object when the first handshake failed.
    void loop() {
        const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / telemetry_rate_));
        auto next = std::chrono::steady_clock::now();
        auto next_stats = next + std::chrono::seconds(5);
        while (running_ && rclcpp::ok()) {
            ensure_receiver();
            connect_links();

            const auto now_steady = std::chrono::steady_clock::now();
            if (now_steady >= next_stats) {
                MspInteract* msp = msp_ptr_.load(std::memory_order_acquire);
                if (msp) {
                    const auto stats = msp->stats();
                    RCLCPP_INFO(
                        get_logger(),
                        "MSP %s: tx=%llu rx=%llu timeouts=%llu rc_ack=%llu/%llu "
                        "checksum=%llu reconnects=%llu",
                        stats.connected ? "up" : "down",
                        static_cast<unsigned long long>(stats.tx_frames),
                        static_cast<unsigned long long>(stats.rx_frames),
                        static_cast<unsigned long long>(stats.timeouts),
                        static_cast<unsigned long long>(stats.rc_ack),
                        static_cast<unsigned long long>(stats.rc_sent),
                        static_cast<unsigned long long>(stats.checksum_errors),
                        static_cast<unsigned long long>(stats.reconnects));
                }
                next_stats = now_steady + std::chrono::seconds(5);
            }

            next += period;
            std::this_thread::sleep_until(next);
            if (std::chrono::steady_clock::now() > next + period * 5) {
                next = std::chrono::steady_clock::now(); // fell behind, resync
            }
        }
    }

    std::string host_;
    int msp_port_ = 5761;
    int rc_port_ = 9004;
    double rate_ = 50.0;
    double telemetry_rate_ = 10.0;
    double imu_rate_ = 50.0;
    double gps_rate_ = 10.0;
    double rc_rate_ = 2.0;
    double extras_rate_ = 2.0;
    double rc_timeout_ = 0.25;
    std::string transport_ = "tcp";
    std::string msp_device_ = "/dev/ttyS3";
    int msp_baud_ = 921600;

    std::mutex mutex_;
    std::mutex link_mutex_;
    std::mutex altitude_mutex_;
    // Link supervision: when the next initial connection may be attempted.
    std::chrono::steady_clock::time_point next_connect_attempt_{};
    // ARM switch gating: AUX1 stays low until arm_release_ (the pulse), and
    std::chrono::steady_clock::time_point arm_release_{};
    std::chrono::steady_clock::time_point last_arm_pulse_{};
    bool armed_ = false;
    bool override_enabled_ = true;
    // True once the flight controller itself reports armed.
    std::atomic<bool> fc_armed_{false};
    bool has_override_ = false;

    std::vector<uint16_t> override_;
    std::chrono::steady_clock::time_point override_stamp_{};

    std::atomic<bool> running_{false};
    std::thread thread_;
    std::thread sender_thread_;

    std::unique_ptr<MspInteract> msp_;
    std::atomic<MspInteract*> msp_ptr_{nullptr};
    RcReceiver receiver_;

    AltitudeFrame latest_altitude_;
    bool has_altitude_ = false;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr rc_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr armed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
    bool use_ground_truth_ = false;
    std::mutex ground_truth_mutex_;
    std::array<double, 3> ground_truth_position_{{0.0, 0.0, 0.0}};
    std::array<double, 3> ground_truth_velocity_{{0.0, 0.0, 0.0}};
    bool ground_truth_valid_ = false;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
    rclcpp::TimerBase::SharedPtr ground_truth_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_pub_;

    LocalEstimator estimator_;
    bool last_fix_ = false;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rc_override_sub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_arm_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_override_srv_;
};

} // namespace flyrat_msp

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

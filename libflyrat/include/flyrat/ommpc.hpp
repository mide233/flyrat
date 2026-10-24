// SPDX-License-Identifier: MIT
//
// On-manifold model predictive control (OMMPC) for a quadrotor.
//
// The formulation follows Lu, Xu and Zhang, "On-manifold model predictive
// control for trajectory tracking on robotic systems" (IEEE TIE 70(9), 2023)
// as used by the SUPER planner's controller: the state (p, v, R) is linearised
// along the reference trajectory in the local coordinates of the manifold, so
// the predictive model is a plain linear time-varying error system solved as a
// QP. Every cycle the measured state is retracted onto SO(3), the error
// delta_x = (p_d - p, v_d - v, log(R^T R_d)) is formed, and the QP returns
// delta_u = u_d - u; the command applied to the flight controller is u_d -
// delta_u.
//
// Inputs are the collective thrust acceleration along the body z axis and the
// body angular rates, which is exactly the interface Betaflight exposes in
// acro mode. The class is ROS-free: it depends on Eigen and OSQP only.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

namespace flyrat {

// One point of the reference trajectory, in the local ENU frame.
struct MpcReference {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double yaw_rate = 0.0;
};

// Measured state of the vehicle, in the local ENU frame.
struct MpcState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity(); // body -> world
    Eigen::Vector3d body_rates = Eigen::Vector3d::Zero();         // rad/s
};

struct MpcParameters {
    int horizon = 20;         // number of prediction steps
    double step = 0.01;       // prediction step, seconds
    double gravity = 9.80665; // m/s^2

    // Error-state weights, ordered (position xy, position z, velocity,
    // attitude roll/pitch, attitude yaw).
    double q_position_xy = 1000.0;
    double q_position_z = 800.0;
    double q_velocity = 20.0;
    double q_attitude_roll_pitch = 40.0;
    double q_attitude_yaw = 40.0;

    // Input weights, ordered (thrust, body rate xy, body rate z).
    double r_thrust = 0.5;
    double r_body_rate_xy = 1.2;
    double r_body_rate_z = 0.6;

    // Per-step weight decay: weight_k = exp(-decay * k / horizon).
    double weight_decay = 0.5;

    double max_body_rate_xy = 6.0; // rad/s
    double max_body_rate_z = 4.0;  // rad/s
    double min_thrust_accel = 1.0; // m/s^2
    double max_thrust_accel = 30.0;

    // Converts a thrust acceleration into a normalised [0, 1] throttle.
    // ThrustModel estimates this online; the default is a 0.705 hover.
    double thrust_to_accel = 9.80665 / 0.705;

    int max_iterations = 4000;
    double eps_abs = 1e-4;
    double eps_rel = 1e-4;
};

struct MpcCommand {
    double thrust = 0.0;                                  // normalised [0, 1]
    Eigen::Vector3d body_rates = Eigen::Vector3d::Zero(); // rad/s
    double thrust_accel = 0.0;                            // m/s^2
    bool solved = false;
    double solve_time_ms = 0.0;
    int iterations = 0;
};

// Linearised error system at one point of the reference: the 9-state error
// (position, velocity, attitude) and the 4 inputs (thrust, body rates).
struct MpcLinearization {
    Eigen::Matrix<double, 9, 9> fx = Eigen::Matrix<double, 9, 9>::Identity();
    Eigen::Matrix<double, 9, 4> fu = Eigen::Matrix<double, 9, 4>::Zero();
};

class Ommpc {
public:
    explicit Ommpc(const MpcParameters& parameters = MpcParameters());
    Ommpc(const Ommpc&) = delete;
    Ommpc& operator=(const Ommpc&) = delete;

    // Solve for a single reference point, held constant over the horizon.
    MpcCommand solve(const MpcState& state, const MpcReference& reference);

    // Solve for a reference trajectory. It is extended with its last point if
    // it is shorter than the horizon.
    MpcCommand solve(const MpcState& state, const std::vector<MpcReference>& references);

    // Error-system linearisation around a reference point. Exposed for tests
    // and for logging; the solver builds the same matrices internally.
    MpcLinearization linearize(
        const Eigen::Quaterniond& attitude, const Eigen::Vector3d& body_rates,
        double thrust_accel) const;

    // Attitude that points the body z axis along `acceleration` (ENU) with the
    // requested heading. Used to turn references into linearisation points.
    static Eigen::Quaterniond attitude_from_acceleration(
        const Eigen::Vector3d& acceleration, double yaw, double gravity);

    const MpcParameters& parameters() const { return params_; }
    void set_thrust_to_accel(double value);

private:
    struct Plan {
        std::vector<Eigen::Vector3d> position;
        std::vector<Eigen::Vector3d> velocity;
        std::vector<Eigen::Quaterniond> attitude;
        std::vector<Eigen::Vector3d> body_rates;
        std::vector<double> thrust_accel;
    };

    Plan build_plan(const std::vector<MpcReference>& references) const;

    MpcParameters params_;
};

} // namespace flyrat

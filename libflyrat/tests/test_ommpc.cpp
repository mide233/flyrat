// SPDX-License-Identifier: MIT
//
// On-manifold MPC checks. Two things are pinned here:
//   1. the linearised error system matches a finite difference of the real
//      error propagation (sign conventions included), and
//   2. the closed loop is stable and tracks with the MPC in the loop, using a
//      rate-lagged quadrotor model that stands in for the flight controller.
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "flyrat/ommpc.hpp"
#include "flyrat/so3.hpp"
#include "flyrat/thrust_model.hpp"

namespace {

// Solve-time bookkeeping: the whole point of an LTV error MPC is that it fits
// in a 100 Hz control loop, so the test reports the measured budget.
double g_max_solve_ms = 0.0;
double g_sum_solve_ms = 0.0;
int g_solves = 0;

void note_solve(const flyrat::MpcCommand& command) {
    g_max_solve_ms = std::max(g_max_solve_ms, command.solve_time_ms);
    g_sum_solve_ms += command.solve_time_ms;
    ++g_solves;
}

int g_failures = 0;

void check(bool ok, const std::string& what) {
    if (ok) {
        std::printf("  ok   %s\n", what.c_str());
    } else {
        std::printf("  FAIL %s\n", what.c_str());
        ++g_failures;
    }
}

bool near(double a, double b, double eps) { return std::fabs(a - b) <= eps; }

constexpr double kGravity = 9.80665;

struct Kinematics {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d attitude = Eigen::Matrix3d::Identity();
};

Kinematics propagate(
    const Kinematics& s, const Eigen::Vector3d& body_rates, double thrust_accel, double dt) {
    Kinematics out;
    out.position = s.position + s.velocity * dt;
    out.velocity = s.velocity +
                   (s.attitude * Eigen::Vector3d::UnitZ() * thrust_accel -
                    kGravity * Eigen::Vector3d::UnitZ()) *
                       dt;
    out.attitude = s.attitude * flyrat::so3::exp(body_rates * dt);
    return out;
}

Eigen::Matrix<double, 9, 1> error_of(const Kinematics& ref, const Kinematics& act) {
    Eigen::Matrix<double, 9, 1> e;
    e.segment<3>(0) = ref.position - act.position;
    e.segment<3>(3) = ref.velocity - act.velocity;
    e.segment<3>(6) = flyrat::so3::log(act.attitude.transpose() * ref.attitude);
    return e;
}

void test_so3() {
    std::printf("so3\n");
    Eigen::Vector3d phi(0.4, -0.7, 1.1);
    const Eigen::Matrix3d r = flyrat::so3::exp(phi);
    check((flyrat::so3::log(r) - phi).norm() < 1e-9, "log(exp(x)) == x");
    check((r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-12, "exp is orthogonal");

    // The left Jacobian exponentiates an increment applied on the left:
    //   exp(J_l(a) d) * exp(a) == exp(a + d) + O(|d|^2)
    // This is the identity the MPC relies on when it retracts a tangent-space
    // perturbation back onto SO(3).
    Eigen::Vector3d a(0.2, 0.3, -0.1), d(1e-6, -2e-6, 3e-6);
    const Eigen::Vector3d mapped = flyrat::so3::log(
        flyrat::so3::exp(flyrat::so3::leftJacobian(a) * d) * flyrat::so3::exp(a));
    check((mapped - (a + d)).norm() < 1e-9, "left Jacobian maps a left increment");
    check(
        (flyrat::so3::leftJacobian(a) * a - a).norm() < 1e-12,
        "left Jacobian leaves its own axis unchanged");
}

void test_linearization() {
    std::printf("linearization\n");
    flyrat::MpcParameters params;
    flyrat::Ommpc mpc(params);

    const Eigen::Vector3d ref_rates(0.5, -0.3, 0.2);
    const double ref_thrust = 12.0;
    const Eigen::Quaterniond ref_att = flyrat::Ommpc::attitude_from_acceleration(
        Eigen::Vector3d(1.0, 0.5, 0.0), 0.3, kGravity);

    const flyrat::MpcLinearization lin = mpc.linearize(ref_att, ref_rates, ref_thrust);

    Kinematics ref;
    ref.position = Eigen::Vector3d(1.0, 2.0, 3.0);
    ref.velocity = Eigen::Vector3d(0.4, -0.2, 0.1);
    ref.attitude = ref_att.toRotationMatrix();

    // State perturbation only: d(u) = 0.
    {
        const double eps = 1e-6;
        Eigen::Matrix<double, 9, 1> dx;
        dx << eps, -2 * eps, 1.5 * eps, 0.5 * eps, -eps, 2 * eps, 0.3 * eps, -0.4 * eps, 0.2 * eps;
        Kinematics act = ref;
        act.position -= dx.segment<3>(0);
        act.velocity -= dx.segment<3>(3);
        act.attitude = flyrat::so3::exp(-dx.segment<3>(6)) * ref.attitude;

        const Eigen::Matrix<double, 9, 1> predicted = lin.fx * dx;
        const Eigen::Matrix<double, 9, 1> actual = error_of(
            propagate(ref, ref_rates, ref_thrust, params.step),
            propagate(act, ref_rates, ref_thrust, params.step));
        const double err = (predicted - actual).norm();
        std::printf("    fx mismatch %.3e\n", err);
        check(err < 1e-6, "Fx matches the finite difference of the error map");
    }

    // Input perturbation only: d(x) = 0, delta_u = u_d - u.
    {
        const double eps = 1e-6;
        Eigen::Matrix<double, 4, 1> du;
        du << eps, 0.5 * eps, -0.7 * eps, 0.4 * eps;
        const Eigen::Vector3d act_rates = ref_rates - du.segment<3>(1);
        const double act_thrust = ref_thrust - du(0);

        const Eigen::Matrix<double, 9, 1> predicted = lin.fu * du;
        const Eigen::Matrix<double, 9, 1> actual = error_of(
            propagate(ref, ref_rates, ref_thrust, params.step),
            propagate(ref, act_rates, act_thrust, params.step));
        const double err = (predicted - actual).norm();
        std::printf("    fu mismatch %.3e\n", err);
        check(err < 1e-6, "Fu matches the finite difference of the error map");
    }
}

struct Plant {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Matrix3d attitude = Eigen::Matrix3d::Identity();
    Eigen::Vector3d body_rates = Eigen::Vector3d::Zero();
    double thrust_accel = kGravity;

    void step(const Eigen::Vector3d& rate_cmd, double thrust_cmd, double dt) {
        // First-order lags stand in for the flight controller's rate loop and
        // for the motor/propeller response.
        const double rate_tau = 0.02;
        const double thrust_tau = 0.03;
        body_rates += (rate_cmd - body_rates) * (dt / rate_tau);
        thrust_accel += (thrust_cmd - thrust_accel) * (dt / thrust_tau);
        velocity += (attitude * Eigen::Vector3d::UnitZ() * thrust_accel -
                     kGravity * Eigen::Vector3d::UnitZ()) *
                    dt;
        position += velocity * dt;
        attitude = attitude * flyrat::so3::exp(body_rates * dt);
    }
};

// Runs the MPC at 100 Hz against the lagged plant at 1 kHz.
bool fly(
    const flyrat::MpcReference& reference, Plant& plant, double seconds, double* final_error,
    double* max_error) {
    flyrat::MpcParameters params;
    params.horizon = 20;
    params.step = 0.01;
    flyrat::Ommpc mpc(params);

    const double control_dt = params.step;
    const double sim_dt = 1e-3;
    Eigen::Vector3d rate_cmd = Eigen::Vector3d::Zero();
    double thrust_cmd = kGravity;

    double t = 0.0;
    int next_control = 0;
    *final_error = 0.0;
    *max_error = 0.0;
    while (t < seconds) {
        if (t + 1e-12 >= next_control * control_dt) {
            flyrat::MpcState state;
            state.position = plant.position;
            state.velocity = plant.velocity;
            state.attitude = Eigen::Quaterniond(plant.attitude);
            state.body_rates = plant.body_rates;

            flyrat::MpcReference ref = reference;
            ref.position += Eigen::Vector3d(0.0, 0.0, 0.0);
            const flyrat::MpcCommand cmd = mpc.solve(state, ref);
            note_solve(cmd);
            if (!cmd.solved)
                return false;
            rate_cmd = cmd.body_rates;
            thrust_cmd = cmd.thrust_accel;
            ++next_control;
        }
        plant.step(rate_cmd, thrust_cmd, sim_dt);
        t += sim_dt;

        const double err = (plant.position - reference.position).norm();
        *max_error = std::max(*max_error, err);
        *final_error = err;
    }
    return true;
}

void test_closed_loop() {
    std::printf("closed loop\n");

    flyrat::MpcReference hover;
    hover.position = Eigen::Vector3d(0.0, 0.0, 1.0);

    {
        Plant plant;
        plant.position = Eigen::Vector3d(1.0, 0.0, 1.0);
        double final_error = 0.0, max_error = 0.0;
        const bool ok = fly(hover, plant, 8.0, &final_error, &max_error);
        std::printf("    position step: final %.3f m, max %.3f m\n", final_error, max_error);
        check(ok, "solver succeeds for the whole run");
        check(final_error < 0.05, "1 m position step converges below 5 cm");
        check((plant.attitude - Eigen::Matrix3d::Identity()).norm() < 0.05,
              "attitude returns to level");
    }

    {
        Plant plant;
        plant.position = Eigen::Vector3d(0.0, 0.0, 1.0);
        plant.attitude = flyrat::so3::exp(Eigen::Vector3d(0.8, -0.5, 0.0));
        double final_error = 0.0, max_error = 0.0;
        const bool ok = fly(hover, plant, 8.0, &final_error, &max_error);
        std::printf("    attitude recovery: final %.3f m, max %.3f m\n", final_error, max_error);
        check(ok, "solver succeeds during attitude recovery");
        check(final_error < 0.10, "recovers from a 50 degree initial tilt");
        check((plant.attitude - Eigen::Matrix3d::Identity()).norm() < 0.05,
              "tilt is driven back to level");
    }

    {
        // A climbing reference exercises the trajectory overload: each control
        // tick hands the solver a horizon of points advancing at 0.2 m/s.
        Plant plant;
        plant.position = Eigen::Vector3d(0.0, 0.0, 1.0);
        const Eigen::Vector3d origin(0.0, 0.0, 1.0);
        const Eigen::Vector3d climb(0.0, 0.0, 0.2);
        flyrat::MpcParameters params;
        flyrat::Ommpc mpc(params);
        const double control_dt = params.step;
        Eigen::Vector3d rate_cmd = Eigen::Vector3d::Zero();
        double thrust_cmd = kGravity;
        double t = 0.0;
        int next_control = 0;
        double max_error = 0.0;
        while (t < 6.0) {
            if (t + 1e-12 >= next_control * control_dt) {
                flyrat::MpcState state;
                state.position = plant.position;
                state.velocity = plant.velocity;
                state.attitude = Eigen::Quaterniond(plant.attitude);
                state.body_rates = plant.body_rates;

                std::vector<flyrat::MpcReference> refs(
                    static_cast<size_t>(params.horizon));
                const double start = next_control * control_dt;
                for (int i = 0; i < params.horizon; ++i) {
                    refs[i].position = origin + climb * (start + i * control_dt);
                    refs[i].velocity = climb;
                }
                const flyrat::MpcCommand cmd = mpc.solve(state, refs);
                note_solve(cmd);
                rate_cmd = cmd.body_rates;
                thrust_cmd = cmd.thrust_accel;
                ++next_control;
            }
            plant.step(rate_cmd, thrust_cmd, 1e-3);
            t += 1e-3;
            max_error = std::max(max_error, (plant.position - (origin + climb * t)).norm());
        }
        const Eigen::Vector3d desired = origin + climb * t;
        const double err = (plant.position - desired).norm();
        const double vel_err = (plant.velocity - climb).norm();
        std::printf("    climb: position %.3f m (max %.3f), velocity %.3f m/s\n",
                    err, max_error, vel_err);
        check(err < 0.10, "tracks a constant-velocity climb");
        check(vel_err < 0.10, "matches the climb rate");
    }

    if (g_solves > 0) {
        std::printf(
            "    solve time: mean %.3f ms, max %.3f ms over %d solves\n",
            g_sum_solve_ms / g_solves, g_max_solve_ms, g_solves);
        check(g_max_solve_ms < 10.0, "worst solve fits a 100 Hz budget");
    }
}

void test_thrust_model() {
    std::printf("thrust model\n");
    flyrat::ThrustModel model(0.705);
    check(near(model.thrust_to_accel(), kGravity / 0.705, 1e-6), "initial gain from hover");

    // Simulate a 20% heavier vehicle: the same throttle produces less thrust.
    const double true_gain = kGravity / 0.80 / 0.8;
    for (int i = 0; i < 400; ++i) {
        const double throttle = 0.45 + 0.01 * std::sin(0.1 * i);
        model.update(throttle, true_gain * throttle);
    }
    std::printf("    converged gain %.2f (true %.2f)\n", model.thrust_to_accel(), true_gain);
    check(near(model.thrust_to_accel(), true_gain, 0.5), "RLS converges to the true gain");
}

} // namespace

int main() {
    test_so3();
    test_linearization();
    test_closed_loop();
    test_thrust_model();
    if (g_failures == 0)
        std::printf("all checks passed\n");
    return g_failures == 0 ? 0 : 1;
}

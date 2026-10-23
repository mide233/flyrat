// SPDX-License-Identifier: MIT
#include "flyrat/ommpc.hpp"

// A system install puts the headers in <prefix>/include/osqp, while the ROS
// vendor package points the include path at that directory itself.
#if __has_include(<osqp/osqp.h>)
#include <osqp/osqp.h>
#else
#include <osqp.h>
#endif

#include "flyrat/so3.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>

namespace flyrat {
namespace {

constexpr int kNx = 9;
constexpr int kNu = 4;

using ErrorState = Eigen::Matrix<double, kNx, 1>;
using ErrorInput = Eigen::Matrix<double, kNu, 1>;

} // namespace

Ommpc::Ommpc(const MpcParameters& parameters)
    : params_(parameters) {
    params_.horizon = std::max(params_.horizon, 2);
    params_.step = std::max(params_.step, 1e-4);
}

void Ommpc::set_thrust_to_accel(double value) {
    if (value > 1e-3)
        params_.thrust_to_accel = value;
}

Eigen::Quaterniond Ommpc::attitude_from_acceleration(
    const Eigen::Vector3d& acceleration, double yaw, double gravity) {
    // The body z axis must carry the thrust that cancels gravity and produces
    // the requested acceleration.
    Eigen::Vector3d z = acceleration + gravity * Eigen::Vector3d::UnitZ();
    if (z.norm() < 1e-6)
        z = Eigen::Vector3d::UnitZ();
    z.normalize();

    Eigen::Vector3d x_hint(std::cos(yaw), std::sin(yaw), 0.0);
    Eigen::Vector3d y = z.cross(x_hint);
    if (y.norm() < 1e-6) {
        // Heading is parallel to the thrust direction: pick any orthogonal one.
        x_hint = Eigen::Vector3d(0.0, 1.0, 0.0);
        y = z.cross(x_hint);
    }
    y.normalize();
    const Eigen::Vector3d x = y.cross(z);

    Eigen::Matrix3d r;
    r.col(0) = x;
    r.col(1) = y;
    r.col(2) = z;
    return Eigen::Quaterniond(r).normalized();
}

MpcLinearization Ommpc::linearize(
    const Eigen::Quaterniond& attitude, const Eigen::Vector3d& body_rates,
    double thrust_accel) const {
    const double t = params_.step;
    const Eigen::Matrix3d r = attitude.toRotationMatrix();

    MpcLinearization lin;
    // d(p) = v
    lin.fx.block<3, 3>(0, 0).setIdentity();
    lin.fx.block<3, 3>(0, 3) = t * Eigen::Matrix3d::Identity();
    // d(v) = -a * R * hat(e3) * d(theta) + R * e3 * d(a)
    lin.fx.block<3, 3>(3, 6) = t * r * so3::hat(Eigen::Vector3d(0.0, 0.0, -thrust_accel));
    // The attitude error lives in the body frame, so it rotates with the body.
    lin.fx.block<3, 3>(6, 6) = so3::exp(-body_rates * t);

    lin.fu.block<3, 1>(3, 0) = t * r * Eigen::Vector3d::UnitZ();
    lin.fu.block<3, 3>(6, 1) = t * so3::leftJacobian(body_rates * t).transpose();
    return lin;
}

Ommpc::Plan Ommpc::build_plan(const std::vector<MpcReference>& references) const {
    const int n = params_.horizon;
    Plan plan;
    plan.position.resize(n);
    plan.velocity.resize(n);
    plan.attitude.resize(n);
    plan.body_rates.resize(n, Eigen::Vector3d::Zero());
    plan.thrust_accel.resize(n);

    for (int i = 0; i < n; ++i) {
        const MpcReference& ref = references[std::min<std::size_t>(i, references.size() - 1)];
        plan.position[i] = ref.position;
        plan.velocity[i] = ref.velocity;
        plan.attitude[i] =
            attitude_from_acceleration(ref.acceleration, ref.yaw, params_.gravity);
        plan.thrust_accel[i] =
            (ref.acceleration + params_.gravity * Eigen::Vector3d::UnitZ()).norm();
    }

    // Reference body rates from the attitude increment between steps.
    for (int i = 0; i < n; ++i) {
        const int a = std::min(i, n - 2);
        const Eigen::Matrix3d rel = plan.attitude[a].toRotationMatrix().transpose() *
                                    plan.attitude[a + 1].toRotationMatrix();
        plan.body_rates[i] = so3::log(rel) / params_.step;
    }
    return plan;
}

MpcCommand Ommpc::solve(const MpcState& state, const MpcReference& reference) {
    return solve(state, std::vector<MpcReference>{reference});
}

MpcCommand Ommpc::solve(const MpcState& state, const std::vector<MpcReference>& references) {
    MpcCommand command;
    if (references.empty())
        return command;

    const auto start = std::chrono::steady_clock::now();
    const int n = params_.horizon;
    const int nv = n * kNu;
    const Plan plan = build_plan(references);

    std::vector<MpcLinearization> linear(n);
    for (int i = 0; i < n; ++i)
        linear[i] = linearize(plan.attitude[i], plan.body_rates[i], plan.thrust_accel[i]);

    // Error state at the current sample: delta = reference - measured, with
    // the attitude error expressed in the body frame.
    ErrorState dx0;
    dx0.segment<3>(0) = plan.position[0] - state.position;
    dx0.segment<3>(3) = plan.velocity[0] - state.velocity;
    const Eigen::Matrix3d r_meas = state.attitude.toRotationMatrix();
    dx0.segment<3>(6) =
        so3::log(r_meas.transpose() * plan.attitude[0].toRotationMatrix());

    // Unroll the linear time-varying error system:
    //   X = psi * dx0 + gamma * U,   X = [dx1 .. dxN]
    Eigen::MatrixXd psi = Eigen::MatrixXd::Zero(kNx * n, kNx);
    Eigen::MatrixXd gamma = Eigen::MatrixXd::Zero(kNx * n, nv);
    {
        Eigen::MatrixXd prev_psi = Eigen::MatrixXd::Identity(kNx, kNx);
        Eigen::MatrixXd prev_gamma = Eigen::MatrixXd::Zero(kNx, nv);
        for (int k = 0; k < n; ++k) {
            Eigen::MatrixXd next_psi = linear[k].fx * prev_psi;
            Eigen::MatrixXd next_gamma = linear[k].fx * prev_gamma;
            next_gamma.block<kNx, kNu>(0, kNu * k) += linear[k].fu;
            psi.block(9 * k, 0, kNx, kNx) = next_psi;
            gamma.block(9 * k, 0, kNx, nv) = next_gamma;
            prev_psi = std::move(next_psi);
            prev_gamma = std::move(next_gamma);
        }
    }

    Eigen::Matrix<double, kNx, 1> q_diag;
    q_diag << params_.q_position_xy, params_.q_position_xy, params_.q_position_z,
        params_.q_velocity, params_.q_velocity, params_.q_velocity,
        params_.q_attitude_roll_pitch, params_.q_attitude_roll_pitch,
        params_.q_attitude_yaw;
    const Eigen::Matrix<double, kNu, 1> r_diag(
        (Eigen::Matrix<double, kNu, 1>() << params_.r_thrust, params_.r_body_rate_xy,
         params_.r_body_rate_xy, params_.r_body_rate_z)
            .finished());

    // Quadratic cost on the error trajectory, with weights decaying over the
    // horizon so the near-term tracking dominates.
    Eigen::MatrixXd gamma_scaled(kNx * n, nv);
    Eigen::VectorXd psi_dot_dx0(kNx * n);
    Eigen::VectorXd quad_weights(nv);
    for (int k = 0; k < n; ++k) {
        const double decay = std::exp(-params_.weight_decay * static_cast<double>(k) / n);
        for (int j = 0; j < kNx; ++j) {
            const double s = std::sqrt(decay * q_diag(j));
            gamma_scaled.row(9 * k + j) = s * gamma.row(9 * k + j);
            psi_dot_dx0(9 * k + j) = s * psi.row(9 * k + j).dot(dx0);
        }
        for (int j = 0; j < kNu; ++j)
            quad_weights(kNu * k + j) = decay * r_diag(j);
    }

    Eigen::MatrixXd hessian = gamma_scaled.transpose() * gamma_scaled;
    hessian.diagonal() += quad_weights;
    // A tiny ridge keeps the factorisation well conditioned when a weight is 0.
    hessian.diagonal().array() += 1e-9;
    const Eigen::VectorXd gradient = gamma_scaled.transpose() * psi_dot_dx0;

    // Input bounds in error coordinates: delta_u = u_d - u, so the absolute
    // input stays inside [min_thrust, max_thrust] x [-max_rate, max_rate].
    Eigen::VectorXd lower(nv);
    Eigen::VectorXd upper(nv);
    for (int k = 0; k < n; ++k) {
        const double a = plan.thrust_accel[k];
        const Eigen::Vector3d& w = plan.body_rates[k];
        upper(kNu * k + 0) = a - params_.min_thrust_accel;
        lower(kNu * k + 0) = -(params_.max_thrust_accel - a);
        upper(kNu * k + 1) = params_.max_body_rate_xy + w.x();
        lower(kNu * k + 1) = w.x() - params_.max_body_rate_xy;
        upper(kNu * k + 2) = params_.max_body_rate_xy + w.y();
        lower(kNu * k + 2) = w.y() - params_.max_body_rate_xy;
        upper(kNu * k + 3) = params_.max_body_rate_z + w.z();
        lower(kNu * k + 3) = w.z() - params_.max_body_rate_z;
    }

    // ---- OSQP ------------------------------------------------------------
    std::vector<c_int> p_indptr(nv + 1);
    std::vector<c_int> p_indices;
    std::vector<c_float> p_values;
    p_indices.reserve(nv * (nv + 1) / 2);
    p_values.reserve(nv * (nv + 1) / 2);
    p_indptr[0] = 0;
    c_int nz = 0;
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i <= j; ++i) {
            p_indices.push_back(i);
            p_values.push_back(static_cast<c_float>(hessian(i, j)));
            ++nz;
        }
        p_indptr[j + 1] = nz;
    }

    std::vector<c_int> a_indptr(nv + 1);
    std::vector<c_int> a_indices(nv);
    std::vector<c_float> a_values(nv, 1.0);
    std::vector<c_float> q_vec(nv);
    std::vector<c_float> l_vec(nv);
    std::vector<c_float> u_vec(nv);
    for (int j = 0; j < nv; ++j) {
        a_indptr[j] = j;
        a_indices[j] = j;
        q_vec[j] = static_cast<c_float>(gradient(j));
        l_vec[j] = static_cast<c_float>(lower(j));
        u_vec[j] = static_cast<c_float>(upper(j));
    }
    a_indptr[nv] = nv;

    csc* p_matrix = csc_matrix(nv, nv, nz, p_values.data(), p_indices.data(), p_indptr.data());
    csc* a_matrix = csc_matrix(nv, nv, nv, a_values.data(), a_indices.data(), a_indptr.data());

    OSQPData data;
    data.n = nv;
    data.m = nv;
    data.P = p_matrix;
    data.A = a_matrix;
    data.q = q_vec.data();
    data.l = l_vec.data();
    data.u = u_vec.data();

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.max_iter = params_.max_iterations;
    settings.eps_abs = static_cast<c_float>(params_.eps_abs);
    settings.eps_rel = static_cast<c_float>(params_.eps_rel);
    settings.polish = 1;

    OSQPWorkspace* work = nullptr;
    const c_int setup_status = osqp_setup(&work, &data, &settings);
    if (setup_status == 0) {
        osqp_solve(work);
        command.iterations = static_cast<int>(work->info->iter);
        const c_int status = work->info->status_val;
        command.solved = status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE;

        // delta_u = u_d - u, so the command is u_d - delta_u.
        ErrorInput delta_u;
        for (int i = 0; i < kNu; ++i)
            delta_u(i) = static_cast<double>(work->solution->x[i]);

        ErrorInput u_d;
        u_d(0) = plan.thrust_accel[0];
        u_d.segment<3>(1) = plan.body_rates[0];
        const ErrorInput u = u_d - delta_u;

        command.thrust_accel = std::clamp(
            u(0), params_.min_thrust_accel, params_.max_thrust_accel);
        command.body_rates = u.segment<3>(1);
        command.body_rates.x() = std::clamp(
            command.body_rates.x(), -params_.max_body_rate_xy, params_.max_body_rate_xy);
        command.body_rates.y() = std::clamp(
            command.body_rates.y(), -params_.max_body_rate_xy, params_.max_body_rate_xy);
        command.body_rates.z() = std::clamp(
            command.body_rates.z(), -params_.max_body_rate_z, params_.max_body_rate_z);
        command.thrust =
            std::clamp(command.thrust_accel / params_.thrust_to_accel, 0.0, 1.0);
    }

    // osqp_cleanup releases the workspace together with the CSC matrices that
    // were handed to osqp_setup; freeing them here would double free.
    if (work != nullptr)
        osqp_cleanup(work);

    command.solve_time_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
            .count();
    return command;
}

} // namespace flyrat

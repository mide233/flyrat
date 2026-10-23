// SPDX-License-Identifier: MIT
//
// Minimal SO(3) helpers for the on-manifold MPC: hat operator, exponential and
// logarithmic maps and the left Jacobian. Header-only and free of ROS.
#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace flyrat {
namespace so3 {

inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return m;
}

inline Eigen::Matrix3d exp(const Eigen::Vector3d& phi) {
    const double theta = phi.norm();
    const Eigen::Matrix3d k = hat(phi);
    if (theta < 1e-8)
        return Eigen::Matrix3d::Identity() + k;
    const double a = std::sin(theta) / theta;
    const double b = (1.0 - std::cos(theta)) / (theta * theta);
    return Eigen::Matrix3d::Identity() + a * k + b * k * k;
}

inline Eigen::Vector3d log(const Eigen::Matrix3d& r) {
    const double cos_theta = std::clamp((r.trace() - 1.0) * 0.5, -1.0, 1.0);
    const Eigen::Vector3d vee(r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1));
    // acos() loses half the significant digits near the identity. The
    // antisymmetric part carries sin(theta) to full precision, so take the
    // angle from atan2(sin, cos) instead.
    const double sin_theta = (r - r.transpose()).norm() * 0.3535533905932738;
    const double theta = std::atan2(sin_theta, cos_theta);
    if (theta < 1e-6)
        return 0.5 * (1.0 + theta * theta / 6.0) * vee;
    if (M_PI - theta > 1e-6)
        return (theta / (2.0 * sin_theta)) * vee;
    // Near pi the antisymmetric part vanishes; recover the axis from the
    // symmetric part instead. r = I + 2 u u^T - ... at exactly pi.
    Eigen::Matrix3d sym = (r + Eigen::Matrix3d::Identity()) * 0.5;
    int i = 0;
    if (sym(1, 1) > sym(i, i))
        i = 1;
    if (sym(2, 2) > sym(i, i))
        i = 2;
    Eigen::Vector3d axis = Eigen::Vector3d::Zero();
    axis(i) = std::sqrt(std::max(sym(i, i), 0.0));
    if (axis(i) < 1e-8)
        return Eigen::Vector3d::Zero();
    for (int k = 0; k < 3; ++k) {
        if (k != i)
            axis(k) = sym(k, i) / axis(i);
    }
    axis.normalize();
    // The axis sign is ambiguous at exactly pi; pick the one that matches the
    // (tiny) antisymmetric part when it still carries information.
    if (axis.dot(vee) < 0.0)
        axis = -axis;
    return theta * axis;
}

// Left Jacobian: exp(a) exp(b) ~= exp(a + J_l(a) b) to first order.
inline Eigen::Matrix3d leftJacobian(const Eigen::Vector3d& phi) {
    const double theta = phi.norm();
    const Eigen::Matrix3d k = hat(phi);
    if (theta < 1e-8)
        return Eigen::Matrix3d::Identity() + 0.5 * k;
    const double a = (1.0 - std::cos(theta)) / (theta * theta);
    const double b = (theta - std::sin(theta)) / (theta * theta * theta);
    return Eigen::Matrix3d::Identity() + a * k + b * k * k;
}

} // namespace so3
} // namespace flyrat

/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

// Ceres 2.2 removed LocalParameterization.  The upstream VINS pose block is a
// 7-dimensional (position + quaternion) parameterization with a 6-dimensional
// tangent space, which maps directly onto ceres::Manifold.
class PoseLocalParameterization : public ceres::Manifold
{
  public:
    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    bool PlusJacobian(const double *x, double *jacobian) const override;
    bool Minus(const double *y, const double *x, double *y_minus_x) const override;
    bool MinusJacobian(const double *x, double *jacobian) const override;
    int AmbientSize() const override { return 7; }
    int TangentSize() const override { return 6; }
};

/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

class PoseManifold : public ceres::Manifold {
public:
    int AmbientSize() const override { return 7; }  // qw, qx, qy, qz, tx, ty, tz
    int TangentSize() const override { return 6; }  // 3 rotation + 3 translation

    bool Plus(const double* x, const double* delta, double* x_plus_delta) const override;

    bool PlusJacobian(const double* x, double* jacobian) const override;

    bool Minus(const double* y, const double* x, double* y_minus_x) const override;

    bool MinusJacobian(const double* x, double* jacobian) const override;
};

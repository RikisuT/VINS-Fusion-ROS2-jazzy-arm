/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "pose_local_parameterization.h"

bool PoseManifold::Plus(const double* x, const double* delta, double* x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = Utility::deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}

bool PoseManifold::PlusJacobian(const double* /*x*/, double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();
    return true;
}

bool PoseManifold::Minus(const double* y, const double* x, double* y_minus_x_ptr) const {
    Eigen::Map<const Eigen::Quaterniond> q_y(y);
    Eigen::Map<const Eigen::Quaterniond> q_x(x);
    Eigen::Map<const Eigen::Vector3d> t_y(y + 3);
    Eigen::Map<const Eigen::Vector3d> t_x(x + 3);

    Eigen::Quaterniond delta_q = q_y * q_x.conjugate().normalized();
    Eigen::Vector3d delta_t = t_y - t_x;

    // Small angle approximation for rotation
    Eigen::Vector3d delta_rot = 2.0 * delta_q.vec() / delta_q.w();

    // Assign to output (avoid shadowing with different names)
    Eigen::Map<Eigen::Vector3d> delta_rot_out(y_minus_x_ptr);
    delta_rot_out = delta_rot;

    Eigen::Map<Eigen::Vector3d> delta_t_out(y_minus_x_ptr + 3);
    delta_t_out = delta_t;

    return true;
}

bool PoseManifold::MinusJacobian(const double* /*x*/, double* jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
    j.setZero();
    j.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    j.block<3, 3>(3, 4) = -Eigen::Matrix3d::Identity();
    return true;
}

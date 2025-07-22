#include "camodocal/gpl/EigenQuaternionParameterization.h" // Corrected relative path
#include "ceres/manifold.h" // Keep this, though main ceres.h might pull it
#include <ceres/internal/autodiff.h> // <<< ADD THIS LINE for ceres::MatrixRef
#include <cmath>
#include <Eigen/Geometry>
#include <Eigen/Core>

// Define M_PI if not already defined (standard in cmath for some systems, or <numbers> in C++20)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace camodocal
{

// --- REMOVE THE DEFINITION OF EigenQuaternionProduct FROM HERE ---
// It should ONLY be defined in the header due to being a template member function.
// template<typename T>
// void
// EigenQuaternionParameterization::EigenQuaternionProduct(const T z[4], const T w[4], T zw[4]) const
// {
//     // ... (ORIGINAL BODY)
// }


bool EigenQuaternionParameterization::Plus(const double* x,
                                           const double* delta,
                                           double* x_plus_delta) const
{
    // ... (Your previously updated Plus implementation - no changes here)
    Eigen::Map<const Eigen::Vector3d> angular_delta(delta);
    Eigen::Map<const Eigen::Vector4d> q_eigen_array(x);

    Eigen::Quaterniond current_q(q_eigen_array(3),
                                 q_eigen_array(0),
                                 q_eigen_array(1),
                                 q_eigen_array(2));

    const double kSquaredAngleTolerance = 1e-12;
    double norm_delta_squared = angular_delta.squaredNorm();

    if (norm_delta_squared > kSquaredAngleTolerance)
    {
        double norm_delta = std::sqrt(norm_delta_squared);
        Eigen::Quaterniond delta_q;
        delta_q.vec() = angular_delta * std::sin(norm_delta / 2.0) / norm_delta;
        delta_q.w() = std::cos(norm_delta / 2.0);

        Eigen::Quaterniond new_q = delta_q * current_q;
        new_q.normalize();

        x_plus_delta[0] = new_q.x();
        x_plus_delta[1] = new_q.y();
        x_plus_delta[2] = new_q.z();
        x_plus_delta[3] = new_q.w();
    }
    else
    {
        for (int i = 0; i < 4; ++i)
        {
            x_plus_delta[i] = x[i];
        }
    }
    return true;
}


bool EigenQuaternionParameterization::Minus(const double* x,
                                            const double* y,
                                            double* minus) const
{
    // ... (Your previously updated Minus implementation - no changes here)
    Eigen::Map<const Eigen::Vector4d> x_eigen_array(x);
    Eigen::Map<const Eigen::Vector4d> y_eigen_array(y);

    Eigen::Quaterniond q_x(x_eigen_array(3), x_eigen_array(0), x_eigen_array(1), x_eigen_array(2));
    Eigen::Quaterniond q_y(y_eigen_array(3), y_eigen_array(0), y_eigen_array(1), y_eigen_array(2));

    Eigen::Quaterniond delta_q = q_y * q_x.inverse();
    delta_q.normalize();

    double half_angle_norm = delta_q.vec().norm();
    double angle = 2.0 * std::atan2(half_angle_norm, delta_q.w());

    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }

    if (std::abs(angle) < 1e-9) {
        minus[0] = 0.0;
        minus[1] = 0.0;
        minus[2] = 0.0;
    } else {
        Eigen::Vector3d axis = delta_q.vec().normalized();
        minus[0] = angle * axis.x();
        minus[1] = angle * axis.y();
        minus[2] = angle * axis.z();
    }
    return true;
}


bool EigenQuaternionParameterization::PlusJacobian(const double* x,
                                                   double* jacobian) const
{
    // NO CHANGE NEEDED HERE, just ensure ceres::MatrixRef is now available
    ceres::MatrixRef J(jacobian, 4, 3);

    const double qx = x[0];
    const double qy = x[1];
    const double qz = x[2];
    const double qw = x[3];

    J(0, 0) = qw;  J(0, 1) = -qz; J(0, 2) = qy;
    J(1, 0) = qz;  J(1, 1) = qw;  J(1, 2) = -qx;
    J(2, 0) = -qy; J(2, 1) = qx;  J(2, 2) = qw;
    J(3, 0) = -qx; J(3, 1) = -qy; J(3, 2) = -qz;

    J *= 0.5;

    return true;
}


bool EigenQuaternionParameterization::MinusJacobian(const double* x,
                                                    double* jacobian) const
{
    // NO CHANGE NEEDED HERE, just ensure ceres::MatrixRef is now available
    ceres::MatrixRef J(jacobian, 3, 4);

    const double qx = x[0];
    const double qy = x[1];
    const double qz = x[2];
    const double qw = x[3];

    const double norm_squared = qx*qx + qy*qy + qz*qz + qw*qw;
    const double scale = 2.0 / norm_squared;

    J(0, 0) = -qw; J(0, 1) = qz;  J(0, 2) = -qy; J(0, 3) = qx;
    J(1, 0) = -qz; J(1, 1) = -qw; J(1, 2) = qx;  J(1, 3) = qy;
    J(2, 0) = qy;  J(2, 1) = -qx; J(2, 2) = -qw; J(2, 3) = qz;

    J *= scale;

    return true;
}

} // namespace camodocal
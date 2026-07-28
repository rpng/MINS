// Numerically verifies the time-offset Jacobian (ComputeTimeOffsetJacobian2D/3D) via
// a combined central finite difference: both clone poses are shifted simultaneously
// by (exp_so3(w*eps)*R, p - v*eps), which models increasing dt by eps (backward shift).
#include <gtest/gtest.h>
#include <Eigen/Core>
#include "update/wheel/UpdaterWheel.h"
#include "utils/quat_ops.h"

using namespace mins;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

TEST(WheelTimeOffset2D, AnalyticalMatchesNumerical) {
    const double eps = 1e-6;
    const double tol = 1e-6;

    Matrix3d R_GtoI0 = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.15));
    Vector3d p_I0inG(1.0, 0.5, 0.0);
    Matrix3d R_GtoI1 = ov_core::exp_so3(Vector3d(0.05, -0.15, 0.3));
    Vector3d p_I1inG(2.0, 1.0, 0.0);
    Matrix3d R_ItoO = ov_core::exp_so3(Vector3d(0.0, 0.0, 0.2));
    Vector3d p_IinO(0.1, 0.0, -0.2);

    // Arbitrary velocities: w in body frame, v in global frame.
    Vector3d w0(0.01, 0.02, 0.1), v0(0.5, 0.1, 0.0);
    Vector3d w1(0.02, -0.01, 0.05), v1(0.6, -0.1, 0.0);

    // Zero-residual nominal 2D measurement.
    Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
    Vector3d e3(0, 0, 1);
    Eigen::Matrix<double, 2, 3> Lambda = Eigen::Matrix<double, 2, 3>::Zero();
    Lambda.block(0, 0, 2, 2) = Eigen::Matrix2d::Identity();
    Matrix3d RO0toO1 = R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose();
    double th = e3.dot(ov_core::log_so3(RO0toO1));
    Eigen::Vector2d d_est = Lambda * R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG - R_GtoI0.transpose() * pOinI);
    double x = d_est(0), y = d_est(1);

    const auto [H_poses, H_ext] = UpdaterWheel::ComputeJacobians2D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO);
    Vector3d H_dt = UpdaterWheel::ComputeTimeOffsetJacobian2D(H_poses, w0, v0, w1, v1);

    // Combined FD: shift both poses backward in time by eps (models dt += eps).
    // R(t-eps) ~ exp_so3(w*eps)*R,  p(t-eps) ~ p - v*eps.
    Matrix3d R0p = ov_core::exp_so3(w0 * eps) * R_GtoI0;
    Vector3d p0p = p_I0inG - v0 * eps;
    Matrix3d R1p = ov_core::exp_so3(w1 * eps) * R_GtoI1;
    Vector3d p1p = p_I1inG - v1 * eps;

    Matrix3d R0m = ov_core::exp_so3(-w0 * eps) * R_GtoI0;
    Vector3d p0m = p_I0inG + v0 * eps;
    Matrix3d R1m = ov_core::exp_so3(-w1 * eps) * R_GtoI1;
    Vector3d p1m = p_I1inG + v1 * eps;

    Vector3d res_plus = UpdaterWheel::ComputeResidual2D(R0p, p0p, R1p, p1p, R_ItoO, p_IinO, th, x, y);
    Vector3d res_minus = UpdaterWheel::ComputeResidual2D(R0m, p0m, R1m, p1m, R_ItoO, p_IinO, th, x, y);
    Vector3d H_dt_numerical = (res_plus - res_minus) / (2.0 * eps);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(H_dt(i), H_dt_numerical(i), tol)
            << "2D H_dt mismatch at row=" << i;
        if (std::abs(H_dt_numerical(i)) > tol) {
            EXPECT_GT(H_dt(i) * H_dt_numerical(i), 0.0)
                << "2D H_dt sign mismatch at row=" << i;
        }
    }
}

TEST(WheelTimeOffset3D, AnalyticalMatchesNumerical) {
    const double eps = 1e-6;
    const double tol = 1e-6;

    Matrix3d R_GtoI0 = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.15));
    Vector3d p_I0inG(1.0, 0.5, 0.0);
    Matrix3d R_GtoI1 = ov_core::exp_so3(Vector3d(0.05, -0.15, 0.3));
    Vector3d p_I1inG(2.0, 1.0, 0.0);
    Matrix3d R_ItoO = ov_core::exp_so3(Vector3d(0.0, 0.0, 0.2));
    Vector3d p_IinO(0.1, 0.0, -0.2);

    Vector3d w0(0.01, 0.02, 0.1), v0(0.5, 0.1, 0.0);
    Vector3d w1(0.02, -0.01, 0.05), v1(0.6, -0.1, 0.0);

    // Zero-residual nominal 3D measurement.
    Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
    Matrix3d R_3D = R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose();
    Vector3d p_3D = R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG - R_GtoI0.transpose() * pOinI);

    const auto [H_poses, H_ext] = UpdaterWheel::ComputeJacobians3D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO);
    Eigen::Matrix<double, 6, 1> H_dt = UpdaterWheel::ComputeTimeOffsetJacobian3D(H_poses, w0, v0, w1, v1);

    Matrix3d R0p = ov_core::exp_so3(w0 * eps) * R_GtoI0;
    Vector3d p0p = p_I0inG - v0 * eps;
    Matrix3d R1p = ov_core::exp_so3(w1 * eps) * R_GtoI1;
    Vector3d p1p = p_I1inG - v1 * eps;

    Matrix3d R0m = ov_core::exp_so3(-w0 * eps) * R_GtoI0;
    Vector3d p0m = p_I0inG + v0 * eps;
    Matrix3d R1m = ov_core::exp_so3(-w1 * eps) * R_GtoI1;
    Vector3d p1m = p_I1inG + v1 * eps;

    Eigen::Matrix<double, 6, 1> res_plus = UpdaterWheel::ComputeResidual3D(R0p, p0p, R1p, p1p, R_ItoO, p_IinO, R_3D, p_3D);
    Eigen::Matrix<double, 6, 1> res_minus = UpdaterWheel::ComputeResidual3D(R0m, p0m, R1m, p1m, R_ItoO, p_IinO, R_3D, p_3D);
    Eigen::Matrix<double, 6, 1> H_dt_numerical = (res_plus - res_minus) / (2.0 * eps);

    for (int i = 0; i < 6; i++) {
        EXPECT_NEAR(H_dt(i), H_dt_numerical(i), tol)
            << "3D H_dt mismatch at row=" << i;
        if (std::abs(H_dt_numerical(i)) > tol) {
            EXPECT_GT(H_dt(i) * H_dt_numerical(i), 0.0)
                << "3D H_dt sign mismatch at row=" << i;
        }
    }
}

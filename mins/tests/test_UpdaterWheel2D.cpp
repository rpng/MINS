// Numerically verifies UpdaterWheel's 2D analytical Jacobians using central finite differences.
// Perturbs each state dimension, builds H_numerical, and compares to wheel::ComputeJacobians2D.
#include <gtest/gtest.h>
#include <Eigen/Core>
#include "update/wheel/WheelJacobians.h"
#include "utils/quat_ops.h"

using namespace mins;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

TEST(WheelJacobian2D, AnalyticalMatchesNumerical) {
    // Rotation cols: sign = +1 (residual changes directly with rotation error-state).
    // Position cols: sign = -1 (res = meas - h(state), so h increases means res decreases).
    const double epsilon = 1e-6;
    const double tolerance = 1e-6;

    Matrix3d R_GtoI0 = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.15));
    Vector3d p_I0inG(1.0, 0.5, 0.0);
    Matrix3d R_GtoI1 = ov_core::exp_so3(Vector3d(0.05, -0.15, 0.3));
    Vector3d p_I1inG(2.0, 1.0, 0.0);
    Matrix3d R_ItoO = ov_core::exp_so3(Vector3d(0.0, 0.0, 0.2));
    Vector3d p_IinO(0.1, 0.0, -0.2);

    // Zero-residual nominal measurement.
    Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
    Eigen::Matrix<double, 2, 3> Lambda = Eigen::Matrix<double, 2, 3>::Zero();
    Lambda.block(0, 0, 2, 2) = Eigen::Matrix2d::Identity();
    Vector3d e3(0, 0, 1);
    double th = (e3.transpose() * ov_core::log_so3(R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose()))(0);
    Eigen::Vector2d d_est = Lambda * R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG - R_GtoI0.transpose() * pOinI);
    double x = d_est(0);
    double y = d_est(1);

    const auto [H_poses, H_ext] = wheel::ComputeJacobians2D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO);

    // --- H_poses: perturb pose0 (cols 0-5) and pose1 (cols 6-11) ---
    MatrixXd H_poses_numerical = MatrixXd::Zero(3, 12);
    for (int i = 0; i < 6; i++) {
        // pose0
        Matrix3d R0_plus = R_GtoI0, R0_minus = R_GtoI0;
        Vector3d p0_plus = p_I0inG, p0_minus = p_I0inG;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R0_plus = ov_core::exp_so3(epsilon * axis) * R_GtoI0;
            R0_minus = ov_core::exp_so3(-epsilon * axis) * R_GtoI0;
        } else {
            p0_plus(i - 3) += epsilon;
            p0_minus(i - 3) -= epsilon;
        }
        Vector3d res_plus = wheel::ComputeResidual2D(R0_plus, p0_plus, R_GtoI1, p_I1inG, R_ItoO, p_IinO, th, x, y);
        Vector3d res_minus = wheel::ComputeResidual2D(R0_minus, p0_minus, R_GtoI1, p_I1inG, R_ItoO, p_IinO, th, x, y);
        double sign = (i < 3) ? 1.0 : -1.0;
        H_poses_numerical.col(i) = sign * (res_plus - res_minus) / (2.0 * epsilon);

        // pose1
        Matrix3d R1_plus = R_GtoI1, R1_minus = R_GtoI1;
        Vector3d p1_plus = p_I1inG, p1_minus = p_I1inG;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R1_plus = ov_core::exp_so3(epsilon * axis) * R_GtoI1;
            R1_minus = ov_core::exp_so3(-epsilon * axis) * R_GtoI1;
        } else {
            p1_plus(i - 3) += epsilon;
            p1_minus(i - 3) -= epsilon;
        }
        res_plus = wheel::ComputeResidual2D(R_GtoI0, p_I0inG, R1_plus, p1_plus, R_ItoO, p_IinO, th, x, y);
        res_minus = wheel::ComputeResidual2D(R_GtoI0, p_I0inG, R1_minus, p1_minus, R_ItoO, p_IinO, th, x, y);
        H_poses_numerical.col(6 + i) = sign * (res_plus - res_minus) / (2.0 * epsilon);
    }

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 12; col++) {
            EXPECT_NEAR(H_poses(row, col), H_poses_numerical(row, col), tolerance)
                << "H_poses mismatch at row=" << row << " col=" << col;
            if (std::abs(H_poses_numerical(row, col)) > tolerance) {
                EXPECT_GT(H_poses(row, col) * H_poses_numerical(row, col), 0.0)
                    << "H_poses sign mismatch at row=" << row << " col=" << col;
            }
        }
    }

    // --- H_ext: perturb extrinsic (R_ItoO, p_IinO) ---
    MatrixXd H_ext_numerical = MatrixXd::Zero(3, 6);
    for (int i = 0; i < 6; i++) {
        Matrix3d R_plus = R_ItoO, R_minus = R_ItoO;
        Vector3d p_plus = p_IinO, p_minus = p_IinO;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R_plus = ov_core::exp_so3(epsilon * axis) * R_ItoO;
            R_minus = ov_core::exp_so3(-epsilon * axis) * R_ItoO;
        } else {
            p_plus(i - 3) += epsilon;
            p_minus(i - 3) -= epsilon;
        }
        Vector3d res_plus = wheel::ComputeResidual2D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_plus, p_plus, th, x, y);
        Vector3d res_minus = wheel::ComputeResidual2D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_minus, p_minus, th, x, y);
        double sign = (i < 3) ? 1.0 : -1.0;
        H_ext_numerical.col(i) = sign * (res_plus - res_minus) / (2.0 * epsilon);
    }

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 6; col++) {
            EXPECT_NEAR(H_ext(row, col), H_ext_numerical(row, col), tolerance)
                << "H_ext mismatch at row=" << row << " col=" << col;
            if (std::abs(H_ext_numerical(row, col)) > tolerance) {
                EXPECT_GT(H_ext(row, col) * H_ext_numerical(row, col), 0.0)
                    << "H_ext sign mismatch at row=" << row << " col=" << col;
            }
        }
    }
}

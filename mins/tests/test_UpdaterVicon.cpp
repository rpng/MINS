// Numerically verifies UpdaterVicon's analytical Jacobians using central finite differences.
// Perturbs each state dimension, collects residuals via UpdaterVicon::ComputeResidual,
// builds H_numerical, and compares to UpdaterVicon::ComputeJacobians.
#include <gtest/gtest.h>
#include <Eigen/Core>
#include "update/vicon/UpdaterVicon.h"
#include "update/vicon/ViconTypes.h"
#include "utils/quat_ops.h"

using namespace mins;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

TEST(ViconJacobian, AnalyticalMatchesNumerical) {
    // Rotation columns (0-2): H = d(residual)/d(delta_theta) — same sign as residual difference.
    // Position columns (3-5): H = d(z_predicted)/d(delta_p) — opposite sign (r = z - h(x)).
    // The sign multiplier below reflects this mixed convention.
    const double epsilon = 1e-6;
    const double tolerance = 1e-6;

    Matrix3d R_GtoI = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.15));
    Vector3d p_IinG(1.0, -0.5, 0.3);
    Matrix3d R_ItoX = ov_core::exp_so3(Vector3d(-0.05, 0.1, 0.08));
    Vector3d p_IinX(0.1, 0.0, -0.2);

    // Zero-residual nominal measurement.
    Eigen::Matrix<double, 6, 1> z = Eigen::Matrix<double, 6, 1>::Zero();
    z.head(3) = ov_core::log_so3(R_ItoX * R_GtoI);
    z.tail(3) = p_IinG + R_GtoI.transpose() * (-R_ItoX.transpose() * p_IinX);

    // Analytical Jacobians from UpdaterVicon.
    const auto [dz_dI, dz_dcalib] = UpdaterVicon::ComputeJacobians(R_GtoI, R_ItoX, p_IinX);

    // --- dz_dI: perturb IMU pose ---
    MatrixXd dz_dI_numerical = MatrixXd::Zero(6, 6);
    for (int i = 0; i < 6; i++) {
        Matrix3d R_plus = R_GtoI;
        Matrix3d R_minus = R_GtoI;
        Vector3d p_plus = p_IinG;
        Vector3d p_minus = p_IinG;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R_plus = ov_core::exp_so3(epsilon * axis) * R_GtoI;
            R_minus = ov_core::exp_so3(-epsilon * axis) * R_GtoI;
        } else {
            p_plus(i - 3) += epsilon;
            p_minus(i - 3) -= epsilon;
        }
        Eigen::Matrix<double, 6, 1> res_plus =
            UpdaterVicon::ComputeResidual(R_plus, p_plus, R_ItoX, p_IinX, z);
        Eigen::Matrix<double, 6, 1> res_minus =
            UpdaterVicon::ComputeResidual(R_minus, p_minus, R_ItoX, p_IinX, z);
        // Rotation cols: H = d(r)/d(delta_theta); position cols: H = -d(r)/d(delta_p).
        double sign = (i < 3) ? 1.0 : -1.0;
        dz_dI_numerical.col(i) = sign * (res_plus - res_minus) / (2.0 * epsilon);
    }

    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 6; col++) {
            EXPECT_NEAR(dz_dI(row, col), dz_dI_numerical(row, col), tolerance)
                << "dz_dI mismatch at row=" << row << " col=" << col;
        }
    }

    // --- dz_dcalib: perturb Vicon extrinsic calibration ---
    MatrixXd dz_dcalib_numerical = MatrixXd::Zero(6, 6);
    for (int i = 0; i < 6; i++) {
        Matrix3d R_plus = R_ItoX;
        Matrix3d R_minus = R_ItoX;
        Vector3d p_plus = p_IinX;
        Vector3d p_minus = p_IinX;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R_plus = ov_core::exp_so3(epsilon * axis) * R_ItoX;
            R_minus = ov_core::exp_so3(-epsilon * axis) * R_ItoX;
        } else {
            p_plus(i - 3) += epsilon;
            p_minus(i - 3) -= epsilon;
        }
        Eigen::Matrix<double, 6, 1> res_plus =
            UpdaterVicon::ComputeResidual(R_GtoI, p_IinG, R_plus, p_plus, z);
        Eigen::Matrix<double, 6, 1> res_minus =
            UpdaterVicon::ComputeResidual(R_GtoI, p_IinG, R_minus, p_minus, z);
        double sign = (i < 3) ? 1.0 : -1.0;
        dz_dcalib_numerical.col(i) = sign * (res_plus - res_minus) / (2.0 * epsilon);
    }

    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 6; col++) {
            EXPECT_NEAR(dz_dcalib(row, col), dz_dcalib_numerical(row, col), tolerance)
                << "dz_dcalib mismatch at row=" << row << " col=" << col;
        }
    }
}

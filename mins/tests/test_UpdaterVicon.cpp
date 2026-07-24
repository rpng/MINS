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

    Matrix3d rotation_global_to_imu = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.15));
    Vector3d position_imu_in_global(1.0, -0.5, 0.3);
    Matrix3d rotation_imu_to_vicon = ov_core::exp_so3(Vector3d(-0.05, 0.1, 0.08));
    Vector3d position_imu_in_vicon(0.1, 0.0, -0.2);

    // Zero-residual nominal measurement.
    Eigen::Matrix<double, 6, 1> measurement_pose = Eigen::Matrix<double, 6, 1>::Zero();
    measurement_pose.head(3) = ov_core::log_so3(rotation_imu_to_vicon * rotation_global_to_imu);
    measurement_pose.tail(3) = position_imu_in_global +
                               rotation_global_to_imu.transpose() *
                               (-rotation_imu_to_vicon.transpose() * position_imu_in_vicon);

    // Analytical Jacobians from UpdaterVicon.
    MatrixXd dz_dI, dz_dcalib;
    UpdaterVicon::ComputeJacobians(rotation_global_to_imu, rotation_imu_to_vicon,
                                   position_imu_in_vicon, dz_dI, dz_dcalib);

    // --- dz_dI: perturb IMU pose ---
    MatrixXd dz_dI_numerical = MatrixXd::Zero(6, 6);
    for (int i = 0; i < 6; i++) {
        Matrix3d R_plus = rotation_global_to_imu;
        Matrix3d R_minus = rotation_global_to_imu;
        Vector3d p_plus = position_imu_in_global;
        Vector3d p_minus = position_imu_in_global;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R_plus = ov_core::exp_so3(epsilon * axis) * rotation_global_to_imu;
            R_minus = ov_core::exp_so3(-epsilon * axis) * rotation_global_to_imu;
        } else {
            p_plus(i - 3) += epsilon;
            p_minus(i - 3) -= epsilon;
        }
        Eigen::Matrix<double, 6, 1> res_plus =
            UpdaterVicon::ComputeResidual(R_plus, p_plus, rotation_imu_to_vicon,
                                          position_imu_in_vicon, measurement_pose);
        Eigen::Matrix<double, 6, 1> res_minus =
            UpdaterVicon::ComputeResidual(R_minus, p_minus, rotation_imu_to_vicon,
                                          position_imu_in_vicon, measurement_pose);
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
        Matrix3d R_plus = rotation_imu_to_vicon;
        Matrix3d R_minus = rotation_imu_to_vicon;
        Vector3d p_plus = position_imu_in_vicon;
        Vector3d p_minus = position_imu_in_vicon;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R_plus = ov_core::exp_so3(epsilon * axis) * rotation_imu_to_vicon;
            R_minus = ov_core::exp_so3(-epsilon * axis) * rotation_imu_to_vicon;
        } else {
            p_plus(i - 3) += epsilon;
            p_minus(i - 3) -= epsilon;
        }
        Eigen::Matrix<double, 6, 1> res_plus =
            UpdaterVicon::ComputeResidual(rotation_global_to_imu, position_imu_in_global,
                                          R_plus, p_plus, measurement_pose);
        Eigen::Matrix<double, 6, 1> res_minus =
            UpdaterVicon::ComputeResidual(rotation_global_to_imu, position_imu_in_global,
                                          R_minus, p_minus, measurement_pose);
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

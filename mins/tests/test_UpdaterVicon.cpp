/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Unit tests for mins/src/update/vicon/UpdaterVicon. The first numerically verifies the
// analytical Jacobians with central finite differences of UpdaterVicon::ComputeResidual; the
// rest drive the measurement stack and the EKF update on a state holding clones.
#include <gtest/gtest.h>
#include <Eigen/Core>
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsLidar.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "update/vicon/UpdaterVicon.h"
#include "update/vicon/ViconTypes.h"
#include "utils/opencv_yaml_parse.h"
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
            if (std::abs(dz_dI_numerical(row, col)) > tolerance) {
                EXPECT_GT(dz_dI(row, col) * dz_dI_numerical(row, col), 0.0)
                    << "dz_dI sign mismatch at row=" << row << " col=" << col;
            }
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
            if (std::abs(dz_dcalib_numerical(row, col)) > tolerance) {
                EXPECT_GT(dz_dcalib(row, col) * dz_dcalib_numerical(row, col), 0.0)
                    << "dz_dcalib sign mismatch at row=" << row << " col=" << col;
            }
        }
    }
}

namespace {
struct UpdaterViconAccess : UpdaterVicon {
    using UpdaterVicon::UpdaterVicon;
    using UpdaterVicon::data_stack;
    using UpdaterVicon::update;
};

/// Vicon extrinsics, rotated and offset on every axis so a dropped term shows.
const Matrix3d R_I_TO_X = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.3));
const Vector3d P_I_IN_X(0.2, -0.1, 0.4);

/// Clone times of the test trajectory. Five clones give the cubic interpolation two polynomials.
const std::vector<double> CLONE_TIMES = {0.0, 0.1, 0.2, 0.3, 0.4};

/// The IMU position along the test trajectory. It is quadratic, so the cubic clone
/// interpolation reproduces it exactly between clones. The IMU never rotates.
Vector3d TrajectoryPosition(double time) { return Vector3d(2.0 * time, time * time, 0.1 * time); }

/// Vicon-only state with one sensor and a clone at each of CLONE_TIMES. The options come from
/// the simulation config, since the interpolation error table the update reads is only parsed.
std::shared_ptr<State> MakeCloneState(bool do_calib_dt = false, bool do_calib_ext = false) {
    std::shared_ptr<OptionsEstimator> op = std::make_shared<OptionsEstimator>();
    op->load(std::make_shared<ov_core::YamlParser>(MINS_CONFIG_DIR "/simulation/config.yaml"));
    op->cam->enabled = op->gps->enabled = op->wheel->enabled = op->lidar->enabled = false;
    op->vicon->enabled = true;
    // Interpolate between clones with the polynomial; the IMU residual needs a propagated IMU.
    op->use_imu_res = false;
    op->vicon->max_n = 1;
    op->vicon->dt[0] = 0.0;
    Eigen::Matrix<double, 7, 1> extrinsic;
    extrinsic << ov_core::rot_2_quat(R_I_TO_X), P_I_IN_X;
    op->vicon->extrinsics[0] = extrinsic;
    op->vicon->do_calib_dt = do_calib_dt;
    op->vicon->do_calib_ext = do_calib_ext;

    std::shared_ptr<State> state = std::make_shared<State>(op);
    for (double time : CLONE_TIMES) {
        Eigen::Matrix<double, 16, 1> imu_value = Eigen::Matrix<double, 16, 1>::Zero();
        imu_value(3) = 1;  // JPL identity quaternion
        imu_value.block(4, 0, 3, 1) = TrajectoryPosition(time);
        state->imu->set_value(imu_value);
        state->imu->set_fej(imu_value);
        state->time = time;
        StateHelper::augment_clone(state);
    }
    return state;
}

/// The noise-free Vicon pose at the given time.
ViconData PoseAt(double time) {
    ViconData data;
    data.time = time;
    data.id = 0;
    data.pose.head(3) = ov_core::log_so3(R_I_TO_X);
    data.pose.tail(3) = TrajectoryPosition(time) - R_I_TO_X.transpose() * P_I_IN_X;
    return data;
}

}  // namespace

TEST(ViconFeedMeasurement, KeepsEveryReadingAndCapsTheTimeHistory) {
    UpdaterViconAccess updater(MakeCloneState());
    for (int i = 0; i < 150; i++) {
        updater.feed_measurement(PoseAt(0.001 * i));
    }
    ASSERT_EQ(updater.data_stack.size(), 150u);
    EXPECT_EQ(updater.data_stack.back().time, 0.149);
    // The cap pops before it pushes, so it holds one more than the 100 it checks for.
    EXPECT_EQ(updater.t_hist.at(0).size(), 101u);
    EXPECT_EQ(updater.t_hist.at(0).back(), 0.149);
}

TEST(ViconUpdate, AConsistentPoseIsAcceptedAndShrinksTheCovariance) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterViconAccess updater(state);
    const double trace_before = state->cov.trace();
    EXPECT_TRUE(updater.update(PoseAt(0.12)));
    EXPECT_LT(state->cov.trace(), trace_before);
}

TEST(ViconUpdate, PullsTheClonesTowardAnOffsetPose) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterViconAccess updater(state);
    ViconData data = PoseAt(0.1);
    data.pose(3) += 0.005;
    data.pose.head(3) = ov_core::log_so3(R_I_TO_X * ov_core::rot_z(0.005));
    const double x_before = state->clones.at(0.1)->pos()(0);
    EXPECT_TRUE(updater.update(data));
    EXPECT_GT(state->clones.at(0.1)->pos()(0), x_before + 1e-4);
    // The measurement says R_GtoI = rot_z(0.005), so the clone yaw moves that way.
    EXPECT_GT(ov_core::log_so3(state->clones.at(0.1)->Rot())(2), 1e-4);
}

TEST(ViconUpdate, AnInconsistentPoseIsConsumedButLeavesTheStateAlone) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterViconAccess updater(state);
    ViconData data = PoseAt(0.12);
    data.pose.tail(3) += Vector3d(10.0, -10.0, 10.0);
    const MatrixXd cov_before = state->cov;
    const Eigen::Matrix<double, 7, 1> pose_before = state->clones.at(0.1)->value();
    // The chi2 gate rejects it, yet update reports it handled so try_update drops it.
    EXPECT_TRUE(updater.update(data));
    EXPECT_TRUE(state->cov.isApprox(cov_before));
    EXPECT_TRUE(state->clones.at(0.1)->value().isApprox(pose_before));
}

TEST(ViconUpdate, APoseOutsideTheClonesIsNotUsed) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterViconAccess updater(state);
    const MatrixXd cov_before = state->cov;
    EXPECT_FALSE(updater.update(PoseAt(5.0)));
    EXPECT_TRUE(state->cov.isApprox(cov_before));
}

TEST(ViconUpdate, CalibratesTheExtrinsicAndTimeOffsetToo) {
    std::shared_ptr<State> state = MakeCloneState(true, true);
    UpdaterViconAccess updater(state);
    const int ext = state->vicon_extrinsic.at(0)->id();
    const int dt = state->vicon_dt.at(0)->id();
    const double ext_var_before = state->cov.block(ext, ext, 6, 6).trace();
    const double dt_var_before = state->cov(dt, dt);
    EXPECT_TRUE(updater.update(PoseAt(0.12)));
    EXPECT_LT(state->cov.block(ext, ext, 6, 6).trace(), ext_var_before);
    EXPECT_LT(state->cov(dt, dt), dt_var_before);
}

TEST(ViconTryUpdate, UsesACoveredPoseDropsAStaleOneAndKeepsOneNotYetCovered) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterViconAccess updater(state);
    updater.feed_measurement(PoseAt(-1.0));
    updater.feed_measurement(PoseAt(0.12));
    updater.feed_measurement(PoseAt(5.0));
    const double trace_before = state->cov.trace();
    updater.try_update();
    ASSERT_EQ(updater.data_stack.size(), 1u);
    EXPECT_EQ(updater.data_stack.front().time, 5.0);
    EXPECT_LT(state->cov.trace(), trace_before);
}

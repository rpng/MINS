/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Checks the covariance that UpdaterGPS::transform_state_to_ENU hands over to the ENU frame.
#include <gtest/gtest.h>
#include <Eigen/Core>
#include "options/OptionsEstimator.h"
#include "options/OptionsCamera.h"
#include "options/OptionsGPS.h"
#include "options/OptionsLidar.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "update/gps/GPSTypes.h"
#include "update/gps/PoseJPL_4DOF.h"
#include "update/gps/UpdaterGPS.h"
#include "utils/quat_ops.h"

using namespace mins;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace {
struct UpdaterGPSAccess : UpdaterGPS {
    using UpdaterGPS::UpdaterGPS;
    using UpdaterGPS::transform_state_to_ENU;
};

// Appends a variable with diagonal covariance sigma^2 and no cross terms to the rest of the state.
void add_variable(const std::shared_ptr<State>& state, const std::shared_ptr<ov_type::Type>& var, const Eigen::VectorXd& sigma) {
    int n = var->size();
    MatrixXd H_L = sigma.cwiseInverse().asDiagonal();
    MatrixXd H_R = MatrixXd::Zero(n, state->imu->size());
    ASSERT_TRUE(StateHelper::initialize_invertible(state, var, {state->imu}, H_R, H_L, MatrixXd::Identity(n, n), Eigen::VectorXd::Zero(n),
                                                   "test", false));
}
}  // namespace

TEST(GPSTransformToENU, ClonePositionCovarianceIsRotated) {
    auto op = std::make_shared<OptionsEstimator>();
    op->load(nullptr);
    op->cam->enabled = op->vicon->enabled = op->wheel->enabled = op->lidar->enabled = false;
    op->gps->enabled = true;  // State only creates trans_WtoE with GPS on
    op->gps->max_n = 0;      // no GPS calibration states needed
    op->gps->init_cov_inflation = 1.0;
    auto state = std::make_shared<State>(op);

    // IMU pose covariance different from the clone one so any mix-up shows
    state->cov.setZero();
    state->cov.block(0, 0, 3, 3) = 0.01 * Matrix3d::Identity();
    state->cov.block(3, 3, 3, 3) = Vector3d(4.0, 1.0, 9.0).asDiagonal();

    auto clone = std::make_shared<ov_type::PoseJPL>();
    Eigen::Matrix<double, 7, 1> pose;
    pose << 0, 0, 0, 1, 10.0, -5.0, 2.0;
    clone->set_value(pose);
    clone->set_fej(pose);
    Eigen::VectorXd sigma_clone(6);
    sigma_clone << 0.1, 0.1, 0.1, 1.5, 0.7, 0.3;
    add_variable(state, clone, sigma_clone);
    Matrix3d P_clone_pos = sigma_clone.tail(3).cwiseAbs2().asDiagonal();
    state->clones.insert({1.0, clone});

    // WtoE is known exactly, so the new clone position covariance must be R * P * R^T
    Eigen::Matrix<double, 7, 1> wtoe;
    wtoe.head(4) = ov_core::rot_2_quat(ov_core::rot_z(0.7));
    wtoe.tail(3) << 100.0, 200.0, 5.0;
    state->trans_WtoE->set_value(wtoe);
    state->trans_WtoE->set_fej(wtoe);
    Matrix3d RWtoE = state->trans_WtoE->Rot();
    add_variable(state, state->trans_WtoE, Eigen::VectorXd::Constant(4, 1e-9));

    UpdaterGPSAccess updater(state);
    updater.transform_state_to_ENU();

    int c = clone->id() + 3;
    Matrix3d expected = RWtoE * P_clone_pos * RWtoE.transpose();
    EXPECT_TRUE(state->cov.block(c, c, 3, 3).isApprox(expected, 1e-9)) << state->cov.block(c, c, 3, 3);
    EXPECT_TRUE(state->cov.block(c, 3, 3, 3).isZero(1e-9)) << "clone and IMU positions got coupled";
}

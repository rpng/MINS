/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Unit tests for mins/src/update/gps/UpdaterGPS: its measurement stack, keyframe selection,
// world to ENU initialization, the covariance handed over to the ENU frame, and the EKF update.
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
#include "types/Vec.h"
#include "update/gps/GPSTypes.h"
#include "update/gps/PoseJPL_4DOF.h"
#include "update/gps/UpdaterGPS.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/quat_ops.h"

using namespace mins;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace {
struct UpdaterGPSAccess : UpdaterGPS {
    using UpdaterGPS::UpdaterGPS;
    using UpdaterGPS::transform_state_to_ENU;
    using UpdaterGPS::construct_init_linsys;
    using UpdaterGPS::data_stack;
    using UpdaterGPS::get_initial_guess;
    using UpdaterGPS::try_initialization;
    using UpdaterGPS::update;
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

namespace {

/// Where the GPS antenna sits in the IMU frame. Off every axis so a dropped lever arm shows.
const Vector3d P_GPS_IN_I(0.3, -0.2, 0.5);

/// Clone times of the test trajectory. Five clones give the cubic interpolation two polynomials.
const std::vector<double> CLONE_TIMES = {0.0, 0.1, 0.2, 0.3, 0.4};

/// Readings strictly between clones, plus one on a clone, all inside the interpolation span.
const std::vector<double> FIX_TIMES = {0.12, 0.2, 0.33};

/// Measurement noise of every fix, m.
const double FIX_NOISE = 0.1;

/// The world to ENU transform the initialization tests should recover.
const double YAW_WTOE = 0.7;
const Vector3d P_W_IN_E(100.0, 200.0, 5.0);

/// The IMU position along the test trajectory. It curves, so a yaw is observable from it, and
/// it is quadratic, so the cubic clone interpolation reproduces it exactly between clones.
Vector3d TrajectoryPosition(double time) { return Vector3d(2.0 * time, time * time, 0.1 * time); }

/// Options for a GPS-only estimator with one receiver. They come from the simulation config,
/// since the interpolation error table the update reads has no defaults and is only parsed.
std::shared_ptr<OptionsEstimator> MakeGPSOnlyOptions(bool do_calib_dt, bool do_calib_ext) {
    std::shared_ptr<OptionsEstimator> op = std::make_shared<OptionsEstimator>();
    op->load(std::make_shared<ov_core::YamlParser>(MINS_CONFIG_DIR "/simulation/config.yaml"));
    op->cam->enabled = op->vicon->enabled = op->wheel->enabled = op->lidar->enabled = false;
    op->gps->enabled = true;
    // Interpolate between clones with the polynomial; the IMU residual needs a propagated IMU.
    op->use_imu_res = false;
    op->gps->max_n = 1;
    op->gps->dt[0] = 0.0;
    op->gps->extrinsics[0] = P_GPS_IN_I;
    op->gps->do_calib_dt = do_calib_dt;
    op->gps->do_calib_ext = do_calib_ext;
    op->gps->init_distance = 0.5;
    op->gps->init_cov_inflation = 1.0;
    return op;
}

/// GPS-only state holding a clone at each of CLONE_TIMES along the test trajectory, unrotated.
std::shared_ptr<State> MakeCloneState(bool do_calib_dt = false, bool do_calib_ext = false) {
    std::shared_ptr<State> state = std::make_shared<State>(MakeGPSOnlyOptions(do_calib_dt, do_calib_ext));
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

Matrix3d RotationWtoE() { return ov_core::rot_z(YAW_WTOE); }

/// A noise-free fix at the given time, with the world already being the ENU frame.
GPSData FixInWorld(double time) {
    GPSData data;
    data.time = time;
    data.id = 0;
    data.meas = TrajectoryPosition(time) + P_GPS_IN_I;
    data.noise = Vector3d::Constant(FIX_NOISE);
    return data;
}

/// A noise-free fix at the given time, in an ENU frame YAW_WTOE and P_W_IN_E away from the world.
GPSData FixInENU(double time) {
    GPSData data = FixInWorld(time);
    data.meas = RotationWtoE() * data.meas + P_W_IN_E;
    return data;
}

std::vector<GPSData> FixesInENU(const std::vector<double> &times = FIX_TIMES) {
    std::vector<GPSData> data_vec;
    for (double time : times) {
        data_vec.push_back(FixInENU(time));
    }
    return data_vec;
}

/// The whitened residual of the initialization linear system at the current state.
Eigen::VectorXd InitResidual(UpdaterGPSAccess &updater, const Matrix3d &RWtoE, const Vector3d &pWinE) {
    MatrixXd Hx, Hi, R;
    Eigen::VectorXd res;
    VEC_TYPE x_order;
    updater.construct_init_linsys(FixesInENU(), RWtoE, pWinE, Hx, Hi, R, res, x_order);
    return res;
}

}  // namespace

// ---- Measurement stack ----

TEST(GPSFeedMeasurement, KeepsEveryFixInFeedOrder) {
    UpdaterGPSAccess updater(MakeCloneState());
    for (double time : FIX_TIMES) {
        updater.feed_measurement(FixInWorld(time));
    }
    ASSERT_EQ(updater.data_stack.size(), FIX_TIMES.size());
    for (size_t i = 0; i < FIX_TIMES.size(); i++) {
        EXPECT_EQ(updater.data_stack.at(i).time, FIX_TIMES.at(i));
    }
}

TEST(GPSFeedMeasurement, CapsTheTimeHistory) {
    UpdaterGPSAccess updater(MakeCloneState());
    for (int i = 0; i < 150; i++) {
        updater.feed_measurement(FixInWorld(i));
    }
    // One past the 100 the cap is written against, since the pop happens before the push.
    ASSERT_EQ(updater.t_hist.at(0).size(), 101u);
    EXPECT_EQ(updater.t_hist.at(0).front(), 49.0);
    EXPECT_EQ(updater.t_hist.at(0).back(), 149.0);
}

// ---- Keyframes ----

TEST(GPSAddKeyframes, SkipsAFixOlderThanEveryClone) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    GPSData data = FixInWorld(-1.0);
    updater.add_keyframes(data);
    EXPECT_TRUE(state->keyframes_candidate.empty());
}

TEST(GPSAddKeyframes, TheFirstFixMakesItsCloneACandidate) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    GPSData data = FixInWorld(0.12);
    updater.add_keyframes(data);
    EXPECT_EQ(state->keyframes_candidate, std::vector<double>{0.1});
}

TEST(GPSAddKeyframes, DoesNotRepeatACandidate) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    GPSData data = FixInWorld(0.12);
    updater.add_keyframes(data);
    data.time = 0.15;
    updater.add_keyframes(data);
    EXPECT_EQ(state->keyframes_candidate, std::vector<double>{0.1});
}

TEST(GPSAddKeyframes, NeedsAMetreFromTheLastCandidate) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    GPSData data = FixInWorld(0.05);
    updater.add_keyframes(data);
    // The clone at 0.3 is only 0.6 m from the one at 0.0.
    data.time = 0.35;
    updater.add_keyframes(data);
    EXPECT_EQ(state->keyframes_candidate, std::vector<double>{0.0});
    // Pull that clone 2 m away and it qualifies.
    Eigen::Matrix<double, 7, 1> pose = state->clones.at(0.3)->value();
    pose(4) += 2.0;
    state->clones.at(0.3)->set_value(pose);
    updater.add_keyframes(data);
    EXPECT_EQ(state->keyframes_candidate, (std::vector<double>{0.0, 0.3}));
}

// ---- Initialization ----

TEST(GPSInitialGuess, BothSolversRecoverTheTransform) {
    // The guess interpolates linearly between clones, which is only exact on a clone.
    const std::vector<GPSData> fixes = FixesInENU({0.1, 0.2, 0.3});
    for (bool closed_form : {true, false}) {
        std::shared_ptr<State> state = MakeCloneState();
        state->op->gps->init_closed_form = closed_form;
        UpdaterGPSAccess updater(state);
        Matrix3d RWtoE;
        Vector3d pWinE;
        ASSERT_TRUE(updater.get_initial_guess(fixes, RWtoE, pWinE)) << closed_form;
        EXPECT_TRUE(RWtoE.isApprox(RotationWtoE(), 1e-6)) << closed_form << "\n" << RWtoE;
        EXPECT_TRUE(pWinE.isApprox(P_W_IN_E, 1e-6)) << closed_form << "\n" << pWinE.transpose();
    }
}

TEST(GPSInitLinearSystem, ResidualVanishesAtTheTrueTransform) {
    std::shared_ptr<State> state = MakeCloneState(true, true);
    UpdaterGPSAccess updater(state);
    EXPECT_LT(InitResidual(updater, RotationWtoE(), P_W_IN_E).norm(), 1e-9);
}

TEST(GPSInitLinearSystem, StateJacobianMatchesNumerical) {
    std::shared_ptr<State> state = MakeCloneState(true, true);
    UpdaterGPSAccess updater(state);
    MatrixXd Hx, Hi, R;
    Eigen::VectorXd res;
    VEC_TYPE x_order;
    updater.construct_init_linsys(FixesInENU(), RotationWtoE(), P_W_IN_E, Hx, Hi, R, res, x_order);

    // res = z - h(x), so H is minus the derivative of the residual, column by column.
    const double eps = 1e-6;
    int col = 0;
    for (const std::shared_ptr<ov_type::Type> &var : x_order) {
        const Eigen::VectorXd value = var->value();
        for (int k = 0; k < var->size(); k++) {
            Eigen::VectorXd dx = Eigen::VectorXd::Zero(var->size());
            dx(k) = eps;
            var->update(dx);
            state->build_polynomial_data(false);
            Eigen::VectorXd res_plus = InitResidual(updater, RotationWtoE(), P_W_IN_E);
            var->set_value(value);
            var->update(-dx);
            state->build_polynomial_data(false);
            Eigen::VectorXd res_minus = InitResidual(updater, RotationWtoE(), P_W_IN_E);
            var->set_value(value);
            state->build_polynomial_data(false);
            Eigen::VectorXd numerical = -(res_plus - res_minus) / (2 * eps);
            EXPECT_LT((Hx.col(col + k) - numerical).norm(), 1e-5 * (1 + numerical.norm()))
                << "column " << col + k << "\nanalytical " << Hx.col(col + k).transpose() << "\nnumerical  " << numerical.transpose();
        }
        col += var->size();
    }
}

TEST(GPSInitLinearSystem, TransformJacobianMatchesNumerical) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    MatrixXd Hx, Hi, R;
    Eigen::VectorXd res;
    VEC_TYPE x_order;
    updater.construct_init_linsys(FixesInENU(), RotationWtoE(), P_W_IN_E, Hx, Hi, R, res, x_order);

    // Perturb through the 4 DOF type itself, so the test uses its error convention.
    const Eigen::Matrix<double, 7, 1> wtoe = state->trans_WtoE->value();
    const double eps = 1e-6;
    for (int k = 0; k < 4; k++) {
        Eigen::VectorXd dx = Eigen::VectorXd::Zero(4);
        dx(k) = eps;
        state->trans_WtoE->set_value(wtoe);
        state->trans_WtoE->update(dx);
        Eigen::VectorXd res_plus = InitResidual(updater, state->trans_WtoE->Rot(), state->trans_WtoE->pos());
        state->trans_WtoE->set_value(wtoe);
        state->trans_WtoE->update(-dx);
        Eigen::VectorXd res_minus = InitResidual(updater, state->trans_WtoE->Rot(), state->trans_WtoE->pos());
        Eigen::VectorXd numerical = -(res_plus - res_minus) / (2 * eps);
        EXPECT_LT((Hi.col(k) - numerical).norm(), 1e-5 * (1 + numerical.norm()))
            << "column " << k << "\nanalytical " << Hi.col(k).transpose() << "\nnumerical  " << numerical.transpose();
    }
}

TEST(GPSInitialization, NeedsThreeFixes) {
    UpdaterGPSAccess updater(MakeCloneState());
    updater.feed_measurement(FixInENU(0.12));
    updater.feed_measurement(FixInENU(0.2));
    EXPECT_FALSE(updater.try_initialization());
    EXPECT_EQ(updater.data_stack.size(), 2u);
}

TEST(GPSInitialization, IgnoresFixesOutsideTheClones) {
    UpdaterGPSAccess updater(MakeCloneState());
    updater.feed_measurement(FixInENU(0.12));
    updater.feed_measurement(FixInENU(0.2));
    updater.feed_measurement(FixInENU(5.0));
    EXPECT_FALSE(updater.try_initialization());
}

TEST(GPSInitialization, WaitsUntilTheClonesHaveTravelledFarEnough) {
    std::shared_ptr<State> state = MakeCloneState();
    state->op->gps->init_distance = 100.0;
    UpdaterGPSAccess updater(state);
    for (const GPSData &data : FixesInENU()) {
        updater.feed_measurement(data);
    }
    EXPECT_FALSE(updater.try_initialization());
    EXPECT_FALSE(updater.initialized);
}

TEST(GPSInitialization, RecoversTheTransformAndMovesTheStateToENU) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    for (const GPSData &data : FixesInENU()) {
        updater.feed_measurement(data);
    }
    ASSERT_TRUE(updater.try_initialization());
    EXPECT_TRUE(updater.initialized);
    EXPECT_TRUE(updater.data_stack.empty()) << "the fixes used to initialize are still queued";

    Eigen::Matrix<double, 7, 1> expected;
    expected.head(4) = ov_core::rot_2_quat(RotationWtoE());
    expected.tail(3) = P_W_IN_E;
    EXPECT_TRUE(updater.init_WtoE.isApprox(expected, 1e-6)) << updater.init_WtoE.transpose();
    EXPECT_EQ(updater.init_WtoE_std.size(), 4);

    for (double time : CLONE_TIMES) {
        Vector3d expected_p = RotationWtoE() * TrajectoryPosition(time) + P_W_IN_E;
        EXPECT_TRUE(state->clones.at(time)->pos().isApprox(expected_p, 1e-6)) << time << ": " << state->clones.at(time)->pos().transpose();
        EXPECT_TRUE(state->clones.at(time)->Rot().isApprox(RotationWtoE().transpose(), 1e-6)) << time;
    }
}

// ---- Update ----

TEST(GPSUpdate, AConsistentFixIsAcceptedAndShrinksTheCovariance) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    const double trace_before = state->cov.trace();
    EXPECT_TRUE(updater.update(FixInWorld(0.12)));
    EXPECT_LT(state->cov.trace(), trace_before);
}

TEST(GPSUpdate, AFixWithABadHeightFallsBackToTheHorizontalPart) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    GPSData data = FixInWorld(0.12);
    data.meas(2) += 100.0;
    const double z_before = state->clones.at(0.1)->pos()(2);
    EXPECT_TRUE(updater.update(data));
    EXPECT_NEAR(state->clones.at(0.1)->pos()(2), z_before, 1e-6) << "the bad height went in";
}

TEST(GPSUpdate, AWildlyInconsistentFixIsRejectedAndLeavesTheStateAlone) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    GPSData data = FixInWorld(0.12);
    data.meas += Vector3d(100.0, -100.0, 100.0);
    const MatrixXd cov_before = state->cov;
    const Eigen::Matrix<double, 7, 1> pose_before = state->clones.at(0.1)->value();
    EXPECT_FALSE(updater.update(data));
    EXPECT_TRUE(state->cov.isApprox(cov_before));
    EXPECT_TRUE(state->clones.at(0.1)->value().isApprox(pose_before));
}

TEST(GPSUpdate, AFixOutsideTheClonesIsNotUsed) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    const MatrixXd cov_before = state->cov;
    EXPECT_FALSE(updater.update(FixInWorld(5.0)));
    EXPECT_TRUE(state->cov.isApprox(cov_before));
}

TEST(GPSUpdate, CalibratesTheLeverArmAndTimeOffsetToo) {
    std::shared_ptr<State> state = MakeCloneState(true, true);
    UpdaterGPSAccess updater(state);
    const int ext = state->gps_extrinsic.at(0)->id();
    const int dt = state->gps_dt.at(0)->id();
    const double ext_var_before = state->cov.block(ext, ext, 3, 3).trace();
    const double dt_var_before = state->cov(dt, dt);
    EXPECT_TRUE(updater.update(FixInWorld(0.12)));
    EXPECT_LT(state->cov.block(ext, ext, 3, 3).trace(), ext_var_before);
    EXPECT_LT(state->cov(dt, dt), dt_var_before);
}

TEST(GPSTryUpdate, InitializesFirstThenUsesTheQueuedFixes) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    for (const GPSData &data : FixesInENU()) {
        updater.feed_measurement(data);
    }
    updater.try_update();
    EXPECT_TRUE(updater.initialized);
    updater.feed_measurement(FixInENU(0.25));
    const double trace_before = state->cov.trace();
    updater.try_update();
    EXPECT_TRUE(updater.data_stack.empty());
    EXPECT_LT(state->cov.trace(), trace_before);
}

TEST(GPSTryUpdate, LeavesEverythingQueuedUntilItCanInitialize) {
    UpdaterGPSAccess updater(MakeCloneState());
    updater.feed_measurement(FixInENU(0.12));
    updater.try_update();
    EXPECT_FALSE(updater.initialized);
    EXPECT_EQ(updater.data_stack.size(), 1u);
}

TEST(GPSTryUpdate, DropsAFixOlderThanTheOldestCloneAndKeepsOneNotYetCovered) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterGPSAccess updater(state);
    updater.initialized = true;
    updater.feed_measurement(FixInWorld(-1.0));
    updater.feed_measurement(FixInWorld(5.0));
    updater.try_update();
    ASSERT_EQ(updater.data_stack.size(), 1u);
    EXPECT_EQ(updater.data_stack.front().time, 5.0);
}

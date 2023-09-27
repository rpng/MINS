/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "UpdaterGPS.h"
#include "GPSTypes.h"
#include "MathGPS.h"
#include "PoseJPL_4DOF.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "types/PoseJPL.h"
#include "types/Type.h"
#include "types/Vec.h"
#include "update/UpdaterStatistics.h"
#include "utils/Jabdongsani.h"
#include "utils/Print_Logger.h"
#include "utils/colors.h"

using namespace mins;
using namespace ov_type;

UpdaterGPS::UpdaterGPS(StatePtr state) : state(state) {
  for (int i = 0; i < state->op->gps->max_n; i++) {
    Chi.emplace_back(make_shared<UpdaterStatistics>(state->op->gps->chi2_mult, "GPS", i));
    t_hist.insert({i, deque<double>()});
  }
}

void UpdaterGPS::try_update() {
  // Initialize GPS if not
  if (!initialized) {
    if (!try_initialization())
      return;
  }
  // loop through all the measurements and perform update
  for (auto data = data_stack.begin(); data != data_stack.end();) {
    // update and remove the measurement
    if (update(*data) || (*data).time + state->gps_dt.at((*data).id)->value()(0) < state->oldest_keyframe_time())
      data = data_stack.erase(data);
    else
      data++;
  }
}

bool UpdaterGPS::update(GPSData data) {
  //=========================================================================
  // Linear System
  //=========================================================================
  // Compute the size of the states involved with this measurement
  int total_hx = 0;
  unordered_map<shared_ptr<Type>, size_t> map_hx;
  VecTypePtr Hx_order;
  // Our extrinsics and intrinsics
  shared_ptr<Vec> calibration = state->gps_extrinsic.at(data.id);
  shared_ptr<Vec> timeoffset = state->gps_dt.at(data.id);

  // If doing calibration extrinsics
  if (state->op->gps->do_calib_ext && map_hx.find(calibration) == map_hx.end()) {
    map_hx.insert({calibration, total_hx});
    Hx_order.push_back(calibration);
    total_hx += calibration->size();
  }

  // If doing calibration timeoffset
  if (state->op->gps->do_calib_dt && map_hx.find(timeoffset) == map_hx.end()) {
    map_hx.insert({timeoffset, total_hx});
    Hx_order.push_back(timeoffset);
    total_hx += timeoffset->size();
  }

  VecTypePtr order;
  auto D = Dummy();
  if (state->get_interpolated_jacobian(data.time + timeoffset->value()(0), D.R, D.p, "GPS", data.id, D.VM, order)) {
    for (const auto &type : order) {
      if (map_hx.find(type) == map_hx.end()) {
        map_hx.insert({type, total_hx});
        Hx_order.push_back(type);
        total_hx += type->size();
      }
    }
  } else // return if we cannot find bounding poses.
    return false;

  //=========================================================================
  // Residual
  //=========================================================================
  Matrix3d R_EtoI;
  Vector3d p_IinE;

  if (!state->get_interpolated_pose(data.time + timeoffset->value()(0), R_EtoI, p_IinE))
    return false;
  VectorXd res = data.meas - (p_IinE + R_EtoI.transpose() * state->gps_extrinsic.at(data.id)->value());

  //=========================================================================
  // Jacobian
  //=========================================================================
  // Get Jacobian of interpolated pose in respect to the state
  vector<MatrixXd> dTdx;
  bool success = state->get_interpolated_jacobian(data.time + timeoffset->value()(0), R_EtoI, p_IinE, "GPS", data.id, dTdx, order);
  assert(success);

  MatrixXd H = MatrixXd::Zero(3, total_hx);
  MatrixXd dz_dI = MatrixXd::Zero(3, 6);
  dz_dI.block(0, 0, 3, 3) = -R_EtoI.transpose() * skew_x(calibration->value());
  dz_dI.block(0, 3, 3, 3) = Matrix3d::Identity();

  // CHAINRULE: get state clone Jacobian. This also adds timeoffset jacobian
  for (int i = 0; i < (int)dTdx.size(); i++) {
    assert(map_hx.find(order.at(i)) != map_hx.end());
    H.block(0, map_hx.at(order.at(i)), 3, order.at(i)->size()).noalias() += dz_dI * dTdx.at(i);
  }

  // Extrinsic calibration
  if (state->op->gps->do_calib_ext) {
    // Calculate the Jacobian
    Matrix3d dz_dp_GPSinI = R_EtoI.transpose();
    H.block(0, map_hx.at(calibration), 3, calibration->size()).noalias() += dz_dp_GPSinI;
  }
  //=========================================================================
  // Noise Covariance
  //=========================================================================
  MatrixXd R = Matrix3d::Zero();
  for (int i = 0; i < 3; i++)
    R(i, i) = pow(data.noise(i), 2);

  MatrixXd intr_cov = state->intr_pose_cov(state->op->clone_freq, state->op->intr_order);
  if (!state->have_clone(data.time + timeoffset->value()(0))) {
    R.noalias() += dz_dI * intr_cov * dz_dI.transpose();
  }

  //=========================================================================
  // Update
  //=========================================================================
  // Do Chi check and update!
  if (Chi.at(data.id)->Chi2Check(state, Hx_order, H, res, R)) {
    StateHelper::EKFUpdate(state, Hx_order, H, res, R, "GPS");
    return true;
  }

  // If failed, try 2d without z measurement which is often bad
  H = H.block(0, 0, 2, H.cols()).eval();
  R = R.block(0, 0, 2, 2).eval();
  res = res.block(0, 0, 2, 1).eval();
  if (Chi.at(data.id)->Chi2Check(state, Hx_order, H, res, R)) {
    StateHelper::EKFUpdate(state, Hx_order, H, res, R, "GPS");
    return true;
  }

  return false;
}

bool UpdaterGPS::try_initialization() {

  // load valid GPS measurements
  vector<GPSData> data_init;
  for (const auto data : data_stack) {
    auto D = Dummy();
    if (state->get_interpolated_jacobian(data.time + state->gps_dt.at(data.id)->value()(0), D.R, D.p, "GPS", data.id, D.VM, D.VT)) {
      data_init.push_back(data);
    }
  }

  // Return if we do not have at least three GPS measurements
  if (data_init.size() < 3) {
    PRINT2(YELLOW "[GPS]: Not enough measurements for initialization (%d of 3).\n" RESET, data_init.size());
    return false;
  }

  // If we have not moved that much in the xy plane, then just return
  double dist = 0;
  Vector3d p0 = state->clones.begin()->second->pos();
  for (const auto &clone : state->clones) {
    Vector3d p1 = clone.second->pos();
    dist += (p1 - p0).norm();
    p0 = p1;
  }
  if (dist < state->op->gps->init_distance) {
    PRINT2(YELLOW "[GPS]: Not reached initialization distance (%.2fm < %.2fm)\n" RESET, dist, state->op->gps->init_distance);
    return false;
  }

  // Debug info
  PRINT2(CYAN "[GPS]: Reached initialization distance (%.2fm >= %.2fm)\n" RESET, dist, state->op->gps->init_distance);
  PRINT2(CYAN "[GPS]: Using %d gps measurements to compute ENUtoWorld transform\n" RESET, data_init.size());

  // get initial guess
  Matrix3d RWtoE;
  Vector3d pWinE;
  if (!get_initial_guess(data_init, RWtoE, pWinE))
    return false;

  // construct linear system for initialization
  MatrixXd Hx, Hi, R;
  VectorXd res;
  vector<shared_ptr<ov_type::Type>> x_order;
  construct_init_linsys(data_init, RWtoE, pWinE, Hx, Hi, R, res, x_order);

  // Initialize our state!
  if (!StateHelper::initialize(state, state->trans_WtoE, x_order, Hx, Hi, R, res, 1e9, "GPS", false)) {
    // Failed initialization. Remove some measurements
    bool odd = true;
    for (auto i = data_stack.begin(); i != data_stack.end();) {
      if (odd) {
        odd = false;
        i = data_stack.erase(i);
      } else {
        odd = true;
        ++i;
      }
    }
    return false;
  }

  // Initialization successful. Remove used measurements
  for (auto i = data_stack.begin(); i != data_stack.end();) {
    if (std::find(data_init.begin(), data_init.end(), *i) != data_init.end())
      i = data_stack.erase(i);
    else
      ++i;
  }

  auto q = state->trans_WtoE->quat();
  auto p = state->trans_WtoE->pos();
  PRINT2(CYAN "[GPS]: ENUtoWorld transform initialized: " RESET);
  PRINT2(CYAN "q_WtoE = %.3f,%.3f,%.3f,%.3f | p_WinE = %.3f,%.3f,%.3f\n" RESET, q(0), q(1), q(2), q(3), p(0), p(1), p(2));

  // Transform the state to ENU coordinate after initialization
  transform_state_to_ENU();

  // Record our success
  initialized = true;
  return true;
}

bool UpdaterGPS::get_initial_guess(vector<GPSData> data_init, Matrix3d &RWtoE, Vector3d &pWinE) {

  // We want to use horn's solution to calculate the best fit transform between these two frames
  Vector3d pGPSinE_mean = Vector3d::Zero();
  Vector3d pGPSinW_mean = Vector3d::Zero();
  vector<Vector3d> vec_pGPSinW;
  vector<Vector3d> vec_pGPSinE;

  // Compute the centroid of the GPS measurements and the VIO measurements
  for (int i = 0; i < (int)data_init.size(); i++) {
    // get bounding poses
    double dt = state->gps_dt[data_init.at(i).id]->value()(0);
    Vector3d pGPSinI = state->gps_extrinsic.at(data_init.at(i).id)->value();
    Matrix3d RWtoI;
    Vector3d pIinW;
    bool success = state->get_interpolated_pose_linear(data_init.at(i).time + dt, RWtoI, pIinW);
    assert(success);

    // Append to our centroid, and our vector of calculated positions
    pGPSinE_mean += data_init.at(i).meas / data_init.size();
    pGPSinW_mean += (pIinW + RWtoI.transpose() * pGPSinI) / data_init.size();
    vec_pGPSinW.emplace_back(pIinW + RWtoI.transpose() * pGPSinI);
    vec_pGPSinE.emplace_back(data_init.at(i).meas);
  }

  // Directly compute the 4 dof solution from Chao's polynomial method with ransac
  if (!state->op->gps->init_closed_form) {
    // Build up our J matrix with the left and right quaterions
    Matrix4d J = Matrix4d::Zero();
    for (size_t f = 0; f < vec_pGPSinE.size(); f++) {
      Matrix<double, 4, 1> u_iq = Matrix<double, 4, 1>::Zero();
      Matrix<double, 4, 1> v_iq = Matrix<double, 4, 1>::Zero();
      u_iq.block(0, 0, 3, 1) = vec_pGPSinE[f] - pGPSinE_mean;
      v_iq.block(0, 0, 3, 1) = vec_pGPSinW[f] - pGPSinW_mean;
      J += MathGPS::Left_q(u_iq).transpose() * MathGPS::Right_q(v_iq);
    }
    // Solve for the best fit solution
    EigenSolver<MatrixXd> es(J);
    MatrixXd EV = es.eigenvectors().real();
    VectorXd eval = es.eigenvalues().real();
    // Finds the max eigen value
    int max_i = 0;
    double max_e = eval(0);
    for (int m = 1; m < 4; m++) {
      if (eval(m) > max_e) {
        max_e = eval(m);
        max_i = m;
      }
    }
    // Pull our just the rotation about the z-axis
    Matrix<double, 4, 1> q_G_2_E = EV.block(0, max_i, 4, 1);
    Matrix<double, 3, 1> rpy = rot2rpy(quat_2_Rot(q_G_2_E));
    RWtoE = rot_z(rpy(2, 0));
    pWinE = pGPSinE_mean - RWtoE * pGPSinW_mean;

  } else {
    // Directly compute the 4 dof solution from Chao's polynomial method with ransac
    if (!MathGPS::Ransac_4Dof(vec_pGPSinE, vec_pGPSinW, RWtoE, pWinE, 1, 100 * data_init.at(0).noise(0))) {
      PRINT2(REDPURPLE "[GPS]: Fail to find good solution for 4DOF initialization\n" RESET);
      return false;
    }
  }
  return true;
}

void UpdaterGPS::construct_init_linsys(vector<GPSData> data_init, Matrix3d RWtoE, Vector3d pWinE, MatrixXd &Hx, MatrixXd &Hi, MatrixXd &R, VectorXd &res, VEC_TYPE &x_order) {

  //=========================================================================
  // Linear System
  //=========================================================================
  // Total number of measurements
  int total_meas = data_init.size();

  // Compute the size of the states involved with this feature
  int total_hx = 0;
  unordered_map<shared_ptr<Type>, size_t> map_hx;
  for (const auto &data : data_init) {
    // Our calibrations
    shared_ptr<Vec> calibration = state->gps_extrinsic.at(data.id);
    shared_ptr<Vec> timeoffset = state->gps_dt.at(data.id);

    // If doing calibration extrinsics
    if (state->op->gps->do_calib_ext && map_hx.find(calibration) == map_hx.end()) {
      map_hx.insert({calibration, total_hx});
      x_order.push_back(calibration);
      total_hx += calibration->size();
    }

    // If doing calibration timeoffset
    if (state->op->gps->do_calib_dt && map_hx.find(timeoffset) == map_hx.end()) {
      map_hx.insert({timeoffset, total_hx});
      x_order.push_back(timeoffset);
      total_hx += timeoffset->size();
    }

    VecTypePtr order;
    auto D = Dummy();
    bool success = state->get_interpolated_jacobian(data.time + timeoffset->value()(0), D.R, D.p, "GPS", data.id, D.VM, order);
    assert(success);
    for (const auto &type : order) {
      if (map_hx.find(type) == map_hx.end()) {
        map_hx.insert({type, total_hx});
        x_order.push_back(type);
        total_hx += type->size();
      }
    }
  }

  // Set trans_WtoE value
  Matrix<double, 7, 1> gps_state;
  gps_state.block(0, 0, 4, 1) = rot_2_quat(RWtoE);
  gps_state.block(4, 0, 3, 1) = pWinE;
  state->trans_WtoE->set_value(gps_state);
  state->trans_WtoE->set_fej(gps_state);

  // Allocate our residual and Jacobians
  res = VectorXd::Zero(3 * total_meas);
  MatrixXd Rinv = MatrixXd::Zero(3 * total_meas, 3 * total_meas);
  Hx = MatrixXd::Zero(3 * total_meas, total_hx);
  Hi = MatrixXd::Zero(3 * total_meas, 4); // 4DoF

  for (int i = 0; i < (int)data_init.size(); i++) {
    double mt = data_init.at(i).time;
    int id = data_init.at(i).id;
    Vector3d meas = data_init.at(i).meas;
    Vector3d noise = data_init.at(i).noise;
    double dt = state->gps_dt.at(id)->value()(0);
    Vector3d pGPSinI = state->gps_extrinsic.at(id)->value();

    //=========================================================================
    // Residual
    //=========================================================================
    Matrix3d R_WtoI;
    Vector3d p_IinW;
    bool success = state->get_interpolated_pose(mt + dt, R_WtoI, p_IinW);
    assert(success);

    res.block(3 * i, 0, 3, 1) = meas - (pWinE + RWtoE * (p_IinW + R_WtoI.transpose() * pGPSinI));

    //=========================================================================
    // Jacobian
    //=========================================================================
    // Get Jacobian of interpolated pose in respect to the state
    vector<MatrixXd> dTdx;
    VecTypePtr order;
    success = state->get_interpolated_jacobian(mt + dt, R_WtoI, p_IinW, "GPS", id, dTdx, order);
    assert(success);

    MatrixXd dz_dI = MatrixXd::Zero(3, 6);
    dz_dI.block(0, 0, 3, 3) = -RWtoE * R_WtoI.transpose() * skew_x(pGPSinI);
    dz_dI.block(0, 3, 3, 3) = RWtoE;

    // CHAINRULE: get state clone Jacobian. This also adds timeoffset jacobian
    for (int j = 0; j < (int)dTdx.size(); j++) {
      assert(map_hx.find(order.at(j)) != map_hx.end());
      Hx.block(3 * i, map_hx.at(order.at(j)), 3, order.at(j)->size()).noalias() += dz_dI * dTdx.at(j);
    }

    // Extrinsic calibration
    if (state->op->gps->do_calib_ext) {
      // Calculate the Jacobian
      Matrix3d dz_dp_GPSinI = RWtoE * R_WtoI.transpose();
      Hx.block(0, map_hx.at(state->gps_extrinsic[id]), 3, state->gps_extrinsic[id]->size()).noalias() += dz_dp_GPSinI;
    }

    Hi.block(3 * i, 0, 3, 1) = (skew_x(RWtoE * (p_IinW + R_WtoI.transpose() * pGPSinI))).block(0, 2, 3, 1);
    Hi.block(3 * i, 1, 3, 3) = Matrix3d::Identity();

    //=========================================================================
    // Noise Covariance with inflation x10
    //=========================================================================
    for (int j = 0; j < 3; j++)
      Rinv(3 * i + j, 3 * i + j) = 1.0 / pow(noise(j) * 10, 2);
  }

  // Whiten the noise and get LLT of the noise covariance matrix
  LLT<MatrixXd> lltOfRinv(Rinv);
  MatrixXd Rinv_U = lltOfRinv.matrixL().transpose();
  Hx = Rinv_U * Hx;
  Hi = Rinv_U * Hi;
  res = Rinv_U * res;
  R = MatrixXd::Identity(3 * total_meas, 3 * total_meas);
}

void UpdaterGPS::transform_state_to_ENU() {
  // Marginalize all the SLAM features as they may inject inconsistency after transformation
  for (auto &feat : state->cam_SLAM_features)
    feat.second->should_marg = true;
  StateHelper::marginalize_slam(state);

  // Now transform every state into ENU frame
  // We will actually remove the newly added GPS transform state
  MatrixXd H = MatrixXd::Zero(state->cov.cols() - 4, state->cov.cols());
  H.block(0, 0, state->cov.cols() - 4, state->cov.cols() - 4).setIdentity();

  // Load values
  Matrix3d RWtoE = state->trans_WtoE->Rot();
  Vector3d pWinE = state->trans_WtoE->pos();
  Matrix3d RWtoI = state->imu->Rot();
  Vector3d pIinW = state->imu->pos();
  Vector3d vIinW = state->imu->vel();

  // Change IMU mean
  Matrix<double, 16, 1> newImu = state->imu->value();
  newImu.block(0, 0, 4, 1) = rot_2_quat(RWtoI * RWtoE.transpose());
  newImu.block(4, 0, 3, 1) = RWtoE * pIinW + pWinE;
  newImu.block(7, 0, 3, 1) = RWtoE * vIinW;
  state->imu->set_value(newImu);
  state->imu->set_fej(newImu);

  // Jacobian
  H.block(0, state->trans_WtoE->id(), 3, 1) = -(RWtoI * RWtoE.transpose()).block(0, 2, 3, 1);
  H.block(3, state->trans_WtoE->id(), 3, 1) = skew_x(RWtoE * pIinW).block(0, 2, 3, 1);
  H.block(3, state->trans_WtoE->id() + 1, 3, 3).setIdentity();
  H.block(3, 3, 3, 3) = RWtoE;
  H.block(6, 6, 3, 3) = RWtoE;
  H.block(6, state->trans_WtoE->id(), 3, 1) = skew_x(RWtoE * vIinW).block(0, 2, 3, 1);

  // Change historical IMU poses mean
  for (auto &clone : state->clones) {
    // Skip IMU if duplicated
    if (clone.second->id() == state->imu->id())
      continue;
    Matrix3d RWtoCl = clone.second->Rot();
    Vector3d pClinW = clone.second->pos();
    Matrix<double, 7, 1> cloneInENU;
    cloneInENU.block(0, 0, 4, 1).noalias() = rot_2_quat(RWtoCl * RWtoE.transpose());
    cloneInENU.block(4, 0, 3, 1).noalias() = RWtoE * pClinW + pWinE;
    clone.second->set_value(cloneInENU);
    clone.second->set_fej(cloneInENU);

    // Jacobian
    H.block(clone.second->id(), state->trans_WtoE->id(), 3, 1) = -(RWtoCl * RWtoE.transpose()).block(0, 2, 3, 1);
    H.block(clone.second->id() + 3, state->trans_WtoE->id(), 3, 1) = skew_x(RWtoE * pClinW).block(0, 2, 3, 1);
    H.block(clone.second->id() + 3, state->trans_WtoE->id() + 1, 3, 3).setIdentity();
    H.block(clone.second->id() + 3, 3, 3, 3) = RWtoE;
  }

  // New Covariance
  MatrixXd newCov = H * state->cov * H.transpose();
  StateHelper::marginalize(state, state->trans_WtoE);
  state->cov = newCov * state->op->gps->init_cov_inflation;
  PRINT2(CYAN "[GPS]: Performed state transform from World to ENU\n" RESET);
}
void UpdaterGPS::feed_measurement(const GPSData &data) {
  data_stack.push_back(data);
  t_hist.at(data.id).size() > 100 ? t_hist.at(data.id).pop_front() : void(); // remove if we have too many
  t_hist.at(data.id).push_back(data.time);
}

void UpdaterGPS::add_keyframes(GPSData &data) {
  // return if we have less than 2 clones
  double clone_t;
  if (!state->closest_older_clone_time(data.time + state->gps_dt[data.id]->value()(0), clone_t))
    return;

  // Find one step older if this is IMU pose
  if (state->clones.at(clone_t)->id() == state->imu->id() && !state->closest_older_clone_time(clone_t, clone_t))
    return;

  // set t0 as keyframe candidate if both keyframes and keyframes_candidate are empty
  if (state->keyframes.empty() && state->keyframes_candidate.empty()) {
    state->keyframes_candidate.push_back(clone_t);
    return;
  }

  // return if we already have this clone as keyframe
  if (find(state->keyframes.begin(), state->keyframes.end(), clone_t) != state->keyframes.end())
    return;

  // return if we already have this clone as keyframe candidate
  if (find(state->keyframes_candidate.begin(), state->keyframes_candidate.end(), clone_t) != state->keyframes_candidate.end())
    return;

  // insert a keyframe if we traveled more than 1 m from last keyframe
  double last_keyframe = !state->keyframes_candidate.empty() ? state->keyframes_candidate.back() : state->keyframes.back();
  if ((state->clones[clone_t]->pos() - state->clones[last_keyframe]->pos()).norm() > 1) {
    state->keyframes_candidate.push_back(clone_t);
  }
}

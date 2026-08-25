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

#include "UpdaterWheel.h"
#include "WheelJacobians.h"
#include "WheelTypes.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsWheel.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "update/UpdaterStatistics.h"
#include "utils/Print_Logger.h"
#include "utils/colors.h"

using namespace mins;
using namespace mins::wheel;
using namespace Eigen;
using namespace std;
using namespace ov_type;
using namespace ov_core;

/// Wheel measurements older than this many seconds are dropped from the stack.
static constexpr double MEASUREMENT_HISTORY_SECONDS = 100;

/// Maximum number of measurement timestamps kept for frequency bookkeeping.
static constexpr size_t MAX_TIME_HISTORY = 100;

UpdaterWheel::UpdaterWheel(StatePtr state) : state(state) { Chi = make_shared<UpdaterStatistics>(state->op->wheel->chi2_mult, "WHEEL"); }

void UpdaterWheel::try_update() {
  // If we just want to update the oldest to newest
  if (state->op->wheel->reuse_of_information) {
    // return if we did not reach the max clone size yet
    if (state->clone_window() > state->op->window_size)
      return;

    // find available newest time
    double t_up;
    if (!state->closest_older_clone_time(data_stack.back().time + state->wheel_dt->value()(0), t_up))
      return;
    update(state->oldest_clone_time(), t_up);
    return;
  }

  // check last updated clone time still exist in the state
  if (!state->have_clone(last_updated_clone_time)) {
    // find the closest newer clone time if it does not exist (mostly due to ZUPT)
    if (!state->closest_newer_clone_time(last_updated_clone_time, last_updated_clone_time)) {
      PRINT2(RED "[Wheel] Cannot find proper last updated clone time.\n" RESET);
      last_updated_clone_time = state->newest_clone_time(); // reset
      return;
    }
  }

  for (auto clone : state->clones) {
    // skip if we already updated this clone
    if (clone.first <= last_updated_clone_time)
      continue;

    // iteratively update the system
    if (!update(last_updated_clone_time, clone.first))
      break;
  }
}

bool UpdaterWheel::update(double time0, double time1) {
  // collect wheel measurements to preintegrate
  vector<WheelData> data_vec;
  double toff = state->wheel_dt->value()(0);
  if (!select_wheel_data(time0 - toff, time1 - toff, data_vec))
    return false;

  // Reset preintegrating values
  th_2D = 0;
  x_2D = 0;
  y_2D = 0;
  R_3D.setIdentity();
  p_3D.setZero();
  Cov_2D.setZero();
  Cov_3D.setZero();
  dth_di_2D.setZero();
  dx_di_2D.setZero();
  dy_di_2D.setZero();
  dR_di_3D.setZero();
  dp_di_3D.setZero();

  // Loop through all wheel messages, compute preintegrated measurement and covariance of it
  // Compute wheel intrinsic parameter Jacobians if intrinsic calibration is enabled
  for (size_t i = 0; i < data_vec.size() - 1; i++) {
    double dt = data_vec[i + 1].time - data_vec[i].time;
    // Perform 3D integration
    if (IsWheel3D(state->op->wheel->type)) {
      if (state->op->wheel->do_calib_int)
        preintegration_intrinsics_3D(dt, data_vec[i]);
      preintegration_3D(dt, data_vec[i], data_vec[i + 1]);
    } else { // Perform 2D integration
      if (state->op->wheel->do_calib_int)
        preintegration_intrinsics_2D(dt, data_vec[i]);
      preintegration_2D(dt, data_vec[i], data_vec[i + 1]);
    }
  }

  // get clones to update
  // Compute the linear system for the given measurements
  MatrixXd H;
  VectorXd res;
  vector<shared_ptr<ov_type::Type>> x_order;
  if (IsWheel3D(state->op->wheel->type))
    compute_linear_system_3D(H, res, time0, time1);
  else
    compute_linear_system_2D(H, res, time0, time1);

  // Notate what we are updating. Ordering matters
  x_order.push_back(state->clones.at(time0));
  x_order.push_back(state->clones.at(time1));
  if (state->op->wheel->do_calib_ext)
    x_order.push_back(state->wheel_extrinsic);
  if (state->op->wheel->do_calib_dt)
    x_order.push_back(state->wheel_dt);
  if (state->op->wheel->do_calib_int)
    x_order.push_back(state->wheel_intrinsic);

  // Perform update
  if (IsWheel3D(state->op->wheel->type)) {
    if (Chi->Chi2Check(state, x_order, H, res, Cov_3D))
      StateHelper::EKFUpdate(state, x_order, H, res, Cov_3D, "WHEEL");
  } else {
    if (Chi->Chi2Check(state, x_order, H, res, Cov_2D))
      StateHelper::EKFUpdate(state, x_order, H, res, Cov_2D, "WHEEL");
  }

  // record last updated time and return success
  last_updated_clone_time = time1;
  return true;
}

bool UpdaterWheel::select_wheel_data(double time0, double time1, vector<WheelData> &data_vec) {
  // Ensure we have some measurements in the first place!
  if (data_stack.empty()) {
    PRINT1("[wheel]: There are no wheel measurements. Cannot select wheel data.\n");
    return false;
  }

  // Check we can reach the time1
  if (data_stack.at(data_stack.size() - 1).time <= time1 || data_stack.front().time > time0) {
    PRINT1("[wheel]: Not enough wheel measurements to update.\n");
    return false;
  }

  // Loop through and find all the needed measurements
  // Note we split measurements based on the given time
  for (size_t i = 0; i < data_stack.size() - 1; i++) {

    // If the next timestamp is newer than time0 but current timestamp is older than time0
    // Then we should "split" our current wheel measurement
    if (data_stack[i + 1].time > time0 && data_stack.at(i).time < time0) {
      WheelData data = interpolate_data(data_stack.at(i), data_stack[i + 1], time0);
      data_vec.push_back(data);
      continue;
    }

    // If our wheel measurement is between time0 and time1
    // Then we should just append the whole measurement time
    if (data_stack.at(i).time >= time0 && data_stack[i + 1].time <= time1) {
      data_vec.push_back(data_stack.at(i));
      continue;
    }

    // If the current timestamp is greater than time1
    // We should just "split" the NEXT wheel measurement to time1,
    // NOTE: we add the current time, and then the time at the end of the interval (so we can get a dt)
    // NOTE: we also break out of this loop, as this is the last wheel measurement we need!
    if (data_stack[i + 1].time > time1) {
      // If we have a very low frequency wheel then, we could have only recorded the first integration (i.e. case 1) and nothing else
      // In this case, both the current wheel measurement and the next is greater than the desired interpolation, thus we should just cut
      // the current at the desired time Else, we have hit CASE2 and this wheel measurement is not past time1, thus add the whole wheel
      // reading
      if (data_stack.at(i).time > time1) {
        WheelData data = interpolate_data(data_stack.at(i - 1), data_stack.at(i), time1);
        data_vec.push_back(data);
      } else {
        data_vec.push_back(data_stack.at(i));
      }
      // If the added wheel message doesn't end exactly at time1
      // Then we need to add another one that is right at time1
      if (data_vec.at(data_vec.size() - 1).time != time1) {
        WheelData data = interpolate_data(data_stack.at(i), data_stack.at(i + 1), time1);
        data_vec.push_back(data);
      }
      break;
    }
  }

  // Check that we have at least one measurement to propagate with
  if (data_vec.size() < 2) {
    PRINT1("[wheel]: No wheel measurement between the clones is found.\n");
    return false;
  }

  // Loop through and ensure we do not have a zero dt values
  // This would cause the noise covariance to be Infinity
  for (size_t i = 0; i < data_vec.size() - 1; i++) {
    if (abs(data_vec[i + 1].time - data_vec[i].time) < 1e-12) {
      data_vec.erase(data_vec.begin() + i);
      i--;
    }
  }

  // Success :D
  return true;
}

/**
 * Given a measurement, this will compute the linear system of the new measurements in respect to the state
 * This will return a "small" H, res, and R which are only of a single measurement and sub-set of the state
 */
void UpdaterWheel::compute_linear_system_2D(MatrixXd &H, VectorXd &res, double time0, double time1) {
  shared_ptr<PoseJPL> pose0 = state->clones.at(time0);
  shared_ptr<PoseJPL> pose1 = state->clones.at(time1);
  Matrix3d RItoO = state->wheel_extrinsic->Rot();
  Vector3d pIinO = state->wheel_extrinsic->pos();

  // Residual at current state values
  res = ComputeResidual2D(pose0->Rot(), pose0->pos(), pose1->Rot(), pose1->pos(), RItoO, pIinO, th_2D, x_2D, y_2D);

  // Jacobians at FEJ state values
  int H_size = 12;
  int H_count = 12;
  H_size += (state->op->wheel->do_calib_ext) ? 6 : 0;
  H_size += (state->op->wheel->do_calib_dt) ? 1 : 0;
  H_size += (state->op->wheel->do_calib_int) ? 3 : 0;
  H = MatrixXd::Zero(3, H_size);

  const auto [H_poses, H_ext] = ComputeJacobians2D(pose0->Rot_fej(), pose0->pos_fej(), pose1->Rot_fej(), pose1->pos_fej(), RItoO, pIinO);
  H.block(0, 0, 3, 12) = H_poses;

  if (state->op->wheel->do_calib_ext) {
    H.block(0, H_count, 3, 6) = H_ext;
    H_count += 6;
  }

  if (state->op->wheel->do_calib_dt) {
    assert(state->cpis.find(time0) != state->cpis.end());
    assert(state->cpis.find(time1) != state->cpis.end());
    H.col(H_count) = ComputeTimeOffsetJacobian(H_poses, state->cpis.at(time0).w, state->cpis.at(time0).v, state->cpis.at(time1).w, state->cpis.at(time1).v);
    H_count += 1;
  }

  if (state->op->wheel->do_calib_int) {
    H.block(0, H_count, 1, 3) = -dth_di_2D;
    H.block(1, H_count, 1, 3) = -dx_di_2D;
    H.block(2, H_count, 1, 3) = -dy_di_2D;
  }
}

void UpdaterWheel::compute_linear_system_3D(MatrixXd &H, VectorXd &res, double time0, double time1) {
  shared_ptr<PoseJPL> pose0 = state->clones.at(time0);
  shared_ptr<PoseJPL> pose1 = state->clones.at(time1);
  Matrix3d RItoO = state->wheel_extrinsic->Rot();
  Vector3d pIinO = state->wheel_extrinsic->pos();
  // Residual at current state values
  res = ComputeResidual3D(pose0->Rot(), pose0->pos(), pose1->Rot(), pose1->pos(), RItoO, pIinO, R_3D, p_3D);
  // Jacobians at FEJ state values
  int H_size = 12;
  int H_count = 12;
  H_size += (state->op->wheel->do_calib_ext) ? 6 : 0;
  H_size += (state->op->wheel->do_calib_dt) ? 1 : 0;
  H_size += (state->op->wheel->do_calib_int) ? 3 : 0;
  H = MatrixXd::Zero(6, H_size);
  const auto [H_poses, H_ext] = ComputeJacobians3D(pose0->Rot_fej(), pose0->pos_fej(), pose1->Rot_fej(), pose1->pos_fej(), RItoO, pIinO);
  H.block(0, 0, 6, 12) = H_poses;
  if (state->op->wheel->do_calib_ext) {
    H.block(0, H_count, 6, 6) = H_ext;
    H_count += 6;
  }
  if (state->op->wheel->do_calib_dt) {
    assert(state->cpis.find(time0) != state->cpis.end());
    assert(state->cpis.find(time1) != state->cpis.end());
    H.col(H_count) = ComputeTimeOffsetJacobian(H_poses, state->cpis.at(time0).w, state->cpis.at(time0).v, state->cpis.at(time1).w, state->cpis.at(time1).v);
    H_count += 1;
  }
  if (state->op->wheel->do_calib_int) {
    H.block(0, H_count, 3, 3) = -dR_di_3D;
    H.block(3, H_count, 3, 3) = -dp_di_3D;
  }
}

void UpdaterWheel::preintegration_intrinsics_2D(double dt, const WheelData &data) {
  double rl = state->wheel_intrinsic->value()(0);
  double rr = state->wheel_intrinsic->value()(1);
  double b = state->wheel_intrinsic->value()(2);
  AccumulateIntrinsicJacobians2D(dt, data.m1, data.m2, th_2D, rl, rr, b, dth_di_2D, dx_di_2D, dy_di_2D);
}

void UpdaterWheel::preintegration_intrinsics_3D(double dt, const WheelData &data) {
  double rl = state->wheel_intrinsic->value()(0);
  double rr = state->wheel_intrinsic->value()(1);
  double b = state->wheel_intrinsic->value()(2);
  AccumulateIntrinsicJacobians3D(dt, data.m1, data.m2, R_3D, rl, rr, b, dR_di_3D, dp_di_3D);
}

void UpdaterWheel::preintegration_2D(double dt, const WheelData &data1, const WheelData &data2) {

  // load intrinsic values
  double rl = state->wheel_intrinsic->value()(0);
  double rr = state->wheel_intrinsic->value()(1);
  double b = state->wheel_intrinsic->value()(2);

  // compute the velocities at the odometry frame
  double w1 = 0, w2 = 0, v1 = 0, v2 = 0;
  switch (ModalityOf(state->op->wheel->type)) {
  case WheelModality::Angular:
    w1 = (data1.m2 * rr - data1.m1 * rl) / b;
    v1 = (data1.m2 * rr + data1.m1 * rl) / 2;
    w2 = (data2.m2 * rr - data2.m1 * rl) / b;
    v2 = (data2.m2 * rr + data2.m1 * rl) / 2;
    break;
  case WheelModality::Linear:
    w1 = (data1.m2 - data1.m1) / b;
    v1 = (data1.m2 + data1.m1) / 2;
    w2 = (data2.m2 - data2.m1) / b;
    v2 = (data2.m2 + data2.m1) / 2;
    break;
  case WheelModality::Centered:
    w1 = data1.m1;
    v1 = data1.m2;
    w2 = data2.m1;
    v2 = data2.m2;
    break;
  }

  // =========================================================
  // Compute means
  // =========================================================
  double w_alpha = (w2 - w1) / dt;
  double v_jerk = (v2 - v1) / dt;

  // k1 ================
  double w = w1;
  double v = v1;
  double k1_th = -w * dt;
  double k1_x = v * 1 * dt;
  double k1_y = -v * 0 * dt;

  // k2 ================
  double th2 = 0.5 * k1_th;
  w += 0.5 * w_alpha * dt;
  v += 0.5 * v_jerk * dt;
  double k2_th = -w * dt;
  double k2_x = v * cos(th2) * dt;
  double k2_y = -v * sin(th2) * dt;

  // k3 ================
  double th3 = 0.5 * k2_th;
  double k3_th = -w * dt;
  double k3_x = v * cos(th3) * dt;
  double k3_y = -v * sin(th3) * dt;

  // k4 ================
  double th4 = k3_th;
  w += 0.5 * w_alpha * dt;
  v += 0.5 * v_jerk * dt;
  double k4_th = -w * dt;
  double k4_x = v * cos(th4) * dt;
  double k4_y = -v * sin(th4) * dt;

  // integrated value
  double th_next = th_2D + (1.0 / 6.0) * (k1_th + 2 * k2_th + 2 * k3_th + k4_th);
  double x_next = x_2D + (1.0 / 6.0) * (k1_x + 2 * k2_x + 2 * k3_x + k4_x);
  double y_next = y_2D + (1.0 / 6.0) * (k1_y + 2 * k2_y + 2 * k3_y + k4_y);

  if (abs(w1) < SMALL_ANGULAR_RATE) // In case w is too small, apply L'Hopital rule
    y_next = y_2D - v1 * sin(th_2D - w1 * dt) * dt;
  else // use discrete integration value for y because it is working better for some unknown reason...
    y_next = y_2D - (v1 * (cos(th_2D - w1 * dt) - cos(th_2D))) / w1;

  // Compute noise Jacobians respect to measurements
  Matrix<double, 1, 2> Hwn, Hvn;
  switch (ModalityOf(state->op->wheel->type)) {
  case WheelModality::Angular:
    Hwn(0, 0) = rl / b;
    Hwn(0, 1) = -rr / b;
    Hvn(0, 0) = -rl / 2;
    Hvn(0, 1) = -rr / 2;
    break;
  case WheelModality::Linear:
    Hwn(0, 0) = 1.0 / b;
    Hwn(0, 1) = -1.0 / b;
    Hvn(0, 0) = -1.0 / 2;
    Hvn(0, 1) = -1.0 / 2;
    break;
  case WheelModality::Centered:
    Hwn(0, 0) = 1;
    Hwn(0, 1) = 0;
    Hvn(0, 0) = 0;
    Hvn(0, 1) = 1;
    break;
  }

  // Compute Jacobians respect to state preintegrated state and the measurement
  double h_thw = dt;
  double h_xth = (v1 * (cos(th_2D - w1 * dt) - cos(th_2D))) / w1;
  double h_yth = -(v1 * (sin(th_2D - w1 * dt) - sin(th_2D))) / w1;
  double h_xw = (v1 * (sin(th_2D - w1 * dt) - sin(th_2D))) / w1 / w1 + (v1 * cos(th_2D - w1 * dt) * dt) / w1;
  double h_yw = (v1 * (cos(th_2D - w1 * dt) - cos(th_2D))) / w1 / w1 - (v1 * sin(th_2D - w1 * dt) * dt) / w1;
  double h_xv = -(sin(th_2D - w1 * dt) - sin(th_2D)) / w1;
  double h_yv = -(cos(th_2D - w1 * dt) - cos(th_2D)) / w1;

  // In case w is too small, apply L'Hopital rule
  if (abs(w1) < SMALL_ANGULAR_RATE) {
    h_xth = v1 * sin(th_2D) * dt;
    h_yth = v1 * cos(th_2D) * dt;
    h_xw = v1 * sin(th_2D) * dt * dt / 2;
    h_yw = v1 * cos(th_2D) * dt * dt / 2;
    h_xv = cos(th_2D) * dt;
    h_yv = -sin(th_2D) * dt;
  }

  // Compute the Jacobians with respect to the current preintegrated states
  Matrix3d Phi_tr = Matrix3d::Identity();
  Phi_tr(1, 0) = h_xth;
  Phi_tr(2, 0) = h_yth;

  // compute noise Jacobian
  Matrix<double, 3, 2> Phi_ns = Matrix<double, 3, 2>::Zero();
  Phi_ns.block(0, 0, 1, 2) = h_thw * Hwn;
  Phi_ns.block(1, 0, 1, 2) = h_xw * Hwn + h_xv * Hvn;
  Phi_ns.block(2, 0, 1, 2) = h_yw * Hwn + h_yv * Hvn;

  // Compute Measurement covariance
  Matrix2d Q = Matrix2d::Zero();
  switch (ModalityOf(state->op->wheel->type)) {
  case WheelModality::Angular:
    Q = pow(state->op->wheel->noise_w, 2) / dt * Matrix2d::Identity();
    break;
  case WheelModality::Linear:
    Q = pow(state->op->wheel->noise_v, 2) / dt * Matrix2d::Identity();
    break;
  case WheelModality::Centered:
    Q(0, 0) = pow(state->op->wheel->noise_w, 2) / dt;
    Q(1, 1) = pow(state->op->wheel->noise_v, 2) / dt;
    break;
  }

  // integrate noise covarinace
  Cov_2D = Phi_tr * Cov_2D * Phi_tr.transpose() + Phi_ns * Q * Phi_ns.transpose();
  Cov_2D = 0.5 * (Cov_2D + Cov_2D.transpose());

  // integrate the measurement
  th_2D = th_next;
  x_2D = x_next;
  y_2D = y_next;
}

void UpdaterWheel::preintegration_3D(double dt, const WheelData &data1, const WheelData &data2) {

  // load intrinsic values
  double rl = state->wheel_intrinsic->value()(0);
  double rr = state->wheel_intrinsic->value()(1);
  double b = state->wheel_intrinsic->value()(2);

  // compute the velocities at the odometry frame
  Vector3d w_hat1, v_hat1, w_hat2, v_hat2;
  switch (ModalityOf(state->op->wheel->type)) {
  case WheelModality::Angular:
    w_hat1 << 0, 0, (data1.m2 * rr - data1.m1 * rl) / b;
    v_hat1 << (data1.m2 * rr + data1.m1 * rl) / 2, 0, 0;
    w_hat2 << 0, 0, (data2.m2 * rr - data2.m1 * rl) / b;
    v_hat2 << (data2.m2 * rr + data2.m1 * rl) / 2, 0, 0;
    break;
  case WheelModality::Linear:
    w_hat1 << 0, 0, (data1.m2 - data1.m1) / b;
    v_hat1 << (data1.m2 + data1.m1) / 2, 0, 0;
    w_hat2 << 0, 0, (data2.m2 - data2.m1) / b;
    v_hat2 << (data2.m2 + data2.m1) / 2, 0, 0;
    break;
  case WheelModality::Centered:
    w_hat1 << 0, 0, data1.m1;
    v_hat1 << data1.m2, 0, 0;
    w_hat2 << 0, 0, data2.m1;
    v_hat2 << data2.m2, 0, 0;
    break;
  }

  // =========================================================
  // Compute means
  // =========================================================
  Vector3d w_hat = w_hat1;
  Vector3d v_hat = v_hat1;
  Vector3d w_alpha = (w_hat2 - w_hat1) / dt;
  Vector3d v_jerk = (v_hat2 - v_hat1) / dt;
  Vector4d q_local = rot_2_quat(R_3D);

  // k1 ================
  Vector4d dq_0 = {0, 0, 0, 1};
  Vector4d q0_dot = 0.5 * Omega(w_hat) * dq_0;
  Matrix3d R_Gto0 = quat_2_Rot(quat_multiply(dq_0, q_local));
  Vector3d p0_dot = R_Gto0.transpose() * v_hat;
  Vector4d k1_q = q0_dot * dt;
  Vector3d k1_p = p0_dot * dt;

  // k2 ================
  w_hat += 0.5 * w_alpha * dt;
  v_hat += 0.5 * v_jerk * dt;
  Vector4d dq_1 = quatnorm(dq_0 + 0.5 * k1_q);
  Vector4d q1_dot = 0.5 * Omega(w_hat) * dq_1;
  Matrix3d R_Gto1 = quat_2_Rot(quat_multiply(dq_1, q_local));
  Vector3d p1_dot = R_Gto1.transpose() * v_hat;
  Vector4d k2_q = q1_dot * dt;
  Vector3d k2_p = p1_dot * dt;

  // k3 ================
  Vector4d dq_2 = quatnorm(dq_0 + 0.5 * k2_q);
  Vector4d q2_dot = 0.5 * Omega(w_hat) * dq_2;
  Matrix3d R_Gto2 = quat_2_Rot(quat_multiply(dq_2, q_local));
  Vector3d p2_dot = R_Gto2.transpose() * v_hat;
  Vector4d k3_q = q2_dot * dt;
  Vector3d k3_p = p2_dot * dt;

  // k4 ================
  w_hat += 0.5 * w_alpha * dt;
  v_hat += 0.5 * v_jerk * dt;
  Vector4d dq_3 = quatnorm(dq_0 + k3_q);
  Vector4d q3_dot = 0.5 * Omega(w_hat) * dq_3;
  Matrix3d R_Gto3 = quat_2_Rot(quat_multiply(dq_3, q_local));
  Vector3d p3_dot = R_Gto3.transpose() * v_hat;
  Vector4d k4_q = q3_dot * dt;
  Vector3d k4_p = p3_dot * dt;

  // integrated value
  Vector4d dq = quatnorm(dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q);
  Vector4d new_q = quat_multiply(dq, q_local);
  Matrix3d R_new = quat_2_Rot(new_q);
  Vector3d new_p = p_3D + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;

  // compute measurement noise
  Matrix<double, 6, 6> Q = Matrix<double, 6, 6>::Zero();
  switch (ModalityOf(state->op->wheel->type)) {
  case WheelModality::Angular:
    Q.block(0, 0, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(1, 1, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(2, 2, 1, 1) << pow(state->op->wheel->noise_w, 2) / dt;
    Q.block(3, 3, 1, 1) << pow(state->op->wheel->noise_v, 2) / dt;
    Q.block(4, 4, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(5, 5, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    break;
  case WheelModality::Linear:
    Q.block(0, 0, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(1, 1, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(2, 2, 1, 1) << 2 * pow(state->op->wheel->noise_v, 2) / b / b / dt;
    Q.block(3, 3, 1, 1) << pow(state->op->wheel->noise_v, 2) / 2 / dt;
    Q.block(4, 4, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(5, 5, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    break;
  case WheelModality::Centered:
    Q.block(0, 0, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(1, 1, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(2, 2, 1, 1) << pow(state->op->wheel->noise_w, 2) / dt;
    Q.block(3, 3, 1, 1) << pow(state->op->wheel->noise_v, 2) / dt;
    Q.block(4, 4, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(5, 5, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    break;
  }

  // Compute the Jacobians with respect to the current preintegrated measurements
  Matrix<double, 6, 6> Phi_tr = ComputePhiTr3D(R_3D, R_new, p_3D, new_p);

  // Compute the Jacobians with respect to the current preintegrated noises
  Matrix<double, 6, 6> Phi_ns = Matrix<double, 6, 6>::Zero();
  Phi_ns.block(0, 0, 3, 3) = dt * Matrix3d::Identity();
  Phi_ns.block(3, 3, 3, 3) = R_3D.transpose() * dt;

  // integrate noise covarinace
  Cov_3D = Phi_tr * Cov_3D * Phi_tr.transpose() + Phi_ns * Q * Phi_ns.transpose();
  Cov_3D = 0.5 * (Cov_3D + Cov_3D.transpose());

  // integrate the measurement
  R_3D = R_new;
  p_3D = new_p;
}

bool UpdaterWheel::get_bounding_data(double t_given, vector<WheelData> &data_stack, WheelData &data1, WheelData &data2) {

  // check if requested time is in valid area
  if (t_given > data_stack.back().time || t_given < data_stack.front().time)
    return false;

  // data_stack is ascending order!
  for (int i = 0; i < (int)data_stack.size() - 1; i++) {
    if (t_given >= data_stack.at(i).time && t_given < data_stack.at(i + 1).time) {
      data1 = data_stack.at(i);
      data2 = data_stack.at(i + 1);
      return true;
    }
  }
  return false;
}
void UpdaterWheel::feed_measurement(const WheelData &data) {
  // read the time before touching the stack, as data may alias one of its elements
  double time = data.time;
  data_stack.push_back(data);

  // erase measurements that are to old
  for (auto it = data_stack.begin(); it != data_stack.end();) {
    if (time - it->time > MEASUREMENT_HISTORY_SECONDS)
      it = data_stack.erase(it);
    else
      ++it;
  }

  if (t_hist.size() > MAX_TIME_HISTORY) { // remove if we have too many
    t_hist.pop_front();
  }
  t_hist.push_back(time);
}

WheelData UpdaterWheel::interpolate_data(const WheelData &data1, const WheelData &data2, double timestamp) {
  // time-distance lambda
  double lambda = (timestamp - data1.time) / (data2.time - data1.time);
  // interpolate between the two times
  WheelData data;
  data.time = timestamp;
  data.m1 = (1 - lambda) * data1.m1 + lambda * data2.m1;
  data.m2 = (1 - lambda) * data1.m2 + lambda * data2.m2;
  return data;
}

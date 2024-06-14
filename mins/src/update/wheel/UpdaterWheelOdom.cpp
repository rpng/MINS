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

#include "UpdaterRoverWheel.h"
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
using namespace Eigen;
using namespace std;
using namespace ov_type;
using namespace ov_core;

UpdaterRoverWheel::UpdaterRoverWheel(StatePtr state) : state(state) { Chi = make_shared<UpdaterStatistics>(state->op->wheel->chi2_mult, "WHEEL"); }

void UpdaterRoverWheel::try_update() {
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

bool UpdaterRoverWheel::update(double time0, double time1) {
  // collect wheel measurements to preintegrate
  vector<RoverWheelData> data_vec;
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
    if (state->op->wheel->do_calib_int)
      preintegration_intrinsics_3D(dt, data_vec[i]);
    preintegration_3D(dt, data_vec[i], data_vec[i + 1]);
  }

  // get clones to update
  // Compute the linear system for the given measurements
  MatrixXd H;
  VectorXd res;
  vector<shared_ptr<ov_type::Type>> x_order;
  compute_linear_system_3D(H, res, time0, time1);

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
  if (Chi->Chi2Check(state, x_order, H, res, Cov_3D))
    StateHelper::EKFUpdate(state, x_order, H, res, Cov_3D, "WHEEL");

  // record last updated time and return success
  last_updated_clone_time = time1;
  return true;
}

bool UpdaterRoverWheel::select_wheel_data(double time0, double time1, vector<RoverWheelData> &data_vec) {
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
      RoverWheelData data = interpolate_data(data_stack.at(i), data_stack[i + 1], time0);
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
        RoverWheelData data = interpolate_data(data_stack.at(i - 1), data_stack.at(i), time1);
        data_vec.push_back(data);
      } else {
        data_vec.push_back(data_stack.at(i));
      }
      // If the added wheel message doesn't end exactly at time1
      // Then we need to add another one that is right at time1
      if (data_vec.at(data_vec.size() - 1).time != time1) {
        RoverWheelData data = interpolate_data(data_stack.at(i), data_stack.at(i + 1), time1);
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
void UpdaterRoverWheel::compute_linear_system_3D(MatrixXd &H, VectorXd &res, double time0, double time1) {

  // Load state values
  shared_ptr<PoseJPL> pose0 = state->clones.at(time0);
  shared_ptr<PoseJPL> pose1 = state->clones.at(time1);
  Vector3d pI0inG = pose0->pos();
  Vector3d pI1inG = pose1->pos();
  Matrix3d RGtoI0 = pose0->Rot();
  Matrix3d RGtoI1 = pose1->Rot();
  Vector3d pIinO = state->wheel_extrinsic->pos();
  Matrix3d RItoO = state->wheel_extrinsic->Rot();
  Vector3d pOinI = -RItoO.transpose() * pIinO;
  Matrix3d RO0toO1 = RItoO * RGtoI1 * RGtoI0.transpose() * RItoO.transpose();
  Matrix3d RO1toO0 = RO0toO1.transpose();

  // Compute Orientation and position measurement residual
  res = Matrix<double, 6, 1>::Zero();
  // orientation
  Matrix3d R_est = RO0toO1;
  res.block(0, 0, 3, 1) = -log_so3(R_3D * R_est.transpose());
  // position
  Vector3d p_est = RItoO * RGtoI0 * (pI1inG + RGtoI1.transpose() * pOinI - pI0inG - RGtoI0.transpose() * pOinI);
  res.block(3, 0, 3, 1) = p_3D - p_est;

  // Now compute Jacobians!

  // compute the size of the Jacobian
  int H_size = 12; // Default size for pose of clone 1 and 2
  int H_count = 12;
  H_size += (state->op->wheel->do_calib_ext) ? 6 : 0;
  H_size += (state->op->wheel->do_calib_dt) ? 1 : 0;
  H_size += (state->op->wheel->do_calib_int) ? 3 : 0;
  H = MatrixXd::Zero(6, H_size);

  // Overwrite FEJ
  pI0inG = pose0->pos_fej();
  pI1inG = pose1->pos_fej();
  RGtoI0 = pose0->Rot_fej();
  RGtoI1 = pose1->Rot_fej();
  RO0toO1 = RItoO * RGtoI1 * RGtoI0.transpose() * RItoO.transpose();
  RO1toO0 = RO0toO1.transpose();

  // Jacobians in respect to current state
  // orientation
  Matrix3d dzr_dth0 = -RItoO * RGtoI1 * RGtoI0.transpose();
  Matrix3d dzr_dth1 = RItoO;
  // position
  Matrix3d dzp_dth0 = RItoO * skew_x(RGtoI0 * pI1inG + RGtoI0 * RGtoI1.transpose() * pOinI - RGtoI0 * pI0inG);
  Matrix3d dzp_dp0 = -RItoO * RGtoI0;
  Matrix3d dzp_dth1 = -RItoO * RGtoI0 * RGtoI1.transpose() * skew_x(pOinI);
  Matrix3d dzp_dp1 = RItoO * RGtoI0;

  // Derivative theta change wrt oldest pose0 and pose1
  H.block(0, 0, 3, 3) = dzr_dth0;
  H.block(0, 6, 3, 3) = dzr_dth1;
  // Derivative position change wrt oldest pose0 and pose1
  H.block(3, 0, 3, 3) = dzp_dth0;
  H.block(3, 3, 3, 3) = dzp_dp0;
  H.block(3, 6, 3, 3) = dzp_dth1;
  H.block(3, 9, 3, 3) = dzp_dp1;

  // Jacobian wrt wheel to IMU extrinsics
  if (state->op->wheel->do_calib_ext) {
    Matrix3d dzr_dthcalib = (Matrix3d::Identity() - RO0toO1);
    Matrix3d dzp_dpcalib = -RO1toO0 + Matrix3d::Identity();
    Matrix3d dzp_dthcalib = skew_x(RItoO * RGtoI0 * (pI1inG - pI0inG) - RO1toO0 * pIinO) + RO1toO0 * skew_x(pIinO);
    H.block(0, H_count, 3, 3) = dzr_dthcalib;
    H.block(3, H_count, 3, 3) = dzp_dthcalib;
    H.block(3, H_count + 3, 3, 3) = dzp_dpcalib;
    H_count += 6;
  }

  // Jacobian wrt wheel timeoffset.
  if (state->op->wheel->do_calib_dt) {
    // should be able to find imu wv
    assert(state->cpis.find(time0) != state->cpis.end());
    assert(state->cpis.find(time1) != state->cpis.end());
    Vector3d w0 = state->cpis.at(time0).w;
    Vector3d v0 = state->cpis.at(time0).v;
    Vector3d w1 = state->cpis.at(time1).w;
    Vector3d v1 = state->cpis.at(time1).v;

    // Put it in the Jacobian matrix
    H.block(0, H_count, 3, 1) = dzr_dth0 * w0 + dzr_dth1 * w1;
    H.block(3, H_count, 3, 1) = dzp_dth0 * w0 + dzp_dp0 * v0 + dzp_dth1 * w1 + dzp_dp1 * v1;
    H_count += 1;
  }

  // Jacobian wrt wheel intrinsics.
  if (state->op->wheel->do_calib_int) {
    // Note they are the opposite sign
    H.block(0, H_count, 3, 3) = -dR_di_3D;
    H.block(3, H_count, 3, 3) = -dp_di_3D;
  }
}

void UpdaterRoverWheel::preintegration_intrinsics_3D(double dt, RoverWheelData data) {
  // load measurement

  double r = state->wheel_intrinsic->value()(0);
  double b = state->wheel_intrinsic->value()(2);
  double t = state->wheel_intrinsic->value()(1);

  double phi_a, phi_b, phi_c, phi_d, w_a, w_b, w_c, w_d;

  phi_a = data.ph_a;
  phi_b = data.ph_b;
  phi_c = data.ph_c;
  phi_d = data.ph_d;

  w_a = data.w_a;
  w_b = data.w_b;
  w_c = data.w_c;
  w_d = data.w_d;

  double vx, vy;

  double vx_a, vx_b, vx_c, vx_d, vy_a, vy_b, vy_c, vy_d;
  vx_a = r * w_a * cos(phi_a);
  vx_b = r * w_b * cos(phi_b);
  vx_c = r * w_c * cos(phi_c);
  vx_d = r * w_d * cos(phi_d);

  vy_a = r * w_a * sin(phi_a);
  vy_b = r * w_b * sin(phi_b);
  vy_c = r * w_c * sin(phi_c);
  vy_d = r * w_d * sin(phi_d);

  vx = 0.25 * (vx_a + vx_b + vx_c + vx_d);
  vy = 0.25 * (vy_a + vy_b + vy_c + vy_d);

  double px_a, px_b, px_c, px_d, py_a, py_b, py_c, py_d;

  px_a = -b / 2;
  py_a = t / 2;

  px_b = b / 2;
  py_b = t / 2;

  px_c = b / 2;
  py_c = t / 2;

  px_d = -b / 2;
  py_d = -t / 2;

  double w_ = (((vx_a - vx) / -py_a) + ((vy_a - vy) / px_a) + ((vx_b - vx) / -py_b) + ((vy_b - vy) / px_b) + ((vx_c - vx) / -py_c) + ((vy_c - vy) / px_c) + ((vx_d - vx) / -py_d) +
               ((vy_d - vy) / px_d)) *
              0.125;

  // Compute angular and linear velocity
  Vector3d w(0., 0., w_);
  Vector3d v(vx, vy, 0.);

  // Compute Jacobians of w and v respect to intrinsics
  Matrix3d Hwx = Matrix3d::Zero();
  Hwx(2, 0) = (b * (-w_a * cos(phi_a) - w_b * cos(phi_b) - w_c * cos(phi_c) + 3 * w_d * cos(phi_d)) +
               2 * t * (-w_a * sin(phi_a) + w_b * sin(phi_b) + w_c * sin(phi_c) - w_d * sin(phi_d))) /
              (8 * b * t);
  // Ableiten nach b
  Hwx(2, 1) = (w_a * sin(phi_a) - w_b * sin(phi_b) - w_c * sin(phi_c) + w_d * sin(phi_d)) / (4 * b * b);
  // Ableiten nach t
  Hwx(2, 2) = (w_a * cos(phi_a) + w_b * cos(phi_b) + w_c * cos(phi_c) - 3 * w_d * cos(phi_d)) / (8 * t * t);

  Matrix3d Hvx = Matrix3d::Zero();
  Hvx(0, 0) = 0.25 * (w_a * cos(phi_a) + w_b * cos(phi_b) + w_c * cos(phi_c) + w_d * cos(phi_d));

  // Compute Jacobians of integtrated measurement of this step
  Matrix3d R = exp_so3(-w * dt);
  Matrix3d Hth = Jl_so3(-w * dt) * dt;

  // integrate the intrinsic Jacobians
  dp_di_3D = dp_di_3D - R_3D.transpose() * skew_x(v * dt) * dR_di_3D + R_3D.transpose() * Hvx * dt;
  dR_di_3D = R * dR_di_3D + Hth * Hwx;
}

void UpdaterRoverWheel::perform_calc(RoverWheelData data, Vector3d w, Vector3d v) {

  double r = state->wheel_intrinsic->value()(0);
  double b = state->wheel_intrinsic->value()(1);
  double t = state->wheel_intrinsic->value()(2);

  double phi_a, phi_b, phi_c, phi_d, w_a, w_b, w_c, w_d;

  phi_a = data.ph_a;
  phi_b = data.ph_b;
  phi_c = data.ph_c;
  phi_d = data.ph_d;

  w_a = data.w_a;
  w_b = data.w_b;
  w_c = data.w_c;
  w_d = data.w_d;

  double vx, vy;

  double vx_a, vx_b, vx_c, vx_d, vy_a, vy_b, vy_c, vy_d;
  vx_a = r * w_a * cos(phi_a);
  vx_b = r * w_b * cos(phi_b);
  vx_c = r * w_c * cos(phi_c);
  vx_d = r * w_d * cos(phi_d);

  vy_a = r * w_a * sin(phi_a);
  vy_b = r * w_b * sin(phi_b);
  vy_c = r * w_c * sin(phi_c);
  vy_d = r * w_d * sin(phi_d);

  vx = 0.25 * (vx_a + vx_b + vx_c + vx_d);
  vy = 0.25 * (vy_a + vy_b + vy_c + vy_d);

  double px_a, px_b, px_c, px_d, py_a, py_b, py_c, py_d;

  px_a = -b / 2;
  py_a = t / 2;

  px_b = b / 2;
  py_b = t / 2;

  px_c = b / 2;
  py_c = t / 2;

  px_d = -b / 2;
  py_d = -t / 2;

  double w_ = (((vx_a - vx) / -py_a) + ((vy_a - vy) / px_a) + ((vx_b - vx) / -py_b) + ((vy_b - vy) / px_b) + ((vx_c - vx) / -py_c) + ((vy_c - vy) / px_c) + ((vx_d - vx) / -py_d) +
               ((vy_d - vy) / px_d)) *
              0.125;
  w << 0.0, 0.0, w_;
  v << vx, vy, 0.0;
}

void UpdaterRoverWheel::preintegration_3D(double dt, RoverWheelData data1, RoverWheelData data2) {

  // load intrinsic values
  // double r = state->wheel_intrinsic->value()(0);
  // double b = state->wheel_intrinsic->value()(1);
  // double t = state->wheel_intrinsic->value()(2);

  // compute the velocities at the odometry frame
  Vector3d w_hat1, v_hat1, w_hat2, v_hat2;

  perform_calc(data1, w_hat1, v_hat1);
  perform_calc(data2, w_hat2, v_hat2);

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
  Q.block(0, 0, 1, 1) << pow(state->op->wheel->noise_w, 2) / dt;
  Q.block(1, 1, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
  Q.block(2, 2, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
  Q.block(3, 3, 1, 1) << pow(state->op->wheel->noise_v, 2) / dt;
  Q.block(4, 4, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
  Q.block(5, 5, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;

  // Compute the Jacobians with respect to the current preintegrated measurements
  Matrix<double, 6, 6> Phi_tr = Matrix<double, 6, 6>::Zero();
  Phi_tr.block(0, 0, 3, 3) = R_new * R_3D.transpose();
  Phi_tr.block(3, 0, 3, 3) = -R_3D.transpose() * skew_x(R_3D.transpose() * (new_p - p_3D));
  Phi_tr.block(3, 3, 3, 3) = Matrix3d::Identity();

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

bool UpdaterRoverWheel::get_bounding_data(double t_given, vector<RoverWheelData> &data_stack, RoverWheelData &data1, RoverWheelData &data2) {

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

void UpdaterRoverWheel::feed_measurement(RoverWheelData data) {
  data_stack.push_back(data);

  // erase measurements that are to old
  for (auto it = data_stack.begin(); it != data_stack.end();) {
    if (data.time - it->time > 100)
      it = data_stack.erase(it);
    else
      ++it;
  }

  t_hist.size() > 100 ? t_hist.pop_front() : void(); // remove if we have too many
  t_hist.push_back(data.time);
}

RoverWheelData UpdaterRoverWheel::interpolate_data(const RoverWheelData data1, const RoverWheelData data2, double timestamp) {
  // time-distance lambda
  double lambda = (timestamp - data1.time) / (data2.time - data1.time);
  // interpolate between the two times
  RoverWheelData data;
  data.time = timestamp;
  data.ph_a = (1 - lambda) * data1.ph_a + lambda * data2.ph_a;
  data.ph_b = (1 - lambda) * data1.ph_b + lambda * data2.ph_b;
  data.ph_c = (1 - lambda) * data1.ph_c + lambda * data2.ph_c;
  data.ph_d = (1 - lambda) * data1.ph_d + lambda * data2.ph_d;

  data.w_a = (1 - lambda) * data1.w_a + lambda * data2.w_a;
  data.w_b = (1 - lambda) * data1.w_b + lambda * data2.w_b;
  data.w_c = (1 - lambda) * data1.w_c + lambda * data2.w_c;
  data.w_d = (1 - lambda) * data1.w_d + lambda * data2.w_d;
  return data;
}

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
    if (state->op->wheel->type == "Wheel3DAng" || state->op->wheel->type == "Wheel3DLin" || state->op->wheel->type == "Wheel3DCen") {
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
  if (state->op->wheel->type == "Wheel3DAng" || state->op->wheel->type == "Wheel3DLin" || state->op->wheel->type == "Wheel3DCen")
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
  if (state->op->wheel->type == "Wheel3DAng" || state->op->wheel->type == "Wheel3DLin" || state->op->wheel->type == "Wheel3DCen") {
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

  // Create projection matrix
  Vector3d e3(0, 0, 1);
  Matrix<double, 2, 3> Lambda = Matrix<double, 2, 3>::Zero();
  Lambda.block(0, 0, 2, 2) = Matrix2d::Identity();

  // Compute Orientation and position measurement residual
  res = Vector3d::Zero();
  double theta_est = e3.transpose() * log_so3(RItoO * RGtoI1 * RGtoI0.transpose() * RItoO.transpose());
  res(0, 0) = theta_est - th_2D;
  Vector2d d_int(x_2D, y_2D);
  Vector2d d_est = Lambda * RItoO * RGtoI0 * (pI1inG + RGtoI1.transpose() * pOinI - pI0inG - RGtoI0.transpose() * pOinI);
  res.block(1, 0, 2, 1) = d_int - d_est;

  // Now compute Jacobians!

  // compute the size of the Jacobian
  int H_size = 12; // Default size for pose of clone 1 and 2
  int H_count = 12;
  H_size += (state->op->wheel->do_calib_ext) ? 6 : 0;
  H_size += (state->op->wheel->do_calib_dt) ? 1 : 0;
  H_size += (state->op->wheel->do_calib_int) ? 3 : 0;
  H = MatrixXd::Zero(3, H_size);

  // Overwrite FEJ
  pI0inG = pose0->pos_fej();
  pI1inG = pose1->pos_fej();
  RGtoI0 = pose0->Rot_fej();
  RGtoI1 = pose1->Rot_fej();
  RO0toO1 = RItoO * RGtoI1 * RGtoI0.transpose() * RItoO.transpose();
  RO1toO0 = RO0toO1.transpose();

  // Jacobians in respect to current state
  // orientation
  Matrix<double, 1, 3> dzr_dth0 = -e3.transpose() * RItoO * RGtoI1 * RGtoI0.transpose();
  Matrix<double, 1, 3> dzr_dth1 = e3.transpose() * RItoO;
  // position
  Matrix<double, 2, 3> dzp_dth0 = Lambda * RItoO * skew_x(RGtoI0 * (pI1inG + RGtoI1.transpose() * pOinI - pI0inG));
  Matrix<double, 2, 3> dzp_dp0 = -Lambda * RItoO * RGtoI0;
  Matrix<double, 2, 3> dzp_dth1 = -Lambda * RItoO * RGtoI0 * RGtoI1.transpose() * skew_x(pOinI);
  Matrix<double, 2, 3> dzp_dp1 = Lambda * RItoO * RGtoI0;

  // Derivative orientation change wrt oldest pose0 and pose1
  H.block(0, 0, 1, 3) = dzr_dth0;
  H.block(0, 6, 1, 3) = dzr_dth1;
  // Derivative position change wrt oldest pose0 and pose1
  H.block(1, 0, 2, 3) = dzp_dth0;
  H.block(1, 3, 2, 3) = dzp_dp0;
  H.block(1, 6, 2, 3) = dzp_dth1;
  H.block(1, 9, 2, 3) = dzp_dp1;

  // Jacobian wrt wheel to IMU extrinsics
  if (state->op->wheel->do_calib_ext) {
    Matrix<double, 1, 3> dzr_dthcalib = e3.transpose() * (Matrix3d::Identity() - RO0toO1);
    Matrix<double, 2, 3> dzp_dthcalib = Lambda * (skew_x(RItoO * RGtoI0 * (pI1inG - pI0inG) - RO1toO0 * pIinO) + RO1toO0 * skew_x(pIinO));
    Matrix<double, 2, 3> dzp_dpcalib = Lambda * (-RO1toO0 + Matrix3d::Identity());
    H.block(0, H_count, 1, 3) = dzr_dthcalib;
    H.block(1, H_count, 2, 3) = dzp_dthcalib;
    H.block(1, H_count + 3, 2, 3) = dzp_dpcalib;
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
    H(0, H_count) = (dzr_dth0 * w0 + dzr_dth1 * w1)(0, 0);
    H.block(1, H_count, 2, 1) = (dzp_dth0 * w0 + dzp_dp0 * v0 + dzp_dth1 * w1 + dzp_dp1 * v1);
    H_count += 1;
  }

  // Jacobian wrt wheel intrinsics.
  if (state->op->wheel->do_calib_int) {
    // Note they are the opposite sign
    H.block(0, H_count, 1, 3) = -dth_di_2D;
    H.block(1, H_count, 1, 3) = -dx_di_2D;
    H.block(2, H_count, 1, 3) = -dy_di_2D;
  }
}

/**
 * Given a measurement, this will compute the linear system of the new measurements in respect to the state
 * This will return a "small" H, res, and R which are only of a single measurement and sub-set of the state
 */
void UpdaterWheel::compute_linear_system_3D(MatrixXd &H, VectorXd &res, double time0, double time1) {

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

void UpdaterWheel::preintegration_intrinsics_2D(double dt, WheelData data) {
  // load measurement
  double w_l = data.m1;
  double w_r = data.m2;

  // load intrinsic values
  double rl = state->wheel_intrinsic->value()(0);
  double rr = state->wheel_intrinsic->value()(1);
  double b = state->wheel_intrinsic->value()(2);

  // compute the velocities of the wheel odometry frame
  double w = (w_r * rr - w_l * rl) / b;
  double v = (w_r * rr + w_l * rl) / 2;

  // Compute Jacobians of w and v respect to intrinsics
  Matrix<double, 1, 3> Hwx = Matrix<double, 1, 3>::Zero();
  Hwx(0, 0) = -w_l / b;
  Hwx(0, 1) = w_r / b;
  Hwx(0, 2) = -(w_r * rr - w_l * rl) / (b * b);
  Matrix<double, 1, 3> Hvx = Matrix<double, 1, 3>::Zero();
  Hvx(0, 0) = w_l / 2;
  Hvx(0, 1) = w_r / 2;

  // Compute Jacobians of integtrated measurement of this step
  double h_thw = dt;
  double h_xth = (v * (cos(th_2D - w * dt) - cos(th_2D))) / w;
  double h_yth = -(v * (sin(th_2D - w * dt) - sin(th_2D))) / w;
  double h_xw = (v * (sin(th_2D - w * dt) - sin(th_2D))) / w / w + (v * cos(th_2D - w * dt) * dt) / w;
  double h_yw = (v * (cos(th_2D - w * dt) - cos(th_2D))) / w / w - (v * sin(th_2D - w * dt) * dt) / w;
  double h_xv = -(sin(th_2D - w * dt) - sin(th_2D)) / w;
  double h_yv = -(cos(th_2D - w * dt) - cos(th_2D)) / w;

  // In case w is too small, apply L'Hopital rule
  if (abs(w) < 0.0001) {
    h_xth = v * sin(th_2D) * dt;
    h_yth = v * cos(th_2D) * dt;
    h_xw = v * sin(th_2D) * dt * dt / 2;
    h_yw = v * cos(th_2D) * dt * dt / 2;
    h_xv = cos(th_2D) * dt;
    h_yv = -sin(th_2D) * dt;
  }

  // integrate the intrinsic Jacobians
  dx_di_2D = dx_di_2D + h_xth * dth_di_2D + h_xw * Hwx + h_xv * Hvx;
  dy_di_2D = dy_di_2D + h_yth * dth_di_2D + h_yw * Hwx + h_yv * Hvx;
  dth_di_2D = dth_di_2D + h_thw * Hwx;
}

void UpdaterWheel::preintegration_intrinsics_3D(double dt, WheelData data) {
  // load measurement
  double w_l = data.m1;
  double w_r = data.m2;

  // load intrinsic values
  double rl = state->wheel_intrinsic->value()(0);
  double rr = state->wheel_intrinsic->value()(1);
  double b = state->wheel_intrinsic->value()(2);

  // compute the velocities of the wheel odometry frame
  Vector3d w(0, 0, (w_r * rr - w_l * rl) / b);
  Vector3d v((w_r * rr + w_l * rl) / 2, 0, 0);

  // Compute Jacobians of w and v respect to intrinsics
  Matrix3d Hwx = Matrix3d::Zero();
  Hwx(2, 0) = -w_l / b;
  Hwx(2, 1) = w_r / b;
  Hwx(2, 2) = -(w_r * rr - w_l * rl) / (b * b);
  Matrix3d Hvx = Matrix3d::Zero();
  Hvx(0, 0) = w_l / 2;
  Hvx(0, 1) = w_r / 2;

  // Compute Jacobians of integtrated measurement of this step
  Matrix3d R = exp_so3(-w * dt);
  Matrix3d Hth = Jl_so3(-w * dt) * dt;

  // integrate the intrinsic Jacobians
  dp_di_3D = dp_di_3D - R_3D.transpose() * skew_x(v * dt) * dR_di_3D + R_3D.transpose() * Hvx * dt;
  dR_di_3D = R * dR_di_3D + Hth * Hwx;
}

void UpdaterWheel::preintegration_2D(double dt, WheelData data1, WheelData data2) {

  // load intrinsic values
  double rl = state->wheel_intrinsic->value()(0);
  double rr = state->wheel_intrinsic->value()(1);
  double b = state->wheel_intrinsic->value()(2);

  // compute the velocities at the odometry frame
  double w1, w2, v1, v2;
  if (state->op->wheel->type == "Wheel2DAng") {
    w1 = (data1.m2 * rr - data1.m1 * rl) / b;
    v1 = (data1.m2 * rr + data1.m1 * rl) / 2;
    w2 = (data2.m2 * rr - data2.m1 * rl) / b;
    v2 = (data2.m2 * rr + data2.m1 * rl) / 2;
  } else if (state->op->wheel->type == "Wheel2DLin") {
    w1 = (data1.m2 - data1.m1) / b;
    v1 = (data1.m2 + data1.m1) / 2;
    w2 = (data2.m2 - data2.m1) / b;
    v2 = (data2.m2 + data2.m1) / 2;
  } else if (state->op->wheel->type == "Wheel2DCen") {
    w1 = data1.m1;
    v1 = data1.m2;
    w2 = data2.m1;
    v2 = data2.m2;
  } else {
    PRINT4("Wrong wheel type selected!");
    exit(EXIT_FAILURE);
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

  if (abs(w1) < 0.0001) // In case w is too small, apply L'Hopital rule
    y_next = y_2D - v1 * sin(th_2D - w1 * dt) * dt;
  else // use discrete integration value for y because it is working better for some unknown reason...
    y_next = y_2D - (v1 * (cos(th_2D - w1 * dt) - cos(th_2D))) / w1;

  // Compute noise Jacobians respect to measurements
  Matrix<double, 1, 2> Hwn, Hvn;
  if (state->op->wheel->type == "Wheel2DAng") {
    Hwn(0, 0) = rl / b;
    Hwn(0, 1) = -rr / b;
    Hvn(0, 0) = -rl / 2;
    Hvn(0, 1) = -rr / 2;
  } else if (state->op->wheel->type == "Wheel2DLin") {
    Hwn(0, 0) = 1.0 / b;
    Hwn(0, 1) = -1.0 / b;
    Hvn(0, 0) = -1.0 / 2;
    Hvn(0, 1) = -1.0 / 2;
  } else if (state->op->wheel->type == "Wheel2DCen") {
    Hwn(0, 0) = 1;
    Hwn(0, 1) = 0;
    Hvn(0, 0) = 0;
    Hvn(0, 1) = 1;
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
  if (abs(w1) < 0.0001) {
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
  if (state->op->wheel->type == "Wheel2DAng") {
    Q = pow(state->op->wheel->noise_w, 2) / dt * Matrix2d::Identity();
  } else if (state->op->wheel->type == "Wheel2DLin") {
    Q = pow(state->op->wheel->noise_v, 2) / dt * Matrix2d::Identity();
  } else if (state->op->wheel->type == "Wheel2DCen") {
    Q(0, 0) = pow(state->op->wheel->noise_w, 2) / dt;
    Q(1, 1) = pow(state->op->wheel->noise_v, 2) / dt;
  }

  // integrate noise covarinace
  Cov_2D = Phi_tr * Cov_2D * Phi_tr.transpose() + Phi_ns * Q * Phi_ns.transpose();
  Cov_2D = 0.5 * (Cov_2D + Cov_2D.transpose());

  // integrate the measurement
  th_2D = th_next;
  x_2D = x_next;
  y_2D = y_next;
}

void UpdaterWheel::preintegration_3D(double dt, WheelData data1, WheelData data2) {

  // load intrinsic values
  double rl = state->wheel_intrinsic->value()(0);
  double rr = state->wheel_intrinsic->value()(1);
  double b = state->wheel_intrinsic->value()(2);

  // compute the velocities at the odometry frame
  Vector3d w_hat1, v_hat1, w_hat2, v_hat2;
  if (state->op->wheel->type == "Wheel3DAng") {
    w_hat1 << 0, 0, (data1.m2 * rr - data1.m1 * rl) / b;
    v_hat1 << (data1.m2 * rr + data1.m1 * rl) / 2, 0, 0;
    w_hat2 << 0, 0, (data2.m2 * rr - data2.m1 * rl) / b;
    v_hat2 << (data2.m2 * rr + data2.m1 * rl) / 2, 0, 0;
  } else if (state->op->wheel->type == "Wheel3DLin") {
    w_hat1 << 0, 0, (data1.m2 - data1.m1) / b;
    v_hat1 << (data1.m2 + data1.m1) / 2, 0, 0;
    w_hat2 << 0, 0, (data2.m2 - data2.m1) / b;
    v_hat2 << (data2.m2 + data2.m1) / 2, 0, 0;
  } else if (state->op->wheel->type == "Wheel3DCen") {
    w_hat1 << 0, 0, data1.m1;
    v_hat1 << data1.m2, 0, 0;
    w_hat2 << 0, 0, data2.m1;
    v_hat2 << data2.m2, 0, 0;
  } else {
    PRINT4("Wrong wheel type selected!");
    exit(EXIT_FAILURE);
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
  if (state->op->wheel->type == "Wheel3DAng") {
    Q.block(0, 0, 1, 1) << pow(state->op->wheel->noise_w, 2) / dt;
    Q.block(1, 1, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(2, 2, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(3, 3, 1, 1) << pow(state->op->wheel->noise_w, 2) / dt;
    Q.block(4, 4, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(5, 5, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
  } else if (state->op->wheel->type == "Wheel3DLin") {
    Q.block(0, 0, 1, 1) << pow(state->op->wheel->noise_v, 2) / b / b / dt;
    Q.block(1, 1, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(2, 2, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(3, 3, 1, 1) << pow(state->op->wheel->noise_v, 2) / 2 / 2 / dt;
    Q.block(4, 4, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(5, 5, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
  } else if (state->op->wheel->type == "Wheel3DCen") {
    Q.block(0, 0, 1, 1) << pow(state->op->wheel->noise_w, 2) / dt;
    Q.block(1, 1, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(2, 2, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(3, 3, 1, 1) << pow(state->op->wheel->noise_v, 2) / dt;
    Q.block(4, 4, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
    Q.block(5, 5, 1, 1) << pow(state->op->wheel->noise_p, 2) / dt;
  } else {
    PRINT4(RED "[MINS] Invalid wheel type provided.\n" RESET);
    exit(EXIT_FAILURE);
  }

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
void UpdaterWheel::feed_measurement(WheelData data) {
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

WheelData UpdaterWheel::interpolate_data(const WheelData data1, const WheelData data2, double timestamp) {
  // time-distance lambda
  double lambda = (timestamp - data1.time) / (data2.time - data1.time);
  // interpolate between the two times
  WheelData data;
  data.time = timestamp;
  data.m1 = (1 - lambda) * data1.m1 + lambda * data2.m1;
  data.m2 = (1 - lambda) * data1.m2 + lambda * data2.m2;
  return data;
}

/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This code is implemented based on:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "I_Initializer.h"

#include "options/OptionsEstimator.h"
#include "options/OptionsInit.h"
#include "state/Propagator.h"
#include "utils/Print_Logger.h"
#include "utils/colors.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

using namespace std;
using namespace Eigen;
using namespace mins;
using namespace ov_core;

bool I_Initializer::initialization(Matrix<double, 17, 1> &imustate) {

  // Return if we don't have any measurements
  if (imu_pp->imu_data.size() < 2) {
    return false;
  }

  // Newest and oldest imu timestamp
  double newesttime = imu_pp->imu_data.back().timestamp;
  double oldesttime = imu_pp->imu_data.front().timestamp;

  // Return if we don't have enough for two windows
  if (newesttime - oldesttime < 2 * op->init->window_time) {
    PRINT0(YELLOW "[INIT-IMU]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // First lets collect a window of IMU readings from the newest measurement to the oldest
  std::vector<ImuData> window_1to0, window_2to1;
  for (const ImuData &data : imu_pp->imu_data) {
    if (data.timestamp > newesttime - 1 * op->init->window_time && data.timestamp <= newesttime - 0 * op->init->window_time) {
      window_1to0.push_back(data);
    }
    if (data.timestamp > newesttime - 2 * op->init->window_time && data.timestamp <= newesttime - 1 * op->init->window_time) {
      window_2to1.push_back(data);
    }
  }

  // Return if both of these failed
  if (window_1to0.size() < 2 || window_2to1.size() < 2) {
    PRINT0(YELLOW "[INIT-IMU]: unable to select window of IMU readings, not enough readings\n" RESET);
    return false;
  }

  // Calculate the sample variance for the newest window from 1 to 0
  Vector3d a_avg_1to0 = Vector3d::Zero();
  for (const ImuData &data : window_1to0) {
    a_avg_1to0 += data.am;
  }
  a_avg_1to0 /= (int)window_1to0.size();
  double a_var_1to0 = 0;
  for (const ImuData &data : window_1to0) {
    a_var_1to0 += (data.am - a_avg_1to0).dot(data.am - a_avg_1to0);
  }
  a_var_1to0 = std::sqrt(a_var_1to0 / ((int)window_1to0.size() - 1));

  // Calculate the sample variance for the second newest window from 2 to 1
  Vector3d a_avg_2to1 = Vector3d::Zero();
  Vector3d w_avg_2to1 = Vector3d::Zero();
  for (const ImuData &data : window_2to1) {
    a_avg_2to1 += data.am;
    w_avg_2to1 += data.wm;
  }
  a_avg_2to1 = a_avg_2to1 / window_2to1.size();
  w_avg_2to1 = w_avg_2to1 / window_2to1.size();
  double a_var_2to1 = 0;
  for (const ImuData &data : window_2to1) {
    a_var_2to1 += (data.am - a_avg_2to1).dot(data.am - a_avg_2to1);
  }
  a_var_2to1 = std::sqrt(a_var_2to1 / ((int)window_2to1.size() - 1));
  // PRINT2(BOLDGREEN "[INIT-IMU]: IMU excitation, %.4f,%.4f\n" RESET, a_var_1to0, a_var_2to1);

  // If it is below the threshold and we want to wait till we detect a jerk
  if (a_var_1to0 < op->init->imu_thresh) {
    PRINT2(YELLOW "[INIT-IMU]: %.2f no IMU excitation, below threshold %.4f < %.4f\n" RESET, newesttime, a_var_1to0, op->init->imu_thresh);
    return false;
  }

  // We should also check that the old state was below the threshold!
  // This is the case when we have started up moving, and thus we need to wait for a period of stationary motion
  if (a_var_2to1 > op->init->imu_thresh) {
    PRINT2(YELLOW "[INIT-IMU]: %.2f too much IMU excitation, above threshold %.4f > %.4f\n" RESET, newesttime, a_var_2to1, op->init->imu_thresh);
    return false;
  }

  // Get z axis, which aligns with -g (z_in_G=0,0,1)
  Vector3d z_axis = a_avg_2to1 / a_avg_2to1.norm();

  // Create an x_axis
  Vector3d e_1(1, 0, 0);

  // Make x_axis perpendicular to z
  Vector3d x_axis = e_1 - z_axis * z_axis.transpose() * e_1;
  x_axis = x_axis / x_axis.norm();

  // Get z from the cross product of these two
  Vector3d y_axis = skew_x(z_axis) * x_axis;

  // From these axes get rotation
  Matrix3d Ro;
  Ro.block(0, 0, 3, 1) = x_axis;
  Ro.block(0, 1, 3, 1) = y_axis;
  Ro.block(0, 2, 3, 1) = z_axis;

  // Create our state variables
  Matrix<double, 4, 1> q_GtoI = rot_2_quat(Ro);

  // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
  Vector3d bg = w_avg_2to1;
  Vector3d ba = a_avg_2to1 - quat_2_Rot(q_GtoI) * op->gravity;

  // Set our state variables
  imustate(0) = window_2to1.at(window_2to1.size() - 1).timestamp;
  imustate.block(1, 0, 4, 1) = q_GtoI;
  imustate.block(5, 0, 3, 1) = Vector3d::Zero();
  imustate.block(8, 0, 3, 1) = Vector3d::Zero();
  imustate.block(11, 0, 3, 1) = bg;
  imustate.block(14, 0, 3, 1) = ba;

  // Return :D
  PRINT0(GREEN "[Init Debug] Static IMU initialization\n" RESET);
  return true;
}
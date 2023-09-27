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

#include "ConstBsplineSE3.h"
#include "sim/BsplineSE3.h"
#include "utils/Print_Logger.h"

using namespace mins;
using namespace ov_core;

void ConstBsplineSE3::feed_trajectory(vector<VectorXd> &traj_points) {

  // Apply planner motion constraint to the trajectory points if requested
  if (const_planar) {
    for (size_t i = 0; i < traj_points.size() - 1; i++) {
      traj_points.at(i)(3, 0) = 0.0;                                       // set z zero
      traj_points.at(i)(4, 0) = 0.0;                                       // set roll zero
      traj_points.at(i)(5, 0) = 0.0;                                       // set pitch zero
      traj_points.at(i)(7, 0) = sqrt(1 - pow(traj_points.at(i)(6, 0), 2)); // set reset the w of quaternion
    }
  }

  // Get dt
  double dt = traj_points.at(traj_points.size() - 1)(0) - traj_points.at(traj_points.size() - 2)(0);
  vector<VectorXd> traj_points_loop;
  double curr_t = traj_points.at(0)(0);
  for (auto point : traj_points) {
    // Create a new point with a new time
    VectorXd new_point = point;
    new_point(0) = curr_t; // replace the time to current time

    // Put in the new traj
    traj_points_loop.push_back(new_point);
    // Time move on
    curr_t += dt;
  }
  // replace the original trajectory points
  traj_points = traj_points_loop;

  // If we don't have nonholonomic constraint, use regular B-Spline
  if (const_holonomic) {
    spline = make_shared<BsplineSE3>();
    spline->feed_trajectory(traj_points);
    start_time = spline->get_start_time();
    end_time = traj_points.back()(0);
    return;
  }

  // Find the average frequency to use as our uniform timesteps
  double sum_dt = 0;
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    sum_dt += traj_points.at(i + 1)(0) - traj_points.at(i)(0);
  }
  dt = sum_dt / (traj_points.size() - 1);
  dt = (dt < 0.05) ? 0.05 : dt;
  PRINT1("[B-SPLINE]: control point dt = %.3f (original dt of %.3f)\n", dt, sum_dt / (traj_points.size() - 1));

  // convert all our trajectory points into SE(3) matrices
  // we are given [timestamp, p_IinG, q_GtoI]
  AlignedEigenVec3d positions;
  Vector3d p_IinG_prev = traj_points.at(0).block(1, 0, 3, 1);
  for (size_t i = 0; i < traj_points.size() - 1; i++) {
    Vector3d p_IinG = traj_points.at(i).block(1, 0, 3, 1);
    // ensure there is a "motion" so that no standing still.
    if ((p_IinG - p_IinG_prev).norm() < 0.1)
      continue;

    positions.insert({traj_points.at(i)(0), p_IinG});
    p_IinG_prev = p_IinG;
  }

  // Get the oldest timestamp
  double timestamp_min = INFINITY;
  double timestamp_max = -INFINITY;
  for (const auto &pose : positions) {
    if (pose.first <= timestamp_min) {
      timestamp_min = pose.first;
    }
    if (pose.first >= timestamp_max) {
      timestamp_max = pose.first;
    }
  }

  // then create spline control points
  double timestamp_curr = timestamp_min;
  while (true) {

    // Get bounding posed for the current time
    // If we didn't find a bounding pose, then that means we are at the end of the dataset
    // Thus break out of this loop since we have created our max number of control points
    double t0, t1;
    Vector3d pos0, pos1;
    if (!find_bounding_positions(timestamp_curr, positions, t0, pos0, t1, pos1))
      break;

    // Linear interpolation and append to our control points
    double lambda = (timestamp_curr - t0) / (t1 - t0);
    double t = timestamp_curr;
    control_positions.insert({t, (1 - lambda) * pos0 + lambda * pos1});
    timestamp_curr += dt;
  }

  // The start time of the system is two dt in since we need at least two older control points
  start_time = timestamp_min + 2 * dt;
  end_time = timestamp_curr;
  PRINT1("[B-SPLINE]: start trajectory time of %.6f\n", start_time);
}

bool ConstBsplineSE3::get_pose(double timestamp, Matrix3d &R_GtoI, Vector3d &p_IinG) {
  // If we don't have nonholonomic constraint, use regular Bspline
  if (const_holonomic)
    return spline->get_pose(timestamp, R_GtoI, p_IinG);
  else {
    // Get the bounding poses for the desired timestamp
    double t0, t1, t2, t3;
    Vector3d pos0, pos1, pos2, pos3;
    bool success = find_bounding_control_positions(timestamp, control_positions, t0, pos0, t1, pos1, t2, pos2, t3, pos3);
    // PRINT1("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success =
    // %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

    // Return failure if we can't get bounding poses
    if (!success) {
      R_GtoI.setIdentity();
      p_IinG.setZero();
      return false;
    }

    // Our De Boor-Cox matrix scalars
    double DT = (t2 - t1);
    double u = (timestamp - t1) / DT;
    double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
    double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
    double b2 = 1.0 / 6.0 * (u * u * u);

    // Finally get the interpolated pose
    // Get the interpolated pose
    if (!get_orientation(timestamp, R_GtoI))
      return false;
    p_IinG = pos0 + b0 * (pos1 - pos0) + b1 * (pos2 - pos1) + b2 * (pos3 - pos2);
    return true;
  }
}

bool ConstBsplineSE3::get_velocity(double timestamp, Matrix3d &R_GtoI, Vector3d &p_IinG, Vector3d &w_IinI, Vector3d &v_IinG) {
  // If we don't have nonholonomic constraint, use regular Bspline
  if (const_holonomic)
    return spline->get_velocity(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG);
  else {
    // Get the bounding poses for the desired timestamp
    double t0, t1, t2, t3;
    Vector3d pos0, pos1, pos2, pos3;
    bool success = find_bounding_control_positions(timestamp, control_positions, t0, pos0, t1, pos1, t2, pos2, t3, pos3);
    // PRINT1("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success =
    // %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

    // Return failure if we can't get bounding poses
    if (!success) {
      w_IinI.setZero();
      v_IinG.setZero();
      return false;
    }

    // Our De Boor-Cox matrix scalars
    double DT = (t2 - t1);
    double u = (timestamp - t1) / DT;
    double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
    double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
    double b2 = 1.0 / 6.0 * (u * u * u);
    double b0dot = 1.0 / (6.0 * DT) * (3 - 6 * u + 3 * u * u);
    double b1dot = 1.0 / (6.0 * DT) * (3 + 6 * u - 6 * u * u);
    double b2dot = 1.0 / (6.0 * DT) * (3 * u * u);

    // Get the interpolated pose
    if (!get_orientation(timestamp, R_GtoI))
      return false;
    p_IinG = pos0 + b0 * (pos1 - pos0) + b1 * (pos2 - pos1) + b2 * (pos3 - pos2);

    // Get the interpolated velocities
    if (!get_angular_velocity(timestamp, w_IinI))
      return false;
    v_IinG = b0dot * (pos1 - pos0) + b1dot * (pos2 - pos1) + b2dot * (pos3 - pos2);
    return true;
  }
}

bool ConstBsplineSE3::get_acceleration(double timestamp, Matrix3d &R_GtoI, Vector3d &p_IinG, Vector3d &w_IinI, Vector3d &v_IinG, Vector3d &alpha_IinI, Vector3d &a_IinG) {
  // If we don't have nonholonomic constraint, use regular Bspline
  if (const_holonomic)
    return spline->get_acceleration(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);
  else {
    // Get the bounding poses for the desired timestamp
    double t0, t1, t2, t3;
    Vector3d pos0, pos1, pos2, pos3;
    bool success = find_bounding_control_positions(timestamp, control_positions, t0, pos0, t1, pos1, t2, pos2, t3, pos3);

    // Return failure if we can't get bounding poses
    if (!success) {
      alpha_IinI.setZero();
      a_IinG.setZero();
      return false;
    }

    // Our De Boor-Cox matrix scalars
    double DT = (t2 - t1);
    double u = (timestamp - t1) / DT;
    double b0 = 1.0 / 6.0 * (5 + 3 * u - 3 * u * u + u * u * u);
    double b1 = 1.0 / 6.0 * (1 + 3 * u + 3 * u * u - 2 * u * u * u);
    double b2 = 1.0 / 6.0 * (u * u * u);
    double b0dot = 1.0 / (6.0 * DT) * (3 - 6 * u + 3 * u * u);
    double b1dot = 1.0 / (6.0 * DT) * (3 + 6 * u - 6 * u * u);
    double b2dot = 1.0 / (6.0 * DT) * (3 * u * u);
    double b0dotdot = 1.0 / (6.0 * DT * DT) * (-6 + 6 * u);
    double b1dotdot = 1.0 / (6.0 * DT * DT) * (6 - 12 * u);
    double b2dotdot = 1.0 / (6.0 * DT * DT) * (6 * u);

    // Get the interpolated pose
    if (!get_orientation(timestamp, R_GtoI))
      return false;
    p_IinG = pos0 + b0 * (pos1 - pos0) + b1 * (pos2 - pos1) + b2 * (pos3 - pos2);

    // Get the interpolated velocities
    if (!get_angular_velocity(timestamp, w_IinI))
      return false;
    v_IinG = b0dot * (pos1 - pos0) + b1dot * (pos2 - pos1) + b2dot * (pos3 - pos2);

    // Finally get the interpolated velocities
    if (!get_angular_acceleration(timestamp, alpha_IinI))
      return false;
    a_IinG = b0dotdot * (pos1 - pos0) + b1dotdot * (pos2 - pos1) + b2dotdot * (pos3 - pos2);
    return true;
  }
}
bool ConstBsplineSE3::get_orientation(double timestamp, Matrix3d &R_GtoI) {

  // compute velocity first.
  Vector3d v_IinG;
  if (!get_linear_velocity(timestamp, v_IinG))
    return false;

  if (v_IinG.norm() < 0.01)
    return false;
  // Now compute rotation that is same direction of velocity
  Vector3d r1 = v_IinG / v_IinG.norm();
  Vector3d g; // This is gravity direction.
  g << 0, 0, 1;
  Vector3d r3 = g - r1 * (r1.transpose() * g);
  r3 = r3 / r3.norm();
  Vector3d r2 = skew_x(r3) * r1;
  r2 = r2 / r2.norm();

  R_GtoI.block(0, 0, 1, 3) = r1.transpose();
  R_GtoI.block(1, 0, 1, 3) = r2.transpose();
  R_GtoI.block(2, 0, 1, 3) = r3.transpose();
  return true;
}

bool ConstBsplineSE3::get_linear_velocity(double timestamp, Vector3d &v_IinG) {

  // Get the bounding poses for the desired timestamp
  double t0, t1, t2, t3;
  Vector3d pos0, pos1, pos2, pos3;
  bool success = find_bounding_control_positions(timestamp, control_positions, t0, pos0, t1, pos1, t2, pos2, t3, pos3);
  // PRINT1("[SIM]: time curr = %.6f | dt1 = %.3f | dt2 = %.3f | dt3 = %.3f | dt4 = %.3f | success =
  // %d\n",timestamp,t0-timestamp,t1-timestamp,t2-timestamp,t3-timestamp,(int)success);

  // Return failure if we can't get bounding poses
  if (!success) {
    v_IinG.setZero();
    return false;
  }

  // Our De Boor-Cox matrix scalars
  double DT = (t2 - t1);
  double u = (timestamp - t1) / DT;
  double b0dot = 1.0 / (6.0 * DT) * (3 - 6 * u + 3 * u * u);
  double b1dot = 1.0 / (6.0 * DT) * (3 + 6 * u - 6 * u * u);
  double b2dot = 1.0 / (6.0 * DT) * (3 * u * u);
  v_IinG = b0dot * (pos1 - pos0) + b1dot * (pos2 - pos1) + b2dot * (pos3 - pos2);
  return true;
}

bool ConstBsplineSE3::get_angular_velocity(double timestamp, Vector3d &w_IinI) {
  // Also compute the rotation of epsilon timestep
  Matrix3d R_GtoI3, R_GtoI2, R_GtoI1, R_GtoI0;
  if (!get_orientation(timestamp + 2 * epsilon, R_GtoI3))
    return false;
  if (!get_orientation(timestamp + epsilon, R_GtoI2))
    return false;
  if (!get_orientation(timestamp - epsilon, R_GtoI1))
    return false;
  if (!get_orientation(timestamp - 2 * epsilon, R_GtoI0))
    return false;

  w_IinI = (-log_so3(R_GtoI0 * R_GtoI3.transpose()) - 8 * log_so3(R_GtoI2 * R_GtoI1.transpose())) / (12 * epsilon);
  return true;
}

bool ConstBsplineSE3::get_angular_acceleration(double timestamp, Vector3d &alpha_IinI) {

  // Also compute the rotation of epsilon timestep
  Vector3d w_IinI3, w_IinI2, w_IinI1, w_IinI0;
  if (!get_angular_velocity(timestamp + 2 * epsilon, w_IinI3))
    return false;
  if (!get_angular_velocity(timestamp + epsilon, w_IinI2))
    return false;
  if (!get_angular_velocity(timestamp - epsilon, w_IinI1))
    return false;
  if (!get_angular_velocity(timestamp - 2 * epsilon, w_IinI0))
    return false;

  alpha_IinI = (-w_IinI3 + 8 * w_IinI2 - 8 * w_IinI1 + w_IinI0) / 12 / epsilon;
  return true;
}

bool ConstBsplineSE3::find_bounding_control_positions(const double timestamp, const AlignedEigenVec3d &positions, double &t0, Vector3d &pos0, double &t1, Vector3d &pos1,
                                                      double &t2, Vector3d &pos2, double &t3, Vector3d &pos3) {

  // Set the default values
  t0 = -1;
  t1 = -1;
  t2 = -1;
  t3 = -1;

  // Get the two bounding poses
  bool success = find_bounding_positions(timestamp, positions, t1, pos1, t2, pos2);

  // Return false if this was a failure
  if (!success)
    return false;

  // Now find the poses that are below and above
  auto iter_t1 = positions.find(t1);
  auto iter_t2 = positions.find(t2);

  // Check that t1 is not the first timestamp
  if (iter_t1 == positions.begin()) {
    return false;
  }

  // Move the older pose backwards in time
  // Move the newer one forwards in time
  auto iter_t0 = --iter_t1;
  auto iter_t3 = ++iter_t2;

  // Check that it is valid
  if (iter_t3 == positions.end()) {
    return false;
  }

  // Set the oldest one
  t0 = iter_t0->first;
  pos0 = iter_t0->second;

  // Set the newest one
  t3 = iter_t3->first;
  pos3 = iter_t3->second;

  // Assert the timestamps
  if (success) {
    assert(t0 < t1);
    assert(t1 < t2);
    assert(t2 < t3);
  }

  // Return true if we found all four bounding poses
  return success;
}

bool ConstBsplineSE3::find_bounding_positions(const double timestamp, const AlignedEigenVec3d &poses, double &t0, Vector3d &pos0, double &t1, Vector3d &pos1) {

  // Set the default values
  t0 = -1;
  t1 = -1;

  // Find the bounding poses
  bool found_older = false;
  bool found_newer = false;

  // Find the bounding poses for interpolation.
  auto lower_bound = poses.lower_bound(timestamp); // Finds timestamp or next(timestamp) if not available
  auto upper_bound = poses.upper_bound(timestamp); // Finds next(timestamp)

  if (lower_bound != poses.end()) {
    // Check that the lower bound is the timestamp.
    // If not then we move iterator to previous timestamp so that the timestamp is bounded
    if (lower_bound->first == timestamp) {
      found_older = true;
    } else if (lower_bound != poses.begin()) {
      --lower_bound;
      found_older = true;
    }
  }

  if (upper_bound != poses.end()) {
    found_newer = true;
  }

  // If we found the older one, set it
  if (found_older) {
    t0 = lower_bound->first;
    pos0 = lower_bound->second;
  }

  // If we found the newest one, set it
  if (found_newer) {
    t1 = upper_bound->first;
    pos1 = upper_bound->second;
  }

  // Assert the timestamps
  if (found_older && found_newer)
    assert(t0 < t1);

  // Return true if we found both bounds
  return (found_older && found_newer);
}

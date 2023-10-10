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

#include "Simulator.h"
#include "ConstBsplineSE3.h"
#include "SimulationPlane.h"
#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsIMU.h"
#include "options/OptionsLidar.h"
#include "options/OptionsSimulation.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "update/cam/CamTypes.h"
#include "update/gps/GPSTypes.h"
#include "update/vicon/ViconTypes.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include "utils/dataset_reader.h"
#include "utils/quat_ops.h"
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace ov_core;
using namespace mins;
using namespace Eigen;

Simulator::Simulator(shared_ptr<Options> op) : op(op) {

  // Nice startup message
  PRINT1("=======================================\n");
  PRINT1("MULTI-SENSOR SIMULATOR STARTING\n");
  PRINT1("=======================================\n");

  // Load the seeds for the random number generators
  seed_ptrb = mt19937(op->sim->seed);                     // calibration perturbation
  seed_imu = mt19937(op->sim->seed);                      // imu measurement
  seed_wheel = mt19937(op->sim->seed);                    // wheel measurement
  seed_init = mt19937(op->sim->seed);                     // wheel measurement
  for (int i = 0; i < op->sim->est_true->cam->max_n; i++) // camera measurement
    seed_cams.emplace_back(op->sim->seed + i);
  for (int i = 0; i < op->sim->est_true->vicon->max_n; i++)
    seed_vicons.emplace_back(op->sim->seed + i); // vicon measurement
  for (int i = 0; i < op->sim->est_true->lidar->max_n; i++)
    seed_lidars.emplace_back(op->sim->seed + i); // lidar measurement
  for (int i = 0; i < op->sim->est_true->gps->max_n; i++)
    seed_gps.emplace_back(op->sim->seed + i); // gps measurement

  //===============================================================
  //===============================================================

  // Load the groundtruth trajectory and its spline
  DatasetReader::load_simulated_trajectory(op->sim->BSpline_path, traj_data);
  spline = make_shared<ConstBsplineSE3>(op->sim->const_holonomic, op->sim->const_planar);
  spline->feed_trajectory(traj_data);

  // Get the pose at the initial timestamp
  Matrix3d R_GtoIi;
  Vector3d p_IiinG;
  timestamp = spline->start_time;
  while (true) {
    if (timestamp > spline->end_time) {
      PRINT4(RED "[SIM]: unable to find jolt in the groundtruth data to initialize at. 1\n" RESET);
      exit(EXIT_FAILURE);
    }
    // get pose
    if (!get_imu_pose(timestamp, R_GtoIi, p_IiinG)) {
      timestamp += 0.1;
      continue;
    }
    break;
  }

  // Find the timestamp that we move enough to be considered "moved"
  double distance = 0.0;
  while (true) {
    // Get the pose at the current timestamp
    Matrix3d R_GtoI;
    Vector3d p_IinG;
    if (!get_imu_pose(timestamp, R_GtoI, p_IinG)) {
      PRINT4(RED "[SIM]: unable to find jolt in the groundtruth data to initialize at. 2\n" RESET);
      exit(EXIT_FAILURE);
    }
    // Append to our scalar distance
    distance += (p_IinG - p_IiinG).norm();
    p_IiinG = p_IinG;
    // Now check if we have an acceleration, else move forward in time
    if (distance > op->sim->distance_threshold)
      break;

    timestamp += 0.1;
  }

  PRINT1("[SIM]: moved %.3f seconds into the dataset where it starts moving\n", timestamp - spline->start_time);

  // Set all our timestamps as starting from the minimum spline time
  // set asynchronous starting times
  timestamp_last_imu = timestamp;
  timestamp_last_wheel = timestamp;
  for (int i = 0; i < op->sim->est_true->cam->max_n; i++)
    timestamp_last_cams.push_back(timestamp + (double)i / op->sim->est_true->cam->max_n / op->sim->freq_cam);
  for (int i = 0; i < op->sim->est_true->vicon->max_n; i++)
    timestamp_last_vicons.push_back(timestamp + (double)i / op->sim->est_true->vicon->max_n / op->sim->freq_vicon);
  for (int i = 0; i < op->sim->est_true->lidar->max_n; i++)
    timestamp_last_lidars.push_back(timestamp + (double)i / op->sim->est_true->lidar->max_n / op->sim->freq_lidar);
  for (int i = 0; i < op->sim->est_true->gps->max_n; i++)
    timestamp_last_gpss.push_back(timestamp + (double)i / op->sim->est_true->gps->max_n / op->sim->freq_gps);

  // Set initial random biases
  if (!op->sim->remove_noise) {
    std::uniform_real_distribution<double> n(-3.0, 3.0);
    for (int i = 0; i < 3; i++) {
      true_imu_ba(i) = n(seed_imu);
      true_imu_bg(i) = n(seed_imu);
    }
  } else {
    true_imu_ba = Vector3d::Zero();
    true_imu_bg = Vector3d::Zero();
  }
  bias_times.push_back(timestamp_last_imu - 1.0 / op->sim->freq_imu);
  vec_ba.push_back(true_imu_ba);
  vec_bg.push_back(true_imu_bg);
  bias_times.push_back(timestamp_last_imu);
  vec_ba.push_back(true_imu_ba);
  vec_bg.push_back(true_imu_bg);

  // Our simulation is running
  is_running = true;

  //===============================================================
  //===============================================================

  // Perturb all calibration if we should
  perturb_calibration();

  // Generate camera feature map
  if (op->sim->est_true->cam->enabled)
    create_camera_features();

  // Generate lidar plane map
  if (op->sim->est_true->lidar->enabled) {
    // Try loading plane data from file
    if (!load_plane_data(op->sim->planes_path)) {
      // if failed, generate simple one
      generate_planes();
    }
  }
}

void Simulator::create_camera_features() {
  // We will create synthetic camera frames and ensure that each has enough features
  double dt = 1.0 / op->sim->freq_cam / op->sim->est_true->cam->max_n;
  size_t mapsize = cam_featmap.size();
  PRINT1("[SIM]: Generating map features at %d rate\n", (int)(1.0 / dt));

  // Loop through each camera
  // NOTE: we loop through cameras here so that the feature map for camera 1 will always be the same
  // NOTE: thus when we add more cameras the first camera should get the same measurements
  for (int i = 0; i < op->sim->est_true->cam->max_n; i++) {

    // Reset the start time
    double time_synth = spline->start_time;

    // Loop through each pose and generate our feature map in them!!!!
    while (true) {

      // Get the pose at the current timestep
      Matrix3d R_GtoI;
      Vector3d p_IinG, w_IinI, v_IinG;
      if (!get_imu_velocity(time_synth, R_GtoI, p_IinG, w_IinI, v_IinG))
        break;

      // increase max feature distance if speed is too fast
      op->sim->max_feature_gen_distance = max(v_IinG.norm() * 5, op->sim->max_feature_gen_distance);

      // Get the uv features for this frame
      vector<pair<size_t, VectorXf>> uvs = project_pointcloud(R_GtoI, p_IinG, i);
      // If we do not have enough, generate more
      if ((int)uvs.size() < op->sim->est_true->cam->n_pts) {
        generate_points(R_GtoI, p_IinG, i, cam_featmap, op->sim->est_true->cam->n_pts - (int)uvs.size());
      }

      // Move forward in time
      time_synth += dt;
    }

    // Debug print
    PRINT1("[SIM]: Generated %d map features in total over %d frames (camera %d)\n", (int)(cam_featmap.size() - mapsize), (int)((time_synth - spline->start_time) / dt), i);
    mapsize = cam_featmap.size();
  }
}

void Simulator::perturb_calibration() {

  // Note: perturb only that our estimator calibrates!

  // One std generator
  std::normal_distribution<double> w(0, 1);

  // vicon
  if (op->est->vicon->enabled) {
    for (int i = 0; i < op->est->vicon->max_n; i++) {
      // Time offset
      if (op->est->vicon->do_calib_dt && op->sim->do_perturb)
        op->est->vicon->dt.at(i) += sqrt(op->est->vicon->init_cov_dt) * w(seed_ptrb);
      if (op->est->vicon->do_calib_ext && op->sim->do_perturb) {
        // Extrinsic - Orientation
        double std = sqrt(op->est->vicon->init_cov_ex_o);
        Vector3d w_vec(std * w(seed_ptrb), std * w(seed_ptrb), std * w(seed_ptrb));
        op->est->vicon->extrinsics.at(i).head(4) = rot_2_quat(exp_so3(w_vec) * quat_2_Rot(op->est->vicon->extrinsics.at(i).head(4)));
        // Extrinsic - Position
        for (int r = 4; r < 7; r++)
          op->est->vicon->extrinsics.at(i)(r) += sqrt(op->est->vicon->init_cov_ex_o) * w(seed_ptrb);
      }
    }
  }

  // lidar
  if (op->est->lidar->enabled) {
    for (int i = 0; i < op->est->lidar->max_n; i++) {
      // Time offset
      if (op->est->lidar->do_calib_dt && op->sim->do_perturb)
        op->est->lidar->dt.at(i) += sqrt(op->est->lidar->init_cov_dt) * w(seed_ptrb);
      // Extrinsic - Orientation
      if (op->est->lidar->do_calib_ext && op->sim->do_perturb) {
        double std = sqrt(op->est->lidar->init_cov_ex_o);
        Vector3d w_vec(std * w(seed_ptrb), std * w(seed_ptrb), std * w(seed_ptrb));
        op->est->lidar->extrinsics.at(i).head(4) = rot_2_quat(exp_so3(w_vec) * quat_2_Rot(op->est->lidar->extrinsics.at(i).head(4)));
      }
      // Extrinsic - Position
      if (op->est->lidar->do_calib_ext && op->sim->do_perturb) {
        for (int r = 4; r < 7; r++)
          op->est->lidar->extrinsics.at(i)(r) += sqrt(op->est->lidar->init_cov_ex_p) * w(seed_ptrb);
      }
    }
  }

  // wheel
  if (op->est->wheel->enabled) {
    // Time offset
    if (op->est->wheel->do_calib_dt && op->sim->do_perturb)
      op->est->wheel->dt += sqrt(op->est->wheel->init_cov_dt) * w(seed_ptrb);
    // Extrinsic - Orientation
    if (op->est->wheel->do_calib_ext && op->sim->do_perturb) {
      double std = sqrt(op->est->wheel->init_cov_ex_o);
      Vector3d w_vec(std * w(seed_ptrb), std * w(seed_ptrb), std * w(seed_ptrb));
      op->est->wheel->extrinsics.head(4) = rot_2_quat(exp_so3(w_vec) * quat_2_Rot(op->est->wheel->extrinsics.head(4)));
    }
    // Extrinsic - Position
    if (op->est->wheel->do_calib_ext && op->sim->do_perturb) {
      for (int r = 4; r < 7; r++)
        op->est->wheel->extrinsics(r) += sqrt(op->est->wheel->init_cov_ex_p) * w(seed_ptrb);
    }
    // intrinsics (wheel radius and baseline)
    if (op->est->wheel->do_calib_int && op->sim->do_perturb) {
      op->est->wheel->intrinsics(0) += sqrt(op->est->wheel->init_cov_in_r) * w(seed_ptrb);
      op->est->wheel->intrinsics(1) += sqrt(op->est->wheel->init_cov_in_r) * w(seed_ptrb);
    }
    if (op->est->wheel->do_calib_int && op->sim->do_perturb)
      op->est->wheel->intrinsics(2) += sqrt(op->est->wheel->init_cov_in_b) * w(seed_ptrb);
  }

  // gps
  if (op->est->gps->enabled) {
    for (int i = 0; i < op->est->gps->max_n; i++) {
      // timeoffset
      if (op->est->gps->do_calib_dt && op->sim->do_perturb)
        op->est->gps->dt.at(i) += sqrt(op->est->gps->init_cov_dt) * w(seed_ptrb);
      // extrinsic transform (position)
      if (op->est->gps->do_calib_ext && op->sim->do_perturb) {
        for (int r = 0; r < 3; r++)
          op->est->gps->extrinsics.at(i)(r) += sqrt(op->est->gps->init_cov_ex) * w(seed_ptrb);
      }
    }
  }

  // camera
  if (op->est->cam->enabled) {
    for (int i = 0; i < op->est->cam->max_n; i++) {
      // timeoffset
      if (op->est->cam->do_calib_dt && op->sim->do_perturb) {
        // perturb only one of the stereo camera dt
        if (op->est->cam->use_stereo && op->est->cam->stereo_pairs.find(i) != op->est->cam->stereo_pairs.end()) {
          if (i < op->est->cam->stereo_pairs.at(i))
            op->est->cam->dt.at(i) += sqrt(op->est->cam->init_cov_dt) * w(seed_ptrb);
          else
            op->est->cam->dt.at(i) = op->est->cam->dt.at(op->est->cam->stereo_pairs.at(i));
        } else {
          op->est->cam->dt.at(i) += sqrt(op->est->cam->init_cov_dt) * w(seed_ptrb);
        }
      }
      // extrinsic transform (orientation)
      if (op->est->cam->do_calib_ext && op->sim->do_perturb) {
        double std = sqrt(op->est->cam->init_cov_ex_o);
        Vector3d w_vec(std * w(seed_ptrb), std * w(seed_ptrb), std * w(seed_ptrb));
        op->est->cam->extrinsics.at(i).head(4) = rot_2_quat(exp_so3(w_vec) * quat_2_Rot(op->est->cam->extrinsics.at(i).head(4)));
      }
      // extrinsic transform (position)
      if (op->est->cam->do_calib_ext && op->sim->do_perturb) {
        for (int r = 4; r < 7; r++)
          op->est->cam->extrinsics.at(i)(r) += sqrt(op->est->cam->init_cov_ex_p) * w(seed_ptrb);
      }
      // Camera intrinsic properties (k1, k2, p1, p2)
      if (op->est->cam->do_calib_int && op->sim->do_perturb) {
        for (int k = 0; k < 2; k++)
          op->est->cam->intrinsics.at(i)(k) += sqrt(op->est->cam->init_cov_in_k) * w(seed_ptrb);
      }
      // Camera intrinsic properties (k1, k2, p1, p2)
      if (op->est->cam->do_calib_int && op->sim->do_perturb) {
        for (int k = 2; k < 4; k++)
          op->est->cam->intrinsics.at(i)(k) += sqrt(op->est->cam->init_cov_in_c) * w(seed_ptrb);
      }
      // Camera intrinsic properties (r1, r2, r3, r4)
      if (op->est->cam->do_calib_int && op->sim->do_perturb) {
        for (int r = 4; r < 6; r++)
          op->est->cam->intrinsics.at(i)(r) += sqrt(op->est->cam->init_cov_in_r) * w(seed_ptrb);
        for (int r = 6; r < 8; r++)
          op->est->cam->intrinsics.at(i)(r) += (sqrt(op->est->cam->init_cov_in_r) / 10.0) * w(seed_ptrb);
      }
    }
  }
}

bool Simulator::get_imu_state(double timestamp, Matrix<double, 17, 1> &imustate) {
  // Current state values
  Matrix3d R_GtoI;
  Vector3d p_IinG, w_IinI, v_IinG;

  // Get the pose, velocity, and acceleration
  //  bool success_vel = spline->get_velocity(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG);
  bool success_vel = get_imu_velocity(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG);

  // Find the bounding bias values
  bool success_bias = false;
  size_t id_loc = 0;
  for (size_t i = 0; i < bias_times.size() - 1; i++) {
    if (bias_times.at(i) < timestamp && bias_times[i + 1] >= timestamp) {
      id_loc = i;
      success_bias = true;
      break;
    }
  }

  // If failed, then that means we don't have any more spline or bias
  if (!success_vel || !success_bias) {
    return false;
  }

  // Interpolate our biases (they will be at every imu timestep)
  double lambda = (timestamp - bias_times.at(id_loc)) / (bias_times.at(id_loc + 1) - bias_times.at(id_loc));
  Vector3d true_bg_interp = (1 - lambda) * vec_bg.at(id_loc) + lambda * vec_bg.at(id_loc + 1);
  Vector3d true_ba_interp = (1 - lambda) * vec_ba.at(id_loc) + lambda * vec_ba.at(id_loc + 1);

  Matrix3d RWtoE = quat_2_Rot(op->sim->WtoE_trans.block(0, 0, 4, 1));
  Vector3d pWinE = op->sim->WtoE_trans.block(4, 0, 3, 1);

  // Finally lets create the current state
  imustate(0, 0) = timestamp;
  imustate.block(1, 0, 4, 1) = trans_gt_to_ENU ? rot_2_quat(R_GtoI * RWtoE.transpose()) : rot_2_quat(R_GtoI);
  imustate.block(5, 0, 3, 1) = trans_gt_to_ENU ? pWinE + RWtoE * p_IinG : p_IinG;
  imustate.block(8, 0, 3, 1) = trans_gt_to_ENU ? RWtoE * v_IinG : v_IinG;
  imustate.block(11, 0, 3, 1) = true_bg_interp;
  imustate.block(14, 0, 3, 1) = true_ba_interp;
  return true;
}

Vector4d Simulator::imu_rmse_nees(double time, Matrix<double, 7, 1> imu, Matrix<double, 6, 6> cov) {
  Eigen::Matrix<double, 17, 1> state_gt;
  get_imu_state(time, state_gt);

  // Quaternion error
  Eigen::Matrix<double, 4, 1> quat_gt, quat_st, quat_diff;
  quat_gt << state_gt(1, 0), state_gt(2, 0), state_gt(3, 0), state_gt(4, 0);
  quat_st << imu(0, 0), imu(1, 0), imu(2, 0), imu(3, 0);
  quat_diff = quat_multiply(quat_st, Inv(quat_gt));
  double rmse_ori = (180 / M_PI) * 2 * quat_diff.block(0, 0, 3, 1).norm();

  // Difference between positions
  double dx = imu(4, 0) - state_gt(5, 0);
  double dy = imu(5, 0) - state_gt(6, 0);
  double dz = imu(6, 0) - state_gt(7, 0);
  double rmse_pos = sqrt(dx * dx + dy * dy + dz * dz);

  // Calculate NEES values
  Eigen::Vector3d quat_diff_vec = quat_diff.block(0, 0, 3, 1);
  Eigen::Vector3d cov_vec = cov.block(0, 0, 3, 3).inverse() * 2 * quat_diff.block(0, 0, 3, 1);
  double ori_nees = 2 * quat_diff_vec.dot(cov_vec);
  Eigen::Vector3d errpos = imu.block(4, 0, 3, 1) - state_gt.block(5, 0, 3, 1);
  double pos_nees = errpos.transpose() * cov.block(3, 3, 3, 3).inverse() * errpos;

  return {rmse_ori, rmse_pos, ori_nees, pos_nees};
}

bool Simulator::get_next_imu(ImuData &imu) {

  // check turn
  string sensor_type;
  int sensor_indx;
  sim_turn(sensor_type, sensor_indx);
  if (sensor_type != "IMU")
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_imu += 1.0 / op->sim->freq_imu;
  timestamp = timestamp_last_imu;
  imu.timestamp = timestamp_last_imu;

  // Current state values
  Matrix3d R_GtoI;
  Vector3d p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG;

  // Get the pose, velocity, and acceleration
  // NOTE: we get the acceleration between our two imu
  // NOTE: this is because we are using a constant measurement model for integration
  if (!get_imu_acceleration(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG)) {
    is_running = false;
    return false;
  }

  // Transform omega and linear acceleration into imu frame
  Vector3d omega_inI = w_IinI;
  Vector3d accel_inI = R_GtoI * (a_IinG + op->sim->est_true->gravity);

  // Now add noise to these measurements
  double dt = 1.0 / op->sim->freq_imu;
  if (op->sim->remove_noise) {
    // Append the current true bias to our history
    bias_times.push_back(timestamp_last_imu);
    vec_bg.push_back(true_imu_bg);
    vec_ba.push_back(true_imu_ba);
    // Get treu IMU measurement
    imu.wm = omega_inI + true_imu_bg;
    imu.am = accel_inI + true_imu_ba;
  } else {
    // Append the current true bias to our history
    bias_times.push_back(timestamp_last_imu);
    vec_bg.push_back(true_imu_bg);
    vec_ba.push_back(true_imu_ba);

    // Add noise
    std::normal_distribution<double> w(0, 1);
    imu.wm(0) = omega_inI(0) + true_imu_bg(0) + op->sim->est_true->imu->sigma_w / sqrt(dt) * w(seed_imu);
    imu.wm(1) = omega_inI(1) + true_imu_bg(1) + op->sim->est_true->imu->sigma_w / sqrt(dt) * w(seed_imu);
    imu.wm(2) = omega_inI(2) + true_imu_bg(2) + op->sim->est_true->imu->sigma_w / sqrt(dt) * w(seed_imu);
    imu.am(0) = accel_inI(0) + true_imu_ba(0) + op->sim->est_true->imu->sigma_a / sqrt(dt) * w(seed_imu);
    imu.am(1) = accel_inI(1) + true_imu_ba(1) + op->sim->est_true->imu->sigma_a / sqrt(dt) * w(seed_imu);
    imu.am(2) = accel_inI(2) + true_imu_ba(2) + op->sim->est_true->imu->sigma_a / sqrt(dt) * w(seed_imu);

    // Move the biases forward in time
    true_imu_bg(0) += op->sim->est_true->imu->sigma_wb * sqrt(dt) * w(seed_imu);
    true_imu_bg(1) += op->sim->est_true->imu->sigma_wb * sqrt(dt) * w(seed_imu);
    true_imu_bg(2) += op->sim->est_true->imu->sigma_wb * sqrt(dt) * w(seed_imu);
    true_imu_ba(0) += op->sim->est_true->imu->sigma_ab * sqrt(dt) * w(seed_imu);
    true_imu_ba(1) += op->sim->est_true->imu->sigma_ab * sqrt(dt) * w(seed_imu);
    true_imu_ba(2) += op->sim->est_true->imu->sigma_ab * sqrt(dt) * w(seed_imu);
  }

  // Return success
  PRINT1("[SIM] IMU measurement: %.3f", imu.timestamp);
  PRINT1("|%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n", imu.wm(0), imu.wm(1), imu.wm(2), imu.am(0), imu.am(1), imu.am(2));
  return true;
}

bool Simulator::get_next_cam(CamSimData &cam) {
  // check turn
  string sensor_type;
  int sensor_indx;
  sim_turn(sensor_type, sensor_indx);
  if (sensor_type != "CAM")
    return false;

  // check if this is a stereo cam
  vector<int> cam_ids = {sensor_indx};
  if (op->sim->est_true->cam->use_stereo && op->sim->est_true->cam->stereo_pairs.find(sensor_indx) != op->sim->est_true->cam->stereo_pairs.end())
    cam_ids.push_back(op->sim->est_true->cam->stereo_pairs.at(sensor_indx));

  // Else lets do a new measurement!!!
  timestamp_last_cams.at(cam_ids.at(0)) += 1.0 / op->sim->freq_cam;
  if (op->sim->est_true->cam->use_stereo && cam_ids.size() == 2)
    timestamp_last_cams.at(cam_ids.at(1)) = timestamp_last_cams.at(cam_ids.at(0));
  timestamp = timestamp_last_cams.at(cam_ids.at(0));
  cam.time = timestamp_last_cams.at(cam_ids.at(0)) - op->sim->est_true->cam->dt.at(cam_ids.at(0));

  // Get the pose at the current timestep
  Matrix3d R_GtoI;
  Vector3d p_IinG;
  if (!get_imu_pose(timestamp, R_GtoI, p_IinG)) {
    is_running = false;
    return false;
  }

  for (auto cam_id : cam_ids) {
    // Get the uv features for this frame
    vector<pair<size_t, VectorXf>> uvs = project_pointcloud(R_GtoI, p_IinG, cam_id);

    // If we do not have enough, generate more
    if ((int)uvs.size() < op->sim->est_true->cam->n_pts) {
      PRINT1(YELLOW "[SIM]: cam %d not enough features (%d < %d)\n" RESET, (int)cam_id, (int)uvs.size(), op->sim->est_true->cam->n_pts);
    }

    // If greater than only select the first set
    if ((int)uvs.size() > op->sim->est_true->cam->n_pts) {
      uvs.erase(uvs.begin() + op->sim->est_true->cam->n_pts, uvs.end());
    }

    // Append the map size so all cameras have unique features in them (but the same map)
    for (auto &uv : uvs) {
      uv.first += cam_ids.at(0) * cam_featmap.size();
    }

    // Loop through and add noise to each uv measurement
    if (!op->sim->remove_noise) {
      std::normal_distribution<double> w(0, 1);
      for (auto &uv : uvs) {
        uv.second(0) += op->sim->est_true->cam->sigma_pix * w(seed_cams.at(cam_id));
        uv.second(1) += op->sim->est_true->cam->sigma_pix * w(seed_cams.at(cam_id));
      }
    }

    // Push back for this camera
    cam.uvs.push_back(uvs);
    cam.ids.push_back(cam_id);
  }

  // Return success
  for (int i = 0; i < (int)cam.uvs.size(); i++) {
    PRINT1("[SIM] Cam measurement: %.3f|%d|%d\n", cam.time, cam.ids.at(i), cam.uvs.at(i).size());
  }
  return true;
}

bool Simulator::get_next_vicon(ViconData &vicon) {

  // check turn
  string sensor_type;
  sim_turn(sensor_type, vicon.id);
  if (sensor_type != "VICON")
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_vicons.at(vicon.id) += 1.0 / op->sim->freq_vicon;
  timestamp = timestamp_last_vicons.at(vicon.id);
  vicon.time = timestamp - op->sim->est_true->vicon->dt.at(vicon.id);

  // Get the pose at the current timestep
  Matrix3d R_GtoI;
  Vector3d p_IinG;
  if (!get_imu_pose(timestamp, R_GtoI, p_IinG)) {
    is_running = false;
    return false;
  }

  // Grab our extrinsic values
  Matrix<double, 3, 3> R_ItoV = quat_2_Rot(op->sim->est_true->vicon->extrinsics.at(vicon.id).block(0, 0, 4, 1));
  Matrix<double, 3, 1> p_IinV = op->sim->est_true->vicon->extrinsics.at(vicon.id).block(4, 0, 3, 1);
  Matrix<double, 3, 1> p_VinI = -R_ItoV.transpose() * p_IinV;

  // Build pose measurement
  vicon.pose.block(0, 0, 3, 1) = log_so3(R_ItoV * R_GtoI);
  vicon.pose.block(3, 0, 3, 1) = p_IinG + R_GtoI.transpose() * p_VinI;

  // add noise
  if (!op->sim->remove_noise) {
    Vector3d w_vec;
    std::normal_distribution<double> w(0, 1);
    double ptrb = op->sim->est_true->vicon->noise_o;
    w_vec << ptrb * w(seed_vicons.at(vicon.id)), ptrb * w(seed_vicons.at(vicon.id)), ptrb * w(seed_vicons.at(vicon.id));
    vicon.pose.block(0, 0, 3, 1) = log_so3(exp_so3(w_vec) * exp_so3(vicon.pose.block(0, 0, 3, 1))); // add ori noise
    for (int i = 3; i < 6; i++)
      vicon.pose(i) += op->sim->est_true->vicon->noise_p * w(seed_vicons.at(vicon.id)); // add pos noise
  }

  // Return success
  PRINT1("[SIM] VICON measurement: %.3f|%d|", vicon.time, vicon.id);
  PRINT1("%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n", vicon.pose(0), vicon.pose(1), vicon.pose(2), vicon.pose(3), vicon.pose(4), vicon.pose(5));
  return true;
}

bool Simulator::get_next_gps(GPSData &gps) {

  // check turn
  string sensor_type;
  sim_turn(sensor_type, gps.id);
  if (sensor_type != "GPS")
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_gpss.at(gps.id) += 1.0 / op->sim->freq_gps;
  timestamp = timestamp_last_gpss.at(gps.id);
  gps.time = timestamp - op->sim->est_true->gps->dt.at(gps.id);

  // Get the pose at the current timestep
  Matrix3d R_GtoI;
  Vector3d p_IinG;
  if (!get_imu_pose(timestamp, R_GtoI, p_IinG)) {
    is_running = false;
    return false;
  }

  Matrix<double, 4, 1> q_V_to_E = op->sim->WtoE_trans.block(0, 0, 4, 1);
  Matrix<double, 3, 1> p_V_in_E = op->sim->WtoE_trans.block(4, 0, 3, 1);
  // gps is in Global. We transform imu to VIO coordinate
  gps.meas = p_V_in_E + quat_2_Rot(q_V_to_E) * (p_IinG + R_GtoI.transpose() * op->sim->est_true->gps->extrinsics.at(gps.id));

  // add noise
  if (!op->sim->remove_noise) {
    std::normal_distribution<double> noise(0, 1);
    gps.meas(0) += op->sim->est_true->gps->noise * noise(seed_gps.at(gps.id));
    gps.meas(1) += op->sim->est_true->gps->noise * noise(seed_gps.at(gps.id));
    gps.meas(2) += op->sim->est_true->gps->noise * noise(seed_gps.at(gps.id));
  }

  gps.noise << op->sim->est_true->gps->noise, op->sim->est_true->gps->noise, op->sim->est_true->gps->noise;

  // Return success
  PRINT1("[SIM] GPS measurement: %.3f|%d|", gps.time, gps.id);
  PRINT1("%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n", gps.meas(0), gps.meas(1), gps.meas(2), gps.noise(0), gps.noise(1), gps.noise(2));
  return true;
}

bool Simulator::get_next_wheel(WheelData &wheel) {
  // check turn
  int id;
  string sensor_type;
  sim_turn(sensor_type, id);
  if (sensor_type != "WHEEL")
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_wheel += 1.0 / op->sim->freq_wheel;
  timestamp = timestamp_last_wheel;
  wheel.time = timestamp - op->sim->est_true->wheel->dt;

  // Current state values
  Matrix3d RGtoO;
  Vector3d pOinG, wOinO, vOinG;

  // If failed, then that means we don't have any more spline
  // Thus we should stop the simulation
  if (!spline->get_velocity(timestamp, RGtoO, pOinG, wOinO, vOinG)) {
    is_running = false;
    return false;
  }

  double dt = 1.0 / op->sim->freq_wheel;
  std::normal_distribution<double> noise(0, 1);

  double v = (RGtoO * vOinG)(0);
  double w = wOinO(2);
  double rl = op->sim->est_true->wheel->intrinsics(0);
  double rr = op->sim->est_true->wheel->intrinsics(1);
  double b = op->sim->est_true->wheel->intrinsics(2);

  // Now, formulate measurements depend on what type of measurement we want
  if (op->sim->est_true->wheel->type == "Wheel2DAng" || op->sim->est_true->wheel->type == "Wheel3DAng") {
    // compute each wheels angular velocity
    double wl = (2 * v - b * w) / 2 / rl;
    double wr = (2 * v + b * w) / 2 / rr;
    if (!op->sim->remove_noise) {
      wl += op->sim->est_true->wheel->noise_w / sqrt(dt) * noise(seed_wheel);
      wr += op->sim->est_true->wheel->noise_w / sqrt(dt) * noise(seed_wheel);
    }
    wheel.m1 = wl;
    wheel.m2 = wr;
  } else if (op->sim->est_true->wheel->type == "Wheel2DLin" || op->sim->est_true->wheel->type == "Wheel3DLin") {
    // compute each wheels linear velocity
    double vl = (2 * v - b * w) / 2;
    double vr = (2 * v + b * w) / 2;
    if (!op->sim->remove_noise) {
      vl += op->sim->est_true->wheel->noise_v / sqrt(dt) * noise(seed_wheel);
      vr += op->sim->est_true->wheel->noise_v / sqrt(dt) * noise(seed_wheel);
    }
    wheel.m1 = vl;
    wheel.m2 = vr;
  } else if (op->sim->est_true->wheel->type == "Wheel2DCen" || op->sim->est_true->wheel->type == "Wheel3DCen") {
    if (!op->sim->remove_noise) {
      v += op->sim->est_true->wheel->noise_v / sqrt(dt) * noise(seed_wheel);
      w += op->sim->est_true->wheel->noise_w / sqrt(dt) * noise(seed_wheel);
    }
    wheel.m1 = w;
    wheel.m2 = v;
  } else {
    PRINT4("No valid wheel measurement type selected\n");
    exit(EXIT_FAILURE);
  }
  // pass success
  PRINT1("[SIM] Wheel measurement: %.3f|%s|%.3f,%.3f\n", wheel.time, op->sim->est_true->wheel->type.c_str(), wheel.m1, wheel.m2);
  return true;
}

bool Simulator::get_next_lidar(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar) {

  // check turn
  string sensor_type;
  int lidar_id = -1;
  sim_turn(sensor_type, lidar_id);
  if (sensor_type != "LIDAR")
    return false;

  // Else lets do a new measurement!!!
  timestamp_last_lidars.at(lidar_id) += 1.0 / op->sim->freq_lidar;
  timestamp = timestamp_last_lidars.at(lidar_id);

  if (!get_lidar_pointcloud(lidar, timestamp, lidar_id, op->sim->est_true->lidar))
    return false;

  // Success!
  PRINT1("[SIM] LiDAR measurement: %.3f|%d|%d\n", timestamp - op->sim->est_true->lidar->dt.at(lidar_id), lidar_id, lidar->points.size());
  return true;
}

bool Simulator::get_lidar_pointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar, double time, int id, std::shared_ptr<OptionsLidar> lidar_op) {
  // Lidar id and timestamp
  lidar->header.frame_id = to_string(id);
  lidar->header.stamp = (unsigned long)((time - lidar_op->dt.at(id)) * 1000); // Delivered in micro second

  // Get the pose at the current timestep
  Matrix3d R_GtoI;
  Vector3d p_IinG;
  if (!get_imu_pose(time, R_GtoI, p_IinG)) {
    is_running = false;
    return false;
  }

  // Grab our extrinsic values
  Matrix<double, 3, 3> R_ItoL = quat_2_Rot(lidar_op->extrinsics.at(id).block(0, 0, 4, 1));
  Matrix<double, 3, 1> p_IinL = lidar_op->extrinsics.at(id).block(4, 0, 3, 1);
  Matrix<double, 3, 1> p_LinI = -R_ItoL.transpose() * p_IinL;
  // Loop through each ray and create pointcloud
  int cnt = 0;
  int total = lidar_op->v_angles.at(id).size() * lidar_op->h_angles.at(id).size();
  lidar->points.reserve(total);
  for (auto v_angle : lidar_op->v_angles.at(id)) {
    for (auto h_angle : lidar_op->h_angles.at(id)) {

      // Refresh true IMU pose if motion (un)distortion is enabled, to create distortion of due to motion
      if (lidar_op->raw_remove_motion_blur) {
        double dt = (total - cnt) * lidar_op->raw_point_dt;
        get_imu_pose(time - dt, R_GtoI, p_IinG);
        cnt++;
      }

      // Ray in the local frame of reference
      Matrix<double, 6, 1> ray;
      ray.head(3) = p_IinG + R_GtoI.transpose() * p_LinI;
      ray(3) = sin(M_PI / 180.0 * (90.0 - v_angle)) * cos(M_PI / 180.0 * h_angle);
      ray(4) = sin(M_PI / 180.0 * (90.0 - v_angle)) * sin(M_PI / 180.0 * h_angle);
      ray(5) = cos(M_PI / 180.0 * (90.0 - v_angle));
      ray.tail(3) = R_GtoI.transpose() * R_ItoL.transpose() * ray.tail(3);
      double th = M_PI / 180.0 * h_angle;
      double ph = M_PI / 180 * (90 - v_angle);

      // Do intersection with all planes in our environment
      double d = INFINITY; // distance to the plane
      for (auto &plane : lidar_planes) {
        double tmp_range = 0.0;
        if (plane->calculate_intersection(ray, tmp_range)) {
          d = min(d, tmp_range);
        }
      }

      // continue if the point is out of range
      if (d > lidar_op->max_range || d < lidar_op->min_range)
        continue;

      // create point
      pcl::PointXYZ p;
      p.x = d * sin(ph) * cos(th);
      p.y = d * sin(ph) * sin(th);
      p.z = d * cos(ph);

      // Add noise
      std::normal_distribution<float> w(0, 1);
      if (!op->sim->remove_noise) {
        p.x += (float)lidar_op->raw_noise * w(seed_lidars.at(id));
        p.y += (float)lidar_op->raw_noise * w(seed_lidars.at(id));
        p.z += (float)lidar_op->raw_noise * w(seed_lidars.at(id));
      }

      // Append to the lidar pointcloud (by ring id)
      lidar->points.push_back(p);
    }
  }
  return true;
}

vector<pair<size_t, VectorXf>> Simulator::project_pointcloud(const Matrix3d &R_GtoI, const Vector3d &p_IinG, int camid) {

  // Assert we have good camera
  assert(camid < op->sim->est_true->cam->max_n);
  assert((int)op->sim->est_true->cam->intrinsics.size() == op->sim->est_true->cam->max_n);
  assert((int)op->sim->est_true->cam->extrinsics.size() == op->sim->est_true->cam->max_n);

  // Grab our extrinsic and intrinsic values
  Matrix<double, 3, 3> R_ItoC = quat_2_Rot(op->sim->est_true->cam->extrinsics.at(camid).block(0, 0, 4, 1));
  Matrix<double, 3, 1> p_IinC = op->sim->est_true->cam->extrinsics.at(camid).block(4, 0, 3, 1);
  shared_ptr<CamBase> camera;
  if (op->sim->est_true->cam->distortion_model.at(camid) == "equidistant") {
    camera = std::make_shared<ov_core::CamEqui>(op->sim->est_true->cam->wh.at(camid)[0], op->sim->est_true->cam->wh.at(camid)[1]);
    camera->set_value(op->sim->est_true->cam->intrinsics.at(camid));
  } else {
    camera = std::make_shared<ov_core::CamRadtan>(op->sim->est_true->cam->wh.at(camid)[0], op->sim->est_true->cam->wh.at(camid)[1]);
    camera->set_value(op->sim->est_true->cam->intrinsics.at(camid));
  }

  // Our projected uv true measurements
  vector<pair<size_t, VectorXf>> uvs;

  // Loop through our map
  for (const auto &feat : cam_featmap) {

    // Transform feature into current camera frame
    Vector3d p_FinI = R_GtoI * (feat.second - p_IinG);
    Vector3d p_FinC = R_ItoC * p_FinI + p_IinC;

    // Skip cloud if too far away
    if (p_FinC(2) > op->sim->max_feature_gen_distance || p_FinC(2) < 0.1)
      continue;

    // Project to normalized coordinates
    Vector2f uv_norm;
    uv_norm << (float)(p_FinC(0) / p_FinC(2)), (float)(p_FinC(1) / p_FinC(2));

    // Distort the normalized coordinates
    Vector2f uv_dist;
    uv_dist = camera->distort_f(uv_norm);

    // Check that it is inside our bounds
    if (uv_dist(0) < 0 || uv_dist(0) > camera->w() || uv_dist(1) < 0 || uv_dist(1) > camera->h()) {
      continue;
    }

    // Else we can add this as a good projection
    uvs.push_back({feat.first, uv_dist});
  }

  // Return our projections
  return uvs;
}

void Simulator::generate_points(const Matrix3d &R_GtoI, const Vector3d &p_IinG, int camid, unordered_map<size_t, Vector3d> &feats, int numpts) {

  // Assert we have good camera
  assert(camid < op->sim->est_true->cam->max_n);
  assert((int)op->sim->est_true->cam->intrinsics.size() == op->sim->est_true->cam->max_n);
  assert((int)op->sim->est_true->cam->extrinsics.size() == op->sim->est_true->cam->max_n);

  // Grab our extrinsic and intrinsic values
  Matrix<double, 3, 3> R_ItoC = quat_2_Rot(op->sim->est_true->cam->extrinsics.at(camid).block(0, 0, 4, 1));
  Matrix<double, 3, 1> p_IinC = op->sim->est_true->cam->extrinsics.at(camid).block(4, 0, 3, 1);
  shared_ptr<CamBase> camera;
  if (op->sim->est_true->cam->distortion_model.at(camid) == "equidistant") {
    camera = std::make_shared<ov_core::CamEqui>(op->sim->est_true->cam->wh.at(camid)[0], op->sim->est_true->cam->wh.at(camid)[1]);
    camera->set_value(op->sim->est_true->cam->intrinsics.at(camid));
  } else {
    camera = std::make_shared<ov_core::CamRadtan>(op->sim->est_true->cam->wh.at(camid)[0], op->sim->est_true->cam->wh.at(camid)[1]);
    camera->set_value(op->sim->est_true->cam->intrinsics.at(camid));
  }

  // Generate the desired number of features
  for (int i = 0; i < numpts; i++) {

    // Uniformly randomly generate within our fov
    uniform_real_distribution<double> gen_u(0, camera->w());
    uniform_real_distribution<double> gen_v(0, camera->h());
    double u_dist = gen_u(seed_init);
    double v_dist = gen_v(seed_init);

    // Convert to opencv format
    cv::Point2f uv_dist((float)u_dist, (float)v_dist);

    // Undistort this point to our normalized coordinates
    cv::Point2f uv_norm;
    uv_norm = camera->undistort_cv(uv_dist);

    // Generate a random depth
    uniform_real_distribution<double> gen_depth(op->sim->min_feature_gen_distance, op->sim->max_feature_gen_distance);
    double depth = gen_depth(seed_init);

    // Get the 3d point
    Vector3d bearing;
    bearing << uv_norm.x, uv_norm.y, 1;
    Vector3d p_FinC;
    p_FinC = depth * bearing;

    // Move to the global frame of reference
    Vector3d p_FinI = R_ItoC.transpose() * (p_FinC - p_IinC);
    Vector3d p_FinG = R_GtoI.transpose() * p_FinI + p_IinG;

    // Append this as a new feature
    cam_featmap.insert({cam_feat_id, p_FinG});
    cam_feat_id++;
  }
}

bool Simulator::get_imu_pose(double timestamp, Matrix3d &R_GtoI, Vector3d &p_IinG) {
  if (!op->sim->est_true->wheel->enabled)
    return spline->get_pose(timestamp, R_GtoI, p_IinG);
  // When we use wheel odoemtry, the default pose is wheel odometry.
  // Therefore using extrinsic of wheel-IMU, we find IMU pose
  Matrix3d R_GtoW, R_ItoW;
  Vector3d p_WinG, p_IinW;
  bool success_pose = spline->get_pose(timestamp, R_GtoW, p_WinG);
  R_ItoW = quat_2_Rot(op->sim->est_true->wheel->extrinsics.block(0, 0, 4, 1));
  p_IinW = op->sim->est_true->wheel->extrinsics.block(4, 0, 3, 1);
  R_GtoI = R_ItoW.transpose() * R_GtoW;
  p_IinG = p_WinG + R_GtoW.transpose() * p_IinW;
  return success_pose;
}

bool Simulator::get_imu_velocity(double timestamp, Matrix3d &R_GtoI, Vector3d &p_IinG, Vector3d &w_IinI, Vector3d &v_IinG) {
  if (!op->sim->est_true->wheel->enabled)
    return spline->get_velocity(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG);
  // When we use wheel odoemtry, the default pose is wheel odometry.
  // Therefore using extrinsic of wheel-IMU, we find IMU pose
  Matrix3d R_GtoO;
  Vector3d p_OinG, w_OinO, v_OinG;
  bool success_vel = spline->get_velocity(timestamp, R_GtoO, p_OinG, w_OinO, v_OinG);
  Matrix3d R_ItoO = quat_2_Rot(op->sim->est_true->wheel->extrinsics.block(0, 0, 4, 1));
  Vector3d p_IinO = op->sim->est_true->wheel->extrinsics.block(4, 0, 3, 1);
  R_GtoI = R_ItoO.transpose() * R_GtoO;
  p_IinG = p_OinG + R_GtoO.transpose() * p_IinO;
  w_IinI = R_ItoO.transpose() * w_OinO;
  v_IinG = v_OinG + R_GtoO.transpose() * skew_x(w_OinO) * p_IinO;
  return success_vel;
}

bool Simulator::get_imu_acceleration(double timestamp, Matrix3d &R_GtoI, Vector3d &p_IinG, Vector3d &w_IinI, Vector3d &v_IinG, Vector3d &alpha_IinI, Vector3d &a_IinG) {
  if (!op->sim->est_true->wheel->enabled)
    return spline->get_acceleration(timestamp, R_GtoI, p_IinG, w_IinI, v_IinG, alpha_IinI, a_IinG);

  Matrix3d R_GtoO;
  Vector3d p_OinG, w_OinO, v_OinG, alpha_OinO, a_OinG;
  bool success_accel = spline->get_acceleration(timestamp, R_GtoO, p_OinG, w_OinO, v_OinG, alpha_OinO, a_OinG);
  Matrix3d R_ItoO = quat_2_Rot(op->sim->est_true->wheel->extrinsics.block(0, 0, 4, 1));
  Vector3d p_IinO = op->sim->est_true->wheel->extrinsics.block(4, 0, 3, 1);
  R_GtoI = R_ItoO.transpose() * R_GtoO;
  p_IinG = p_OinG + R_GtoO.transpose() * p_IinO;
  w_IinI = R_ItoO.transpose() * w_OinO;
  v_IinG = v_OinG + R_GtoO.transpose() * skew_x(w_OinO) * p_IinO;

  // Also compute the rotation of epsilon timestep
  Matrix3d R;
  Vector3d p, w;
  Vector3d v_IinG3, v_IinG2, v_IinG1, v_IinG0;
  if (!get_imu_velocity(timestamp + 2 * spline->epsilon, R, p, w, v_IinG3))
    return false;
  if (!get_imu_velocity(timestamp + spline->epsilon, R, p, w, v_IinG2))
    return false;
  if (!get_imu_velocity(timestamp - spline->epsilon, R, p, w, v_IinG1))
    return false;
  if (!get_imu_velocity(timestamp - 2 * spline->epsilon, R, p, w, v_IinG0))
    return false;

  a_IinG = (-v_IinG3 + 8 * v_IinG2 - 8 * v_IinG1 + v_IinG0) / 12 / spline->epsilon;

  return success_accel;
}

bool Simulator::load_plane_data(string path_planes) {

  // Try to open our groundtruth file
  ifstream file;
  file.open(path_planes);
  if (!file) {
    PRINT4("ERROR: Unable to open simulation plane file...");
    PRINT4("ERROR: %s", path_planes.c_str());
    return false;
  }

  // Debug print
  string base_filename = path_planes.substr(path_planes.find_last_of("/\\") + 1);
  PRINT3("[SIM]: loaded planes %s\n", base_filename.c_str());

  // Loop through each line of this file
  string current_line;
  double units_to_metric = 1.0;
  vector<vector<double>> good_planes;
  while (getline(file, current_line)) {

    // Skip if we start with a comment
    if (!current_line.find("#"))
      continue;

    // Loop variables
    istringstream s(current_line);
    string field;
    vector<double> data;

    // Loop through this line (start_x, start_y, end_x, end_y)
    while (getline(s, field, ',')) {
      // Skip if empty
      if (field.empty())
        continue;
      // save the data to our vector
      data.push_back(atof(field.c_str()));
    }

    // We have the conversion factor between the units of the file and meters
    if (data.size() == 1)
      units_to_metric = data.at(0);

    // A 2d line on our floor plan, lets create the plane for it
    if (data.size() == 4)
      good_planes.push_back(data);
  }

  if (good_planes.empty()) {
    PRINT4("ERROR: There is no plane from file...");
    return false;
  }

  // Finally close the file
  file.close();

  // Min and max xy locations so we can add a floor to our system
  double min_x = INFINITY;
  double max_x = -INFINITY;
  double min_y = INFINITY;
  double max_y = -INFINITY;

  // Loop through all the good planes and add them
  for (auto &data : good_planes) {

    // Min max of all units
    min_x = min(data.at(0), min_x);
    min_x = min(data.at(2), min_x);
    max_x = max(data.at(0), max_x);
    max_x = max(data.at(2), max_x);
    min_y = min(data.at(1), min_y);
    min_y = min(data.at(3), min_y);
    max_y = max(data.at(1), max_y);
    max_y = max(data.at(3), max_y);

    // Calculate the points and add the plane
    Vector3d pt_tl, pt_tr, pt_bl, pt_br;
    pt_tl << units_to_metric * data.at(0), units_to_metric * data.at(1), 4;
    pt_tr << units_to_metric * data.at(2), units_to_metric * data.at(3), 4;
    pt_bl << units_to_metric * data.at(0), units_to_metric * data.at(1), -1;
    pt_br << units_to_metric * data.at(2), units_to_metric * data.at(3), -1;
    lidar_planes.push_back(make_shared<SimulationPlane>(pt_tl, pt_tr, pt_bl, pt_br));
  }

  // Error if we don't have any data
  if (lidar_planes.empty()) {
    PRINT4("ERROR: Could not parse any data from the file!!");
    PRINT4("ERROR: %s", base_filename.c_str());
    exit(EXIT_FAILURE);
  }

  // Finally, lets add the floor and ceiling
  Vector3d pt_tl, pt_tr, pt_bl, pt_br;
  pt_tl << units_to_metric * min_x, units_to_metric * min_y, -1;
  pt_tr << units_to_metric * min_x, units_to_metric * max_y, -1;
  pt_bl << units_to_metric * max_x, units_to_metric * min_y, -1;
  pt_br << units_to_metric * max_x, units_to_metric * max_y, -1;
  lidar_planes.push_back(make_shared<SimulationPlane>(pt_tl, pt_tr, pt_bl, pt_br));
  pt_tl << units_to_metric * min_x, units_to_metric * min_y, 4;
  pt_tr << units_to_metric * min_x, units_to_metric * max_y, 4;
  pt_bl << units_to_metric * max_x, units_to_metric * min_y, 4;
  pt_br << units_to_metric * max_x, units_to_metric * max_y, 4;
  lidar_planes.push_back(make_shared<SimulationPlane>(pt_tl, pt_tr, pt_bl, pt_br));

  PRINT3("[SIM]: loaded %d planes, scale: %.5f from the floorplan\n", (int)lidar_planes.size(), units_to_metric);

  // success
  return true;
}

void Simulator::sim_turn(string &sensor_type, int &sensor_indx) {

  // find the minimum next measurement time and corresponding sensor type & id
  // Start with IMU
  double sensor_time = timestamp_last_imu + 1.0 / op->sim->freq_imu;
  sensor_type = "IMU";
  sensor_indx = 0;

  // cam
  if (op->sim->est_true->cam->enabled) {
    for (int i = 0; i < op->sim->est_true->cam->max_n; i++) {
      if (sensor_time > timestamp_last_cams.at(i) + 1.0 / op->sim->freq_cam) {
        sensor_type = "CAM";
        sensor_time = timestamp_last_cams.at(i) + 1.0 / op->sim->freq_cam;
        sensor_indx = i;
      }
    }
  }

  // vicon
  if (op->sim->est_true->vicon->enabled) {
    for (int i = 0; i < op->sim->est_true->vicon->max_n; i++) {
      if (sensor_time > timestamp_last_vicons.at(i) + 1.0 / op->sim->freq_vicon) {
        sensor_type = "VICON";
        sensor_time = timestamp_last_vicons.at(i) + 1.0 / op->sim->freq_vicon;
        sensor_indx = i;
      }
    }
  }

  // gps
  if (op->sim->est_true->gps->enabled) {
    for (int i = 0; i < op->sim->est_true->gps->max_n; i++) {
      if (sensor_time > timestamp_last_gpss.at(i) + 1.0 / op->sim->freq_gps) {
        sensor_type = "GPS";
        sensor_time = timestamp_last_gpss.at(i) + 1.0 / op->sim->freq_gps;
        sensor_indx = i;
      }
    }
  }

  // lidar
  if (op->sim->est_true->lidar->enabled) {
    for (int i = 0; i < op->sim->est_true->lidar->max_n; i++) {
      if (sensor_time > timestamp_last_lidars.at(i) + 1.0 / op->sim->freq_lidar) {
        sensor_type = "LIDAR";
        sensor_time = timestamp_last_lidars.at(i) + 1.0 / op->sim->freq_lidar;
        sensor_indx = i;
      }
    }
  }

  // wheel
  if (op->sim->est_true->wheel->enabled) {
    if (sensor_time > timestamp_last_wheel + 1.0 / op->sim->freq_wheel) {
      sensor_type = "WHEEL";
      sensor_time = timestamp_last_wheel + 1.0 / op->sim->freq_wheel;
      sensor_indx = 0;
    }
  }
  assert(timestamp <= sensor_time);
}

vector<Vector3d> Simulator::get_cam_map_vec() {
  vector<Vector3d> feats;
  for (auto const &feat : cam_featmap)
    feats.push_back(feat.second);
  return feats;
}

void Simulator::generate_planes() {
  PRINT4("ERROR: Generating planes around the trajectory.");

  // Calculate min and max of the trajectory
  Eigen::Vector3d min = INFINITY * Eigen::Vector3d::Ones();
  Eigen::Vector3d max = -INFINITY * Eigen::Vector3d::Ones();
  for (size_t i = 0; i < traj_data.size() - 1; i++) {
    if (traj_data.at(i)(0) < spline->start_time)
      continue;
    Eigen::Vector3d pos = traj_data.at(i).block(1, 0, 3, 1);
    min = min.cwiseMin(pos);
    max = max.cwiseMax(pos);
  }
  double multi_xy = 2; // 0.7
  double multi_z = 1;  // 0.24

  min.block(0, 0, 2, 1) -= multi_xy * op->sim->min_feature_gen_distance * Eigen::Vector2d::Ones();
  min(2) -= multi_z * op->sim->min_feature_gen_distance;
  max.block(0, 0, 2, 1) += multi_xy * op->sim->min_feature_gen_distance * Eigen::Vector2d::Ones();
  max(2) += multi_z * op->sim->min_feature_gen_distance;
  Eigen::Vector3d delta = max - min;

  // Each corner of the cube (bottom, then top)
  //(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 1), (1, 0, 1), (1, 1, 1),  (0, 1, 1)
  Eigen::Vector3d b1 = Eigen::Vector3d(min(0), min(1), min(2));
  Eigen::Vector3d b2 = Eigen::Vector3d(min(0) + delta(0), min(1), min(2));
  Eigen::Vector3d b3 = Eigen::Vector3d(min(0), min(1) + delta(1), min(2));
  Eigen::Vector3d b4 = Eigen::Vector3d(min(0) + delta(0), min(1) + delta(1), min(2));
  Eigen::Vector3d t1 = Eigen::Vector3d(b1(0), b1(1), b1(2) + delta(2));
  Eigen::Vector3d t2 = Eigen::Vector3d(b2(0), b2(1), b2(2) + delta(2));
  Eigen::Vector3d t3 = Eigen::Vector3d(b3(0), b3(1), b3(2) + delta(2));
  Eigen::Vector3d t4 = Eigen::Vector3d(b4(0), b4(1), b4(2) + delta(2));

  // Create our *cube* around the trajectory
  // Eigen::Vector3d &_pt_top_left, Eigen::Vector3d &_pt_top_right, Eigen::Vector3d &_pt_bottom_left, Eigen::Vector3d &_pt_bottom_right
  lidar_planes.push_back(make_shared<SimulationPlane>(b1, b2, b3, b4));
  lidar_planes.push_back(make_shared<SimulationPlane>(t3, t4, t2, t1));
  lidar_planes.push_back(make_shared<SimulationPlane>(t3, t1, b3, b1));
  lidar_planes.push_back(make_shared<SimulationPlane>(t1, t2, b1, b2));
  lidar_planes.push_back(make_shared<SimulationPlane>(t2, t4, b2, b4));
  lidar_planes.push_back(make_shared<SimulationPlane>(t4, t3, b4, b3));
}
std::vector<std::shared_ptr<SimulationPlane>> Simulator::get_lidar_planes() { return lidar_planes; }
std::unordered_map<size_t, Vector3d> Simulator::get_cam_map() { return cam_featmap; }

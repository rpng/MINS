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

#include "Initializer.h"
#include "feat/FeatureDatabase.h"
#include "imu/I_Initializer.h"
#include "imu_wheel/IW_Initializer.h"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsInit.h"
#include "options/OptionsLidar.h"
#include "options/OptionsSimulation.h"
#include "options/OptionsWheel.h"
#include "sim/Simulator.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "update/cam/UpdaterCamera.h"
#include "update/gps/GPSTypes.h"
#include "update/gps/PoseJPL_4DOF.h"
#include "update/gps/UpdaterGPS.h"
#include "update/lidar/LidarHelper.h"
#include "update/lidar/LidarTypes.h"
#include "update/lidar/UpdaterLidar.h"
#include "update/lidar/ikd_Tree.h"
#include "update/wheel/UpdaterWheel.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include "utils/TimeChecker.h"
#include "utils/dataset_reader.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;
using namespace Eigen;
using namespace mins;
using namespace ov_core;

Initializer::Initializer(shared_ptr<State> state, PP pp_imu, UP_WHL up_whl, UP_GPS up_gps, UP_CAM up_cam, UP_LDR up_ldr, SIM sim)
    : sim(sim), pp_imu(pp_imu), up_gps(up_gps), up_whl(up_whl), up_ldr(up_ldr), up_cam(up_cam), state(state) {

  tc = make_shared<TimeChecker>();

  // Initialize with ground truth
  if (state->op->init->use_gt) {
    PRINT2(GREEN "[INIT]: Initialize with ground truth.\n" RESET);
    return;
  }

  // Initialize with IMU-wheel
  if (!state->op->init->imu_only_init && state->op->wheel->enabled) {
    PRINT2("[INIT]: Initialize with imu-wheel.\n");
    shared_ptr<IW_Initializer::IW_Initializer_Options> op = make_shared<IW_Initializer::IW_Initializer_Options>();
    op->wheel_extrinsic = state->wheel_extrinsic;
    op->wheel_intrinsic = state->wheel_intrinsic;
    op->wheel_dt = state->wheel_dt;
    op->wheel_type = state->op->wheel->type;
    op->threshold = state->op->init->imu_wheel_thresh;
    op->gravity = state->op->gravity;
    op->imu_gravity_aligned = state->op->init->imu_gravity_aligned;
    iw_init = make_shared<IW_Initializer>(op, pp_imu, up_whl);
    return;
  }

  // Initialize with IMU only
  PRINT2(GREEN "[INIT]: Initialize with imu.\n" RESET);
  i_init = make_shared<I_Initializer>(pp_imu, state->op);
}

bool Initializer::try_initializtion() {
  if (tc->counter == 0) {
    tc->ding("[INIT]: Total initialization time");
    tc->counter++;
  }
  // IMU information to be initialized
  Matrix<double, 17, 1> imustate;
  bool init_success;
  if (state->op->init->use_gt) // perform ground truth initialization
    init_success = gt_initialization(imustate);
  else if (!state->op->init->imu_only_init && state->op->wheel->enabled) // perform IMU-Wheel initialization
    init_success = iw_init->initialization(imustate);
  else // perform IMU initialization
    init_success = i_init->initialization(imustate);

  // return if init failed
  if (!init_success) {
    delete_old_measurements();
    return false;
  }

  // Success! Set the state variables
  set_state(imustate);

  // Also initialize GNSS and LiDAR if enabled
  state->op->init->use_gt_gnss ? init_gnss_sim() : void();
  state->op->init->use_gt_lidar ? init_lidar_sim() : void();
  return true;
}

bool Initializer::gt_initialization(Eigen::Matrix<double, 17, 1> &imustate) {
  // Initialize our filter with the ground truth
  if (sim != nullptr) // Initialize with simulation
    return sim->get_imu_state(sim->timestamp, imustate);
  else if (!state->op->init->path_gt.empty()) { // Initialize with ground truth file
    map<double, Eigen::Matrix<double, 17, 1>> gt_states;
    ov_core::DatasetReader::load_gt_file(state->op->init->path_gt, gt_states);
    if (ov_core::DatasetReader::get_gt_state(pp_imu->imu_data.front().timestamp, imustate, gt_states)) {
      return true;
    } else {
      pp_imu->imu_data.erase(pp_imu->imu_data.begin());
      return false;
    }
  }
  PRINT2(RED "Tried to initialize from ground truth but both simulator and gt file not available.\n");
  std::exit(EXIT_FAILURE);
  return false;
}

void Initializer::delete_old_measurements() {
  // also delete old measurements
  if (pp_imu->imu_data.back().timestamp - pp_imu->imu_data.front().timestamp > 3 * state->op->init->window_time) {
    double old_time = pp_imu->imu_data.back().timestamp - 3 * state->op->init->window_time;

    // IMU
    int del_imu = 0;
    for (auto data = pp_imu->imu_data.begin(); !pp_imu->imu_data.empty() && (*data).timestamp < old_time;) {
      del_imu++;
      data = pp_imu->imu_data.erase(data);
    }
    del_imu > 0 ? PRINT1(YELLOW "[INIT]: Delete IMU stack. Del: %d, Remain: %d\n" RESET, del_imu, pp_imu->imu_data.size()) : void();

    // Camera
    if (state->op->cam->enabled) {
      // delete camera measurements in 1/100 cam Hz because it is costy
      double cam_hz = (up_cam->t_hist.begin()->second.size() - 1) / (up_cam->t_hist.begin()->second.back() - up_cam->t_hist.begin()->second.front());
      if (last_cam_delete_t + 100.0 / cam_hz < old_time) {
        for (int i = 0; i < up_cam->trackDATABASE.size(); i++) {
          auto db = up_cam->trackDATABASE.at(i);
          auto tr = up_cam->trackFEATS.at(i)->get_feature_database();
          int db_sz = db->size();
          int tk_sz = tr->size();
          db->cleanup_measurements(old_time);
          tr->cleanup_measurements(old_time);
          db_sz -= db->size();
          tk_sz -= tr->size();
          db_sz > 0 ? PRINT1(YELLOW "[INIT]: Delete Cam%d feat DB. Del: %d, Remain: %d\n" RESET, i, db_sz, db->size()) : void();
          tk_sz > 0 ? PRINT1(YELLOW "[INIT]: Delete Cam%d trak DB. Del: %d, Remain: %d\n" RESET, i, db_sz, tr->size()) : void();
        }
        // record cam deletion time
        last_cam_delete_t = old_time;
      }
    }

    // wheel
    if (state->op->wheel->enabled) {
      int del_whl = 0;
      for (auto data = up_whl->data_stack.begin(); !up_whl->data_stack.empty() && (*data).time < old_time;) {
        del_whl++;
        data = up_whl->data_stack.erase(data);
      }
      del_whl > 0 ? PRINT1(YELLOW "[INIT]: Delete Wheel stack. Del: %d, Remain: %d\n" RESET, del_whl, up_whl->data_stack.size()) : void();
    }

    // lidar
    if (state->op->lidar->enabled) {
      int del_ldr = 0;
      for (auto data = up_ldr->stack_lidar_raw.begin(); !up_ldr->stack_lidar_raw.empty() && (*data)->time < old_time;) {
        del_ldr++;
        data = up_ldr->stack_lidar_raw.erase(data);
      }
      del_ldr > 0 ? PRINT1(YELLOW "[INIT]: Delete LiDAR stack. Del: %d, Remain: %d\n" RESET, del_ldr, up_ldr->stack_lidar_raw.size()) : void();

      assert(up_ldr->stack_lidar_new.size() == 0);
      assert(up_ldr->stack_lidar_used.size() == 0);
    }

    // gps
    if (state->op->gps->enabled) {
      int del_gps = 0;
      for (auto data = up_gps->data_stack.begin(); !up_gps->data_stack.empty() && (*data).time < old_time;) {
        del_gps++;
        data = up_gps->data_stack.erase(data);
      }
      del_gps > 0 ? PRINT1(YELLOW "[INIT]: Delete GNSS stack. Del: %d, Remain: %d\n" RESET, del_gps, up_gps->data_stack.size()) : void();
    }
  }
}

void Initializer::set_state(Matrix<double, 17, 1> imustate) {

  // Initialize the system
  state->imu->set_value(imustate.block(1, 0, 16, 1));
  state->imu->set_fej(imustate.block(1, 0, 16, 1));

  // Fix the global yaw and position gauge freedoms
  vector<shared_ptr<ov_type::Type>> order = {state->imu};
  MatrixXd Cov = state->op->init->cov_size * MatrixXd::Identity(state->imu->size(), state->imu->size());
  StateHelper::set_initial_covariance(state, Cov, order);

  // Make velocity uncertainty a bit bigger
  //    state->cov.block(state->imu->v()->id(), state->imu->v()->id(), 3, 3) *= 2;

  // A VIO system has 4dof unobservabile directions which can be arbitrarily picked.
  // This means that on startup, we can fix the yaw and position to be 100 percent known.
  // Thus, after determining the global to current IMU orientation after initialization, we can propagate the global error
  // into the new IMU pose. In this case the position is directly equivalent, but the orientation needs to be propagated.
  //    auto q_id = state->imu->q()->id();
  //    state->cov(q_id + 2, q_id + 2) = 0.0;
  //    state->cov.block(state->imu->p()->id(), state->imu->p()->id(), 3, 3).setZero();

  // Propagate into the current local IMU frame
  // R_GtoI = R_GtoI*R_GtoG -> H = R_GtoI
  //    Matrix3d R_GtoI = quat_2_Rot(imustate.block(1, 0, 4, 1));
  //    state->cov.block(q_id, q_id, 3, 3) = R_GtoI * state->cov.block(q_id, q_id, 3, 3).eval() * R_GtoI.transpose();

  // Set the state time
  state->time = imustate(0, 0);
  state->startup_time = imustate(0, 0);
  state->initialized = true;

  // Print what we init'ed with
  auto q = state->imu->quat();
  auto p = state->imu->pos();
  auto v = state->imu->vel();
  auto bg = state->imu->bias_g();
  auto ba = state->imu->bias_a();

  PRINT2(GREEN);
  PRINT2("[INIT]: Initialized.\n");
  tc->dong("[INIT]: Total initialization time");
  tc->print("[INIT]: Total initialization time");
  PRINT2("[INIT]: time = %.4f\n", state->time);
  PRINT2("[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\n", q(0), q(1), q(2), q(3));
  PRINT2("[INIT]: position = %.4f, %.4f, %.4f\n", p(0), p(1), p(2));
  PRINT2("[INIT]: velocity = %.4f, %.4f, %.4f\n", v(0), v(1), v(2));
  PRINT2("[INIT]: bias gyro = %.4f, %.4f, %.4f\n", bg(0), bg(1), bg(2));
  PRINT2("[INIT]: bias accl = %.4f, %.4f, %.4f\n", ba(0), ba(1), ba(2));
  PRINT2(RESET);
}

void Initializer::init_gnss_sim() {
  assert(sim != nullptr);

  if (!state->op->gps->enabled)
    return;

  // Load values
  Matrix<double, 3, 3> RWtoE = quat_2_Rot(sim->op->sim->WtoE_trans.block(0, 0, 4, 1));
  Matrix<double, 3, 1> pWinE = sim->op->sim->WtoE_trans.block(4, 0, 3, 1);
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

  // Set trans_WtoE value
  Matrix<double, 7, 1> gps_state;
  gps_state.block(0, 0, 4, 1) = rot_2_quat(RWtoE);
  gps_state.block(4, 0, 3, 1) = pWinE;
  state->trans_WtoE->set_value(gps_state);

  // Set flags to be true
  up_gps->initialized = true;
  sim->trans_gt_to_ENU = true;

  // Print
  auto q = state->trans_WtoE->quat();
  auto p = state->trans_WtoE->pos();
  PRINT2(CYAN "[GPS]: ENUtoWorld transform initialized: " RESET);
  PRINT2(CYAN "q_WtoE = %.3f,%.3f,%.3f,%.3f | p_WinE = %.3f,%.3f,%.3f\n" RESET, q(0), q(1), q(2), q(3), p(0), p(1), p(2));
  PRINT2(CYAN "[GPS]: Performed state transform from World to ENU using ground truth.\n" RESET);
}

// set true initial map for LiDAR
void Initializer::init_lidar_sim() {
  assert(sim != nullptr);

  auto op = state->op->lidar;
  if (!op->enabled)
    return;

  for (int i = 0; i < op->max_n; i++) {
    // Give dense setup to create dense pointcloud
    op->v_angles.at(i).clear();
    for (int j = -90; j < 90; j++) {
      op->v_angles.at(i).push_back(j);
      op->h_angles.at(i).push_back(j);
    }

    // Get the lidar point cloud
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar(new pcl::PointCloud<pcl::PointXYZ>);
    bool success = sim->get_lidar_pointcloud(lidar, sim->timestamp - op->dt.at(i), i, op);
    assert(success);

    sim->timestamp_last_lidars.at(i) = sim->timestamp - op->dt.at(i);

    // Make sure the map is not initialized yet
    assert(!up_ldr->ikd_data.at(i)->tree->initialized());

    // Record first ever measurement time to set up reference time (ref. LidarTypes.h)
    up_ldr->FT < 0 ? up_ldr->FT = sim->timestamp - op->dt.at(i) : double();

    // Convert the lidar pointcloud into LiDARData format and initialize the map
    auto lidatdata = std::make_shared<LiDARData>(sim->timestamp - op->dt.at(i), up_ldr->FT, i, lidar, op->max_range, op->min_range);
    LidarHelper::init_map_local(lidatdata, up_ldr->ikd_data.at(i), op);
    PRINT2(CYAN "[LiDAR%d]: map initialized using ground truth.\n" RESET, i);
  }
}
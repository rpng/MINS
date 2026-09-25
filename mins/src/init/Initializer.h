/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_INITIALIZER_H
#define MINS_INITIALIZER_H

#include "Eigen/Eigen"
#include <memory>

using namespace std;

namespace mins {

class State;
class Propagator;
class Simulator;
class UpdaterWheel;
class UpdaterGPS;
class UpdaterLidar;
class UpdaterCamera;
class IW_Initializer;
class I_Initializer;
class TimeChecker;
typedef shared_ptr<Propagator> PP;
typedef shared_ptr<UpdaterWheel> UP_WHL;
typedef shared_ptr<UpdaterGPS> UP_GPS;
typedef shared_ptr<UpdaterLidar> UP_LDR;
typedef shared_ptr<UpdaterCamera> UP_CAM;
typedef shared_ptr<Simulator> SIM;

class Initializer {
public:
  /// State initializer
  Initializer(shared_ptr<State> state, PP pp_imu, UP_WHL up_whl, UP_GPS up_gps, UP_CAM up_cam, UP_LDR up_ldr, SIM sim = nullptr);

  /// Initialization
  bool try_initializtion();

private:
  /// try initialization using ground truth
  bool gt_initialization(Eigen::Matrix<double, 17, 1> &imustate);

  /// Delete too old measurements
  void delete_old_measurements();

  /// Set state after initialization success
  void set_state(Eigen::Matrix<double, 17, 1> imustate);

  /// initialize the state in GNSS reference coordinate using simulated value
  void init_gnss_sim();

  /// Give dense LiDAR pointcloud to initialize LiDAR map using simulation
  void init_lidar_sim();

  /// Simulation pointer
  shared_ptr<Simulator> sim;

  /// Initialization methods
  shared_ptr<IW_Initializer> iw_init;
  shared_ptr<I_Initializer> i_init;

  /// Each sensor handler
  shared_ptr<Propagator> pp_imu;
  shared_ptr<UpdaterGPS> up_gps;
  shared_ptr<UpdaterWheel> up_whl;
  shared_ptr<UpdaterLidar> up_ldr;
  shared_ptr<UpdaterCamera> up_cam;

  /// Last time we removed old sensor information
  double last_cam_delete_t = -1;

  /// Time recorder
  shared_ptr<TimeChecker> tc;

  /// state
  shared_ptr<State> state;
};
} // namespace mins

#endif // MINS_INITIALIZER_H

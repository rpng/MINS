/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_UPDATERLIDAR_H
#define MINS_UPDATERLIDAR_H

#include <Eigen/Eigen>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "update/lidar/PointCloud.h"

using namespace std;
using namespace Eigen;
namespace mins {
class State;
class UpdaterStatistics;
struct iKDDATA;
struct OptionsLidar;
struct LiDARData;
class UpdaterLidar {

public:
  /// LiDAR updater
  UpdaterLidar(shared_ptr<State> state);

  /// Get lidar measurement
  void feed_measurement(std::shared_ptr<mins::PointCloud<mins::PointXYZ>> lidar);

  /// Try update with available measurements
  void try_update();

  /// Propagate the map frame
  void propagate_map_frame();

  /// Get the LiDAR pose at t_given in global
  pair<Matrix3d, Vector3d> get_pose_LinG(int lidar_id, double lidar_time);

  /// Chi stat
  std::vector<std::shared_ptr<UpdaterStatistics>> Chi;

  /// measurement times
  map<int, deque<double>> t_hist;

  /// Lidar map information including ikd-tree
  vector<shared_ptr<iKDDATA>> ikd_data;

private:
  friend class Initializer;

  /// Perform update
  bool update(std::shared_ptr<LiDARData> lidar, shared_ptr<iKDDATA> ikd, shared_ptr<UpdaterStatistics> chi);

  /// Check stacked information and delete old data
  void erase_invalid_dataset();

  /// Processable and processed lidar pointclouds
  vector<std::shared_ptr<LiDARData>> stack_lidar_raw;  // raw lidar stack that not been processed yet
  vector<std::shared_ptr<LiDARData>> stack_lidar_new;  // new lidar points that will be used for EKF update
  vector<std::shared_ptr<LiDARData>> stack_lidar_used; // used lidar points that will be registered to map

  /// First measurement time. Used for reference time (ref. LidarTypes.h)
  double FT = -1;

  /// State
  shared_ptr<State> state;
};
} // namespace mins

#endif // MINS_UPDATERLIDAR_H

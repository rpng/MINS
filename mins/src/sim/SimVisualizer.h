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

#ifndef MINS_SIMVISUALIZER_H
#define MINS_SIMVISUALIZER_H

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <memory>

namespace tf {
class TransformBroadcaster;
}
namespace ros {
class Publisher;
class NodeHandle;
} // namespace ros

namespace mins {
class Simulator;
class SystemManager;
class ROSPublisher;
class SimVisualizer {
public:
  SimVisualizer(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<SystemManager> sys, std::shared_ptr<Simulator> sim = nullptr);

  ~SimVisualizer(){};

  /// Publish cam feature map
  void publish_sim_cam_features();

  /// Publish Lidar structure (eq simulated map)
  void publish_lidar_structure();

  /// Publish groundtruth (if we have it)
  void publish_groundtruth();

  /// Final visualization before shutdown
  void visualize_final();

private:
  /// Global node handler
  std::shared_ptr<ros::NodeHandle> nh;

  /// Core application of the filter system
  std::shared_ptr<SystemManager> sys;

  /// Simulator (is nullptr if we are not sim'ing)
  std::shared_ptr<Simulator> sim;

  std::map<double, Eigen::Matrix<double, 17, 1>> gt_states;

  /// RMSE and NEES of the pose estimation
  double sum_rmse_ori = 0.0;
  double sum_rmse_pos = 0.0;
  double sum_nees_ori = 0.0;
  double sum_nees_pos = 0.0;
  size_t sum_cnt = 0;

  /// For publish
  std::shared_ptr<ros::Publisher> pub_pathgt, pub_posegt, pub_sim_lidar_map, pub_sim_cam_points;
  unsigned int poses_seq_gt = 0;
  std::vector<geometry_msgs::PoseStamped> poses_gt;
  std::shared_ptr<tf::TransformBroadcaster> mTfBr;
  bool traj_in_enu = false;
};
} // namespace mins

#endif // MINS_SIMVISUALIZER_H

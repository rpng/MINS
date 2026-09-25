/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_SIMVISUALIZER_H
#define MINS_SIMVISUALIZER_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <visualization_msgs/msg/marker_array.hpp>

namespace mins {
class Simulator;
class SystemManager;
class ROS2Publisher;
class Sim2Visualizer {
public:
  Sim2Visualizer(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<SystemManager> sys, std::shared_ptr<Simulator> sim = nullptr);

  ~Sim2Visualizer(){};

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
  std::shared_ptr<rclcpp::Node> node;

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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_sim_cam_points;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_sim_lidar_map;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_posegt;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_pathgt;

  unsigned int poses_seq_gt = 0;
  std::vector<geometry_msgs::msg::PoseStamped> poses_gt;
  std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBr;
  bool traj_in_enu = false;
};
} // namespace mins

#endif // MINS_SIMVISUALIZER_H

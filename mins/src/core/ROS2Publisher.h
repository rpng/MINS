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

#ifndef MINS_ROSPUBLISHER_H
#define MINS_ROSPUBLISHER_H

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <image_transport/image_transport.hpp>
#include <memory>
#include <vector>

namespace pcl {
class PointXYZ;
template <class pointT> class PointCloud;
} // namespace pcl

namespace mins {

class SystemManager;
struct GPSData;
struct ViconData;
class State;
struct LiDARData;
struct Options;
class ROS2Publisher {
public:
  /// ROS message publisher
  ROS2Publisher(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<SystemManager> sys, std::shared_ptr<Options> op);

  ~ROS2Publisher(){};

  /// Visualize state, camera & LiDAR features
  void visualize();

  /// Publish current IMU state (pose, ang/lin velocities)
  void publish_imu();

  /// Publish the gps measurements
  void publish_gps(GPSData gps, bool isGeodetic);

  /// Publish the vicon measurements
  void publish_vicon(ViconData data);

  /// Publish the active tracking image
  void publish_cam_images(std::vector<int> cam_ids);
  void publish_cam_images(int cam_id);

  /// Publish lidar point cloud
  void publish_lidar_cloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar);

  void reset_paths();

private:
  /// Publish the current state
  void publish_state();

  /// Publish current camera features
  void publish_cam_features();

  /// Publish current lidar map
  void publish_lidar_map();

  /// Publish TFs
  void publish_tf();

  /// Global node handler
  std::shared_ptr<rclcpp::Node> node;

  /// Core application of the filter system
  std::shared_ptr<SystemManager> sys;

  /// Options
  std::shared_ptr<Options> op;

  // Our publishers
  std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBr;
  std::vector<image_transport::Publisher> pub_cam_image;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_imu_pose;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odom;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_imu_path;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cam_msckf, pub_cam_slam;

  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pub_gps_pose;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> pub_gps_path;

  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pub_vicon_pose;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> pub_vicon_path;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pub_lidar_cloud;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> pub_lidar_map;

  // For path viz
  unsigned int seq_imu = 0;
  std::vector<unsigned int> seq_gps, seq_vicon;
  std::vector<geometry_msgs::msg::PoseStamped> path_imu;
  std::vector<std::vector<geometry_msgs::msg::PoseStamped>> path_gps, path_vicon;
  bool traj_in_enu = false;
};
} // namespace mins

#endif // MINS_ROSPUBLISHER_H

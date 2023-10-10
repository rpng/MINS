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

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <memory>
#include <vector>

namespace ros {
class Publisher;
class NodeHandle;
} // namespace ros
namespace image_transport {
class Publisher;
}
namespace tf {
class TransformBroadcaster;
}
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
class ROSPublisher {
public:
  /// ROS message publisher
  ROSPublisher(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<SystemManager> sys, std::shared_ptr<Options> op);

  ~ROSPublisher(){};

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
  std::shared_ptr<ros::NodeHandle> nh;

  /// Core application of the filter system
  std::shared_ptr<SystemManager> sys;

  /// Options
  std::shared_ptr<Options> op;

  // Our publishers
  std::shared_ptr<tf::TransformBroadcaster> mTfBr;
  std::vector<image_transport::Publisher> pub_cam_image;
  ros::Publisher pub_imu_pose, pub_imu_odom, pub_imu_path, pub_cam_msckf, pub_cam_slam;
  std::vector<ros::Publisher> pub_gps_pose, pub_gps_path, pub_vicon_pose, pub_vicon_path, pub_lidar_cloud, pub_lidar_map;

  // For path viz
  unsigned int seq_imu = 0;
  std::vector<unsigned int> seq_gps, seq_vicon;
  std::vector<geometry_msgs::PoseStamped> path_imu;
  std::vector<std::vector<geometry_msgs::PoseStamped>> path_gps, path_vicon;
  bool traj_in_enu = false;
};
} // namespace mins

#endif // MINS_ROSPUBLISHER_H

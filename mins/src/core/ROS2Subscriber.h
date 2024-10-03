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

#ifndef MINS_ROSSUBSCRIBER_H
#define MINS_ROSSUBSCRIBER_H

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
using namespace sensor_msgs::msg;
using namespace nav_msgs::msg;
namespace mins {

class SystemManager;
struct OptionsEstimator;
class ROS2Publisher;
class ROS2Subscriber {

public:
  /// ROS message subscriber
  ROS2Subscriber(shared_ptr<rclcpp::Node> node, shared_ptr<SystemManager> sys, shared_ptr<ROS2Publisher> pub);

  /// Callback for IMU
  void callback_inertial(const Imu::SharedPtr msg);

  /// Callback for Wheel
  void callback_wheel(const JointState::SharedPtr msg);

  /// Callback for GNSS
  void callback_gnss(const NavSatFix::SharedPtr msg, int gps_id);

  /// Callback for LiDAR
  void callback_lidar(const PointCloud2::SharedPtr msg, int lidar_id);

  /// Callback for monocular camera (image, compressed image)
  void callback_monocular_I(const Image::ConstSharedPtr msg, int cam_id);
  void callback_monocular_C(const CompressedImage::ConstSharedPtr msg, int cam_id);

  /// Callback for synchronized stereo camera (image, compressed image)
  void callback_stereo_I(const Image::ConstSharedPtr msg0, const Image::ConstSharedPtr msg1, int cam_id0, int cam_id1);
  void callback_stereo_C(const CompressedImage::ConstSharedPtr msg0, const CompressedImage::ConstSharedPtr msg1, int cam_id0, int cam_id1);

  void reset_service(std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);

private:
  /// Global node handler
  shared_ptr<rclcpp::Node> node;

  /// MINS system
  shared_ptr<SystemManager> sys;

  /// ROS publisher
  shared_ptr<ROS2Publisher> pub;

  /// Options
  shared_ptr<OptionsEstimator> op;

  /// Our subscribers and camera synchronizers
  rclcpp::Subscription<Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<JointState>::SharedPtr sub_wheel;

  vector<rclcpp::SubscriptionBase::SharedPtr> subs;
  vector<std::shared_ptr<message_filters::Subscriber<Image>>> image_subs;
  vector<std::shared_ptr<message_filters::Subscriber<CompressedImage>>> cimage_subs;

  typedef message_filters::sync_policies::ApproximateTime<Image, Image> sync_pol;
  vector<shared_ptr<message_filters::Synchronizer<sync_pol>>> sync_cam;

  typedef message_filters::sync_policies::ApproximateTime<CompressedImage, CompressedImage> csync_pol;
  vector<shared_ptr<message_filters::Synchronizer<csync_pol>>> csync_cam;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
};
} // namespace mins

#endif // MINS_ROSSUBSCRIBER_H

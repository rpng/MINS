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

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
using namespace sensor_msgs::msg;
namespace mins {

class SystemManager;
struct OptionsEstimator;
class ROSPublisher;
class ROSSubscriber {

public:
  /// ROS message subscriber
  ROSSubscriber(shared_ptr<rclcpp::Node> node, shared_ptr<SystemManager> sys, shared_ptr<ROSPublisher> pub);

  /// Callback for IMU
  void callback_inertial(const Imu::SharedPtr msg);

  /// Callback for Wheel
  void callback_wheel(const JointState::SharedPtr msg);

  /// Callback for GNSS
  void callback_gnss(const NavSatFix::SharedPtr msg, int gps_id);

  /// Callback for LiDAR
  void callback_lidar(const PointCloud2::SharedPtr msg, int lidar_id);

  /// Callback for monocular camera (image, compressed image)
  void callback_monocular_I(const Image::SharedPtr msg, int cam_id);
  void callback_monocular_C(const CompressedImage::SharedPtr msg, int cam_id);

  /// Callback for synchronized stereo camera (image, compressed image)
  void callback_stereo_I(const Image::ConstSharedPtr msg0, const Image::ConstSharedPtr msg1, int cam_id0, int cam_id1);
  void callback_stereo_C(const CompressedImage::ConstSharedPtr msg0, const CompressedImage::ConstSharedPtr msg1, int cam_id0, int cam_id1);

private:
  /// Global node handler
  shared_ptr<rclcpp::Node> node;

  /// MINS system
  shared_ptr<SystemManager> sys;

  /// ROS publisher
  shared_ptr<ROSPublisher> pub;

  /// Options
  shared_ptr<OptionsEstimator> op;

  /// Our subscribers and camera synchronizers
  rclcpp::Subscription<Imu>::SharedPtr sub_imu;

  vector<rclcpp::SubscriptionBase::WeakPtr> subs;
  vector<rclcpp::Subscription<Image>::SharedPtr> image_subs;
  typedef message_filters::sync_policies::ApproximateTime<Image, Image> sync_pol;
  vector<shared_ptr<message_filters::Synchronizer<sync_pol>>> sync_cam;
  vector<shared_ptr<message_filters::Subscriber<Image>>> sync_subs_cam;
};
} // namespace mins

#endif // MINS_ROSSUBSCRIBER_H

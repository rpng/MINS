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

#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
using namespace sensor_msgs;
namespace mins {

class SystemManager;
struct OptionsEstimator;
class ROSPublisher;
class ROSSubscriber {

public:
  /// ROS message subscriber
  ROSSubscriber(shared_ptr<ros::NodeHandle> nh, shared_ptr<SystemManager> sys, shared_ptr<ROSPublisher> pub);

  /// Callback for IMU
  void callback_inertial(const Imu::ConstPtr &msg);

  /// Callback for Wheel
  void callback_wheel(const JointStateConstPtr &msg);

  /// Callback for GNSS
  void callback_gnss(const NavSatFixConstPtr &msg, int gps_id);

  /// Callback for LiDAR
  void callback_lidar(const PointCloud2ConstPtr &msg, int lidar_id);

  /// Callback for monocular camera (image, compressed image)
  void callback_monocular_I(const ImageConstPtr &msg, int cam_id);
  void callback_monocular_C(const CompressedImageConstPtr &msg, int cam_id);

  /// Callback for synchronized stereo camera (image, compressed image)
  void callback_stereo_I(const ImageConstPtr &msg0, const ImageConstPtr &msg1, int cam_id0, int cam_id1);
  void callback_stereo_C(const CompressedImageConstPtr &msg0, const CompressedImageConstPtr &msg1, int cam_id0, int cam_id1);

private:
  /// Global node handler
  shared_ptr<ros::NodeHandle> nh;

  /// MINS system
  shared_ptr<SystemManager> sys;

  /// ROS publisher
  shared_ptr<ROSPublisher> pub;

  /// Options
  shared_ptr<OptionsEstimator> op;

  /// Our subscribers and camera synchronizers
  vector<ros::Subscriber> subs;
  typedef message_filters::sync_policies::ApproximateTime<Image, Image> sync_pol;
  vector<shared_ptr<message_filters::Synchronizer<sync_pol>>> sync_cam;
  vector<shared_ptr<message_filters::Subscriber<Image>>> sync_subs_cam;
};
} // namespace mins

#endif // MINS_ROSSUBSCRIBER_H

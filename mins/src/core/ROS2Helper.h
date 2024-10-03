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

#ifndef MINS_ROSVISUALIZER_HELPER_H
#define MINS_ROSVISUALIZER_HELPER_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/transform_datatypes.h>

using namespace std;
using namespace sensor_msgs;

namespace ov_core {
struct ImuData;
struct CameraData;
} // namespace ov_core
namespace ov_type {
struct PoseJPL;
struct Vec;
} // namespace ov_type
namespace pcl {
class PointXYZ;
template <class pointT> class PointCloud;
} // namespace pcl

namespace mins {
class State;
struct OptionsCamera;
struct WheelData;
struct RoverWheelData;
struct ViconData;
struct GPSData;
struct TLIOData;

/**
 * @brief Helper class that handles some common versions into and out of ROS formats
 */
class ROS2Helper {

public:
  /// Collection of format converters
  static sensor_msgs::msg::PointCloud2 ToPointcloud(const vector<Eigen::Vector3d> &feats, string frame_id);

  static geometry_msgs::msg::TransformStamped Pose2TF(const shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans);

  static geometry_msgs::msg::TransformStamped Pos2TF(const shared_ptr<ov_type::Vec> &pos, bool flip_trans);

  static ov_core::ImuData Imu2Data(const sensor_msgs::msg::Imu::SharedPtr msg);

  static bool Image2Data(const sensor_msgs::msg::Image::ConstSharedPtr msg, int cam_id, ov_core::CameraData &cam, shared_ptr<OptionsCamera> op);

  static bool Image2Data(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg, int cam_id, ov_core::CameraData &cam, shared_ptr<OptionsCamera> op);

  static WheelData JointState2Data(const sensor_msgs::msg::JointState::SharedPtr msg);

  static WheelData Odometry2Data(const nav_msgs::msg::Odometry::SharedPtr msg);

  static ViconData PoseStamped2Data(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int id);

  static ViconData PoseStamped2Data(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg, int id);

  static GPSData PoseStamped2Data(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int id, double noise);

  static std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> rosPC2pclPC(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int id);

  static GPSData NavSatFix2Data(const sensor_msgs::msg::NavSatFix::SharedPtr msg, int id);

  static GPSData NavSatFix2Data(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg, int id);

  static nav_msgs::msg::Odometry ToOdometry(Eigen::Matrix<double, 13, 1> state);

  static geometry_msgs::msg::PoseWithCovarianceStamped ToPoseCov(Eigen::Matrix<double, 7, 1> state);

  static geometry_msgs::msg::PoseStamped ToENU(geometry_msgs::msg::PoseStamped pose, Eigen::Matrix<double, 7, 1> trans_WtoE);
};

} // namespace mins

#endif // MINS_ROSVISUALIZER_HELPER_H

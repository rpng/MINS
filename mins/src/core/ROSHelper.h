/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_ROSVISUALIZER_HELPER_H
#define MINS_ROSVISUALIZER_HELPER_H

#include <Eigen/Eigen>
// forward decls - full PointCloud.h only needed in .cpp files that build/read clouds
namespace mins {
struct PointXYZ;
struct PointXYZI;
template <class PointT> struct PointCloud;
} // namespace mins
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

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
} // namespace pcl

namespace mins {
class State;
struct OptionsCamera;
struct WheelData;
struct ViconData;
struct GPSData;

/**
 * @brief Helper class that handles some common versions into and out of ROS formats
 */
class ROSHelper {

public:
  /// Collection of format converters
  static sensor_msgs::PointCloud2 ToPointcloud(const vector<Eigen::Vector3d> &feats, string frame_id);

  static tf::StampedTransform Pose2TF(const shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans);

  static tf::StampedTransform Pos2TF(const shared_ptr<ov_type::Vec> &pos, bool flip_trans);

  static ov_core::ImuData Imu2Data(const sensor_msgs::Imu::ConstPtr &msg);

  static bool Image2Data(const sensor_msgs::ImageConstPtr &msg, int cam_id, ov_core::CameraData &cam, shared_ptr<OptionsCamera> op);

  static bool Image2Data(const sensor_msgs::CompressedImageConstPtr &msg, int cam_id, ov_core::CameraData &cam, shared_ptr<OptionsCamera> op);

  static WheelData JointState2Data(const sensor_msgs::JointStateConstPtr &msg);

  static WheelData Odometry2Data(const nav_msgs::OdometryPtr &msg);

  static ViconData PoseStamped2Data(const geometry_msgs::PoseStampedPtr &msg, int id);

  static GPSData PoseStamped2Data(const geometry_msgs::PoseStampedPtr &msg, int id, double noise);

  static std::shared_ptr<mins::PointCloud<mins::PointXYZ>> rosPC2pclPC(const sensor_msgs::PointCloud2ConstPtr &msg, int id);

  static GPSData NavSatFix2Data(const sensor_msgs::NavSatFixPtr &msg, int id);

  static GPSData NavSatFix2Data(const sensor_msgs::NavSatFixConstPtr &msg, int id);

  static nav_msgs::Odometry ToOdometry(Eigen::Matrix<double, 13, 1> state);

  static geometry_msgs::PoseWithCovarianceStamped ToPoseCov(Eigen::Matrix<double, 7, 1> state);

  static geometry_msgs::PoseStamped ToENU(geometry_msgs::PoseStamped pose, Eigen::Matrix<double, 7, 1> trans_WtoE);
};

} // namespace mins

#endif // MINS_ROSVISUALIZER_HELPER_H

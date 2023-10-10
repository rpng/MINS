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

#include "ROSHelper.h"
#include "options/OptionsCamera.h"
#include "pcl_conversions/pcl_conversions.h"
#include "state/State.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "update/gps/GPSTypes.h"
#include "update/vicon/ViconTypes.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/point_cloud2_iterator.h>

using namespace mins;
PointCloud2 ROSHelper::ToPointcloud(const std::vector<Eigen::Vector3d> &feats, std::string frame_id) {

  // Declare message and sizes
  PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = ros::Time::now();
  cloud.width = feats.size();
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false; // there may be invalid points
  // Setup pointcloud fields
  PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(feats.size());

  // Iterators
  PointCloud2Iterator<float> out_x(cloud, "x");
  PointCloud2Iterator<float> out_y(cloud, "y");
  PointCloud2Iterator<float> out_z(cloud, "z");

  // Fill our iterators
  for (const auto &pt : feats) {
    *out_x = (float)pt(0);
    ++out_x;
    *out_y = (float)pt(1);
    ++out_y;
    *out_z = (float)pt(2);
    ++out_z;
  }

  return cloud;
}

tf::StampedTransform ROSHelper::Pose2TF(const shared_ptr<ov_type::PoseJPL> &pose, bool flip_trans) {

  // Need to flip the transform to the imu frame
  Eigen::Vector4d q_ItoC = pose->quat();
  Eigen::Vector3d p_CinI = pose->pos();
  if (flip_trans) {
    p_CinI = -pose->Rot().transpose() * pose->pos();
  }

  // publish our transform on TF
  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
  // NOTE: a rotation from ItoC in JPL has the same xyzw as a CtoI Hamilton rotation
  tf::StampedTransform trans;
  trans.stamp_ = ros::Time::now();
  tf::Quaternion quat(q_ItoC(0), q_ItoC(1), q_ItoC(2), q_ItoC(3));
  trans.setRotation(quat);
  tf::Vector3 orig(p_CinI(0), p_CinI(1), p_CinI(2));
  trans.setOrigin(orig);
  return trans;
}

tf::StampedTransform ROSHelper::Pos2TF(const shared_ptr<ov_type::Vec> &pos, bool flip_trans) {
  assert(pos->size() == 3);
  // Need to flip the transform to the imu frame
  Eigen::Vector3d p_XinI = pos->value();

  // publish our transform on TF
  tf::StampedTransform trans;
  trans.stamp_ = ros::Time::now();
  tf::Quaternion quat(0, 0, 0, 1);
  trans.setRotation(quat);
  tf::Vector3 orig(p_XinI(0), p_XinI(1), p_XinI(2));
  trans.setOrigin(orig);
  return trans;
}

ov_core::ImuData ROSHelper::Imu2Data(const Imu::ConstPtr &msg) {
  ov_core::ImuData message;
  message.timestamp = msg->header.stamp.toSec();
  message.wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
  return message;
}

std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ROSHelper::rosPC2pclPC(const sensor_msgs::PointCloud2ConstPtr &msg, int id) {
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_pc2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *pcl_pc2);
  pcl_pc2->header.frame_id = to_string(id);                 // overwrite the id match with system number
  pcl_pc2->header.stamp = msg->header.stamp.toSec() * 1000; // deliver this in msec
  return pcl_pc2;
}

GPSData ROSHelper::NavSatFix2Data(const sensor_msgs::NavSatFixPtr &msg, int id) {
  GPSData data;
  data.time = msg->header.stamp.toSec();
  data.id = id;
  data.meas(0) = msg->latitude;
  data.meas(1) = msg->longitude;
  data.meas(2) = msg->altitude;
  data.noise(0) = msg->position_covariance.at(0);
  data.noise(1) = msg->position_covariance.at(4);
  data.noise(2) = msg->position_covariance.at(8);
  return data;
}

GPSData ROSHelper::NavSatFix2Data(const sensor_msgs::NavSatFixConstPtr &msg, int id) {
  GPSData data;
  data.time = msg->header.stamp.toSec();
  data.id = id;
  data.meas(0) = msg->latitude;
  data.meas(1) = msg->longitude;
  data.meas(2) = msg->altitude;
  data.noise(0) = msg->position_covariance.at(0);
  data.noise(1) = msg->position_covariance.at(4);
  data.noise(2) = msg->position_covariance.at(8);
  return data;
}

bool ROSHelper::Image2Data(const ImageConstPtr &msg, int cam_id, ov_core::CameraData &cam, shared_ptr<OptionsCamera> op) {

  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg, image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    PRINT4("cv_bridge exception: %s", e.what());
    return false;
  }

  // Create the measurement
  cam.timestamp = cv_ptr->header.stamp.toSec();
  cam.sensor_ids.push_back(cam_id);
  cam.images.push_back(cv_ptr->image.clone());

  // Load the mask if we are using it, else it is empty
  if (op->use_mask.at(cam_id)) {
    cam.masks.push_back(op->masks.at(cam_id));
  } else {
    cam.masks.push_back(cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1));
  }
  return true;
}

WheelData ROSHelper::JointState2Data(const JointStateConstPtr &msg) {
  WheelData data;
  data.time = msg->header.stamp.toSec();
  data.m1 = msg->velocity.at(0); // front_left_wheel_joint velocity
  data.m2 = msg->velocity.at(1); // front_right_wheel_joint velocity
  return data;
}

WheelData ROSHelper::Odometry2Data(const nav_msgs::OdometryPtr &msg) {
  WheelData data;
  data.time = msg->header.stamp.toSec();
  data.m1 = msg->twist.twist.angular.z;
  data.m2 = msg->twist.twist.linear.x;
  return data;
}

ViconData ROSHelper::PoseStamped2Data(const geometry_msgs::PoseStampedPtr &msg, int id) {
  Eigen::Vector4d q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  Eigen::Vector3d p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  ViconData vicon;
  vicon.time = msg->header.stamp.toSec();
  vicon.id = id;
  vicon.pose.block(0, 0, 3, 1) = ov_core::log_so3(ov_core::quat_2_Rot(q));
  vicon.pose.block(3, 0, 3, 1) = p;
  return vicon;
}

GPSData ROSHelper::PoseStamped2Data(const geometry_msgs::PoseStampedPtr &msg, int id, double noise) {
  Eigen::Vector3d p(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  GPSData gps;
  gps.time = msg->header.stamp.toSec();
  gps.id = id;
  gps.meas = p;
  gps.noise = noise * Eigen::Vector3d::Ones();
  return gps;
}

bool ROSHelper::Image2Data(const CompressedImageConstPtr &msg, int cam_id, ov_core::CameraData &cam, shared_ptr<OptionsCamera> op) {

  // Create the measurement
  cam.timestamp = msg->header.stamp.toSec();
  cam.sensor_ids.push_back(cam_id);
  cam.images.push_back(cv::imdecode(cv::Mat(msg->data), 0));

  // Load the mask if we are using it, else it is empty
  // TODO: in the future we should get this from external pixel segmentation
  if (op->use_mask.at(cam_id)) {
    cam.masks.push_back(op->masks.at(cam_id));
  } else {
    cam.masks.push_back(cv::Mat::zeros(cam.images[0].rows, cam.images[0].cols, CV_8UC1));
  }
  return true;
}

nav_msgs::Odometry ROSHelper::ToOdometry(Eigen::Matrix<double, 13, 1> state) {
  nav_msgs::Odometry odom;
  // The POSE component (orientation and position)
  odom.pose.pose.orientation.x = state(0);
  odom.pose.pose.orientation.y = state(1);
  odom.pose.pose.orientation.z = state(2);
  odom.pose.pose.orientation.w = state(3);
  odom.pose.pose.position.x = state(4);
  odom.pose.pose.position.y = state(5);
  odom.pose.pose.position.z = state(6);
  // The TWIST component (angular and linear velocities)
  odom.twist.twist.linear.x = state(7);
  odom.twist.twist.linear.y = state(8);
  odom.twist.twist.linear.z = state(9);
  odom.twist.twist.angular.x = state(10);
  odom.twist.twist.angular.y = state(11);
  odom.twist.twist.angular.z = state(12);
  return odom;
}

geometry_msgs::PoseWithCovarianceStamped ROSHelper::ToPoseCov(Eigen::Matrix<double, 7, 1> state) {
  geometry_msgs::PoseWithCovarianceStamped odom;
  // The POSE component (orientation and position)
  odom.pose.pose.orientation.x = state(0);
  odom.pose.pose.orientation.y = state(1);
  odom.pose.pose.orientation.z = state(2);
  odom.pose.pose.orientation.w = state(3);
  odom.pose.pose.position.x = state(4);
  odom.pose.pose.position.y = state(5);
  odom.pose.pose.position.z = state(6);
  return odom;
}

geometry_msgs::PoseStamped ROSHelper::ToENU(geometry_msgs::PoseStamped pose, Eigen::Matrix<double, 7, 1> trans_WtoE) {
  // Take the readings out
  Vector4d q_WtoI;
  q_WtoI(0) = pose.pose.orientation.x;
  q_WtoI(1) = pose.pose.orientation.y;
  q_WtoI(2) = pose.pose.orientation.z;
  q_WtoI(3) = pose.pose.orientation.w;
  Matrix3d R_WtoI = ov_core::quat_2_Rot(q_WtoI);
  Vector3d p_IinW;
  p_IinW(0) = pose.pose.position.x;
  p_IinW(1) = pose.pose.position.y;
  p_IinW(2) = pose.pose.position.z;

  // Get the transformation
  Matrix3d R_WtoE = ov_core::quat_2_Rot(trans_WtoE.head(4));
  Vector3d p_WinE = trans_WtoE.tail(3);

  // Compute pose in ENU
  Matrix3d R_EtoI = R_WtoI * R_WtoE.transpose();
  Vector4d q_EtoI = ov_core::rot_2_quat(R_EtoI);
  Vector3d p_IinE = p_WinE + R_WtoE * p_IinW;

  // construct pose message and return
  geometry_msgs::PoseStamped poseENU;
  poseENU.header = pose.header;
  poseENU.pose.orientation.x = q_EtoI(0);
  poseENU.pose.orientation.y = q_EtoI(1);
  poseENU.pose.orientation.z = q_EtoI(2);
  poseENU.pose.orientation.w = q_EtoI(3);
  poseENU.pose.position.x = p_IinE(0);
  poseENU.pose.position.y = p_IinE(1);
  poseENU.pose.position.z = p_IinE(2);
  return poseENU;
}
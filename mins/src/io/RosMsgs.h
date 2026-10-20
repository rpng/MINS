/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_IO_ROSMSGS_H
#define MINS_IO_ROSMSGS_H

#include <Eigen/Core>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace ov_core {
struct ImuData;
}

namespace mins {

/// Field layout of a sensor_msgs/PointCloud2, enough for fromPC2
struct PointField {
  std::string name;
  std::uint32_t offset;
  std::uint8_t datatype;
  std::uint32_t count;
};

struct PointCloud2 {
  double stamp;
  std::uint32_t height, width, point_step;
  std::vector<PointField> fields;
  std::vector<std::uint8_t> data;
};

/// A timestamped pose (geometry_msgs/PoseStamped, TransformStamped), q is JPL-ordered x y z w
struct StampedPose {
  double stamp;
  Eigen::Vector3d p;
  Eigen::Vector4d q;
};

struct NavSatFix {
  double stamp;
  double latitude, longitude, altitude;
  double covariance[9];
};

struct JointState {
  double stamp;
  std::vector<std::string> name;
  std::vector<double> position, velocity, effort;
};

/// nav_msgs/Odometry, only what MINS reads
struct Odometry {
  double stamp;
  Eigen::Vector3d linear, angular;
};

/// Decoders for ROS1-serialized messages. Each one throws if the bytes do not match the type exactly.
namespace rosmsg {

/// The std_msgs/Header stamp every stamped message starts with, without decoding the rest
double stamp(const std::vector<std::uint8_t> &data);

ov_core::ImuData imu(const std::vector<std::uint8_t> &data);

/// sensor_msgs/Image or sensor_msgs/CompressedImage to 8-bit grayscale, the way cv_bridge's MONO8 conversion does it
cv::Mat image(const std::string &type, const std::vector<std::uint8_t> &data, double &stamp);

PointCloud2 pointcloud2(const std::vector<std::uint8_t> &data);

NavSatFix navsatfix(const std::vector<std::uint8_t> &data);

JointState jointstate(const std::vector<std::uint8_t> &data);

Odometry odometry(const std::vector<std::uint8_t> &data);

/// geometry_msgs/PoseStamped or geometry_msgs/TransformStamped
StampedPose pose(const std::string &type, const std::vector<std::uint8_t> &data);

} // namespace rosmsg
} // namespace mins

#endif // MINS_IO_ROSMSGS_H

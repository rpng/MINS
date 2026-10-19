/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "RosMsgs.h"

#include "utils/sensor_data.h"
#include <cstring>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

using namespace mins;

namespace {

/// Reads the ROS1 wire format: little-endian scalars, uint32 length before strings and variable arrays
class Reader {
public:
  Reader(const std::vector<std::uint8_t> &d, const char *type) : p(d.data()), end(d.data() + d.size()), type(type) {}

  template <class T> T get() {
    T v;
    std::memcpy(&v, take(sizeof(T)), sizeof(T));
    return v;
  }

  std::string str() {
    std::uint32_t n = get<std::uint32_t>();
    const std::uint8_t *s = take(n);
    return std::string(reinterpret_cast<const char *>(s), n);
  }

  std::vector<std::uint8_t> bytes() {
    std::uint32_t n = get<std::uint32_t>();
    const std::uint8_t *s = take(n);
    return std::vector<std::uint8_t>(s, s + n);
  }

  std::vector<double> doubles() {
    std::vector<double> v(get<std::uint32_t>());
    for (auto &x : v)
      x = get<double>();
    return v;
  }

  void skip(std::size_t n) { take(n); }

  /// std_msgs/Header: seq, stamp, frame_id. Returns the stamp as ros::Time::toSec() does.
  double header() {
    get<std::uint32_t>();
    std::uint32_t sec = get<std::uint32_t>(), nsec = get<std::uint32_t>();
    str();
    return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec);
  }

  Eigen::Vector3d vec3() {
    double x = get<double>(), y = get<double>(), z = get<double>();
    return {x, y, z};
  }

  Eigen::Vector4d quat() {
    double x = get<double>(), y = get<double>(), z = get<double>(), w = get<double>();
    return {x, y, z, w};
  }

  /// Every byte must be consumed, else the message is not the type we think it is
  void done() {
    if (p != end)
      throw std::runtime_error(std::string("rosmsg: ") + type + " has " + std::to_string(end - p) + " unread bytes");
  }

private:
  const std::uint8_t *take(std::size_t n) {
    if (static_cast<std::size_t>(end - p) < n)
      throw std::runtime_error(std::string("rosmsg: ") + type + " is shorter than its layout");
    const std::uint8_t *r = p;
    p += n;
    return r;
  }

  const std::uint8_t *p, *end;
  const char *type;
};

} // namespace

double rosmsg::stamp(const std::vector<std::uint8_t> &data) {
  if (data.size() < 12)
    throw std::runtime_error("rosmsg: message too short for a header");
  std::uint32_t sec, nsec;
  std::memcpy(&sec, data.data() + 4, 4);
  std::memcpy(&nsec, data.data() + 8, 4);
  return static_cast<double>(sec) + 1e-9 * static_cast<double>(nsec);
}

ov_core::ImuData rosmsg::imu(const std::vector<std::uint8_t> &data) {
  Reader r(data, "sensor_msgs/Imu");
  ov_core::ImuData m;
  m.timestamp = r.header();
  r.quat();
  r.skip(9 * 8);
  m.wm = r.vec3();
  r.skip(9 * 8);
  m.am = r.vec3();
  r.skip(9 * 8);
  r.done();
  return m;
}

cv::Mat rosmsg::image(const std::string &type, const std::vector<std::uint8_t> &data, double &stamp) {
  if (type == "sensor_msgs/CompressedImage") {
    Reader r(data, "sensor_msgs/CompressedImage");
    stamp = r.header();
    r.str();
    std::vector<std::uint8_t> bytes = r.bytes();
    r.done();
    return cv::imdecode(cv::Mat(bytes), cv::IMREAD_GRAYSCALE);
  }

  Reader r(data, "sensor_msgs/Image");
  stamp = r.header();
  std::uint32_t height = r.get<std::uint32_t>(), width = r.get<std::uint32_t>();
  std::string encoding = r.str();
  r.get<std::uint8_t>(); // is_bigendian
  std::uint32_t step = r.get<std::uint32_t>();
  std::vector<std::uint8_t> bytes = r.bytes();
  r.done();
  if (bytes.size() < static_cast<std::size_t>(step) * height)
    throw std::runtime_error("rosmsg: sensor_msgs/Image data is smaller than step * height");

  auto wrap = [&](int cv_type) { return cv::Mat(static_cast<int>(height), static_cast<int>(width), cv_type, bytes.data(), step); };
  cv::Mat gray;
  if (encoding == "mono8" || encoding == "8UC1")
    gray = wrap(CV_8UC1).clone();
  else if (encoding == "mono16" || encoding == "16UC1")
    wrap(CV_16UC1).convertTo(gray, CV_8U, 255.0 / 65535.0); // same scaling as cv_bridge
  else if (encoding == "bgr8")
    cv::cvtColor(wrap(CV_8UC3), gray, cv::COLOR_BGR2GRAY);
  else if (encoding == "rgb8")
    cv::cvtColor(wrap(CV_8UC3), gray, cv::COLOR_RGB2GRAY);
  else if (encoding == "bgra8")
    cv::cvtColor(wrap(CV_8UC4), gray, cv::COLOR_BGRA2GRAY);
  else if (encoding == "rgba8")
    cv::cvtColor(wrap(CV_8UC4), gray, cv::COLOR_RGBA2GRAY);
  else if (encoding == "bayer_rggb8")
    cv::cvtColor(wrap(CV_8UC1), gray, cv::COLOR_BayerBG2GRAY);
  else if (encoding == "bayer_bggr8")
    cv::cvtColor(wrap(CV_8UC1), gray, cv::COLOR_BayerRG2GRAY);
  else if (encoding == "bayer_gbrg8")
    cv::cvtColor(wrap(CV_8UC1), gray, cv::COLOR_BayerGR2GRAY);
  else if (encoding == "bayer_grbg8")
    cv::cvtColor(wrap(CV_8UC1), gray, cv::COLOR_BayerGB2GRAY);
  else
    throw std::runtime_error("rosmsg: unsupported image encoding " + encoding);
  return gray;
}

PointCloud2 rosmsg::pointcloud2(const std::vector<std::uint8_t> &data) {
  Reader r(data, "sensor_msgs/PointCloud2");
  PointCloud2 m;
  m.stamp = r.header();
  m.height = r.get<std::uint32_t>();
  m.width = r.get<std::uint32_t>();
  m.fields.resize(r.get<std::uint32_t>());
  for (auto &f : m.fields) {
    f.name = r.str();
    f.offset = r.get<std::uint32_t>();
    f.datatype = r.get<std::uint8_t>();
    f.count = r.get<std::uint32_t>();
  }
  if (r.get<std::uint8_t>() != 0)
    throw std::runtime_error("rosmsg: big-endian PointCloud2 is not supported");
  m.point_step = r.get<std::uint32_t>();
  r.get<std::uint32_t>(); // row_step
  m.data = r.bytes();
  r.get<std::uint8_t>(); // is_dense
  r.done();
  if (m.data.size() < static_cast<std::size_t>(m.point_step) * m.width * m.height)
    throw std::runtime_error("rosmsg: sensor_msgs/PointCloud2 data is smaller than its points");
  return m;
}

NavSatFix rosmsg::navsatfix(const std::vector<std::uint8_t> &data) {
  Reader r(data, "sensor_msgs/NavSatFix");
  NavSatFix m;
  m.stamp = r.header();
  r.get<std::int8_t>();   // status
  r.get<std::uint16_t>(); // service
  m.latitude = r.get<double>();
  m.longitude = r.get<double>();
  m.altitude = r.get<double>();
  for (double &c : m.covariance)
    c = r.get<double>();
  r.get<std::uint8_t>(); // position_covariance_type
  r.done();
  return m;
}

JointState rosmsg::jointstate(const std::vector<std::uint8_t> &data) {
  Reader r(data, "sensor_msgs/JointState");
  JointState m;
  m.stamp = r.header();
  m.name.resize(r.get<std::uint32_t>());
  for (auto &n : m.name)
    n = r.str();
  m.position = r.doubles();
  m.velocity = r.doubles();
  m.effort = r.doubles();
  r.done();
  return m;
}

Odometry rosmsg::odometry(const std::vector<std::uint8_t> &data) {
  Reader r(data, "nav_msgs/Odometry");
  Odometry m;
  m.stamp = r.header();
  r.str();  // child_frame_id
  r.vec3(); // pose
  r.quat();
  r.skip(36 * 8);
  m.linear = r.vec3(); // twist
  m.angular = r.vec3();
  r.skip(36 * 8);
  r.done();
  return m;
}

StampedPose rosmsg::pose(const std::string &type, const std::vector<std::uint8_t> &data) {
  Reader r(data, type.c_str());
  StampedPose m;
  m.stamp = r.header();
  if (type == "geometry_msgs/TransformStamped") {
    r.str(); // child_frame_id
    m.p = r.vec3();
    m.q = r.quat();
  } else if (type == "geometry_msgs/PoseStamped") {
    m.p = r.vec3();
    m.q = r.quat();
  } else {
    throw std::runtime_error("rosmsg: not a pose type: " + type);
  }
  r.done();
  return m;
}

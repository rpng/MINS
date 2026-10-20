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
#include <map>
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
  bool big_endian = r.get<std::uint8_t>() != 0;
  std::uint32_t step = r.get<std::uint32_t>();
  std::vector<std::uint8_t> bytes = r.bytes();
  r.done();
  if (bytes.size() < static_cast<std::size_t>(step) * height)
    throw std::runtime_error("rosmsg: sensor_msgs/Image data is smaller than step * height");

  // cv_bridge's mono8 conversion: the color conversion below at the source depth, then 16-bit scaled down to 8-bit.
  // Code -1 is already gray.
  struct Color {
    int code;
    int channels;
  };
  static const std::map<std::string, Color> colors = {
      {"mono", {-1, 1}},
      {"bgr", {cv::COLOR_BGR2GRAY, 3}},
      {"rgb", {cv::COLOR_RGB2GRAY, 3}},
      {"bgra", {cv::COLOR_BGRA2GRAY, 4}},
      {"rgba", {cv::COLOR_RGBA2GRAY, 4}},
      {"bayer_rggb", {cv::COLOR_BayerBG2GRAY, 1}},
      {"bayer_bggr", {cv::COLOR_BayerRG2GRAY, 1}},
      {"bayer_gbrg", {cv::COLOR_BayerGR2GRAY, 1}},
      {"bayer_grbg", {cv::COLOR_BayerGB2GRAY, 1}},
      {"yuv422", {cv::COLOR_YUV2GRAY_UYVY, 2}},
  };
  std::string base = encoding == "8UC1" ? "mono8" : encoding == "16UC1" ? "mono16" : encoding;
  int depth = CV_8U;
  if (base.size() > 2 && base.compare(base.size() - 2, 2, "16") == 0) {
    depth = CV_16U;
    base.resize(base.size() - 2);
  } else if (base.size() > 1 && base.back() == '8') {
    base.pop_back();
  }
  auto it = colors.find(base);
  if (it == colors.end() || (depth == CV_16U && base.compare(0, 3, "yuv") == 0))
    throw std::runtime_error("rosmsg: unsupported image encoding " + encoding);

  cv::Mat raw(static_cast<int>(height), static_cast<int>(width), CV_MAKETYPE(depth, it->second.channels), bytes.data(), step);
  if (depth == CV_16U && big_endian) {
    raw = raw.clone();
    for (int y = 0; y < raw.rows; y++) {
      auto *p = raw.ptr<std::uint16_t>(y);
      for (int i = 0; i < raw.cols * raw.channels(); i++)
        p[i] = static_cast<std::uint16_t>((p[i] >> 8) | (p[i] << 8));
    }
  }
  cv::Mat gray;
  if (it->second.code < 0)
    gray = raw;
  else
    cv::cvtColor(raw, gray, it->second.code);
  if (depth == CV_16U)
    gray.convertTo(gray, CV_8U, 255.0 / 65535.0);
  else if (it->second.code < 0)
    gray = gray.clone();
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

/*
 * Direct conversion between our mins::PointCloud and sensor_msgs PointCloud2, with no PCL
 * in between (saves a full cloud copy per lidar frame). The ROS1 and ROS2 PointCloud2 /
 * PointField messages are field-for-field identical, so one set of templates serves both.
 */
#ifndef MINS_LIDAR_POINTCLOUD2CONVERT_H
#define MINS_LIDAR_POINTCLOUD2CONVERT_H

#include <cstring>
#include <memory>
#include <string>

#include "update/lidar/PointCloud.h"

namespace mins {

// PointField.datatype for float32 (same value in ROS1 and ROS2).
static constexpr std::uint8_t PC2_FLOAT32 = 7;

// sensor_msgs PointCloud2 -> XYZ cloud. Reads x/y/z by field offset, so any point layout
// (Velodyne/Ouster clouds with ring, time, ... extra fields) parses correctly. Assumes the
// x/y/z fields are float32, which every lidar driver emits.
template <class PC2> std::shared_ptr<PointCloud<PointXYZ>> fromPC2(const PC2 &msg) {
  auto out = std::make_shared<PointCloud<PointXYZ>>();
  int ox = -1, oy = -1, oz = -1;
  for (const auto &f : msg.fields) {
    if (f.name == "x")
      ox = static_cast<int>(f.offset);
    else if (f.name == "y")
      oy = static_cast<int>(f.offset);
    else if (f.name == "z")
      oz = static_cast<int>(f.offset);
  }
  if (ox < 0 || oy < 0 || oz < 0)
    return out;
  const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;
  out->points.reserve(n);
  for (std::size_t i = 0; i < n; i++) {
    const unsigned char *b = &msg.data[i * msg.point_step];
    float x, y, z;
    std::memcpy(&x, b + ox, 4);
    std::memcpy(&y, b + oy, 4);
    std::memcpy(&z, b + oz, 4);
    out->push_back(PointXYZ(x, y, z));
  }
  return out;
}

namespace detail {
template <class PC2> void pc2_set_field(PC2 &out, int k, const char *name, std::uint32_t offset) {
  out.fields[k].name = name;
  out.fields[k].offset = offset;
  out.fields[k].datatype = PC2_FLOAT32;
  out.fields[k].count = 1;
}
template <class PC2> void pc2_alloc(PC2 &out, std::uint32_t npoints, std::uint32_t nfields, std::uint32_t point_step) {
  out.height = 1;
  out.width = npoints;
  out.is_bigendian = false;
  out.is_dense = false;
  out.point_step = point_step;
  out.row_step = point_step * npoints;
  out.fields.resize(nfields);
  out.data.resize(static_cast<std::size_t>(out.row_step));
}
} // namespace detail

// XYZ cloud -> PointCloud2 (3 float fields).
template <class PC2> void toPC2(const PointCloud<PointXYZ> &in, PC2 &out) {
  detail::pc2_alloc(out, static_cast<std::uint32_t>(in.points.size()), 3, 12);
  detail::pc2_set_field(out, 0, "x", 0);
  detail::pc2_set_field(out, 1, "y", 4);
  detail::pc2_set_field(out, 2, "z", 8);
  for (std::size_t i = 0; i < in.points.size(); i++) {
    float v[3] = {in.points[i].x, in.points[i].y, in.points[i].z};
    std::memcpy(&out.data[i * 12], v, 12);
  }
}

// XYZI cloud -> PointCloud2 (4 float fields).
template <class PC2> void toPC2(const PointCloud<PointXYZI> &in, PC2 &out) {
  detail::pc2_alloc(out, static_cast<std::uint32_t>(in.points.size()), 4, 16);
  detail::pc2_set_field(out, 0, "x", 0);
  detail::pc2_set_field(out, 1, "y", 4);
  detail::pc2_set_field(out, 2, "z", 8);
  detail::pc2_set_field(out, 3, "intensity", 12);
  for (std::size_t i = 0; i < in.points.size(); i++) {
    float v[4] = {in.points[i].x, in.points[i].y, in.points[i].z, in.points[i].intensity};
    std::memcpy(&out.data[i * 16], v, 16);
  }
}

} // namespace mins

#endif // MINS_LIDAR_POINTCLOUD2CONVERT_H

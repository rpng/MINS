/*
 * Minimal in-house replacement for the small slice of PCL that MINS used:
 * two point types, a vector-backed cloud, a rigid transform, and a centroid
 * voxel-grid downsample. This drops PCL (and its boost/flann/vtk transitive
 * weight) from every build - ROS1, ROS2, and the ROS-free CLI.
 */
#ifndef MINS_LIDAR_POINTCLOUD_H
#define MINS_LIDAR_POINTCLOUD_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mins {

struct PointXYZ {
  float x = 0.0f, y = 0.0f, z = 0.0f;
  PointXYZ() = default;
  PointXYZ(float px, float py, float pz) : x(px), y(py), z(pz) {}
  Eigen::Vector3f getVector3fMap() const { return Eigen::Vector3f(x, y, z); }
};

struct PointXYZI {
  float x = 0.0f, y = 0.0f, z = 0.0f, intensity = 0.0f;
  PointXYZI() = default;
  PointXYZI(float px, float py, float pz, float pi = 0.0f) : x(px), y(py), z(pz), intensity(pi) {}
  Eigen::Vector3f getVector3fMap() const { return Eigen::Vector3f(x, y, z); }
};

// Cloud metadata. MINS stashes the lidar id in frame_id and the timestamp in stamp,
// same as it did with pcl::PointCloud::header.
struct CloudHeader {
  std::uint64_t stamp = 0;
  std::string frame_id;
};

// std::vector-backed cloud exposing only the members MINS actually used from pcl::PointCloud.
template <class PointT> struct PointCloud {
  using VectorType = std::vector<PointT, Eigen::aligned_allocator<PointT>>;
  VectorType points;
  CloudHeader header;
  std::uint32_t width = 0, height = 1;
  bool is_dense = false;

  std::size_t size() const { return points.size(); }
  bool empty() const { return points.empty(); }
  void clear() { points.clear(); width = 0; }
  void reserve(std::size_t n) { points.reserve(n); }
  void resize(std::size_t n) {
    points.resize(n);
    width = static_cast<std::uint32_t>(n);
    height = 1;
  }
  void push_back(const PointT &p) {
    points.push_back(p);
    width = static_cast<std::uint32_t>(points.size());
    height = 1;
  }
  PointT &at(std::size_t i) { return points.at(i); }
  const PointT &at(std::size_t i) const { return points.at(i); }
  PointT &back() { return points.back(); }
  const PointT &back() const { return points.back(); }
  PointT &operator[](std::size_t i) { return points[i]; }
  const PointT &operator[](std::size_t i) const { return points[i]; }
  typename VectorType::iterator begin() { return points.begin(); }
  typename VectorType::iterator end() { return points.end(); }
  typename VectorType::const_iterator begin() const { return points.begin(); }
  typename VectorType::const_iterator end() const { return points.end(); }
  std::shared_ptr<PointCloud<PointT>> makeShared() const { return std::make_shared<PointCloud<PointT>>(*this); }
};

// out = T * in for the xyz of each point; other fields copied through.
// Aliasing-safe: &in may equal &out (LidarHelper does an in-place transform).
template <class PointT, class Derived>
void transformPointCloud(const PointCloud<PointT> &in, PointCloud<PointT> &out, const Eigen::MatrixBase<Derived> &T) {
  const Eigen::Matrix3f R = T.template block<3, 3>(0, 0).template cast<float>();
  const Eigen::Vector3f t = T.template block<3, 1>(0, 3).template cast<float>();
  typename PointCloud<PointT>::VectorType res(in.points.size());
  for (std::size_t i = 0; i < in.points.size(); i++) {
    PointT p = in.points[i];
    const Eigen::Vector3f q = R * Eigen::Vector3f(p.x, p.y, p.z) + t;
    p.x = q.x();
    p.y = q.y();
    p.z = q.z();
    res[i] = p;
  }
  out.points = std::move(res);
  out.width = static_cast<std::uint32_t>(out.points.size());
  out.height = 1;
}

// Transform a single point by an affine (rotation + translation). Found via ADL on the point
// type, the way pcl::transformPoint was. Mirrors LidarHelper's per-point transform.
template <class PointT> PointT transformPoint(const PointT &in, const Eigen::Affine3f &T) {
  PointT out = in;
  const Eigen::Vector3f q = T * Eigen::Vector3f(in.x, in.y, in.z);
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  return out;
}

// --- centroid voxel-grid downsample (matches pcl::VoxelGrid: one point per occupied voxel = mean) ---
inline void voxel_add(Eigen::Vector4f &s, const PointXYZ &p) { s += Eigen::Vector4f(p.x, p.y, p.z, 0.0f); }
inline void voxel_add(Eigen::Vector4f &s, const PointXYZI &p) { s += Eigen::Vector4f(p.x, p.y, p.z, p.intensity); }
inline PointXYZ voxel_mean(const PointXYZ &, const Eigen::Vector4f &m) { return PointXYZ(m.x(), m.y(), m.z()); }
inline PointXYZI voxel_mean(const PointXYZI &, const Eigen::Vector4f &m) { return PointXYZI(m.x(), m.y(), m.z(), m.w()); }

template <class PointT> void voxel_downsample(const PointCloud<PointT> &in, PointCloud<PointT> &out, float leaf) {
  if (leaf <= 0.0f || in.points.empty()) {
    if (&out != &in)
      out = in;
    return;
  }
  // index origin at the cloud min corner, as pcl does, so voxel boundaries line up
  Eigen::Vector3f mn(INFINITY, INFINITY, INFINITY);
  for (const auto &p : in.points)
    mn = mn.cwiseMin(Eigen::Vector3f(p.x, p.y, p.z));
  const float inv = 1.0f / leaf;
  struct Acc {
    Eigen::Vector4f sum = Eigen::Vector4f::Zero();
    int n = 0;
  };
  // Accumulate in a hash map (O(n)), then sort the (few) occupied voxels by key so the
  // output order is deterministic - important because it feeds the ikd-Tree insertion order.
  std::unordered_map<std::int64_t, Acc> voxels;
  voxels.reserve(in.points.size());
  for (const auto &p : in.points) {
    auto ijk = [&](float c, float m) { return static_cast<std::int64_t>(std::floor((c - m) * inv)); };
    std::int64_t i = ijk(p.x, mn.x()), j = ijk(p.y, mn.y()), k = ijk(p.z, mn.z());
    std::int64_t key = (i & 0x1FFFFF) | ((j & 0x1FFFFF) << 21) | ((k & 0x1FFFFF) << 42);
    Acc &a = voxels[key];
    voxel_add(a.sum, p);
    a.n++;
  }
  std::vector<const std::pair<const std::int64_t, Acc> *> items;
  items.reserve(voxels.size());
  for (const auto &kv : voxels)
    items.push_back(&kv);
  std::sort(items.begin(), items.end(), [](auto *a, auto *b) { return a->first < b->first; });
  out.points.clear();
  out.points.reserve(items.size());
  for (const auto *kv : items)
    out.points.push_back(voxel_mean(PointT(), kv->second.sum / static_cast<float>(kv->second.n)));
  out.width = static_cast<std::uint32_t>(out.points.size());
  out.height = 1;
}

} // namespace mins

#endif // MINS_LIDAR_POINTCLOUD_H

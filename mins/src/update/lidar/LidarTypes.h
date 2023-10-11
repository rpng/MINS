/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
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

#ifndef MINS_LIDARTYPES_H
#define MINS_LIDARTYPES_H

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>

using namespace Eigen;
namespace pcl {
class PointXYZ;
class PointXYZI;
template <class pointT> class PointCloud;
} // namespace pcl
typedef std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> POINTCLOUD_XYZI_PTR;
template <class pointT> class KD_TREE;

namespace mins {
struct LiDARData {
  LiDARData(double time, double ref_time, int id, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointcloud, double max_range, double min_range);

  LiDARData();

  /// Timestamp of the reading
  double time;

  /// ID of the LiDAR
  int id;

  /// Boolean for icp success
  bool icp_success;

  /// Transformation from LiDAR to Map
  Eigen::Matrix4f T_LtoM;

  /// Down sampled (if applied) pointcloud. Note that we use intensity as timestamp of the point
  POINTCLOUD_XYZI_PTR pointcloud;

  /// Down sampled (if applied) pointcloud. Note that we use intensity as timestamp of the point
  POINTCLOUD_XYZI_PTR pointcloud_in_map;

  /// Original pointcloud. Note that we use intensity as timestamp of the point
  POINTCLOUD_XYZI_PTR pointcloud_original;

  /// Return position of the point corresponding to the given ID
  Eigen::Vector3d p(int p_id);
  Eigen::Vector3d pfinM(int p_id);

  /// Size of the pointcloud (possibly down sampled)
  int size();
};

struct iKDDATA {
  iKDDATA(int id);
  // ikd tree
  std::shared_ptr<KD_TREE<pcl::PointXYZI>> tree;
  // time this tree (map) is anchored at
  double time;
  // ID of corresponding LiDAR
  int id;
  // last time this map is updated
  double last_up_time;
};
} // namespace mins

#endif // MINS_LIDARTYPES_H

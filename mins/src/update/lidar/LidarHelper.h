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

#ifndef MINS_LIDARHELPER_H
#define MINS_LIDARHELPER_H
#include <memory>
#include <pointmatcher/PointMatcher.h>

using namespace std;
using namespace Eigen;

namespace pcl {
class PointXYZ;
class PointXYZI;
template <class pointT> class PointCloud;
} // namespace pcl
typedef std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> POINTCLOUD_XYZI_PTR;
template <class pointT> class KD_TREE;
namespace mins {
class State;
struct iKDDATA;
struct LiDARData;
struct OptionsLidar;
class LidarHelper {

public:
  /// Remove motion blue using CPI
  static bool remove_motion_blur(shared_ptr<State> state, shared_ptr<LiDARData> lidar_inL, shared_ptr<OptionsLidar> op);

  /// Downsample LiDAR pointcloud
  static void downsample(shared_ptr<LiDARData> lidar, double downsample_size);
  static void downsample(POINTCLOUD_XYZI_PTR lidar, double downsample_size);

  /// Initialize Map given LiDAR pointcloud in local LiDAR frame lidar_inL
  static void init_map_local(const shared_ptr<LiDARData> &lidar_inL, shared_ptr<iKDDATA> ikd, shared_ptr<OptionsLidar> op, bool prop = false);

  /// Transform lidar pointcloud to the map frame
  static bool transform_to_map(shared_ptr<State> state, shared_ptr<LiDARData> lidar, shared_ptr<iKDDATA> ikd);

  /// Register a new lidar scan to the map
  static void register_scan(shared_ptr<State> state, shared_ptr<LiDARData> lidar, shared_ptr<iKDDATA> ikd);

  /// propagate the map to the newest clone time
  static void propagate_map_to_newest_clone(shared_ptr<State> state, shared_ptr<iKDDATA> ikd, shared_ptr<OptionsLidar> op, double FT);

  /// Get neighbors with checks
  static bool get_neighbors(Vector3d pfinM, POINTCLOUD_XYZI_PTR neighbors, shared_ptr<KD_TREE<pcl::PointXYZI>> tree, shared_ptr<OptionsLidar> op);

  /// compute the plane information with given pointcloud
  static bool compute_plane(Vector4d &plane_abcd, POINTCLOUD_XYZI_PTR pointcloud, shared_ptr<OptionsLidar> op);

  /// format convert
  static PointMatcher<float>::DataPoints PCL2DM(POINTCLOUD_XYZI_PTR pcl_points);
};
} // namespace mins

#endif // MINS_LIDARHELPER_H

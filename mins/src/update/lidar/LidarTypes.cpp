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

#include "LidarTypes.h"
#include "ikd_Tree.h"
#include "pcl/point_cloud.h"

using namespace mins;
LiDARData::LiDARData(double time, double ref_time, int id, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pointcloud, double max_range, double min_range) : time(time), id(id) {
  // copy over the point cloud
  // copy measurement time of the point so that it can be filtered with time.
  // Note: we only store relative time to "reference time" because intensity (float) has only 4 bytes
  // Note: this "reference time" should be the same for all other pointclouds.
  this->pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  this->pointcloud_original = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  this->pointcloud_in_map = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  this->pointcloud->points.reserve(pointcloud->points.size());
  this->pointcloud_original->points.reserve(pointcloud->points.size());
  for (int i = 0; i < (int)pointcloud->size(); i++) {
    // filter out-of-range measurements
    float n = pointcloud->points[i].getVector3fMap().norm();
    if (n > max_range || n < min_range)
      continue;

    // Copy
    this->pointcloud->push_back(pcl::PointXYZI());
    this->pointcloud->back().x = pointcloud->points[i].x;
    this->pointcloud->back().y = pointcloud->points[i].y;
    this->pointcloud->back().z = pointcloud->points[i].z;
    this->pointcloud->back().intensity = time - ref_time;

    // Copy
    this->pointcloud_original->push_back(pcl::PointXYZI());
    this->pointcloud_original->back().x = pointcloud->points[i].x;
    this->pointcloud_original->back().y = pointcloud->points[i].y;
    this->pointcloud_original->back().z = pointcloud->points[i].z;
    this->pointcloud_original->back().intensity = time - ref_time;
  }
}

LiDARData::LiDARData() {
  time = -1;
  id = -1;
  pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pointcloud_original = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pointcloud_in_map = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
}

Eigen::Vector3d LiDARData::p(int p_id) {
  auto p = pointcloud->points.at(p_id);
  return {p.x, p.y, p.z};
}

Eigen::Vector3d LiDARData::pfinM(int p_id) {
  auto p = pointcloud_in_map->points.at(p_id);
  return {p.x, p.y, p.z};
}

/// Size of the pointcloud (possibly down sampled)
int LiDARData::size() { return pointcloud->size(); }

iKDDATA::iKDDATA(int id) {
  tree = std::make_shared<KD_TREE<pcl::PointXYZI>>();
  this->id = id;
}
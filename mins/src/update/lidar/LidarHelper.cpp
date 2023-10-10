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

#include "LidarHelper.h"
#include "LidarTypes.h"
#include "ikd_Tree.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsLidar.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "state/State.h"
#include "types/PoseJPL.h"
#include "utils/Jabdongsani.h"
#include "utils/Print_Logger.h"
#include "utils/colors.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

using namespace mins;
using namespace ov_type;

bool LidarHelper::remove_motion_blur(shared_ptr<State> state, shared_ptr<LiDARData> lidar_inL, shared_ptr<OptionsLidar> op) {
  // if this option is not enabled, return true
  if (!op->raw_remove_motion_blur)
    return true;

  // Here we undistort the lidar pointcloud using state
  // Get calibration info
  Matrix3f RItoL = state->lidar_extrinsic.at(lidar_inL->id)->Rot().cast<float>();
  Matrix3f RLtoI = RItoL.transpose();
  Vector3f pIinL = state->lidar_extrinsic.at(lidar_inL->id)->pos().cast<float>();
  Vector3f pLinI = -RLtoI * pIinL;
  double dt = state->lidar_dt.at(lidar_inL->id)->value()(0);

  // Make sure we can undistort all the points
  auto D = Dummy();
  if (state->get_interpolated_pose(lidar_inL->time + dt, D.R, D.p))
    return false;
  int total = op->v_angles.at(lidar_inL->id).size() * op->h_angles.at(lidar_inL->id).size();
  if (state->get_interpolated_pose(lidar_inL->time + dt - total * op->raw_point_dt, D.R, D.p))
    return false;

  // Get IMU pose at the measurement time
  Matrix3d RGtoI;
  Vector3d pIinG;
  bool success = state->get_interpolated_pose(lidar_inL->time + dt, RGtoI, pIinG);
  assert(success);

  // Precompute lidar pose in Global
  Matrix3f RGtoL = RItoL * RGtoI.cast<float>();
  Vector3f pLinG = pIinG.cast<float>() + RGtoI.transpose().cast<float>() * pLinI;

  // now loop though point cloud and transform it to the first point time
  for (int i = 0; i < (int)lidar_inL->pointcloud->size(); i++) {
    // Get IMU pose at time i
    Matrix3d RGtoIi;
    Vector3d pIiinG;
    success = state->get_interpolated_pose(lidar_inL->time + dt - (total - i) * op->raw_point_dt, RGtoIi, pIiinG);
    assert(success);

    // Compute transform from Li to L
    Affine3f T_LitoL;
    T_LitoL.matrix().block(0, 0, 3, 3) = RGtoL * RGtoIi.transpose().cast<float>() * RLtoI;
    T_LitoL.matrix().block(0, 3, 3, 1) = RGtoL * (pIiinG.cast<float>() + RGtoIi.transpose().cast<float>() * pLinI - pLinG);

    // Transform this point
    lidar_inL->pointcloud->points[i] = transformPoint(lidar_inL->pointcloud->points[i], T_LitoL);
  }
  return true;
}

void LidarHelper::downsample(shared_ptr<LiDARData> lidar, double downsample_size) {
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
  downSizeFilter.setLeafSize((float)downsample_size, (float)downsample_size, (float)downsample_size);
  downSizeFilter.setInputCloud((*lidar->pointcloud).makeShared());
  downSizeFilter.filter(*lidar->pointcloud);
}

void LidarHelper::downsample(POINTCLOUD_XYZI_PTR lidar, double downsample_size) {
  pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
  downSizeFilter.setLeafSize((float)downsample_size, (float)downsample_size, (float)downsample_size);
  downSizeFilter.setInputCloud((*lidar).makeShared());
  downSizeFilter.filter(*lidar);
}

void LidarHelper::init_map_local(const shared_ptr<LiDARData> &lidar_inL, shared_ptr<iKDDATA> ikd, shared_ptr<OptionsLidar> op, bool prop) {
  /// Initialize map using lidar scan in lidar frame.
  // Reset if already initialized
  ikd->tree->initialized() ? ikd->tree->reset() : void();

  // Register pointcloud to the map
  ikd->time = lidar_inL->time;
  ikd->last_up_time = prop ? ikd->last_up_time : lidar_inL->time;
  ikd->tree->set_downsample_param(op->map_downsample_size);
  ikd->tree->Build(lidar_inL->pointcloud->points);
}

bool LidarHelper::transform_to_map(shared_ptr<State> state, shared_ptr<LiDARData> lidar, shared_ptr<iKDDATA> ikd) {
  //===================================================================
  // Compute Transformation using Estimate
  //===================================================================
  shared_ptr<PoseJPL> calibration = state->lidar_extrinsic.at(lidar->id);
  shared_ptr<Vec> timeoffset = state->lidar_dt.at(lidar->id);
  double dt = timeoffset->value()(0);
  Matrix3d RItoL = calibration->Rot();
  Matrix3d RLtoI = RItoL.transpose();
  Vector3d pIinL = calibration->pos();
  Vector3d pLinI = -RLtoI * pIinL;

  // Get interpolated pose for lidar
  Matrix3d RGtoI, RGtoA;
  Vector3d pIinG, pAinG;
  if (!state->get_interpolated_pose(lidar->time + dt, RGtoI, pIinG)) {
    PRINT4("UpdaterLidar::register_scan::Cannot get pose for LiDAR at %.4f\n", lidar->time + dt);
    return false;
  }
  state->get_interpolated_pose(ikd->time + dt, RGtoA, pAinG);
  if (!state->get_interpolated_pose(ikd->time + dt, RGtoA, pAinG)) {
    PRINT4("UpdaterLidar::register_scan::Cannot get pose for map at %.4f\n", ikd->time + dt);
    state->print_info();
    assert(0);
  }

  // New LiDAR frame
  Matrix3d RGtoL = RItoL * RGtoI;
  Vector3d pLinG = pIinG + RGtoI.transpose() * pLinI;
  Matrix3d RGtoM = RItoL * RGtoA;
  Vector3d pMinG = pAinG + RGtoA.transpose() * pLinI;
  Matrix3d RLtoM = RGtoM * RGtoL.transpose();
  Vector3d pLinM = RGtoM * (pLinG - pMinG);

  Matrix4f T_LtoM = Matrix4f::Identity();
  T_LtoM.block(0, 0, 3, 3) = RLtoM.cast<float>();
  T_LtoM.block(0, 3, 3, 1) = pLinM.cast<float>();

  //===================================================================
  // Run ICP if we use it to register the map
  //===================================================================
  if (state->op->lidar->map_use_icp) {
    POINTCLOUD_XYZI_PTR map_pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    ikd->tree->flatten(ikd->tree->Root_Node, map_pointcloud->points, NOT_RECORD);
    POINTCLOUD_XYZI_PTR new_pointcloud = lidar->pointcloud;

    typedef PointMatcher<float> PM;
    PM::ICP icp;
    icp.setDefault();
    // Adjust referenceDataPointsFilters
    auto params = PM::Parameters();
    params["maxDist"] = to_string(state->op->lidar->map_icp_dist);
    icp.referenceDataPointsFilters.clear();
    icp.referenceDataPointsFilters.push_back(PM::get().DataPointsFilterRegistrar.create("MaxDistDataPointsFilter", params));
    icp.referenceDataPointsFilters.push_back(PM::get().DataPointsFilterRegistrar.create("SamplingSurfaceNormalDataPointsFilter"));

    PM::DataPoints map_points = PCL2DM(map_pointcloud);
    PM::DataPoints new_points = PCL2DM(new_pointcloud);
    PM::TransformationParameters T_icp = icp.compute(new_points, map_points, T_LtoM);

    // Get transform
    Matrix4f T_diff = T_icp * T_LtoM.inverse();
    if (icp.getMaxNumIterationsReached() || T_diff.block(0, 3, 3, 1).norm() > 1) {
      lidar->T_LtoM = T_LtoM;
      lidar->icp_success = false;
    } else {
      lidar->T_LtoM = T_icp;
      lidar->icp_success = true;
    }
  } else {
    lidar->T_LtoM = T_LtoM;
    lidar->icp_success = true;
  }

  //===================================================================
  // Register new scan
  //===================================================================
  pcl::transformPointCloud(*(lidar->pointcloud), *(lidar->pointcloud_in_map), lidar->T_LtoM);
  return true;
}

void LidarHelper::register_scan(shared_ptr<State> state, shared_ptr<LiDARData> lidar, shared_ptr<iKDDATA> ikd) {
  // If ICP failed, retry it here
  lidar->icp_success ? bool() : transform_to_map(state, lidar, ikd);

  // If we have successful ICP, use the info to register the scan in the map
  if (lidar->icp_success) {
    pcl::PointCloud<pcl::PointXYZI> tr_points;
    pcl::transformPointCloud(*(lidar->pointcloud_original), tr_points, lidar->T_LtoM);
    ikd->tree->Add_Points(tr_points.points, state->op->lidar->map_do_downsample);
    ikd->last_up_time = lidar->time;
  }
}

void LidarHelper::propagate_map_to_newest_clone(shared_ptr<State> state, shared_ptr<iKDDATA> ikd, shared_ptr<OptionsLidar> op, double FT) {

  // Load map info
  POINTCLOUD_XYZI_PTR map_points = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  ikd->tree->flatten(ikd->tree->Root_Node, map_points->points, NOT_RECORD);
  op->map_do_downsample ? downsample(map_points, state->op->lidar->map_downsample_size) : void();

  // Delete old points
  shared_ptr<LiDARData> lidar_inM = shared_ptr<LiDARData>(new LiDARData);
  lidar_inM->pointcloud->points.reserve(map_points->size());
  for (auto &pt : map_points->points) {
    if (pt.intensity + FT + op->map_decay_time < state->time)
      continue;
    if (Vector3f(pt.x, pt.y, pt.z).norm() > state->op->lidar->map_decay_dist)
      continue;
    // Keep this feature
    lidar_inM->pointcloud->push_back(pt);
  }

  // Load calibration info
  Matrix3d RItoL = state->lidar_extrinsic.at(ikd->id)->Rot();
  Matrix3d RLtoI = RItoL.transpose();
  Vector3d pIinL = state->lidar_extrinsic.at(ikd->id)->pos();
  Vector3d pLinI = -RLtoI * pIinL;
  double dt = state->lidar_dt.at(ikd->id)->value()(0);

  // Load state info
  Matrix3d RGtoIold;
  Vector3d pIoldinG;
  if (!state->get_interpolated_pose(ikd->time + dt, RGtoIold, pIoldinG)) {
    PRINT4(RED "[LiDAR]propagate_map_to_newest_clone::Cannot get the bounding_poses_n for Map %d!\n" RESET, ikd->id);
    PRINT4(RED "[LiDAR]propagate_map_to_newest_clone::ikd_anchor_time.at(lidar_inL->id): %.4f\n", ikd->time);
    PRINT4(RED "[LiDAR]propagate_map_to_newest_clone::dt: %.4f\n", dt);
    PRINT4(RED "[LiDAR]propagate_map_to_newest_clone::lidar_inL->time + dt: %.4f\n", ikd->time + dt);
    state->print_info();
    exit(EXIT_FAILURE);
  }
  Matrix3d RGtoInew = state->clones.at(state->newest_clone_time())->Rot();
  Vector3d pInewinG = state->clones.at(state->newest_clone_time())->pos();

  // Compute transformation
  Matrix3d RGtoLold = RItoL * RGtoIold;
  Vector3d pLoldinG = pIoldinG + RGtoIold.transpose() * pLinI;
  Matrix3d RGtoLnew = RItoL * RGtoInew;
  Vector3d pLnewinG = pInewinG + RGtoInew.transpose() * pLinI;
  Matrix3d RLoldtoLnew = RGtoLnew * RGtoLold.transpose();
  Vector3d pLoldinLnew = RGtoLnew * (pLoldinG - pLnewinG);
  Matrix4d tr = Matrix4d::Identity();
  tr.block(0, 0, 3, 3) = RLoldtoLnew;
  tr.block(0, 3, 3, 1) = pLoldinLnew;

  // transform the pointcloud
  pcl::transformPointCloud(*lidar_inM->pointcloud, *lidar_inM->pointcloud, tr);
  lidar_inM->id = ikd->id;
  lidar_inM->time = state->newest_clone_time() - dt;

  init_map_local(lidar_inM, ikd, state->op->lidar, true);
}

bool LidarHelper::get_neighbors(Vector3d pfinM, POINTCLOUD_XYZI_PTR neighbors, shared_ptr<KD_TREE<pcl::PointXYZI>> tree, shared_ptr<OptionsLidar> op) {
  // Transform the point to map to find neighbors from the map.
  // Note "_" mean pcl variable
  pcl::PointXYZI pfinM_;
  pfinM_.x = pfinM(0);
  pfinM_.y = pfinM(1);
  pfinM_.z = pfinM(2);
  // Now find neighbors from the map
  vector<float> neighbors_d;
  tree->Nearest_Search(pfinM_, op->map_ngbr_num, neighbors->points, neighbors_d);

  // continue if we didn't find enough points
  if ((int)neighbors->size() < op->map_ngbr_num || (int)neighbors_d.size() < op->map_ngbr_num)
    return false;

  // continue if near point distance is too far
  if (neighbors_d.back() > op->map_ngbr_max_d)
    return false;

  // All good.
  return true;
}

bool LidarHelper::compute_plane(Vector4d &plane_abcd, POINTCLOUD_XYZI_PTR pointcloud, shared_ptr<OptionsLidar> op) {
  // construct Ax = b
  MatrixXd A = MatrixXd::Zero(pointcloud->size(), 3);
  double d = 0;
  for (int j = 0; j < (int)pointcloud->size(); j++) {
    A(j, 0) = pointcloud->points[j].x;
    A(j, 1) = pointcloud->points[j].y;
    A(j, 2) = pointcloud->points[j].z;
    d += A.block(j, 0, 1, 3).norm();
  }
  d /= pointcloud->size();
  VectorXd b = -VectorXd::Ones(pointcloud->size()) * d;

  // Check the condition number of A.
  JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
  MatrixXd singularValues;
  singularValues.resize(svd.singularValues().rows(), 1);
  singularValues = svd.singularValues();
  double condA = singularValues(0, 0) / singularValues(singularValues.rows() - 1, 0);
  if (condA > op->plane_max_condi) {
    return false;
  }

  // construct un-scaled plane coefficients
  plane_abcd.head(3) = A.colPivHouseholderQr().solve(b);
  plane_abcd(3) = d;

  // sanity check
  Vector4d pl = plane_abcd / plane_abcd.head(3).norm();
  for (int j = 0; j < (int)pointcloud->size(); j++) {
    pcl::PointXYZI pt = pointcloud->points[j];
    if (fabs(pl(0) * pt.x + pl(1) * pt.y + pl(2) * pt.z + pl(3)) > op->plane_max_p2pd) {
      return false;
    }
  }
  // All good :)
  return true;
}

PointMatcher<float>::DataPoints LidarHelper::PCL2DM(POINTCLOUD_XYZI_PTR pcl_points) {
  // Labels. Can it work with xyzi??
  PointMatcher<float>::DataPoints::Labels labels;
  labels.push_back(PointMatcher<float>::DataPoints::Label("x", 1));
  labels.push_back(PointMatcher<float>::DataPoints::Label("y", 1));
  labels.push_back(PointMatcher<float>::DataPoints::Label("z", 1));
  labels.push_back(PointMatcher<float>::DataPoints::Label("pad", 1));

  // Convert the pcl points to matrix. Bottom row is 1 (pad)
  MatrixXf matrix = MatrixXf::Ones(4, pcl_points->size());
  for (int i = 0; i < pcl_points->size(); i++) {
    matrix(0, i) = pcl_points->points.at(i).x;
    matrix(1, i) = pcl_points->points.at(i).y;
    matrix(2, i) = pcl_points->points.at(i).z;
  }

  // return PointMatcher<float>::DataPoints
  return {matrix, labels};
}
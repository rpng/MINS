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

#include "SimVisualizer.h"
#include "SimulationPlane.h"
#include "core/ROSHelper.h"
#include "core/SystemManager.h"
#include "nav_msgs/Path.h"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsInit.h"
#include "options/OptionsLidar.h"
#include "options/OptionsSimulation.h"
#include "sensor_msgs/PointCloud2.h"
#include "sim/Simulator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "update/gps/PoseJPL_4DOF.h"
#include "update/gps/UpdaterGPS.h"
#include "utils/Print_Logger.h"
#include "utils/dataset_reader.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace ov_core;
using namespace ov_type;
using namespace mins;
using namespace visualization_msgs;
SimVisualizer::SimVisualizer(shared_ptr<ros::NodeHandle> nh, shared_ptr<SystemManager> sys, shared_ptr<Simulator> sim) : nh(nh), sys(sys), sim(sim) {

  // Publish simulated camera point cloud if simulation enabled
  if (sim != nullptr && sys->state->op->cam->enabled) {
    pub_sim_cam_points = make_shared<ros::Publisher>(nh->advertise<sensor_msgs::PointCloud2>("/mins/cam/points_sim", 2));
    PRINT1("Publishing: %s\n", pub_sim_cam_points->getTopic().c_str());
  }

  if (sim != nullptr && sys->state->op->lidar->enabled) {
    pub_sim_lidar_map = make_shared<ros::Publisher>(nh->advertise<MarkerArray>("/mins/sim_lidar_map", 2));
    PRINT1("Publishing: %s\n", pub_sim_lidar_map->getTopic().c_str());
  }

  // Groundtruth publishers
  mTfBr = make_shared<tf::TransformBroadcaster>();
  pub_posegt = make_shared<ros::Publisher>(nh->advertise<geometry_msgs::PoseStamped>("/mins/imu/pose_gt", 2));
  PRINT1("Publishing: %s\n", pub_posegt->getTopic().c_str());
  pub_pathgt = make_shared<ros::Publisher>(nh->advertise<nav_msgs::Path>("/mins/imu/path_gt", 2));
  PRINT1("Publishing: %s\n", pub_pathgt->getTopic().c_str());
}

void SimVisualizer::publish_sim_cam_features() {
  // Check if we have subscribers
  if (sim == nullptr || pub_sim_cam_points->getNumSubscribers() == 0)
    return;

  // Get our good SIMULATION features
  vector<Eigen::Vector3d> feats_sim = sim->get_cam_map_vec();
  sensor_msgs::PointCloud2 cloud_SIM = ROSHelper::ToPointcloud(feats_sim, "global");
  pub_sim_cam_points->publish(cloud_SIM);
}

void SimVisualizer::publish_groundtruth() {
  // Transform the path history if GNSS is initialized
  if (sys->state->op->gps->enabled && sys->up_gps->initialized && !traj_in_enu) {
    // Transform individual pose stored in the path
    for (auto &pose : poses_gt) {
      pose = ROSHelper::ToENU(pose, sys->state->trans_WtoE->value());
    }
    // We only transform the trajectory once.
    traj_in_enu = true;
  }

  // Our groundtruth state
  Eigen::Matrix<double, 17, 1> state_gt;
  if (sim != nullptr) { // Get ground truth from simulation if available
    if (!sim->get_imu_state(sys->state->time, state_gt))
      return;
  } else {                   // Otherwise, try to get it from the file
    if (gt_states.empty()) { // Load gt file if didn't get yet
      if (sys->state->op->init->path_gt.empty())
        return;
      DatasetReader::load_gt_file(sys->state->op->init->path_gt, gt_states);
    }
    // Get gt pose
    if (!DatasetReader::get_gt_state(sys->state->time, state_gt, gt_states))
      return;
  }

  // Create pose of IMU
  geometry_msgs::PoseStamped poseIinM;
  poseIinM.header.stamp = ros::Time(sys->state->time);
  poseIinM.header.seq = poses_seq_gt;
  poseIinM.header.frame_id = "global";
  poseIinM.pose.orientation.x = state_gt(1, 0);
  poseIinM.pose.orientation.y = state_gt(2, 0);
  poseIinM.pose.orientation.z = state_gt(3, 0);
  poseIinM.pose.orientation.w = state_gt(4, 0);
  poseIinM.pose.position.x = state_gt(5, 0);
  poseIinM.pose.position.y = state_gt(6, 0);
  poseIinM.pose.position.z = state_gt(7, 0);
  pub_posegt->publish(poseIinM);

  // Append to our pose vector
  poses_gt.push_back(poseIinM);

  // Create our path (IMU)
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arrIMU;
  arrIMU.header.stamp = ros::Time::now();
  arrIMU.header.seq = poses_seq_gt;
  arrIMU.header.frame_id = "global";
  for (size_t i = 0; i < poses_gt.size(); i += floor((double)poses_gt.size() / 16384.0) + 1) {
    arrIMU.poses.push_back(poses_gt[i]);
  }
  pub_pathgt->publish(arrIMU);

  // Move them forward in time
  poses_seq_gt++;

  // Publish our transform on TF
  tf::StampedTransform trans;
  trans.stamp_ = ros::Time::now();
  trans.frame_id_ = "global";
  trans.child_frame_id_ = "truth";
  tf::Quaternion quat(state_gt(1, 0), state_gt(2, 0), state_gt(3, 0), state_gt(4, 0));
  trans.setRotation(quat);
  tf::Vector3 orig(state_gt(5, 0), state_gt(6, 0), state_gt(7, 0));
  trans.setOrigin(orig);
  mTfBr->sendTransform(trans);

  //==========================================================================
  //==========================================================================
  if (sim != nullptr) {
    auto imu_pose = sys->state->imu->pose();
    Vector4d rmse_nees = sim->imu_rmse_nees(sys->state->time, imu_pose->value(), StateHelper::get_marginal_covariance(sys->state, {imu_pose}));

    // Update our average variables
    if (!isnan(rmse_nees(2)) && !isnan(rmse_nees(3))) {
      sum_rmse_ori += rmse_nees(0);
      sum_rmse_pos += rmse_nees(1);
      sum_nees_ori += rmse_nees(2);
      sum_nees_pos += rmse_nees(3);
      sum_cnt++;
    }

    // Nice display for the user
    printf(REDPURPLE);
    PRINT2("\033[A%.2f | RMSE: %.3f, %.3f (deg,m) | ", sys->state->time, rmse_nees(0), rmse_nees(1));
    PRINT2("RMSE avg: %.3f, %.3f (deg,m) | ", sum_rmse_ori / sum_cnt, sum_rmse_pos / sum_cnt);
    PRINT2("NEES: %.1f, %.1f | ", rmse_nees(2), rmse_nees(3));
    PRINT2("NEES avg: %.1f, %.1f\n\n", sum_nees_ori / sum_cnt, sum_nees_pos / sum_cnt);
    printf(RESET);
  }
}

void SimVisualizer::publish_lidar_structure() {
  // Skip the rest of we are not doing simulation
  if (sim == nullptr || sim->get_lidar_planes().empty())
    return;

  // copy over the simulated planes
  vector<shared_ptr<SimulationPlane>> planes = sim->get_lidar_planes();

  // Transform the planes if GPS is enabled and initialized
  Matrix3d RWtoE = quat_2_Rot(sim->op->sim->WtoE_trans.block(0, 0, 4, 1));
  Vector3d pWinE = sim->op->sim->WtoE_trans.block(4, 0, 3, 1);
  for (auto &plane : planes) {
    plane->pt_top_left = sim->trans_gt_to_ENU ? pWinE + RWtoE * plane->pt_top_left.eval() : plane->pt_top_left.eval();
    plane->pt_top_right = sim->trans_gt_to_ENU ? pWinE + RWtoE * plane->pt_top_right.eval() : plane->pt_top_right.eval();
    plane->pt_bottom_left = sim->trans_gt_to_ENU ? pWinE + RWtoE * plane->pt_bottom_left.eval() : plane->pt_bottom_left.eval();
    plane->pt_bottom_right = sim->trans_gt_to_ENU ? pWinE + RWtoE * plane->pt_bottom_right.eval() : plane->pt_bottom_right.eval();
  }

  // Our marker array
  MarkerArray marker_arr;
  // Else lets get all the planes
  int ct = 0;
  for (auto &plane : planes) {
    // Our plane will be a line list
    Marker marker_plane;
    marker_plane.header.frame_id = "global";
    marker_plane.header.stamp = ros::Time::now();
    marker_plane.ns = "sim_lidar_map";
    marker_plane.id = ct;
    marker_plane.type = Marker::LINE_LIST;
    marker_plane.action = Marker::MODIFY;
    marker_plane.scale.x = 0.03;
    marker_plane.color.b = 1.0;
    marker_plane.color.a = 1.0;

    // Convert our 4 points to the right format
    geometry_msgs::Point pt_tl, pt_tr, pt_bl, pt_br;
    pt_tl.x = plane->pt_top_left(0);
    pt_tl.y = plane->pt_top_left(1);
    pt_tl.z = plane->pt_top_left(2);
    pt_tr.x = plane->pt_top_right(0);
    pt_tr.y = plane->pt_top_right(1);
    pt_tr.z = plane->pt_top_right(2);
    pt_bl.x = plane->pt_bottom_left(0);
    pt_bl.y = plane->pt_bottom_left(1);
    pt_bl.z = plane->pt_bottom_left(2);
    pt_br.x = plane->pt_bottom_right(0);
    pt_br.y = plane->pt_bottom_right(1);
    pt_br.z = plane->pt_bottom_right(2);

    // Add the 4 bounding points
    marker_plane.points.push_back(pt_tl);
    marker_plane.points.push_back(pt_tr);
    marker_plane.points.push_back(pt_tr);
    marker_plane.points.push_back(pt_br);
    marker_plane.points.push_back(pt_br);
    marker_plane.points.push_back(pt_bl);
    marker_plane.points.push_back(pt_bl);
    marker_plane.points.push_back(pt_tl);

    // Add cross across middle of the plane
    marker_plane.points.push_back(pt_tl);
    marker_plane.points.push_back(pt_br);
    marker_plane.points.push_back(pt_tr);
    marker_plane.points.push_back(pt_bl);

    // Append and move plane count forward
    marker_arr.markers.push_back(marker_plane);
    ct++;
  }

  pub_sim_lidar_map->publish(marker_arr);
}

void SimVisualizer::visualize_final() {
  PRINT2(BOLDYELLOW "RMSE average: %.3f, %.3f (deg,m)\n" RESET, sum_rmse_ori / sum_cnt, sum_rmse_pos / sum_cnt);
  PRINT2(BOLDYELLOW "NEES average: %.3f, %.3f (deg,m)\n" RESET, sum_nees_ori / sum_cnt, sum_nees_pos / sum_cnt);
}
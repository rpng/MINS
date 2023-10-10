/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This code is implemented based on:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "ROSPublisher.h"
#include "ROSHelper.h"
#include "SystemManager.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsIMU.h"
#include "options/OptionsLidar.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "update/cam/UpdaterCamera.h"
#include "update/gps/GPSTypes.h"
#include "update/gps/MathGPS.h"
#include "update/gps/PoseJPL_4DOF.h"
#include "update/gps/UpdaterGPS.h"
#include "update/lidar/LidarTypes.h"
#include "update/lidar/UpdaterLidar.h"
#include "update/lidar/ikd_Tree.h"
#include "update/vicon/ViconTypes.h"
#include "utils/Print_Logger.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>

using namespace std;
using namespace Eigen;
using namespace mins;
using namespace ov_type;
ROSPublisher::ROSPublisher(shared_ptr<ros::NodeHandle> nh, shared_ptr<SystemManager> sys, shared_ptr<Options> op) : nh(nh), sys(sys), op(op) {

  // Setup our transform broadcaster
  mTfBr = make_shared<tf::TransformBroadcaster>();

  // Basic IMU publish
  pub_imu_pose = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/mins/imu/pose", 2);
  PRINT1("Publishing: %s\n", pub_imu_pose.getTopic().c_str());
  pub_imu_odom = nh->advertise<nav_msgs::Odometry>("/mins/imu/odom", 2);
  PRINT1("Publishing: %s\n", pub_imu_odom.getTopic().c_str());
  pub_imu_path = nh->advertise<nav_msgs::Path>("/mins/imu/path", 2);
  PRINT1("Publishing: %s\n", pub_imu_path.getTopic().c_str());

  // CAM
  if (sys->state->op->cam->enabled) {
    pub_cam_msckf = nh->advertise<sensor_msgs::PointCloud2>("/mins/cam/msckf", 2);
    PRINT1("Publishing: %s\n", pub_cam_msckf.getTopic().c_str());
    pub_cam_slam = nh->advertise<sensor_msgs::PointCloud2>("/mins/cam/slam", 2);
    PRINT1("Publishing: %s\n", pub_cam_msckf.getTopic().c_str());
    for (int i = 0; i < op->est->cam->max_n; i++) {
      image_transport::ImageTransport it(*nh); // Our tracking image
      pub_cam_image.push_back(it.advertise("/mins/cam" + to_string(i) + "/track_img", 2));
      PRINT1("Publishing: %s\n", pub_cam_image.back().getTopic().c_str());
    }
  }

  // GPS
  if (sys->state->op->gps->enabled) {
    for (int i = 0; i < sys->state->op->gps->max_n; i++) {
      seq_gps.push_back(0);
      pub_gps_pose.emplace_back();
      pub_gps_pose.back() = nh->advertise<geometry_msgs::PoseStamped>("/mins/gps" + to_string(i) + "/pose", 2);
      PRINT1("Publishing: %s\n", pub_gps_pose.back().getTopic().c_str());
      pub_gps_path.emplace_back();
      pub_gps_path.back() = nh->advertise<nav_msgs::Path>("/mins/gps" + to_string(i) + "/path", 2);
      PRINT1("Publishing: %s\n", pub_gps_path.back().getTopic().c_str());
    }
    path_gps = vector<vector<geometry_msgs::PoseStamped>>(sys->state->op->gps->max_n);
  }

  // VICON
  if (sys->state->op->vicon->enabled) {
    for (int i = 0; i < sys->state->op->vicon->max_n; i++) {
      seq_vicon.push_back(0);
      pub_vicon_pose.emplace_back();
      pub_vicon_pose.back() = nh->advertise<geometry_msgs::PoseStamped>("/mins/vicon" + to_string(i) + "/pose", 2);
      PRINT1("Publishing: %s\n", pub_vicon_pose.back().getTopic().c_str());
      pub_vicon_path.emplace_back();
      pub_vicon_path.back() = nh->advertise<nav_msgs::Path>("/mins/vicon" + to_string(i) + "/path", 2);
      PRINT1("Publishing: %s\n", pub_vicon_path.back().getTopic().c_str());
    }
    path_vicon = vector<vector<geometry_msgs::PoseStamped>>(sys->state->op->vicon->max_n);
  }

  // LIDAR
  if (sys->state->op->lidar->enabled) {
    for (int i = 0; i < sys->state->op->lidar->max_n; i++) {
      pub_lidar_cloud.emplace_back();
      pub_lidar_cloud.back() = nh->advertise<sensor_msgs::PointCloud2>("/mins/lidar" + to_string(i) + "/points", 2);
      PRINT1("Publishing: %s\n", pub_lidar_cloud.back().getTopic().c_str());
      pub_lidar_map.emplace_back();
      pub_lidar_map.back() = nh->advertise<sensor_msgs::PointCloud2>("/mins/lidar" + to_string(i) + "/map", 2);
      PRINT1("Publishing: %s\n", pub_lidar_map.back().getTopic().c_str());
    }
  }
}

void ROSPublisher::visualize() {
  // Return if we have not inited
  if (!sys->state->initialized)
    return;

  // publish state
  publish_state();

  // publish camera
  if (sys->state->op->cam->enabled)
    publish_cam_features();

  // publish lidar
  if (sys->state->op->lidar->enabled)
    publish_lidar_map();
}

void ROSPublisher::publish_imu() {

  // Return if we have not initialized
  if (!sys->state->initialized)
    return;

  // Our odometry message
  Matrix<double, 13, 1> odom;
  odom.block(0, 0, 4, 1) = sys->state->imu->quat();
  odom.block(4, 0, 3, 1) = sys->state->imu->pos();
  odom.block(7, 0, 3, 1) = sys->state->imu->vel();
  odom.block(10, 0, 3, 1) = sys->state->have_cpi(sys->state->time) ? sys->state->cpis.at(sys->state->time).w : Vector3d::Zero();
  nav_msgs::Odometry odomIinG = ROSHelper::ToOdometry(odom);
  odomIinG.header.stamp = ros::Time(sys->state->time);
  odomIinG.header.frame_id = "global";
  odomIinG.child_frame_id = "imu";

  // Finally set the covariance in the message (in the order position then orientation as per ros convention)
  // TODO: this currently is an approximation since this should actually evolve over our propagation period
  // TODO: but to save time we only propagate the mean and not the uncertainty, but maybe we should try to prop the covariance?
  vector<shared_ptr<Type>> var_pq, var_v;
  var_pq.push_back(sys->state->imu->pose()->p());
  var_pq.push_back(sys->state->imu->pose()->q());
  var_v.push_back(sys->state->imu->v());
  Matrix<double, 6, 6> covariance_posori = StateHelper::get_marginal_covariance(sys->state, var_pq);
  Matrix<double, 6, 6> covariance_linang = pow(op->est->imu->sigma_w, 2) * Matrix<double, 6, 6>::Identity();
  covariance_linang.block(0, 0, 3, 3) = StateHelper::get_marginal_covariance(sys->state, var_v);
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      odomIinG.pose.covariance[6 * r + c] = covariance_posori(r, c);
      odomIinG.twist.covariance[6 * r + c] = (isnan(covariance_linang(r, c))) ? 0 : covariance_linang(r, c);
    }
  }

  // Finally, publish the resulting odometry message
  pub_imu_odom.publish(odomIinG);

  // Publish TF
  publish_tf();
}

void ROSPublisher::publish_tf() {
  // Publish our transform on TF
  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
  // NOTE: a rotation from GtoI in JPL has the same xyzw as a ItoG Hamilton rotation
  tf::StampedTransform trans = ROSHelper::Pose2TF(sys->state->imu->pose(), false);
  trans.frame_id_ = "global";
  trans.child_frame_id_ = "imu";
  mTfBr->sendTransform(trans);

  // Loop through each sensor calibration and publish it
  if (sys->state->op->cam->enabled) {
    for (const auto &calib : sys->state->cam_extrinsic) {
      tf::StampedTransform trans_calib = ROSHelper::Pose2TF(calib.second, true);
      trans_calib.frame_id_ = "imu";
      trans_calib.child_frame_id_ = "cam" + to_string(calib.first);
      mTfBr->sendTransform(trans_calib);
    }
  }

  if (sys->state->op->vicon->enabled) {
    for (const auto &calib : sys->state->vicon_extrinsic) {
      tf::StampedTransform trans_calib = ROSHelper::Pose2TF(calib.second, true);
      trans_calib.frame_id_ = "imu";
      trans_calib.child_frame_id_ = "vicon" + to_string(calib.first);
      mTfBr->sendTransform(trans_calib);
    }
  }

  if (sys->state->op->gps->enabled) {
    for (const auto &calib : sys->state->gps_extrinsic) {
      tf::StampedTransform trans_calib = ROSHelper::Pos2TF(calib.second, true);
      trans_calib.frame_id_ = "imu";
      trans_calib.child_frame_id_ = "gps" + to_string(calib.first);
      mTfBr->sendTransform(trans_calib);
    }
  }

  if (sys->state->op->lidar->enabled) {
    for (const auto &calib : sys->state->lidar_extrinsic) {
      tf::StampedTransform trans_calib = ROSHelper::Pose2TF(calib.second, true);
      trans_calib.frame_id_ = "imu";
      trans_calib.child_frame_id_ = "lidar" + to_string(calib.first);
      mTfBr->sendTransform(trans_calib);
    }
  }

  if (sys->state->op->wheel->enabled) {
    tf::StampedTransform trans_calib = ROSHelper::Pose2TF(sys->state->wheel_extrinsic, true);
    trans_calib.frame_id_ = "imu";
    trans_calib.child_frame_id_ = "wheel";
    mTfBr->sendTransform(trans_calib);
  }

  // Publish clone poses
  int clone_count = 0;
  for (const auto &C : sys->state->clones) {
    tf::StampedTransform trans = ROSHelper::Pose2TF(C.second, false);
    trans.frame_id_ = "global";
    trans.child_frame_id_ = "c" + to_string(clone_count++);
    mTfBr->sendTransform(trans);
  }
}

void ROSPublisher::publish_state() {
  // Transform the path history if GNSS is initialized
  if (sys->state->op->gps->enabled && sys->up_gps->initialized && !traj_in_enu) {
    // Transform individual pose stored in the path
    for (auto &pose : path_imu) {
      pose = ROSHelper::ToENU(pose, sys->state->trans_WtoE->value());
    }
    // We only transform the trajectory once.
    traj_in_enu = true;
  }

  // Create pose of IMU (note we use the bag time)
  geometry_msgs::PoseWithCovarianceStamped poseIinM = ROSHelper::ToPoseCov(sys->state->imu->value().block(0, 0, 7, 1));
  poseIinM.header.stamp = ros::Time(sys->state->time);
  poseIinM.header.seq = seq_imu;
  poseIinM.header.frame_id = "global";

  // Finally set the covariance in the message (in the order position then orientation as per ros convention)
  vector<shared_ptr<Type>> statevars;
  statevars.push_back(sys->state->imu->pose()->p());
  statevars.push_back(sys->state->imu->pose()->q());
  Matrix<double, 6, 6> covariance_posori = StateHelper::get_marginal_covariance(sys->state, statevars);
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      poseIinM.pose.covariance[6 * r + c] = covariance_posori(r, c);
    }
  }
  pub_imu_pose.publish(poseIinM);

  //=========================================================
  //=========================================================

  // Append to our pose vector
  geometry_msgs::PoseStamped posetemp;
  posetemp.header = poseIinM.header;
  posetemp.pose = poseIinM.pose.pose;
  path_imu.push_back(posetemp);

  // Create our path (IMU)
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::Path arrIMU;
  arrIMU.header.stamp = ros::Time::now();
  arrIMU.header.seq = seq_imu;
  arrIMU.header.frame_id = "global";
  for (int i = 0; i < (int)path_imu.size(); i += floor((double)path_imu.size() / 16384.0) + 1) {
    arrIMU.poses.push_back(path_imu.at(i));
  }
  pub_imu_path.publish(arrIMU);

  // Move them forward in time
  seq_imu++;

  // Publish TF
  publish_tf();
}

void ROSPublisher::publish_cam_images(vector<int> cam_ids) {
  for (auto cam_id : cam_ids)
    publish_cam_images(cam_id);
}

void ROSPublisher::publish_cam_images(int cam_id) {
  // Publish image at cam 0 rate and have all the images from each camera
  if (cam_id != 0)
    return;

  for (int i = 0; i < op->est->cam->max_n; i++) {
    // skip if no subscriber
    if (pub_cam_image.at(i).getNumSubscribers() == 0)
      continue;

    // skip if this is larger id pair of stereo
    if (op->est->cam->stereo_pairs.find(i) != op->est->cam->stereo_pairs.end() && i > op->est->cam->stereo_pairs.at(i))
      continue;

    // Create our message & Publish
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    pub_cam_image.at(i).publish(cv_bridge::CvImage(header, "bgr8", sys->up_cam->get_track_img(i)).toImageMsg());
  }
}

void ROSPublisher::publish_cam_features() {

  // Check if we have subscribers
  if (pub_cam_msckf.getNumSubscribers() == 0 && pub_cam_slam.getNumSubscribers() == 0)
    return;

  // Get our good MSCKF features
  vector<Vector3d> feats_msckf = sys->up_cam->get_used_msckf();
  sensor_msgs::PointCloud2 cloud = ROSHelper::ToPointcloud(feats_msckf, "global");
  pub_cam_msckf.publish(cloud);

  // Get our good SLAM features
  vector<Vector3d> feats_slam = sys->state->get_features_SLAM();
  sensor_msgs::PointCloud2 cloud_SLAM = ROSHelper::ToPointcloud(feats_slam, "global");
  pub_cam_slam.publish(cloud_SLAM);
}

void ROSPublisher::publish_gps(GPSData gps, bool isGeodetic) {

  // Visualize GPS measurements when we have datum
  if (sys->gps_datum.hasNaN())
    return;

  // Convert from a geodetic WGS-84 coordinated to East-North-Up
  if (isGeodetic)
    gps.meas = MathGPS::GeodeticToEnu(gps.meas, sys->gps_datum);

  // Now put the measurement in publisher
  geometry_msgs::PoseStamped poseGPSinENU;
  poseGPSinENU.header.stamp = ros::Time::now();
  poseGPSinENU.header.seq = seq_gps[gps.id];
  poseGPSinENU.header.frame_id = "global";
  poseGPSinENU.pose.position.x = gps.meas(0);
  poseGPSinENU.pose.position.y = gps.meas(1);
  poseGPSinENU.pose.position.z = gps.meas(2);
  poseGPSinENU.pose.orientation.x = 0.0;
  poseGPSinENU.pose.orientation.y = 0.0;
  poseGPSinENU.pose.orientation.z = 0.0;
  poseGPSinENU.pose.orientation.w = 1.0;
  pub_gps_pose[gps.id].publish(poseGPSinENU);

  // Append to our poses vector and create GPS path
  path_gps[gps.id].push_back(poseGPSinENU);
  nav_msgs::Path arrGPS;
  arrGPS.header.stamp = ros::Time::now();
  arrGPS.header.seq = seq_gps[gps.id];
  arrGPS.header.frame_id = "global";
  arrGPS.poses = path_gps[gps.id];
  pub_gps_path[gps.id].publish(arrGPS);

  // move sequence forward
  seq_gps[gps.id]++;
}

void ROSPublisher::publish_vicon(ViconData data) {

  // Now put the measurement in publisher
  geometry_msgs::PoseStamped poseVicon;
  poseVicon.header.stamp = ros::Time::now();
  poseVicon.header.seq = seq_vicon[data.id];
  poseVicon.header.frame_id = "global";
  poseVicon.pose.position.x = data.pose(3, 0);
  poseVicon.pose.position.y = data.pose(4, 0);
  poseVicon.pose.position.z = data.pose(5, 0);
  Vector4d q = ov_core::rot_2_quat(ov_core::exp_so3(data.pose.block(0, 0, 3, 1)));
  poseVicon.pose.orientation.x = q(0);
  poseVicon.pose.orientation.y = q(1);
  poseVicon.pose.orientation.z = q(2);
  poseVicon.pose.orientation.w = q(3);
  pub_vicon_pose[data.id].publish(poseVicon);

  // Append to our poses vector and create vicon path
  path_vicon[data.id].push_back(poseVicon);
  nav_msgs::Path arrVicon;
  arrVicon.header.stamp = ros::Time::now();
  arrVicon.header.seq = seq_vicon[data.id];
  arrVicon.header.frame_id = "global";
  arrVicon.poses = path_vicon[data.id];
  pub_vicon_path[data.id].publish(arrVicon);

  // move sequence forward
  seq_vicon[data.id]++;
}

void ROSPublisher::publish_lidar_cloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar) {
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*lidar, output);
  output.header.frame_id = "lidar" + lidar->header.frame_id;
  output.header.stamp = ros::Time::now();
  pub_lidar_cloud.at(stoi(lidar->header.frame_id)).publish(output);
}

void ROSPublisher::publish_lidar_map() {
  // Publish LiDAR map
  for (const auto &ikd : sys->up_ldr->ikd_data) {
    // Return if the map is not initialized
    if (!ikd->tree->initialized())
      return;
    // Get sensor calibration parameters
    double dt = sys->state->lidar_dt.at(ikd->id)->value()(0);
    Matrix3d RItoL = sys->state->lidar_extrinsic.at(ikd->id)->Rot();
    Vector3d pIinL = sys->state->lidar_extrinsic.at(ikd->id)->pos();

    // Get IMU pose at map time
    Matrix3d RGtoI;
    Vector3d pIinG;
    if (!sys->state->get_interpolated_pose(ikd->time + dt, RGtoI, pIinG))
      continue;

    // map pose
    Matrix3d RGtoM = RItoL * RGtoI;
    Vector4d qGtoM = rot_2_quat(RGtoM);
    Vector3d pMinG = pIinG + RGtoI.transpose() * (-RItoL.transpose() * pIinL);

    // Publish tf of map
    tf::StampedTransform trans;
    trans.stamp_ = ros::Time::now();
    tf::Quaternion quat(qGtoM(0), qGtoM(1), qGtoM(2), qGtoM(3));
    trans.setRotation(quat);
    tf::Vector3 orig(pMinG(0), pMinG(1), pMinG(2));
    trans.setOrigin(orig);
    trans.frame_id_ = "global";
    trans.child_frame_id_ = "map" + to_string(ikd->id);
    mTfBr->sendTransform(trans);

    if (pub_lidar_map.at(ikd->id).getNumSubscribers() == 0)
      continue;

    // Publish pointcloud in the global frame
    // This is slower because it requires pointcloud transform
    POINTCLOUD_XYZI_PTR map_inL(new pcl::PointCloud<pcl::PointXYZI>);
    ikd->tree->flatten(ikd->tree->Root_Node, map_inL->points, NOT_RECORD);
    pair<Matrix3d, Vector3d> pose_LinG = sys->up_ldr->get_pose_LinG(ikd->id, ikd->time);
    Matrix4d tr = Matrix4d::Identity();
    tr.block(0, 0, 3, 3) = pose_LinG.first.transpose();
    tr.block(0, 3, 3, 1) = pose_LinG.second;
    POINTCLOUD_XYZI_PTR map_inG(new pcl::PointCloud<pcl::PointXYZI>);
    map_inL->height = map_inL->points.size();
    map_inL->width = 1;
    pcl::transformPointCloud(*map_inL, *map_inG, tr);
    sensor_msgs::PointCloud2 map_pointcloud;
    pcl::toROSMsg(*map_inG, map_pointcloud);
    map_pointcloud.header.frame_id = "global";
    map_pointcloud.header.stamp = ros::Time::now();
    pub_lidar_map.at(ikd->id).publish(map_pointcloud);
  }
}
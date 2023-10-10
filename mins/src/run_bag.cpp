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

#include "core/ROSHelper.h"
#include "core/ROSPublisher.h"
#include "core/SystemManager.h"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsIMU.h"
#include "options/OptionsLidar.h"
#include "options/OptionsSystem.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "sim/SimVisualizer.h"
#include "update/gps/GPSTypes.h"
#include "update/vicon/ViconTypes.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include "utils/State_Logger.h"
#include "utils/TimeChecker.h"
#include "utils/dataset_reader.h"
#include "utils/opencv_yaml_parse.h"
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>

using namespace std;
using namespace mins;

shared_ptr<SystemManager> sys;
shared_ptr<ROSPublisher> pub;
shared_ptr<SimVisualizer> sim_viz;
shared_ptr<Options> op;
shared_ptr<State_Logger> save;

rosbag::View view;
rosbag::Bag bag;
vector<rosbag::MessageInstance> msgs;
ros::Time time_init, time_finish;
vector<map<double, int>> cam_map; // {cam[i], {img_time, idx in vec}}
vector<int> used_index;

// read parameters, init system, build map('cam_map') for cam msg ptr
void system_setup(int argc, char **argv);
// find the closest stereo pair's image msg ptr based on map('cam_map')
bool find_stereo_pair(double meas_t, int cam_id, int &idx);
// feed the camera measurement to the system. automatically find stereo pair measurement if it is
bool feed_camera(int cam_id, int idx);

// Main function
int main(int argc, char **argv) {

  // Load parameters
  system_setup(argc, argv);

  // process measurements
  for (int i = 0; i < (int)msgs.size(); i++) {
    if (!ros::ok() || msgs.at(i).getTime() > time_finish)
      break;

    // skip until start time
    if (msgs.at(i).getTime() < time_init)
      continue;

    // skip used measurements. Usually stereo img
    if (find(used_index.begin(), used_index.end(), i) != used_index.end()) {
      used_index.erase(std::remove(used_index.begin(), used_index.end(), i), used_index.end());
      continue;
    }

    // ===================== IMU =====================
    if (msgs.at(i).getTopic() == op->est->imu->topic) {
      ov_core::ImuData imu = ROSHelper::Imu2Data(msgs.at(i).instantiate<sensor_msgs::Imu>());
      PRINT1(GREEN "[BAG] IMU measurement: %.3f" RESET, imu.timestamp);
      PRINT1(GREEN "|%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, imu.wm(0), imu.wm(1), imu.wm(2), imu.am(0), imu.am(1), imu.am(2));
      bool visualize = sys->feed_measurement_imu(imu);
      pub->publish_imu();
      if (visualize) {
        pub->visualize();
        sim_viz->publish_groundtruth();
        op->sys->save_trajectory ? save->save_trajectory_to_file(sys) : void();
      }
      continue;
    }

    // ===================== CAM =====================
    if (op->est->cam->enabled) {
      for (int cam_id = 0; cam_id < op->est->cam->max_n; cam_id++) {
        if (msgs.at(i).getTopic() == op->est->cam->topic.at(cam_id)) {
          if (feed_camera(cam_id, i)) // this also adds index to [used_index] if stereo
            pub->publish_cam_images({cam_id, i});
          continue;
        }
      }
    }

    // ===================== WHEEL =====================
    if (op->est->wheel->enabled && msgs.at(i).getTopic() == op->est->wheel->topic) {
      auto wheel_j = msgs.at(i).instantiate<sensor_msgs::JointState>();
      auto wheel_o = msgs.at(i).instantiate<nav_msgs::Odometry>();

      WheelData data;
      if (wheel_j != nullptr) {
        data = ROSHelper::JointState2Data(wheel_j);
      } else {
        data = ROSHelper::Odometry2Data(wheel_o);
      }

      PRINT1(MAGENTA "[BAG] WHL measurement: %.3f|%.3f,%.3f\n" RESET, data.time, data.m1, data.m2);
      sys->feed_measurement_wheel(data);
      continue;
    }

    // ===================== GPS =====================
    if (op->est->gps->enabled) {
      for (int gps_id = 0; gps_id < op->est->gps->max_n; gps_id++) {
        if (msgs.at(i).getTopic() == op->est->gps->topic.at(gps_id)) {
          auto ptr_fix = msgs.at(i).instantiate<sensor_msgs::NavSatFix>();
          auto ptr_geo = msgs.at(i).instantiate<geometry_msgs::PoseStamped>();
          GPSData data = (ptr_fix != nullptr ? ROSHelper::NavSatFix2Data(ptr_fix, gps_id) : ROSHelper::PoseStamped2Data(ptr_geo, gps_id, op->est->gps->noise));
          // In case GNSS message does not have GNSS noise value or we want to overwrite it, use preset values
          data.noise(0) <= 0.0 || op->est->gps->overwrite_noise ? data.noise(0) = op->est->gps->noise : double();
          data.noise(1) <= 0.0 || op->est->gps->overwrite_noise ? data.noise(1) = op->est->gps->noise : double();
          data.noise(2) <= 0.0 || op->est->gps->overwrite_noise ? data.noise(2) = op->est->gps->noise * 2 : double();
          PRINT1(CYAN "[BAG] GPS measurement: %.3f|%d|" RESET, data.time, data.id);
          PRINT1(CYAN "%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n" RESET, data.meas(0), data.meas(1), data.meas(2), data.noise(0), data.noise(1), data.noise(2));
          sys->feed_measurement_gps(data, ptr_fix != nullptr);
          pub->publish_gps(data, ptr_fix != nullptr);
          continue;
        }
      }
    }

    // ==================== LiDAR ====================
    if (op->est->lidar->enabled) {
      for (int lidar_id = 0; lidar_id < op->est->lidar->max_n; lidar_id++) {
        if (msgs.at(i).getTopic() == op->est->lidar->topic.at(lidar_id)) {
          std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> data = ROSHelper::rosPC2pclPC(msgs.at(i).instantiate<sensor_msgs::PointCloud2>(), lidar_id);
          PRINT1("[BAG] LDR measurement: %.3f|%d|%d\n", (double)data->header.stamp / 1000, lidar_id, data->points.size());
          sys->feed_measurement_lidar(data);
          pub->publish_lidar_cloud(data);
          continue;
        }
      }
    }

    // ==================== VICON ====================
    if (op->est->vicon->enabled) {
      for (int vicon_id = 0; vicon_id < op->est->vicon->max_n; vicon_id++) {
        if (msgs.at(i).getTopic() == op->est->vicon->topic.at(vicon_id)) {
          ViconData data = ROSHelper::PoseStamped2Data(msgs.at(i).instantiate<geometry_msgs::PoseStamped>(), vicon_id);
          PRINT1("[BAG] VCN measurement: %.3f|%d|", data.time, data.id);
          PRINT1("%.3f,%.3f,%.3f|%.3f,%.3f,%.3f\n", data.pose(0), data.pose(1), data.pose(2), data.pose(3), data.pose(4), data.pose(5));
          sys->feed_measurement_vicon(data);
          pub->publish_vicon(data);
          continue;
        }
      }
    }
  }

  // Final visualization
  sys->visualize_final();
  op->sys->save_timing ? save->save_timing_to_file(sys->tc_sensors->get_total_sum()) : void();
  save->check_files();

  ros::shutdown();

  // Done!
  return EXIT_SUCCESS;
}

bool feed_camera(int cam_id, int idx) {
  sensor_msgs::CompressedImage::Ptr img_c_ptr = msgs.at(idx).instantiate<sensor_msgs::CompressedImage>();
  sensor_msgs::Image::Ptr img_i_ptr = msgs.at(idx).instantiate<sensor_msgs::Image>();
  // In case the image does not have timestamp (then 0), overwrite it with message time.
  img_c_ptr != nullptr && img_c_ptr->header.stamp == ros::Time(0) ? img_c_ptr->header.stamp = msgs.at(idx).getTime() : ros::Time();
  img_i_ptr != nullptr && img_i_ptr->header.stamp == ros::Time(0) ? img_i_ptr->header.stamp = msgs.at(idx).getTime() : ros::Time();
  sensor_msgs::CompressedImage::ConstPtr img_c = img_c_ptr;
  sensor_msgs::Image::ConstPtr img_i = img_i_ptr;
  ov_core::CameraData cam;

  // MONO
  if (op->est->cam->stereo_pairs.find(cam_id) == op->est->cam->stereo_pairs.end()) {
    if (img_c != nullptr) {
      if (ROSHelper::Image2Data(img_c, cam_id, cam, op->est->cam)) {
        PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d\n" RESET, img_c->header.stamp.toSec(), cam_id);
        sys->feed_measurement_camera(cam);
        pub->publish_cam_images(cam_id);
      } else {
        PRINT3(YELLOW "Failed to convert image to a proper format.\n" RESET);
      }
    } else {
      if (ROSHelper::Image2Data(img_i, cam_id, cam, op->est->cam)) {
        PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d\n" RESET, img_i->header.stamp.toSec(), cam_id);
        sys->feed_measurement_camera(cam);
        pub->publish_cam_images(cam_id);
      } else {
        PRINT3(YELLOW "Failed to convert image to a proper format.\n" RESET);
      }
    }
    return true;
  }

  // STEREO - find stereo measurement
  int msgs_id; // this is index position in msgs vector
  int stereo_id = op->est->cam->stereo_pairs.at(cam_id);
  double meas_t = img_c != nullptr ? img_c->header.stamp.toSec() : img_i->header.stamp.toSec();
  if (find_stereo_pair(meas_t, cam_id, msgs_id)) {

    // should be always in the future
    if (msgs_id < idx)
      return false;

    // record this index
    used_index.push_back(msgs_id);

    // get stereo pair image
    if (img_c != nullptr) {
      bool success0 = ROSHelper::Image2Data(img_c, cam_id, cam, op->est->cam);
      bool success1 = ROSHelper::Image2Data(msgs.at(msgs_id).instantiate<sensor_msgs::CompressedImage>(), stereo_id, cam, op->est->cam);
      if (success0 && success1) {
        PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d|%d\n" RESET, img_c->header.stamp.toSec(), cam_id, stereo_id);
        sys->feed_measurement_camera(cam);
        pub->publish_cam_images({cam_id, stereo_id});
      }
    } else {
      bool success0 = ROSHelper::Image2Data(img_i, cam_id, cam, op->est->cam);
      bool success1 = ROSHelper::Image2Data(msgs.at(msgs_id).instantiate<sensor_msgs::Image>(), stereo_id, cam, op->est->cam);
      if (success0 && success1) {
        PRINT1(BLUE "[BAG] CAM measurement: %.3f|%d|%d\n" RESET, img_i->header.stamp.toSec(), cam_id, stereo_id);
        sys->feed_measurement_camera(cam);
        pub->publish_cam_images({cam_id, stereo_id});
      }
    }
  } else {
    PRINT4(RED "Cannot find proper stereo pair of CAM%d!\n" RESET, cam_id);
    return false;
  }
  return true;
}

bool find_stereo_pair(double meas_t, int cam_id, int &idx) {
  // Get the stereo pair's cam id
  int stereo_id = op->est->cam->stereo_pairs.at(cam_id);

  // get lower bound of stereo pair's message
  assert(!cam_map.at(stereo_id).empty());
  auto pair_lb = cam_map.at(stereo_id).lower_bound(meas_t);

  // check one previous message
  if (distance(cam_map.at(stereo_id).begin(), pair_lb) > 0) {
    // get lb and its prev measurement info
    double t_1 = pair_lb->first;
    int idx_1 = pair_lb->second;
    pair_lb--;
    double t_0 = pair_lb->first;
    int idx_0 = pair_lb->second;

    // get the closest index
    idx = abs(meas_t - t_0) < abs(meas_t - t_1) ? idx_0 : idx_1;
  } else {
    // this is the closest measurement you can get
    idx = pair_lb->second;
  }

  // filter too large time gap
  sensor_msgs::CompressedImage::ConstPtr stereo_img_c = msgs.at(idx).instantiate<sensor_msgs::CompressedImage>();
  sensor_msgs::Image::ConstPtr stereo_img_i = msgs.at(idx).instantiate<sensor_msgs::Image>();
  if (stereo_img_c != nullptr)
    return abs(stereo_img_c->header.stamp.toSec() - meas_t) < 0.01;
  else if (stereo_img_i != nullptr)
    return abs(stereo_img_i->header.stamp.toSec() - meas_t) < 0.01;
  else {
    PRINT4(RED "Image topic has unmatched message types!. Exiting.\n" RESET);
    exit(EXIT_FAILURE);
  }
}

void system_setup(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  string config_path = "unset_path_to_config.yaml";
  argc > 1 ? config_path = argv[1] : string();

  // Launch our ros node
  ros::init(argc, argv, "mins_bag");
  auto nh = make_shared<ros::NodeHandle>("~");
  nh->param<string>("config_path", config_path, config_path);

  // Init options
  auto parser = make_shared<ov_core::YamlParser>(config_path);
  parser->set_node_handler(nh);
  op = make_shared<Options>();
  op->load_print(parser);
  op->sys->save_prints ? mins::Print_Logger::open_file(op->sys->path_state, true) : void();

  sys = make_shared<SystemManager>(op->est);
  pub = make_shared<ROSPublisher>(nh, sys, op);
  sim_viz = make_shared<SimVisualizer>(nh, sys);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT4(RED "unable to parse all parameters, please fix\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Load rosbag here, and find messages we can play
  PRINT2("[BAG] Reading the bag file    ");
  bag.open(op->sys->path_bag, rosbag::bagmode::Read);
  view.addQuery(bag);

  // Check to make sure we have data to play
  if (view.size() == 0) {
    PRINT4(RED "\nNo messages to play on specified topics. Exiting.\n" RESET);
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  // load rosbag msg itr
  msgs.reserve(view.size());
  cam_map = vector<map<double, int>>(op->est->cam->max_n);
  int cnt = 0;
  for (const rosbag::MessageInstance &msg : view) {
    if (!ros::ok())
      break;
    PRINT2("\b\b\b%02d%%", (int)((cnt++ * 100) / view.size()));

    // check IMU ========================================
    if (msg.getTopic() == op->est->imu->topic) {
      assert(msg.instantiate<sensor_msgs::Imu>() != nullptr);
      msgs.push_back(msg);
      continue;
    }

    // check CAM ========================================
    if (op->est->cam->enabled) {
      bool keep_msg = false;
      for (int id = 0; id < op->est->cam->max_n; id++) {
        if (msg.getTopic() == op->est->cam->topic.at(id)) {
          sensor_msgs::CompressedImage::ConstPtr img_c = msg.instantiate<sensor_msgs::CompressedImage>();
          sensor_msgs::Image::ConstPtr img_i = msg.instantiate<sensor_msgs::Image>();
          if (img_c != nullptr)
            cam_map.at(id).insert({img_c->header.stamp.toSec(), msgs.size()});
          else if (img_i != nullptr)
            cam_map.at(id).insert({img_i->header.stamp.toSec(), msgs.size()});
          else {
            PRINT4(RED "\nImage topic has unmatched message types!. Exiting.\n" RESET);
            exit(EXIT_FAILURE);
          }
          keep_msg = true;
          break;
        }
      }
      if (keep_msg) {
        msgs.push_back(msg);
        continue;
      }
    }

    // check WHEEL ========================================
    if (op->est->wheel->enabled && msg.getTopic() == op->est->wheel->topic) {
      auto wheel_j = msg.instantiate<sensor_msgs::JointState>();
      auto wheel_o = msg.instantiate<nav_msgs::Odometry>();
      assert(wheel_j != nullptr || wheel_o != nullptr);
      if (wheel_j != nullptr && op->est->wheel->sub_topics.at(0) == wheel_j->name.at(0))
        msgs.push_back(msg);
      else if (wheel_o != nullptr)
        msgs.push_back(msg);
      else {
        PRINT4(RED "Wheel type setting is wrong!\n" RESET);
        std::exit(EXIT_FAILURE);
      }
      continue;
    }

    // check LiDAR ========================================
    if (op->est->lidar->enabled) {
      bool keep_msg = false;
      for (int id = 0; id < op->est->lidar->max_n; id++) {
        if (msg.getTopic() == op->est->lidar->topic.at(id)) {
          assert(msg.instantiate<sensor_msgs::PointCloud2>() != nullptr);
          keep_msg = true;
          break;
        }
      }
      if (keep_msg) {
        msgs.push_back(msg);
        continue;
      }
    }

    // check GPS ========================================
    if (op->est->gps->enabled) {
      bool keep_msg = false;
      for (int id = 0; id < op->est->gps->max_n; id++) {
        if (msg.getTopic() == op->est->gps->topic.at(id)) {
          auto ptr_fix = msg.instantiate<sensor_msgs::NavSatFix>();
          auto ptr_geo = msg.instantiate<geometry_msgs::PoseStamped>();
          assert(ptr_fix != nullptr || ptr_geo != nullptr);
          keep_msg = true;
          break;
        }
      }
      if (keep_msg) {
        msgs.push_back(msg);
        continue;
      }
    }

    // check VICON ========================================
    if (op->est->vicon->enabled) {
      bool keep_msg = false;
      for (int id = 0; id < op->est->vicon->max_n; id++) {
        if (msg.getTopic() == op->est->vicon->topic.at(id)) {
          assert(msg.instantiate<geometry_msgs::PoseStamped>() != nullptr);
          keep_msg = true;
          break;
        }
      }
      if (keep_msg) {
        msgs.push_back(msg);
        continue;
      }
    }
  }
  PRINT2("\n");

  // bag run times
  time_init = view.getBeginTime() + ros::Duration(op->sys->bag_start);
  op->sys->bag_durr = view.getEndTime().toSec() < op->sys->bag_durr ? view.getEndTime().toSec() : op->sys->bag_durr;
  time_finish = (op->sys->bag_durr < 0) ? view.getEndTime() : time_init + ros::Duration(op->sys->bag_durr);

  // Create state log files
  save = make_shared<State_Logger>(op);
}

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

#include "functions/ErrorPlot.h"

#if ROS_AVAILABLE == 2
#include "rclcpp/rclcpp.hpp"
#else
#include "ros/ros.h"
#endif
#include "utils/colors.h"
#include "utils/print.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif

int main(int argc, char **argv) {
  // Path to save the plots
  std::string load_path = argv[1];
  std::string save_path = load_path;

  // visualize the plots?
  bool visualize = true;
  if (argc > 3) {
    std::string vis = argv[3];
    if (vis == "false" || vis == "False" || vis == "FALSE")
      visualize = false;
  }

  // overwrite options if ROS params exist
#if ROS_AVAILABLE == 2
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  auto node = std::make_shared<rclcpp::Node>("plot_consistency", options);
  node->get_parameter<std::string>("/plot_consistency/load_path", load_path);
  node->get_parameter<std::string>("/plot_consistency/save_path", save_path);
  node->get_parameter<bool>("/plot_consistency/visualize", visualize);
#else
  ros::init(argc, argv, "plot_consistency");
  ros::NodeHandle nh;
  ros::param::get("/plot_consistency/load_path", load_path);
  ros::param::get("/plot_consistency/save_path", save_path);
  ros::param::get("/plot_consistency/visualize", visualize);
#endif
  // Create save directory
  save_path += "/Plot/";
  if (argc > 2)
    save_path = argv[2];
  save_path += save_path.back() != '/' ? "/" : "";
  boost::filesystem::create_directories(save_path.c_str());

  // Create our trajectory object
  mins_eval::ErrorPlot traj(load_path, save_path);

  // Plot the state errors
  PRINT_INFO("Plotting IMU...\n");
  traj.plot_imu(traj.imu_data);

  // VICON
  if (!traj.vicon_dt_data.empty()) {
    PRINT_INFO("Plotting VICON timeoffset...\n");
    for (int i = 0; i < (int)traj.vicon_dt_data.size(); i++) {
      vector<string> names = {"t-error (s)"};
      traj.plot_vector(traj.vicon_dt_data[i], "VICON" + to_string(i) + " Time Offset", names);
    }
  }
  if (!traj.vicon_ext_data.empty()) {
    PRINT_INFO("Plotting VICON extrinsics...\n");
    for (int i = 0; i < (int)traj.vicon_ext_data.size(); i++) {
      traj.plot_extrinsic(traj.vicon_ext_data[i], "VICON" + to_string(i));
    }
  }

  // CAM
  if (!traj.cam_dt_data.empty()) {
    PRINT_INFO("Plotting CAM timeoffset...\n");
    for (int i = 0; i < (int)traj.cam_dt_data.size(); i++) {
      vector<string> names = {"t-error (s)"};
      traj.plot_vector(traj.cam_dt_data[i], "CAM" + to_string(i) + " Time Offset", names);
    }
  }
  if (!traj.cam_ext_data.empty()) {
    PRINT_INFO("Plotting CAM extrinsics...\n");
    for (int i = 0; i < (int)traj.cam_ext_data.size(); i++) {
      traj.plot_extrinsic(traj.cam_ext_data[i], "CAM" + to_string(i));
    }
  }
  if (!traj.cam_int_data.empty()) {
    PRINT_INFO("Plotting CAM intrinsics...\n");
    for (int i = 0; i < (int)traj.cam_int_data.size(); i++) {
      vector<string> names = {"k1", "k2", "k3", "k4", "d1", "d2", "d3", "d4"};
      traj.plot_vector(traj.cam_int_data[i], "CAM" + to_string(i) + " Intrinsic", names);
    }
  }

  // GPS
  if (!traj.gps_dt_data.empty()) {
    PRINT_INFO("Plotting GPS timeoffset...\n");
    for (int i = 0; i < (int)traj.gps_dt_data.size(); i++) {
      vector<string> names = {"t-error (s)"};
      traj.plot_vector(traj.gps_dt_data[i], "GPS" + to_string(i) + " Time Offset", names);
    }
  }
  if (!traj.gps_ext_data.empty()) {
    PRINT_INFO("Plotting GPS extrinsics...\n");
    for (int i = 0; i < (int)traj.gps_ext_data.size(); i++) {
      vector<string> names = {"x-error (m)", "y-error (m)", "z-error (m)"};
      traj.plot_vector(traj.gps_ext_data[i], "GPS" + to_string(i) + " Extrinsic Position", names);
    }
  }

  // WHEEL
  if (!traj.wheel_dt_data.ests.empty()) {
    PRINT_INFO("Plotting WHEEL timeoffset...\n");
    vector<string> names = {"t-error (s)"};
    traj.plot_vector(traj.wheel_dt_data, "WHEEL Time Offset", names);
  }
  if (!traj.wheel_ext_data.ests.empty()) {
    PRINT_INFO("Plotting WHEEL extrinsics...\n");
    traj.plot_extrinsic(traj.wheel_ext_data, "WHEEL");
  }
  if (!traj.wheel_int_data.ests.empty()) {
    PRINT_INFO("Plotting WHEEL intrinsics...\n");
    vector<string> names = {"rl-error (m)", "rr-error (m)", "b-error (m)"};
    traj.plot_vector(traj.wheel_int_data, "WHEEL Intrinsic", names);
  }

  // LiDAR
  if (!traj.lidar_dt_data.empty()) {
    PRINT_INFO("Plotting LiDAR timeoffset...\n");
    for (int i = 0; i < (int)traj.lidar_dt_data.size(); i++) {
      vector<string> names = {"t-error (s)"};
      traj.plot_vector(traj.lidar_dt_data[i], "LiDAR" + to_string(i) + " Time Offset", names);
    }
  }
  if (!traj.lidar_ext_data.empty()) {
    PRINT_INFO("Plotting LiDAR extrinsics...\n");
    for (int i = 0; i < (int)traj.lidar_ext_data.size(); i++) {
      traj.plot_extrinsic(traj.lidar_ext_data[i], "LiDAR" + to_string(i));
    }
  }

#ifdef HAVE_PYTHONLIBS
  if (visualize)
    matplotlibcpp::show(true);
#endif

  // Done!
  return EXIT_SUCCESS;
}

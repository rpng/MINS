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

#include <memory>

#include "core/ROSPublisher.h"
#include "core/SystemManager.h"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsIMU.h"
#include "options/OptionsSystem.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "sim/SimVisualizer.h"
#include "sim/Simulator.h"
#include "update/cam/CamTypes.h"
#include "update/gps/GPSTypes.h"
#include "update/lidar/UpdaterLidar.h"
#include "update/vicon/ViconTypes.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include "utils/State_Logger.h"
#include "utils/TimeChecker.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/sensor_data.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

using namespace mins;
using namespace std;
using namespace Eigen;

shared_ptr<Simulator> sim;
shared_ptr<SystemManager> sys;
shared_ptr<ROSPublisher> pub;
shared_ptr<SimVisualizer> sim_viz;
shared_ptr<Options> op;
shared_ptr<State_Logger> save;

void system_setup(int argc, char **argv);

// Main function
int main(int argc, char **argv) {
  // Load parameters
  system_setup(argc, argv);
  // run simulation
  while (sim->ok() && ros::ok()) {
    // imu: get the next simulated imu measurement if we have it
    ov_core::ImuData imu;
    if (sim->get_next_imu(imu)) {
      bool visualize = sys->feed_measurement_imu(imu);
      pub->publish_imu();
      if (visualize) {
        // If GPS is initialized, make sure visualization is in ENU
        sim->trans_gt_to_ENU = op->est->gps->enabled && sys->up_gps->initialized;
        pub->visualize();
        sim_viz->publish_groundtruth();
        op->sys->save_state ? save->save_state_to_file(sys, sim) : void();
        op->sys->save_trajectory ? save->save_trajectory_to_file(sys) : void();
      }
    }

    // cam: get the next simulated camera uv measurements if we have them
    CamSimData cam;
    if (sim->get_next_cam(cam)) {
      sys->feed_measurement_camsim(cam);
      pub->publish_cam_images(cam.ids);
      sim_viz->publish_sim_cam_features();
    }

    GPSData gps;
    if (sim->get_next_gps(gps)) {
      sys->feed_measurement_gps(gps, false);
      pub->publish_gps(gps, false);
    }

    WheelData wheel;
    if (sim->get_next_wheel(wheel)) {
      sys->feed_measurement_wheel(wheel);
    }

    // LIDAR: get the next simulated lidar range measurements
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar(new pcl::PointCloud<pcl::PointXYZ>);
    if (sim->get_next_lidar(lidar)) {
      sys->feed_measurement_lidar(lidar);
      pub->publish_lidar_cloud(lidar);
      sim_viz->publish_lidar_structure();
    }

    // vicon: get the next simulated vicon measurement if we have it
    ViconData vicon;
    if (sim->get_next_vicon(vicon)) {
      sys->feed_measurement_vicon(vicon);
      pub->publish_vicon(vicon);
    }
  }

  // Final visualization
  sys->visualize_final();
  sim_viz->visualize_final();
  op->sys->save_timing ? save->save_timing_to_file(sys->tc_sensors->get_total_sum()) : void();
  save->check_files();

  ros::shutdown();
  return EXIT_SUCCESS;
}

void system_setup(int argc, char **argv) {
  // Ensure we have a path, if the user passes it then we should use it
  string config_path = "unset_path_to_config.yaml";
  argc > 1 ? config_path = argv[1] : string();

  // Launch our ros node
  ros::init(argc, argv, "mins_simulation");
  auto nh = make_shared<ros::NodeHandle>("~");
  nh->param<string>("config_path", config_path, config_path);

  // Load the config
  auto parser = make_shared<ov_core::YamlParser>(config_path);
  parser->set_node_handler(nh);
  op = make_shared<Options>();
  op->load_print(parser);
  op->sys->save_prints ? mins::Print_Logger::open_file(op->sys->path_state, true) : void();

  // Create our system
  sim = make_shared<Simulator>(op);
  sys = make_shared<SystemManager>(op->est, sim);
  pub = make_shared<ROSPublisher>(nh, sys, op);
  sim_viz = make_shared<SimVisualizer>(nh, sys, sim);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT4(RED "unable to parse all parameters, please fix\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Create state log files
  save = make_shared<State_Logger>(op, sim);
}

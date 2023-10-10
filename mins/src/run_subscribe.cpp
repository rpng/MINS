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
#include "core/ROSSubscriber.h"
#include "core/SystemManager.h"
#include "options/Options.h"
#include "options/OptionsSystem.h"
#include "utils/Print_Logger.h"
#include "utils/State_Logger.h"
#include "utils/TimeChecker.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include <ros/ros.h>

using namespace std;
using namespace mins;

// Main function
int main(int argc, char **argv) {

  // Ensure we have a path, if the user passes it then we should use it
  string config_path = "unset_path_to_config.yaml";
  argc > 1 ? config_path = argv[1] : string();

  // Launch our ros node
  ros::init(argc, argv, "mins_subscribe");
  auto nh = make_shared<ros::NodeHandle>("~");
  nh->param<string>("config_path", config_path, config_path);

  // Load the config
  auto parser = make_shared<ov_core::YamlParser>(config_path);
  parser->set_node_handler(nh);
  shared_ptr<Options> op = make_shared<Options>();
  op->load_print(parser);
  shared_ptr<State_Logger> save = make_shared<State_Logger>(op);

  // Create our system
  shared_ptr<SystemManager> sys = make_shared<SystemManager>(op->est);
  shared_ptr<ROSPublisher> pub = make_shared<ROSPublisher>(nh, sys, op);
  shared_ptr<ROSSubscriber> sub = make_shared<ROSSubscriber>(nh, sys, pub);

  // Ensure we read in all parameters required
  if (!parser->successful()) {
    PRINT4(RED "unable to parse all parameters, please fix\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Spin off to ROS
  ros::spin();

  // Final visualization
  sys->visualize_final();
  op->sys->save_timing ? save->save_timing_to_file(sys->tc_sensors->get_total_sum()) : void();
  save->check_files();

  ros::shutdown();

  // Done!
  return EXIT_SUCCESS;
}

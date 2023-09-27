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

#include "OptionsInit.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

void mins::OptionsInit::load(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_init";
    parser->parse_external(f, "init", "window_time", window_time);
    parser->parse_external(f, "init", "imu_thresh", imu_thresh);
    parser->parse_external(f, "init", "imu_wheel_thresh", imu_wheel_thresh, false);
    parser->parse_external(f, "init", "imu_only_init", imu_only_init);
    parser->parse_external(f, "init", "imu_gravity_aligned", imu_gravity_aligned, false);
    parser->parse_external(f, "init", "use_gt", use_gt);
    parser->parse_external(f, "init", "use_gt_gnss", use_gt_gnss, false);
    parser->parse_external(f, "init", "use_gt_lidar", use_gt_lidar, false);
    parser->parse_external(f, "init", "cov_size", cov_size);
  }
}
void mins::OptionsInit::print() {
  PRINT1(BOLDBLUE "Options - Initialization\n" RESET);
  PRINT1("\t- window_time: %.2f\n", window_time);
  PRINT1("\t- imu_thresh: %.2f\n", imu_thresh);
  PRINT1("\t- imu_wheel_thresh: %.2f\n", imu_wheel_thresh);
  PRINT1("\t- imu_only_init: %s\n", imu_only_init ? "true" : "false");
  PRINT1("\t- imu_gravity_aligned: %s\n", imu_gravity_aligned ? "true" : "false");
  PRINT1("\t- use_gt: %s\n", use_gt ? "true" : "false");
  PRINT1("\t- use_gt_gnss: %s\n", use_gt_gnss ? "true" : "false");
  PRINT1("\t- use_gt_lidar: %s\n", use_gt_lidar ? "true" : "false");
  PRINT1("\t- cov_size: %.4f\n", cov_size);
}

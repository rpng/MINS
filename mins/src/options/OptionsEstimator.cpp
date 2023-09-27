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

#include "OptionsEstimator.h"
#include "OptionsCamera.h"
#include "OptionsGPS.h"
#include "OptionsIMU.h"
#include "OptionsInit.h"
#include "OptionsLidar.h"
#include "OptionsVicon.h"
#include "OptionsWheel.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

void mins::OptionsEstimator::load(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_estimator";
    parser->parse_external(f, "est", "gravity_mag", gravity(2));
    parser->parse_external(f, "est", "use_imu_res", use_imu_res);
    parser->parse_external(f, "est", "use_imu_cov", use_imu_cov);
    parser->parse_external(f, "est", "use_pol_cov", use_pol_cov);
    parser->parse_external(f, "est", "window_size", window_size);
    parser->parse_external(f, "est", "clone_freq", clone_freq);
    parser->parse_external(f, "est", "dt_extrapolation", dt_exp);
    parser->parse_external(f, "est", "intr_order", intr_order);
    parser->parse_external(f, "est", "dynamic_cloning", dynamic_cloning);
    parser->parse_external(f, "est", "intr_error_mlt", intr_err.mlt);
    for (int i = 1; i < 40; i++) {
      parser->parse_external(f, "intr_ori", "Hz_" + std::to_string(i), intr_err.reading_ori_slope, false);
      parser->parse_external(f, "intr_pos", "Hz_" + std::to_string(i), intr_err.reading_pos_slope, false);
      intr_err.set_values(i);
    }
    parser->parse_external(f, "est", "intr_error_ori_thr", intr_err.threshold_ori);
    parser->parse_external(f, "est", "intr_error_pos_thr", intr_err.threshold_pos);
    double thr_mlt = 1.0;
    parser->parse_external(f, "est", "intr_error_thr_mlt", thr_mlt);
    intr_err.threshold_ori *= thr_mlt;
    intr_err.threshold_pos *= thr_mlt;
    // Must have at least one order
    if (intr_order < 1 || intr_order % 2 == 0) {
      PRINT4(RED "Estimator polynomial order should be >= 1 and odd number. Current value: %d\n" RESET, intr_order);
      std::exit(EXIT_FAILURE);
    }

    // Check if max clone works with interpolation order
    double max_clone_size = window_size * clone_freq + 1;
    if (max_clone_size < intr_order + 1) {
      PRINT4(RED "Max clone size is smaller than required for polynomial interpolation (%d < %f + 1).\n" RESET, max_clone_size, intr_order);
      PRINT4(RED "Set Max clone size = %d.\n" RESET, intr_order + 1);
      std::exit(EXIT_FAILURE);
    }
    // Set max_clone_size >= 6 (5 + 1) so that clones can cover the largest interpolation order
    if (dynamic_cloning && max_clone_size < 6) {
      PRINT4(RED "Max clone size is smaller than required for polynomial interpolation (%d < 6).\n" RESET, max_clone_size);
      PRINT4(RED "Set Max clone size = 6.\n" RESET);
      std::exit(EXIT_FAILURE);
    }

    if (use_imu_cov + use_pol_cov > 1) {
      PRINT4(RED "More than 1 cov method enabled.\n" RESET);
      PRINT4(RED "use_imu_cov: %s\n" RESET, use_imu_cov ? "true" : "false");
      PRINT4(RED "use_pol_cov: %s\n" RESET, use_pol_cov ? "true" : "false");
      std::exit(EXIT_FAILURE);
    }
  }
  init = make_shared<OptionsInit>();
  lidar = make_shared<OptionsLidar>();
  wheel = make_shared<OptionsWheel>();
  gps = make_shared<OptionsGPS>();
  vicon = make_shared<OptionsVicon>();
  cam = make_shared<OptionsCamera>();
  imu = make_shared<OptionsIMU>();
  init->load(parser);
  imu->load(parser);
  cam->load(parser);
  gps->load(parser);
  wheel->load(parser);
  lidar->load(parser);
  vicon->load(parser);
}

void mins::OptionsEstimator::print() {
  PRINT1(BOLDBLUE "Options - Estimator\n" RESET);
  PRINT1("\t- gravity: %.3f, %.3f, %.3f\n", 0.0, 0.0, gravity(2));
  PRINT1("\t- window_size: %f\n", window_size);
  PRINT1("\t- clone_freq: %d\n", clone_freq);
  PRINT1("\t- dt_extrapolation: %.2f\n", dt_exp);
  PRINT1("\t- intr_order: %d\n", intr_order);
  PRINT1("\t- dynamic_cloning: %s\n", dynamic_cloning ? "true" : "false");
  PRINT1("\t- use_imu_res: %s\n", use_imu_res ? "true" : "false");
  PRINT1("\t- use_imu_cov: %s\n", use_imu_cov ? "true" : "false");
  PRINT1("\t- use_pol_cov: %s\n", use_pol_cov ? "true" : "false");
  PRINT1("\t- intr_error_mlt: %.3f\n", intr_err.mlt);
  PRINT1("\t- intr_error_threshold_ori: %.6f\n", intr_err.threshold_ori);
  PRINT1("\t- intr_error_threshold_pos: %.6f\n", intr_err.threshold_pos);
  imu->print();
  cam->print();
  vicon->print();
  gps->print();
  wheel->print();
  lidar->print();
  init->print();
}

void mins::OptionsEstimator::load_print(const std::shared_ptr<ov_core::YamlParser> &parser) {
  load(parser);
  print();
}

void mins::OptionsEstimator::interpolation_error::set_values(int hz) {
  assert(reading_ori_slope.size() == 5);
  assert(reading_pos_slope.size() == 5);
  bool valid_slope = true;
  for (int i = 0; i < 5; i++) {
    if (reading_ori_slope.at(i) < 0 || reading_pos_slope.at(i) < 0)
      valid_slope = false;
  }

  // return if the reading is not valid
  if (!valid_slope) {
    // reset the reading value
    for (int i = 0; i < 5; i++) {
      reading_ori_slope.at(i) = -1;
      reading_pos_slope.at(i) = -1;
    }
    return;
  }

  // Insert a new map for this clone Hz
  ori_slope.insert({hz, std::map<int, double>()});
  pos_slope.insert({hz, std::map<int, double>()});

  // Insert order values
  int cnt = 0;
  for (int j = 1; j <= 9; j += 2) {
    ori_slope.at(hz).insert({j, reading_ori_slope[cnt]});
    pos_slope.at(hz).insert({j, reading_pos_slope[cnt]});
    cnt++;
  }

  // reset the reading value
  for (int i = 0; i < 5; i++) {
    reading_ori_slope.at(i) = -1;
    reading_pos_slope.at(i) = -1;
  }
}

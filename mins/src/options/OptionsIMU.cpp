/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "OptionsIMU.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

void mins::OptionsIMU::load(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_imu";
    parser->parse_external(f, "imu", "gyro_noise", sigma_w);
    parser->parse_external(f, "imu", "gyro_bias", sigma_wb);
    parser->parse_external(f, "imu", "accel_noise", sigma_a);
    parser->parse_external(f, "imu", "accel_bias", sigma_ab);
    parser->parse_external(f, "imu", "topic", topic);
  }
}

void mins::OptionsIMU::print() {
  PRINT1(BOLDBLUE "Options - IMU\n" RESET);
  PRINT1("\t- gyro_noise: %.9f\n", sigma_w);
  PRINT1("\t- gyro_bias: %.9f\n", sigma_a);
  PRINT1("\t- accel_noise: %.9f\n", sigma_wb);
  PRINT1("\t- accel_bias: %.9f\n", sigma_ab);
  PRINT1("\t- topic: %s\n", topic.c_str());
}
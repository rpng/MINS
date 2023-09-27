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
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

#ifndef MINS_OPTIONSINIT_H
#define MINS_OPTIONSINIT_H

#include <memory>
#include <string>
namespace ov_core {
class YamlParser;
}
namespace mins {

/**
 * @brief Struct which stores all options needed for state initialization.
 */
struct OptionsInit {

  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  void print();

  /// Amount of time we will initialize over (seconds)
  double window_time = 1.0;

  /// Variance threshold on our acceleration to be classified as moving
  double imu_thresh = 1.0;

  /// Threshold for smoothness check of the initialization
  double imu_wheel_thresh = 0.1;

  /// boolean for state initialization using only IMU
  bool imu_only_init = false;

  /// boolean for state initialization constraining IMU z is aligned with gravity
  bool imu_gravity_aligned = false;

  /// boolean for state initialization using ground truth
  bool use_gt = false;

  /// boolean for ENU-world initialization using simulation
  bool use_gt_gnss = false;

  /// boolean to provide dense initial map to LiDAR using simulation
  bool use_gt_lidar = false;

  /// ground truth file path (It will be provided from system option if we have one)
  std::string path_gt;

  /// Size of initial state covariance
  double cov_size = 1e-4;
};
} // namespace mins

#endif // MINS_OPTIONSINIT_H

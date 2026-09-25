/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_OPTIONSIMU_H
#define MINS_OPTIONSIMU_H

#include <memory>
#include <string>

namespace ov_core {
class YamlParser;
}
namespace mins {

/**
 * @brief Struct which stores all IMU options needed for state estimation.
 */
struct OptionsIMU {

  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  void print();

  /// rostopic to subscribe
  std::string topic;

  /// IMU noise (gyroscope and accelerometer)
  /// Gyroscope white noise (rad/s/sqrt(hz))
  double sigma_w = 1.6968e-04;

  /// Gyroscope random walk (rad/s^2/sqrt(hz))
  double sigma_wb = 1.9393e-05;

  /// Accelerometer white noise (m/s^2/sqrt(hz))
  double sigma_a = 2.0000e-03;

  /// Accelerometer random walk (m/s^3/sqrt(hz))
  double sigma_ab = 3.0000e-03;
};
} // namespace mins
#endif // MINS_OPTIONSIMU_H
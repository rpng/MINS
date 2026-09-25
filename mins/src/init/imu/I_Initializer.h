/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * This code is implemented based on:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_I_INITIALIZER_H
#define MINS_I_INITIALIZER_H

#include "Eigen/Eigen"
#include "memory"

namespace mins {
class Propagator;
struct OptionsEstimator;
class I_Initializer {
public:
  /// IMU only static initializer
  I_Initializer(std::shared_ptr<Propagator> imu_pp, std::shared_ptr<OptionsEstimator> op) : op(op), imu_pp(imu_pp){};

  /// Try initialization
  bool initialization(Eigen::Matrix<double, 17, 1> &imustate);

private:
  /// Options
  std::shared_ptr<OptionsEstimator> op;

  /// IMU sensor data
  std::shared_ptr<Propagator> imu_pp;
};
} // namespace mins

#endif // MINS_I_INITIALIZER_H

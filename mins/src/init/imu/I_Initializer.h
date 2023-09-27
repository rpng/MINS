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

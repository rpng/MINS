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

#ifndef MINS_OPTIONSSIMULATION_H
#define MINS_OPTIONSSIMULATION_H

#include <Eigen/Eigen>
#include <memory>

namespace ov_core {
class YamlParser;
}
namespace mins {

struct OptionsEstimator;
struct OptionsSimulation {
  OptionsSimulation();

  /**
   * @brief This function will load print out all simulated parameters.
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void load_print(const std::shared_ptr<ov_core::YamlParser> &parser);

  /// Measurement noise seed.
  int seed = 0;

  /// If we should perturb the calibration that the estimator starts with
  bool do_perturb = true;

  /// Remove noise
  bool remove_noise = false;

  /// Path to the trajectory we will b-spline and simulate on. Should be time(s),pos(xyz),ori(xyzw) format.
  std::string BSpline_path;

  /// Path to the planes for LiDAR measurement
  std::string planes_path;

  /// boolean for adding nonholonomic constraint to simulated trajectory
  bool const_holonomic = true;

  /// boolean for adding planar constraint to simulated trajectory
  bool const_planar = false;

  /// We will start simulating after we have moved this much along the b-spline.
  /// This prevents static starts as we init from groundtruth in simulation.
  double distance_threshold = 1.2;

  /// Frequency (Hz) that we will simulate our sensors
  double freq_gps = 1.0;
  double freq_cam = 10.0;
  double freq_imu = 400.0;
  double freq_vicon = 10.0;
  double freq_lidar = 10.0;
  double freq_wheel = 100.0;

  /// Feature distance we generate features from (minimum)
  double min_feature_gen_distance = 5;

  /// Feature distance we generate features from (maximum)
  double max_feature_gen_distance = 10;

  /// World(local)-to-ENU transformation (q_WtoE, p_WinE).
  Eigen::VectorXd WtoE_trans;

  /// Estimator options contains the TRUE values
  std::shared_ptr<OptionsEstimator> est_true;
};

} // namespace mins

#endif // MINS_OPTIONSSIMULATION_H
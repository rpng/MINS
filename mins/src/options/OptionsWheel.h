/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
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

#ifndef MINS_OPTIONSWHEEL_H
#define MINS_OPTIONSWHEEL_H

#include <Eigen/Eigen>
#include <memory>
#include <vector>

using namespace std;
namespace ov_core {
class YamlParser;
}
namespace mins {
/**
 * @brief Struct which stores all wheel options needed for state estimation.
 */
struct OptionsWheel {

  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  void print();

  /// boolean for sensor use
  bool enabled = true;

  /// rostopic to subscribe
  std::string topic;

  vector<string> sub_topics = {"front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"};

  /// Type of the wheel
  string type;

  /// Noise settings for the measurements (angular/linear velocities & planar motion constraint)
  double noise_w = 0.005;
  double noise_v = 0.005;
  double noise_p = 0.01;

  /// Initial calibration covariance (timeoffset, extrinsic orientation/position, intrinsic base length/wheel radii)
  double init_cov_dt = 1e-4;
  double init_cov_ex_o = 1e-4;
  double init_cov_ex_p = 1e-3;
  double init_cov_in_b = 1e-4;
  double init_cov_in_r = 1e-4;

  /// Wheel timeoffset. Default value 0,0
  double dt = 0.0;

  /// Chi threshold for outlier rejection
  double chi2_mult = 1;

  /// wheel extrinsics (q_ItoW, p_IinW). Default value identity
  Eigen::VectorXd extrinsics;

  /// wheel intrinsics (r_l, r_r, b)
  Eigen::Vector3d intrinsics;

  /// Bool to determine whether or not to calibrate timeoffset
  bool do_calib_dt = true;

  /// Bool to determine whether or not to calibrate extrinsics
  bool do_calib_ext = true;

  /// Bool to determine whether or not to calibrate intrinsics
  bool do_calib_int = true;

  /// Enabling this will always create relative pose measurement between the oldest and newest clones
  bool reuse_of_information = false;
};
} // namespace mins

#endif // MINS_OPTIONSWHEEL_H

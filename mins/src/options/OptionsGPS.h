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

#ifndef MINS_OPTIONSGPS_H
#define MINS_OPTIONSGPS_H

#include <Eigen/Eigen>
#include <map>
#include <memory>
#include <vector>
namespace ov_core {
class YamlParser;
}
namespace mins {

/**
 * @brief Struct which stores all gps options needed for state estimation.
 */
struct OptionsGPS {

  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  void print();

  /// boolean for usage of sensor
  bool enabled = true;

  /// Max number of sensor
  int max_n = 2;

  /// rostopic to subscribe
  std::vector<std::string> topic;

  /// Initial calibration covariance (timeoffset, extrinsic)
  double init_cov_dt = 1e-3;
  double init_cov_ex = 1e-3;

  /// Chi threshold for outlier rejection
  double chi2_mult = 2;

  /// Map between gps id and timeoffset. Default value 0
  std::map<size_t, double> dt;

  /// Map between gps id and extrinsic (p_IinG). Default value identity
  std::map<size_t, Eigen::Vector3d> extrinsics;

  /// Bool to determine whether or not to calibrate timeoffset
  bool do_calib_dt = true;

  /// Bool to determine whether or not to calibrate extrinsics
  bool do_calib_ext = true;

  /// Distance threshold for initializing GPS initialization
  double init_distance = 30;

  /// Boolean to get the initial guess in closed form solution
  bool init_closed_form = true;

  /// coefficient for covariance inflation after ENU-World initalization
  double init_cov_inflation = 1.0;

  /// Simulated GPS noise (standard deviation)
  bool overwrite_noise = false;
  double noise = 0.1;

private:
  void load_i(const std::shared_ptr<ov_core::YamlParser> &parser, int i);
  void print_i(int i);
};
} // namespace mins
#endif // MINS_OPTIONSGPS_H

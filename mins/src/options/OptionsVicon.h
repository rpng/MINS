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

#ifndef MINS_OPTIONSVICON_H
#define MINS_OPTIONSVICON_H

#include <Eigen/Eigen>
#include <memory>
#include <vector>

namespace ov_core {
class YamlParser;
}
namespace mins {

/**
 * @brief Struct which stores all vicon options needed for state estimation.
 */
struct OptionsVicon {

  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  void print();

  /// boolean for sensor use
  bool enabled = true;

  /// Max number of sensor
  int max_n = 2;

  /// rostopic to subscribe
  std::vector<std::string> topic;

  /// Measurement noise
  double noise_o = 0.01;
  double noise_p = 0.01;
  double chi2_mult = 1;

  /// Initial calibration covariance (timeoffset, extrinsic orientation, extrinsic position)
  double init_cov_dt = 1e-3;
  double init_cov_ex_o = 1e-3;
  double init_cov_ex_p = 1e-3;

  /// Map between vicon id and timeoffset. Default value 0
  std::map<size_t, double> dt;

  /// Map between vicon id and extrinsics (q_ItoV, p_IinV). Default value identity
  std::map<size_t, Eigen::VectorXd> extrinsics;

  /// Bool to determine whether or not to calibrate timeoffset
  bool do_calib_dt = false;

  /// Bool to determine whether or not to calibrate extrinsics
  bool do_calib_ext = false;

private:
  void load_i(const std::shared_ptr<ov_core::YamlParser> &parser, int i);
  void print_i(int i);
};
} // namespace mins

#endif // MINS_OPTIONSVICON_H

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

#ifndef MINS_OPTIONSSYSTEM_H
#define MINS_OPTIONSSYSTEM_H

#include <memory>
#include <string>
namespace ov_core {
class YamlParser;
}

namespace mins {

struct OptionsSystem {

  void load_print(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  /// Bag info
  double bag_start = 0;
  double bag_durr = -1;
  std::string path_bag;

  /// ground truth file path
  std::string path_gt;

  /// If we should record the timing performance to file
  bool save_timing = false;

  /// If we should record full state estimation to file
  bool save_state = false;

  /// If we should record the trajectory (pose) estimation to file
  bool save_trajectory = false;

  /// Log outputs
  bool save_prints = false;

  /// The path to the file we will record the state information into
  std::string path_state;

  /// The path to the file we will record the timing into
  std::string path_timing;

  /// The path to the file we will record the trajectory into
  std::string path_trajectory;

  /// Experiment number
  int exp_id = 0;

  /// Message verbosity
  int verbosity = 2;
};
} // namespace mins

#endif // MINS_OPTIONSSYSTEM_H

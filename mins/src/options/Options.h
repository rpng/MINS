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

#ifndef MINS_OPTIONS_H
#define MINS_OPTIONS_H

#include <memory>
using namespace std;
namespace ov_core {
class YamlParser;
}

namespace mins {

struct OptionsEstimator;
struct OptionsSimulation;
struct OptionsSystem;
/**
 * @brief Struct which stores all options needed for MINS.
 */
struct Options {
  Options();

  ~Options(){};

  /**
   * @brief This function will load the parameters of the system and print.
   * @param parser If not null, this parser will be used to load our parameters
   */
  void load_print(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  /// System options
  std::shared_ptr<OptionsSystem> sys;

  /// Simulator options
  std::shared_ptr<OptionsSimulation> sim;

  /// Estimator options
  std::shared_ptr<OptionsEstimator> est;
};

} // namespace mins

#endif // MINS_OPTIONS_H
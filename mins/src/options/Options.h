/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
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
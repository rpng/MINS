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

#include "Options.h"
#include "OptionsEstimator.h"
#include "OptionsGPS.h"
#include "OptionsInit.h"
#include "OptionsSimulation.h"
#include "OptionsSystem.h"
#include "OptionsVicon.h"
#include "OptionsWheel.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

mins::Options::Options() {
  sys = std::make_shared<OptionsSystem>();
  sim = std::make_shared<OptionsSimulation>();
  est = std::make_shared<OptionsEstimator>();
};

void mins::Options::load_print(const std::shared_ptr<ov_core::YamlParser> &parser) {
  // Load parameters
  sys->load_print(parser);
  est->load_print(parser);
  sim->load_print(parser);

  // force nonholonomic constraint to simulated trajectory when using wheel measurements
  if (est->wheel->enabled && sim->const_holonomic && boost::filesystem::exists(parser->get_config_folder() + "config_simulation.yaml")) {
    PRINT3(YELLOW "Enable non-holonomic constraint for wheel simulation.\n" RESET);
    sim->const_holonomic = false;
  }

  // Both GPS and Vicon provide global measurements that do not match each other.
  if (est->gps->enabled && est->vicon->enabled) {
    PRINT4(RED "Both GPS and Vicon are enabled. Disable one of them.\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Match the ground truth file path if we are initializing from ground truth
  if (est->init->use_gt)
    est->init->path_gt = sys->path_gt;
}
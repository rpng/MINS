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

#include "OptionsSystem.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

void mins::OptionsSystem::load_print(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_system";
    parser->parse_external(f, "sys", "verbosity", verbosity);
    mins::Print_Logger::setPrintLevel(verbosity);
    ov_core::Printer::setPrintLevel(Print_Logger::get_ov_verbosity(verbosity));
    parser->parse_external(f, "sys", "save_timing", save_timing);
    parser->parse_external(f, "sys", "path_timing", path_timing);
    parser->parse_external(f, "sys", "save_state", save_state);
    parser->parse_external(f, "sys", "path_state", path_state);
    parser->parse_external(f, "sys", "save_trajectory", save_trajectory);
    parser->parse_external(f, "sys", "path_trajectory", path_trajectory);
    parser->parse_external(f, "sys", "save_prints", save_prints);
    parser->parse_external(f, "sys", "exp_id", exp_id);
    parser->parse_external(f, "sys", "path_bag", path_bag);
    parser->parse_external(f, "sys", "bag_start", bag_start);
    parser->parse_external(f, "sys", "bag_durr", bag_durr);
    parser->parse_external(f, "sys", "path_gt", path_gt, false);
    // Replace MINS_DIR if we have it
    auto dir = ament_index_cpp::get_package_share_directory("mins");
    path_timing.substr(0, 8) == "MINS_DIR" ? path_timing.replace(0, 8, dir) : std::string();
    path_state.substr(0, 8) == "MINS_DIR" ? path_state.replace(0, 8, dir) : std::string();
    path_trajectory.substr(0, 8) == "MINS_DIR" ? path_trajectory.replace(0, 8, dir) : std::string();
  }
  PRINT1(BOLDBLUE "Options - System\n" RESET);
  PRINT1("\t- save_timing: %s\n", save_timing ? "true" : "false");
  save_timing ? PRINT1("\t- path_timing: %s\n", path_timing.c_str()) : PRINT1("");
  PRINT1("\t- save_state: %s\n", save_state ? "true" : "false");
  save_state ? PRINT1("\t- path_state: %s\n", path_state.c_str()) : PRINT1("");
  PRINT1("\t- save_trajectory: %s\n", save_trajectory ? "true" : "false");
  save_trajectory ? PRINT1("\t- path_trajectory: %s\n", path_trajectory.c_str()) : PRINT1("");
  PRINT1("\t- save_prints: %s\n", save_prints ? "true" : "false");
  PRINT1("\t- exp_id: %d\n", exp_id);
  PRINT1("\t- path_bag: %s\n", path_bag.c_str());
  PRINT1("\t- bag_start: %.1f\n", bag_start);
  PRINT1("\t- bag_durr: %.1f\n", bag_durr);
  path_gt.empty() ? PRINT1("") : PRINT1("\t- path_gt: %s\n", path_gt.c_str());
}

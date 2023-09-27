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

#include "OptionsGPS.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

void mins::OptionsGPS::load(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_gps";
    if (!boost::filesystem::exists(parser->get_config_folder() + f + ".yaml")) {
      enabled = false;
      return;
    }
    parser->parse_external(f, "gps", "enabled", enabled);
    parser->parse_external(f, "gps", "max_n", max_n);
    parser->parse_external(f, "gps", "chi2_mult", chi2_mult);
    parser->parse_external(f, "gps", "do_calib_ext", do_calib_ext);
    parser->parse_external(f, "gps", "do_calib_dt", do_calib_dt);
    parser->parse_external(f, "gps", "init_distance", init_distance);
    parser->parse_external(f, "gps", "init_closed_form", init_closed_form);
    parser->parse_external(f, "gps", "init_cov_dt", init_cov_dt);
    parser->parse_external(f, "gps", "init_cov_ex", init_cov_ex);
    parser->parse_external(f, "gps", "init_cov_inflation", init_cov_inflation);
    parser->parse_external(f, "gps", "overwrite_noise", overwrite_noise);
    parser->parse_external(f, "gps", "noise", noise);
    for (int i = 0; i < max_n; i++) {
      load_i(parser, i);
    }
  }
}

void mins::OptionsGPS::load_i(const std::shared_ptr<ov_core::YamlParser> &parser, int i) {
  std::string f = "config_gps";
  // Timeoffset
  double toff = 0.0;
  parser->parse_external(f, "gps" + std::to_string(i), "timeoffset", toff);
  dt.insert({i, toff});

  // Extrinsics
  std::vector<double> T = {0, 0, 0};
  parser->parse_external(f, "gps" + std::to_string(i), "pGinI", T);
  Eigen::Vector3d eigen(T.at(0), T.at(1), T.at(2));
  extrinsics.insert({i, eigen});

  std::string gps_topic;
  parser->parse_external(f, "gps" + std::to_string(i), "topic", gps_topic);
  topic.push_back(gps_topic);
}

void mins::OptionsGPS::print() {
  if (!enabled)
    return;
  PRINT1(BOLDBLUE "Options - GNSS\n" RESET);
  PRINT1("\t- enabled: %s\n", enabled ? "true" : "false");
  PRINT1("\t- max_n: %d\n", max_n);
  PRINT1("\t- chi2_mult: %.4f\n", chi2_mult);
  PRINT1("\t- do_calib_ext: %s\n", do_calib_ext ? "true" : "false");
  PRINT1("\t- do_calib_dt: %s\n", do_calib_dt ? "true" : "false");
  PRINT1("\t- init_closed_form: %s\n", init_closed_form ? "true" : "false");
  PRINT1("\t- init_cov_dt: %.6f\n", init_cov_dt);
  PRINT1("\t- init_cov_ex: %.6f\n", init_cov_ex);
  PRINT1("\t- init_cov_inflation: %.6f\n", init_cov_inflation);
  PRINT1("\t- init_distance: %.2f\n", init_distance);
  PRINT1("\t- overwrite_noise: %s\n", overwrite_noise ? "true" : "false");
  PRINT1("\t- noise: %.2f\n", noise);
  for (int i = 0; i < max_n; i++) {
    print_i(i);
  }
}

void mins::OptionsGPS::print_i(int i) {
  PRINT1("\t- GPS%d:\n", i);
  PRINT1("\t\t- timeoffset: %.3f\n", i, dt.at(i));
  PRINT1("\t\t- pGinI: %.3f %.3f %.3f\n", i, extrinsics.at(i)(0), extrinsics.at(i)(1), extrinsics.at(i)(2));
  PRINT1("\t\t- topic: %s\n", topic.at(i).c_str());
}
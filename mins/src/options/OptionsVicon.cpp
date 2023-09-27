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

#include "OptionsVicon.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

void mins::OptionsVicon::load(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_vicon";
    if (!boost::filesystem::exists(parser->get_config_folder() + f + ".yaml")) {
      enabled = false;
      return;
    }
    parser->parse_external(f, "vicon", "enabled", enabled);
    parser->parse_external(f, "vicon", "max_n", max_n);
    parser->parse_external(f, "vicon", "noise_o", noise_o);
    parser->parse_external(f, "vicon", "noise_p", noise_p);
    parser->parse_external(f, "vicon", "chi2_mult", chi2_mult);
    parser->parse_external(f, "vicon", "do_calib_dt", do_calib_dt);
    parser->parse_external(f, "vicon", "do_calib_ext", do_calib_ext);
    parser->parse_external(f, "vicon", "init_cov_dt", init_cov_dt);
    parser->parse_external(f, "vicon", "init_cov_ex_o", init_cov_ex_o);
    parser->parse_external(f, "vicon", "init_cov_ex_p", init_cov_ex_p);
    for (int i = 0; i < max_n; i++) {
      load_i(parser, i);
    }
  }
}

void mins::OptionsVicon::load_i(const std::shared_ptr<ov_core::YamlParser> &parser, int i) {
  std::string f = "config_vicon";
  double toff = 0.0;
  parser->parse_external(f, "vicon" + std::to_string(i), "timeoffset", toff);
  dt.insert({i, toff});

  // Extrinsics
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  parser->parse_external(f, "vicon" + std::to_string(i), "T_imu_vicon", T);
  Eigen::Matrix<double, 7, 1> eigen;
  eigen.block(0, 0, 4, 1) = ov_core::rot_2_quat(T.block(0, 0, 3, 3).transpose());
  eigen.block(4, 0, 3, 1) = -T.block(0, 0, 3, 3).transpose() * T.block(0, 3, 3, 1);
  extrinsics.insert({i, eigen});

  std::string vicon_topic;
  parser->parse_external(f, "vicon" + std::to_string(i), "topic", vicon_topic);
  topic.push_back(vicon_topic);
}

void mins::OptionsVicon::print() {
  if (!enabled)
    return;
  PRINT1(BOLDBLUE "Options - Vicon\n" RESET);
  PRINT1("\t- vicon enabled: %s\n", enabled ? "true" : "false");
  PRINT1("\t- n_vicons: %d\n", max_n);
  PRINT1("\t- noise_o: %.4f\n", noise_o);
  PRINT1("\t- noise_p: %.4f\n", noise_p);
  PRINT1("\t- chi2_mult: %.4f\n", chi2_mult);
  PRINT1("\t- calib_vicon_extrinsics: %s\n", do_calib_ext ? "true" : "false");
  PRINT1("\t- calib_vicon_timeoffset: %s\n", do_calib_dt ? "true" : "false");
  PRINT1("\t- init_cov_dt: %.4f\n", init_cov_dt);
  PRINT1("\t- init_cov_ex_o: %.4f\n", init_cov_ex_o);
  PRINT1("\t- init_cov_ex_p: %.4f\n", init_cov_ex_p);
  for (int i = 0; i < max_n; i++) {
    print_i(i);
  }
}

void mins::OptionsVicon::print_i(int i) {
  PRINT1("\t- vicon%d:\n", i);
  PRINT1("\t\t- timeoffset: %6.3f\n", dt.at(i));
  PRINT1("\t\t- T_imu_vicon:\n");
  Eigen::Matrix3d R = ov_core::quat_2_Rot(extrinsics.at(i).head(4)).transpose();
  Eigen::Vector3d p = -R * extrinsics.at(i).tail(3);
  PRINT1("\t\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(0), R(3), R(6), p(0));
  PRINT1("\t\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(1), R(4), R(7), p(1));
  PRINT1("\t\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(2), R(5), R(8), p(2));
  PRINT1("\t\t\t- [ 0.000,  0.000,  0.000,  1.000]\n");
  PRINT1("\t\t- topic: %s\n", topic.at(i).c_str());
}

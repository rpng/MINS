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

#include "OptionsWheel.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

void mins::OptionsWheel::load(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_wheel";
    if (!boost::filesystem::exists(parser->get_config_folder() + f + ".yaml")) {
      enabled = false;
      return;
    }
    parser->parse_external(f, "wheel", "enabled", enabled);
    parser->parse_external(f, "wheel", "chi2_mult", chi2_mult);
    parser->parse_external(f, "wheel", "noise_w", noise_w);
    parser->parse_external(f, "wheel", "noise_v", noise_v);
    parser->parse_external(f, "wheel", "noise_p", noise_p);
    parser->parse_external(f, "wheel", "do_calib_ext", do_calib_ext);
    parser->parse_external(f, "wheel", "do_calib_dt", do_calib_dt);
    parser->parse_external(f, "wheel", "do_calib_int", do_calib_int);
    parser->parse_external(f, "wheel", "init_cov_dt", init_cov_dt);
    parser->parse_external(f, "wheel", "init_cov_ex_o", init_cov_ex_o);
    parser->parse_external(f, "wheel", "init_cov_ex_p", init_cov_ex_p);
    parser->parse_external(f, "wheel", "init_cov_in_b", init_cov_in_b);
    parser->parse_external(f, "wheel", "init_cov_in_r", init_cov_in_r);
    parser->parse_external(f, "wheel", "reuse_of_information", reuse_of_information);
    parser->parse_external(f, "wheel", "topic", topic);
    // parse the sub topics for joint status
    std::string sub_topics_;
    parser->parse_external(f, "wheel", "sub_topics", sub_topics_, false);
    if (!sub_topics_.empty()) {
      sub_topics.clear();
      while (true) {
        size_t pos = 0;
        std::string token;
        if ((pos = sub_topics_.find(", ")) != std::string::npos) {
          token = sub_topics_.substr(0, pos);
          sub_topics.push_back(token);
          sub_topics_.erase(0, pos + 2);
        } else {
          sub_topics.push_back(sub_topics_);
          break;
        }
      }
    }

    parser->parse_external(f, "wheel", "timeoffset", dt);

    // Extrinsics
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    parser->parse_external(f, "wheel", "T_imu_wheel", T);
    Eigen::Matrix<double, 7, 1> eigen;
    eigen.block(0, 0, 4, 1) = ov_core::rot_2_quat(T.block(0, 0, 3, 3).transpose());
    eigen.block(4, 0, 3, 1) = -T.block(0, 0, 3, 3).transpose() * T.block(0, 3, 3, 1);
    extrinsics = eigen;

    // Intrinsics
    std::vector<double> I = {1, 1, 2};
    parser->parse_external(f, "wheel", "intrinsics", I);
    Eigen::Vector3d intr;
    intr << I[0], I[1], I[2];
    intrinsics = intr;

    parser->parse_external(f, "wheel", "type", type);
    if (type != "Wheel2DAng" && type != "Wheel2DLin" && type != "Wheel2DCen" && type != "Wheel3DAng" && type != "Wheel3DLin" && type != "Wheel3DCen") {
      PRINT4(RED "%s is not a supported type of wheel.\n" RESET, type.c_str());
      PRINT4(RED "Available: Wheel2DAng, Wheel2DLin, Wheel2DCen, Wheel3DAng, Wheel3DLin, Wheel3DCen\n" RESET);
      exit(EXIT_FAILURE);
    }

    // should disable intrinsic calibration if not using angular velocity of wheel
    if (type != "Wheel2DAng" && type != "Wheel3DAng") {
      do_calib_int = false;
    }
  }
}
void mins::OptionsWheel::print() {
  if (!enabled)
    return;
  PRINT1(BOLDBLUE "Options - Wheel\n" RESET);
  PRINT1("\t- wheel enabled: %s\n", enabled ? "true" : "false");
  PRINT1("\t- chi2_mult: %.4f\n", chi2_mult);
  PRINT1("\t- noise_w: %.4f\n", noise_w);
  PRINT1("\t- noise_v: %.4f\n", noise_v);
  PRINT1("\t- noise_p: %.4f\n", noise_p);
  PRINT1("\t- do_calib_ext: %s\n", do_calib_ext ? "true" : "false");
  PRINT1("\t- do_calib_dt: %s\n", do_calib_dt ? "true" : "false");
  PRINT1("\t- do_calib_int: %s\n", do_calib_int ? "true" : "false");
  PRINT1("\t- reuse_of_information: %s\n", reuse_of_information ? "true" : "false");
  PRINT1("\t- init_cov_ex_o: %.6f\n", init_cov_ex_o);
  PRINT1("\t- init_cov_ex_p: %.6f\n", init_cov_ex_p);
  PRINT1("\t- init_cov_in_b: %.6f\n", init_cov_in_b);
  PRINT1("\t- init_cov_in_r: %.6f\n", init_cov_in_r);
  PRINT1("\t- init_cov_in_r: %.6f\n", init_cov_in_r);
  PRINT1("\t- WheelType: %s\n", type.c_str());
  PRINT1("\t- timeoffset: %6.3f\n", dt);
  PRINT1("\t- T_imu_wheel:\n");
  Eigen::Matrix3d R = ov_core::quat_2_Rot(extrinsics.head(4)).transpose();
  Eigen::Vector3d p = -R * extrinsics.tail(3);
  PRINT1("\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(0), R(3), R(6), p(0));
  PRINT1("\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(1), R(4), R(7), p(1));
  PRINT1("\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(2), R(5), R(8), p(2));
  PRINT1("\t\t- [ 0.000,  0.000,  0.000,  1.000]\n");
  PRINT1("\t- topic: %s\n", topic.c_str());
  PRINT1("\t- sub topics: ");
  for (auto s : sub_topics)
    PRINT1("%s, ", s.c_str());
  PRINT1("\n");
}

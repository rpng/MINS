/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This code is implemented based on:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "OptionsSimulation.h"
#include "OptionsEstimator.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

mins::OptionsSimulation::OptionsSimulation() { est_true = std::make_shared<OptionsEstimator>(); }

void mins::OptionsSimulation::load_print(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_simulation";
    if (!boost::filesystem::exists(parser->get_config_folder() + f + ".yaml")) {
      return;
    }
    parser->parse_external(f, "sim", "seed", seed);
    parser->parse_external(f, "sim", "do_perturb", do_perturb);
    parser->parse_external(f, "sim", "remove_noise", remove_noise);
    parser->parse_external(f, "sim", "BSpline_path", BSpline_path);
    parser->parse_external(f, "sim", "planes_path", planes_path);
    parser->parse_external(f, "sim", "const_holonomic", const_holonomic);
    parser->parse_external(f, "sim", "const_planar", const_planar);
    parser->parse_external(f, "sim", "distance_threshold", distance_threshold);
    parser->parse_external(f, "sim", "freq_cam", freq_cam);
    parser->parse_external(f, "sim", "freq_imu", freq_imu);
    parser->parse_external(f, "sim", "freq_gps", freq_gps);
    parser->parse_external(f, "sim", "freq_wheel", freq_wheel);
    parser->parse_external(f, "sim", "freq_vicon", freq_vicon);
    parser->parse_external(f, "sim", "freq_lidar", freq_lidar);
    parser->parse_external(f, "sim", "min_feature_gen_dist", min_feature_gen_distance);
    parser->parse_external(f, "sim", "max_feature_gen_dist", max_feature_gen_distance);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    parser->parse_external(f, "sim", "T_WtoE", T);
    WtoE_trans = Eigen::Matrix<double, 7, 1>::Zero();
    WtoE_trans.block(0, 0, 4, 1) = ov_core::rot_2_quat(T.block(0, 0, 3, 3));
    WtoE_trans.block(4, 0, 3, 1) = T.block(0, 3, 3, 1);
    // Replace MINS_DATA_DIR if we have it

    auto dir = ament_index_cpp::get_package_share_directory("mins");
    BSpline_path.substr(0, 13) == "MINS_DATA_DIR" ? BSpline_path.replace(0, 13, dir) : std::string();
    planes_path.substr(0, 13) == "MINS_DATA_DIR" ? planes_path.replace(0, 13, dir) : std::string();

    // Load Ground truth estimator parameters
    est_true->load(parser);
  }
  PRINT1(BOLDBLUE "Options - Simulator\n" RESET);
  PRINT1("\t- seed: %d \n", seed);
  PRINT1("\t- do_perturb: %s\n", do_perturb ? "true" : "false");
  PRINT1("\t- remove_noise: %s\n", remove_noise ? "true" : "false");
  PRINT1("\t- BSpline_path: %s\n", BSpline_path.c_str());
  PRINT1("\t- planes_path: %s\n", planes_path.c_str());
  PRINT1("\t- const_holonomic: %s\n", const_holonomic ? "true" : "false");
  PRINT1("\t- const_planar: %s\n", const_planar ? "true" : "false");
  PRINT1("\t- distance_threshold: %.2f\n", distance_threshold);
  PRINT1("\t- freq_cam: %.2f\n", freq_cam);
  PRINT1("\t- freq_imu: %.2f\n", freq_imu);
  PRINT1("\t- freq_gps: %.2f\n", freq_gps);
  PRINT1("\t- freq_wheel: %.2f\n", freq_wheel);
  PRINT1("\t- freq_vicon: %.2f\n", freq_vicon);
  PRINT1("\t- freq_lidar: %.2f\n", freq_lidar);
  PRINT1("\t- min_feature_gen_distance: %.2f\n", min_feature_gen_distance);
  PRINT1("\t- max_feature_gen_distance: %.2f\n", max_feature_gen_distance);
  PRINT1("\t- T_WtoE:\n");
  Eigen::Matrix3d R = ov_core::quat_2_Rot(WtoE_trans.block(0, 0, 4, 1));
  Eigen::Vector3d p = WtoE_trans.block(4, 0, 3, 1);
  PRINT1("\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(0), R(1), R(2), p(0));
  PRINT1("\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(3), R(4), R(5), p(1));
  PRINT1("\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(6), R(7), R(8), p(2));
  PRINT1("\t\t- [ 0.000,  0.000,  0.000,  1.000]\n");
}
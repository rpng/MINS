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

#include "OptionsLidar.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

void mins::OptionsLidar::load(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_lidar";
    if (!boost::filesystem::exists(parser->get_config_folder() + f + ".yaml")) {
      enabled = false;
      return;
    }
    parser->parse_external(f, "lidar", "enabled", enabled);
    parser->parse_external(f, "lidar", "max_n", max_n);
    parser->parse_external(f, "lidar", "chi2_mult", chi2_mult);
    parser->parse_external(f, "lidar", "do_calib_dt", do_calib_dt);
    parser->parse_external(f, "lidar", "do_calib_ext", do_calib_ext);
    parser->parse_external(f, "lidar", "init_cov_dt", init_cov_dt);
    parser->parse_external(f, "lidar", "init_cov_ex_o", init_cov_ex_o);
    parser->parse_external(f, "lidar", "init_cov_ex_p", init_cov_ex_p);
    parser->parse_external(f, "lidar", "min_range", min_range);
    parser->parse_external(f, "lidar", "max_range", max_range);
    parser->parse_external(f, "lidar", "raw_do_downsample", raw_do_downsample);
    parser->parse_external(f, "lidar", "raw_downsample_size", raw_downsample_size);
    parser->parse_external(f, "lidar", "raw_remove_motion_blur", raw_remove_motion_blur);
    parser->parse_external(f, "lidar", "raw_point_dt", raw_point_dt);
    parser->parse_external(f, "lidar", "raw_noise", raw_noise);
    parser->parse_external(f, "lidar", "map_noise", map_noise);
    parser->parse_external(f, "lidar", "map_ngbr_num", map_ngbr_num);
    parser->parse_external(f, "lidar", "map_ngbr_max_d", map_ngbr_max_d);
    parser->parse_external(f, "lidar", "map_downsample_size", map_downsample_size);
    parser->parse_external(f, "lidar", "map_do_downsample", map_do_downsample);
    parser->parse_external(f, "lidar", "map_decay_time", map_decay_time);
    parser->parse_external(f, "lidar", "map_decay_dist", map_decay_dist);
    parser->parse_external(f, "lidar", "map_use_icp", map_use_icp);
    parser->parse_external(f, "lidar", "map_icp_dist", map_icp_dist);
    parser->parse_external(f, "lidar", "plane_max_condi", plane_max_condi);
    parser->parse_external(f, "lidar", "plane_max_p2pd", plane_max_p2pd);
    for (int i = 0; i < max_n; i++) {
      load_i(parser, i);
    }
  }
}

void mins::OptionsLidar::load_i(const std::shared_ptr<ov_core::YamlParser> &parser, int i) {
  std::string f = "config_lidar";
  double toff = 0.0;
  parser->parse_external(f, "lidar" + std::to_string(i), "timeoffset", toff);
  dt.insert({i, toff});

  // Extrinsics
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  parser->parse_external(f, "lidar" + std::to_string(i), "T_imu_lidar", T);
  Eigen::Matrix<double, 7, 1> eigen;
  eigen.block(0, 0, 4, 1) = ov_core::rot_2_quat(T.block(0, 0, 3, 3).transpose());
  eigen.block(4, 0, 3, 1) = -T.block(0, 0, 3, 3).transpose() * T.block(0, 3, 3, 1);
  extrinsics.insert({i, eigen});

  std::vector<double> v_angle = {-1, 0, 1};
  parser->parse_external(f, "lidar" + std::to_string(i), "v_angles", v_angle, false);
  v_angles.insert({i, v_angle});

  double h_resol = 0.5;
  parser->parse_external(f, "lidar" + std::to_string(i), "h_resolution", h_resol, false);
  h_resolution.insert({i, h_resol});

  double h_st = -180;
  parser->parse_external(f, "lidar" + std::to_string(i), "h_start", h_st, false);
  h_start.insert({i, h_st});

  double h_ed = 180;
  parser->parse_external(f, "lidar" + std::to_string(i), "h_end", h_ed, false);
  h_end.insert({i, h_ed});

  // compute and get horizontal angles
  std::vector<double> horizontal_angles;
  for (double ang = h_start.at(i); ang <= h_end.at(i);) {
    horizontal_angles.push_back(ang);
    ang += h_resolution.at(i);
  }
  h_angles.insert({i, horizontal_angles});

  std::string lidar_topic;
  parser->parse_external(f, "lidar" + std::to_string(i), "topic", lidar_topic);
  topic.push_back(lidar_topic);
}

void mins::OptionsLidar::print() {
  if (!enabled)
    return;
  PRINT1(BOLDBLUE "Options - LiDAR\n" RESET);
  PRINT1("\t- lidar enabled: %s\n", enabled ? "true" : "false");
  PRINT1("\t- n_lidars: %d\n", max_n);
  PRINT1("\t- chi2_mult: %.4f\n", chi2_mult);
  PRINT1("\t- max_range: %.2f\n", max_range);
  PRINT1("\t- min_range: %.2f\n", min_range);
  PRINT1("\t- calib_lidar_extrinsics: %s\n", do_calib_ext ? "true" : "false");
  PRINT1("\t- calib_lidar_timeoffset: %s\n", do_calib_dt ? "true" : "false");
  PRINT1("\t- init_cov_dt: %.4f\n", init_cov_dt);
  PRINT1("\t- init_cov_ex_o: %.4f\n", init_cov_ex_o);
  PRINT1("\t- init_cov_ex_p: %.4f\n", init_cov_ex_p);
  for (int i = 0; i < max_n; i++) {
    print_i(i);
  }
}

void mins::OptionsLidar::print_i(int i) {
  PRINT1("\t- LiDAR%d:\n", i);
  PRINT1("\t\t- timeoffset: %6.3f\n", dt.at(i));
  PRINT1("\t\t- n_channel: %d\n", v_angles.at(i).size());
  PRINT1("\t\t- angular_resolution: %.3f\n", h_resolution.at(i));
  PRINT1("\t\t- fov_angle_start: %.3f\n", h_start.at(i));
  PRINT1("\t\t- fov_angle_end: %.3f\n", h_end.at(i));
  PRINT1("\t\t- T_imu_lidar:\n");
  Eigen::Matrix3d R = ov_core::quat_2_Rot(extrinsics.at(i).head(4)).transpose();
  Eigen::Vector3d p = -R * extrinsics.at(i).tail(3);
  PRINT1("\t\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(0), R(3), R(6), p(0));
  PRINT1("\t\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(1), R(4), R(7), p(1));
  PRINT1("\t\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(2), R(5), R(8), p(2));
  PRINT1("\t\t\t- [ 0.000,  0.000,  0.000,  1.000]\n");
  PRINT1("\t\t- topic: %s\n", topic.at(i).c_str());
}
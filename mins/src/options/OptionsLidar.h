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

#ifndef MINS_OPTIONSLIDAR_H
#define MINS_OPTIONSLIDAR_H

#include <Eigen/Eigen>
#include <map>
#include <memory>
#include <vector>
namespace ov_core {
class YamlParser;
}
namespace mins {

/**
 * @brief Struct which stores all lidar options needed for state estimation.
 */
struct OptionsLidar {

  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  void print();

  /// boolean for usage of sensor
  bool enabled = true;

  /// Max number of sensor
  int max_n = 2;

  /// rostopic to subscribe
  std::vector<std::string> topic;

  /// Chi threshold for outlier rejection
  double chi2_mult = 1;

  /// initial covariance values for calibration parameters
  double init_cov_dt = 1e-3;
  double init_cov_ex_o = 1e-3;
  double init_cov_ex_p = 1e-3;

  /// Lidar timeoffset. Default value 0,0
  std::map<size_t, double> dt;

  /// Map between lidar_id and lidar extrinsics (q_ItoV, p_IinV). Default value identity
  std::map<size_t, Eigen::VectorXd> extrinsics;

  /// Bool to determine whether or not to calibrate lidar to IMU timeoffset
  bool do_calib_dt = false;

  /// Bool to determine whether or not to calibrate IMU-to-lidar pose
  bool do_calib_ext = false;

  /// min max range of the LiDAR pointcloud used
  double max_range = 100.0;
  double min_range = 0.05;

  /// Vertical, Horizontal angles of 3D LiDAR scan
  std::map<size_t, std::vector<double>> v_angles, h_angles;
  std::map<size_t, double> h_resolution;
  std::map<size_t, double> h_start;
  std::map<size_t, double> h_end;

  // Raw pointcloud parameters
  bool raw_do_downsample = false;
  double raw_downsample_size = 0.5;
  double raw_noise = 0.01;
  bool raw_remove_motion_blur = true;
  double raw_point_dt = 1e-6;

  // IKD tree parameters
  int map_ngbr_num = 5;             // minimum number of neighbors for plane extraction
  double map_ngbr_max_d = 1;        // maximum distance of the most far neighbor (m)
  double map_downsample_size = 0.1; // ikd tree voxcel size (m)
  bool map_do_downsample = true;
  double map_noise = 1.0;
  double map_decay_time = 15;
  double map_decay_dist = 50;
  bool map_use_icp = true;
  double map_icp_dist = 20;
  double plane_max_condi = 100.0;
  double plane_max_p2pd = 0.1; // point to plane distance threshold. for plane extraction (m)

private:
  void load_i(const std::shared_ptr<ov_core::YamlParser> &parser, int i);
  void print_i(int i);
};
} // namespace mins

#endif // MINS_OPTIONSLIDAR_H

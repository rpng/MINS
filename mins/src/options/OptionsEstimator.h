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

#ifndef MINS_OPTIONSESTIMATOR_H
#define MINS_OPTIONSESTIMATOR_H

#include <Eigen/Eigen>
#include <memory>
using namespace std;
namespace ov_core {
class YamlParser;
}
namespace mins {
struct OptionsIMU;
struct OptionsCamera;
struct OptionsVicon;
struct OptionsGPS;
struct OptionsWheel;
struct OptionsLidar;
struct OptionsInit;

/**
 * @brief Struct which stores all options needed for state estimation.
 */
struct OptionsEstimator {
  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  void print();

  void load_print(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  /// IMU options
  shared_ptr<OptionsIMU> imu;

  /// Camera options
  shared_ptr<OptionsCamera> cam;

  /// Vicon options
  shared_ptr<OptionsVicon> vicon;

  /// GPS options
  shared_ptr<OptionsGPS> gps;

  /// Wheel options
  shared_ptr<OptionsWheel> wheel;

  /// LiDAR options
  shared_ptr<OptionsLidar> lidar;

  /// Initialization options
  shared_ptr<OptionsInit> init;

  /// Gravity in the global frame
  Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

  /// Sliding window size
  double window_size = 0.5; // s

  /// Clone hz
  int clone_freq = 10;

  /// Allowance of extrapolation (s)
  double dt_exp = 0.01;

  /// Computes the interpolation error covariance
  struct interpolation_error {
  public:
    Eigen::MatrixXd pose_cov(int clone_hz, int order, double est_A, double est_a) {
      Eigen::MatrixXd err_cov = Eigen::MatrixXd::Zero(6, 6);
      err_cov.block(0, 0, 3, 3) = ori_cov(clone_hz, order, est_A) * Eigen::Matrix3d::Identity();
      err_cov.block(3, 3, 3, 3) = pos_cov(clone_hz, order, est_a) * Eigen::Matrix3d::Identity();
      return err_cov;
    }

    Eigen::MatrixXd pose_std(int clone_hz, int order, double est_A, double est_a) {
      Eigen::MatrixXd err_cov = Eigen::MatrixXd::Zero(6, 6);
      err_cov.block(0, 0, 3, 3) = ori_std(clone_hz, order, est_A) * Eigen::Matrix3d::Identity();
      err_cov.block(3, 3, 3, 3) = pos_std(clone_hz, order, est_a) * Eigen::Matrix3d::Identity();
      return err_cov;
    }

    double ori_cov(int clone_hz, int order, double est_A) { return std::pow(ori_std(clone_hz, order, est_A), 2); }

    double ori_std(int clone_hz, int order, double est_A) { return mlt * est_A * ori_slope.at(clone_hz).at(order); }

    double pos_cov(int clone_hz, int order, double est_a) { return std::pow(pos_std(clone_hz, order, est_a), 2); }

    double pos_std(int clone_hz, int order, double est_a) { return mlt * est_a * pos_slope.at(clone_hz).at(order); }

    // Threshold for dynamic cloning
    double threshold_ori = 0.01;  // rad
    double threshold_pos = 0.001; // m

    double mlt = 1.0;

    vector<int> available_clone_hz() {
      vector<int> vec_clone_hz;
      for (const auto &clone_hz : ori_slope) {
        vec_clone_hz.push_back(clone_hz.first);
      }
      return vec_clone_hz;
    }

  private:
    friend struct OptionsEstimator;
    // parameter reading
    std::vector<double> reading_ori_slope = {-1, -1, -1, -1, -1};
    std::vector<double> reading_pos_slope = {-1, -1, -1, -1, -1};

    // {Clone HZ, {order, slope}}
    std::map<int, std::map<int, double>> ori_slope;
    std::map<int, std::map<int, double>> pos_slope;

    void set_values(int hz);
  };

  interpolation_error intr_err;

  /// order of polynomial for interpolation
  int intr_order = 3;

  /// use imu prediction for residual computation
  bool use_imu_res = false;
  bool use_imu_cov = false;
  bool use_pol_cov = false;

  /// boolean for dynamic cloning
  bool dynamic_cloning = true;
};

} // namespace mins

#endif // MINS_OPTIONSESTIMATOR_H

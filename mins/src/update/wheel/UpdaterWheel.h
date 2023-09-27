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

#ifndef MINS_UPDATERWHEEL_H
#define MINS_UPDATERWHEEL_H

#include <Eigen/Eigen>
#include <deque>
#include <memory>

namespace mins {

class State;
class UpdaterStatistics;
struct WheelData;
class UpdaterWheel {

public:
  /// Wheel updater
  UpdaterWheel(std::shared_ptr<State> state);

  /// Get wheel measurement
  void feed_measurement(WheelData data);

  /// Try update with available measurements
  void try_update();

  /// chi-2 checker
  std::shared_ptr<UpdaterStatistics> Chi;

  /// measurement history
  std::deque<double> t_hist;

private:
  friend class Initializer;
  friend class IW_Initializer;
  /**
   * @brief Checks if we have enough clones and handover two last clone times to update
   * @param state current state info
   * @param time0 start timestamp of the update
   * @param time1 end timestamp of the update
   */
  bool update(double time0, double time1);

  /// get two wheel data bounding the t_given time
  bool get_bounding_data(double t_given, std::vector<WheelData> &data_stack, WheelData &data1, WheelData &data2);

  /**
   * @brief Nice helper function that will linearly interpolate between two wheel messages.
   * This should be used instead of just "cutting" wheel messages that bound the clone times
   * Give better timeoffset if we use this function, could try other orders/splines if the wheel is slow.
   * @param data1 wheel at begining of interpolation interval
   * @param data2 wheel at end of interpolation interval
   * @param timestamp Timestamp being interpolated to
   */
  static WheelData interpolate_data(const WheelData data1, const WheelData data2, double timestamp);

  /**
   * @brief Compute 2D or 3D Jacobians of intrinsic parameters during preintegration
   * @param dt time interval between the preintegration steps
   * @param WheelData wheel measurement of the current step
   */
  void preintegration_intrinsics_2D(double dt, WheelData data);
  void preintegration_intrinsics_3D(double dt, WheelData data);

  /**
   * @brief Compute 2D or 3D linearized system vectors and matrices
   * @param H the Jacobian matrix of the state
   * @param res measurement residual
   * @param time0 the timestamp of starting pose (pose0) to update
   * @param time1 the timestamp of end pose (pose1) to update
   */
  void compute_linear_system_2D(Eigen::MatrixXd &H, Eigen::VectorXd &res, double time0, double time1);
  void compute_linear_system_3D(Eigen::MatrixXd &H, Eigen::VectorXd &res, double time0, double time1);

  /**
   * @brief Preintegrate 2D or 3D wheel measurement including transition matrix and noise Jacobians
   * @param dt time interval between the preintegration steps
   * @param data1 wheel measurement of the start time
   * @param data2 wheel measurement of the end time
   */
  void preintegration_2D(double dt, WheelData data1, WheelData data2);
  void preintegration_3D(double dt, WheelData data1, WheelData data2);

  /**
   * @brief Collects a set of wheel measurements between time0 and time1
   * @param data_vec output as a set of wheel measurements
   * @param time0 start timestamp of the update
   * @param state end timestamp of the update
   */
  bool select_wheel_data(double time0, double time1, std::vector<WheelData> &data_vec);

  /// Our history of wheel messages (time, ang_left, ang_right)
  std::vector<WheelData> data_stack;

  /// preintegrated 2D measurement
  double th_2D;
  double x_2D;
  double y_2D;

  /// preintegrated 3D measurement
  Eigen::Matrix3d R_3D;
  Eigen::Vector3d p_3D;

  /// Preintegrated 2D/3D measurement noise covariance
  Eigen::Matrix3d Cov_2D = Eigen::Matrix3d::Zero();
  Eigen::MatrixXd Cov_3D = Eigen::MatrixXd::Zero(6, 6);

  /// Preintegrated 2D jacobian respect to intrinsics
  Eigen::Matrix<double, 1, 3> dth_di_2D;
  Eigen::Matrix<double, 1, 3> dx_di_2D;
  Eigen::Matrix<double, 1, 3> dy_di_2D;

  /// Preintegrated 3D jacobian respect to intrinsics
  Eigen::Matrix3d dR_di_3D;
  Eigen::Matrix3d dp_di_3D;

  /// record of last updated clone time
  double last_updated_clone_time = -1;

  /// State
  std::shared_ptr<State> state;
};
} // namespace mins
#endif // MINS_UPDATERWHEEL_H

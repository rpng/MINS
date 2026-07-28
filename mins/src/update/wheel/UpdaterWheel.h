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
#include <utility>

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

  /**
   * \brief Compute the 3-DOF 2D wheel odometry residual.
   * \param[in] R_GtoI0 Rotation from global to IMU frame at time 0.
   * \param[in] p_I0inG Position of IMU at time 0 expressed in global frame.
   * \param[in] R_GtoI1 Rotation from global to IMU frame at time 1.
   * \param[in] p_I1inG Position of IMU at time 1 expressed in global frame.
   * \param[in] R_ItoO Rotation from IMU frame to wheel odometry frame.
   * \param[in] p_IinO Position of IMU expressed in wheel odometry frame.
   * \param[in] th Preintegrated 2D heading angle from wheel odometry.
   * \param[in] x Preintegrated 2D forward displacement from wheel odometry.
   * \param[in] y Preintegrated 2D lateral displacement from wheel odometry.
   * \return Residual 3-vector [heading_error, dx_error, dy_error].
   */
  static Eigen::Vector3d ComputeResidual2D(const Eigen::Matrix3d &R_GtoI0, const Eigen::Vector3d &p_I0inG,
                                            const Eigen::Matrix3d &R_GtoI1, const Eigen::Vector3d &p_I1inG,
                                            const Eigen::Matrix3d &R_ItoO, const Eigen::Vector3d &p_IinO,
                                            double th, double x, double y);

  /**
   * \brief Compute analytical Jacobians of the 2D wheel odometry residual.
   * Returns Jacobians w.r.t. the two IMU poses (3x12) and the wheel extrinsic (3x6).
   * Pose columns are ordered [delta_th0(3), delta_p0(3), delta_th1(3), delta_p1(3)].
   * Extrinsic columns are ordered [delta_thO(3), delta_pO(3)].
   * \param[in] R_GtoI0 Rotation from global to IMU frame at time 0.
   * \param[in] p_I0inG Position of IMU at time 0 expressed in global frame.
   * \param[in] R_GtoI1 Rotation from global to IMU frame at time 1.
   * \param[in] p_I1inG Position of IMU at time 1 expressed in global frame.
   * \param[in] R_ItoO Rotation from IMU frame to wheel odometry frame.
   * \param[in] p_IinO Position of IMU expressed in wheel odometry frame.
   * \return {H_poses (3x12), H_ext (3x6)} Jacobians w.r.t. IMU poses and extrinsic calibration.
   */
  static std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ComputeJacobians2D(const Eigen::Matrix3d &R_GtoI0, const Eigen::Vector3d &p_I0inG,
                                                                          const Eigen::Matrix3d &R_GtoI1, const Eigen::Vector3d &p_I1inG,
                                                                          const Eigen::Matrix3d &R_ItoO, const Eigen::Vector3d &p_IinO);

  /**
   * \brief Compute the 6-DOF 3D wheel odometry residual.
   * \param[in] R_GtoI0 Rotation from global to IMU frame at time 0.
   * \param[in] p_I0inG Position of IMU at time 0 expressed in global frame.
   * \param[in] R_GtoI1 Rotation from global to IMU frame at time 1.
   * \param[in] p_I1inG Position of IMU at time 1 expressed in global frame.
   * \param[in] R_ItoO Rotation from IMU frame to wheel odometry frame.
   * \param[in] p_IinO Position of IMU expressed in wheel odometry frame.
   * \param[in] R_3D Preintegrated 3D orientation from wheel odometry.
   * \param[in] p_3D Preintegrated 3D position from wheel odometry.
   * \return Residual 6-vector [orientation_error_3, position_error_3].
   */
  static Eigen::Matrix<double, 6, 1> ComputeResidual3D(const Eigen::Matrix3d &R_GtoI0, const Eigen::Vector3d &p_I0inG,
                                                         const Eigen::Matrix3d &R_GtoI1, const Eigen::Vector3d &p_I1inG,
                                                         const Eigen::Matrix3d &R_ItoO, const Eigen::Vector3d &p_IinO,
                                                         const Eigen::Matrix3d &R_3D, const Eigen::Vector3d &p_3D);

  /**
   * \brief Compute analytical Jacobians of the 3D wheel odometry residual.
   * Returns Jacobians w.r.t. the two IMU poses (6x12) and the wheel extrinsic (6x6).
   * Pose columns are ordered [delta_th0(3), delta_p0(3), delta_th1(3), delta_p1(3)].
   * Extrinsic columns are ordered [delta_thO(3), delta_pO(3)].
   * \param[in] R_GtoI0 Rotation from global to IMU frame at time 0.
   * \param[in] p_I0inG Position of IMU at time 0 expressed in global frame.
   * \param[in] R_GtoI1 Rotation from global to IMU frame at time 1.
   * \param[in] p_I1inG Position of IMU at time 1 expressed in global frame.
   * \param[in] R_ItoO Rotation from IMU frame to wheel odometry frame.
   * \param[in] p_IinO Position of IMU expressed in wheel odometry frame.
   * \return {H_poses (6x12), H_ext (6x6)} Jacobians w.r.t. IMU poses and extrinsic calibration.
   */
  static std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ComputeJacobians3D(const Eigen::Matrix3d &R_GtoI0, const Eigen::Vector3d &p_I0inG,
                                                                          const Eigen::Matrix3d &R_GtoI1, const Eigen::Vector3d &p_I1inG,
                                                                          const Eigen::Matrix3d &R_ItoO, const Eigen::Vector3d &p_IinO);

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

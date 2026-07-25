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

#ifndef MINS_UPDATERVICON_H
#define MINS_UPDATERVICON_H

#include <Eigen/Core>
#include <deque>
#include <map>
#include <memory>
#include <vector>

using namespace std;

namespace mins {
class State;
class UpdaterStatistics;
struct ViconData;

class UpdaterVicon {

public:
  /// Vicon updater
  UpdaterVicon(shared_ptr<State> state);

  /**
   * \brief Compute the 6-DOF Vicon measurement residual.
   *
   * \param[in] rotation_global_to_imu Rotation matrix from global frame to IMU frame.
   * \param[in] position_imu_in_global Position of the IMU expressed in the global frame.
   * \param[in] rotation_imu_to_vicon Rotation matrix from IMU frame to Vicon sensor frame.
   * \param[in] position_imu_in_vicon Position of the IMU expressed in the Vicon sensor frame.
   * \param[in] measurement_pose Stacked 6-vector [orientation_3, position_3] from the Vicon measurement.
   * \return Residual 6-vector [orientation_error_3, position_error_3].
   */
  static Eigen::Matrix<double, 6, 1> ComputeResidual(const Eigen::Matrix3d &rotation_global_to_imu,
                                                      const Eigen::Vector3d &position_imu_in_global,
                                                      const Eigen::Matrix3d &rotation_imu_to_vicon,
                                                      const Eigen::Vector3d &position_imu_in_vicon,
                                                      const Eigen::Matrix<double, 6, 1> &measurement_pose);

  /**
   * \brief Compute analytical Jacobians of the Vicon residual.
   *
   * Both output matrices are 6x6; columns follow [delta_rotation, delta_position].
   *
   * \param[in] rotation_global_to_imu Rotation matrix from global frame to IMU frame.
   * \param[in] rotation_imu_to_vicon Rotation matrix from IMU frame to Vicon sensor frame.
   * \param[in] position_imu_in_vicon Position of the IMU expressed in the Vicon sensor frame.
   * \param[out] dz_dI Jacobian of the residual with respect to the IMU pose state.
   * \param[out] dz_dcalib Jacobian of the residual with respect to the extrinsic calibration.
   */
  static void ComputeJacobians(const Eigen::Matrix3d &rotation_global_to_imu,
                                const Eigen::Matrix3d &rotation_imu_to_vicon,
                                const Eigen::Vector3d &position_imu_in_vicon,
                                Eigen::MatrixXd &dz_dI,
                                Eigen::MatrixXd &dz_dcalib);

  /// feed vicon measurement
  void feed_measurement(const ViconData &data);

  /// try update
  void try_update();

  /// chi stat
  std::vector<std::shared_ptr<UpdaterStatistics>> Chi;

  /// measurement time history
  map<int, deque<double>> t_hist;

protected:
  /// perform update
  bool update(ViconData m);

  /// stack of measurements
  std::vector<ViconData> data_stack;

  /// state
  shared_ptr<State> state;
};

} // namespace mins

#endif // MINS_UPDATERVICON_H

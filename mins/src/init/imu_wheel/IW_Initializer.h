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

#ifndef MINS_IW_INITIALIZER_H
#define MINS_IW_INITIALIZER_H

#include <Eigen/Eigen>
#include <memory>
#include <vector>

using namespace Eigen;
using namespace std;

namespace ov_core {
class ImuData;
}

namespace ov_type {
class PoseJPL;
class Vec;
} // namespace ov_type

namespace mins {
class Simulator;
class Propagator;
class UpdaterWheel;

class IW_Initializer {
public:
  /// Option struct for IW initialization
  struct IW_Initializer_Options {
    shared_ptr<ov_type::PoseJPL> wheel_extrinsic;
    shared_ptr<ov_type::Vec> wheel_dt;
    shared_ptr<ov_type::Vec> wheel_intrinsic;
    string wheel_type;
    double threshold;
    Vector3d gravity;
    bool imu_gravity_aligned;
  };

  /// IMU-Wheel based initializer
  IW_Initializer(shared_ptr<IW_Initializer_Options> op, shared_ptr<Propagator> imu_pp, shared_ptr<UpdaterWheel> wheel_up);

  /// Try initialization
  bool initialization(Matrix<double, 17, 1> &imustate);

private:
  /// Static initialization using steady-motion constraint
  bool static_initialization(VectorXd &init, vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);

  /// Dynamic initialization using wheel and IMU 1 order integration
  bool dynamic_initialization(VectorXd &init, vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);

  /// Get IMU and wheel measurements within common window
  bool get_IMU_Wheel_data(vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);

  /// Compute IMU bg by comparing with wheel measurements
  bool init_bg_interpolate_imu(Vector3d &bg, vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);

  /// Compute IMU linear velocity from wheel measurement
  Vector3d init_vI_from_wheel(pair<double, VectorXd> wheel_data);

  /// Compute gravity direction by simply solving linear system
  Vector3d init_gI_simple(Vector3d bg, Vector3d v_I0inI0, vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);

  /// Compute gravity direction by solving constrained problem
  bool init_gI_dongsi(Vector3d bg, Vector3d v_I0inI0, vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data, Vector3d &gravity_inI0);

  /// Compute IMU ba given all the other information
  Vector3d init_ba(Vector3d bg, Vector3d v_I0inI0, Vector3d g_inI0, vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);

  /// Compute linear velocity residual for sanity check
  VectorXd residual(Vector3d bg, Vector3d ba, Vector3d v_I0inI0, Vector3d g_inI0, vector<ov_core::ImuData> &imu_data, vector<pair<double, VectorXd>> &wheel_data);

  /// IMU orientation propagation
  static void IMU_prop_rk4(double dt, const Vector3d &w_hat1, const Vector3d &w_hat2, Vector4d &new_q);

  /// Gram-Schmidt method to recover global rotation
  static Matrix3d gram_schmidt(const Vector3d &gravity_inI);

  /// Get coefficients for solving constrained problem
  static VectorXd compute_dongsi_coeff(MatrixXd &D, const MatrixXd &d, double gravity_mag);

  /// Options for IW initialization
  shared_ptr<IW_Initializer_Options> op;

  /// IMU and wheel sensor data
  shared_ptr<Propagator> imu_pp;
  shared_ptr<UpdaterWheel> wheel_up;

  /// number of continuous initialization success
  int cnt_smooth = -1;

  /// Previous initialization value
  VectorXd prev_init;

  // wheel calibration parameters
  Matrix3d R_OtoI;
  Vector3d p_IinO;
  double toff;
  double rl;
  double rr;
  double base_length;
};

} // namespace mins

#endif // MINS_IW_INITIALIZER_H

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

#ifndef MINS_STATE_H
#define MINS_STATE_H
#include <Eigen/Eigen>
#include <memory>
#include <unordered_map>

using namespace std;
using namespace Eigen;
namespace ov_core {
class CamBase;
}
namespace ov_type {
class Type;
class IMU;
class Landmark;
class PoseJPL;
class Vec;
class PoseJPL_4DOF;
} // namespace ov_type
namespace mins {
class TimeChecker;
class Propagator;
class Simulator;
struct STAT;
struct OptionsEstimator;

/**
 * @brief State of our filter
 *
 * This state has all the current estimates for the filter.
 * This system is modeled after the MSCKF filter, thus we have a sliding window of clones.
 * We additionally have more parameters for online estimation of calibration and SLAM features.
 * We also have the covariance of the system, which should be managed using the StateHelper class.
 */
class State {

public:
  /**
   * @brief Default Constructor (will initialize variables to defaults)
   * @param options_ Options structure containing filter options
   */
  State(shared_ptr<OptionsEstimator> op, std::shared_ptr<Simulator> sim = nullptr);

  ~State() {}

  /// CPI information struct
  struct CPI {
    // Time of this information
    double t = -1;
    // CPI integration values
    double dt = -1;      // t - clone_t
    double clone_t = -1; // clone time that this relative pose info is anchored at
    Matrix3d R_I0toIk = Matrix3d::Identity();
    Vector3d alpha_I0toIk = Vector3d::Zero(); // p_IinG - pCinG
    MatrixXd Q = MatrixXd::Zero(6, 6);        // the uncertainty of this relative pose
    // velocities and biases
    Vector3d w = Vector3d::Zero();
    Vector3d v = Vector3d::Zero();
    Vector3d bg = Vector3d::Zero();
    Vector3d ba = Vector3d::Zero();
  };

  /// The oldest keyframe time. Mostly for GPS
  double oldest_keyframe_time();

  /// The oldest none keyframe clone time
  double oldest_clone_time();

  /// The second oldest none keyframe clone time
  double oldest_2nd_clone_time();

  /// The third oldest none keyframe clone time
  double oldest_3rd_clone_time();

  /// The newest clone time
  double newest_clone_time();

  /// The second newest clone time
  double newest_2nd_clone_time();

  /// clone time closest to t_given
  bool closest_clone_time(double t_given, double &t);

  /// clone time closest to t_given but not IMU
  bool closest_clone_time_not_imu(double t_given, double &t);

  /// clone time closest to t_given but older
  bool closest_older_clone_time(double t_given, double &t);

  /// clone time closest to t_given but newer
  bool closest_newer_clone_time(double t_given, double &t);

  /// two clone times that bounds t_given
  bool bounding_times(double t_given, double &t0, double &t1);

  /// n bounding poses and time around t_given
  bool bounding_poses_n(size_t n_order, vector<shared_ptr<ov_type::PoseJPL>> &poses, vector<double> &times, double t_given);

  /// checking availability to call [bounding_poses_n] function
  bool bounding_poses_n_check(size_t n_order, double meas_t);

  /// return interpolated pose at t_given using polynomial or imu integration (CPI)
  bool get_interpolated_pose(double t_given, Matrix3d &RGtoI, Vector3d &pIinG);

  /// return interpolated pose at t_given using polynomial
  bool get_interpolated_pose_poly(double t_given, Matrix3d &RGtoI, Vector3d &pIinG);

  /// return interpolated pose at t_given using linear interpolation
  bool get_interpolated_pose_linear(double t_given, Matrix3d &RGtoI, Vector3d &pIinG);

  /// return interpolated pose at t_given using imu integration (CPI)
  bool get_interpolated_pose_imu(double t_given, Matrix3d &RGtoI, Vector3d &pIinG);

  /// return interpolated pose and jacobian
  bool get_interpolated_jacobian(double t_given, Matrix3d &RGtoI, Vector3d &pIinG, string sensor, int id, vector<MatrixXd> &dTdx, vector<shared_ptr<ov_type::Type>> &order);

  /// noise model for interpolation
  MatrixXd intr_pose_cov(int hz, int order);
  double intr_ori_cov(int hz, int order);
  double intr_pos_cov(int hz, int order);
  double intr_ori_std(int hz, int order);
  double intr_pos_std(int hz, int order);

  /// Check if we have polynomial for interpolation
  bool have_polynomial();

  /// Returns 3d SLAM features in the global frame
  vector<Vector3d> get_features_SLAM();

  /// Build polynomial data with current IMU clones. fej: boolean for build fej or est polynomial
  void build_polynomial_data(bool fej);

  /// Append both fej and est polynomial information with current IMU clones
  void add_polynomial();

  /// Append fej or est polynomial information with current IMU clones
  void add_polynomial(bool fej);

  /// Print state information
  void print_info();

  /// Print polynomial information
  void print_poly_info();

  /// Print CPI information
  void print_cpi_info();

  /// return current clone window size
  double clone_window();

  /// check if we have clone at t_given
  bool have_clone(double t_given);

  /// check if we have CPI info at t_given
  bool have_cpi(double t_given);

  /// flush old data
  void flush_old_data();

  /// Connect propagater to use its function to compute CPI
  void hook_propagator(shared_ptr<Propagator> p_ptr);

  bool initialized = false;

  /// Current timestamp (should be the last update time!)
  double time = -1;

  // Startup time of the filter
  double startup_time = -1;

  /// Struct containing filter options
  shared_ptr<OptionsEstimator> op;

  /// Pointer to the "active" IMU state (q_GtoI, p_IinG, v_IinG, bg, ba)
  shared_ptr<ov_type::IMU> imu;

  /// Map between imaging times and clone poses (q_GtoIk, p_IkinG)
  map<double, shared_ptr<ov_type::PoseJPL>> clones;

  /// Timestamp of Keyframes. Only for GNSS initialization and will not be marginalized until initialized.
  vector<double> keyframes;

  /// Timestamp of clones that we will convert to keyframes.
  vector<double> keyframes_candidate;

  /// Our current set of SLAM features (3d positions)
  unordered_map<size_t, shared_ptr<ov_type::Landmark>> cam_SLAM_features;

  /// Time offset base IMU to camera (t_imu = t_cam + t_off)
  unordered_map<size_t, shared_ptr<ov_type::Vec>> cam_dt;

  /// Calibration poses for each camera (R_ItoC, p_IinC)
  unordered_map<size_t, shared_ptr<ov_type::PoseJPL>> cam_extrinsic;

  /// Camera intrinsics
  unordered_map<size_t, shared_ptr<ov_type::Vec>> cam_intrinsic;

  /// Camera intrinsics camera objects
  unordered_map<size_t, shared_ptr<ov_core::CamBase>> cam_intrinsic_model;

  /// Time offset base IMU to vicon (t_imu = t_vicon + t_off)
  unordered_map<size_t, shared_ptr<ov_type::Vec>> vicon_dt;

  /// Calibration poses for each vicon (R_ItoV, p_IinV)
  unordered_map<size_t, shared_ptr<ov_type::PoseJPL>> vicon_extrinsic;

  /// Time offset base IMU to gps (t_imu = t_gps + t_off)
  unordered_map<size_t, shared_ptr<ov_type::Vec>> gps_dt;

  /// Calibration poses for each gps (p_GinI)
  unordered_map<size_t, shared_ptr<ov_type::Vec>> gps_extrinsic;

  /// Time offset base IMU to lidar (t_imu = t_lidar + t_off)
  unordered_map<size_t, shared_ptr<ov_type::Vec>> lidar_dt;

  /// Calibration poses for each lidar (R_ItoL, p_IinL)
  unordered_map<size_t, shared_ptr<ov_type::PoseJPL>> lidar_extrinsic;

  /// Time offset base IMU to wheel (t_imu = t_wheel + t_off)
  shared_ptr<ov_type::Vec> wheel_dt;

  /// Calibration pose for wheel (R_ItoO, p_IinO)
  shared_ptr<ov_type::PoseJPL> wheel_extrinsic;

  /// Wheel intrinsics (left wheel radius, right wheel radius, baseline length)
  shared_ptr<ov_type::Vec> wheel_intrinsic;

  /// Calibration poses of VIO "world frame" to the GPS ENU datum (R_WtoE, p_WinE)
  shared_ptr<ov_type::PoseJPL_4DOF> trans_WtoE;

  /// current estimated magnitude of linear and angular accelerations
  shared_ptr<STAT> est_a;
  shared_ptr<STAT> est_A;

  /// CPI information
  map<double, CPI> cpis;

  /// Covariance of all active variables
  MatrixXd cov;

  /// Timeing record
  std::shared_ptr<TimeChecker> tc;

  /// For easy debugging
  std::shared_ptr<Simulator> sim;

private:
  // Define that the state helper is a friend class of this class
  // This will allow it to access the below functions which should normally not be called
  // This prevents a developer from thinking that the "insert clone" will actually correctly add it to the covariance
  friend class StateHelper;

  /// Saved polynomial interpolated Jacobian info
  struct INTR_JCB_INFO {
    double t;
    Matrix3d RGtoI;
    Vector3d pIinG;
    vector<MatrixXd> dTdx;
    vector<double> times;
    vector<shared_ptr<ov_type::Type>> order;
  };

  /// Saved jacobian interpolation info to save computation
  unordered_map<string, unordered_map<int, map<double, INTR_JCB_INFO>>> intr_jcb;

  /// polynomial data
  struct polynomial_data {
    VectorXd diff_vec_ori;
    VectorXd diff_vec_pos;
    MatrixXd V_t_inv;
    VectorXd coeffs_ori;
    VectorXd coeffs_pos;
    MatrixXd dbth_dth0;
    MatrixXd dbp_dp0;
    vector<MatrixXd> JlinOtoiInv;
  };

  /// polynomial to compute interpolation using best estimate or FEJ values
  map<double, polynomial_data> _polynomial_est, _polynomial_fej;

  /// Init sensor states
  void set_camera_state(int &current_id);
  void set_vicon_state(int &current_id);
  void set_gps_state(int &current_id);
  void set_wheel_state(int &current_id);
  void set_lidar_state(int &current_id);

  /// Init state covariance
  void set_state_covariance(int &current_id);

  /// Check polynomial health
  bool check_polynomial(double t_given);

  /// Check if we have saved jacobian information at t_given
  bool have_jacobian(double t_given, string sensor, int id);

  /// Create a new cpi information with linear interpolation
  bool create_new_cpi_linear(double t_given);

  /// Create a new cpi information with integration
  bool create_new_cpi_integrate(double t_given);

  /// Vector of variables
  vector<shared_ptr<ov_type::Type>> variables;

  /// imu propagator for CPI computation
  std::shared_ptr<Propagator> prop;
};

} // namespace mins

#endif // MINS_STATE_H

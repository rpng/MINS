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

#ifndef MINS_SIMULATOR_H
#define MINS_SIMULATOR_H

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <memory>
#include <random>
#include <unordered_map>
#include <vector>

namespace ov_core {
struct ImuData;
}

namespace pcl {
class PointXYZ;
template <class pointT> class PointCloud;
} // namespace pcl

using namespace Eigen;
namespace mins {
struct Options;
struct OptionsLidar;
struct CamSimData;
struct ViconData;
struct GPSData;
struct WheelData;
struct SimulationPlane;
class ConstBsplineSE3;

/**
 * @brief Master simulator class that generated visual-inertial measurements
 *
 * Given a trajectory this will generate a SE(3) @ref ov_core::BsplineSE3 for that trajectory.
 * This allows us to get the inertial measurement information at each timestep during this trajectory.
 * After creating the bspline we will generate an environmental feature map which will be used as our feature measurements.
 * This map will be projected into the frame at each timestep to get our "raw" uv measurements.
 * We inject bias and white noises into our inertial readings while adding our white noise to the uv measurements also.
 * The user should specify the sensor rates that they desire along with the seeds of the random number generators.
 *
 */
class Simulator {

public:
  /**
   * @brief Default constructor, will load all configuration variables
   * @param params_ VioManager parameters. Should have already been loaded from cmd.
   */
  Simulator(std::shared_ptr<Options> op);

  ~Simulator(){};

  /**
   * @brief Returns if we are actively simulating
   * @return True if we still have simulation data
   */
  bool ok() { return is_running; }

  /**
   * @brief Gets the next simulated imu measurement if it is it's turn.
   * @param imu ov_core::ImuData
   * @return True if we have a measurement
   */
  bool get_next_imu(ov_core::ImuData &imu);

  /**
   * @brief Gets the next simulated camera measurement if it is it's turn.
   * @param cam mins::CamSimData
   * @return True if we have a measurement
   */
  bool get_next_cam(CamSimData &cam);

  /**
   * @brief Gets the next simulated vicon measurement if it is it's turn.
   * @param vicon mins::ViconData
   * @return True if we have a measurement
   */
  bool get_next_vicon(ViconData &vicon);

  /**
   * @brief Gets the next simulated gps measurement if it is it's turn.
   * @param gps mins::GPSData
   * @return True if we have a measurement
   */
  bool get_next_gps(GPSData &gps);

  /**
   * @brief Gets the next simulated wheel measurement if it is it's turn.
   * @param wheel mins::WheelData
   * @return True if we have a measurement
   */
  bool get_next_wheel(WheelData &wheel);

  /**
   * @brief Gets the next simulated lidar measurement if it is it's turn.
   * @param lidar mins::LidarData
   * @return True if we have a measurement
   */
  bool get_next_lidar(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar);

  /// boolean for transforming groundtruth after GPS initialization
  bool trans_gt_to_ENU = false;

protected:
  friend class Initializer;
  friend class SimVisualizer;
  friend class Sim2Visualizer;
  friend class State_Logger;

  /// Get LiDAR plane info
  std::vector<std::shared_ptr<SimulationPlane>> get_lidar_planes();

  /// Returns the true 3d map of camera features
  std::vector<Vector3d> get_cam_map_vec();

  /// Returns the true 3d map of camera features
  std::unordered_map<size_t, Vector3d> get_cam_map();

  /// Returns RMSE and NEES of IMU pose (ori rmse, pos rmse, ori nees, pos nees)
  Vector4d imu_rmse_nees(double time, Matrix<double, 7, 1> imu, Matrix<double, 6, 6> cov);

  /// a wrapper function of spline that returns IMU pose & velocities & accelerations
  bool get_imu_acceleration(double timestamp, Matrix3d &R_GtoI, Vector3d &p_IinG, Vector3d &w_IinI, Vector3d &v_IinG, Vector3d &alpha_IinI, Vector3d &a_IinG);

  /// a wrapper function of spline that returns IMU pose & velocities
  bool get_imu_velocity(double timestamp, Matrix3d &R_GtoI, Vector3d &p_IinG, Vector3d &w_IinI, Vector3d &v_IinG);

  /// a wrapper function of spline that returns IMU pose
  bool get_imu_pose(double timestamp, Matrix3d &R_GtoI, Vector3d &p_IinG);

  /**
   * @brief Get the simulation state at a specified timestep
   * @param timestamp Timestamp we want to get the state at
   * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
   * @return True if we have a state
   */
  bool get_imu_state(double timestamp, Matrix<double, 17, 1> &imustate);

  /**
   * @brief Will get a set of perturbed parameters
   * @param gen_state Random number gen to use
   * @param op_ Parameters we will perturb
   */
  void perturb_calibration();

  /**
   * @brief Projects the passed map features into the desired camera frame.
   * @param R_GtoI Orientation of the IMU pose
   * @param p_IinG Position of the IMU pose
   * @param camid Camera id of the camera sensor we want to project into
   * @param feats Our set of 3d features
   * @return True distorted raw image measurements and their ids for the specified camera
   */
  std::vector<std::pair<size_t, VectorXf>> project_pointcloud(const Matrix3d &R_GtoI, const Vector3d &p_IinG, int camid);

  /// create synthetic camera frames and ensure that each has enough features
  void create_camera_features();

  /// load plane information from file for LiDAR measurements
  bool load_plane_data(std::string path_planes);

  /// Generate LiDAR pointcloud
  bool get_lidar_pointcloud(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar, double time, int id, std::shared_ptr<OptionsLidar> lidar_op);

  /**
   * @brief Will generate points in the fov of the specified camera
   * @param R_GtoI Orientation of the IMU pose
   * @param p_IinG Position of the IMU pose
   * @param camid Camera id of the camera sensor we want to project into
   * @param[out] feats Map we will append new features to
   * @param numpts Number of points we should generate
   */
  void generate_points(const Matrix3d &R_GtoI, const Vector3d &p_IinG, int camid, std::unordered_map<size_t, Vector3d> &feats, int numpts);

  /// This will generate planes given the trajectory
  void generate_planes();

  /// Find which simulated sensor measurement comes next
  void sim_turn(std::string &sensor_type, int &sensor_indx);

  /// Our loaded trajectory data (timestamp(s), q_GtoI, p_IinG)
  std::vector<Eigen::VectorXd> traj_data;

  /// Our b-spline that can create nonholonomic trajectory
  std::shared_ptr<ConstBsplineSE3> spline;

  /// Mersenne twister PRNG for measurements
  std::mt19937 seed_imu;
  std::mt19937 seed_wheel;
  std::mt19937 seed_init;
  std::vector<std::mt19937> seed_cams;
  std::vector<std::mt19937> seed_vicons;
  std::vector<std::mt19937> seed_lidars;
  std::vector<std::mt19937> seed_gps;

  /// Mersenne twister PRNG for state perturbations
  std::mt19937 seed_ptrb;

  /// If our simulation is running
  bool is_running;

  /// Current timestamp of the system
  double timestamp;

  /// All the options about this system
  std::shared_ptr<Options> op;

  /// Last sensor simulation times
  double timestamp_last_imu;
  double timestamp_last_wheel;
  std::vector<double> timestamp_last_cams;
  std::vector<double> timestamp_last_vicons;
  std::vector<double> timestamp_last_lidars;
  std::vector<double> timestamp_last_gpss;

  /// Our running acceleration gyroscope, bias
  Vector3d true_imu_ba, true_imu_bg;

  /// Our history of true biases
  std::vector<double> bias_times;
  std::vector<Vector3d> vec_ba, vec_bg;

  /// Our camera map of 3d features
  size_t cam_feat_id = 0;
  std::unordered_map<size_t, Vector3d> cam_featmap;

  /// Vector of 3d environmental planes that we will intercept our lidar with
  std::vector<std::shared_ptr<SimulationPlane>> lidar_planes;
};

} // namespace mins

#endif // MINS_SIMULATOR_H

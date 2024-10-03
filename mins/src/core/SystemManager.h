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

#ifndef MINS_SYSTEMMANAGER_H
#define MINS_SYSTEMMANAGER_H

#include <Eigen/Eigen>
#include <boost/shared_ptr.hpp>
#include <memory>

namespace pcl {
class PointXYZ;
template <class pointT> class PointCloud;
} // namespace pcl

namespace ov_core {
class ImuData;
class CameraData;
} // namespace ov_core

namespace mins {
struct OptionsEstimator;
class Initializer;
class Propagator;
class State;
class Simulator;
struct ViconData;
struct CamSimData;
struct GPSData;
struct STAT;
struct WheelData;
struct RoverWheelData;
struct TLIOData;
class UpdaterCamera;
class UpdaterGPS;
class UpdaterLidar;
class UpdaterVicon;
class UpdaterWheel;
class UpdaterRoverWheel;
class UpdaterTLIO;
class TimeChecker;

class SystemManager {

public:
  /// Multi-sensor system handles various sensors and estimation strategies
  SystemManager(std::shared_ptr<OptionsEstimator> op, std::shared_ptr<Simulator> sim = nullptr);

  ~SystemManager(){};

  void init();

  /// IMU measurement feeder
  bool feed_measurement_imu(const ov_core::ImuData &imu);

  /// VICON measurement feeder
  void feed_measurement_vicon(const ViconData &vicon);

  /// CAM (real) measurement feeder
  void feed_measurement_camera(const ov_core::CameraData &cam);

  /// CAM (simulation) measurement feeder
  void feed_measurement_camsim(const CamSimData &cam);

  /// GPS measurement feeder
  void feed_measurement_gps(GPSData gps, bool isGeodetic);

  /// Wheel measurement feeder
  void feed_measurement_wheel(const WheelData &wheel);

  void feed_measurement_rover(const RoverWheelData &wheel);

  void feed_measurement_tlio(const TLIOData &tlio);

  /// LiDAR measurement feeder
  void feed_measurement_lidar(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar);
  /**
   * @brief After the run has ended, print results
   */
  void visualize_final();

  std::shared_ptr<OptionsEstimator> op;
  std::shared_ptr<Simulator> sim;

  /// Our master state object :D
  std::shared_ptr<State> state;

  /// Propagator of our state
  std::shared_ptr<Propagator> prop;

  /// GPS updater
  std::shared_ptr<UpdaterGPS> up_gps;

  /// LiDAR updater
  std::shared_ptr<UpdaterLidar> up_ldr;

  /// Camera updater
  std::shared_ptr<UpdaterCamera> up_cam;

  /// State initializer
  std::shared_ptr<Initializer> initializer;

  /// GPS datum setup
  Eigen::Vector3d gps_datum = Eigen::Vector3d::Ones() * NAN;

  /// Timing recorder for analysis
  std::shared_ptr<TimeChecker> tc_sensors;

protected:
  /// Determine next clone time
  bool get_next_clone_time(double &clone_time, double meas_t);

  /// Based on ang/lin acceleration, dynamically change cloning Hz and interpolation order
  void dynamic_cloning(int &clone_freq, int &intr_order);

  /// Compute angular and linear accelerations used for dynamic cloning
  void compute_accelerations();

  /// Nice print of the state and calibration results
  void print_status();

  /// VICON updater
  std::shared_ptr<UpdaterVicon> up_vcn;

  /// Wheel updater
  std::shared_ptr<UpdaterWheel> up_whl;
  std::shared_ptr<UpdaterRoverWheel> up_whl_rover;
  std::shared_ptr<UpdaterTLIO> up_tlio;

  /// Average order and cloning frequency of the system
  std::shared_ptr<STAT> avg_order, avg_freq;

  /// Total distance traveled
  double distance = 0;
};
} // namespace mins

#endif // MINS_SYSTEMMANAGER_H

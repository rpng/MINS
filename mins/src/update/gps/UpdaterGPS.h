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

#ifndef MINS_UPDATERGPS_H
#define MINS_UPDATERGPS_H

#include <Eigen/Eigen>
#include <deque>
#include <memory>
#include <vector>

using namespace std;
using namespace Eigen;
namespace ov_type {
class Type;
}
namespace mins {
class State;
class UpdaterStatistics;
struct GPSData;
typedef vector<shared_ptr<ov_type::Type>> VEC_TYPE;
class UpdaterGPS {

public:
  /// GNSS updater
  UpdaterGPS(shared_ptr<State> state);

  /// get gps data
  void feed_measurement(const GPSData &data);

  /// find available gps data and try update
  void try_update();

  /// add keyframes at gps measurement time
  void add_keyframes(GPSData &data);

  /// status of GPS being initialized
  bool initialized = false;

  /// chi status
  vector<shared_ptr<UpdaterStatistics>> Chi;

  /// measurement timing
  map<int, deque<double>> t_hist;

protected:
  friend class Initializer;

  /// try initialization
  bool try_initialization();

  /// get T_WtoE initialial guess for initialization
  bool get_initial_guess(vector<GPSData> data_init, Matrix3d &RWtoE, Vector3d &pWinE);

  /// construct linear system for delayed initialization
  void construct_init_linsys(vector<GPSData> data_init, Matrix3d RWtoE, Vector3d pWinE, MatrixXd &Hx, MatrixXd &Hi, MatrixXd &R, VectorXd &res, VEC_TYPE &x_order);

  /// transform the state to ENU coordinate
  void transform_state_to_ENU();

  /// perform EKF update with GNSS measurement
  bool update(GPSData data);

  /// Stack of gnss measurements
  vector<GPSData> data_stack;

  /// State
  shared_ptr<State> state;
};
} // namespace mins
#endif // MINS_UPDATERGPS_H

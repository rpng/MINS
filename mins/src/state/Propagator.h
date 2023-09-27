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

#ifndef MINS_STATE_PROPAGATOR_H
#define MINS_STATE_PROPAGATOR_H

#include <Eigen/Eigen>
#include <memory>
#include <queue>

using namespace Eigen;
using namespace std;

namespace ov_core {
class CpiV1;
struct ImuData;
} // namespace ov_core

namespace ov_type {
class IMU;
class Type;
} // namespace ov_type

namespace mins {
class State;
class Propagator {
  typedef Matrix<double, 15, 15> Matrix15;

public:
  /**
   * @brief Default constructor
   */
  Propagator(shared_ptr<State> state);
  /**
   * @brief Stores incoming inertial readings
   * @param message Contains our timestamp and inertial information
   */
  void feed_imu(const ov_core::ImuData &message);

  /**
   * @brief Helper function that given current IMU data, will select IMU readings between the two times.
   *
   * This will create measurements that we will integrate with, and an extra measurement at the end.
   * We use the @ref interpolate_data() function to "cut" the IMU readings at the begining and end of the integration.
   * The timestamps passed should already take into account the timeoffset values.
   *
   * @param imu_data IMU data we will select measurements from
   * @param time0 Start timestamp
   * @param time1 End timestamp
   * @param warn If we should warn if we don't have enough IMU to propagate with (e.g. fast get_propagator will get warnings otherwise)
   * @return Vector of measurements (if we could compute them)
   */
  bool select_imu_readings(double time0, double time1, vector<ov_core::ImuData> &data_vec);

  /// Given t_given, find two bounding imu data
  bool get_bounding_data(double t_given, vector<ov_core::ImuData> &data_stack, ov_core::ImuData &data1, ov_core::ImuData &data2);

  /**
   * @brief Nice helper function that will linearly interpolate between two IMU messages.
   *
   * This should be used instead of just "cutting" IMU messages that bound the camera times
   * Give better timeoffset if we use this function, could try other orders/splines if the IMU is slow.
   *
   * @param imu_1 IMU at begining of interpolation interval
   * @param imu_2 IMU at end of interpolation interval
   * @param timestamp Timestamp being interpolated to
   */
  static ov_core::ImuData interpolate_data(const ov_core::ImuData &imu_1, const ov_core::ImuData &imu_2, double timestamp);

  /// Propagate the state
  void propagate(double timestamp);

  /// reset CPI class and clone info at given time
  void reset_cpi(double clone_t);

  /// message time history
  deque<double> t_hist;

  /// Our history of IMU messages (time, angular velocity, linear acceleration)
  vector<ov_core::ImuData> imu_data;

protected:
  /**
   * @brief Propagates the state forward using the IMU data and computes the noise covariance and state-transition
   * matrix of this interval.
   *
   * This function can be replaced with analytical/numerical integration or when using a different state representation.
   * This contains our state transition matrix along with how our noise evolves in time.
   * If you have other state variables besides the IMU that evolve you would add them here.
   * See the @ref error_prop page for details on how this was derived.
   *
   * @param state Pointer to state
   * @param data_minus IMU readings at beginning of interval
   * @param data_plus IMU readings at end of interval
   * @param F State-transition matrix over the interval
   * @param Qd Discrete-time noise covariance over the interval
   */
  void predict_and_compute(const ov_core::ImuData &data_minus, const ov_core::ImuData &data_plus, Matrix15 &F, Matrix15 &Qd);

  /**
   * @brief RK4 IMU mean propagation.
   *
   * See this wikipedia page on [Runge-Kutta Methods](https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods).
   * We are doing a RK4 method, [this wolframe page](http://mathworld.wolfram.com/Runge-KuttaMethod.html) has the forth order equation
   * defined below. We define function \f$ f(t,y) \f$ where y is a function of time t, see @ref imu_kinematic for the definition of the
   * continous-time functions.
   *
   * \f{align*}{
   * {k_1} &= f({t_0}, y_0) \Delta t  \\
   * {k_2} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_1} ) \Delta t \\
   * {k_3} &= f( {t_0}+{\Delta t \over 2}, y_0 + {1 \over 2}{k_2} ) \Delta t \\
   * {k_4} &= f( {t_0} + {\Delta t}, y_0 + {k_3} ) \Delta t \\
   * y_{0+\Delta t} &= y_0 + \left( {{1 \over 6}{k_1} + {1 \over 3}{k_2} + {1 \over 3}{k_3} + {1 \over 6}{k_4}} \right)
   * \f}
   *
   * @param state Pointer to state
   * @param dt Time we should integrate over
   * @param w_hat1 Angular velocity with bias removed
   * @param a_hat1 Linear acceleration with bias removed
   * @param w_hat2 Next angular velocity with bias removed
   * @param a_hat2 Next linear acceleration with bias removed
   * @param new_q The resulting new orientation after integration
   * @param new_v The resulting new velocity after integration
   * @param new_p The resulting new position after integration
   */
  void predict_mean_rk4(shared_ptr<ov_type::IMU> imu, double dt, const Vector3d &w_hat1, const Vector3d &a_hat1, const Vector3d &w_hat2, const Vector3d &a_hat2, Vector4d &new_q,
                        Vector3d &new_v, Vector3d &new_p);

  /// state
  shared_ptr<State> state;

  /// CPI: Continuous time preintegration
  shared_ptr<ov_core::CpiV1> cpiv1;
  double cpi_clone_t;
};

} // namespace mins
#endif // MINS_STATE_PROPAGATOR_H

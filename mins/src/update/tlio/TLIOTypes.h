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

#ifndef MINS_ODOMTYPES_H
#define MINS_ODOMTYPES_H

#include <Eigen/Eigen>

namespace mins {
struct TLIOData {

  /// Timestamp of the reading
  double time;

  // Position update, first 3 entrys are orientation, last 3 position
  Eigen::Vector<double, 6> meas;

  /// covariance estimate
  Eigen::Matrix<double, 6, 6> cov;

  int id = 0;

  /// Sort function to allow for using of STL containers
  bool operator<(const TLIOData &other) const { return time < other.time; }
};
} // namespace mins

#endif // MINS_ODOMTYPES_H

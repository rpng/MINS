/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_VICONTYPES_H
#define MINS_VICONTYPES_H

#include <Eigen/Eigen>

namespace mins {
struct ViconData {

  /// Timestamp of the reading
  double time;

  /// ID of the vicon
  int id;

  /// Pose measurement. First 3x1 is orientation, last 3x1 is position measurement
  Eigen::Matrix<double, 6, 1> pose;

  /// Sort function to allow for using of STL containers
  bool operator<(const ViconData &other) const { return time < other.time; }
};
} // namespace mins

#endif // MINS_VICONTYPES_H

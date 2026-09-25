/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_GPSTYPES_H
#define MINS_GPSTYPES_H

#include <Eigen/Eigen>
namespace mins {
struct GPSData {

  /// Timestamp of the reading
  double time;

  /// ID of the sensor
  int id;

  /// Position measurement.
  Eigen::Vector3d meas;

  /// Measurement noise
  Eigen::Vector3d noise;

  /// "equal" check operator
  bool operator==(const GPSData &b) {
    if (time == b.time && id == b.id && meas == b.meas && noise == b.noise)
      return true;
    return false;
  }

  /// Sort function to allow for using of STL containers
  bool operator<(const GPSData &other) const { return time < other.time; }
};
} // namespace mins
#endif // MINS_GPSTYPES_H

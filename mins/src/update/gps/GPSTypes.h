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
#include <algorithm>
#include <cmath>
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

/**
 * \brief Measurement noise (standard deviation) from a NavSatFix position covariance, whose diagonal holds variances.
 *
 * A negative or NaN variance gives 0, which every caller replaces with the configured noise.
 *
 * \param[in] covariance Row-major 3x3 position covariance in m^2.
 * \return East, north and up standard deviations in m.
 */
inline Eigen::Vector3d gps_noise_from_covariance(const double *covariance) {
  return Eigen::Vector3d(std::sqrt(std::max(0.0, covariance[0])), std::sqrt(std::max(0.0, covariance[4])), std::sqrt(std::max(0.0, covariance[8])));
}
} // namespace mins
#endif // MINS_GPSTYPES_H

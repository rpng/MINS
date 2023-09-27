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

#ifndef MINS_WHEELTYPES_H
#define MINS_WHEELTYPES_H

namespace mins {

/// Types of wheel measurements supported
/// Wheel2DAng: left/right wheel angular velocities. Additionally supports intrinsic (left/right wheel radii & base length) calibration
/// Wheel2DLin: left/right wheel linear velocities.
/// Wheel2DCen: angular/linear wheel velocities of the wheel odometry frame.
/// Wheel3DAng: left/right wheel angular velocities + planar motion constraint. Additionally supports intrinsic calibration
/// Wheel3DLin: left/right wheel linear velocities + planar motion constraint.
/// Wheel3DCen: angular/linear wheel velocities of the wheel odometry frame + planar motion constraint.

struct WheelData {

  /// Timestamp of the reading
  double time = -1;

  /// Sensor reading 1 (left wheel reading or angular velocity)
  double m1 = 0;

  /// Sensor reading 2 (right wheel reading or linear velocity)
  double m2 = 0;

  /// Sort function to allow for using of STL containers
  bool operator<(const WheelData &other) const { return time < other.time; }
};
} // namespace mins

#endif // MINS_WHEELTYPES_H

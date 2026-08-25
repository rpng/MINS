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

#include <string>

namespace mins {

/// What the two wheel readings actually measure.
enum class WheelModality {
  Angular,  ///< Left and right wheel angular velocities. The only modality supporting intrinsic calibration.
  Linear,   ///< Left and right wheel linear velocities.
  Centered, ///< Angular and linear velocity of the wheel odometry frame itself.
};

/// Wheel measurement model. Each 3D variant adds a planar motion constraint to its 2D counterpart.
enum class WheelType {
  Wheel2DAng,
  Wheel2DLin,
  Wheel2DCen,
  Wheel3DAng,
  Wheel3DLin,
  Wheel3DCen,
};

/**
 * \brief Name of a wheel type, spelled as it appears in the config yaml.
 * \param[in] type Wheel type to name.
 * \return Type name, or an empty string for an out-of-range value.
 */
inline const char *ToString(WheelType type) {
  switch (type) {
  case WheelType::Wheel2DAng:
    return "Wheel2DAng";
  case WheelType::Wheel2DLin:
    return "Wheel2DLin";
  case WheelType::Wheel2DCen:
    return "Wheel2DCen";
  case WheelType::Wheel3DAng:
    return "Wheel3DAng";
  case WheelType::Wheel3DLin:
    return "Wheel3DLin";
  case WheelType::Wheel3DCen:
    return "Wheel3DCen";
  }
  return "";
}

/// Every supported wheel type, in declaration order.
static constexpr WheelType ALL_WHEEL_TYPES[] = {WheelType::Wheel2DAng, WheelType::Wheel2DLin, WheelType::Wheel2DCen,
                                                WheelType::Wheel3DAng, WheelType::Wheel3DLin, WheelType::Wheel3DCen};

/**
 * \brief Resolve a config yaml type name to a wheel type.
 * \param[in] name Type name to resolve.
 * \param[out] type Resolved type, left untouched when the name is not supported.
 * \return True when the name names a supported type.
 */
inline bool ParseWheelType(const std::string &name, WheelType &type) {
  for (WheelType candidate : ALL_WHEEL_TYPES) {
    if (name == ToString(candidate)) {
      type = candidate;
      return true;
    }
  }
  return false;
}

/**
 * \brief Whether a type carries the planar motion constraint.
 * \param[in] type Wheel type to test.
 * \return True for the 3D variants.
 */
inline bool IsWheel3D(WheelType type) { return type == WheelType::Wheel3DAng || type == WheelType::Wheel3DLin || type == WheelType::Wheel3DCen; }

/**
 * \brief What the readings of a given type measure, ignoring its 2D/3D flavor.
 * \param[in] type Wheel type to inspect.
 * \return Modality of the type.
 */
inline WheelModality ModalityOf(WheelType type) {
  switch (type) {
  case WheelType::Wheel2DAng:
  case WheelType::Wheel3DAng:
    return WheelModality::Angular;
  case WheelType::Wheel2DLin:
  case WheelType::Wheel3DLin:
    return WheelModality::Linear;
  case WheelType::Wheel2DCen:
  case WheelType::Wheel3DCen:
    return WheelModality::Centered;
  }
  return WheelModality::Angular;
}

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

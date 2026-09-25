/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023-2026 Woosik Lee
 * Copyright (C) 2023-2026 Guoquan Huang
 * Copyright (C) 2023-2026 MINS Contributors
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

#ifndef MINS_PACKAGE_PATH_H
#define MINS_PACKAGE_PATH_H

#include <string>

namespace mins {
// Resolve an installed package's directory. Defined at the ROS boundary
// (PackagePath.cpp) so the shared core stays free of ros::package / ament.
std::string get_package_path(const std::string &pkg);
} // namespace mins

#endif // MINS_PACKAGE_PATH_H

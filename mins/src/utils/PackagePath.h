/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
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

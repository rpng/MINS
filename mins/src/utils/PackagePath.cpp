/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// This is the only translation unit that touches the ROS package-path API,
// keeping ros::package / ament out of the shared core.
#include "utils/PackagePath.h"
#include <cstdlib>
#if ROS_AVAILABLE == 1
#include <ros/package.h>
#elif ROS_AVAILABLE == 2
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

std::string mins::get_package_path(const std::string &pkg) {
#if ROS_AVAILABLE == 1
  return ros::package::getPath(pkg);
#elif ROS_AVAILABLE == 2
  return ament_index_cpp::get_package_share_directory(pkg);
#else
  // No ROS package system. Resolve "<pkg>" under $MINS_ROOT (the source/repo root)
  // so the headless build can still find config + data; empty if unset.
  const char *root = std::getenv("MINS_ROOT");
  return root ? std::string(root) + "/" + pkg : std::string();
#endif
}

/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_FS_COMPAT_H
#define MINS_FS_COMPAT_H

// GCC 7 (ROS Melodic) ships filesystem only under <experimental/>. Everything newer has <filesystem>.
#if defined(__has_include) && __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#endif // MINS_FS_COMPAT_H

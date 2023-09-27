/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This code is implemented based on:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "Print_Logger.h"
#include <boost/filesystem.hpp>
#include <cstdarg>
#include <cstring>
#include <iostream>

using namespace mins;

// Need to define the static variable for everything to work
int Print_Logger::current_print_level = 2;
FILE *Print_Logger::pFile;

void Print_Logger::open_file(const std::string &path, bool remove_exist) {
  // Find unique log file name
  std::string output_path;
  for (int i = 0; i < 10000; i++) {
    output_path = path + "/mins_log" + std::to_string(i) + ".txt";
    if (boost::filesystem::exists(output_path)) {
      if (remove_exist) {
        boost::filesystem::remove(output_path);
        break;
      }
      continue;
    } else
      break;
  }
  boost::filesystem::create_directories(boost::filesystem::path(output_path.c_str()).parent_path());
  pFile = fopen(output_path.c_str(), "w");
}

void Print_Logger::close_file() {
  if (pFile != NULL)
    fclose(pFile);
}

void Print_Logger::setPrintLevel(int level) {
  if (level < 0 || level > 5) {
    PRINT4("Invalid print level requested: %d\n", level);
    PRINT4("Valid levels are: 0, 1, 2, 3, 4, 5\n");
    std::exit(EXIT_FAILURE);
  }

  Print_Logger::current_print_level = level;
  PRINT2("Setting printing level to: %d\n", level);
}

void Print_Logger::debugPrint(int level, const char location[], const char line[], const char *format, ...) {
  // log the message if file is open
  if (pFile != NULL) {
    va_list args;
    va_start(args, format);
    vfprintf(pFile, format, args);
    va_end(args);
  }

  // Only print for the current debug level
  if (level < Print_Logger::current_print_level)
    return;

  // Print the location info first for our debug output
  // Truncate the filename to the max size for the filepath
  if (Print_Logger::current_print_level < 1) {
    std::string path(location);
    std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
    if (base_filename.size() > MAX_FILE_PATH_LEGTH) {
      printf("%s", base_filename.substr(base_filename.size() - MAX_FILE_PATH_LEGTH, base_filename.size()).c_str());
    } else {
      printf("%s", base_filename.c_str());
    }
    printf(":%s ", line);
  }
  // Print the rest of the args
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
}

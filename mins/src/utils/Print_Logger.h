/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef MINS_CORE_PRINT_H
#define MINS_CORE_PRINT_H

#include <cstdint>
#include <string>

namespace mins {
/// Wrapping function of ov_core::Printer does more similar things and save log
class Print_Logger {
public:
  /// Open file for logging. WARNING: THIS CAN SLOW DOWN THE SYSTEM
  static void open_file(const std::string &path, bool remove_exist);

  /// Close logging file
  static void close_file();

  /**
   * @brief Set the print level to use for all future printing to stdout.
   * @param level The debug level to use
   */
  static void setPrintLevel(int level);

  /**
   * @brief The print function that prints to stdout.
   * @param level the print level for this print call
   * @param location the location the print was made from
   * @param line the line the print was made from
   * @param format The printf format
   */
  static void debugPrint(int level, const char location[], const char line[], const char *format, ...);

  /// Return the string verbosity of the OpenVINS equivalent to MINS verbosity
  static std::string get_ov_verbosity(int verbosity) {
    if (verbosity == 0)
      return "ALL";
    if (verbosity == 1)
      return "DEBUG";
    if (verbosity == 2)
      return "INFO";
    if (verbosity == 3)
      return "WARNING";
    if (verbosity == 4)
      return "ERROR";
    if (verbosity == 5)
      return "SILENT";
    return "Unable to get proper value";
  }

  /// The current print level
  static int current_print_level;

  /// File to save log
  static FILE *pFile;

private:
  /// The max length for the file path. This is to avoid very long file paths from
  static constexpr uint32_t MAX_FILE_PATH_LEGTH = 30;
};

} /* namespace mins */

/*
 * Converts anything to a string
 */
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

/*
 * The different Types of print levels
 */
// Get the file name directly https://stackoverflow.com/questions/8487986/file-macro-shows-full-path
#define PRINT0(...) mins::Print_Logger::debugPrint(0, __FILE__, TOSTRING(__LINE__), __VA_ARGS__)
#define PRINT1(...) mins::Print_Logger::debugPrint(1, __FILE__, TOSTRING(__LINE__), __VA_ARGS__)
#define PRINT2(...) mins::Print_Logger::debugPrint(2, __FILE__, TOSTRING(__LINE__), __VA_ARGS__)
#define PRINT3(...) mins::Print_Logger::debugPrint(3, __FILE__, TOSTRING(__LINE__), __VA_ARGS__)
#define PRINT4(...) mins::Print_Logger::debugPrint(4, __FILE__, TOSTRING(__LINE__), __VA_ARGS__)
#define PRINTLINE printf("%s:%d\n", strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__, __LINE__);
#endif /* mins_PRINT_H */

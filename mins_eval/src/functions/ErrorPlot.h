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

#ifndef MINS_EVAL_ERRORPLOT
#define MINS_EVAL_ERRORPLOT

#include <fstream>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>

#include <Eigen/Eigen>

#include "utils/Loader.h"
#include "utils/Statistics.h"

#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#ifdef HAVE_PYTHONLIBS

// import the c++ wrapper for matplot lib
// https://github.com/lava/matplotlib-cpp
// sudo apt-get install python-matplotlib python-numpy python2.7-dev
#include "plot/matplotlibcpp.h"

#endif
using namespace std;
using namespace ov_core;
using namespace Eigen;
using VEC = vector<vector<Eigen::VectorXd>>;

namespace mins_eval {
class DATA {
public:
  VEC ests;
  VEC stds;
  VEC gts;
};
/**
 * @brief A single simulation run (the full state not just pose).
 *
 * This should match the recording logic that is in the mins::RosVisualizer in which we write both estimate, their deviation, and
 * groundtruth to three files. We enforce that these files first contain the current IMU state, then timeoffset, number of cameras, then
 * the camera calibration states. If we are not performing calibration these should all be written to file, just their deviation should be
 * zero as they are 100% certain.
 */
class ErrorPlot {

public:
  /**
   * @brief Default constructor that will load our data from file
   * @param path Path to the directory with groundtruth, est, and deviations
   * @param count The number of runs we want to plot
   */
  ErrorPlot(string load_path, string save_path);

  bool load_data(string load_path, string sensor, DATA &data);
  /**
   * @brief Will plot the state error and its three sigma bounds
   * @param doplotting True if you want to display the plots
   * @param max_time Max number of second we want to plot
   */
  void plot_imu(DATA data, double max_time = INFINITY);

  /**
   * @brief Will plot the camera calibration extrinsic transform
   * @param doplotting True if you want to display the plots
   * @param max_time Max number of second we want to plot
   */
  void plot_extrinsic(DATA data, string sensor, double max_time = INFINITY);

  void plot_vector(DATA data, string sensor, vector<string> names, double max_time = INFINITY);

  // Trajectory data (loaded from file and timestamp intersected)
  DATA imu_data;
  DATA wheel_ext_data, wheel_int_data, wheel_dt_data;
  vector<DATA> vicon_ext_data, vicon_dt_data;
  vector<DATA> lidar_ext_data, lidar_dt_data;
  vector<DATA> gps_ext_data, gps_dt_data;
  vector<DATA> cam_ext_data, cam_int_data, cam_dt_data;

protected:
  // Save path for plots
  string save_path;
  // Plot color set
  vector<string> colors = {"blue", "red", "green", "magenta", "black", "yellow", "LightBlue", "Orange", "cyan", "Gray"};

  double start_time;

  int figure_id = 1;
#ifdef HAVE_PYTHONLIBS

  /**
   * @brief Plots three different statistic values and sigma bounds
   * @param sx X-axis error
   * @param sy Y-axis error
   * @param sz Z-axis error
   * @param color_err MATLAB color string for error line (blue, red, etc.)
   * @param color_std MATLAB color string for deviation (blue, red, etc.)
   */
  void plot_3errors(ov_eval::Statistics sx, ov_eval::Statistics sy, ov_eval::Statistics sz, string color_err, string color_std) {

    // Zero our time arrays
    double starttime1 = (sx.timestamps.empty()) ? 0 : sx.timestamps[0];
    double endtime1 = (sx.timestamps.empty()) ? 0 : sx.timestamps[sx.timestamps.size() - 1];
    for (size_t i = 0; i < sx.timestamps.size(); i++) {
      sx.timestamps[i] -= starttime1;
    }
    double starttime2 = (sy.timestamps.empty()) ? 0 : sy.timestamps[0];
    double endtime2 = (sy.timestamps.empty()) ? 0 : sy.timestamps[sy.timestamps.size() - 1];
    for (size_t i = 0; i < sy.timestamps.size(); i++) {
      sy.timestamps[i] -= starttime2;
    }
    double starttime3 = (sz.timestamps.empty()) ? 0 : sz.timestamps[0];
    double endtime3 = (sz.timestamps.empty()) ? 0 : sz.timestamps[sz.timestamps.size() - 1];
    for (size_t i = 0; i < sz.timestamps.size(); i++) {
      sz.timestamps[i] -= starttime3;
    }

    // Parameters that define the line styles
    map<string, string> params_value, params_bound;
    // params_value.insert({"label","error"});
    params_value.insert({"linestyle", "-"});
    params_value.insert({"color", color_err});
    // params_bound.insert({"label","3 sigma bound"});
    params_bound.insert({"linestyle", "--"});
    params_bound.insert({"color", color_std});

    // Plot our error value
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::plot(sx.timestamps, sx.values, params_value);
    if (!sx.values_bound.empty()) {
      matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
      for (size_t i = 0; i < sx.timestamps.size(); i++) {
        sx.values_bound[i] *= -1;
      }
      matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime1 - starttime1);

    // Plot our error value
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::plot(sy.timestamps, sy.values, params_value);
    if (!sy.values_bound.empty()) {
      matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
      for (size_t i = 0; i < sy.timestamps.size(); i++) {
        sy.values_bound[i] *= -1;
      }
      matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime2 - starttime2);

    // Plot our error value
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::plot(sz.timestamps, sz.values, params_value);
    if (!sz.values_bound.empty()) {
      matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
      for (size_t i = 0; i < sz.timestamps.size(); i++) {
        sz.values_bound[i] *= -1;
      }
      matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime3 - starttime3);
  }

  /**
   * @brief Plots four different statistic values and sigma bounds
   * @param sx Error one
   * @param sy Error two
   * @param sz Error three
   * @param sk Error four
   * @param color_err MATLAB color string for error line (blue, red, etc.)
   * @param color_std MATLAB color string for deviation (blue, red, etc.)
   */
  void plot_4errors(ov_eval::Statistics sx, ov_eval::Statistics sy, ov_eval::Statistics sz, ov_eval::Statistics sk, string color_err,
                    string color_std) {

    // Zero our time arrays
    double starttime1 = (sx.timestamps.empty()) ? 0 : sx.timestamps[0];
    double endtime1 = (sx.timestamps.empty()) ? 0 : sx.timestamps[sx.timestamps.size() - 1];
    for (size_t i = 0; i < sx.timestamps.size(); i++) {
      sx.timestamps[i] -= starttime1;
    }
    double starttime2 = (sy.timestamps.empty()) ? 0 : sy.timestamps[0];
    double endtime2 = (sy.timestamps.empty()) ? 0 : sy.timestamps[sy.timestamps.size() - 1];
    for (size_t i = 0; i < sy.timestamps.size(); i++) {
      sy.timestamps[i] -= starttime2;
    }
    double starttime3 = (sz.timestamps.empty()) ? 0 : sz.timestamps[0];
    double endtime3 = (sz.timestamps.empty()) ? 0 : sz.timestamps[sz.timestamps.size() - 1];
    for (size_t i = 0; i < sz.timestamps.size(); i++) {
      sz.timestamps[i] -= starttime3;
    }
    double starttime4 = (sk.timestamps.empty()) ? 0 : sk.timestamps[0];
    double endtime4 = (sk.timestamps.empty()) ? 0 : sk.timestamps[sk.timestamps.size() - 1];
    for (size_t i = 0; i < sk.timestamps.size(); i++) {
      sk.timestamps[i] -= starttime4;
    }

    // Parameters that define the line styles
    map<string, string> params_value, params_bound;
    // params_value.insert({"label","error"});
    params_value.insert({"linestyle", "-"});
    params_value.insert({"color", color_err});
    // params_bound.insert({"label","3 sigma bound"});
    params_bound.insert({"linestyle", "--"});
    params_bound.insert({"color", color_std});

    // Plot our error value
    matplotlibcpp::subplot(4, 1, 1);
    matplotlibcpp::plot(sx.timestamps, sx.values, params_value);
    if (!sx.values_bound.empty()) {
      matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
      for (size_t i = 0; i < sx.timestamps.size(); i++) {
        sx.values_bound[i] *= -1;
      }
      matplotlibcpp::plot(sx.timestamps, sx.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime1 - starttime1);

    // Plot our error value
    matplotlibcpp::subplot(4, 1, 2);
    matplotlibcpp::plot(sy.timestamps, sy.values, params_value);
    if (!sy.values_bound.empty()) {
      matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
      for (size_t i = 0; i < sy.timestamps.size(); i++) {
        sy.values_bound[i] *= -1;
      }
      matplotlibcpp::plot(sy.timestamps, sy.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime2 - starttime2);

    // Plot our error value
    matplotlibcpp::subplot(4, 1, 3);
    matplotlibcpp::plot(sz.timestamps, sz.values, params_value);
    if (!sz.values_bound.empty()) {
      matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
      for (size_t i = 0; i < sz.timestamps.size(); i++) {
        sz.values_bound[i] *= -1;
      }
      matplotlibcpp::plot(sz.timestamps, sz.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime3 - starttime3);

    // Plot our error value
    matplotlibcpp::subplot(4, 1, 4);
    matplotlibcpp::plot(sk.timestamps, sk.values, params_value);
    if (!sk.values_bound.empty()) {
      matplotlibcpp::plot(sk.timestamps, sk.values_bound, params_bound);
      for (size_t i = 0; i < sk.timestamps.size(); i++) {
        sk.values_bound[i] *= -1;
      }
      matplotlibcpp::plot(sk.timestamps, sk.values_bound, params_bound);
    }
    matplotlibcpp::xlim(0.0, endtime4 - starttime4);
  }

  /**
   * @brief Plots general vector error different statistic values and sigma bounds
   */
  void plot_errors(vector<ov_eval::Statistics> vs, string sensor, vector<string> names, string color) {
    matplotlibcpp::figure(figure_id);

    for (int i = 0; i < (int)vs.size(); i++) {
      // Zero our time arrays
      double s_time = (vs[i].timestamps.empty()) ? 0 : vs[i].timestamps[0];
      double e_time = (vs[i].timestamps.empty()) ? 0 : vs[i].timestamps[vs[i].timestamps.size() - 1];
      for (double &timestamp : vs[i].timestamps)
        timestamp -= start_time;

      // Parameters that define the line styles
      map<string, string> params_value, params_bound;
      params_value.insert({"linestyle", "-"});
      params_value.insert({"color", color});
      params_bound.insert({"linestyle", "--"});
      params_bound.insert({"color", color});

      // Plot our error value
      matplotlibcpp::subplot(vs.size(), 1, i + 1);
      if (i == 0)
        matplotlibcpp::title(sensor + " Error");
      matplotlibcpp::plot(vs[i].timestamps, vs[i].values, params_value);
      matplotlibcpp::plot(vs[i].timestamps, vs[i].values_bound, params_bound);
      for (size_t j = 0; j < vs[i].timestamps.size(); j++)
        vs[i].values_bound[j] *= -1;
      matplotlibcpp::plot(vs[i].timestamps, vs[i].values_bound, params_bound);
      matplotlibcpp::xlim(0.0, e_time - s_time);
      matplotlibcpp::xlabel("dataset time (s)");
      matplotlibcpp::ylabel(names[i]);
    }
  }

#endif
};

} // namespace mins_eval

#endif // MINS_EVAL_ERRORPLOT

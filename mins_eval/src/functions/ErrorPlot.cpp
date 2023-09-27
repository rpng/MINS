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

#include "ErrorPlot.h"

using namespace mins_eval;
using namespace std;


ErrorPlot::ErrorPlot(string load_path, string save_path) : save_path(save_path) {

  load_data(load_path, "imu", imu_data);

  // wheel
  load_data(load_path, "wheel_ext", wheel_ext_data);
  load_data(load_path, "wheel_int", wheel_int_data);
  load_data(load_path, "wheel_dt", wheel_dt_data);

  for (int j = 0; j < 10; j++) {
    DATA vicon_ext_data_j;
    if (load_data(load_path, "vicon" + to_string(j) + "_ext", vicon_ext_data_j)) {
      vicon_ext_data.push_back(vicon_ext_data_j);
    }
    DATA vicon_dt_data_j;
    if (load_data(load_path, "vicon" + to_string(j) + "_dt", vicon_dt_data_j)) {
      vicon_dt_data.push_back(vicon_dt_data_j);
    }
  }

  for (int j = 0; j < 10; j++) {
    DATA lidar_ext_data_j;
    if (load_data(load_path, "lidar" + to_string(j) + "_ext", lidar_ext_data_j)) {
      lidar_ext_data.push_back(lidar_ext_data_j);
    }
    DATA lidar_dt_data_j;
    if (load_data(load_path, "lidar" + to_string(j) + "_dt", lidar_dt_data_j)) {
      lidar_dt_data.push_back(lidar_dt_data_j);
    }
  }

  for (int j = 0; j < 10; j++) {
    DATA gps_ext_data_j;
    if (load_data(load_path, "gps" + to_string(j) + "_ext", gps_ext_data_j)) {
      gps_ext_data.push_back(gps_ext_data_j);
    }
    DATA gps_dt_data_j;
    if (load_data(load_path, "gps" + to_string(j) + "_dt", gps_dt_data_j)) {
      gps_dt_data.push_back(gps_dt_data_j);
    }
  }

  for (int j = 0; j < 10; j++) {
    DATA cam_ext_data_j;
    if (load_data(load_path, "cam" + to_string(j) + "_ext", cam_ext_data_j)) {
      cam_ext_data.push_back(cam_ext_data_j);
    }
    DATA cam_dt_data_j;
    if (load_data(load_path, "cam" + to_string(j) + "_dt", cam_dt_data_j)) {
      cam_dt_data.push_back(cam_dt_data_j);
    }
    DATA cam_int_data_j;
    if (load_data(load_path, "cam" + to_string(j) + "_int", cam_int_data_j)) {
      cam_int_data.push_back(cam_int_data_j);
    }
  }

  //===============================================================
  if (imu_data.ests.size() < 1) {
    int n = load_path.length();
    char char_array[n + 1];
    strcpy(char_array, load_path.c_str());
    // Ensure we have a path
    printf(RED "[LOAD]: The folder %s has no files\n" RESET, char_array);
    printf(RED "[LOAD]: rosrun mins_eval plot_consistency <load directory> <save directory=load directory> <visualize=true>\n" RESET);
    exit(EXIT_FAILURE);
  }

  if (!vicon_ext_data.empty())
    assert(imu_data.ests.size() == vicon_ext_data[0].ests.size());

  if (!gps_ext_data.empty())
    assert(imu_data.ests.size() == gps_ext_data[0].ests.size());

  start_time = imu_data.ests[0][0](0);

  // ensure that we have enough colors to visualize
  assert(imu_data.ests.size() <= colors.size());
}

bool ErrorPlot::load_data(string load_path, string sensor, DATA &data) {

  int n_data = 0;

  for (int i = 0; i < 100; i++) {

    // Our file paths
    string path_est = load_path + "/" + to_string(i) + "/" + sensor + "_est.txt";
    string path_std = load_path + "/" + to_string(i) + "/" + sensor + "_std.txt";
    string path_gt = load_path + "/" + to_string(i) + "/" + sensor + "_gt.txt";

    // Check if the file exists
    ifstream file(path_est);
    if (!file.is_open()) {
      file.close();
      continue;
    }

    // Load from file if we can open the file
    vector<VectorXd> est, std, gt;
    ov_eval::Loader::load_simulation(path_est, est);
    ov_eval::Loader::load_simulation(path_std, std);
    ov_eval::Loader::load_simulation(path_gt, gt);

    // keep the vis size up to 1000
    vector<VectorXd> est_tmp, std_tmp, gt_tmp;
    for (size_t i = 0; i < est.size(); i += floor(est.size() / 1000.0) + 1) {
      est_tmp.push_back(est[i]);
      std_tmp.push_back(std[i]);
      gt_tmp.push_back(gt[i]);
    }
    est = est_tmp;
    std = std_tmp;
    gt = gt_tmp;

    /// Assert they are of equal length
    assert(est.size() == std.size());
    assert(est.size() == gt.size());

    // Append to the global array
    data.ests.push_back(est);
    data.stds.push_back(std);
    data.gts.push_back(gt);

    n_data++;
  }

  /// Assert they are of equal length
  assert(data.ests.size() == data.stds.size());
  assert(data.ests.size() == data.gts.size());

  if (n_data > 0)
    return true;
  else
    return false;
}

void ErrorPlot::plot_imu(DATA data, double max_time) {
  // Create all the figures we will plot on top of

#ifdef HAVE_PYTHONLIBS
  for (int i = 0; i < 5; i++) {
    matplotlibcpp::figure_size(600, 70 + 120 * 3);
    matplotlibcpp::figure(figure_id + i);
  }
#endif

  // Loop through each of the plots
  for (int n = 0; n < (int)data.ests.size(); n++) {

    // This specific run values
    vector<VectorXd> est = data.ests[n];
    vector<VectorXd> std = data.stds[n];
    vector<VectorXd> gt = data.gts[n];

    // Errors for each xyz direction
    ov_eval::Statistics error_ori[3], error_pos[3], error_vel[3], error_bg[3], error_ba[3];

    // Calculate the position and orientation error at every timestep
    for (size_t i = 0; i < est.size(); i++) {

      // Exit if we have reached our max time
      if ((est[i](0) - start_time) > max_time)
        break;

      // Assert our times are the same
      assert(est[i](0) == gt[i](0));

      // Calculate orientation error
      // NOTE: we define our error as e_R = -Log(R*Rhat^T)
      Matrix3d e_R = quat_2_Rot(gt[i].block(1, 0, 4, 1)) * quat_2_Rot(est[i].block(1, 0, 4, 1)).transpose();
      Vector3d ori_err = -180.0 / M_PI * log_so3(e_R);
      for (int j = 0; j < 3; j++) {
        error_ori[j].timestamps.push_back(est[i](0));
        error_ori[j].values.push_back(ori_err(j));
        error_ori[j].values_bound.push_back(3 * 180.0 / M_PI * std[i](1 + j));
        error_ori[j].calculate();
      }

      // Calculate position error
      Vector3d pos_err = gt[i].block(5, 0, 3, 1) - est[i].block(5, 0, 3, 1);
      for (int j = 0; j < 3; j++) {
        error_pos[j].timestamps.push_back(est[i](0));
        error_pos[j].values.push_back(pos_err(j));
        error_pos[j].values_bound.push_back(3 * std[i](4 + j));
        error_pos[j].calculate();
      }

      // Calculate velocity error
      Vector3d vel_err = gt[i].block(8, 0, 3, 1) - est[i].block(8, 0, 3, 1);
      for (int j = 0; j < 3; j++) {
        error_vel[j].timestamps.push_back(est[i](0));
        error_vel[j].values.push_back(vel_err(j));
        error_vel[j].values_bound.push_back(3 * std[i](7 + j));
        error_vel[j].calculate();
      }

      // Calculate gyro bias error
      Vector3d bg_err = gt[i].block(11, 0, 3, 1) - est[i].block(11, 0, 3, 1);
      for (int j = 0; j < 3; j++) {
        error_bg[j].timestamps.push_back(est[i](0));
        error_bg[j].values.push_back(bg_err(j));
        error_bg[j].values_bound.push_back(3 * std[i](10 + j));
        error_bg[j].calculate();
      }

      // Calculate accel bias error
      Vector3d ba_err = gt[i].block(14, 0, 3, 1) - est[i].block(14, 0, 3, 1);
      for (int j = 0; j < 3; j++) {
        error_ba[j].timestamps.push_back(est[i](0));
        error_ba[j].values.push_back(ba_err(j));
        error_ba[j].values_bound.push_back(3 * std[i](13 + j));
        error_ba[j].calculate();
      }
    }

#ifndef HAVE_PYTHONLIBS
    printf("Unable to plot the state error, just returning..");
    continue;
#endif
#ifdef HAVE_PYTHONLIBS

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure(1);
    plot_3errors(error_ori[0], error_ori[1], error_ori[2], colors[n], colors[n]);
    // Update the title and axis labels
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::title("IMU Orientation Error");
    matplotlibcpp::ylabel("x-error (deg)");
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::ylabel("y-error (deg)");
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::ylabel("z-error (deg)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::save(save_path + "IMU_ori.png");
    matplotlibcpp::show(false);
    //=====================================================

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure(2);
    plot_3errors(error_pos[0], error_pos[1], error_pos[2], colors[n], colors[n]);
    // Update the title and axis labels
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::title("IMU Position Error");
    matplotlibcpp::ylabel("x-error (m)");
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::ylabel("y-error (m)");
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::ylabel("z-error (m)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::save(save_path + "IMU_pos.png");
    matplotlibcpp::show(false);
    //=====================================================

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure(3);
    plot_3errors(error_vel[0], error_vel[1], error_vel[2], colors[n], colors[n]);
    // Update the title and axis labels
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::title("IMU Velocity Error");
    matplotlibcpp::ylabel("x-error (m/s)");
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::ylabel("y-error (m/s)");
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::ylabel("z-error (m/s)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::save(save_path + "IMU_vel.png");
    matplotlibcpp::show(false);
    //=====================================================

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure(4);
    plot_3errors(error_bg[0], error_bg[1], error_bg[2], colors[n], colors[n]);
    // Update the title and axis labels
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::title("IMU Gyroscope Bias Error");
    matplotlibcpp::ylabel("x-error (rad/s)");
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::ylabel("y-error (rad/s)");
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::ylabel("z-error (rad/s)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::save(save_path + "IMU_bg.png");
    matplotlibcpp::show(false);
    //=====================================================

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure(5);
    plot_3errors(error_ba[0], error_ba[1], error_ba[2], colors[n], colors[n]);
    // Update the title and axis labels
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::title("IMU Accelerometer Bias Error");
    matplotlibcpp::ylabel("x-error (m/s^2)");
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::ylabel("y-error (m/s^2)");
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::ylabel("z-error (m/s^2)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::save(save_path + "IMU_ba.png");
    matplotlibcpp::show(false);
    //=====================================================
#endif
  }

  figure_id += 5;
}

void ErrorPlot::plot_extrinsic(DATA data, string sensor, double max_time) {
#ifdef HAVE_PYTHONLIBS
  matplotlibcpp::figure_size(600, 70 + 120 * 3);
  matplotlibcpp::figure(figure_id);
  matplotlibcpp::figure_size(600, 70 + 120 * 3);
  matplotlibcpp::figure(figure_id + 1);
#endif

  for (int n = 0; n < (int)data.ests.size(); n++) {

    // This specific run values
    vector<VectorXd> est = data.ests[n];
    vector<VectorXd> std = data.stds[n];
    vector<VectorXd> gt = data.gts[n];

    // Camera extrinsics statistic storage
    vector<ov_eval::Statistics> error_ori, error_pos;
    for (int j = 0; j < 3; j++) {
      error_ori.push_back(ov_eval::Statistics());
      error_pos.push_back(ov_eval::Statistics());
    }

    for (size_t i = 0; i < est.size(); i++) {

      // Exit if we have reached our max time
      if ((est[i](0) - start_time) > max_time)
        break;

      // Assert our times are the same
      assert(est[i](0) == gt[i](0));

      // NOTE: we define our error as e_R = -Log(R*Rhat^T)
      Matrix3d e_R = quat_2_Rot(gt[i].block(1, 0, 4, 1)) * quat_2_Rot(est[i].block(1, 0, 4, 1)).transpose();
      Vector3d ori_err = -180.0 / M_PI * log_so3(e_R);
      for (int j = 0; j < 3; j++) {
        error_ori[j].timestamps.push_back(est[i](0));
        error_ori[j].values.push_back(ori_err(j));
        error_ori[j].values_bound.push_back(3 * 180.0 / M_PI * std[i](1 + j));
        error_pos[j].timestamps.push_back(est[i](0));
        error_pos[j].values.push_back(gt[i](5 + j) - est[i](5 + j));
        error_pos[j].values_bound.push_back(3 * std[i](4 + j));
      }
    }

#ifndef HAVE_PYTHONLIBS
    printf("Unable to plot the timeoffset error, just returning..");
    return;
#endif
#ifdef HAVE_PYTHONLIBS

    // Plot line colors
    assert(error_ori.size() <= colors.size());

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure(figure_id);
    plot_3errors(error_ori[0], error_ori[1], error_ori[2], colors[n], colors[n]);

    // Update the title and axis labels
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::title(sensor + " Extrinsic Orientation Error");
    matplotlibcpp::ylabel("x-error (deg)");
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::ylabel("y-error (deg)");
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::ylabel("z-error (deg)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::save(save_path + sensor + "_Extrinsic_Orientation.png");
    matplotlibcpp::show(false);
    //=====================================================

    //=====================================================
    // Plot this figure
    matplotlibcpp::figure(figure_id + 1);
    plot_3errors(error_pos[0], error_pos[1], error_pos[2], colors[n], colors[n]);

    // Update the title and axis labels
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::title(sensor + " Extrinsic Position Error");
    matplotlibcpp::ylabel("x-error (m)");
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::ylabel("y-error (m)");
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::ylabel("z-error (m)");
    matplotlibcpp::xlabel("dataset time (s)");
    matplotlibcpp::save(save_path + sensor + "_Extrinsic_Position.png");
    matplotlibcpp::show(false);
    //=====================================================
#endif
  }

  figure_id += 2;
}

void ErrorPlot::plot_vector(DATA data, string sensor, vector<string> names, double max_time) {
  int vec_size = data.ests[0][0].size() - 1;
#ifdef HAVE_PYTHONLIBS
  matplotlibcpp::figure_size(600, 70 + 120 * vec_size);
  matplotlibcpp::figure(figure_id);
#endif

  for (size_t i = 0; i < data.ests.size(); i++) {

    // This specific run values
    vector<VectorXd> est = data.ests[i];
    vector<VectorXd> std = data.stds[i];
    vector<VectorXd> gt = data.gts[i];

    // Camera extrinsics statistic storage
    vector<ov_eval::Statistics> error;
    for (int j = 0; j < vec_size; j++)
      error.emplace_back();

    for (size_t j = 0; j < est.size(); j++) {
      // Exit if we have reached our max time
      if ((est[j](0) - start_time) > max_time)
        break;

      for (int k = 0; k < vec_size; k++) {
        error[k].timestamps.push_back(est[j](0));
        error[k].values.push_back(gt[j](1 + k) - est[j](1 + k));
        error[k].values_bound.push_back(3 * std[j](1 + k));
      }
    }

#ifndef HAVE_PYTHONLIBS
    printf("Unable to plot the timeoffset error, just returning..");
    return;
#endif
#ifdef HAVE_PYTHONLIBS

    //=====================================================
    // Plot this figure
    plot_errors(error, sensor, names, colors[i]);
#endif
  }
#ifdef HAVE_PYTHONLIBS
  string sensor_wo_space;
  for (auto c : sensor) {
    if (c != ' ')
      sensor_wo_space += c;
    else
      sensor_wo_space += '_';
  }

  matplotlibcpp::save(save_path + sensor_wo_space + ".png");
  matplotlibcpp::show(false);
#endif
  figure_id += 1;
}

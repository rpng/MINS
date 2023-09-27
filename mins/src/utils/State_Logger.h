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

#ifndef MINS_LOGGER_H
#define MINS_LOGGER_H

#include "boost/filesystem/fstream.hpp"
#include "boost/filesystem/operations.hpp"
#include "options/Options.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsLidar.h"
#include "options/OptionsSimulation.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "sim/Simulator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "update/gps/UpdaterGPS.h"
#include "utils/colors.h"

using namespace std;
using namespace Eigen;
using namespace ov_type;

namespace mins {
typedef shared_ptr<State> State_ptr;

class State_Logger {
public:
  ~State_Logger(){};

  /// Constructor of State, ground truth, and timing logger.
  State_Logger(shared_ptr<Options> op, shared_ptr<Simulator> sim = nullptr) : op(op) {

    // Create output files
    if (op->sys->save_state && sim == nullptr)
      PRINT4(RED "Cannot save the state info because the simulator is not being used.\n" RESET);
    if (op->sys->save_state && sim != nullptr) {
      // file paths of our savings
      string path_state = op->sys->path_state + (op->sys->path_state.back() != '/' ? "/" : "");

      // IMU should be always in use, as it is used for state propagation
      filepath_est.insert({"imu", path_state + "imu_est.txt"});
      filepath_std.insert({"imu", path_state + "imu_std.txt"});
      filepath_gth.insert({"imu", path_state + "imu_gt.txt"});

      // camera
      if (op->est->cam->enabled) {
        for (int i = 0; i < op->est->cam->max_n; i++) {
          string id = to_string(i);
          if (op->est->cam->do_calib_dt) {
            filepath_est.insert({"cam" + id + "dt", path_state + "cam" + id + "_dt_est.txt"});
            filepath_std.insert({"cam" + id + "dt", path_state + "cam" + id + "_dt_std.txt"});
            filepath_gth.insert({"cam" + id + "dt", path_state + "cam" + id + "_dt_gt.txt"});
          }
          if (op->est->cam->do_calib_ext) {
            filepath_est.insert({"cam" + id + "ext", path_state + "cam" + id + "_ext_est.txt"});
            filepath_std.insert({"cam" + id + "ext", path_state + "cam" + id + "_ext_std.txt"});
            filepath_gth.insert({"cam" + id + "ext", path_state + "cam" + id + "_ext_gt.txt"});
          }
          if (op->est->cam->do_calib_int) {
            filepath_est.insert({"cam" + id + "int", path_state + "cam" + id + "_int_est.txt"});
            filepath_std.insert({"cam" + id + "int", path_state + "cam" + id + "_int_std.txt"});
            filepath_gth.insert({"cam" + id + "int", path_state + "cam" + id + "_int_gt.txt"});
          }
        }
      }

      // gps
      if (op->est->gps->enabled) {
        for (int i = 0; i < op->est->gps->max_n; i++) {
          string id = to_string(i);
          if (op->est->gps->do_calib_dt) {
            filepath_est.insert({"gps" + id + "dt", path_state + "gps" + id + "_dt_est.txt"});
            filepath_std.insert({"gps" + id + "dt", path_state + "gps" + id + "_dt_std.txt"});
            filepath_gth.insert({"gps" + id + "dt", path_state + "gps" + id + "_dt_gt.txt"});
          }
          if (op->est->gps->do_calib_ext) {
            filepath_est.insert({"gps" + id + "ext", path_state + "gps" + id + "_ext_est.txt"});
            filepath_std.insert({"gps" + id + "ext", path_state + "gps" + id + "_ext_std.txt"});
            filepath_gth.insert({"gps" + id + "ext", path_state + "gps" + id + "_ext_gt.txt"});
          }
        }
      }

      // lidar
      if (op->est->lidar->enabled) {
        for (int i = 0; i < op->est->lidar->max_n; i++) {
          string id = to_string(i);
          if (op->est->lidar->do_calib_dt) {
            filepath_est.insert({"lidar" + id + "dt", path_state + "lidar" + id + "_dt_est.txt"});
            filepath_std.insert({"lidar" + id + "dt", path_state + "lidar" + id + "_dt_std.txt"});
            filepath_gth.insert({"lidar" + id + "dt", path_state + "lidar" + id + "_dt_gt.txt"});
          }
          if (op->est->lidar->do_calib_ext) {
            filepath_est.insert({"lidar" + id + "ext", path_state + "lidar" + id + "_ext_est.txt"});
            filepath_std.insert({"lidar" + id + "ext", path_state + "lidar" + id + "_ext_std.txt"});
            filepath_gth.insert({"lidar" + id + "ext", path_state + "lidar" + id + "_ext_gt.txt"});
          }
        }
      }

      // vicon
      if (op->est->vicon->enabled) {
        for (int i = 0; i < op->est->vicon->max_n; i++) {
          string id = to_string(i);
          if (op->est->vicon->do_calib_dt) {
            filepath_est.insert({"vicon" + id + "dt", path_state + "vicon" + id + "_dt_est.txt"});
            filepath_std.insert({"vicon" + id + "dt", path_state + "vicon" + id + "_dt_std.txt"});
            filepath_gth.insert({"vicon" + id + "dt", path_state + "vicon" + id + "_dt_gt.txt"});
          }
          if (op->est->vicon->do_calib_ext) {
            filepath_est.insert({"vicon" + id + "ext", path_state + "vicon" + id + "_ext_est.txt"});
            filepath_std.insert({"vicon" + id + "ext", path_state + "vicon" + id + "_ext_std.txt"});
            filepath_gth.insert({"vicon" + id + "ext", path_state + "vicon" + id + "_ext_gt.txt"});
          }
        }
      }

      // wheel
      if (op->est->wheel->enabled) {
        if (op->est->wheel->do_calib_dt) {
          filepath_est.insert({"wheeldt", path_state + "wheel_dt_est.txt"});
          filepath_std.insert({"wheeldt", path_state + "wheel_dt_std.txt"});
          filepath_gth.insert({"wheeldt", path_state + "wheel_dt_gt.txt"});
        }
        if (op->est->wheel->do_calib_ext) {
          filepath_est.insert({"wheelext", path_state + "wheel_ext_est.txt"});
          filepath_std.insert({"wheelext", path_state + "wheel_ext_std.txt"});
          filepath_gth.insert({"wheelext", path_state + "wheel_ext_gt.txt"});
        }
        if (op->est->wheel->do_calib_int) {
          filepath_est.insert({"wheelint", path_state + "wheel_int_est.txt"});
          filepath_std.insert({"wheelint", path_state + "wheel_int_std.txt"});
          filepath_gth.insert({"wheelint", path_state + "wheel_int_gt.txt"});
        }
      }

      for (auto pair : filepath_est) {
        string sensor = pair.first;
        // If the files exist, then delete it
        boost::filesystem::exists(filepath_est.at(sensor)) && boost::filesystem::remove(filepath_est.at(sensor));
        boost::filesystem::exists(filepath_std.at(sensor)) && boost::filesystem::remove(filepath_std.at(sensor));
        boost::filesystem::exists(filepath_gth.at(sensor)) && boost::filesystem::remove(filepath_gth.at(sensor));

        // Create folder path to this location if not exists
        boost::filesystem::create_directories(boost::filesystem::path(filepath_est.at(sensor).c_str()).parent_path());
        boost::filesystem::create_directories(boost::filesystem::path(filepath_std.at(sensor).c_str()).parent_path());
        boost::filesystem::create_directories(boost::filesystem::path(filepath_gth.at(sensor).c_str()).parent_path());

        // Open the files
        shared_ptr<ofstream> of_est_tmp = make_shared<ofstream>();
        shared_ptr<ofstream> of_std_tmp = make_shared<ofstream>();
        shared_ptr<ofstream> of_gt_tmp = make_shared<ofstream>();
        of_est_tmp->open(filepath_est.at(sensor).c_str());
        of_std_tmp->open(filepath_std.at(sensor).c_str());
        of_gt_tmp->open(filepath_gth.at(sensor).c_str());
        if (!of_est_tmp->is_open()) {
          PRINT4(RED "Cannot open state estimate recording file: %s\n" RESET, filepath_est.at(sensor).c_str());
          exit(EXIT_FAILURE);
        }
        if (!of_std_tmp->is_open()) {
          PRINT4(RED "Cannot open state covariance file: %s\n" RESET, filepath_std.at(sensor).c_str());
          exit(EXIT_FAILURE);
        }
        if (!of_gt_tmp->is_open()) {
          PRINT4(RED "Cannot open state groundtruth file: %s\n" RESET, filepath_gth.at(sensor).c_str());
          exit(EXIT_FAILURE);
        }
        of_est_tmp->setf(ios::fixed, ios::floatfield);
        of_std_tmp->setf(ios::fixed, ios::floatfield);
        of_gt_tmp->setf(ios::fixed, ios::floatfield);
        of_est_tmp->precision(6);
        of_std_tmp->precision(6);
        of_gt_tmp->precision(6);

        // append to the vector
        of_est.insert({sensor, of_est_tmp});
        of_std.insert({sensor, of_std_tmp});
        of_gt.insert({sensor, of_gt_tmp});
      }
    }

    // trajectory
    if (op->sys->save_trajectory) {
      // Create folder path to this location if not exists
      boost::filesystem::exists(op->sys->path_trajectory.c_str()) && boost::filesystem::remove(op->sys->path_trajectory.c_str());
      boost::filesystem::create_directories(boost::filesystem::path(op->sys->path_trajectory.c_str()).parent_path());
      of_traj.open(op->sys->path_trajectory.c_str());
      if (!of_traj.is_open()) {
        PRINT4(RED "Cannot open trajectory recording file: %s\n" RESET, op->sys->path_trajectory.c_str());
        exit(EXIT_FAILURE);
      }
      of_traj.setf(ios::fixed, ios::floatfield);
      of_traj << "# timestamp(s) tx ty tz qx qy qz qw Pr11 Pr12 Pr13 Pr22 Pr23 Pr33 Pt11 Pt12 Pt13 Pt22 Pt23 Pt33\n";
    }

    // Time
    if (op->sys->save_timing) {
      // Create folder path to this location if not exists
      boost::filesystem::exists(op->sys->path_timing.c_str()) && boost::filesystem::remove(op->sys->path_timing.c_str());
      boost::filesystem::create_directories(boost::filesystem::path(op->sys->path_timing.c_str()).parent_path());
      of_time.open(op->sys->path_timing.c_str());
      if (!of_time.is_open()) {
        PRINT4(RED "Cannot open timing recording file: %s\n" RESET, op->sys->path_timing.c_str());
        exit(EXIT_FAILURE);
      }
      of_time.setf(ios::fixed, ios::floatfield);
      of_time.precision(6);
    }
  }

  /// save the state (estimation, covariance) and the ground truth to file
  void save_state_to_file(shared_ptr<SystemManager> sys, shared_ptr<Simulator> sim) {
    // Do not record if GNSS is enabled but not initialized yet
    if (sys->state->op->gps->enabled && !sys->up_gps->initialized)
      return;

    cnt_state++;
    // id of ofstream vector for sensors
    auto ss = sys->state;
    auto st = op->sim->est_true;

    // save cam
    if (op->est->cam->enabled) {
      for (int i = 0; i < op->est->cam->max_n; i++) {
        string sid = "cam" + to_string(i);
        if (op->est->cam->do_calib_dt)
          save_dt(ss, ss->cam_dt.at(i), st->cam->dt.at(i), of_est.at(sid + "dt"), of_std.at(sid + "dt"), of_gt.at(sid + "dt"));
        if (op->est->cam->do_calib_ext)
          save_ext(ss, ss->cam_extrinsic.at(i), st->cam->extrinsics.at(i), of_est.at(sid + "ext"), of_std.at(sid + "ext"), of_gt.at(sid + "ext"));
        if (op->est->cam->do_calib_int)
          save_vec(ss, ss->cam_intrinsic.at(i), st->cam->intrinsics.at(i), 8, of_est.at(sid + "int"), of_std.at(sid + "int"), of_gt.at(sid + "int"));
      }
    }

    // save gps
    if (op->est->gps->enabled) {
      for (int i = 0; i < op->est->gps->max_n; i++) {
        string sid = "gps" + to_string(i);
        if (op->est->gps->do_calib_dt) {
          save_dt(ss, ss->gps_dt.at(i), st->gps->dt.at(i), of_est.at(sid + "dt"), of_std.at(sid + "dt"), of_gt.at(sid + "dt"));
        }
        if (op->est->gps->do_calib_ext) {
          save_vec(ss, ss->gps_extrinsic.at(i), st->gps->extrinsics.at(i), 3, of_est.at(sid + "ext"), of_std.at(sid + "ext"), of_gt.at(sid + "ext"));
        }
      }
    }

    // save imu
    save_imu(ss, sim, of_est.at("imu"), of_std.at("imu"), of_gt.at("imu"));

    // save lidar
    if (op->est->lidar->enabled) {
      for (int i = 0; i < op->est->lidar->max_n; i++) {
        string sid = "lidar" + to_string(i);
        if (op->est->lidar->do_calib_dt) {
          save_dt(ss, ss->lidar_dt.at(i), st->lidar->dt.at(i), of_est.at(sid + "dt"), of_std.at(sid + "dt"), of_gt.at(sid + "dt"));
        }
        if (op->est->lidar->do_calib_ext) {
          save_ext(ss, ss->lidar_extrinsic.at(i), st->lidar->extrinsics.at(i), of_est.at(sid + "ext"), of_std.at(sid + "ext"), of_gt.at(sid + "ext"));
        }
      }
    }

    // save vicon
    if (op->est->vicon->enabled) {
      for (int i = 0; i < op->est->vicon->max_n; i++) {
        string sid = "vicon" + to_string(i);
        if (op->est->vicon->do_calib_dt) {
          save_dt(ss, ss->vicon_dt.at(i), st->vicon->dt.at(i), of_est.at(sid + "dt"), of_std.at(sid + "dt"), of_gt.at(sid + "dt"));
        }
        if (op->est->vicon->do_calib_ext) {
          save_ext(ss, ss->vicon_extrinsic.at(i), st->vicon->extrinsics.at(i), of_est.at(sid + "ext"), of_std.at(sid + "ext"), of_gt.at(sid + "ext"));
        }
      }
    }

    // save wheel
    if (op->est->wheel->enabled) {
      string sid = "wheel";
      if (op->est->wheel->do_calib_dt) {
        save_dt(ss, ss->wheel_dt, st->wheel->dt, of_est.at(sid + "dt"), of_std.at(sid + "dt"), of_gt.at(sid + "dt"));
      }
      if (op->est->wheel->do_calib_ext) {
        save_ext(ss, ss->wheel_extrinsic, st->wheel->extrinsics, of_est.at(sid + "ext"), of_std.at(sid + "ext"), of_gt.at(sid + "ext"));
      }
      if (op->est->wheel->do_calib_int) {
        save_vec(ss, ss->wheel_intrinsic, st->wheel->intrinsics, 3, of_est.at(sid + "int"), of_std.at(sid + "int"), of_gt.at(sid + "int"));
      }
    }
  }

  /// Save current IMU pose to a file
  void save_trajectory_to_file(shared_ptr<SystemManager> sys) {
    // Do not record if GNSS is enabled but not initialized yet
    if (sys->state->op->gps->enabled && !sys->up_gps->initialized)
      return;
    cnt_traj++;
    // Load pose info
    VectorXd q = sys->state->imu->quat();
    Vector3d p = sys->state->imu->pos();
    MatrixXd P = StateHelper::get_marginal_covariance(sys->state, {sys->state->imu->pose()});
    // Log
    of_traj.precision(6);
    of_traj << sys->state->time << " ";
    of_traj << p(0) << " " << p(1) << " " << p(2) << " ";
    of_traj << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " ";
    of_traj.precision(10);
    of_traj << P(0, 0) << " " << P(0, 1) << " " << P(0, 2) << " " << P(1, 1) << " " << P(1, 2) << " " << P(2, 2) << " ";
    of_traj << P(3, 3) << " " << P(3, 4) << " " << P(3, 5) << " " << P(4, 4) << " " << P(4, 5) << " " << P(5, 5) << endl;
  }

  /// Save given time to a file
  void save_timing_to_file(double t) {
    cnt_time++;
    of_time << t << endl;
    total_t = t;
  }

  /// Remove files if system crash at the beginning or did not record anything
  void check_files() {
    // State files
    if (op->sys->save_state && (total_t < 0 || cnt_state == 0)) {
      for (auto pair : filepath_est) {
        string sensor = pair.first;
        // If the files exist, then delete it
        boost::filesystem::exists(filepath_est.at(sensor)) && boost::filesystem::remove(filepath_est.at(sensor));
        boost::filesystem::exists(filepath_std.at(sensor)) && boost::filesystem::remove(filepath_std.at(sensor));
        boost::filesystem::exists(filepath_gth.at(sensor)) && boost::filesystem::remove(filepath_gth.at(sensor));
      }
    }

    // Trajectory
    if (op->sys->save_trajectory && (total_t < 0 || cnt_traj == 0)) {
      boost::filesystem::exists(op->sys->path_trajectory.c_str()) && boost::filesystem::remove(op->sys->path_trajectory.c_str());
    }

    // Time
    if (op->sys->save_timing && (total_t < 0 || cnt_time == 0 || (cnt_state == 0 && cnt_traj == 0))) {
      boost::filesystem::exists(op->sys->path_timing.c_str()) && boost::filesystem::remove(op->sys->path_timing.c_str());
    }
  }

private:
  /// Save the timeoffset of the sensor (sensor-to-IMU) to file
  void save_dt(State_ptr state, shared_ptr<Vec> est_dt, double true_dt, shared_ptr<ofstream> of_est, shared_ptr<ofstream> of_std, shared_ptr<ofstream> of_gt) {
    // ground truth
    *of_gt << state->time << " ";
    *of_gt << true_dt << " ";
    *of_gt << endl;
    // estimated value
    *of_est << state->time << " ";
    *of_est << est_dt->value()(0) << " ";
    *of_est << endl;
    // standard deviation of the estimated state
    MatrixXd cov = StateHelper::get_marginal_covariance(state, {est_dt});
    *of_std << state->time << " ";
    *of_std << sqrt(cov(0, 0)) << " ";
    *of_std << endl;
  }

  /// Save the extrinsics of the sensor (sensor-to-IMU: qItoS, pIinS) to file
  void save_ext(State_ptr state, shared_ptr<PoseJPL> est_ext, MatrixXd true_ext, shared_ptr<ofstream> of_est, shared_ptr<ofstream> of_std, shared_ptr<ofstream> of_gt) {
    // ground truth
    *of_gt << state->time << " ";
    for (int i = 0; i < 7; i++)
      *of_gt << true_ext(i) << " ";
    *of_gt << endl;
    // estimated value
    *of_est << state->time << " ";
    for (int i = 0; i < 7; i++)
      *of_est << est_ext->value()(i) << " ";
    *of_est << endl;
    // standard deviation of the estimated state
    MatrixXd cov = StateHelper::get_marginal_covariance(state, {est_ext});
    *of_std << state->time << " ";
    for (int i = 0; i < cov.cols(); i++)
      *of_std << sqrt(cov(i, i)) << " ";
    *of_std << endl;
  }

  /// Save general vector type state to file. Need to specify the size of the vector.
  void save_vec(State_ptr state, shared_ptr<Vec> est_vec, MatrixXd true_vec, int sz, shared_ptr<ofstream> of_est, shared_ptr<ofstream> of_std, shared_ptr<ofstream> of_gt) {
    // ground truth
    *of_gt << state->time << " ";
    for (int i = 0; i < sz; i++)
      *of_gt << true_vec(i) << " ";
    *of_gt << endl;
    // estimated value
    *of_est << state->time << " ";
    for (int i = 0; i < sz; i++)
      *of_est << est_vec->value()(i) << " ";
    *of_est << endl;
    // standard deviation of the estimated state
    MatrixXd cov = StateHelper::get_marginal_covariance(state, {est_vec});
    *of_std << state->time << " ";
    for (int i = 0; i < cov.cols(); i++)
      *of_std << sqrt(cov(i, i)) << " ";
    *of_std << endl;
  }

  /// Save IMU state (qGtoI, pIinG, vIinG, bg, ba) to file
  void save_imu(State_ptr state, shared_ptr<Simulator> sim, shared_ptr<ofstream> of_est, shared_ptr<ofstream> of_std, shared_ptr<ofstream> of_gt) {
    // ground truth
    Matrix<double, 17, 1> state_gt;
    assert(sim->get_imu_state(state->time, state_gt));
    for (int i = 0; i < 17; i++)
      *of_gt << state_gt(i) << " ";
    *of_gt << endl;
    // estimated value
    auto state_est = state->imu->value();
    *of_est << state->time << " ";
    for (int i = 0; i < 16; i++)
      *of_est << state_est(i) << " ";
    *of_est << endl;
    // standard deviation of the estimated state
    MatrixXd cov = StateHelper::get_marginal_covariance(state, {state->imu});
    *of_std << state->time << " ";
    for (int i = 0; i < cov.cols(); i++)
      *of_std << sqrt(cov(i, i)) << " ";
    *of_std << endl;
  }

  /// Options of the system
  shared_ptr<Options> op;

  /// file handlers of estimation, standard deviation, and the ground truth
  unordered_map<string, shared_ptr<ofstream>> of_est, of_std, of_gt;
  unordered_map<string, string> filepath_est, filepath_std, filepath_gth;

  /// file handlers of time and trajectory
  ofstream of_time, of_traj;

  /// counter of each logging function
  int cnt_state = 0;
  int cnt_traj = 0;
  int cnt_time = 0;

  /// Saved timing
  double total_t = -1;
};

} // namespace mins
#endif // MINS_LOGGER_H

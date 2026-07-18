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

// Headless simulation runner - no ROS. Runs the simulator through the estimator
// and prints/saves accuracy, so the core can be built and exercised without ROS.

#include <memory>

#include "core/SystemManager.h"
#include "options/Options.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsSystem.h"
#include "sim/Simulator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/cam/CamTypes.h"
#include "update/gps/GPSTypes.h"
#include "update/gps/UpdaterGPS.h"
#include "update/vicon/ViconTypes.h"
#include "update/wheel/WheelTypes.h"
#include "utils/Print_Logger.h"
#include "utils/State_Logger.h"
#include "utils/TimeChecker.h"
#include "utils/colors.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/sensor_data.h"
#include "update/lidar/PointCloud.h"
using namespace mins;
using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
  // Config path comes from argv only (no ROS param server in this build)
  string config_path = "unset_path_to_config.yaml";
  if (argc > 1)
    config_path = argv[1];

  auto parser = make_shared<ov_core::YamlParser>(config_path);
  auto op = make_shared<Options>();
  op->load_print(parser);
  op->sys->save_prints ? Print_Logger::open_file(op->sys->path_state, true) : void();

  auto sim = make_shared<Simulator>(op);
  auto sys = make_shared<SystemManager>(op->est, sim);
  auto save = make_shared<State_Logger>(op, sim);

  if (!parser->successful()) {
    PRINT4(RED "unable to parse all parameters, please fix\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Running accuracy (same math the ROS visualizer does, minus the publishing)
  double sum_rmse_ori = 0, sum_rmse_pos = 0, sum_nees_ori = 0, sum_nees_pos = 0;
  int sum_cnt = 0;

  while (sim->ok()) {
    // IMU: propagate, and on a filter update print/accumulate accuracy
    ov_core::ImuData imu;
    if (sim->get_next_imu(imu)) {
      if (sys->feed_measurement_imu(imu)) {
        sim->trans_gt_to_ENU = op->est->gps->enabled && sys->up_gps->initialized;
        auto imu_pose = sys->state->imu->pose();
        Vector4d rn = sim->imu_rmse_nees(sys->state->time, imu_pose->value(), StateHelper::get_marginal_covariance(sys->state, {imu_pose}));
        if (!isnan(rn(2)) && !isnan(rn(3))) {
          sum_rmse_ori += rn(0);
          sum_rmse_pos += rn(1);
          sum_nees_ori += rn(2);
          sum_nees_pos += rn(3);
          sum_cnt++;
        }
        PRINT2("\033[A%.2f | RMSE: %.3f, %.3f (deg,m) | RMSE avg: %.3f, %.3f | NEES avg: %.1f, %.1f\n\n", sys->state->time, rn(0), rn(1),
               sum_rmse_ori / sum_cnt, sum_rmse_pos / sum_cnt, sum_nees_ori / sum_cnt, sum_nees_pos / sum_cnt);
        op->sys->save_state ? save->save_state_to_file(sys, sim) : void();
        op->sys->save_trajectory ? save->save_trajectory_to_file(sys) : void();
      }
    }

    CamSimData cam;
    if (sim->get_next_cam(cam))
      sys->feed_measurement_camsim(cam);

    GPSData gps;
    if (sim->get_next_gps(gps))
      sys->feed_measurement_gps(gps, false);

    WheelData wheel;
    if (sim->get_next_wheel(wheel))
      sys->feed_measurement_wheel(wheel);

    std::shared_ptr<mins::PointCloud<mins::PointXYZ>> lidar(new mins::PointCloud<mins::PointXYZ>);
    if (sim->get_next_lidar(lidar))
      sys->feed_measurement_lidar(lidar);

    ViconData vicon;
    if (sim->get_next_vicon(vicon))
      sys->feed_measurement_vicon(vicon);
  }

  sys->visualize_final();
  PRINT2(BOLDYELLOW "RMSE average: %.3f, %.3f (deg,m)\n" RESET, sum_rmse_ori / sum_cnt, sum_rmse_pos / sum_cnt);
  PRINT2(BOLDYELLOW "NEES average: %.3f, %.3f (deg,m)\n" RESET, sum_nees_ori / sum_cnt, sum_nees_pos / sum_cnt);
  op->sys->save_timing ? save->save_timing_to_file(sys->tc_sensors->get_total_sum()) : void();
  save->check_files();
  return EXIT_SUCCESS;
}

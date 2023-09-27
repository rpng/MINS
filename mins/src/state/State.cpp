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

#include "State.h"
#include "Propagator.h"
#include "cam/CamEqui.h"
#include "cam/CamRadtan.h"
#include "cpi/CpiV1.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsIMU.h"
#include "options/OptionsInit.h"
#include "options/OptionsLidar.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "update/gps/PoseJPL_4DOF.h"
#include "utils/Jabdongsani.h"
#include "utils/Print_Logger.h"
#include "utils/TimeChecker.h"

using namespace std;
using namespace Eigen;
using namespace ov_core;
using namespace mins;

State::State(shared_ptr<OptionsEstimator> op, std::shared_ptr<Simulator> sim) : op(op), sim(sim) {
  // set imu state
  int current_id = 0;
  imu = make_shared<ov_type::IMU>();
  imu->set_local_id(current_id);
  variables.push_back(imu);
  current_id += imu->size();

  // set camera state
  if (op->cam->enabled)
    set_camera_state(current_id);

  // set vicon state
  if (op->vicon->enabled)
    set_vicon_state(current_id);

  // set gps state
  if (op->gps->enabled)
    set_gps_state(current_id);

  // set Wheel state
  if (op->wheel->enabled)
    set_wheel_state(current_id);

  // set LiDAR state
  if (op->lidar->enabled)
    set_lidar_state(current_id);

  // set state covariance
  set_state_covariance(current_id);

  // init timing logger
  tc = make_shared<TimeChecker>();

  // init acc calculator
  est_a = make_shared<STAT>();
  est_A = make_shared<STAT>();
}

void State::set_camera_state(int &current_id) {
  // Loop through each camera and create timeoffset, extrinsic, and intrinsic
  for (int i = 0; i < op->cam->max_n; i++) {

    // Allocate extrinsic transform
    auto pose = make_shared<ov_type::PoseJPL>();
    pose->set_value(op->cam->extrinsics.at(i));
    pose->set_fej(op->cam->extrinsics.at(i));

    // Allocate intrinsics for this camera
    auto intrin = make_shared<ov_type::Vec>(8);
    intrin->set_value(op->cam->intrinsics.at(i));
    intrin->set_fej(op->cam->intrinsics.at(i));

    // Add these to the corresponding maps
    cam_extrinsic.insert({i, pose});
    cam_intrinsic.insert({i, intrin});
    if (op->cam->distortion_model.at(i) == "equidistant") {
      cam_intrinsic_model.insert({i, make_shared<ov_core::CamEqui>(op->cam->wh.at(i)[0], op->cam->wh.at(i)[1])});
      cam_intrinsic_model.at(i)->set_value(op->cam->intrinsics.at(i));
    } else {
      cam_intrinsic_model.insert({i, make_shared<ov_core::CamRadtan>(op->cam->wh.at(i)[0], op->cam->wh.at(i)[1])});
      cam_intrinsic_model.at(i)->set_value(op->cam->intrinsics.at(i));
    }

    // If calibrating camera extrinsic, add to variables
    if (op->cam->do_calib_ext) {
      pose->set_local_id(current_id);
      variables.push_back(pose);
      current_id += pose->size();
    }

    // If calibrating camera intrinsic, add to variables
    if (op->cam->do_calib_int) {
      intrin->set_local_id(current_id);
      variables.push_back(intrin);
      current_id += intrin->size();
    }

    // Allocate timeoffset
    auto dt = make_shared<ov_type::Vec>(1);
    dt->set_value(VectorXd::Ones(1) * op->cam->dt.at(i));
    dt->set_fej(VectorXd::Ones(1) * op->cam->dt.at(i));

    // Note: Stereo cam should have one timeoffset!
    if (op->cam->use_stereo && op->cam->stereo_pairs.find(i) != op->cam->stereo_pairs.end() && cam_dt.find(op->cam->stereo_pairs.at(i)) != cam_dt.end()) {
      cam_dt.insert({i, cam_dt.at(op->cam->stereo_pairs.at(i))});
    } else {
      cam_dt.insert({i, dt});
      if (op->cam->do_calib_dt) {
        dt->set_local_id(current_id);
        variables.push_back(dt);
        current_id += dt->size();
      }
    }
  }
}

void State::set_vicon_state(int &current_id) {
  // Loop through each camera and create timeoffset, extrinsic, and intrinsic
  for (int i = 0; i < op->vicon->max_n; i++) {
    // Allocate timeoffset
    auto dt = make_shared<ov_type::Vec>(1);
    dt->set_value(VectorXd::Ones(1) * op->vicon->dt.at(i));
    dt->set_fej(VectorXd::Ones(1) * op->vicon->dt.at(i));

    // Allocate extrinsic transform
    auto pose = make_shared<ov_type::PoseJPL>();
    pose->set_value(op->vicon->extrinsics.at(i));
    pose->set_fej(op->vicon->extrinsics.at(i));

    // Add these to the corresponding maps
    vicon_dt.insert({i, dt});
    vicon_extrinsic.insert({i, pose});

    if (op->vicon->do_calib_dt) {
      dt->set_local_id(current_id);
      variables.push_back(dt);
      current_id += dt->size();
    }

    // If calibrating camera extrinsic, add to variables
    if (op->vicon->do_calib_ext) {
      pose->set_local_id(current_id);
      variables.push_back(pose);
      current_id += pose->size();
    }
  }
}

vector<Vector3d> State::get_features_SLAM() {
  vector<Vector3d> slam_feats;
  for (auto &f : cam_SLAM_features) {
    if (ov_type::LandmarkRepresentation::is_relative_representation(f.second->_feat_representation)) {
      // Assert that we have an anchor pose for this feature
      assert(f.second->_anchor_cam_id != -1);
      // Get calibration for our anchor camera
      Matrix<double, 3, 3> R_ItoC = cam_extrinsic.at(f.second->_anchor_cam_id)->Rot();
      Matrix<double, 3, 1> p_IinC = cam_extrinsic.at(f.second->_anchor_cam_id)->pos();
      // Anchor pose orientation and position
      Matrix<double, 3, 3> R_GtoI = clones.at(f.second->_anchor_clone_timestamp)->Rot();
      Matrix<double, 3, 1> p_IinG = clones.at(f.second->_anchor_clone_timestamp)->pos();
      // Feature in the global frame
      slam_feats.emplace_back(R_GtoI.transpose() * R_ItoC.transpose() * (f.second->get_xyz(false) - p_IinC) + p_IinG);
    } else {
      slam_feats.push_back(f.second->get_xyz(false));
    }
  }
  return slam_feats;
}

void State::set_gps_state(int &current_id) {
  // set World to ENU transformation
  trans_WtoE = make_shared<ov_type::PoseJPL_4DOF>();

  // Loop through each camera and create timeoffset, extrinsic, and intrinsic
  for (int i = 0; i < op->gps->max_n; i++) {
    // Allocate timeoffset
    auto dt = make_shared<ov_type::Vec>(1);
    dt->set_value(VectorXd::Ones(1) * op->gps->dt.at(i));
    dt->set_fej(VectorXd::Ones(1) * op->gps->dt.at(i));

    // Allocate extrinsic transform
    auto pose = make_shared<ov_type::Vec>(3);
    pose->set_value(op->gps->extrinsics.at(i));
    pose->set_fej(op->gps->extrinsics.at(i));

    // Add these to the corresponding maps
    gps_dt.insert({i, dt});
    gps_extrinsic.insert({i, pose});

    if (op->gps->do_calib_dt) {
      dt->set_local_id(current_id);
      variables.push_back(dt);
      current_id += dt->size();
    }

    // If calibrating camera extrinsic, add to variables
    if (op->gps->do_calib_ext) {
      pose->set_local_id(current_id);
      variables.push_back(pose);
      current_id += pose->size();
    }
  }
}

void State::set_wheel_state(int &current_id) {
  // Loop through each camera and create timeoffset, extrinsic, and intrinsic
  // Allocate timeoffset
  auto dt = make_shared<ov_type::Vec>(1);
  dt->set_value(VectorXd::Ones(1) * op->wheel->dt);
  dt->set_fej(VectorXd::Ones(1) * op->wheel->dt);

  // Allocate extrinsic transform
  auto pose = make_shared<ov_type::PoseJPL>();
  pose->set_value(op->wheel->extrinsics);
  pose->set_fej(op->wheel->extrinsics);

  // Allocate intrinsics
  auto intrin = make_shared<ov_type::Vec>(3);
  intrin->set_value(op->wheel->intrinsics);
  intrin->set_fej(op->wheel->intrinsics);

  // Add these to the corresponding maps
  wheel_dt = dt;
  wheel_extrinsic = pose;
  wheel_intrinsic = intrin;

  if (op->wheel->do_calib_dt) {
    dt->set_local_id(current_id);
    variables.push_back(dt);
    current_id += dt->size();
  }

  // If calibrating camera extrinsic, add to variables
  if (op->wheel->do_calib_ext) {
    pose->set_local_id(current_id);
    variables.push_back(pose);
    current_id += pose->size();
  }

  // If calibrating camera extrinsic, add to variables
  if (op->wheel->do_calib_int) {
    intrin->set_local_id(current_id);
    variables.push_back(intrin);
    current_id += intrin->size();
  }
}

void State::set_lidar_state(int &current_id) {
  // Loop through each camera and create timeoffset, extrinsic, and intrinsic
  for (int i = 0; i < op->lidar->max_n; i++) {
    // Allocate timeoffset
    auto dt = make_shared<ov_type::Vec>(1);
    dt->set_value(VectorXd::Ones(1) * op->lidar->dt.at(i));
    dt->set_fej(VectorXd::Ones(1) * op->lidar->dt.at(i));

    // Allocate extrinsic transform
    auto pose = make_shared<ov_type::PoseJPL>();
    pose->set_value(op->lidar->extrinsics.at(i));
    pose->set_fej(op->lidar->extrinsics.at(i));

    // Add these to the corresponding maps
    lidar_dt.insert({i, dt});
    lidar_extrinsic.insert({i, pose});

    if (op->lidar->do_calib_dt) {
      dt->set_local_id(current_id);
      variables.push_back(dt);
      current_id += dt->size();
    }

    // If calibrating camera extrinsic, add to variables
    if (op->lidar->do_calib_ext) {
      pose->set_local_id(current_id);
      variables.push_back(pose);
      current_id += pose->size();
    }
  }
}

void State::set_state_covariance(int &current_id) {
  // Finally initialize our covariance to small value
  cov = op->init->cov_size * MatrixXd::Identity(current_id, current_id);
  // Make velocity uncertainty a bit bigger
  cov.block(imu->v()->id(), imu->v()->id(), 3, 3) *= 2;
  Matrix3d I3 = Matrix3d::Identity();
  Matrix2d I2 = Matrix2d::Identity();

  // set priors for calibration parameters
  if (op->cam->enabled) {
    for (int i = 0; i < op->cam->max_n; i++) {
      if (op->cam->do_calib_dt) {
        cov(cam_dt.at(i)->id(), cam_dt.at(i)->id()) = op->cam->init_cov_dt;
      }
      if (op->cam->do_calib_ext) {
        cov.block(cam_extrinsic.at(i)->id(), cam_extrinsic.at(i)->id(), 3, 3) = op->cam->init_cov_ex_o * I3;
        cov.block(cam_extrinsic.at(i)->id() + 3, cam_extrinsic.at(i)->id() + 3, 3, 3) = op->cam->init_cov_ex_p * I3;
      }
      if (op->cam->do_calib_int) {
        cov.block(cam_intrinsic.at(i)->id(), cam_intrinsic.at(i)->id(), 2, 2) = op->cam->init_cov_in_k * I2;
        cov.block(cam_intrinsic.at(i)->id() + 2, cam_intrinsic.at(i)->id() + 2, 2, 2) = op->cam->init_cov_in_c * I2;
        cov.block(cam_intrinsic.at(i)->id() + 4, cam_intrinsic.at(i)->id() + 4, 2, 2) = op->cam->init_cov_in_r * I2;
        if (op->cam->init_cov_in_r > 1e-5) {
          cov.block(cam_intrinsic.at(i)->id() + 6, cam_intrinsic.at(i)->id() + 6, 2, 2) = pow(sqrt(op->cam->init_cov_in_r) / 10.0, 2) * I2;
        } else {
          cov.block(cam_intrinsic.at(i)->id() + 6, cam_intrinsic.at(i)->id() + 6, 2, 2) = op->cam->init_cov_in_r * I2;
        }
      }
    }
  }
  if (op->vicon->enabled) {
    for (int i = 0; i < op->vicon->max_n; i++) {
      if (op->vicon->do_calib_dt) {
        cov(vicon_dt.at(i)->id(), vicon_dt.at(i)->id()) = op->vicon->init_cov_dt;
      }
      if (op->vicon->do_calib_ext) {
        cov.block(vicon_extrinsic.at(i)->id(), vicon_extrinsic.at(i)->id(), 3, 3) = op->vicon->init_cov_ex_o * I3;
        cov.block(vicon_extrinsic.at(i)->id() + 3, vicon_extrinsic.at(i)->id() + 3, 3, 3) = op->vicon->init_cov_ex_p * I3;
      }
    }
  }
  if (op->gps->enabled) {
    for (int i = 0; i < op->gps->max_n; i++) {
      if (op->gps->do_calib_dt) {
        cov(gps_dt.at(i)->id(), gps_dt.at(i)->id()) = op->gps->init_cov_dt;
      }
      if (op->gps->do_calib_ext) {
        cov.block(gps_extrinsic.at(i)->id(), gps_extrinsic.at(i)->id(), 3, 3) = op->gps->init_cov_ex * I3;
      }
    }
  }
  if (op->wheel->enabled) {
    if (op->wheel->do_calib_dt) {
      cov(wheel_dt->id(), wheel_dt->id()) = op->wheel->init_cov_dt;
    }
    if (op->wheel->do_calib_ext) {
      cov.block(wheel_extrinsic->id(), wheel_extrinsic->id(), 3, 3) = op->wheel->init_cov_ex_o * I3;
      cov.block(wheel_extrinsic->id() + 3, wheel_extrinsic->id() + 3, 3, 3) = op->wheel->init_cov_ex_p * I3;
    }
    if (op->wheel->do_calib_int) {
      cov.block(wheel_intrinsic->id(), wheel_intrinsic->id(), 2, 2) = op->wheel->init_cov_in_r * Matrix2d::Identity();
      cov(wheel_intrinsic->id() + 2, wheel_intrinsic->id() + 2) = op->wheel->init_cov_in_b;
    }
  }
  if (op->lidar->enabled) {
    for (int i = 0; i < op->lidar->max_n; i++) {
      if (op->lidar->do_calib_dt) {
        cov(lidar_dt.at(i)->id(), lidar_dt.at(i)->id()) = op->lidar->init_cov_dt;
      }
      if (op->lidar->do_calib_ext) {
        cov.block(lidar_extrinsic.at(i)->id(), lidar_extrinsic.at(i)->id(), 3, 3) = op->lidar->init_cov_ex_o * I3;
        cov.block(lidar_extrinsic.at(i)->id() + 3, lidar_extrinsic.at(i)->id() + 3, 3, 3) = op->lidar->init_cov_ex_p * I3;
      }
    }
  }
}

bool State::check_polynomial(double t_given) {
  // check if we have polynomial
  if (_polynomial_fej.empty()) {
    PRINT0(YELLOW "State::check_polynomial:t_given:%.4f. Failed because it is empty.\n" RESET, t_given);
    return false;
  }

  // check if timestamps of polynomial corresponds with clones
  for (const auto &poly : _polynomial_fej) {
    if (!have_clone(poly.first)) {
      PRINT0(YELLOW "State::check_polynomial:t_given:%.4f. Failed because it cannot find corresponding polynomial.\n" RESET, t_given);
      return false;
    }
  }

  // check if we have correct polynomial times for given time (t_given)
  vector<double> times;
  vector<shared_ptr<ov_type::PoseJPL>> poses;
  if (!bounding_poses_n(op->intr_order, poses, times, t_given)) {
    PRINT0(YELLOW "State::check_polynomial:t_given:%.4f. Failed because it cannot find n bounding poses.\n" RESET, t_given);
    return false;
  }

  // check if we have reference clone for reference polynomial
  if (_polynomial_fej.find(times[0]) == _polynomial_fej.end()) {
    PRINT0(YELLOW "State::check_polynomial:Cannot find clone %.4f time from polynomial!\n" RESET, times[0]);
    print_info();
    print_poly_info();
    PRINT0(YELLOW "State::check_polynomial:t_given:%.4f. Failed because polynomial is not created at required time.\n" RESET, t_given);
    return false;
  }

  // Passed all the checks :)
  return true;
}

void State::build_polynomial_data(bool fej) {

  // return if unable to build polynomial
  if ((int)clones.size() < op->intr_order + 1) {
    PRINT4(RED "[State]::build_polynomial_data Not enough clones for interpolation. " RESET);
    PRINT4(RED "#clones: %d, #required: %d\n" RESET, (int)clones.size(), op->intr_order + 1);
    _polynomial_est.clear();
    _polynomial_fej.clear();
    return;
  }

  // Get the times of our pose clones
  vector<double> clone_times;
  for (auto &clone : clones)
    clone_times.push_back(clone.first);

  // Build data for each possible starting polynomial
  map<double, State::polynomial_data> polynomial;
  for (size_t start = 0; start < clones.size() - op->intr_order; start++) {

    vector<shared_ptr<ov_type::PoseJPL>> poses;
    vector<double> times;
    for (size_t i = start; i < start + op->intr_order + 1; i++) {
      poses.push_back(clones.at(clone_times.at(i)));
      times.push_back(clone_times.at(i));
    }

    VectorXd diff_vec_ori(3 * op->intr_order, 1);
    VectorXd diff_vec_pos(3 * op->intr_order, 1);

    polynomial_data pData;

    Matrix3d R_GtoI0 = fej ? poses.at(0)->Rot_fej() : poses.at(0)->Rot();
    Vector3d p_I0inG = fej ? poses.at(0)->pos_fej() : poses.at(0)->pos();
    for (int i = 1; i <= op->intr_order; i++) {
      Matrix3d R_GtoIi_temp = fej ? poses.at(i)->Rot_fej() : poses.at(i)->Rot();
      Vector3d p_IiinG_temp = fej ? poses.at(i)->pos_fej() : poses.at(i)->pos();
      diff_vec_ori.block(3 * (i - 1), 0, 3, 1) = log_so3(R_GtoIi_temp * R_GtoI0.transpose());
      diff_vec_pos.block(3 * (i - 1), 0, 3, 1) = p_IiinG_temp - p_I0inG;
    }

    pData.diff_vec_ori = diff_vec_ori;
    pData.diff_vec_pos = diff_vec_pos;

    // Matirx that holds the constraints used to solve for coefficients
    // V_t*coeffs_ori =  diff_vec_ori
    // V_t*coeffs_pos =  diff_vec_pos
    MatrixXd V_t(3 * op->intr_order, 3 * op->intr_order);
    for (int p = 1; p <= op->intr_order; p++) {
      double dt_p = times.at(p) - times.at(0);
      for (int i = 1; i <= op->intr_order; i++) {
        V_t.block(3 * (p - 1), 3 * (i - 1), 3, 3) = pow(dt_p, i) * Matrix3d::Identity();
      }
    }

    // Get inverse
    MatrixXd V_t_inv = V_t.inverse();

    pData.V_t_inv = V_t_inv;

    // Compute coefficients of the polynomial
    VectorXd coeffs_ori = V_t_inv * diff_vec_ori;
    VectorXd coeffs_pos = V_t_inv * diff_vec_pos;

    pData.coeffs_ori = coeffs_ori;
    pData.coeffs_pos = coeffs_pos;

    MatrixXd dbth_dth0(3 * op->intr_order, 3);
    MatrixXd dbp_dp0(3 * op->intr_order, 3);

    for (int i = 0; i <= op->intr_order - 1; i++) {

      // Build jacobians
      Matrix3d R_GtoIi_temp = fej ? poses.at(i + 1)->Rot_fej() : poses.at(i + 1)->Rot();
      Matrix3d R_o_to_i = R_GtoIi_temp * R_GtoI0.transpose();
      Vector3d theta_o_to_i = log_so3(R_o_to_i);
      Matrix3d JlinOtoiInv = Jl_so3(theta_o_to_i).inverse();

      pData.JlinOtoiInv.emplace_back(JlinOtoiInv);

      dbth_dth0.block(3 * i, 0, 3, 3) = JlinOtoiInv * R_o_to_i;
      dbp_dp0.block(3 * i, 0, 3, 3) = -Matrix<double, 3, 3>::Identity();
    }

    pData.dbth_dth0 = dbth_dth0;
    pData.dbp_dp0 = dbp_dp0;
    polynomial.insert({times[0], pData});
  }
  if (fej)
    _polynomial_fej = polynomial;
  else
    _polynomial_est = polynomial;
}

void State::add_polynomial() {
  // polynomial with fej for Jacobian
  add_polynomial(true);
  // polynomial with best estimate for residual
  op->use_imu_res ? void() : add_polynomial(false);
}

void State::add_polynomial(bool fej) {
  // return if unable to build polynomial
  if ((int)clones.size() < op->intr_order + 1) {
    PRINT1(RED "[State] Cannot add polynomial. #clones: %d, #required: %d\n" RESET, (int)clones.size(), op->intr_order + 1);
    _polynomial_fej.clear();
    _polynomial_est.clear();
    return;
  }

  // Delete polynomial info we do not have clone with
  for (auto it = _polynomial_fej.begin(); it != _polynomial_fej.end();)
    clones.find(it->first) == clones.end() ? _polynomial_fej.erase(it++) : it++;
  for (auto it = _polynomial_est.begin(); it != _polynomial_est.end();)
    clones.find(it->first) == clones.end() ? _polynomial_est.erase(it++) : it++;

  // grab last poses
  auto it = clones.rbegin();
  vector<pair<double, shared_ptr<ov_type::PoseJPL>>> poses;
  for (int i = 0; i < op->intr_order + 1; i++) {
    poses.push_back({it->first, it->second});
    it++;
  }
  // reverse the order since it is new to old.
  reverse(poses.begin(), poses.end());

  // Compute
  Matrix3d R_GtoI0 = fej ? poses.at(0).second->Rot_fej() : poses.at(0).second->Rot();
  Vector3d p_I0inG = fej ? poses.at(0).second->pos_fej() : poses.at(0).second->pos();
  VectorXd diff_vec_ori(3 * op->intr_order, 1);
  VectorXd diff_vec_pos(3 * op->intr_order, 1);
  MatrixXd V_t(3 * op->intr_order, 3 * op->intr_order);
  MatrixXd dbth_dth0(3 * op->intr_order, 3);
  MatrixXd dbp_dp0(3 * op->intr_order, 3);
  vector<MatrixXd> JlinOtoiInv;
  for (int i = 0; i < op->intr_order; i++) {
    // compute relative pose
    Matrix3d R_GtoIi = fej ? poses.at(i + 1).second->Rot_fej() : poses.at(i + 1).second->Rot();
    Vector3d p_IiinG = fej ? poses.at(i + 1).second->pos_fej() : poses.at(i + 1).second->pos();
    Matrix3d R_I0toIi = R_GtoIi * R_GtoI0.transpose();
    Vector3d th_I0toIi = log_so3(R_I0toIi);

    // Get Do, Dp, Dt matrix
    diff_vec_ori.block(3 * i, 0, 3, 1) = th_I0toIi;
    diff_vec_pos.block(3 * i, 0, 3, 1) = p_IiinG - p_I0inG;
    for (int j = 0; j < op->intr_order; j++)
      V_t.block(3 * i, 3 * j, 3, 3) = pow(poses.at(i + 1).first - poses.at(0).first, j + 1) * Matrix3d::Identity();

    // Compute Jacobians
    MatrixXd JlinOtoi_inv = Jl_so3(th_I0toIi).inverse();
    JlinOtoiInv.push_back(JlinOtoi_inv);
    dbth_dth0.block(3 * i, 0, 3, 3) = JlinOtoi_inv * R_I0toIi;
    dbp_dp0.block(3 * i, 0, 3, 3) = -Matrix3d::Identity();
  }

  polynomial_data pData;
  pData.diff_vec_ori = diff_vec_ori;
  pData.diff_vec_pos = diff_vec_pos;
  pData.V_t_inv = V_t.inverse();
  pData.coeffs_ori = pData.V_t_inv * pData.diff_vec_ori;
  pData.coeffs_pos = pData.V_t_inv * pData.diff_vec_pos;
  pData.dbth_dth0 = dbth_dth0;
  pData.dbp_dp0 = dbp_dp0;
  pData.JlinOtoiInv = JlinOtoiInv;

  // insert polynomial
  fej ? _polynomial_fej[poses.at(0).first] = pData : _polynomial_est[poses.at(0).first] = pData;
}

bool State::get_interpolated_pose(double t_given, Matrix3d &RGtoI, Vector3d &pIinG) {
  return op->use_imu_res ? get_interpolated_pose_imu(t_given, RGtoI, pIinG) : get_interpolated_pose_poly(t_given, RGtoI, pIinG);
}

bool State::get_interpolated_pose_poly(double t_given, Matrix3d &RGtoI, Vector3d &pIinG) {
  // return if unable to build polynomial
  if ((int)clones.size() < op->intr_order + 1) {
    PRINT4(RED "[State]::get_interpolated_pose_poly Not enough clones for interpolation (%.3f)." RESET, t_given);
    PRINT4(RED "#clones: %d, #required: %d\n" RESET, (int)clones.size(), op->intr_order + 1);
    return false;
  }

  // Should have valid polynomial
  if (!check_polynomial(t_given))
    return false;

  // Get the poses that we will fit the n_order polynomial to
  vector<double> times;
  vector<shared_ptr<ov_type::PoseJPL>> poses;
  if (!bounding_poses_n(op->intr_order, poses, times, t_given))
    return false;

  // Ori and pos difference
  Matrix3d R_GtoI0 = poses.at(0)->Rot();
  Vector3d p_I0inG = poses.at(0)->pos();

  // Compute coefficients of the polynomial
  polynomial_data pData = _polynomial_est.at(times[0]);

  // Compute A_ori = coeffs_ori*m(t)
  //         A_pos = coeffs_pos*m(t)
  Vector3d A_ori = Vector3d::Zero();
  Vector3d A_pos = Vector3d::Zero();
  MatrixXd m_t(3, 3 * op->intr_order);
  double dt_m = t_given - times.at(0);
  for (int i = 0; i <= op->intr_order - 1; i++) {
    A_ori.noalias() += pow(dt_m, i + 1) * pData.coeffs_ori.block(3 * i, 0, 3, 1);
    A_pos.noalias() += pow(dt_m, i + 1) * pData.coeffs_pos.block(3 * i, 0, 3, 1);
    m_t.block(0, 3 * i, 3, 3) = pow(dt_m, i + 1) * Matrix3d::Identity();
  }

  // Get change of orientation and position into the global frame
  Matrix3d R_IotoIi = exp_so3(A_ori);
  RGtoI = R_IotoIi * R_GtoI0;
  pIinG = p_I0inG + A_pos;
  return true;
}

bool State::get_interpolated_pose_linear(double t_given, Matrix3d &RGtoI, Vector3d &pIinG) {
  double t0, t1;
  if (!bounding_times(t_given, t0, t1))
    return false;

  double lambda = (t_given - t0) / (t1 - t0);
  RGtoI = exp_so3(lambda * log_so3(clones[t1]->Rot() * clones[t0]->Rot().transpose())) * clones[t0]->Rot();
  pIinG = (1 - lambda) * clones[t0]->pos() + lambda * clones[t1]->pos();
  return true;
}

bool State::get_interpolated_pose_imu(double t_given, Matrix3d &RGtoI, Vector3d &pIinG) {
  // Get cpi
  if (!have_cpi(t_given)) {
    PRINT1(YELLOW "State::get_interpolated_pose_imu::Failed to find proper CPI (%.4f).\n" RESET, t_given);
    return false;
  }

  // Compute pose
  auto cpi = cpis.at(t_given);
  Matrix3d R_I0toIk = cpi.R_I0toIk;
  Vector3d alpha = cpi.alpha_I0toIk;
  Matrix3d RGtoI0 = clones.at(cpi.clone_t)->Rot();
  Vector3d pI0inG = clones.at(cpi.clone_t)->pos();
  Vector3d vI0inG = cpis.at(cpi.clone_t).v;
  RGtoI = R_I0toIk * RGtoI0;
  pIinG = pI0inG + vI0inG * cpi.dt - 0.5 * op->gravity * cpi.dt * cpi.dt + RGtoI0.transpose() * alpha;
  return true;
}

bool State::get_interpolated_jacobian(double t_given, Matrix3d &RGtoI, Vector3d &pIinG, string sensor, int id, vector<MatrixXd> &dTdx, vector<shared_ptr<ov_type::Type>> &order) {

  // return if unable to build polynomial
  if ((int)clones.size() < op->intr_order + 1) {
    PRINT0(YELLOW "[State]::get_interpolated_jacobian::Not enough clones for interpolation (%.3f). " RESET, t_given);
    PRINT0(YELLOW "#clones: %d, #required: %d\n" RESET, (int)clones.size(), op->intr_order + 1);
    return false;
  }

  // return the existing value if we already have it.
  if (have_jacobian(t_given, sensor, id)) {
    RGtoI = intr_jcb.at(sensor).at(id).at(t_given).RGtoI;
    pIinG = intr_jcb.at(sensor).at(id).at(t_given).pIinG;
    dTdx = intr_jcb.at(sensor).at(id).at(t_given).dTdx;
    order = intr_jcb.at(sensor).at(id).at(t_given).order;
    PRINT0(YELLOW "State::get_interpolated_jacobian::Get cashed Jacobian (%.3f)\n" RESET, t_given);
    return true;
  }

  if (t_given > newest_clone_time()) {
    PRINT0(YELLOW "State::get_interpolated_jacobian::Requested newer time than the state (t: %.3f, state %.3f)\n" RESET, t_given, newest_clone_time());
    return false;
  }

  if (op->use_imu_res && (t_given > prop->imu_data.back().timestamp)) {
    PRINT1(YELLOW "[State]::get_interpolated_jacobian::Out of CPI boundary (%.3f).\n", t_given);
    print_cpi_info();
    PRINT1(RESET);
    return false;
  }

  // Otherwise, create a new one
  if (!check_polynomial(t_given))
    return false;

  // Get the poses that we will fit the n_order polynomial to
  vector<double> times;
  vector<shared_ptr<ov_type::PoseJPL>> poses;
  if (!bounding_poses_n(op->intr_order, poses, times, t_given))
    return false;

  // Ori and pos difference
  Matrix3d R_GtoI0 = poses.at(0)->Rot_fej();
  Vector3d p_I0inG = poses.at(0)->pos_fej();

  // Compute coefficients of the polynomial
  polynomial_data pData = _polynomial_fej.at(times[0]);

  // Compute A_ori = coeffs_ori*m(t)
  //         A_pos = coeffs_pos*m(t)
  Vector3d A_ori = Vector3d::Zero();
  Vector3d A_pos = Vector3d::Zero();
  MatrixXd m_t(3, 3 * op->intr_order);
  double dt_m = t_given - times.at(0);
  for (int i = 0; i <= op->intr_order - 1; i++) {
    A_ori.noalias() += pow(dt_m, i + 1) * pData.coeffs_ori.block(3 * i, 0, 3, 1);
    A_pos.noalias() += pow(dt_m, i + 1) * pData.coeffs_pos.block(3 * i, 0, 3, 1);
    m_t.block(0, 3 * i, 3, 3) = pow(dt_m, i + 1) * Matrix<double, 3, 3>::Identity();
  }

  // Get change of orientation and position into the global frame
  Matrix3d R_IotoIi = exp_so3(A_ori);
  RGtoI = R_IotoIi * R_GtoI0;
  pIinG = p_I0inG + A_pos;

  // Compute Jacobian of this interpolated pose in respect to the state
  dTdx.clear();
  order.clear();

  // Base jacobian
  Matrix<double, 6, 6> H_0 = Matrix<double, 6, 6>::Zero();

  // Temp matrices
  MatrixXd m_tVinv = m_t * pData.V_t_inv;
  MatrixXd dth_db = -Jl_so3(A_ori) * m_tVinv;

  // Vector of Jacobians for the other poses
  vector<MatrixXd> H_other;
  for (int i = 0; i <= op->intr_order - 1; i++) {
    Matrix3d JlinOtoiInv = pData.JlinOtoiInv.at(i);
    Matrix<double, 6, 6> H_i = Matrix<double, 6, 6>::Zero();
    H_i.block(0, 0, 3, 3) = -dth_db.block(0, 3 * i, 3, 3) * JlinOtoiInv;
    H_i.block(3, 3, 3, 3) = m_tVinv.block(0, 3 * i, 3, 3);
    H_other.push_back(H_i);
  }
  assert((int)H_other.size() == op->intr_order);

  // Base pose jacobian
  H_0.block(0, 0, 3, 3) = dth_db * pData.dbth_dth0 + R_IotoIi;
  H_0.block(3, 3, 3, 3) = Matrix<double, 3, 3>::Identity() + m_tVinv * pData.dbp_dp0;
  dTdx.push_back(H_0);
  order.push_back(poses.at(0));

  // Push back the other poses
  for (size_t i = 0; i < H_other.size(); i++) {
    dTdx.push_back(H_other.at(i));
    order.push_back(poses.at(i + 1));
  }

  // Check if we want to do time offset calibration on this sensor
  shared_ptr<ov_type::Vec> dt;
  bool calib_dt = false;
  if (sensor == "CAM") {
    if (op->cam->do_calib_dt) {
      calib_dt = true;
      dt = cam_dt.at(id);
    }
  } else if (sensor == "GPS") {
    if (op->gps->do_calib_dt) {
      calib_dt = true;
      dt = gps_dt.at(id);
    }

  } else if (sensor == "LIDAR") {
    if (op->lidar->do_calib_dt) {
      calib_dt = true;
      dt = lidar_dt.at(id);
    }

  } else if (sensor == "VICON") {
    if (op->vicon->do_calib_dt) {
      calib_dt = true;
      dt = vicon_dt.at(id);
    }
  } else {
    PRINT4(RED "State::get_interpolated_pose::Jacobian request on wrong sensor: %s.\n" RESET, sensor.c_str());
    exit(EXIT_FAILURE);
  }

  // Jacobian wrt time offset
  if (calib_dt) {
    // Derivative of m(t) wrt time offset
    Matrix<double, 6, 1> dpose_dtoff = Matrix<double, 6, 1>::Zero();
    MatrixXd d_mt_dt = MatrixXd::Zero(3, 3 * op->intr_order);
    for (int i = 0; i <= op->intr_order - 1; i++)
      d_mt_dt.block(0, 3 * i, 3, 3) = (double)(i + 1) * pow(dt_m, i) * Matrix<double, 3, 3>::Identity();
    dpose_dtoff.block(0, 0, 3, 1) = -Jl_so3(A_ori) * d_mt_dt * pData.coeffs_ori;
    dpose_dtoff.block(3, 0, 3, 1) = d_mt_dt * pData.coeffs_pos;

    // Add to Jacobians in respect to the cam-to-basecam time offset
    dTdx.push_back(dpose_dtoff);
    order.push_back(dt);
  }

  // Record for later faster use
  intr_jcb[sensor][id][t_given].RGtoI = RGtoI;
  intr_jcb[sensor][id][t_given].pIinG = pIinG;
  intr_jcb[sensor][id][t_given].dTdx = dTdx;
  intr_jcb[sensor][id][t_given].order = order;
  intr_jcb[sensor][id][t_given].times = times;
  return true;
}

bool State::bounding_times(double t_given, double &t0, double &t1) {
  // return if we have less than 2 clones
  if (clones.size() < 2)
    return false;

  // return if out of clone window
  if (t_given < clones.begin()->first - op->dt_exp || t_given > clones.rbegin()->first + op->dt_exp)
    return false;

  // Get the times of our pose clones
  vector<double> clone_times;
  clone_times.reserve(clones.size());
  for (auto &clone : clones)
    clone_times.push_back(clone.first);

  for (int i = 0; i < (int)clone_times.size() - 1; i++) {
    if (clone_times.at(i) - op->dt_exp <= t_given && t_given <= clone_times.at(i + 1) + op->dt_exp) {
      t0 = clone_times.at(i);
      t1 = clone_times.at(i + 1);
      return true;
    }
  }
  // failed to find bound
  return false;
}

bool State::bounding_poses_n_check(size_t n_order, double meas_t) {
  return (clones.begin()->first - op->dt_exp <= meas_t && meas_t <= clones.rbegin()->first + op->dt_exp && clones.size() >= n_order + 1);
}

bool State::bounding_poses_n(size_t n_order, vector<shared_ptr<ov_type::PoseJPL>> &poses, vector<double> &times, double t_given) {

  // Only support odd orders for now
  assert((n_order + 1) % 2 == 0);

  // Clear old data
  poses.clear();
  times.clear();
  poses.reserve(n_order + 1);
  poses.reserve(n_order + 1);

  // Get the times of our pose clones
  // TODO: can definitely can improve this and use the map
  vector<double> clone_times;
  for (auto &clone : clones) {
    clone_times.push_back(clone.first);
  }
  sort(clone_times.begin(), clone_times.end());

  // Return if we don't have enough poses
  if (clone_times.size() < n_order + 1)
    return false;

  // Grab the closest older and newer clones that bound the state
  double t_b, t_e;
  if (!bounding_times(t_given, t_b, t_e)) {
    PRINT1("[n_bounding] cannot find the bounding pose\n");
    return false;
  }

  shared_ptr<ov_type::PoseJPL> x_b = clones.at(t_b);
  shared_ptr<ov_type::PoseJPL> x_e = clones.at(t_e);

  //========================================================
  //========================================================

  // Find where the bounding poses are
  int n_b = -1;
  int n_e = -1;
  for (int i = 0; i < (int)clone_times.size(); i++) {
    if (clone_times[i] == t_b) {
      n_b = i;
    } else if (clone_times[i] == t_e) {
      n_e = i;
    }
  }

  // Return if we couldn't find them
  if (n_b == -1 || n_e == -1) {
    PRINT3("[n_bounding]: bad index | n_b %d | n_e %d | t_b %.15f | t_e %.15f\n", (int)n_b, (int)n_e, t_b, t_e);
    return false;
  }

  // Number of poses on either side of the interpolated one
  int n_side = (int)((double)(n_order + 1) / 2.0);

  // Start index of the bounding clones
  // Ideally we are evenly space over interpolated pose
  size_t start_index = n_b - n_side + 1;

  // If not enough clones earlier, start from oldest clone
  // If not enough clones newer, end on oldest clone
  if (n_b - n_side + 1 < 0) {
    start_index = 0;
  } else if (n_e + n_side - 1 >= (int)clone_times.size()) {
    start_index = clone_times.size() - 1 - n_order;
  }

  // Return if bad bounds
  if (start_index < 0 || start_index + 1 + n_order > clone_times.size()) {
    PRINT3("[n_bounding]: bad bounds | start_index %d | start_index+1+n_order %d | clone_times.size() %d\n", (int)start_index, (int)(start_index + 1 + n_order),
           (int)clone_times.size());
    return false;
  }

  // Grab the pose timestamps that bound us
  for (size_t i = start_index; i < start_index + 1 + n_order; i++) {
    times.push_back(clone_times.at(i));
    poses.push_back(clones.at(clone_times.at(i)));
  }

  // Return success
  return true;
}

double State::oldest_keyframe_time() {
  if (clones.empty())
    return -INFINITY;

  return clones.begin()->first;
}

double State::oldest_clone_time() {
  if (clones.empty())
    return -INFINITY;

  for (const auto &clone : clones) {
    if (find(keyframes.begin(), keyframes.end(), clone.first) == keyframes.end())
      return clone.first;
  }
  return -INFINITY;
}

double State::oldest_2nd_clone_time() {
  if (clones.size() < 2)
    return -INFINITY;

  double t_2nd;
  bool success = closest_newer_clone_time(oldest_clone_time(), t_2nd);
  assert(success);
  return t_2nd;
}

double State::oldest_3rd_clone_time() {
  if (clones.size() < 3)
    return -INFINITY;

  double t_3rd;
  bool success = closest_newer_clone_time(oldest_2nd_clone_time(), t_3rd);
  assert(success);
  return t_3rd;
}

double State::newest_clone_time() {
  if (clones.empty())
    return -INFINITY;

  return clones.rbegin()->first;
}

double State::newest_2nd_clone_time() {
  if (clones.size() < 2)
    return -INFINITY;

  double t_2nd;
  bool success = closest_older_clone_time(newest_clone_time(), t_2nd);
  assert(success);
  return t_2nd;
}

bool State::closest_clone_time(double t_given, double &t) {
  if (clones.empty())
    return false;

  double t_diff = INFINITY;
  bool found = false;
  for (pair<const double, shared_ptr<ov_type::PoseJPL>> &clone : clones) {
    if (abs(t_given - clone.first) < t_diff) {
      t_diff = abs(t - clone.first);
      t = clone.first;
      found = true;
    }
  }
  return found;
}

bool State::closest_clone_time_not_imu(double t_given, double &t) {
  if (clones.empty())
    return false;

  double t_diff = INFINITY;
  bool found = false;
  for (pair<const double, shared_ptr<ov_type::PoseJPL>> &clone : clones) {
    if (abs(t_given - clone.first) < t_diff && clone.second->id() != imu->id()) {
      t_diff = abs(t - clone.first);
      t = clone.first;
      found = true;
    }
  }
  return found;
}

bool State::closest_older_clone_time(double t_given, double &t) {
  if (clones.empty())
    return false;

  t = -INFINITY;
  bool found = false;
  for (pair<const double, shared_ptr<ov_type::PoseJPL>> &clone : clones) {
    if (clone.first > t && clone.first < t_given) {
      t = clone.first;
      found = true;
    }
  }
  return found;
}

bool State::closest_newer_clone_time(double t_given, double &t) {
  if (clones.empty())
    return false;

  t = INFINITY;
  bool found = false;
  for (pair<const double, shared_ptr<ov_type::PoseJPL>> &clone : clones) {
    if (clone.first < t && clone.first > t_given) {
      t = clone.first;
      found = true;
    }
  }
  return found;
}

void State::print_info() {
  // IMU
  PRINT2("IMU state[%.4f(%d)], ", time, imu->id());
  int state_size = 15;

  // Clones
  PRINT2("%d clones[", clones.size());
  for (auto c : clones) {
    PRINT2("%.4f(%d), ", c.first, c.second->id());
    if (c.second->id() != imu->id())
      state_size += 6;
  }
  PRINT2("\b\b], ");

  // Key frames
  if (!keyframes.empty()) {
    PRINT2("%d keyframes[", keyframes.size());
    for (auto c : keyframes)
      PRINT2("%.3f(%d), ", c, clones.at(c)->id());
    PRINT2("\b\b], ");
  }

  // Key frames
  if (!keyframes.empty()) {
    PRINT2("%d keyframes_candidate[", keyframes_candidate.size());
    for (auto c : keyframes_candidate)
      PRINT2("%.3f(%d), ", c, clones.at(c)->id());
    PRINT2("\b\b], ");
  }

  // Cam
  if (op->cam->enabled) {
    if (!cam_SLAM_features.empty() || op->cam->do_calib_ext || op->cam->do_calib_int || op->cam->do_calib_dt)
      PRINT2("%d Cam[", op->cam->max_n);
    if (!cam_SLAM_features.empty()) {
      PRINT2("%d feat[", cam_SLAM_features.size());
      for (auto c : cam_SLAM_features)
        PRINT2("%d(%d), ", c.first, c.second->id());
      PRINT2("\b\b], ");
      state_size += cam_SLAM_features.size() * 3;
    }
    if (op->cam->do_calib_ext) {
      PRINT2("ext(");
      for (auto c : cam_extrinsic)
        PRINT2("%d, ", c.second->id());
      PRINT2("\b\b), ");
      state_size += op->cam->max_n * 6;
    }
    if (op->cam->do_calib_int) {
      PRINT2("int(");
      for (auto c : cam_intrinsic)
        PRINT2("%d, ", c.second->id());
      PRINT2("\b\b), ");
      state_size += op->cam->max_n * 8;
    }
    if (op->cam->do_calib_dt) {
      PRINT2("dt(");
      for (auto c : cam_dt)
        PRINT2("%d, ", c.second->id());
      PRINT2("\b\b), ");
      state_size += op->cam->max_n * 1;
      state_size -= op->cam->stereo_pairs.size() / 2;
    }
    if (!cam_SLAM_features.empty() || op->cam->do_calib_ext || op->cam->do_calib_int || op->cam->do_calib_dt)
      PRINT2("\b\b], ");
  }
  if (op->gps->enabled) {
    if (op->gps->do_calib_ext || op->gps->do_calib_dt)
      PRINT2("%d GPS[", op->gps->max_n);
    if (op->gps->do_calib_ext) {
      PRINT2("ext(");
      for (auto c : gps_extrinsic)
        PRINT2("%d, ", c.second->id());
      PRINT2("\b\b), ");
      state_size += op->gps->max_n * 3;
    }
    if (op->gps->do_calib_dt) {
      PRINT2("dt(");
      for (auto c : gps_dt)
        PRINT2("%d, ", c.second->id());
      PRINT2("\b\b), ");
      state_size += op->gps->max_n * 1;
    }
    if (op->gps->do_calib_ext || op->gps->do_calib_dt)
      PRINT2("\b\b], ");
  }
  if (op->wheel->enabled) {
    if (op->wheel->do_calib_ext || op->wheel->do_calib_int || op->wheel->do_calib_dt)
      PRINT2("%d Wheel[", op->gps->max_n);
    if (op->wheel->do_calib_ext) {
      PRINT2("ext(%d), ", wheel_extrinsic->id());
      state_size += 6;
    }
    if (op->wheel->do_calib_int) {
      PRINT2("int(%d), ", wheel_intrinsic->id());
      state_size += 3;
    }
    if (op->wheel->do_calib_dt) {
      PRINT2("dt(%d), ", wheel_dt->id());
      state_size += 1;
    }
    if (op->wheel->do_calib_ext || op->wheel->do_calib_int || op->wheel->do_calib_dt)
      PRINT2("\b\b], ");
  }
  if (op->lidar->enabled) {
    if (op->lidar->do_calib_ext || op->lidar->do_calib_dt)
      PRINT2("%d LiDAR[", op->lidar->max_n);
    if (op->lidar->do_calib_ext) {
      PRINT2("ext(");
      for (auto c : lidar_extrinsic)
        PRINT2("%d, ", c.second->id());
      PRINT2("\b\b), ");
      state_size += op->lidar->max_n * 6;
    }
    if (op->lidar->do_calib_dt) {
      PRINT2("dt(");
      for (auto c : lidar_dt)
        PRINT2("%d, ", c.second->id());
      PRINT2("\b\b), ");
      state_size += op->lidar->max_n * 1;
    }
    if (op->lidar->do_calib_ext || op->lidar->do_calib_dt)
      PRINT2("\b\b], ");
  }
  if (op->vicon->enabled) {
    if (op->vicon->do_calib_ext || op->vicon->do_calib_dt)
      PRINT2("%d VICON[", op->vicon->max_n);
    if (op->vicon->do_calib_ext) {
      PRINT2("ext(");
      for (auto c : vicon_extrinsic)
        PRINT2("%d, ", c.second->id());
      PRINT2("\b\b), ");
      state_size += op->vicon->max_n * 6;
    }
    if (op->vicon->do_calib_dt) {
      PRINT2("dt(");
      for (auto c : vicon_dt)
        PRINT2("%d, ", c.second->id());
      PRINT2("\b\b), ");
      state_size += op->vicon->max_n * 1;
    }
    if (op->vicon->do_calib_ext || op->vicon->do_calib_dt)
      PRINT2("\b\b], ");
  }
  PRINT2("Total: %d\n", state_size);
  if (state_size != cov.cols()) {
    PRINT4(RED "state_size (%d) != cov.cols() (%d) \n" RESET, state_size, cov.cols());
    PRINT4(RED "IDs in the state: ");
    for (auto var : variables)
      PRINT4(RED "%d, ", var->id());
    PRINT4(RESET "\n");
    //    exit(EXIT_FAILURE);
  }
}

void State::print_poly_info() {
  PRINT2("%d Polynomial fej anchors[", _polynomial_fej.size());
  for (auto p : _polynomial_fej)
    PRINT2("%.4f,", p.first);
  PRINT2("]\n");
  PRINT2("%d Polynomial est anchors[", _polynomial_est.size());
  for (auto p : _polynomial_est)
    PRINT2("%.4f,", p.first);
  PRINT2("]\n");
}

void State::print_cpi_info() { PRINT2("%d cpi [%.4f - %.4f]\n", cpis.size(), cpis.begin()->first, cpis.rbegin()->first); }

bool State::have_polynomial() { return !_polynomial_fej.empty(); }

double State::clone_window() {
  if (clones.empty())
    return 0.0;

  return newest_clone_time() - oldest_clone_time();
}

MatrixXd State::intr_pose_cov(int hz, int order) { return op->intr_err.pose_cov(hz, order, est_A->mean, est_a->mean); }
double State::intr_ori_cov(int hz, int order) { return op->intr_err.ori_cov(hz, order, est_A->mean); }
double State::intr_pos_cov(int hz, int order) { return op->intr_err.pos_cov(hz, order, est_a->mean); }
double State::intr_ori_std(int hz, int order) { return op->intr_err.ori_std(hz, order, est_A->mean); }
double State::intr_pos_std(int hz, int order) { return op->intr_err.pos_std(hz, order, est_a->mean); }

bool State::have_clone(double t_given) { return clones.find(t_given) != clones.end(); }

bool State::have_cpi(double t_given) {
  // return true if we have cpi at requested time
  if (cpis.find(t_given) != cpis.end() && clones.find(cpis.at(t_given).clone_t) != clones.end()) {
    return true;
  }

  // If not, try to create a new one with linear interpolation
  if (create_new_cpi_linear(t_given)) {
    return true;
  }

  // If failed, try to create a new one by integration.
  return create_new_cpi_integrate(t_given);
}

bool State::create_new_cpi_linear(double t_given) {
  // make sure we do not have CPI at requested time.
  if (cpis.find(t_given) != cpis.end() && clones.find(cpis.at(t_given).clone_t) != clones.end()) {
    PRINT4(RED "State::create_new_cpi_linear:CPI creation requested at already exising time (%.4f)!!!!!\n. " RESET, t_given);
    return true;
  }

  if (cpis.empty())
    return false;

  if (t_given < cpis.begin()->first) {
    PRINT0(YELLOW "State::create_new_cpi_linear::The requested time (%.4f) is out of bound" RESET, t_given);
    PRINT0(YELLOW " %.20f < %.20f (%.20f - %.20f)\n" RESET, t_given, cpis.begin()->first, cpis.rbegin()->first);
    return false;
  }

  if (t_given > cpis.rbegin()->first) {
    PRINT0(YELLOW "State::create_new_cpi_linear::The requested time (%.4f) is out of bound" RESET, t_given);
    PRINT0(YELLOW " (%.20f - %.20f) < %.20f\n" RESET, cpis.begin()->first, cpis.rbegin()->first, t_given);
    return false;
  }

  // Get equal or lower bounding CPI
  double t0;
  if (t_given == cpis.begin()->first) {
    t0 = t_given;
  } else {
    auto i0 = --(cpis.lower_bound(t_given));
    assert(i0 != cpis.end()); // TODO: remove this?
    t0 = i0->first;
  }

  // Get equal or upper bounding CPI
  double t1;
  if (t_given == cpis.rbegin()->first) {
    t1 = t_given;
  } else {
    auto i1 = cpis.upper_bound(t_given);
    assert(i1 != cpis.end()); // TODO: remove this?
    t1 = i1->first;
  }

  auto cpi0 = cpis.at(t0);
  auto cpi1 = cpis.at(t1);

  // Check if they are integrated from the same clone
  if (cpi0.clone_t != cpi1.clone_t) {
    PRINT0(YELLOW "State::create_new_cpi_linear::Clones of the bounding CPIs don't match for requested time (%.4f). " RESET, t_given);
    PRINT0(YELLOW "Bounding CPI0: %.4f (clone: %.4f),  CPI1: %.4f (clone: %.4f)\n" RESET, cpi0.t, cpi0.clone_t, cpi0.t, cpi1.clone_t);
    return false;
  }

  if (cpi0.clone_t < oldest_clone_time()) {
    PRINT0(YELLOW "State::create_new_cpi_linear::CPI clone time is older than the state oldest clone" RESET);
    PRINT0(YELLOW " (%.4f < %.4f).\n" RESET, cpi0.clone_t, oldest_clone_time());
    return false;
  }

  // create a new cpi
  CPI cpi_new;
  cpi_new.t = t_given;
  cpi_new.dt = t_given - cpi0.clone_t;
  cpi_new.clone_t = cpi0.clone_t;
  double lambda = (t_given - t0) / (t1 - t0);
  cpi_new.R_I0toIk = exp_so3(lambda * log_so3(cpi1.R_I0toIk * cpi0.R_I0toIk.transpose())) * cpi0.R_I0toIk;
  cpi_new.alpha_I0toIk = (1 - lambda) * cpi0.alpha_I0toIk + lambda * cpi1.alpha_I0toIk;
  cpi_new.v = (1 - lambda) * cpi0.v + lambda * cpi1.v;
  cpi_new.Q = (1 - lambda) * cpi0.Q + lambda * cpi1.Q;
  cpi_new.bg = (1 - lambda) * cpi0.bg + lambda * cpi1.bg;
  cpi_new.ba = (1 - lambda) * cpi0.ba + lambda * cpi1.ba;
  cpis[t_given] = cpi_new;
  return true;
}

bool State::create_new_cpi_integrate(double t_given) {
  // make sure we do not have CPI at requested time.
  if (cpis.find(t_given) != cpis.end() && clones.find(cpis.at(t_given).clone_t) != clones.end()) {
    PRINT4(RED "State::create_new_cpi:CPI creation requested at already exising time (%.4f)!!!!!\n. " RESET, t_given);
    return true;
  }

  if (cpis.empty())
    return false;

  double clone_t;
  if (!closest_clone_time_not_imu(t_given, clone_t)) {
    PRINT2(YELLOW "State::create_new_cpi:Cannot find clone for requested time %.4f\n" RESET, t_given);
    return false;
  }

  // Get IMU data to compute CPI with
  vector<ImuData> imu_data;
  if (clone_t <= t_given ? !prop->select_imu_readings(clone_t, t_given, imu_data) : !prop->select_imu_readings(t_given, clone_t, imu_data))
    return false;

  // reverse the data vector because we need to propagate backward
  clone_t <= t_given ? void() : reverse(imu_data.begin(), imu_data.end());

  // Init a new CPI
  CpiV1 cpiv1(op->imu->sigma_w, op->imu->sigma_wb, op->imu->sigma_a, op->imu->sigma_ab, true);
  cpiv1.setLinearizationPoints(cpis.at(clone_t).bg, cpis.at(clone_t).ba);
  Matrix3d R_GtoIk = clones.at(clone_t)->Rot();

  // Compute CPI recursively toward retested t
  for (size_t i = 0; i < imu_data.size() - 1; i++) {
    R_GtoIk = cpiv1.R_k2tau * R_GtoIk; // integrate orientation
    cpiv1.feed_IMU(imu_data[i].timestamp, imu_data[i + 1].timestamp, imu_data[i].wm, imu_data[i].am, imu_data[i + 1].wm, imu_data[i + 1].am);
  }

  // create a new cpi
  CPI cpi_new;
  cpi_new.t = t_given;
  cpi_new.dt = t_given - clone_t;
  cpi_new.clone_t = clone_t;
  cpi_new.R_I0toIk = cpiv1.R_k2tau;
  cpi_new.alpha_I0toIk = cpiv1.alpha_tau;
  cpi_new.w = imu_data.back().wm - cpis.at(clone_t).bg;
  cpi_new.v = cpis.at(clone_t).v - op->gravity * cpiv1.DT + R_GtoIk.transpose() * cpiv1.beta_tau;
  cpi_new.bg = cpis.at(clone_t).bg;
  cpi_new.ba = cpis.at(clone_t).ba;
  cpi_new.Q.block(0, 0, 3, 3) = cpiv1.P_meas.block(0, 0, 3, 3);
  cpi_new.Q.block(0, 3, 3, 3) = cpiv1.P_meas.block(0, 12, 3, 3);
  cpi_new.Q.block(3, 0, 3, 3) = cpiv1.P_meas.block(12, 0, 3, 3);
  cpi_new.Q.block(3, 3, 3, 3) = cpiv1.P_meas.block(12, 12, 3, 3);
  cpis[t_given] = cpi_new;
  PRINT1(CYAN "State::create_new_cpi_integrate insert new: cpi: %.4f, clone_t: %.4f\n" RESET, t_given, clone_t);
  return true;
}

bool State::have_jacobian(double t_given, string sensor, int id) {
  // Check the sensor key
  if (intr_jcb.find(sensor) != intr_jcb.end()) {
    // Check the sensor id key
    if (intr_jcb.at(sensor).find(id) != intr_jcb.at(sensor).end()) {
      // Check the measurement time key
      if (intr_jcb.at(sensor).at(id).find(t_given) != intr_jcb.at(sensor).at(id).end()) {
        // Check the clone times key
        for (auto clone_time : intr_jcb.at(sensor).at(id).at(t_given).times) {
          // Return false if we have clone time mismatch.
          if (clones.find(clone_time) == clones.end()) {
            PRINT0(YELLOW "State::have_jacobian: Not have clone(%.4f) for sensor(%s) id(%d) t(%.4f)\n" RESET, clone_time, sensor.c_str(), id, t_given);
            return false;
          }
        }
        // Return true. All good.
        return true;
      } else { // We do not have key for the measurement time. Create one
        PRINT0(YELLOW "State::have_jacobian: Not have key for sensor(%s) id(%d) t(%.4f)\n" RESET, sensor.c_str(), id, t_given);
      }
    } else { // We do not have key for the sensor id. Create one
      PRINT0(YELLOW "State::have_jacobian: Not have key for sensor(%s) id(%d) \n" RESET, sensor.c_str(), id);
    }
  } else { // We do not have key for the sensor. Create one
    PRINT0(YELLOW "State::have_jacobian: Not have key sensor(%s)\n" RESET, sensor.c_str());
  }
  return false;
}
void State::flush_old_data() {
  // Flush old data for faster std::map usage
  double old_t = clones.begin()->first;
  // Flush old jacobian data
  for (auto &sensor : intr_jcb) {
    for (auto &id : sensor.second) {
      for (auto data = id.second.begin(); data != id.second.end();) {
        if (old_t > data->first + op->dt_exp) {
          data = id.second.erase(data);
        } else {
          break;
        }
      }
    }
  }
  // Flush old cpi data
  for (auto data = cpis.begin(); data != cpis.end();) {
    if (old_t > data->second.clone_t) {
      data = cpis.erase(data);
    } else {
      break;
    }
  }
}

void State::hook_propagator(shared_ptr<Propagator> p_ptr) { prop = p_ptr; }

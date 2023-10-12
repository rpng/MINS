/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
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

#include "UpdaterLidar.h"
#include "LidarHelper.h"
#include "LidarTypes.h"
#include "ikd_Tree.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsLidar.h"
#include "pcl/point_cloud.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "update/UpdaterStatistics.h"
#include "utils/Print_Logger.h"
#include "utils/colors.h"
#include <thread>

using namespace std;
using namespace mins;
using namespace ov_core;
using namespace ov_type;

UpdaterLidar::UpdaterLidar(shared_ptr<State> state) : state(state) {
  for (int i = 0; i < state->op->lidar->max_n; i++) {
    Chi.emplace_back(make_shared<UpdaterStatistics>(state->op->lidar->chi2_mult, "LIDAR", i));
    t_hist.insert({i, deque<double>()});
    ikd_data.push_back(make_shared<iKDDATA>(i));
  }
}

void UpdaterLidar::feed_measurement(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> lidar) {
  // Record first ever measurement time to set up reference time (ref. LidarTypes.h)
  FT < 0 ? FT = (double)lidar->header.stamp / 1000 : double();

  // record timestamps
  int id = stoi(lidar->header.frame_id);
  double lidar_t = (double)lidar->header.stamp / 1000;
  t_hist.at(id).size() > 100 ? t_hist.at(id).pop_front() : void(); // remove if we have too many
  t_hist.at(id).push_back(lidar_t);

  // Append the raw pointcloud
  stack_lidar_raw.push_back(std::make_shared<LiDARData>(lidar_t, FT, id, lidar, state->op->lidar->max_range, state->op->lidar->min_range));

  // Check raw lidar stack and construct lidar data
  for (auto it = stack_lidar_raw.begin(); it != stack_lidar_raw.end();) {
    if (state->clones.empty())
      break;

    // Discard this measurement if it is older than the oldest clone time
    if ((*it)->time + state->lidar_dt.at((*it)->id)->value()(0) < state->oldest_clone_time()) {
      it = stack_lidar_raw.erase(it);
      continue;
    }

    // Skip if it is newer than the newest clone time.
    // Add 0.05 to delay processing this measurement for more stable update
    if ((*it)->time + state->lidar_dt.at((*it)->id)->value()(0) + 0.05 > state->newest_clone_time()) {
      it++;
      continue;
    }

    // remove motion blur if enabled
    if (!LidarHelper::remove_motion_blur(state, (*it), state->op->lidar)) {
      it++;
      continue;
    }

    // Do down sample the pointcloud if enabled
    state->op->lidar->raw_do_downsample ? LidarHelper::downsample((*it), state->op->lidar->raw_downsample_size) : void();

    // Initialize ikd map if not initialized OR scan registration failed more than 1 second
    if (!ikd_data.at((*it)->id)->tree->initialized() || ikd_data.at((*it)->id)->last_up_time + 1 < (*it)->time) {
      LidarHelper::init_map_local((*it), ikd_data.at((*it)->id), state->op->lidar);
      it = stack_lidar_raw.erase(it);
      continue;
    }

    // All good, move this to stack to be processed.
    stack_lidar_new.push_back((*it));
    it = stack_lidar_raw.erase(it);
  }

  // Erase invalid dataset
  erase_invalid_dataset();
}

void UpdaterLidar::try_update() {
  // Loop through stacked lidar and process them
  for (auto lidar = stack_lidar_new.begin(); lidar != stack_lidar_new.end();) {
    // Try update
    if (update(*lidar, ikd_data.at((*lidar)->id), Chi.at((*lidar)->id))) {
      // move used lidar from new -> used
      stack_lidar_used.push_back(*lidar);
      lidar = stack_lidar_new.erase(lidar);
    } else {
      // keep it if currently unusable
      lidar++;
    }
  }

  // Register used pointcloud in the map
  for (auto lidar = stack_lidar_used.begin(); lidar != stack_lidar_used.end();) {
    LidarHelper::register_scan(state, *lidar, ikd_data.at((*lidar)->id));
    lidar = stack_lidar_used.erase(lidar);
  }
}

void UpdaterLidar::erase_invalid_dataset() {
  // Erase too old raw
  for (auto old_lidar = stack_lidar_raw.begin(); old_lidar != stack_lidar_raw.end();) {
    double dt = state->lidar_dt.at((*old_lidar)->id)->value()(0);
    (*old_lidar)->time + dt < state->oldest_clone_time() ? old_lidar = stack_lidar_raw.erase(old_lidar) : old_lidar++;
  }

  // Erase too old new
  for (auto old_lidar = stack_lidar_new.begin(); old_lidar != stack_lidar_new.end();) {
    double dt = state->lidar_dt.at((*old_lidar)->id)->value()(0);
    (*old_lidar)->time + dt < state->oldest_clone_time() ? old_lidar = stack_lidar_new.erase(old_lidar) : old_lidar++;
  }

  // Erase too old used
  for (auto old_lidar = stack_lidar_used.begin(); old_lidar != stack_lidar_used.end();) {
    double dt = state->lidar_dt.at((*old_lidar)->id)->value()(0);
    (*old_lidar)->time + dt < state->oldest_clone_time() ? old_lidar = stack_lidar_used.erase(old_lidar) : old_lidar++;
  }
}

void UpdaterLidar::propagate_map_frame() {
  // check if map is older than the clone to be marginalized
  for (int i = 0; i < (int)ikd_data.size(); i++) {
    if (!ikd_data.at(i)->tree->initialized())
      continue;

    // Skip if we do not have enough clones to perform propagation
    if (state->clones.size() < state->op->intr_order + 1)
      continue;

    if (ikd_data.at(i)->time + state->lidar_dt.at(i)->value()(0) <= state->oldest_3rd_clone_time())
      LidarHelper::propagate_map_to_newest_clone(state, ikd_data.at(i), state->op->lidar, FT);
  }
}

/// Loop points to find match, compute linear system, and update
bool UpdaterLidar::update(std::shared_ptr<LiDARData> lidar, shared_ptr<iKDDATA> ikd, shared_ptr<UpdaterStatistics> chi) {
  //     *** How frames look like ***
  //  L(Map at pLinA)                   L
  //  |                                 |
  //  A (Anchor, IMU clone)-- C -- C -- I
  //===================================================================
  // Get State Involved
  //===================================================================
  vector<shared_ptr<ov_type::Type>> H_order;
  int total_hx = 0;
  unordered_map<shared_ptr<Type>, size_t> map_hx;

  // Our calibrations
  shared_ptr<PoseJPL> calibration = state->lidar_extrinsic.at(lidar->id);
  shared_ptr<Vec> timeoffset = state->lidar_dt.at(lidar->id);
  double dt = timeoffset->value()(0);

  // Add to state map if doing calibration
  state->op->lidar->do_calib_ext ? StateHelper::insert_map({calibration}, map_hx, H_order, total_hx) : void();
  state->op->lidar->do_calib_dt ? StateHelper::insert_map({timeoffset}, map_hx, H_order, total_hx) : void();

  // Get the relevant states to process lidar_inL
  Matrix3d RGtoI_est, RGtoA_est, RGtoI_fej, RGtoA_fej;
  Vector3d pIinG_est, pAinG_est, pIinG_fej, pAinG_fej;
  vector<MatrixXd> dTdxI, dTdxA;
  VecTypePtr orderI, orderA;

  if (!state->get_interpolated_jacobian(lidar->time + dt, RGtoI_fej, pIinG_fej, "LIDAR", lidar->id, dTdxI, orderI))
    return false;

  // Add to state map
  StateHelper::insert_map(orderI, map_hx, H_order, total_hx);

  // Get the relevant states to process Map
  if (!state->get_interpolated_jacobian(ikd->time + dt, RGtoA_fej, pAinG_fej, "LIDAR", lidar->id, dTdxA, orderA)) {
    PRINT4(RED);
    PRINT4("[LiDAR]get_linsys::Cannot get the bounding_poses_n for Map %d!\n", lidar->id);
    PRINT4("[LiDAR]get_linsys::ikd_anchor_time.at(lidar_inL->id): %.4f\n", ikd->time);
    PRINT4("[LiDAR]get_linsys::dt: %.4f\n", dt);
    PRINT4("[LiDAR]get_linsys::lidar->time + dt: %.4f\n", ikd->time + dt);
    state->print_info();
    PRINT4("[LiDAR]get_linsys::state->oldest_clone_time(): %.4f\n", state->oldest_clone_time());
    PRINT4(RESET);
    if (ikd->time + dt > state->oldest_clone_time()) {
      cout << "ikd->time + dt > state->oldest_clone_time()" << endl;
      return false;
    } else
      std::exit(EXIT_FAILURE);
  }

  // Add to state map
  StateHelper::insert_map(orderA, map_hx, H_order, total_hx);

  state->get_interpolated_pose(lidar->time + dt, RGtoI_est, pIinG_est);
  state->get_interpolated_pose(ikd->time + dt, RGtoA_est, pAinG_est);

  //=========================================================================
  // Precomputed Transformations
  //=========================================================================
  // calib info
  Matrix3d RItoL_est = calibration->Rot();
  Matrix3d RLtoI_est = RItoL_est.transpose();
  Vector3d pIinL_est = calibration->pos();
  Vector3d pLinI_est = -RLtoI_est * pIinL_est;
  Matrix3d RItoL_fej = calibration->Rot();
  Matrix3d RLtoI_fej = RItoL_fej.transpose();
  Vector3d pIinL_fej = calibration->pos();
  Vector3d pLinI_fej = -RLtoI_fej * pIinL_fej;

  // New LiDAR frame
  Matrix3d RGtoL_est = RItoL_est * RGtoI_est;
  Matrix3d RLtoG_est = RGtoL_est.transpose();
  Vector3d pLinG_est = pIinG_est + RGtoI_est.transpose() * pLinI_est;
  Matrix3d RItoG_fej = RGtoI_fej.transpose();
  Matrix3d RGtoL_fej = RItoL_fej * RGtoI_fej;
  Matrix3d RLtoG_fej = RGtoL_fej.transpose();
  Vector3d pLinG_fej = pIinG_fej + RGtoI_fej.transpose() * pLinI_fej;
  // Map frame
  Matrix3d RGtoM_est = RItoL_est * RGtoA_est;
  Vector3d pMinG_est = pAinG_est + RGtoA_est.transpose() * pLinI_est;
  Matrix3d RGtoM_fej = RItoL_fej * RGtoA_fej;
  Vector3d pMinG_fej = pAinG_fej + RGtoA_fej.transpose() * pLinI_fej;
  // Lidar to Map transformation
  Matrix3d RLtoM_est = RGtoM_est * RLtoG_est;
  Vector3d pLinM_est = RGtoM_est * (pLinG_est - pMinG_est);
  Matrix3d RLtoM_fej = RGtoM_fej * RLtoG_fej;
  Vector3d pLinM_fej = RGtoM_fej * (pLinG_fej - pMinG_fej);

  bool at_clone_m = state->have_clone(lidar->time + dt);
  bool at_clone_a = state->have_clone(ikd->time + dt);
  double ie_o = state->intr_ori_cov(state->op->clone_freq, state->op->intr_order);
  double ie_p = state->intr_pos_cov(state->op->clone_freq, state->op->intr_order);

  // Get marginal covariance of the state for Chi2 test
  MatrixXd P = StateHelper::get_marginal_covariance(state, H_order);

  //=========================================================================
  // Find Match and Compute Linear System (Multi-Threaded)
  //=========================================================================
  // Get LiDAR measurements in Map
  bool success = LidarHelper::transform_to_map(state, lidar, ikd);
  assert(success);
  // vec<{res, H}>
  vector<pair<VectorXd, MatrixXd>> vec_lin(lidar->size());

  // Loop Threads
  int num_threads = std::thread::hardware_concurrency(); // Number of threads to use
  num_threads = (num_threads < 4 ? 4 : num_threads);     // at least 4 threads
  int chunk_size = lidar->size() / num_threads;          // Chunk size for each thread
  vector<thread> threads;
  for (int t = 0; t < num_threads; t++) {
    int start = t * chunk_size;
    int end = (t + 1) * chunk_size;
    t == num_threads - 1 ? end = lidar->size() : int(); // Last thread takes the remaining elements
    threads.emplace_back([&, start, end] {
      // Loop points of the new scan
      for (int i = start; i < end; i++) {
        // Get this point from new scan.
        Vector3d pfinL = lidar->p(i);
        Vector3d pfinM_est = pLinM_est + RLtoM_est * pfinL;
        Vector3d pfinM_fej = pLinM_fej + RLtoM_fej * pfinL;

        // Get neighbors of this point in the map.
        POINTCLOUD_XYZI_PTR neighbors_inM(new pcl::PointCloud<pcl::PointXYZI>);
        if (!LidarHelper::get_neighbors(lidar->pfinM(i), neighbors_inM, ikd->tree, state->op->lidar))
          continue;

        // Get plane coefficients from neighbors
        Vector4d plane_abcd;
        if (!LidarHelper::compute_plane(plane_abcd, neighbors_inM, state->op->lidar))
          continue;

        // Discard bad match
        if (-plane_abcd.head(3).transpose() * pfinM_est - plane_abcd(3) > state->op->lidar->plane_max_p2pd * 3)
          continue;

        // Compute residual and dz_dplane_abc with neighboring map points
        assert(state->op->lidar->map_ngbr_num == (int)neighbors_inM->size());
        int row_size = state->op->lidar->map_ngbr_num + 1; // n of neighbors + 1 new scan point
        VectorXd res = VectorXd::Zero(row_size);
        MatrixXd dz_dplane_abc = MatrixXd::Zero(row_size, 3);

        for (int j = 0; j < state->op->lidar->map_ngbr_num; j++) {
          // residual of neighboring map points: res = 0 - (ax+by+cz+d)
          Vector3d pninM(neighbors_inM->points[j].x, neighbors_inM->points[j].y, neighbors_inM->points[j].z);
          res(j) = -plane_abcd.head(3).transpose() * pninM - plane_abcd(3);
          dz_dplane_abc.block(j, 0, 1, 3) = pninM.transpose();
        }

        // Compute residual and dz_dplane_abc with new scan point
        res(row_size - 1) = -plane_abcd.head(3).transpose() * pfinM_est - plane_abcd(3);
        dz_dplane_abc.block(row_size - 1, 0, 1, 3) = pfinM_fej.transpose();

        //=========================================================================
        // Jacobian
        //=========================================================================
        // Jacobians in respect to interpolated poses and extrinsic
        Matrix<double, 1, 3> dz_dpfinM_bottom_row = plane_abcd.head(3).transpose();
        Matrix<double, 3, 6> dpfinM_dI = Matrix<double, 3, 6>::Zero();
        dpfinM_dI.block(0, 0, 3, 3) = -RGtoM_fej * RItoG_fej * skew_x(RLtoI_fej * (pfinL - pIinL_fej));
        dpfinM_dI.block(0, 3, 3, 3) = RGtoM_fej;
        Matrix<double, 3, 6> dpfinM_dA = Matrix<double, 3, 6>::Zero();
        dpfinM_dA.block(0, 0, 3, 3) = RItoL_fej * skew_x(RGtoA_fej * (pIinG_fej + RItoG_fej * pLinI_fej - pAinG_fej + RLtoG_fej * pfinL));
        dpfinM_dA.block(0, 3, 3, 3) = -RGtoM_fej;
        Matrix<double, 3, 6> dpfinM_dcalib = Matrix<double, 3, 6>::Zero();
        dpfinM_dcalib.block(0, 0, 3, 3) = skew_x(RGtoM_fej * (pIinG_fej - pAinG_fej)) - skew_x(RLtoM_fej * (pIinL_fej - pfinL)) + RLtoM_fej * skew_x(pIinL_fej - pfinL);
        dpfinM_dcalib.block(0, 3, 3, 3) = Matrix3d::Identity() - RLtoM_fej;

        // CHAINRULE: get the total Jacobian in respect to interpolated pose
        double intr_std = 0.0;
        if (!at_clone_m || !at_clone_a) {
          MatrixXd dpfinM_dIntr = (at_clone_m ? MatrixXd::Zero(3, 6) : dpfinM_dI) + (at_clone_a ? MatrixXd::Zero(3, 6) : dpfinM_dA);
          MatrixXd HI = dz_dpfinM_bottom_row * dpfinM_dI;
          intr_std += sqrt(ie_o * (pow(HI(0), 2) + pow(HI(1), 2) + pow(HI(2), 2)) + ie_p * (pow(HI(3), 2) + pow(HI(4), 2) + pow(HI(5), 2)));
        }

        //=========================================================================
        // Jacobian continues
        //=========================================================================
        MatrixXd H = MatrixXd::Zero(row_size, total_hx);
        for (int j = 0; j < (int)dTdxI.size(); j++)
          H.block(row_size - 1, map_hx.at(orderI.at(j)), 1, orderI.at(j)->size()).noalias() += dz_dpfinM_bottom_row * dpfinM_dI * dTdxI.at(j);

        for (int j = 0; j < (int)dTdxA.size(); j++)
          H.block(row_size - 1, map_hx.at(orderA.at(j)), 1, orderA.at(j)->size()).noalias() += dz_dpfinM_bottom_row * dpfinM_dA * dTdxA.at(j);

        if (state->op->lidar->do_calib_ext)
          H.block(row_size - 1, map_hx.at(calibration), 1, calibration->size()).noalias() += dz_dpfinM_bottom_row * dpfinM_dcalib;

        // Whiten the noise
        double raw_noise = plane_abcd.head(3).norm() * state->op->lidar->raw_noise + intr_std;
        double map_noise = plane_abcd.head(3).norm() * state->op->lidar->map_noise + intr_std;
        H.block(row_size - 1, 0, 1, total_hx) /= raw_noise;
        dz_dplane_abc.block(row_size - 1, 0, 1, 3) /= raw_noise;
        dz_dplane_abc.block(0, 0, state->op->lidar->map_ngbr_num, 3) /= map_noise;
        res(row_size - 1) /= raw_noise;
        res.head(state->op->lidar->map_ngbr_num) /= map_noise;

        // Nullspace projection
        StateHelper::nullspace_project_inplace(dz_dplane_abc, H, res);

        // Chi2 test
        if (chi->Chi2Check(P, H, res, MatrixXd::Identity(res.rows(), res.rows()))) {
          vec_lin[i].first = res;
          vec_lin[i].second = H;
        }
      }
    });
  }
  // Wait for all threads to finish
  for (auto &t : threads)
    t.join();

  // get total size of successful residual
  int total_res = 0;
  for (const auto &lin : vec_lin) {
    if (lin.first.size() == 0 || isnan(lin.first.norm()))
      continue;
    total_res += lin.first.size();
  }

  // Construct Big residual and Jacobian
  VectorXd res_big = VectorXd::Zero(total_res);
  MatrixXd H_big = MatrixXd::Zero(total_res, total_hx);
  int cnt = 0;
  for (auto lin : vec_lin) {
    if (lin.first.size() == 0 || isnan(lin.first.norm()))
      continue;
    res_big.block(cnt, 0, lin.first.rows(), 1) = lin.first;
    H_big.block(cnt, 0, lin.first.rows(), total_hx) = lin.second;
    cnt += lin.first.rows();
  }

  // 5. Perform measurement compression
  StateHelper::measurement_compress_inplace(H_big, res_big);
  if (H_big.rows() < 1)
    return false;

  // Our noise is whitened, so make it here after our compression
  MatrixXd R_big = MatrixXd::Identity(res_big.rows(), res_big.rows());

  // 6. With all good features update the state
  //  chi->Chi2Check(StateHelper::get_marginal_covariance(state, H_order), H_big, res_big, R_big, LIDAR, lidar->id, false);
  StateHelper::EKFUpdate(state, H_order, H_big, res_big, R_big, "LIDAR");
  return true;
}

pair<Matrix3d, Vector3d> UpdaterLidar::get_pose_LinG(int lidar_id, double lidar_time) {
  // Get IMU pose
  Matrix3d RGtoI;
  Vector3d pIinG;
  bool success = state->get_interpolated_pose(lidar_time + state->lidar_dt.at(lidar_id)->value()(0), RGtoI, pIinG);
  assert(success);

  // LiDAR frame
  Matrix3d RItoL = state->lidar_extrinsic.at(lidar_id)->Rot();
  Vector3d pIinL = state->lidar_extrinsic.at(lidar_id)->pos();
  Matrix3d RGtoL = RItoL * RGtoI;
  Vector3d pLinG = pIinG + RGtoI.transpose() * (-RItoL.transpose() * pIinL);

  // record this pose
  pair<Matrix3d, Vector3d> pose_LinG;
  pose_LinG.first = RGtoL;
  pose_LinG.second = pLinG;
  return pose_LinG;
}
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

#include "CamHelper.h"
#include "CamTypes.h"
#include "cam/CamBase.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "state/State.h"
#include "types/Landmark.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "utils/Jabdongsani.h"
#include "utils/Print_Logger.h"

using namespace std;
using namespace Eigen;
using namespace mins;
using namespace ov_type;
using namespace ov_core;

MatrixXd CamHelper::get_feature_jacobian_representation(CamFeature &feature) {
  MatrixXd H_f;
  H_f.resize(3, 3);
  // Global XYZ representation
  if (feature.feat_representation == LandmarkRepresentation::Representation::GLOBAL_3D) {
    H_f.setIdentity();
    return H_f;
  }

  // Global inverse depth representation
  if (feature.feat_representation == LandmarkRepresentation::Representation::GLOBAL_FULL_INVERSE_DEPTH) {
    // Get inverse depth representation (should match what is in Landmark.cpp)
    double g_rho = 1 / feature.p_FinG_fej.norm();
    double g_phi = acos(g_rho * feature.p_FinG_fej(2));
    double g_theta = atan2(feature.p_FinG_fej(1), feature.p_FinG_fej(0));
    Matrix<double, 3, 1> p_invFinG;
    p_invFinG(0) = g_theta;
    p_invFinG(1) = g_phi;
    p_invFinG(2) = g_rho;

    // Get inverse depth bearings
    double sin_th = sin(p_invFinG(0, 0));
    double cos_th = cos(p_invFinG(0, 0));
    double sin_phi = sin(p_invFinG(1, 0));
    double cos_phi = cos(p_invFinG(1, 0));
    double rho = p_invFinG(2, 0);

    // Construct the Jacobian
    H_f << -(1.0 / rho) * sin_th * sin_phi, (1.0 / rho) * cos_th * cos_phi, -(1.0 / (rho * rho)) * cos_th * sin_phi, (1.0 / rho) * cos_th * sin_phi, (1.0 / rho) * sin_th * cos_phi,
        -(1.0 / (rho * rho)) * sin_th * sin_phi, 0.0, -(1.0 / rho) * sin_phi, -(1.0 / (rho * rho)) * cos_phi;
    return H_f;
  }

  PRINT4(RED "[CAM]: Landmark representation is not supported.\n" RESET);
  exit(EXIT_FAILURE);
}

CamLinSys CamHelper::get_feature_jacobian_full(shared_ptr<State> state, CamFeature &cam_feat, DB_ptr db, ID_T_POSE imu_poses) {

  //=========================================================================
  // Linear System
  //=========================================================================
  CamLinSys linsys;

  // Compute the size of measurements for this feature and the states involved with this feature
  int total_hx = 0;
  int total_meas = n_meas(cam_feat.feat);
  unordered_map<shared_ptr<Type>, size_t> map_hx;
  vector<double> invalid_measurements;
  for (auto &pair : cam_feat.timestamps) {

    // Our extrinsics and intrinsics
    shared_ptr<PoseJPL> calibration = state->cam_extrinsic.at(pair.first);
    shared_ptr<Vec> distortion = state->cam_intrinsic.at(pair.first);
    shared_ptr<Vec> timeoffset = state->cam_dt.at(pair.first);

    // If doing calibration extrinsics
    if (state->op->cam->do_calib_ext && map_hx.find(calibration) == map_hx.end()) {
      map_hx.insert({calibration, total_hx});
      linsys.Hx_order.push_back(calibration);
      total_hx += calibration->size();
    }

    // If doing calibration intrinsics
    if (state->op->cam->do_calib_int && map_hx.find(distortion) == map_hx.end()) {
      map_hx.insert({distortion, total_hx});
      linsys.Hx_order.push_back(distortion);
      total_hx += distortion->size();
    }

    // If doing calibration timeoffset
    if (state->op->cam->do_calib_dt && map_hx.find(timeoffset) == map_hx.end()) {
      map_hx.insert({timeoffset, total_hx});
      linsys.Hx_order.push_back(timeoffset);
      total_hx += timeoffset->size();
    }

    // Loop through all measurements for this specific camera
    for (auto meas_time = cam_feat.timestamps.at(pair.first).begin(); meas_time != cam_feat.timestamps.at(pair.first).end();) {
      vector<shared_ptr<ov_type::Type>> order;
      auto D = Dummy();
      if (state->get_interpolated_jacobian((*meas_time) + timeoffset->value()(0), D.R, D.p, "CAM", pair.first, D.VM, order)) {
        for (const auto &type : order) {
          if (map_hx.find(type) == map_hx.end()) {
            map_hx.insert({type, total_hx});
            linsys.Hx_order.push_back(type);
            total_hx += type->size();
          }
        }
        meas_time++;
      } else {
        // case fail, remove from list
        total_meas--;                                                    // reduce total meas
        copy_to_db(db, cam_feat.feat, pair.first, (*meas_time));         // copy to db
        invalid_measurements.push_back((*meas_time));                    // mark it as invalid measurement. Will be removed from feature later
        meas_time = cam_feat.timestamps.at(pair.first).erase(meas_time); // erase measurement from feature and move on
      }
    }
  }

  // remove invalid measurements from feature and put it back to db
  cam_feat.feat->clean_invalid_measurements(invalid_measurements);

  // Allocate our residual and Jacobians
  int c = 0;
  linsys.res = VectorXd::Zero(2 * total_meas);
  linsys.Hf = MatrixXd::Zero(2 * total_meas, 3);
  linsys.Hx = MatrixXd::Zero(2 * total_meas, total_hx);
  linsys.R = MatrixXd::Identity(2 * total_meas, 2 * total_meas); // This should be the result of whitening.

  // Derivative of p_FinG in respect to feature representation.
  // This only needs to be computed once and thus we pull it out of the loop
  MatrixXd dp_FinG_dlambda = get_feature_jacobian_representation(cam_feat);
  MatrixXd intr_err_cov = state->intr_pose_cov(state->op->clone_freq, state->op->intr_order);

  // Loop through each camera for this feature
  for (auto const &pair : cam_feat.timestamps) {

    // Our calibration between the IMU and cami frames
    int cam_id = pair.first;
    shared_ptr<Vec> distortion = state->cam_intrinsic.at(cam_id);
    shared_ptr<PoseJPL> calibration = state->cam_extrinsic.at(cam_id);
    shared_ptr<Vec> timeoffset = state->cam_dt.at(cam_id);
    Matrix3d R_ItoC = calibration->Rot();
    Vector3d p_IinC = calibration->pos();
    double dt = timeoffset->value()(0);

    // Loop through all measurements for this specific camera
    for (size_t m = 0; m < cam_feat.timestamps.at(cam_id).size(); m++) {
      //=========================================================================
      // Residual
      //=========================================================================
      double tm = cam_feat.timestamps.at(cam_id).at(m);
      Matrix3d R_GtoI = imu_poses.at(cam_id).at(tm).Rot();
      Vector3d p_IinG = imu_poses.at(cam_id).at(tm).pos();

      // Get current feature in the IMU
      Vector3d p_FinI = R_GtoI * (cam_feat.p_FinG - p_IinG);

      // Project the current feature into the current frame of reference
      Vector3d p_FinC = R_ItoC * p_FinI + p_IinC;
      Vector2d uv_norm;
      uv_norm << p_FinC(0) / p_FinC(2), p_FinC(1) / p_FinC(2);

      // Distort the normalized coordinates (radtan or fisheye)
      Vector2d uv_dist;
      uv_dist = state->cam_intrinsic_model.at(cam_id)->distort_d(uv_norm);

      // Our residual
      Vector2d uv_m;
      uv_m << (double)cam_feat.uvs.at(cam_id).at(m)(0), (double)cam_feat.uvs.at(cam_id).at(m)(1);
      linsys.res.block(2 * c, 0, 2, 1) = uv_m - uv_dist;

      //=========================================================================
      // Jacobian
      //=========================================================================
      // Get Jacobian of interpolated pose in respect to the state
      vector<MatrixXd> dTdx;
      vector<shared_ptr<ov_type::Type>> order;
      state->get_interpolated_jacobian(tm + dt, R_GtoI, p_IinG, "CAM", cam_id, dTdx, order);

      // Compute Jacobians in respect to normalized image coordinates and possibly the camera intrinsics
      MatrixXd dz_dzn, dz_dzeta;
      state->cam_intrinsic_model.at(pair.first)->compute_distort_jacobian(uv_norm, dz_dzn, dz_dzeta);

      // Overwrite FEJ value
      p_FinI = R_GtoI * (cam_feat.p_FinG_fej - p_IinG);
      p_FinC = R_ItoC * p_FinI + p_IinC;

      // Normalized coordinates in respect to projection function
      MatrixXd dzn_dp_FinC = MatrixXd::Zero(2, 3);
      dzn_dp_FinC << 1 / p_FinC(2), 0, -p_FinC(0) / (p_FinC(2) * p_FinC(2)), 0, 1 / p_FinC(2), -p_FinC(1) / (p_FinC(2) * p_FinC(2));

      // Derivative of p_FinC in respect to p_FinI
      MatrixXd dp_FinC_dp_FinG = R_ItoC * R_GtoI;

      // Derivative of p_FinC in respect to 'interpolated' IMU pose
      MatrixXd dp_FinC_dI = MatrixXd::Zero(3, 6);
      dp_FinC_dI.block(0, 0, 3, 3) = R_ItoC * skew_x(p_FinI);
      dp_FinC_dI.block(0, 3, 3, 3) = -dp_FinC_dp_FinG;

      // Precompute some matrices
      MatrixXd dz_dp_FinC = dz_dzn * dzn_dp_FinC;

      // CHAINRULE: get the total Jacobian of in respect to feature and interpolated pose
      Eigen::MatrixXd HI = dz_dp_FinC * dp_FinC_dI;

      //=========================================================================
      // Find the LLT of R inverse so that we can efficiently whiten the noise
      //=========================================================================
      // default covariance
      Eigen::MatrixXd R = pow(state->op->cam->sigma_pix, 2) * Matrix2d::Identity();
      // append interpolated covariance
      if (!state->have_clone(tm + dt)) {
        if (state->op->use_pol_cov) {
          R.noalias() += HI * intr_err_cov * HI.transpose();
        } else if (state->op->use_imu_cov) {
          MatrixXd H_cpi = MatrixXd::Zero(6, 6); // pose to gamma and alpha
          H_cpi.block(0, 0, 3, 3) = Matrix3d::Identity();
          H_cpi.block(3, 3, 3, 3) = state->clones.at(state->cpis.at(tm + dt).clone_t)->Rot_fej().transpose();

          MatrixXd H_ = HI * H_cpi;
          R.noalias() += H_ * state->cpis.at(tm + dt).Q * H_.transpose() * state->op->intr_err.mlt;
        }
      }

      // Find the LLT of R inverse
      MatrixXd R_llt = R.llt().matrixL();
      MatrixXd R_llt_inv = R_llt.llt().solve(MatrixXd::Identity(2, 2));

      // Whiten residue with the measurement noise
      linsys.res.block(2 * c, 0, 2, 1) = R_llt_inv * linsys.res.block(2 * c, 0, 2, 1);

      // Whiten the jacobians.
      dz_dp_FinC = R_llt_inv * dz_dp_FinC;
      dz_dzeta = R_llt_inv * dz_dzeta;

      //=========================================================================
      // Jacobian continues
      //=========================================================================
      linsys.Hf.block(2 * c, 0, 2, linsys.Hf.cols()).noalias() += dz_dp_FinC * dp_FinC_dp_FinG * dp_FinG_dlambda;

      // CHAINRULE: get state clone Jacobian. This also adds timeoffset jacobian
      for (int i = 0; i < (int)dTdx.size(); i++) {
        assert(map_hx.find(order.at(i)) != map_hx.end());
        linsys.Hx.block(2 * c, map_hx.at(order.at(i)), 2, order.at(i)->size()).noalias() += dz_dp_FinC * dp_FinC_dI * dTdx.at(i);
      }

      // Derivative of p_FinCi in respect to camera calibration (R_ItoC, p_IinC)
      if (state->op->cam->do_calib_ext) {
        MatrixXd dp_FinC_dT_ItoC = MatrixXd::Zero(3, 6);
        dp_FinC_dT_ItoC.block(0, 0, 3, 3) = skew_x(p_FinC - p_IinC);
        dp_FinC_dT_ItoC.block(0, 3, 3, 3) = Matrix<double, 3, 3>::Identity();
        // Chainrule it and add it to the big jacobian
        linsys.Hx.block(2 * c, map_hx.at(calibration), 2, calibration->size()).noalias() += dz_dp_FinC * dp_FinC_dT_ItoC;
      }

      // Derivative of measurement in respect to distortion parameters
      if (state->op->cam->do_calib_int) {
        linsys.Hx.block(2 * c, map_hx.at(distortion), 2, distortion->size()) += dz_dzeta;
      }
      // Move the Jacobian and residual index forward
      c++;
    }
  }
  return linsys;
}

void CamHelper::features_containing_older(shared_ptr<State> state, DB_ptr db, double timestamp, V_Feature &feat_found) {
  // Now lets loop through all features, and just make sure they are not old
  auto feats = db->get_internal_data();
  for (auto feat = feats.begin(); feat != feats.end();) {
    // Skip if already deleted
    if ((*feat).second->to_delete) {
      feat++;
      continue;
    }
    // Find features that has measurements older than the specified
    bool found_containing_older = false;
    for (auto const &pair : (*feat).second->timestamps) {
      for (auto meas_t : pair.second) {
        if (meas_t < timestamp - state->cam_dt.at(pair.first)->value()(0)) {
          found_containing_older = true;
          break;
        }
      }
    }
    // If it has an older timestamp, then add it
    if (found_containing_older) {
      (*feat).second->to_delete = true; // mark for deletion
      feat_found.push_back((*feat).second);
    }
    // move forward
    feat++;
  }
}

void CamHelper::features_not_containing_newer(shared_ptr<State> state, DB_ptr db, double timestamp, V_Feature &feat_found) {
  // Now lets loop through all features, and just make sure they are not old
  auto feats = db->get_internal_data();
  for (auto it = feats.begin(); it != feats.end();) {
    // Skip if already deleted
    if ((*it).second->to_delete) {
      it++;
      continue;
    }
    // Find features that does not have measurements newer than the specified
    bool has_newer_measurement = false;
    for (auto const &pair : (*it).second->timestamps) {
      for (auto meas_t : pair.second) {
        if (meas_t > timestamp - state->cam_dt.at(pair.first)->value()(0)) {
          has_newer_measurement = true;
          break;
        }
      }
    }
    // If it is not being actively tracked, then it is old
    if (!has_newer_measurement) {
      (*it).second->to_delete = true; // mark for deletion
      feat_found.push_back((*it).second);
    }
    // move forward
    it++;
  }
}

void CamHelper::get_imu_poses(shared_ptr<State> state, V_Feature &v_feats, DB_ptr db, ID_T_POSE &imu_poses) {
  // loop through feature measurements and compute IMU poses
  for (auto feat = v_feats.begin(); feat != v_feats.end();) {
    get_imu_poses(state, (*feat), db, imu_poses);
    // remove if feature has no measurements
    if (n_meas(*feat) == 0)
      feat = v_feats.erase(feat);
    // also remove feature if it has 1 measurement, except it is slam feature cause slam feature can process 1 measurement
    else if (n_meas(*feat) == 1)
      state->cam_SLAM_features.find((*feat)->featid) == state->cam_SLAM_features.end() ? feat = v_feats.erase(feat) : feat++;
    else // Otherwise this is good
      feat++;
  }
}

void CamHelper::get_imu_poses(shared_ptr<State> state, shared_ptr<ov_core::Feature> feat, DB_ptr db, ID_T_POSE &imu_poses) {
  // loop through feature measurements and compute IMU poses
  vector<double> invalid_times;
  for (auto &meas : feat->timestamps) {
    int cam_id = meas.first;
    double dt = state->cam_dt.at(meas.first)->value()(0);

    // Append a new cam id if not exist
    if (imu_poses.find(cam_id) == imu_poses.end())
      imu_poses.insert({cam_id, unordered_map<double, ov_core::FeatureInitializer::ClonePose>()});

    // now loop through each measurement and compute the IMU pose
    for (auto mtime : meas.second) {
      // Check if we already have this pose
      if (imu_poses.at(cam_id).find(mtime + dt) == imu_poses.at(cam_id).end()) {
        // compute imu pose
        Matrix3d R_GtoI;
        Vector3d p_IinG;
        if (state->get_interpolated_pose(mtime + dt, R_GtoI, p_IinG)) {
          imu_poses.at(cam_id).insert({mtime, FeatureInitializer::ClonePose(R_GtoI, p_IinG)});
        } else {
          copy_to_db(db, feat, meas.first, mtime);
          invalid_times.push_back(mtime);
        }
      }
    }
  }

  // remove all the measurements that do not have bounding poses
  feat->clean_invalid_measurements(invalid_times);
}

void CamHelper::get_cam_poses(State_ptr state, ID_T_POSE &imu_poses, ID_T_POSE &cam_poses) {
  // Get camera pose based on n-order interpolation
  for (const auto &id_t_pose : imu_poses) {
    // Load Cam info
    int cam_id = id_t_pose.first;
    Matrix3d R_ItoC = state->cam_extrinsic.at(cam_id)->Rot();
    Vector3d p_IinC = state->cam_extrinsic.at(cam_id)->pos();

    // Append a new cam id if not exist
    if (cam_poses.find(cam_id) == cam_poses.end())
      cam_poses.insert({cam_id, unordered_map<double, ov_core::FeatureInitializer::ClonePose>()});

    // Append camera pose
    for (auto t_pose : id_t_pose.second) {
      if (cam_poses.at(cam_id).find(t_pose.first) == cam_poses.at(cam_id).end()) {
        Matrix3d R_GtoC = R_ItoC * t_pose.second.Rot();
        Vector3d p_CinG = t_pose.second.pos() - R_GtoC.transpose() * p_IinC;
        cam_poses.at(cam_id).insert({t_pose.first, FeatureInitializer::ClonePose(R_GtoC, p_CinG)});
      }
    }
  }
}

void CamHelper::feature_triangulation(INIT_ptr init, V_Feature &v_feats, DB_ptr db, ID_T_POSE cam_pose) {
  // Loop and try triangulation
  for (auto feat = v_feats.begin(); feat != v_feats.end();) {
    if (!feature_triangulation(init, *feat, cam_pose)) {
      copy_to_db(db, (*feat));
      feat = v_feats.erase(feat);
    } else {
      feat++;
    }
  }
}

bool CamHelper::feature_triangulation(INIT_ptr init, shared_ptr<ov_core::Feature> &feat, ID_T_POSE cam_poses) {
  // return false if not enough measurement
  if (n_meas(feat) < 2)
    return false;

  // Triangulation
  if (!(init->config().triangulate_1d ? init->single_triangulation_1d(feat, cam_poses) : init->single_triangulation(feat, cam_poses)))
    return false;

  // Gauss-newton refinement
  if (init->config().refine_features && !init->single_gaussnewton(feat, cam_poses))
    return false;

  // All good
  return true;
}

void CamHelper::print(DB_ptr db) {
  for (const auto &id_feature : db->get_internal_data()) {
    PRINT2("Feat %lu - ", id_feature.first);
    for (const auto &cam_time : id_feature.second->timestamps) {
      PRINT2("Cam %lu - ", cam_time.first);
      for (auto time : cam_time.second) {
        PRINT2("%.4f ", time);
      }
      PRINT2("\n");
    }
  }
}
void CamHelper::print(V_Feature v_feats) {
  for (const auto &feature : v_feats) {
    print(feature);
  }
}
void CamHelper::print(shared_ptr<Feature> feat) {
  PRINT2("Feat %lu - ", feat->featid);
  for (const auto &cam_time : feat->timestamps) {
    PRINT2("Cam %lu - ", cam_time.first);
    for (auto time : cam_time.second) {
      PRINT2("%.9f ", time);
    }
    PRINT2("\n");
  }
}
void CamHelper::print(ID_T_POSE poses) {
  for (const auto &id_t_pose : poses) {
    PRINT2("Cam id %lu\n", id_t_pose.first);
    for (auto p : id_t_pose.second)
      cout << fixed << p.first << "(" << rot_2_quat(p.second.Rot()).transpose() << ", " << p.second.pos().transpose() << ")" << endl;
  }
}
void CamHelper::copy_to_db(DB_ptr db, shared_ptr<Feature> &feature) {
  for (const auto &cam_id : feature->timestamps) {
    for (const auto &mtime : cam_id.second) {
      copy_to_db(db, feature, cam_id.first, mtime);
    }
  }
}
void CamHelper::copy_to_db(DB_ptr db, vector<shared_ptr<ov_core::Feature>> &vec_feature) {
  for (auto &feature : vec_feature) {
    for (const auto &cam_id : feature->timestamps) {
      for (const auto &mtime : cam_id.second) {
        copy_to_db(db, feature, cam_id.first, mtime);
      }
    }
  }
}
void CamHelper::copy_to_db(DB_ptr db, shared_ptr<Feature> &feature, unsigned long cam_id, double time) {
  // fine the uv measurments correspond to the measurement time
  auto it0 = find(feature->timestamps.at(cam_id).begin(), feature->timestamps.at(cam_id).end(), time);
  assert(it0 != feature->timestamps.at(cam_id).end());
  auto idx0 = distance(feature->timestamps.at(cam_id).begin(), it0);
  // get uv measurement
  Vector2f uv = feature->uvs.at(cam_id).at(idx0).block(0, 0, 2, 1);
  Vector2f uv_n = feature->uvs_norm.at(cam_id).at(idx0).block(0, 0, 2, 1);
  // move measurement
  db->update_feature(feature->featid, time, cam_id, uv(0), uv(1), uv_n(0), uv_n(1));
}

CamFeature CamHelper::create_feature(shared_ptr<Feature> feat, LandmarkRepresentation::Representation rep) {
  CamFeature cam_feat;
  cam_feat.featid = feat->featid;
  cam_feat.anchor_cam_id = feat->anchor_cam_id;
  cam_feat.uvs = feat->uvs;
  cam_feat.uvs_norm = feat->uvs_norm;
  cam_feat.timestamps = feat->timestamps;
  cam_feat.feat_representation = rep;
  cam_feat.p_FinG = feat->p_FinG;
  cam_feat.p_FinG_fej = feat->p_FinG;
  cam_feat.feat = feat;
  return cam_feat;
}

CamFeature CamHelper::create_feature(shared_ptr<Feature> feat, shared_ptr<Landmark> landmark) {
  CamFeature cam_feat;
  cam_feat.featid = feat->featid;
  cam_feat.anchor_cam_id = feat->anchor_cam_id;
  cam_feat.uvs = feat->uvs;
  cam_feat.uvs_norm = feat->uvs_norm;
  cam_feat.timestamps = feat->timestamps;
  cam_feat.feat_representation = landmark->_feat_representation;
  cam_feat.p_FinG = landmark->get_xyz(false);
  cam_feat.p_FinG_fej = landmark->get_xyz(true);
  cam_feat.feat = feat;
  return cam_feat;
}

shared_ptr<Landmark> CamHelper::create_landmark(CamFeature cam_feat) {

  // construct new feature information
  shared_ptr<Landmark> landmark = make_shared<Landmark>(3);
  landmark->_featid = cam_feat.featid;
  landmark->_feat_representation = cam_feat.feat_representation;
  landmark->_unique_camera_id = cam_feat.anchor_cam_id;
  landmark->set_from_xyz(cam_feat.p_FinG, false);
  landmark->set_from_xyz(cam_feat.p_FinG_fej, true);
  return landmark;
}

int CamHelper::n_meas(std::shared_ptr<ov_core::Feature> feat) {
  int n_meas = 0;
  for (const auto &pair : feat->timestamps)
    n_meas += pair.second.size();
  return n_meas;
}

int CamHelper::n_meas(vector<std::shared_ptr<ov_core::Feature>> vec_feat) {
  int num_meas = 0;
  for (const auto &feat : vec_feat)
    num_meas += n_meas(feat);
  return num_meas;
}

int CamHelper::n_meas(shared_ptr<ov_core::FeatureDatabase> db) {
  int num_meas = 0;
  for (const auto &id_feature : db->get_internal_data())
    num_meas += n_meas(id_feature.second);
  return num_meas;
}

int CamHelper::n_feat(std::shared_ptr<ov_core::Feature> feat) { return 1; }
int CamHelper::n_feat(vector<std::shared_ptr<ov_core::Feature>> vec_feat) { return vec_feat.size(); }
int CamHelper::n_feat(shared_ptr<ov_core::FeatureDatabase> db) { return db->get_internal_data().size(); }

void CamHelper::get_features(shared_ptr<State> state, V_Feature &msckf, V_Feature &slam, V_Feature &init, DB_ptr db_feats, deque<double> t_hist) {
  // return if we have just one image so far
  if (t_hist.size() < 2)
    return;

  // Init un-usable feature db
  DB_ptr db_unused = make_shared<FeatureDatabase>();

  // Loop through current SLAM features, if we have tracks of them, grab them for this update!
  for (auto &landmark : state->cam_SLAM_features) {
    assert(landmark.second->_unique_camera_id != -1);
    shared_ptr<Feature> feat2 = db_feats->get_feature(landmark.second->_featid, true);
    feat2 != nullptr ? slam.push_back(feat2) : void();
  }
  db_feats->cleanup();                                  // Remove extracted features from the database so there would be no duplicate.
  remove_unusable_measurements(state, slam, db_unused); // return un-usable measurements to db

  // Get features that have measurements older than second-oldest clone and
  V_Feature feats_pool;
  features_containing_older(state, db_feats, state->oldest_2nd_clone_time(), feats_pool);
  db_feats->cleanup();

  // Get features that have no measurement at the newest frame
  features_not_containing_newer(state, db_feats, t_hist.at(t_hist.size() - 2), feats_pool);
  db_feats->cleanup();
  remove_unusable_measurements(state, feats_pool, db_unused);

  // Sort based on track length long to short
  sort(feats_pool.begin(), feats_pool.end(), feat_sort);

  // Loop through features and decide what to do with them
  ID_T_POSE imu_poses, cam_poses;

  double cam_hz = (t_hist.size() - 1) / (t_hist.back() - t_hist.front());
  int n_pool = feats_pool.size();
  auto feat_init = make_shared<ov_core::FeatureInitializer>(*state->op->cam->featinit_options);
  for (auto feat = feats_pool.begin(); feat != feats_pool.end();) {
    // 0. Break if we have enough features.
    if (n_feat(msckf) >= state->op->cam->max_msckf)
      break;

    // move on if not enough measurement
    if (n_meas(*feat) < 2) {
      copy_to_db(db_unused, (*feat));
      feat = feats_pool.erase(feat);
      continue;
    }

    // 1. Triangulation
    get_imu_poses(state, (*feat), db_unused, imu_poses);
    get_cam_poses(state, imu_poses, cam_poses);
    if (!feature_triangulation(feat_init, *feat, cam_poses)) {
      copy_to_db(db_unused, (*feat));
      feat = feats_pool.erase(feat);
      continue;
    }

    // 2. SLAM initialization feature
    bool max_track = (n_meas(*feat) >= std::min((int)state->op->window_size * (int)cam_hz - 1, 10));
    if (max_track && (int)state->cam_SLAM_features.size() + n_feat(init) < state->op->cam->max_slam) {
      init.push_back(*feat);
      feat = feats_pool.erase(feat);
      continue;
    }

    // 3 MSCKF feature
    msckf.push_back(*feat);
    feat = feats_pool.erase(feat);
  }

  // Move remaining features back to db
  copy_to_db(db_unused, feats_pool);
  db_feats->append_new_measurements(db_unused);
  PRINT1("UpdaterCamera::try_update: # of features:");
  PRINT1(" POOL:%d, SLAM:%d, INIT:%d, MSCKF:%d\n", n_pool, CamHelper::n_feat(slam), CamHelper::n_feat(init), CamHelper::n_feat(msckf));
}

void CamHelper::cleanup_features(shared_ptr<State> state, V_Feature &feat, DB_ptr db, vector<Vector3d> &used, Track_ptr track_feat, DB_ptr track_db) {

  // Save all the MSCKF features used in the update
  for (auto const &feat : feat)
    used.push_back(feat->p_FinG);

  // Get vector of features unused
  vector<shared_ptr<Feature>> vec_unused;
  vec_unused.reserve(db->get_internal_data().size());
  for (auto f : db->get_internal_data())
    vec_unused.push_back(f.second);

  // Sort based on track length
  sort(vec_unused.begin(), vec_unused.end(), feat_sort);

  // Put in the database to return
  DB_ptr db_return = make_shared<FeatureDatabase>();
  copy_to_db(db_return, vec_unused);

  // This allows for measurements to be used in the future if they failed to be used this time
  // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
  auto feats_database = track_feat->get_feature_database();
  feats_database->append_new_measurements(db_return);

  // Cleanup any features older than the marginalization time
  if (state->clone_window() > state->op->window_size) {
    feats_database->cleanup_measurements(state->oldest_clone_time());
    track_db->cleanup_measurements(state->oldest_clone_time());
  }
}

void CamHelper::remove_unusable_measurements(shared_ptr<State> state, V_Feature &v_feats, DB_ptr db) {

  // Loop though the features
  for (auto feat = v_feats.begin(); feat != v_feats.end();) {
    vector<double> invalid_times;
    for (const auto &pair : (*feat)->timestamps) {
      for (const auto &meas_t : pair.second) {
        // check if this measurement is "newer" than our window. We allow extrapolation
        if (meas_t + state->cam_dt.at(pair.first)->value()(0) > state->time + state->op->dt_exp) {
          // return it back to db
          copy_to_db(db, (*feat), pair.first, meas_t);
          invalid_times.push_back(meas_t);
          continue;
        }
        // check if this measurement is "older" than our window. We allow extrapolation
        if (meas_t + state->cam_dt.at(pair.first)->value()(0) < state->oldest_clone_time() - state->op->dt_exp) {
          // discard the measurement as it cannot be processed
          invalid_times.push_back(meas_t);
          continue;
        }
      }
    }

    // remove invalid measurements from feature vector
    (*feat)->clean_invalid_measurements(invalid_times);

    // remove if feature has no measurements
    if (n_meas(*feat) == 0)
      feat = v_feats.erase(feat);
    // also remove feature if it has 1 measurement, except it is slam feature cause slam feature can process 1 measurement
    else if (n_meas(*feat) == 1)
      state->cam_SLAM_features.find((*feat)->featid) == state->cam_SLAM_features.end() ? feat = v_feats.erase(feat) : feat++;
    else // Otherwise this is good
      feat++;
  }
}

bool CamHelper::feat_sort(const shared_ptr<ov_core::Feature> &a, const shared_ptr<ov_core::Feature> &b) {
  size_t asize = 0;
  size_t bsize = 0;
  for (const auto &pair : a->timestamps)
    asize += pair.second.size();
  for (const auto &pair : b->timestamps)
    bsize += pair.second.size();
  return asize > bsize;
};

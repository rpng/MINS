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

#ifndef MINS_UPDATER_CAMHELPER_H
#define MINS_UPDATER_CAMHELPER_H

#include "feat/FeatureInitializer.h"
#include "types/LandmarkRepresentation.h"
#include <Eigen/Eigen>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

namespace ov_core {
class FeatureDatabase;
class TrackBase;
class Feature;
} // namespace ov_core
namespace ov_type {
class Landmark;
}
using namespace std;
using namespace Eigen;
namespace mins {
class State;
struct OptionsCamera;
struct CamFeature;
struct CamLinSys;

typedef shared_ptr<State> State_ptr;
typedef shared_ptr<ov_core::FeatureDatabase> DB_ptr;
typedef vector<shared_ptr<ov_core::Feature>> V_Feature;
typedef unordered_map<size_t, unordered_map<double, ov_core::FeatureInitializer::ClonePose>> ID_T_POSE;
typedef shared_ptr<ov_core::FeatureInitializer> INIT_ptr;
typedef shared_ptr<ov_core::TrackBase> Track_ptr;

/**
 * @brief Class that has helper functions for our updaters.
 *
 * Can compute the Jacobian for a single feature representation.
 * This will create the Jacobian based on what representation our state is in.
 * If we are using the anchor representation then we also have additional Jacobians in respect to the anchor state.
 * Also has functions such as nullspace projection and full jacobian construction.
 * For derivations look at @ref update-feat page which has detailed equations.
 *
 */
class CamHelper {
public:
  /// Return Jacobian of feature in respect to its representation
  static MatrixXd get_feature_jacobian_representation(CamFeature &feature);

  /// Return linear system of the feature
  static CamLinSys get_feature_jacobian_full(State_ptr state, CamFeature &cam_feat, DB_ptr db, ID_T_POSE imu_poses);

  /// print info of given object
  static void print(DB_ptr db);
  static void print(V_Feature v_feats);
  static void print(shared_ptr<ov_core::Feature> feat);
  static void print(ID_T_POSE poses);

  /// copy feature information to the database
  static void copy_to_db(DB_ptr db, shared_ptr<ov_core::Feature> &feature);
  static void copy_to_db(DB_ptr db, vector<shared_ptr<ov_core::Feature>> &vec_feature);
  static void copy_to_db(DB_ptr db, shared_ptr<ov_core::Feature> &feature, unsigned long cam_id, double time);

  /// return number of measurements in the object
  static int n_meas(std::shared_ptr<ov_core::Feature> feat);
  static int n_meas(vector<std::shared_ptr<ov_core::Feature>> vec_feat);
  static int n_meas(shared_ptr<ov_core::FeatureDatabase> db);

  /// return number of features in the object
  static int n_feat(std::shared_ptr<ov_core::Feature> feat);
  static int n_feat(vector<std::shared_ptr<ov_core::Feature>> vec_feat);
  static int n_feat(shared_ptr<ov_core::FeatureDatabase> db);

  /// return features older than the given timestamp
  static void features_containing_older(State_ptr state, DB_ptr db, double timestamp, V_Feature &feat_found);

  /// return features that does not have tracking after given timestamp
  static void features_not_containing_newer(State_ptr state, DB_ptr db, double timestamp, V_Feature &feat_found);

  /// get set of IMU or CAM poses that correspond to the feature measurement times
  static void get_imu_poses(State_ptr state, V_Feature &v_feats, DB_ptr db, ID_T_POSE &imu_poses);
  static void get_imu_poses(State_ptr state, shared_ptr<ov_core::Feature> feat, DB_ptr db, ID_T_POSE &imu_poses);
  static void get_cam_poses(State_ptr state, ID_T_POSE &imu_poses, ID_T_POSE &cam_poses);

  /// Perform triangulation
  static void feature_triangulation(INIT_ptr init, V_Feature &v_feats, DB_ptr db, ID_T_POSE cam_pose);
  static bool feature_triangulation(INIT_ptr init, shared_ptr<ov_core::Feature> &feat, ID_T_POSE cam_poses);

  /// Convert feature format
  static CamFeature create_feature(shared_ptr<ov_core::Feature> feat, ov_type::LandmarkRepresentation::Representation rep);
  static CamFeature create_feature(shared_ptr<ov_core::Feature> feat, shared_ptr<ov_type::Landmark> landmark);
  static shared_ptr<ov_type::Landmark> create_landmark(CamFeature cam_feat);

  /// feature sort function: longer-tracks comes first
  static bool feat_sort(const std::shared_ptr<ov_core::Feature> &a, const std::shared_ptr<ov_core::Feature> &b);

  /// Return features can be used
  static void get_features(State_ptr state, V_Feature &msckf, V_Feature &slam, V_Feature &init, DB_ptr db_feats, deque<double> t_hist);

  /// Put all unused features back to db and check used ones
  static void cleanup_features(State_ptr state, V_Feature &feat, DB_ptr db, vector<Vector3d> &used, Track_ptr track_feat, DB_ptr track_db);

  /// Remove unusable measurements
  static void remove_unusable_measurements(State_ptr state, V_Feature &v_feats, DB_ptr db);
};
} // namespace mins

#endif // MINS_UPDATER_CAMHELPER_H

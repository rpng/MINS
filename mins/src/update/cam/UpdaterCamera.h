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

#ifndef MINS_UPDATERCAMERA_H
#define MINS_UPDATERCAMERA_H
#include <Eigen/Eigen>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <queue>
#include <vector>

using namespace std;
namespace ov_core {
class Feature;
class FeatureDatabase;
class TrackBase;
struct CameraData;
} // namespace ov_core

namespace mins {
class State;
class UpdaterStatistics;
struct CamSimData;
struct TimeChecker;

typedef vector<shared_ptr<ov_core::Feature>> V_Feature;
typedef shared_ptr<ov_core::FeatureDatabase> DB_ptr;

class UpdaterCamera {
public:
  /// Camera updater
  UpdaterCamera(shared_ptr<State> state);

  /// get simulated camera measurement
  void feed_measurement(const CamSimData &camdata);

  /// get real camera measurement
  void feed_measurement(const ov_core::CameraData &camdata);

  /// check available measurements and try update
  void try_update(int cam_id);

  /// return <SCKF> features used in last update for visualization
  vector<Eigen::Vector3d> get_used_msckf();

  /// return images used in last update for visualization
  cv::Mat get_track_img(int cam_id);

  /// Chi information
  map<int, shared_ptr<UpdaterStatistics>> Chi;

  /// measurement time information
  map<int, deque<double>> t_hist;

private:
  friend class Initializer;

  /// marginalize SLAM features that are lost tracking
  void marginalize_slam_features(const ov_core::CameraData &camdata);
  void marginalize_slam_features(const CamSimData &camdata);

  /// perform MSCKF update
  void msckf_update(V_Feature &vec_features, DB_ptr db_unused);

  /// perform SLAM update
  void slam_update(V_Feature &vec_features, DB_ptr db_unused);

  /// perform SLAM feature initialization
  void slam_init(V_Feature &vec_features, DB_ptr db_unused);

  // Good features that where used in the last update (used in visualization)
  vector<Eigen::Vector3d> msckf_used;

  /// Complete history of our feature tracks
  map<int, DB_ptr> trackDATABASE;

  /// Our sparse feature tracker (klt or descriptor)
  map<int, shared_ptr<ov_core::TrackBase>> trackFEATS;

  /// Timing record
  std::shared_ptr<TimeChecker> tc;

  /// State
  shared_ptr<State> state;
};
} // namespace mins
#endif // MINS_UPDATERCAMERA_H

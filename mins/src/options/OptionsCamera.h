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

#ifndef MINS_OPTIONSCAMERA_H
#define MINS_OPTIONSCAMERA_H

#include "track/TrackBase.h"
#include "types/LandmarkRepresentation.h"
#include <map>
#include <memory>
#include <vector>

namespace ov_core {
class YamlParser;
struct FeatureInitializerOptions;
} // namespace ov_core

namespace mins {

/**
 * @brief Struct which stores all camera options needed for state estimation.
 */
struct OptionsCamera {

  /// Nice print function of what parameters we have loaded
  void load(const std::shared_ptr<ov_core::YamlParser> &parser = nullptr);

  void print();

  /// boolean for usage of sensor
  bool enabled = true;

  /// Max number of sensor
  int max_n = 2;

  /// boolean for additional timing analysis print
  bool time_analysis = false;

  /// rostopic to subscribe
  std::vector<std::string> topic;

  /// Cam timeoffset. Default value 0,0
  std::map<size_t, double> dt;

  /// Map between camid and camera intrinsics (fx, fy, cx, cy, d1...d4, cam_w, cam_h)
  std::map<size_t, Eigen::VectorXd> intrinsics;
  std::map<size_t, std::string> distortion_model;
  std::map<size_t, std::vector<int>> wh;

  /// Map between camid and camera extrinsics (q_ItoC, p_IinC).
  std::map<size_t, Eigen::VectorXd> extrinsics;

  std::map<size_t, bool> compressed;

  /// initial covariance values for calibration parameters
  double init_cov_dt = 1e-4;
  double init_cov_ex_o = 1e-4;
  double init_cov_ex_p = 1e-3;
  double init_cov_in_k = 1e-0;
  double init_cov_in_c = 1e-0;
  double init_cov_in_r = 1e-5;

  /// Bool to determine whether or not to calibrate imu-to-camera pose
  bool do_calib_ext = false;

  /// Bool to determine whether or not to calibrate camera intrinsics
  bool do_calib_int = false;

  /// Bool to determine whether or not to calibrate camera to IMU timeoffset
  bool do_calib_dt = false;

  /// If we should try to load a mask and use it to reject invalid features
  std::map<size_t, bool> use_mask;

  /// Mask images for each camera
  std::map<size_t, cv::Mat> masks;

  /// Will half the resolution all tracking image
  bool downsample = false;

  /// The number of points we should extract and track in *each* image frame. This highly effects the computation required for tracking.
  int n_pts = 150;

  /// Fast extraction threshold
  int fast = 20;

  /// Number of grids we should split column-wise to do feature extraction in
  int grid_x = 5;

  /// Number of grids we should split row-wise to do feature extraction in
  int grid_y = 5;

  /// Will check after doing KLT track and remove any features closer than this
  int min_px_dist = 10;

  /// What type of pre-processing histogram method should be applied to images
  ov_core::TrackBase::HistogramMethod histogram = ov_core::TrackBase::HistogramMethod::HISTOGRAM;

  /// KNN ration between top two descriptor matcher which is required to be a good match
  double knn = 0.85;

  /// Parameters used by our feature initialize / triangulator
  std::shared_ptr<ov_core::FeatureInitializerOptions> featinit_options;

  /// Max number of estimated SLAM features
  int max_slam = 25;

  /// Max number of MSCKF features we will use.
  int max_msckf = 1000;

  bool use_stereo = true;

  /// What representation our features are in msckf, slam
  ov_type::LandmarkRepresentation::Representation feat_rep;

  /// Chi threshold for outlier rejection
  double chi2_mult = 1;

  /// Noise sigma for our raw pixel measurements
  double sigma_pix = 1;

  /// Stereo pair information {0 ,1} {1, 0} are both in the map
  std::unordered_map<int, int> stereo_pairs;

private:
  void load_i(const std::shared_ptr<ov_core::YamlParser> &parser, int i);
  void print_i(int i);
};

} // namespace mins

#endif // MINS_OPTIONSCAMERA_H

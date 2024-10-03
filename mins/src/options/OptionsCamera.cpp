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

#include "OptionsCamera.h"
#include "feat/FeatureInitializerOptions.h"
#include "utils/Print_Logger.h"
#include "utils/opencv_yaml_parse.h"

void mins::OptionsCamera::load(const std::shared_ptr<ov_core::YamlParser> &parser) {
  if (parser != nullptr) {
    std::string f = "config_camera";
    if (!boost::filesystem::exists(parser->get_config_folder() + f + ".yaml")) {
      enabled = false;
      return;
    }
    parser->parse_external(f, "cam", "enabled", enabled);
    parser->parse_external(f, "cam", "time_analysis", time_analysis, false);
    parser->parse_external(f, "cam", "use_stereo", use_stereo);
    parser->parse_external(f, "cam", "max_n", max_n);
    parser->parse_external(f, "cam", "do_calib_ext", do_calib_ext);
    parser->parse_external(f, "cam", "do_calib_int", do_calib_int);
    parser->parse_external(f, "cam", "do_calib_dt", do_calib_dt);
    parser->parse_external(f, "cam", "downsample", downsample);
    parser->parse_external(f, "cam", "n_pts", n_pts);
    parser->parse_external(f, "cam", "fast", fast);
    parser->parse_external(f, "cam", "grid_x", grid_x);
    parser->parse_external(f, "cam", "grid_y", grid_y);
    parser->parse_external(f, "cam", "min_px_dist", min_px_dist);
    parser->parse_external(f, "cam", "init_cov_dt", init_cov_dt);
    parser->parse_external(f, "cam", "init_cov_ex_o", init_cov_ex_o);
    parser->parse_external(f, "cam", "init_cov_ex_p", init_cov_ex_p);
    parser->parse_external(f, "cam", "init_cov_in_k", init_cov_in_k);
    parser->parse_external(f, "cam", "init_cov_in_c", init_cov_in_c);
    parser->parse_external(f, "cam", "init_cov_in_r", init_cov_in_r);
    parser->parse_external(f, "cam", "sigma_px", sigma_pix);
    parser->parse_external(f, "cam", "chi2_mult", chi2_mult);
    parser->parse_external(f, "cam", "knn", knn);
    parser->parse_external(f, "cam", "max_slam", max_slam);
    parser->parse_external(f, "cam", "max_msckf", max_msckf);
    featinit_options = std::make_shared<ov_core::FeatureInitializerOptions>();
    parser->parse_external(f, "cam", "fi_triangulate_1d", featinit_options->triangulate_1d, false);
    parser->parse_external(f, "cam", "fi_refine_features", featinit_options->refine_features, false);
    parser->parse_external(f, "cam", "fi_max_runs", featinit_options->max_runs, false);
    parser->parse_external(f, "cam", "fi_init_lamda", featinit_options->init_lamda, false);
    parser->parse_external(f, "cam", "fi_max_lamda", featinit_options->max_lamda, false);
    parser->parse_external(f, "cam", "fi_min_dx", featinit_options->min_dx, false);
    parser->parse_external(f, "cam", "fi_min_dcost", featinit_options->min_dcost, false);
    parser->parse_external(f, "cam", "fi_lam_mult", featinit_options->lam_mult, false);
    parser->parse_external(f, "cam", "fi_min_dist", featinit_options->min_dist, false);
    parser->parse_external(f, "cam", "fi_max_dist", featinit_options->max_dist, false);
    parser->parse_external(f, "cam", "fi_max_baseline", featinit_options->max_baseline, false);
    parser->parse_external(f, "cam", "fi_max_cond_number", featinit_options->max_cond_number, false);

    std::string feat_rep_ = "GLOBAL_3D";
    parser->parse_external(f, "cam", "feat_rep", feat_rep_);
    feat_rep = ov_type::LandmarkRepresentation::from_string(feat_rep_);
    if (feat_rep != ov_type::LandmarkRepresentation::GLOBAL_3D && feat_rep != ov_type::LandmarkRepresentation::GLOBAL_FULL_INVERSE_DEPTH) {
      PRINT4(RED "unsupported feature representation: %s\n" RESET, ov_type::LandmarkRepresentation::as_string(feat_rep).c_str());
      std::exit(EXIT_FAILURE);
    }
    std::string histogram_method_str = "HISTOGRAM";
    parser->parse_external(f, "cam", "histogram_method", histogram_method_str);
    if (histogram_method_str == "NONE") {
      histogram = ov_core::TrackBase::NONE;
    } else if (histogram_method_str == "HISTOGRAM") {
      histogram = ov_core::TrackBase::HISTOGRAM;
    } else if (histogram_method_str == "CLAHE") {
      histogram = ov_core::TrackBase::CLAHE;
    } else {
      PRINT4(RED "OptionsCamera: invalid feature histogram specified: %s. Available: NONE, HISTOGRAM, CLAHE\n" RESET, histogram_method_str.c_str());
      std::exit(EXIT_FAILURE);
    }

    // Loop through each camera and get parameters
    for (int i = 0; i < max_n; i++) {
      load_i(parser, i);
    }

    // Handle stereo camera
    if (use_stereo) {
      std::vector<int> vec_stereo_pair;
      parser->parse_external(f, "cam", "stereo_pair", vec_stereo_pair);
      if ((int)vec_stereo_pair.size() % 2 != 0) {
        PRINT4(RED "Stero pair should be provided even number.\n" RESET);
        exit(EXIT_FAILURE);
      }
      for (int i = 0; i < (int)vec_stereo_pair.size() / 2; i++) {
        if (vec_stereo_pair.at(2 * i) < max_n && vec_stereo_pair.at(2 * i + 1) < max_n) {
          if (stereo_pairs.find(vec_stereo_pair.at(2 * i)) != stereo_pairs.end() || stereo_pairs.find(vec_stereo_pair.at(2 * i + 1)) != stereo_pairs.end()) {
            PRINT4(RED "A camera is paired with more than one camera.\n" RESET);
            exit(EXIT_FAILURE);
          }
          stereo_pairs.insert({vec_stereo_pair.at(2 * i), vec_stereo_pair.at(2 * i + 1)});
          stereo_pairs.insert({vec_stereo_pair.at(2 * i + 1), vec_stereo_pair.at(2 * i)});
        }
      }
      if (vec_stereo_pair.empty()) {
        PRINT4(YELLOW "Stereo is enabled but no pair information provided. Disabling stereo.\n" RESET);
        use_stereo = false;
      }
    }

    // overwrite the timeoffset of the stereo cam
    for (auto pair : stereo_pairs)
      pair.second > pair.first ? dt.at(pair.second) = dt.at(pair.first) : dt.at(pair.first) = dt.at(pair.second);
  }
}

void mins::OptionsCamera::load_i(const std::shared_ptr<ov_core::YamlParser> &parser, int i) {

  // Loop through each camera and get parameters
  std::string f = "config_camera";
  // Time offest
  double toff = 0.0;
  parser->parse_external(f, "cam" + std::to_string(i), "timeoffset", toff);
  dt.insert({i, toff});

  // Intrinsic parameters
  std::vector<double> int1 = {1, 1, 0, 0};
  std::vector<double> int2 = {0, 0, 0, 0};
  parser->parse_external(f, "cam" + std::to_string(i), "intrinsics", int1);
  parser->parse_external(f, "cam" + std::to_string(i), "distortion_coeffs", int2);
  Eigen::VectorXd intrinsic = Eigen::VectorXd::Zero(8);
  intrinsic << int1[0], int1[1], int1[2], int1[3], int2[0], int2[1], int2[2], int2[3];
  intrinsic(0) /= (downsample) ? 2.0 : 1.0;
  intrinsic(1) /= (downsample) ? 2.0 : 1.0;
  intrinsic(2) /= (downsample) ? 2.0 : 1.0;
  intrinsic(3) /= (downsample) ? 2.0 : 1.0;
  intrinsics.insert({i, intrinsic});

  // Distortion model
  std::string dist_model = "radtan";
  parser->parse_external(f, "cam" + std::to_string(i), "distortion_model", dist_model);
  distortion_model.insert({i, dist_model});

  // FOV / resolution
  std::vector<int> wh_i = {1, 1};
  parser->parse_external(f, "cam" + std::to_string(i), "resolution", wh_i);
  wh_i[0] /= (downsample) ? 2.0 : 1.0;
  wh_i[1] /= (downsample) ? 2.0 : 1.0;
  wh.insert({i, wh_i});

  // Extrinsics
  Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
  parser->parse_external(f, "cam" + std::to_string(i), "T_imu_cam", T_CtoI);

  Eigen::Matrix<double, 7, 1> cam_eigen;
  cam_eigen.head(4) = ov_core::rot_2_quat(T_CtoI.block(0, 0, 3, 3).transpose());
  cam_eigen.tail(3) = -T_CtoI.block(0, 0, 3, 3).transpose() * T_CtoI.block(0, 3, 3, 1);
  extrinsics.insert({i, cam_eigen});

  bool use_mask_ = false;
  parser->parse_external(f, "cam" + std::to_string(i), "use_mask", use_mask_, false);
  use_mask.insert({i, use_mask_});

  std::string cam_topic;
  parser->parse_external(f, "cam" + std::to_string(i), "topic", cam_topic);
  topic.push_back(cam_topic);

  bool cam_compressed;
  parser->parse_external(f, "cam" + std::to_string(i), "compressed", cam_compressed, false);
  compressed.insert({i, cam_compressed});

  if (use_mask_) {
    std::string mask_path;
    std::string mask_node = "mask" + std::to_string(i);
    parser->parse_external(f, "cam" + std::to_string(i), "mask", mask_path);
    std::string total_mask_path = parser->get_config_folder() + mask_path;
    if (!boost::filesystem::exists(total_mask_path)) {
      PRINT4(RED "VioManager(): invalid mask path:\n" RESET);
      PRINT4(RED "\t- mask%d - %s\n" RESET, i, total_mask_path.c_str());
      std::exit(EXIT_FAILURE);
    }
    masks.insert({i, cv::imread(total_mask_path, cv::IMREAD_GRAYSCALE)});
  }
}

void mins::OptionsCamera::print() {
  if (!enabled)
    return;
  PRINT1(BOLDBLUE "Options - Camera\n" RESET);
  PRINT1("\t- time_analysis: %s\n", time_analysis ? "true" : "false");
  PRINT1("\t- do_calib_ext: %s\n", do_calib_ext ? "true" : "false");
  PRINT1("\t- do_calib_int: %s\n", do_calib_int ? "true" : "false");
  PRINT1("\t- do_calib_dt: %s\n", do_calib_dt ? "true" : "false");
  for (int i = 0; i < max_n; i++)
    print_i(i);
  PRINT1("\t- use_stereo: %s\n", use_stereo ? "true" : "false");
  PRINT1("\t- downsize cameras: %s\n", downsample ? "true" : "false");
  PRINT1("\t- n_pts: %d\n", n_pts);
  PRINT1("\t- fast threshold: %d\n", fast);
  PRINT1("\t- grid X by Y: %d by %d\n", grid_x, grid_y);
  PRINT1("\t- min px dist: %d\n", min_px_dist);
  PRINT1("\t- hist method: %d\n", (int)histogram);
  PRINT1("\t- knn ratio: %.3f\n", knn);
  PRINT1("\t- max_slam: %d\n", max_slam);
  PRINT1("\t- max_msckf: %d\n", max_msckf);
  PRINT1("\t- max_cameras: %d\n", max_n);
  PRINT1("\t- feat_rep: %s\n", ov_type::LandmarkRepresentation::as_string(feat_rep).c_str());
  PRINT1("\t- chi2_mult: %.1f\n", chi2_mult);
  PRINT1("\t- sigma_pix: %.2f\n", sigma_pix);
  PRINT1("\t- triangulate_1d: %d\n", featinit_options->triangulate_1d);
  PRINT1("\t- refine_features: %d\n", featinit_options->refine_features);
  PRINT1("\t- max_runs: %d\n", featinit_options->max_runs);
  PRINT1("\t- init_lamda: %.3f\n", featinit_options->init_lamda);
  PRINT1("\t- max_lamda: %.3f\n", featinit_options->max_lamda);
  PRINT1("\t- min_dx: %.7f\n", featinit_options->min_dx);
  PRINT1("\t- min_dcost: %.7f\n", featinit_options->min_dcost);
  PRINT1("\t- lam_mult: %.3f\n", featinit_options->lam_mult);
  PRINT1("\t- min_dist: %.3f\n", featinit_options->min_dist);
  PRINT1("\t- max_dist: %.3f\n", featinit_options->max_dist);
  PRINT1("\t- max_baseline: %.3f\n", featinit_options->max_baseline);
  PRINT1("\t- max_cond_number: %.3f\n", featinit_options->max_cond_number);
}

void mins::OptionsCamera::print_i(int i) {
  PRINT1("\t- Cam%d:\n", i);
  PRINT1("\t\t- timeoffset: %6.3f\n", dt.at(i));
  Eigen::VectorXd it = intrinsics.at(i);
  PRINT1("\t\t- intrinsics: [%6.3f, %6.3f, %6.3f, %6.3f]\n", it(0), it(1), it(2), it(3));
  PRINT1("\t\t- distortion_coeffs: [%6.3f, %6.3f, %6.3f, %6.3f]\n", it(4), it(5), it(6), it(7));
  PRINT1("\t\t- model: %s\n", distortion_model.at(i).c_str());
  PRINT1("\t\t- wh: %d %d\n", wh.at(i)[0], wh.at(i)[1]);
  PRINT1("\t\t- T_imu_cam:\n");
  Eigen::Matrix3d R = ov_core::quat_2_Rot(extrinsics.at(i).head(4)).transpose();
  Eigen::Vector3d p = -R * extrinsics.at(i).tail(3);
  PRINT1("\t\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(0), R(3), R(6), p(0));
  PRINT1("\t\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(1), R(4), R(7), p(1));
  PRINT1("\t\t\t- [%6.3f, %6.3f, %6.3f, %6.3f]\n", R(2), R(5), R(8), p(2));
  PRINT1("\t\t\t- [ 0.000,  0.000,  0.000,  1.000]\n");
  PRINT1("\t\t- use_mask: %s\n", use_mask.at(i) ? "true" : "false");
  PRINT1("\t\t- compressed: %s\n", compressed.at(i) ? "true" : "false");
  PRINT1("\t\t- topic: %s\n", topic.at(i).c_str());
}

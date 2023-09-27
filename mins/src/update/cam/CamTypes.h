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

#ifndef MINS_CAMTYPES_H
#define MINS_CAMTYPES_H

#include "types/LandmarkRepresentation.h"
#include <Eigen/Eigen>
#include <memory>
#include <unordered_map>
#include <vector>

namespace ov_core {
class Feature;
}
namespace ov_type {
class Type;
}
namespace mins {

struct CamSimData {

  /// Timestamp of the reading
  double time;

  /// ID of the sensor
  std::vector<int> ids;

  /// Position measurement.
  std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> uvs;
};

struct CamFeature {

  /// Unique ID of this feature
  size_t featid;

  /// UV coordinates that this feature has been seen from (mapped by camera ID)
  std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs;

  /// UV normalized coordinates that this feature has been seen from (mapped by camera ID)
  std::unordered_map<size_t, std::vector<Eigen::VectorXf>> uvs_norm;

  /// Timestamps of each UV measurement (mapped by camera ID)
  std::unordered_map<size_t, std::vector<double>> timestamps;

  /// What representation our feature is in
  ov_type::LandmarkRepresentation::Representation feat_representation;

  /// What camera ID our pose is anchored in!! By default the first measurement is the anchor.
  int anchor_cam_id = -1;

  /// Timestamp of anchor clone
  double anchor_timestamp = -1;

  /// Triangulated position of this feature, in the anchor frame
  Eigen::Vector3d p_FinA;

  /// Triangulated position of this feature, in the anchor frame first estimate
  Eigen::Vector3d p_FinA_fej;

  /// Triangulated position of this feature, in the global frame
  Eigen::Vector3d p_FinG;

  /// Triangulated position of this feature, in the global frame first estimate
  Eigen::Vector3d p_FinG_fej;

  /// original feature shared ptr
  std::shared_ptr<ov_core::Feature> feat;
};

struct CamLinSys {
  Eigen::MatrixXd Hf;
  Eigen::MatrixXd Hx;
  Eigen::MatrixXd R;
  Eigen::VectorXd res;
  std::vector<std::shared_ptr<ov_type::Type>> Hx_order;
  std::unordered_map<std::shared_ptr<ov_type::Type>, size_t> Hx_mapping;

  void print();

  void print_matrix(Eigen::MatrixXd H);

  void print_matrix(Eigen::VectorXd r);
};

} // namespace mins

#endif // MINS_CAMTYPES_H

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

#include "UpdaterVicon.h"
#include "ViconTypes.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsVicon.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/PoseJPL.h"
#include "types/Type.h"
#include "types/Vec.h"
#include "update/UpdaterStatistics.h"
#include "utils/Jabdongsani.h"

using namespace ov_core;
using namespace ov_type;
using namespace mins;
using namespace Eigen;

UpdaterVicon::UpdaterVicon(shared_ptr<State> state) : state(state) {
  for (int i = 0; i < state->op->vicon->max_n; i++) {
    Chi.emplace_back(make_shared<UpdaterStatistics>(state->op->vicon->chi2_mult, "VICON", i));
    t_hist.insert({i, deque<double>()});
  }
}

void UpdaterVicon::try_update() {
  // loop through all the measurements and perform update
  for (int i = 0; i < (int)data_stack.size(); i++) {
    // update and remove the measurement
    if (update(data_stack.at(i))) {
      data_stack.erase(data_stack.begin() + i);
      i--;
    } else if (data_stack.at(i).time + state->vicon_dt[data_stack.at(i).id]->value()(0) < state->oldest_clone_time()) {
      data_stack.erase(data_stack.begin() + i);
      i--;
    }
  }
}

bool UpdaterVicon::update(ViconData m) {
  //=========================================================================
  // Linear System
  //=========================================================================
  // Compute the size of the states involved with this measurement
  int total_hx = 0;
  unordered_map<shared_ptr<Type>, size_t> map_hx;
  vector<shared_ptr<Type>> Hx_order;
  // Our extrinsics and intrinsics
  shared_ptr<PoseJPL> calibration = state->vicon_extrinsic.at(m.id);
  shared_ptr<Vec> timeoffset = state->vicon_dt.at(m.id);

  // If doing calibration extrinsics
  if (state->op->vicon->do_calib_ext && map_hx.find(calibration) == map_hx.end()) {
    map_hx.insert({calibration, total_hx});
    Hx_order.push_back(calibration);
    total_hx += calibration->size();
  }

  // If doing calibration timeoffset
  if (state->op->vicon->do_calib_dt && map_hx.find(timeoffset) == map_hx.end()) {
    map_hx.insert({timeoffset, total_hx});
    Hx_order.push_back(timeoffset);
    total_hx += timeoffset->size();
  }

  // Get Jacobian of interpolated pose in respect to the state
  vector<shared_ptr<Type>> order;
  auto D = Dummy();
  if (state->get_interpolated_jacobian(m.time + timeoffset->value()(0), D.R, D.p, "VICON", m.id, D.VM, order)) {
    for (const auto &type : order) {
      if (map_hx.find(type) == map_hx.end()) {
        map_hx.insert({type, total_hx});
        Hx_order.push_back(type);
        total_hx += type->size();
      }
    }
  } else // return if we cannot find bounding poses.
    return false;

  //=========================================================================
  // Residual
  //=========================================================================
  // calib info
  Matrix3d R_ItoX = calibration->Rot();
  Vector3d p_IinX = calibration->pos();
  Vector3d p_XinI = -R_ItoX.transpose() * p_IinX;
  // State info
  Matrix3d R_GtoI;
  Vector3d p_IinG;
  if (!state->get_interpolated_pose(m.time + timeoffset->value()(0), R_GtoI, p_IinG))
    return false;
  Matrix3d R_ItoG = R_GtoI.transpose();
  // residual
  VectorXd res = VectorXd::Zero(6);
  res.head(3) = -log_so3(exp_so3(m.pose.head(3)) * (R_ItoX * R_GtoI).transpose());
  res.tail(3) = m.pose.tail(3) - (p_IinG + R_ItoG * p_XinI);

  //=========================================================================
  // Jacobian
  //=========================================================================
  // Get Jacobian of interpolated pose in respect to the state
  vector<MatrixXd> dTdx;
  bool success = state->get_interpolated_jacobian(m.time + timeoffset->value()(0), R_GtoI, p_IinG, "VICON", m.id, dTdx, order);
  assert(success);

  MatrixXd H = MatrixXd::Zero(6, total_hx);
  MatrixXd dz_dI = MatrixXd::Zero(6, 6);
  dz_dI.block(0, 0, 3, 3) = R_ItoX;                   // dzR_dIR
  dz_dI.block(3, 0, 3, 3) = -R_ItoG * skew_x(p_XinI); // dzp_dIR
  dz_dI.block(3, 3, 3, 3) = Matrix3d::Identity();     // dzp_dIp

  // CHAINRULE: get state clone Jacobian. This also adds timeoffset jacobian
  for (int i = 0; i < (int)dTdx.size(); i++) {
    assert(map_hx.find(order.at(i)) != map_hx.end());
    H.block(0, map_hx.at(order.at(i)), 6, order.at(i)->size()).noalias() += dz_dI * dTdx.at(i);
  }

  // Extrinsic calibration
  if (state->op->vicon->do_calib_ext) {
    // Calculate the Jacobian
    MatrixXd dz_dcalib = MatrixXd::Zero(6, 6);
    dz_dcalib.block(0, 0, 3, 3) = Matrix3d::Identity();                                     // dzR_dCR
    dz_dcalib.block(3, 0, 3, 3) = R_GtoI.transpose() * R_ItoX.transpose() * skew_x(p_IinX); // dzp_dCR
    dz_dcalib.block(3, 3, 3, 3) = -R_GtoI.transpose() * R_ItoX.transpose();                 // dzp_dCp
    H.block(0, map_hx.at(calibration), 6, calibration->size()).noalias() += dz_dcalib;
  }

  //=========================================================================
  // Noise Covariance
  //=========================================================================
  MatrixXd R = MatrixXd::Zero(6, 6);
  R.block(0, 0, 3, 3).noalias() = Matrix3d::Identity() * pow(state->op->vicon->noise_o, 2);
  R.block(3, 3, 3, 3).noalias() = Matrix3d::Identity() * pow(state->op->vicon->noise_p, 2);

  MatrixXd intr_cov = state->intr_pose_cov(state->op->clone_freq, state->op->intr_order);
  if (!state->have_clone(m.time + timeoffset->value()(0))) {
    R.noalias() += dz_dI * intr_cov * dz_dI.transpose();
  }

  //=========================================================================
  // Update
  //=========================================================================
  // Do Chi check and update!
  if (Chi.at(m.id)->Chi2Check(state, Hx_order, H, res, R)) {
    StateHelper::EKFUpdate(state, Hx_order, H, res, R, "VICON");
  }
  return true;
}
void UpdaterVicon::feed_measurement(const ViconData &data) {
  data_stack.push_back(data);

  // record timestamps
  t_hist.at(data.id).size() > 100 ? t_hist.at(data.id).pop_front() : void(); // remove if we have too many
  t_hist.at(data.id).push_back(data.time);
}

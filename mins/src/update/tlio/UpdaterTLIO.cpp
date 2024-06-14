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

#include "UpdaterTLIO.h"
#include "TLIOTypes.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsTLIO.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/PoseJPL.h"
#include "types/Type.h"
#include "types/Vec.h"
#include "update/UpdaterStatistics.h"
#include "utils/Jabdongsani.h"

#include "utils/Print_Logger.h"

using namespace ov_core;
using namespace ov_type;
using namespace mins;
using namespace Eigen;

UpdaterTLIO::UpdaterTLIO(shared_ptr<State> state) : state(state) { Chi = make_shared<UpdaterStatistics>(state->op->tlio->chi2_mult, "TLIO", 0); }

void UpdaterTLIO::try_update() {
  // loop through all the measurements and perform update
  for (int i = 0; i < (int)data_stack.size(); i++) {
    if (update(data_stack.at(i))) {
      data_stack.erase(data_stack.begin() + i);
      i--;
    } else if (data_stack.at(i).time + state->tlio_dt->value()(0) < state->oldest_clone_time()) {
      data_stack.erase(data_stack.begin() + i);
      i--;
    }
  }
}

bool UpdaterTLIO::update(TLIOData m) {
  // Compute the size of the states involved with this measurement
  int total_hx = 0;
  unordered_map<shared_ptr<Type>, size_t> map_hx;
  VecTypePtr Hx_order;

  VecTypePtr order;
  auto D = Dummy();

  if (!state->get_interpolated_jacobian(m.time, D.R, D.p, "TLIO", m.id, D.VM, order))
    return false;

  for (const auto &type : order) {
    if (map_hx.find(type) == map_hx.end()) {
      map_hx.insert({type, total_hx});
      Hx_order.push_back(type);
      total_hx += type->size();
    }
  }

  Matrix3d R_GtoI;
  Vector3d p_IinG;

  if (!state->get_interpolated_pose(m.time, R_GtoI, p_IinG))
    return false;
  Matrix3d R_ItoG = R_GtoI.transpose();
  // residual
  VectorXd res = VectorXd::Zero(6);
  res.head(3) = -log_so3(exp_so3(m.meas.head(3)) * (R_GtoI).transpose());
  res.tail(3) = m.meas.tail(3) - (p_IinG);

  PRINT4("Meas: %3.3f, %3.3f, %3.3f .\n", m.meas(0), m.meas(1), m.meas(2));
  PRINT4("Pred: %3.3f, %3.3f, %3.3f .\n", p_IinG(0), p_IinG(1), p_IinG(2));
  PRINT4("Resi: %3.3f, %3.3f, %3.3f .\n", res(0), res(1), res(2));
  

  vector<MatrixXd> dTdx;
  bool success = state->get_interpolated_jacobian(m.time, R_GtoI, p_IinG, "TLIO", m.id, dTdx, order);
  assert(success);
  
  // Measurement matrix with respect to variables
  MatrixXd H = MatrixXd::Zero(m.meas.size(), total_hx);

  MatrixXd dz_dI = MatrixXd::Zero(3, 6);
  dz_dI.block(0, 0, 3, 3) = -R_GtoI.transpose();
  dz_dI.block(0, 3, 3, 3) = Matrix3d::Identity();

  // CHAINRULE: get state clone Jacobian. This also adds timeoffset jacobian
  for (int i = 0; i < (int)dTdx.size(); i++) {
    assert(map_hx.find(order.at(i)) != map_hx.end());
    H.block(0, map_hx.at(order.at(i)), 3, order.at(i)->size()).noalias() += dz_dI * dTdx.at(i);
  }

  // Just take the estimated covariance
  MatrixXd R = m.cov;

  PRINT4("Cov (diag): %3.3f, %3.3f, %3.3f\n", R(0, 0), R(1, 1), R(2, 2));

  PRINT4("Chi: %s\n", Chi->Chi2Check(state, Hx_order, H, res, R) ? "yes" : "no");
  if (Chi->Chi2Check(state, Hx_order, H, res, R)) {
    StateHelper::EKFUpdate(state, Hx_order, H, res, R, "TLIO");
    return true;
  }

  return false;
}

void UpdaterTLIO::feed_measurement(const TLIOData &data) {
  data_stack.push_back(data);
  t_hist.size() > 100 ? t_hist.pop_front() : void();
  t_hist.push_back(data.time);
}

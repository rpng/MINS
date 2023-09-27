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

#include "StateHelper.h"
#include "State.h"
#include "cam/CamBase.h"
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "types/IMU.h"
#include "types/Landmark.h"
#include "types/PoseJPL.h"
#include "types/Type.h"
#include "utils/Print_Logger.h"
#include "utils/colors.h"
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace mins;
using namespace std;
using namespace Eigen;

void StateHelper::EKFPropagation(shared_ptr<State> state, const VEC_TYPE &order_NEW, const VEC_TYPE &order_OLD, const MatrixXd &Phi, const MatrixXd &Q) {

  // We need at least one old and new variable
  if (order_NEW.empty() || order_OLD.empty()) {
    PRINT4(RED "StateHelper::EKFPropagation() - Called with empty variable arrays!\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Loop through our Phi order and ensure that they are continuous in memory
  int size_order_NEW = order_NEW.at(0)->size();
  for (size_t i = 0; i < order_NEW.size() - 1; i++) {
    if (order_NEW[i]->id() + order_NEW[i]->size() != order_NEW[i + 1]->id()) {
      PRINT4(RED "StateHelper::EKFPropagation() - Called with non-contiguous state elements!\n" RESET);
      PRINT4(RED "StateHelper::EKFPropagation() - This code only support a state transition which is in the same order as the state\n" RESET);
      exit(EXIT_FAILURE);
    }
    size_order_NEW += order_NEW[i + 1]->size();
  }

  // Size of the old phi matrix
  int size_order_OLD = order_OLD.at(0)->size();
  for (size_t i = 0; i < order_OLD.size() - 1; i++) {
    size_order_OLD += order_OLD[i + 1]->size();
  }

  // Assert that we have correct sizes
  assert(size_order_NEW == Phi.rows());
  assert(size_order_OLD == Phi.cols());
  assert(size_order_NEW == Q.cols());
  assert(size_order_NEW == Q.rows());

  // Get the location in small phi for each measuring variable
  int current_it = 0;
  vector<int> Phi_id;
  for (const auto &var : order_OLD) {
    Phi_id.push_back(current_it);
    current_it += var->size();
  }

  // Loop through all our old states and get the state transition times it
  // Cov_PhiT = [ Pxx ] [ Phi' ]'
  MatrixXd Cov_PhiT = MatrixXd::Zero(state->cov.rows(), Phi.rows());
  for (size_t i = 0; i < order_OLD.size(); i++) {
    shared_ptr<Type> var = order_OLD[i];
    Cov_PhiT.noalias() += state->cov.block(0, var->id(), state->cov.rows(), var->size()) * Phi.block(0, Phi_id[i], Phi.rows(), var->size()).transpose();
  }

  // Get Phi_NEW*Covariance*Phi_NEW^t + Q
  MatrixXd Phi_Cov_PhiT = Q.selfadjointView<Upper>();
  for (size_t i = 0; i < order_OLD.size(); i++) {
    shared_ptr<Type> var = order_OLD[i];
    Phi_Cov_PhiT.noalias() += Phi.block(0, Phi_id[i], Phi.rows(), var->size()) * Cov_PhiT.block(var->id(), 0, var->size(), Phi.rows());
  }

  // We are good to go!
  int start_id = order_NEW.at(0)->id();
  int phi_size = Phi.rows();
  int total_size = state->cov.rows();
  state->cov.block(start_id, 0, phi_size, total_size) = Cov_PhiT.transpose();
  state->cov.block(0, start_id, total_size, phi_size) = Cov_PhiT;
  state->cov.block(start_id, start_id, phi_size, phi_size) = Phi_Cov_PhiT;

  // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
  VectorXd diags = state->cov.diagonal();
  bool found_neg = false;
  for (int i = 0; i < diags.rows(); i++) {
    if (diags(i) < 0.0) {
      PRINT3(RED "StateHelper::EKFPropagation() - diagonal at %d is %.6f\n" RESET, i, diags(i));
      found_neg = true;
    }
  }
  assert(!found_neg);
}

bool StateHelper::EKFUpdate(shared_ptr<State> state, const VEC_TYPE &H_order, const MatrixXd &H, const VectorXd &res, const MatrixXd &R, string type) {

  //==========================================================
  //==========================================================
  // Part of the Kalman Gain K = (P*H^T)*S^{-1} = M*S^{-1}
  assert(res.rows() == R.rows());
  assert(H.rows() == res.rows());
  MatrixXd M_a = MatrixXd::Zero(state->cov.rows(), res.rows());

  // Get the location in small jacobian for each measuring variable
  int current_it = 0;
  vector<int> H_id;
  for (const auto &meas_var : H_order) {
    H_id.push_back(current_it);
    current_it += meas_var->size();
  }

  //==========================================================
  //==========================================================
  // For each active variable find its M = P*H^T
  for (const auto &var : state->variables) {
    // Sum up effect of each subjacobian = K_i= \sum_m (P_im Hm^T)
    for (size_t i = 0; i < H_order.size(); i++) {
      const shared_ptr<Type> &meas_var = H_order.at(i);
      M_a.block(var->id(), 0, var->size(), res.rows()).noalias() +=
          state->cov.block(var->id(), meas_var->id(), var->size(), meas_var->size()) * H.block(0, H_id[i], H.rows(), meas_var->size()).transpose();
    }
  }

  //==========================================================
  //==========================================================
  // Get covariance of the involved terms
  MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);

  // Residual covariance S = H*Cov*H' + R
  MatrixXd S(R.rows(), R.rows());
  S.triangularView<Upper>() = H * P_small * H.transpose();
  S.triangularView<Upper>() += R;

  // Compute Kalman gain
  MatrixXd Sinv = MatrixXd::Identity(R.rows(), R.rows());
  S.selfadjointView<Upper>().llt().solveInPlace(Sinv);
  MatrixXd K = M_a * Sinv.selfadjointView<Upper>();

  // dx, dx_Cov
  VectorXd dx = K * res;
  MatrixXd dx_Cov(state->cov.rows(), state->cov.rows());
  dx_Cov.triangularView<Upper>() = (K * M_a.transpose());

  // We should check if we are not positive semi-definitate
  VectorXd diag1 = state->cov.diagonal();
  VectorXd diag2 = dx_Cov.diagonal();
  assert(diag1.rows() == diag2.rows());
  for (int i = 0; i < diag1.rows(); i++) {
    if (diag1(i) - diag2(i) < 0.0) {
      PRINT3(RED "StateHelper::EKFUpdate::%s - diagonal has %.2f\n" RESET, type.c_str(), diag1(i) - diag2(i));
      return false;
    }
  }

  // Update Covariance
  state->cov.triangularView<Upper>() -= dx_Cov;
  state->cov.triangularView<Lower>() = state->cov.triangularView<Upper>().transpose();

  // Calculate our delta and update all our active states
  for (auto &variable : state->variables)
    variable->update(dx.block(variable->id(), 0, variable->size(), 1));

  // If we are doing online intrinsic calibration we should update our camera objects
  // NOTE: is this the best place to put this update logic??? probably..
  if (state->op->cam->do_calib_int) {
    for (auto const &calib : state->cam_intrinsic) {
      state->cam_intrinsic_model.at(calib.first)->set_value(calib.second->value());
    }
  }

  // build est polynomial for the new state when not using imu prediction
  state->op->use_imu_res ? void() : state->build_polynomial_data(false);
  return true;
}

void StateHelper::set_initial_covariance(shared_ptr<State> state, const MatrixXd &covariance, const vector<shared_ptr<ov_type::Type>> &order) {

  // We need to loop through each element and overwrite the current covariance values
  // For example consider the following:
  // x = [ ori pos ] -> insert into -> x = [ ori bias pos ]
  // P = [ P_oo P_op ] -> P = [ P_oo  0   P_op ]
  //     [ P_po P_pp ]        [  0    P*    0  ]
  //                          [ P_po  0   P_pp ]
  // The key assumption here is that the covariance is block diagonal (cross-terms zero with P* can be dense)
  // This is normally the care on startup (for example between calibration and the initial state

  // For each variable, lets copy over all other variable cross terms
  // Note: this copies over itself to when i_index=k_index
  int i_index = 0;
  for (size_t i = 0; i < order.size(); i++) {
    int k_index = 0;
    for (size_t k = 0; k < order.size(); k++) {
      state->cov.block(order[i]->id(), order[k]->id(), order[i]->size(), order[k]->size()) = covariance.block(i_index, k_index, order[i]->size(), order[k]->size());
      k_index += order[k]->size();
    }
    i_index += order[i]->size();
  }
  state->cov = state->cov.selfadjointView<Upper>();
}

MatrixXd StateHelper::get_marginal_covariance(shared_ptr<State> state, const VEC_TYPE &small_variables) {

  // Calculate the marginal covariance size we need to make our matrix
  int cov_size = 0;
  for (size_t i = 0; i < small_variables.size(); i++) {
    cov_size += small_variables[i]->size();
  }

  // Construct our return covariance
  MatrixXd Small_cov = MatrixXd::Zero(cov_size, cov_size);

  // For each variable, lets copy over all other variable cross terms
  // Note: this copies over itself to when i_index=k_index
  int i_index = 0;
  for (size_t i = 0; i < small_variables.size(); i++) {
    int k_index = 0;
    for (size_t k = 0; k < small_variables.size(); k++) {
      Small_cov.block(i_index, k_index, small_variables[i]->size(), small_variables[k]->size()) =
          state->cov.block(small_variables[i]->id(), small_variables[k]->id(), small_variables[i]->size(), small_variables[k]->size());
      k_index += small_variables[k]->size();
    }
    i_index += small_variables[i]->size();
  }

  // Return the covariance
  // Small_cov = 0.5*(Small_cov+Small_cov.transpose());
  return Small_cov;
}

MatrixXd StateHelper::get_full_covariance(shared_ptr<State> state) { return state->cov; }

void StateHelper::marginalize(shared_ptr<State> state, shared_ptr<Type> marg) {

  // Check if the current state has the element we want to marginalize
  if (find(state->variables.begin(), state->variables.end(), marg) == state->variables.end()) {
    PRINT4(RED "StateHelper::marginalize() - Called on variable that is not in the state\n" RESET);
    PRINT4(RED "StateHelper::marginalize() - Marginalization, does NOT work on sub-variables yet...\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Generic covariance has this form for x_1, x_m, x_2. If we want to remove x_m:
  //
  //  P_(x_1,x_1) P(x_1,x_m) P(x_1,x_2)
  //  P_(x_m,x_1) P(x_m,x_m) P(x_m,x_2)
  //  P_(x_2,x_1) P(x_2,x_m) P(x_2,x_2)
  //
  //  to
  //
  //  P_(x_1,x_1) P(x_1,x_2)
  //  P_(x_2,x_1) P(x_2,x_2)
  //
  // i.e. x_1 goes from 0 to marg_id, x_2 goes from marg_id+marg_size to Cov.rows() in the original covariance

  int marg_size = marg->size();
  int marg_id = marg->id();
  int x2_size = (int)state->cov.rows() - marg_id - marg_size;

  MatrixXd Cov_new(state->cov.rows() - marg_size, state->cov.rows() - marg_size);

  // P_(x_1,x_1)
  Cov_new.block(0, 0, marg_id, marg_id) = state->cov.block(0, 0, marg_id, marg_id);

  // P_(x_1,x_2)
  Cov_new.block(0, marg_id, marg_id, x2_size) = state->cov.block(0, marg_id + marg_size, marg_id, x2_size);

  // P_(x_2,x_1)
  Cov_new.block(marg_id, 0, x2_size, marg_id) = Cov_new.block(0, marg_id, marg_id, x2_size).transpose();

  // P(x_2,x_2)
  Cov_new.block(marg_id, marg_id, x2_size, x2_size) = state->cov.block(marg_id + marg_size, marg_id + marg_size, x2_size, x2_size);

  // Now set new covariance
  // state->cov.resize(Cov_new.rows(),Cov_new.cols());
  state->cov = Cov_new;
  // state->Cov() = 0.5*(Cov_new+Cov_new.transpose());
  assert(state->cov.rows() == Cov_new.rows());

  // Now we keep the remaining variables and update their ordering
  // Note: DOES NOT SUPPORT MARGINALIZING SUBVARIABLES YET!!!!!!!
  VEC_TYPE remaining_variables;
  for (size_t i = 0; i < state->variables.size(); i++) {
    // Only keep non-marginal states
    if (state->variables[i] != marg) {
      if (state->variables[i]->id() > marg_id) {
        // If the variable is "beyond" the marginal one in ordering, need to "move it forward"
        state->variables[i]->set_local_id(state->variables[i]->id() - marg_size);
      }
      remaining_variables.push_back(state->variables[i]);
    }
  }

  // Delete the old state variable to free up its memory
  // NOTE: we don't need to do this anymore since our variable is a shared ptr
  // NOTE: thus this is automatically managed, but this allows outside references to keep the old variable
  // delete marg;
  marg->set_local_id(-1);

  // Now set variables as the remaining ones
  state->variables = remaining_variables;
}

shared_ptr<Type> StateHelper::clone(shared_ptr<State> state, shared_ptr<Type> variable_to_clone) {

  // Get total size of new cloned variables, and the old covariance size
  int total_size = variable_to_clone->size();
  int old_size = (int)state->cov.rows();
  int new_loc = (int)state->cov.rows();

  // Resize both our covariance to the new size
  state->cov.conservativeResizeLike(MatrixXd::Zero(old_size + total_size, old_size + total_size));

  // What is the new state, and variable we inserted
  const VEC_TYPE new_variables = state->variables;
  shared_ptr<Type> new_clone = nullptr;

  // Loop through all variables, and find the variable that we are going to clone
  for (size_t k = 0; k < state->variables.size(); k++) {

    // Skip this if it is not the same
    // First check if the top level variable is the same, then check the sub-variables
    shared_ptr<Type> type_check = state->variables.at(k)->check_if_subvariable(variable_to_clone);
    if (state->variables.at(k) == variable_to_clone) {
      type_check = state->variables.at(k);
    } else if (type_check != variable_to_clone) {
      continue;
    }

    // So we will clone this one
    int old_loc = type_check->id();

    // Copy the covariance elements
    state->cov.block(new_loc, new_loc, total_size, total_size) = state->cov.block(old_loc, old_loc, total_size, total_size);
    state->cov.block(0, new_loc, old_size, total_size) = state->cov.block(0, old_loc, old_size, total_size);
    state->cov.block(new_loc, 0, total_size, old_size) = state->cov.block(old_loc, 0, total_size, old_size);

    // Create clone from the type being cloned
    new_clone = type_check->clone();
    new_clone->set_local_id(new_loc);
    break;
  }

  // Check if the current state has this variable
  if (new_clone == nullptr) {
    PRINT4(RED "StateHelper::clone() - Called on variable is not in the state\n" RESET);
    PRINT4(RED "StateHelper::clone() - Ensure that the variable specified is a variable, or sub-variable..\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Add to variable list and return
  state->variables.push_back(new_clone);
  return new_clone;
}

bool StateHelper::initialize(shared_ptr<State> state, shared_ptr<Type> new_variable, const VEC_TYPE &H_order, MatrixXd &H_R, MatrixXd &H_L, MatrixXd &R, VectorXd &res,
                             double chi_2_mult, string type, bool health_check) {

  // Check that this new variable is not already initialized
  if (find(state->variables.begin(), state->variables.end(), new_variable) != state->variables.end()) {
    PRINT4("StateHelper::initialize_invertible - Called on variable that is already in the state\n");
    PRINT4("StateHelper::initialize_invertible - Found this variable at %d in covariance\n", new_variable->id());
    exit(EXIT_FAILURE);
  }

  // Check that we have isotropic noise (i.e. is diagonal and all the same value)
  assert(R.rows() == R.cols());
  assert(R.rows() > 0);
  for (int r = 0; r < R.rows(); r++) {
    for (int c = 0; c < R.cols(); c++) {
      if (r == c && R(0, 0) != R(r, c)) {
        PRINT4(RED "StateHelper::initialize - Your noise is not isotropic!\n" RESET);
        PRINT4(RED "StateHelper::initialize - Found a value of %.2f verses value of %.2f\n" RESET, R(r, c), R(0, 0));
        exit(EXIT_FAILURE);
      } else if (r != c && R(r, c) != 0.0) {
        PRINT4(RED "StateHelper::initialize - Your noise is not diagonal!\n" RESET);
        PRINT4(RED "StateHelper::initialize - Found a value of %.2f at row %d and column %d\n" RESET, R(r, c), r, c);
        exit(EXIT_FAILURE);
      }
    }
  }

  //==========================================================
  //==========================================================
  // First we perform QR givens to separate the system
  // The top will be a system that depends on the new state, while the bottom does not
  assert(new_variable->size() == H_L.cols());
  Givens_Rotation(H_L, H_R, res);

  // Separate into initializing and updating portions
  // 1. Invertible initializing system
  size_t new_var_size = new_variable->size();
  MatrixXd Hxinit = H_R.block(0, 0, new_var_size, H_R.cols());
  MatrixXd H_finit = H_L.block(0, 0, new_var_size, new_var_size);
  VectorXd resinit = res.block(0, 0, new_var_size, 1);
  MatrixXd Rinit = R.block(0, 0, new_var_size, new_var_size);

  // 2. Nullspace projected updating system
  MatrixXd Hup = H_R.block(new_var_size, 0, H_R.rows() - new_var_size, H_R.cols());
  VectorXd resup = res.block(new_var_size, 0, res.rows() - new_var_size, 1);
  MatrixXd Rup = R.block(new_var_size, new_var_size, R.rows() - new_var_size, R.rows() - new_var_size);

  //==========================================================
  //==========================================================

  // Do mahalanobis distance testing
  if (health_check) {
    MatrixXd P_up = get_marginal_covariance(state, H_order);
    assert(Rup.rows() == Hup.rows());
    assert(Hup.cols() == P_up.cols());
    MatrixXd S = Hup * P_up * Hup.transpose() + Rup;
    double chi2 = resup.transpose() * S.inverse().selfadjointView<Upper>() * resup;

    // Get what our threshold should be
    boost::math::chi_squared chi_squared_dist(res.rows());
    double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    if (chi2 > chi_2_mult * chi2_check) {
      return false;
    }
  }

  //==========================================================
  //==========================================================
  // Finally, initialize it in our state
  if (!StateHelper::initialize_invertible(state, new_variable, H_order, Hxinit, H_finit, Rinit, resinit, type, health_check))
    return false;

  // Update with updating portion
  if (Hup.rows() > 0 && !StateHelper::EKFUpdate(state, H_order, Hup, resup, Rup, type)) {
    // Initialization was successful but update after it is bad.
    // Revert the initialization
    marginalize(state, state->variables.back());
    return false;
  }

  // Return success
  return true;
}

bool StateHelper::initialize_invertible(shared_ptr<State> state, shared_ptr<Type> new_variable, const VEC_TYPE &H_order, const MatrixXd &H_R, const MatrixXd &H_L,
                                        const MatrixXd &R, const VectorXd &res, string type, bool health_check) {

  // Check that this new variable is not already initialized
  if (find(state->variables.begin(), state->variables.end(), new_variable) != state->variables.end()) {
    PRINT4("StateHelper::initialize_invertible - Called on variable that is already in the state\n");
    PRINT4("StateHelper::initialize_invertible - Found this variable at %d in covariance\n", new_variable->id());
    exit(EXIT_FAILURE);
  }

  // Check that we have isotropic noise (i.e. is diagonal and all the same value)
  // TODO: can we simplify this so it doesn't take as much time?
  assert(R.rows() == R.cols());
  assert(R.rows() > 0);
  for (int r = 0; r < R.rows(); r++) {
    for (int c = 0; c < R.cols(); c++) {
      if (r == c && R(0, 0) != R(r, c)) {
        PRINT4(RED "StateHelper::initialize_invertible - Your noise is not isotropic!\n" RESET);
        PRINT4(RED "StateHelper::initialize_invertible - Found a value of %.2f verses value of %.2f\n" RESET, R(r, c), R(0, 0));
        exit(EXIT_FAILURE);
      } else if (r != c && R(r, c) != 0.0) {
        PRINT4(RED "StateHelper::initialize_invertible - Your noise is not diagonal!\n" RESET);
        PRINT4(RED "StateHelper::initialize_invertible - Found a value of %.2f at row %d and column %d\n" RESET, R(r, c), r, c);
        exit(EXIT_FAILURE);
      }
    }
  }

  //==========================================================
  //==========================================================
  // Part of the Kalman Gain K = (P*H^T)*S^{-1} = M*S^{-1}
  assert(res.rows() == R.rows());
  assert(H_L.rows() == res.rows());
  assert(H_L.rows() == H_R.rows());
  MatrixXd M_a = MatrixXd::Zero(state->cov.rows(), res.rows());

  // Get the location in small jacobian for each measuring variable
  int current_it = 0;
  vector<int> H_id;
  for (const auto &meas_var : H_order) {
    H_id.push_back(current_it);
    current_it += meas_var->size();
  }

  //==========================================================
  //==========================================================
  // For each active variable find its M = P*H^T
  for (const auto &var : state->variables) {
    // Sum up effect of each subjacobian= K_i= \sum_m (P_im Hm^T)
    MatrixXd M_i = MatrixXd::Zero(var->size(), res.rows());
    for (size_t i = 0; i < H_order.size(); i++) {
      shared_ptr<Type> meas_var = H_order[i];
      M_i += state->cov.block(var->id(), meas_var->id(), var->size(), meas_var->size()) * H_R.block(0, H_id[i], H_R.rows(), meas_var->size()).transpose();
    }
    M_a.block(var->id(), 0, var->size(), res.rows()) = M_i;
  }

  //==========================================================
  //==========================================================
  // Get covariance of this small jacobian
  MatrixXd P_small = StateHelper::get_marginal_covariance(state, H_order);

  // M = H_R*Cov*H_R' + R
  MatrixXd M(H_R.rows(), H_R.rows());
  M.triangularView<Upper>() = H_R * P_small * H_R.transpose() + R;

  // Covariance of the variable/landmark that will be initialized
  assert(H_L.rows() == H_L.cols());
  assert(H_L.rows() == new_variable->size());
  MatrixXd H_Linv = H_L.inverse();
  MatrixXd P_LL = H_Linv * M.selfadjointView<Upper>() * H_Linv.transpose();

  // Discard suspicious case
  if (health_check) {
    double chi = (H_Linv * res).transpose() * P_LL.inverse() * (H_Linv * res);
    if (chi < 1e-7 || P_LL.diagonal().norm() > 1000) {
      PRINT1(RED "StateHelper::initialize_invertible - Reject suspecious initialization chi: %.3f, P: %.3f\n" RESET, chi, P_LL.diagonal().norm());
      return false;
    }
  }

  // We should check if we are not positive semi-definitate (i.e. negative diagionals is not s.p.d)
  VectorXd diags = P_LL.diagonal();
  for (int i = 0; i < diags.rows(); i++) {
    if (diags(i) < 0.0) {
      PRINT3(RED "StateHelper::initialize_invertible::%s - P_LL diagonal has %.2f\n" RESET, type.c_str(), diags(i));
      return false;
    }
  }

  // Augment the covariance matrix
  size_t oldSize = state->cov.rows();
  state->cov.conservativeResizeLike(MatrixXd::Zero(oldSize + new_variable->size(), oldSize + new_variable->size()));
  state->cov.block(0, oldSize, oldSize, new_variable->size()).noalias() = -M_a * H_Linv.transpose();
  state->cov.block(oldSize, 0, new_variable->size(), oldSize) = state->cov.block(0, oldSize, oldSize, new_variable->size()).transpose();
  state->cov.block(oldSize, oldSize, new_variable->size(), new_variable->size()) = P_LL;

  // Update the variable that will be initialized (invertible systems can only update the new variable).
  // However this update should be almost zero if we already used a conditional Gauss-Newton to solve for the initial estimate
  new_variable->update(H_Linv * res);

  // Now collect results, and add it to the state variables
  new_variable->set_local_id(oldSize);
  state->variables.push_back(new_variable);
  return true;
}

void StateHelper::augment_clone(shared_ptr<State> state) {

  // We can't insert a clone that occured at the same timestamp!
  if (state->have_clone(state->time)) {
    PRINT4(RED "TRIED TO INSERT A CLONE AT THE SAME TIME AS AN EXISTING CLONE, EXITING!#!@#!@#\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Call on our cloner and add it to our vector of types
  // NOTE: this will clone the clone pose to the END of the covariance...
  shared_ptr<Type> posetemp = StateHelper::clone(state, state->imu->pose());

  // Cast to a JPL pose type, check if valid
  shared_ptr<PoseJPL> pose = dynamic_pointer_cast<PoseJPL>(posetemp);
  if (pose == nullptr) {
    PRINT4(RED "INVALID OBJECT RETURNED FROM STATEHELPER CLONE, EXITING!#!@#!@#\n" RESET);
    exit(EXIT_FAILURE);
  }

  // Append the new clone to our clone vector
  state->clones[state->time] = pose;

  // build est polynomial for the new state when not using imu prediction
  state->op->use_imu_res ? void() : state->build_polynomial_data(false);

  state->add_polynomial();
}
void StateHelper::marginalize_slam(shared_ptr<State> state) {
  // Remove SLAM features that have their marginalization flag set
  // We also check that we do not remove any aruoctag landmarks
  for (auto it0 = state->cam_SLAM_features.begin(); it0 != state->cam_SLAM_features.end();) {
    if ((*it0).second->should_marg) {
      marginalize(state, (*it0).second);
      it0 = state->cam_SLAM_features.erase(it0);
    } else {
      it0++;
    }
  }
}
void StateHelper::marginalize_old_clone(shared_ptr<State> state) {
  while (state->clone_window() > state->op->window_size) {
    double marginal_time = state->oldest_clone_time();
    assert(marginal_time != -INFINITY);

    // check if we want to make this keyframe
    auto idx = find(state->keyframes_candidate.begin(), state->keyframes_candidate.end(), marginal_time);
    if (idx != state->keyframes_candidate.end()) {
      state->keyframes.push_back(marginal_time);
      state->keyframes_candidate.erase(idx);
      continue;
    }

    // otherwise, marginalize this time.
    marginalize(state, state->clones.at(marginal_time));
    // Note that the marginalizer should have already deleted the clone
    // Thus we just need to remove the pointer to it from our state
    state->clones.erase(marginal_time);
  }
}

void StateHelper::measurement_compress_inplace(MatrixXd &H_x, VectorXd &res) {

  // Return if H_x is a fat matrix (there is no need to compress in this case)
  if (H_x.rows() <= H_x.cols())
    return;

  // Do measurement compression through givens rotations
  Givens_Rotation(H_x, res);

  // Construct the smaller jacobian and residual after measurement compression
  H_x.conservativeResize(H_x.cols(), H_x.cols());
  res.conservativeResize(H_x.cols(), res.cols());
}

void StateHelper::nullspace_project_inplace(MatrixXd &H_f, MatrixXd &H_x, VectorXd &res) {

  // Apply the left nullspace of H_f to all variables through givens rotations
  Givens_Rotation(H_f, H_x, res);

  // The H_f jacobian max rank is 3 if it is a 3d position, thus size of the left nullspace is Hf.rows()-3
  // NOTE: need to eigen3 eval here since this experiences aliasing!
  // H_f = H_f.block(H_f.cols(),0,H_f.rows()-H_f.cols(),H_f.cols()).eval();
  H_x = H_x.block(H_f.cols(), 0, H_x.rows() - H_f.cols(), H_x.cols()).eval();
  res = res.block(H_f.cols(), 0, res.rows() - H_f.cols(), res.cols()).eval();

  // Sanity check
  assert(H_x.rows() == res.rows());
}

void StateHelper::Givens_Rotation(MatrixXd &A, MatrixXd &B, VectorXd &C) {
  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  // They use "matlab" index notation, thus we need to subtract 1 from all index
  JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < A.cols(); ++n) {
    for (int m = (int)A.rows() - 1; m > n; m--) {
      if (A(m, n) == 0.0)
        continue;
      // Givens matrix G
      tempHo_GR.makeGivens(A(m - 1, n), A(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      // it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (A.block(m - 1, n, 2, A.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      A(m, n) = 0; // enforce 0 for better consistency
      (B.block(m - 1, 0, 2, B.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (C.block(m - 1, 0, 2, C.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }
}

void StateHelper::Givens_Rotation(MatrixXd &A, VectorXd &B) {
  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  // They use "matlab" index notation, thus we need to subtract 1 from all index
  JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < A.cols(); ++n) {
    for (int m = (int)A.rows() - 1; m > n; m--) {
      if (A(m, n) == 0.0)
        continue;
      // Givens matrix G
      tempHo_GR.makeGivens(A(m - 1, n), A(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      // it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (A.block(m - 1, n, 2, A.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      A(m, n) = 0; // enforce 0 for better consistency
      (B.block(m - 1, 0, 2, B.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }
}

void StateHelper::insert_map(vector<shared_ptr<Type>> xs, MAP_TYPE_SIZE &map, vector<shared_ptr<Type>> &order, int &size) {
  for (const auto &x : xs) {
    if (map.find(x) == map.end()) {
      map.insert({x, size});
      order.push_back(x);
      size += x->size();
    }
  }
}
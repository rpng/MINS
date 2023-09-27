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

#include "Propagator.h"
#include "cpi/CpiV1.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsIMU.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "utils/Print_Logger.h"
#include "utils/colors.h"
#include "utils/sensor_data.h"

using namespace mins;
using namespace ov_core;

Propagator::Propagator(shared_ptr<State> state) : state(state) {}

void Propagator::propagate(double timestamp) {
  // First lets construct an IMU vector of measurements we need
  vector<ImuData> data;
  if (!select_imu_readings(state->time, timestamp, data))
    return;

  // We are going to sum up all the state transition matrices, so we can do a single large multiplication at the end
  // Phi_summed = Phi_i*Phi_summed
  // Q_summed = Phi_i*Q_summed*Phi_i^T + Q_i
  // After summing we can multiply the total phi to get the updated covariance
  // We will then add the noise to the IMU portion of the state

  // Loop through all IMU messages, and use them to move the state forward in time
  // This uses the zero'th order quat, and then constant acceleration discrete
  Matrix15 Phi_summed = Matrix15::Identity();
  Matrix15 Qd_summed = Matrix15::Zero();
  Matrix3d R_GtoIk = state->imu->Rot();
  for (size_t i = 0; i < data.size() - 1; i++) {

    // Get the next state Jacobian and noise Jacobian for this IMU reading
    Matrix15 F = Matrix15::Zero();
    Matrix15 Qdi = Matrix15::Zero();
    predict_and_compute(data[i], data[i + 1], F, Qdi);
    // Next we should propagate our IMU covariance
    // Pii' = F*Pii*F.transpose() + G*Q*G.transpose()
    // Pci' = F*Pci and Pic' = Pic*F.transpose()
    // NOTE: Here we are summing the state transition F, so we can do a single multiplication later
    // NOTE: Phi_summed = Phi_i*Phi_summed
    // NOTE: Q_summed = Phi_i*Q_summed*Phi_i^T + G*Q_i*G^T
    Phi_summed = F * Phi_summed;
    Qd_summed = F * Qd_summed * F.transpose() + Qdi;
    Qd_summed = 0.5 * (Qd_summed + Qd_summed.transpose());

    // Compute CPI information if cpiv1 is valid
    if (cpiv1 != nullptr) {
      cpiv1->feed_IMU(data[i].timestamp, data[i + 1].timestamp, data[i].wm, data[i].am, data[i + 1].wm, data[i + 1].am);
      State::CPI cpi;
      cpi.t = data[i + 1].timestamp;
      cpi.dt = cpiv1->DT;
      cpi.clone_t = cpi_clone_t;
      cpi.R_I0toIk = cpiv1->R_k2tau;
      cpi.alpha_I0toIk = cpiv1->alpha_tau;
      cpi.w = data[i + 1].wm - state->imu->bias_g();
      cpi.v = state->cpis.at(cpi_clone_t).v - state->op->gravity * cpiv1->DT + R_GtoIk.transpose() * cpiv1->beta_tau;
      cpi.bg = state->cpis.at(cpi_clone_t).bg;
      cpi.ba = state->cpis.at(cpi_clone_t).ba;
      cpi.Q.block(0, 0, 3, 3) = cpiv1->P_meas.block(0, 0, 3, 3);
      cpi.Q.block(0, 3, 3, 3) = cpiv1->P_meas.block(0, 12, 3, 3);
      cpi.Q.block(3, 0, 3, 3) = cpiv1->P_meas.block(12, 0, 3, 3);
      cpi.Q.block(3, 3, 3, 3) = cpiv1->P_meas.block(12, 12, 3, 3);
      R_GtoIk = cpiv1->R_k2tau * R_GtoIk; // integrate orientation
      state->cpis.insert({cpi.t, cpi});
    }
  }

  // Do the update to the covariance with our "summed" state transition and IMU noise addition...
  StateHelper::EKFPropagation(state, {state->imu}, {state->imu}, Phi_summed, Qd_summed);

  // Set timestamp data
  state->time = timestamp;
}

bool Propagator::select_imu_readings(double time0, double time1, vector<ImuData> &prop_data) {

  // Ensure we have some measurements in the first place!
  if (imu_data.size() < 2) {
    PRINT0(YELLOW "[Prop] Not enough IMU measurements. Requested: %.4f - %.4f\n" RESET, time0, time1);
    return false;
  }

  // Make sure forward request
  if (time1 <= time0) {
    time1 < time0 ? PRINT4(RED "Propagator::select_imu_readings::Backward request. time0: %.4f, time1: %.4f. \n" RESET, time0, time1) : void();
    return false;
  }

  // Make sure we have IMU data to process
  if (imu_data.front().timestamp > time0) {
    PRINT0(YELLOW "Propagator::select_imu_readings::Cannot handle request. " RESET);
    PRINT0(YELLOW "time0 %.4f < oldest imu %.4f\n" RESET, time0, imu_data.front().timestamp);
    return false;
  }

  // Make sure we have IMU data to process
  if (imu_data.back().timestamp < time1) {
    PRINT0(YELLOW "Propagator::select_imu_readings::Cannot handle request. " RESET);
    PRINT1(YELLOW "newest imu %.4f < time1 %.4f\n" RESET, imu_data.back().timestamp, time1);
    return false;
  }

  // Add the first data
  size_t i = 0;
  for (; i < imu_data.size() - 1; i++) {
    if (imu_data.at(i).timestamp <= time0 && time0 <= imu_data.at(i + 1).timestamp) {
      prop_data.push_back(interpolate_data(imu_data.at(i), imu_data.at(i + 1), time0));
      break;
    }
    assert(imu_data.at(i).timestamp <= time0);
  }

  // Add the middle data
  for (i == 0 ? i = 0 : i--; i < imu_data.size() - 1; i++) {
    if (time0 < imu_data.at(i).timestamp && imu_data.at(i + 1).timestamp < time1) {
      prop_data.push_back(imu_data.at(i));
    }
    if (imu_data.at(i + 1).timestamp > time1)
      break;
  }

  // Add the final data
  for (i == 0 ? i = 0 : i--; i < imu_data.size() - 1; i++) {
    if (imu_data.at(i).timestamp <= time1 && time1 <= imu_data.at(i + 1).timestamp) {
      prop_data.push_back(interpolate_data(imu_data.at(i), imu_data.at(i + 1), time1));
      break;
    }
    assert(imu_data.at(i).timestamp <= time1);
  }

  assert(time0 == prop_data.begin()->timestamp);
  assert(time1 == prop_data.rbegin()->timestamp);
  // Success :D
  return true;
}

void Propagator::predict_and_compute(const ImuData &data_minus, const ImuData &data_plus, Matrix15 &F, Matrix15 &Qd) {

  // Set them to zero
  F.setZero();
  Qd.setZero();

  // Time elapsed over interval
  double dt = data_plus.timestamp - data_minus.timestamp;
  // assert(data_plus.timestamp>data_minus.timestamp);

  // Corrected IMU measurements
  Vector3d w_hat = data_minus.wm - state->imu->bias_g();
  Vector3d a_hat = data_minus.am - state->imu->bias_a();
  Vector3d w_hat2 = data_plus.wm - state->imu->bias_g();
  Vector3d a_hat2 = data_plus.am - state->imu->bias_a();

  // Compute the new state mean value
  Vector4d new_q;
  Vector3d new_v, new_p;
  predict_mean_rk4(state->imu, dt, w_hat, a_hat, w_hat2, a_hat2, new_q, new_v, new_p);

  // Get the locations of each entry of the IMU state
  int th_id = state->imu->q()->id() - state->imu->id();
  int p_id = state->imu->p()->id() - state->imu->id();
  int v_id = state->imu->v()->id() - state->imu->id();
  int bg_id = state->imu->bg()->id() - state->imu->id();
  int ba_id = state->imu->ba()->id() - state->imu->id();

  // Allocate noise Jacobian
  Matrix<double, 15, 12> G = Matrix<double, 15, 12>::Zero();

  // Now compute Jacobian of new state wrt old state and noise
  // This is the change in the orientation from the end of the last prop to the current prop
  // This is needed since we need to include the "k-th" updated orientation information
  Matrix<double, 3, 3> Rfej = state->imu->Rot_fej();
  Matrix<double, 3, 3> dR = quat_2_Rot(new_q) * Rfej.transpose();

  Vector3d v_fej = state->imu->vel_fej();
  Vector3d p_fej = state->imu->pos_fej();

  F.block(th_id, th_id, 3, 3) = dR;
  F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-w_hat * dt) * dt;
  // F.block(th_id, bg_id, 3, 3).noalias() = -dR * Jr_so3(-log_so3(dR)) * dt;
  F.block(bg_id, bg_id, 3, 3).setIdentity();
  F.block(v_id, th_id, 3, 3).noalias() = -skew_x(new_v - v_fej + state->op->gravity * dt) * Rfej.transpose();
  // F.block(v_id, th_id, 3, 3).noalias() = -Rfej.transpose() * skew_x(Rfej*(new_v-v_fej+_gravity*dt));
  F.block(v_id, v_id, 3, 3).setIdentity();
  F.block(v_id, ba_id, 3, 3) = -Rfej.transpose() * dt;
  F.block(ba_id, ba_id, 3, 3).setIdentity();
  F.block(p_id, th_id, 3, 3).noalias() = -skew_x(new_p - p_fej - v_fej * dt + 0.5 * state->op->gravity * dt * dt) * Rfej.transpose();
  // F.block(p_id, th_id, 3, 3).noalias() = -0.5 * Rfej.transpose() * skew_x(2*Rfej*(new_p-p_fej-v_fej*dt+0.5*_gravity*dt*dt));
  F.block(p_id, v_id, 3, 3) = Matrix<double, 3, 3>::Identity() * dt;
  F.block(p_id, ba_id, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
  F.block(p_id, p_id, 3, 3).setIdentity();

  G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-w_hat * dt) * dt;
  // G.block(th_id, 0, 3, 3) = -dR * Jr_so3(-log_so3(dR)) * dt;
  G.block(v_id, 3, 3, 3) = -Rfej.transpose() * dt;
  G.block(p_id, 3, 3, 3) = -0.5 * Rfej.transpose() * dt * dt;
  G.block(bg_id, 6, 3, 3) = Matrix<double, 3, 3>::Identity();
  G.block(ba_id, 9, 3, 3) = Matrix<double, 3, 3>::Identity();

  // Construct our discrete noise covariance matrix
  // Note that we need to convert our continuous time noises to discrete
  // Equations (129) amd (130) of Trawny tech report
  Matrix<double, 12, 12> Qc = Matrix<double, 12, 12>::Zero();
  Qc.block(0, 0, 3, 3) = pow(state->op->imu->sigma_w, 2) / dt * Matrix<double, 3, 3>::Identity();
  Qc.block(3, 3, 3, 3) = pow(state->op->imu->sigma_a, 2) / dt * Matrix<double, 3, 3>::Identity();
  Qc.block(6, 6, 3, 3) = pow(state->op->imu->sigma_wb, 2) * dt * Matrix<double, 3, 3>::Identity();
  Qc.block(9, 9, 3, 3) = pow(state->op->imu->sigma_ab, 2) * dt * Matrix<double, 3, 3>::Identity();

  // Compute the noise injected into the state over the interval
  Qd = G * Qc * G.transpose();
  Qd = 0.5 * (Qd + Qd.transpose());

  // Now replace IMU estimate and fej with propagated values
  Matrix<double, 16, 1> imu_x = state->imu->value();
  imu_x.block(0, 0, 4, 1) = new_q;
  imu_x.block(4, 0, 3, 1) = new_p;
  imu_x.block(7, 0, 3, 1) = new_v;
  state->imu->set_value(imu_x);
  state->imu->set_fej(imu_x);
}

void Propagator::predict_mean_rk4(shared_ptr<ov_type::IMU> imu, double dt, const Vector3d &w_hat1, const Vector3d &a_hat1, const Vector3d &w_hat2, const Vector3d &a_hat2,
                                  Vector4d &new_q, Vector3d &new_v, Vector3d &new_p) {

  // Pre-compute things
  Vector3d w_hat = w_hat1;
  Vector3d a_hat = a_hat1;
  Vector3d w_alpha = (w_hat2 - w_hat1) / dt;
  Vector3d a_jerk = (a_hat2 - a_hat1) / dt;

  // y0 ================
  Vector4d q_0 = imu->quat();
  Vector3d p_0 = imu->pos();
  Vector3d v_0 = imu->vel();

  // k1 ================
  Vector4d dq_0 = {0, 0, 0, 1};
  Vector4d q0_dot = 0.5 * Omega(w_hat) * dq_0;
  Vector3d p0_dot = v_0;
  Matrix3d R_Gto0 = quat_2_Rot(quat_multiply(dq_0, q_0));
  Vector3d v0_dot = R_Gto0.transpose() * a_hat - state->op->gravity;

  Vector4d k1_q = q0_dot * dt;
  Vector3d k1_p = p0_dot * dt;
  Vector3d k1_v = v0_dot * dt;

  // k2 ================
  w_hat += 0.5 * w_alpha * dt;
  a_hat += 0.5 * a_jerk * dt;

  Vector4d dq_1 = quatnorm(dq_0 + 0.5 * k1_q);
  // Vector3d p_1 = p_0+0.5*k1_p;
  Vector3d v_1 = v_0 + 0.5 * k1_v;

  Vector4d q1_dot = 0.5 * Omega(w_hat) * dq_1;
  Vector3d p1_dot = v_1;
  Matrix3d R_Gto1 = quat_2_Rot(quat_multiply(dq_1, q_0));
  Vector3d v1_dot = R_Gto1.transpose() * a_hat - state->op->gravity;

  Vector4d k2_q = q1_dot * dt;
  Vector3d k2_p = p1_dot * dt;
  Vector3d k2_v = v1_dot * dt;

  // k3 ================
  Vector4d dq_2 = quatnorm(dq_0 + 0.5 * k2_q);
  // Vector3d p_2 = p_0+0.5*k2_p;
  Vector3d v_2 = v_0 + 0.5 * k2_v;

  Vector4d q2_dot = 0.5 * Omega(w_hat) * dq_2;
  Vector3d p2_dot = v_2;
  Matrix3d R_Gto2 = quat_2_Rot(quat_multiply(dq_2, q_0));
  Vector3d v2_dot = R_Gto2.transpose() * a_hat - state->op->gravity;

  Vector4d k3_q = q2_dot * dt;
  Vector3d k3_p = p2_dot * dt;
  Vector3d k3_v = v2_dot * dt;

  // k4 ================
  w_hat += 0.5 * w_alpha * dt;
  a_hat += 0.5 * a_jerk * dt;

  Vector4d dq_3 = quatnorm(dq_0 + k3_q);
  // Vector3d p_3 = p_0+k3_p;
  Vector3d v_3 = v_0 + k3_v;

  Vector4d q3_dot = 0.5 * Omega(w_hat) * dq_3;
  Vector3d p3_dot = v_3;
  Matrix3d R_Gto3 = quat_2_Rot(quat_multiply(dq_3, q_0));
  Vector3d v3_dot = R_Gto3.transpose() * a_hat - state->op->gravity;

  Vector4d k4_q = q3_dot * dt;
  Vector3d k4_p = p3_dot * dt;
  Vector3d k4_v = v3_dot * dt;

  // y+dt ================
  Vector4d dq = quatnorm(dq_0 + (1.0 / 6.0) * k1_q + (1.0 / 3.0) * k2_q + (1.0 / 3.0) * k3_q + (1.0 / 6.0) * k4_q);
  new_q = quat_multiply(dq, q_0);
  new_p = p_0 + (1.0 / 6.0) * k1_p + (1.0 / 3.0) * k2_p + (1.0 / 3.0) * k3_p + (1.0 / 6.0) * k4_p;
  new_v = v_0 + (1.0 / 6.0) * k1_v + (1.0 / 3.0) * k2_v + (1.0 / 3.0) * k3_v + (1.0 / 6.0) * k4_v;
}

bool Propagator::get_bounding_data(double t_given, vector<ImuData> &data_stack, ImuData &data1, ImuData &data2) {

  // check if requested time is in valid area
  if (t_given > data_stack.back().timestamp || t_given < data_stack.front().timestamp)
    return false;

  // data_stack is ascending order!
  for (int i = 0; i < (int)data_stack.size() - 1; i++) {
    if (t_given >= data_stack.at(i).timestamp && t_given < data_stack.at(i + 1).timestamp) {
      data1 = data_stack.at(i);
      data2 = data_stack.at(i + 1);
      return true;
    }
  }
  return false;
}

void Propagator::feed_imu(const ImuData &message) {
  // Append it to our vector
  imu_data.emplace_back(message);

  // record timestamps
  t_hist.size() > 100 ? t_hist.pop_front() : void(); // remove if we have too many
  t_hist.push_back(message.timestamp);

  // Loop through and delete IMU messages that are older than the oldest clone time
  for (auto data = imu_data.begin(); (*data).timestamp < state->oldest_keyframe_time() - 1;)
    data = imu_data.erase(data);
}

ImuData Propagator::interpolate_data(const ImuData &imu_1, const ImuData &imu_2, double timestamp) {
  // time-distance lambda
  double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
  // PRINT1("lambda - %d\n", lambda);
  // interpolate between the two times
  ImuData data;
  data.timestamp = timestamp;
  data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
  data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
  return data;
}

void Propagator::reset_cpi(double clone_t) {
  // make sure we have the clone in our state
  if (!state->have_clone(clone_t)) {
    PRINT4(RED "Requested to reset CPI at non-existing clone time (%.4f)!\n" RESET, clone_t);
    state->print_info();
    std::exit(EXIT_FAILURE);
  }

  // reset CPI class
  cpiv1 = make_shared<CpiV1>(state->op->imu->sigma_w, state->op->imu->sigma_wb, state->op->imu->sigma_a, state->op->imu->sigma_ab, true);
  cpiv1->setLinearizationPoints(state->imu->bias_g(), state->imu->bias_a());
  cpi_clone_t = clone_t;

  // reset the CPI info we have at this clone time.
  State::CPI cpi;
  cpi.t = clone_t;
  cpi.clone_t = clone_t;
  cpi.dt = 0;
  cpi.v = state->imu->vel();
  cpi.bg = state->imu->bias_g();
  cpi.ba = state->imu->bias_a();
  // erase the cpi value if we have it
  if (state->have_cpi(clone_t)) {
    cpi.w = state->cpis.at(clone_t).w;
  }
  state->cpis[cpi.t] = cpi;
}

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

#include "WheelJacobians.h"
#include "utils/quat_ops.h"

using namespace Eigen;
using namespace std;
using namespace ov_core;

namespace mins::wheel {

Vector3d ComputeResidual2D(const Matrix3d &R_GtoI0, const Vector3d &p_I0inG, const Matrix3d &R_GtoI1, const Vector3d &p_I1inG, const Matrix3d &R_ItoO, const Vector3d &p_IinO,
                           double th, double x, double y) {
  Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
  Vector3d e3(0, 0, 1);
  Matrix<double, 2, 3> Lambda = Matrix<double, 2, 3>::Zero();
  Lambda.block(0, 0, 2, 2) = Matrix2d::Identity();

  Vector3d res = Vector3d::Zero();
  double theta_est = e3.transpose() * log_so3(R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose());
  res(0) = theta_est - th;
  Vector2d d_est = Lambda * R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG - R_GtoI0.transpose() * pOinI);
  res.block(1, 0, 2, 1) = Vector2d(x, y) - d_est;
  return res;
}

pair<MatrixXd, MatrixXd> ComputeJacobians2D(const Matrix3d &R_GtoI0, const Vector3d &p_I0inG, const Matrix3d &R_GtoI1, const Vector3d &p_I1inG, const Matrix3d &R_ItoO,
                                            const Vector3d &p_IinO) {
  Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
  Matrix3d RO0toO1 = R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose();
  Matrix3d RO1toO0 = RO0toO1.transpose();
  Vector3d phi = log_so3(RO0toO1);
  Vector3d e3(0, 0, 1);
  Matrix<double, 2, 3> Lambda = Matrix<double, 2, 3>::Zero();
  Lambda.block(0, 0, 2, 2) = Matrix2d::Identity();

  // d log_so3(R * exp(dth)) / d(dth) = Jr(dth)^-1, corrects for SO(3) curvature.
  Matrix3d Jr_phi_inv = Jr_so3(phi).inverse();
  Matrix<double, 1, 3> dzr_dth0 = -e3.transpose() * Jr_phi_inv * R_ItoO;
  Matrix<double, 1, 3> dzr_dth1 = e3.transpose() * Jr_phi_inv * RO1toO0 * R_ItoO;
  Matrix<double, 2, 3> dzp_dth0 = Lambda * R_ItoO * skew_x(R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG));
  Matrix<double, 2, 3> dzp_dp0 = -Lambda * R_ItoO * R_GtoI0;
  Matrix<double, 2, 3> dzp_dth1 = -Lambda * R_ItoO * R_GtoI0 * R_GtoI1.transpose() * skew_x(pOinI);
  Matrix<double, 2, 3> dzp_dp1 = Lambda * R_ItoO * R_GtoI0;

  MatrixXd H_poses = MatrixXd::Zero(3, 12);
  H_poses.block(0, 0, 1, 3) = dzr_dth0;
  H_poses.block(0, 6, 1, 3) = dzr_dth1;
  H_poses.block(1, 0, 2, 3) = dzp_dth0;
  H_poses.block(1, 3, 2, 3) = dzp_dp0;
  H_poses.block(1, 6, 2, 3) = dzp_dth1;
  H_poses.block(1, 9, 2, 3) = dzp_dp1;

  Matrix<double, 1, 3> dzr_dthcalib = e3.transpose() * Jr_phi_inv * (RO1toO0 - Matrix3d::Identity());
  Matrix<double, 2, 3> dzp_dthcalib = Lambda * (skew_x(R_ItoO * R_GtoI0 * (p_I1inG - p_I0inG) - RO1toO0 * p_IinO) + RO1toO0 * skew_x(p_IinO));
  Matrix<double, 2, 3> dzp_dpcalib = Lambda * (-RO1toO0 + Matrix3d::Identity());

  MatrixXd H_ext = MatrixXd::Zero(3, 6);
  H_ext.block(0, 0, 1, 3) = dzr_dthcalib;
  H_ext.block(1, 0, 2, 3) = dzp_dthcalib;
  H_ext.block(1, 3, 2, 3) = dzp_dpcalib;

  return {H_poses, H_ext};
}

Matrix<double, 6, 1> ComputeResidual3D(const Matrix3d &R_GtoI0, const Vector3d &p_I0inG, const Matrix3d &R_GtoI1, const Vector3d &p_I1inG, const Matrix3d &R_ItoO,
                                       const Vector3d &p_IinO, const Matrix3d &R_3D, const Vector3d &p_3D) {
  Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
  Matrix3d RO0toO1 = R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose();
  Matrix<double, 6, 1> res = Matrix<double, 6, 1>::Zero();
  res.head(3) = -log_so3(R_3D * RO0toO1.transpose());
  res.tail(3) = p_3D - R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG - R_GtoI0.transpose() * pOinI);
  return res;
}

pair<MatrixXd, MatrixXd> ComputeJacobians3D(const Matrix3d &R_GtoI0, const Vector3d &p_I0inG, const Matrix3d &R_GtoI1, const Vector3d &p_I1inG, const Matrix3d &R_ItoO,
                                            const Vector3d &p_IinO) {
  Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
  Matrix3d RO0toO1 = R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose();
  Matrix3d RO1toO0 = RO0toO1.transpose();

  Matrix3d dzr_dth0 = -R_ItoO * R_GtoI1 * R_GtoI0.transpose();
  Matrix3d dzr_dth1 = R_ItoO;
  Matrix3d dzp_dth0 = R_ItoO * skew_x(R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG));
  Matrix3d dzp_dp0 = -R_ItoO * R_GtoI0;
  Matrix3d dzp_dth1 = -R_ItoO * R_GtoI0 * R_GtoI1.transpose() * skew_x(pOinI);
  Matrix3d dzp_dp1 = R_ItoO * R_GtoI0;

  MatrixXd H_poses = MatrixXd::Zero(6, 12);
  H_poses.block(0, 0, 3, 3) = dzr_dth0;
  H_poses.block(0, 6, 3, 3) = dzr_dth1;
  H_poses.block(3, 0, 3, 3) = dzp_dth0;
  H_poses.block(3, 3, 3, 3) = dzp_dp0;
  H_poses.block(3, 6, 3, 3) = dzp_dth1;
  H_poses.block(3, 9, 3, 3) = dzp_dp1;

  Matrix3d dzr_dthcalib = Matrix3d::Identity() - RO0toO1;
  Matrix3d dzp_dthcalib = skew_x(R_ItoO * R_GtoI0 * (p_I1inG - p_I0inG) - RO1toO0 * p_IinO) + RO1toO0 * skew_x(p_IinO);
  Matrix3d dzp_dpcalib = -RO1toO0 + Matrix3d::Identity();

  MatrixXd H_ext = MatrixXd::Zero(6, 6);
  H_ext.block(0, 0, 3, 3) = dzr_dthcalib;
  H_ext.block(3, 0, 3, 3) = dzp_dthcalib;
  H_ext.block(3, 3, 3, 3) = dzp_dpcalib;

  return {H_poses, H_ext};
}

VectorXd ComputeTimeOffsetJacobian(const MatrixXd &H_poses, const Vector3d &w0, const Vector3d &v0, const Vector3d &w1, const Vector3d &v1) {
  Matrix<double, 12, 1> vel;
  vel << w0, v0, w1, v1;
  return H_poses * vel;
}

void AccumulateIntrinsicJacobians2D(double dt, double w_l, double w_r, double th, double rl, double rr, double b, Matrix<double, 1, 3> &dth_di, Matrix<double, 1, 3> &dx_di,
                                    Matrix<double, 1, 3> &dy_di) {
  double w = (w_r * rr - w_l * rl) / b;
  double v = (w_r * rr + w_l * rl) / 2;

  Matrix<double, 1, 3> Hwx = Matrix<double, 1, 3>::Zero();
  Hwx(0, 0) = -w_l / b;
  Hwx(0, 1) = w_r / b;
  Hwx(0, 2) = -(w_r * rr - w_l * rl) / (b * b);
  Matrix<double, 1, 3> Hvx = Matrix<double, 1, 3>::Zero();
  Hvx(0, 0) = w_l / 2;
  Hvx(0, 1) = w_r / 2;

  double h_thw = dt;
  double h_xth = (v * (cos(th - w * dt) - cos(th))) / w;
  double h_yth = -(v * (sin(th - w * dt) - sin(th))) / w;
  double h_xw = (v * (sin(th - w * dt) - sin(th))) / w / w + (v * cos(th - w * dt) * dt) / w;
  double h_yw = (v * (cos(th - w * dt) - cos(th))) / w / w - (v * sin(th - w * dt) * dt) / w;
  double h_xv = -(sin(th - w * dt) - sin(th)) / w;
  double h_yv = -(cos(th - w * dt) - cos(th)) / w;

  if (abs(w) < SMALL_ANGULAR_RATE) {
    h_xth = v * sin(th) * dt;
    h_yth = v * cos(th) * dt;
    h_xw = v * sin(th) * dt * dt / 2;
    h_yw = v * cos(th) * dt * dt / 2;
    h_xv = cos(th) * dt;
    h_yv = -sin(th) * dt;
  }

  dx_di = dx_di + h_xth * dth_di + h_xw * Hwx + h_xv * Hvx;
  dy_di = dy_di + h_yth * dth_di + h_yw * Hwx + h_yv * Hvx;
  dth_di = dth_di + h_thw * Hwx;
}

void AccumulateIntrinsicJacobians3D(double dt, double w_l, double w_r, const Matrix3d &R_3D, double rl, double rr, double b, Matrix3d &dR_di, Matrix3d &dp_di) {
  Vector3d w(0, 0, (w_r * rr - w_l * rl) / b);
  Vector3d v((w_r * rr + w_l * rl) / 2, 0, 0);

  Matrix3d Hwx = Matrix3d::Zero();
  Hwx(2, 0) = -w_l / b;
  Hwx(2, 1) = w_r / b;
  Hwx(2, 2) = -(w_r * rr - w_l * rl) / (b * b);
  Matrix3d Hvx = Matrix3d::Zero();
  Hvx(0, 0) = w_l / 2;
  Hvx(0, 1) = w_r / 2;

  Matrix3d R_step = exp_so3(-w * dt);
  Matrix3d Hth = Jl_so3(-w * dt) * dt;

  dp_di = dp_di - R_3D.transpose() * skew_x(v * dt) * dR_di + R_3D.transpose() * Hvx * dt;
  dR_di = R_step * dR_di + Hth * Hwx;
}

Matrix<double, 6, 6> ComputePhiTr3D(const Matrix3d &R_3D, const Matrix3d &R_new, const Vector3d &p_3D, const Vector3d &new_p) {
  Matrix<double, 6, 6> Phi_tr = Matrix<double, 6, 6>::Zero();
  Phi_tr.block(0, 0, 3, 3) = R_new * R_3D.transpose();
  Phi_tr.block(3, 0, 3, 3) = -R_3D.transpose() * skew_x(R_3D * (new_p - p_3D));
  Phi_tr.block(3, 3, 3, 3) = Matrix3d::Identity();
  return Phi_tr;
}
} // namespace mins::wheel

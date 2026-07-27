// Verifies AccumulateIntrinsicJacobians2D/3D via central finite differences over N preintegration steps.
// The FD helper uses the same exact kinematics that the Jacobian derivation assumes.
#include <gtest/gtest.h>
#include <Eigen/Core>
#include "update/wheel/UpdaterWheel.h"
#include "utils/quat_ops.h"

using namespace mins;
using Eigen::Matrix3d;
using Eigen::Vector3d;

// Exact 2D kinematics for one step (matches the formulas used in AccumulateIntrinsicJacobians2D).
static void step2D(double dt, double w_l, double w_r, double rl, double rr, double b,
                   double &th, double &x, double &y) {
    double w = (w_r * rr - w_l * rl) / b;
    double v = (w_r * rr + w_l * rl) / 2;
    double th_prev = th;
    if (abs(w) < 0.0001) {
        x += v * cos(th_prev) * dt;
        y += -v * sin(th_prev) * dt;
    } else {
        x += -v * (sin(th_prev - w * dt) - sin(th_prev)) / w;
        y += -v * (cos(th_prev - w * dt) - cos(th_prev)) / w;
    }
    th = th_prev - w * dt;
}

// First-order 3D kinematics for one step (matches AccumulateIntrinsicJacobians3D derivation).
static void step3D(double dt, double w_l, double w_r, double rl, double rr, double b,
                   Matrix3d &R_3D, Vector3d &p_3D) {
    Vector3d w(0, 0, (w_r * rr - w_l * rl) / b);
    Vector3d v((w_r * rr + w_l * rl) / 2, 0, 0);
    p_3D += R_3D.transpose() * v * dt;
    R_3D = ov_core::exp_so3(-w * dt) * R_3D;
}

TEST(WheelIntrinsics2D, AnalyticalMatchesNumerical) {
    const double eps = 1e-6;
    const double tol = 1e-5;

    double rl = 0.1, rr = 0.11, b = 0.5;
    double dt = 0.01;

    // Three steps with varying wheel speeds.
    double wl[3] = {2.0, 2.1, 1.9};
    double wr[3] = {2.5, 2.4, 2.6};

    // Accumulate analytical Jacobians.
    Eigen::Matrix<double, 1, 3> dth_di = Eigen::Matrix<double, 1, 3>::Zero();
    Eigen::Matrix<double, 1, 3> dx_di  = Eigen::Matrix<double, 1, 3>::Zero();
    Eigen::Matrix<double, 1, 3> dy_di  = Eigen::Matrix<double, 1, 3>::Zero();
    double th = 0, x = 0, y = 0;
    for (int i = 0; i < 3; i++) {
        UpdaterWheel::AccumulateIntrinsicJacobians2D(dt, wl[i], wr[i], th, rl, rr, b, dth_di, dx_di, dy_di);
        step2D(dt, wl[i], wr[i], rl, rr, b, th, x, y);
    }

    // FD: perturb each intrinsic, re-run measurement integration.
    for (int j = 0; j < 3; j++) {
        double rl_p = rl, rr_p = rr, b_p = b;
        double rl_m = rl, rr_m = rr, b_m = b;
        if (j == 0) { rl_p += eps; rl_m -= eps; }
        else if (j == 1) { rr_p += eps; rr_m -= eps; }
        else { b_p += eps; b_m -= eps; }

        double th_p = 0, x_p = 0, y_p = 0;
        double th_m = 0, x_m = 0, y_m = 0;
        for (int i = 0; i < 3; i++) {
            step2D(dt, wl[i], wr[i], rl_p, rr_p, b_p, th_p, x_p, y_p);
            step2D(dt, wl[i], wr[i], rl_m, rr_m, b_m, th_m, x_m, y_m);
        }

        double fd_th = (th_p - th_m) / (2.0 * eps);
        double fd_x  = (x_p - x_m)  / (2.0 * eps);
        double fd_y  = (y_p - y_m)  / (2.0 * eps);

        // dth_di accumulates dt*d(w)/d(int), but th_next = th - w*dt, so dth_di = -d(th)/d(int) = -fd_th.
        EXPECT_NEAR(dth_di(j), -fd_th, tol) << "dth_di[" << j << "]";
        EXPECT_NEAR(dx_di(j),   fd_x,  tol) << "dx_di["  << j << "]";
        EXPECT_NEAR(dy_di(j),   fd_y,  tol) << "dy_di["  << j << "]";
    }
}

TEST(WheelIntrinsics3D, AnalyticalMatchesNumerical) {
    const double eps = 1e-6;
    const double tol = 1e-5;

    double rl = 0.1, rr = 0.11, b = 0.5;
    double dt = 0.01;

    double wl[3] = {2.0, 2.1, 1.9};
    double wr[3] = {2.5, 2.4, 2.6};

    // Accumulate analytical Jacobians.
    Matrix3d dR_di = Matrix3d::Zero();
    Matrix3d dp_di = Matrix3d::Zero();
    Matrix3d R_3D = Matrix3d::Identity();
    Vector3d p_3D = Vector3d::Zero();
    for (int i = 0; i < 3; i++) {
        UpdaterWheel::AccumulateIntrinsicJacobians3D(dt, wl[i], wr[i], R_3D, rl, rr, b, dR_di, dp_di);
        step3D(dt, wl[i], wr[i], rl, rr, b, R_3D, p_3D);
    }

    // FD: perturb each intrinsic.
    for (int j = 0; j < 3; j++) {
        double rl_p = rl, rr_p = rr, b_p = b;
        double rl_m = rl, rr_m = rr, b_m = b;
        if (j == 0) { rl_p += eps; rl_m -= eps; }
        else if (j == 1) { rr_p += eps; rr_m -= eps; }
        else { b_p += eps; b_m -= eps; }

        Matrix3d R_p = Matrix3d::Identity(); Vector3d p_p = Vector3d::Zero();
        Matrix3d R_m = Matrix3d::Identity(); Vector3d p_m = Vector3d::Zero();
        for (int i = 0; i < 3; i++) {
            step3D(dt, wl[i], wr[i], rl_p, rr_p, b_p, R_p, p_p);
            step3D(dt, wl[i], wr[i], rl_m, rr_m, b_m, R_m, p_m);
        }

        // FD rotation tangent: log(R_p * R_m^T) / (2*eps).
        // dR_di accumulates Jl*dt*Hwx; since w = (wr*rr - wl*rl)/b, d(w)/d(rl) = -wl/b < 0
        // while the FD tangent from (rl+eps) has +sign → dR_di(:,j) = -dR_fd.
        Vector3d dR_fd = ov_core::log_so3(R_p * R_m.transpose()) / (2.0 * eps);
        Vector3d dp_fd = (p_p - p_m) / (2.0 * eps);

        for (int row = 0; row < 3; row++) {
            EXPECT_NEAR(dR_di(row, j), -dR_fd(row), tol) << "dR_di[" << row << "," << j << "]";
            EXPECT_NEAR(dp_di(row, j),  dp_fd(row), tol) << "dp_di[" << row << "," << j << "]";
        }
    }
}

// Tests wheel intrinsic Jacobians at the residual level for Wheel2DAng and Wheel3DAng.
// FDs ComputeResidual2D/3D w.r.t. each intrinsic (rl, rr, b), comparing to the analytical H:
//   2D: H_int = -[dth_di; dx_di; dy_di]
//   3D: H_int = -[dR_di; dp_di]
// Sign=-1 matches Vec error-state convention (same as position columns in test_UpdaterWheel2D).
// Covers turning, straight (L'Hopital branch), and varying-speed motion.
#include <Eigen/Core>
#include <gtest/gtest.h>
#include "update/wheel/UpdaterWheel.h"
#include "utils/quat_ops.h"

using namespace mins;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

static void step2D(double dt, double wl, double wr, double rl, double rr, double b,
                   double &th, double &x, double &y) {
    double w = (wr * rr - wl * rl) / b;
    double v = (wr * rr + wl * rl) / 2;
    double th_prev = th;
    if (std::abs(w) < 0.0001) {
        x += v * std::cos(th_prev) * dt;
        y += -v * std::sin(th_prev) * dt;
    } else {
        x += -v * (std::sin(th_prev - w * dt) - std::sin(th_prev)) / w;
        y += -v * (std::cos(th_prev - w * dt) - std::cos(th_prev)) / w;
    }
    th = th_prev - w * dt;
}

static void step3D(double dt, double wl, double wr, double rl, double rr, double b,
                   Matrix3d &R_3D, Vector3d &p_3D) {
    Vector3d w(0, 0, (wr * rr - wl * rl) / b);
    Vector3d v((wr * rr + wl * rl) / 2, 0, 0);
    p_3D += R_3D.transpose() * v * dt;
    R_3D = ov_core::exp_so3(-w * dt) * R_3D;
}

static void integrate2D(const double *wl, const double *wr, int N, double dt,
                        double rl, double rr, double b,
                        double &th, double &x, double &y,
                        Eigen::Matrix<double, 1, 3> &dth_di,
                        Eigen::Matrix<double, 1, 3> &dx_di,
                        Eigen::Matrix<double, 1, 3> &dy_di) {
    th = x = y = 0;
    dth_di.setZero();
    dx_di.setZero();
    dy_di.setZero();
    for (int i = 0; i < N; i++) {
        UpdaterWheel::AccumulateIntrinsicJacobians2D(dt, wl[i], wr[i], th, rl, rr, b, dth_di, dx_di, dy_di);
        step2D(dt, wl[i], wr[i], rl, rr, b, th, x, y);
    }
}

static void integrate3D(const double *wl, const double *wr, int N, double dt,
                        double rl, double rr, double b,
                        Matrix3d &R_3D, Vector3d &p_3D,
                        Matrix3d &dR_di, Matrix3d &dp_di) {
    R_3D = Matrix3d::Identity();
    p_3D = Vector3d::Zero();
    dR_di = Matrix3d::Zero();
    dp_di = Matrix3d::Zero();
    for (int i = 0; i < N; i++) {
        UpdaterWheel::AccumulateIntrinsicJacobians3D(dt, wl[i], wr[i], R_3D, rl, rr, b, dR_di, dp_di);
        step3D(dt, wl[i], wr[i], rl, rr, b, R_3D, p_3D);
    }
}

static void run2DResidualTest(const double *wl, const double *wr, int N, const char *label) {
    const double eps = 1e-6;
    const double tol = 1e-5;
    const double rl = 0.1, rr = 0.11, b = 0.5, dt = 0.01;

    double th, x, y;
    Eigen::Matrix<double, 1, 3> dth_di, dx_di, dy_di;
    integrate2D(wl, wr, N, dt, rl, rr, b, th, x, y, dth_di, dx_di, dy_di);

    Matrix3d R_GtoI0 = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.15));
    Vector3d p_I0inG(1.0, 0.5, 0.0);
    Matrix3d R_GtoI1 = ov_core::exp_so3(Vector3d(0.05, -0.15, 0.3));
    Vector3d p_I1inG(2.0, 1.0, 0.0);
    Matrix3d R_ItoO = ov_core::exp_so3(Vector3d(0.0, 0.0, 0.2));
    Vector3d p_IinO(0.1, 0.0, -0.2);

    // H_int = -[dth_di; dx_di; dy_di]  (Vec sign convention: error = hat - true)
    MatrixXd H_int = MatrixXd::Zero(3, 3);
    H_int.row(0) = -dth_di;
    H_int.row(1) = -dx_di;
    H_int.row(2) = -dy_di;

    for (int j = 0; j < 3; j++) {
        double rl_p = rl, rr_p = rr, b_p = b;
        double rl_m = rl, rr_m = rr, b_m = b;
        if (j == 0) {
            rl_p += eps;
            rl_m -= eps;
        } else if (j == 1) {
            rr_p += eps;
            rr_m -= eps;
        } else {
            b_p += eps;
            b_m -= eps;
        }

        double th_p, x_p, y_p, th_m, x_m, y_m;
        Eigen::Matrix<double, 1, 3> dum1, dum2, dum3;
        integrate2D(wl, wr, N, dt, rl_p, rr_p, b_p, th_p, x_p, y_p, dum1, dum2, dum3);
        integrate2D(wl, wr, N, dt, rl_m, rr_m, b_m, th_m, x_m, y_m, dum1, dum2, dum3);

        Vector3d res_p = UpdaterWheel::ComputeResidual2D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO, th_p, x_p, y_p);
        Vector3d res_m = UpdaterWheel::ComputeResidual2D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO, th_m, x_m, y_m);

        Vector3d fd_col = -(res_p - res_m) / (2.0 * eps);

        for (int row = 0; row < 3; row++) {
            EXPECT_NEAR(H_int(row, j), fd_col(row), tol) << label << " row=" << row << " col=" << j;
        }
    }
}

static void run3DResidualTest(const double *wl, const double *wr, int N, const char *label) {
    const double eps = 1e-6;
    const double tol = 1e-5;
    const double rl = 0.1, rr = 0.11, b = 0.5, dt = 0.01;

    Matrix3d R_3D, dR_di;
    Vector3d p_3D;
    Matrix3d dp_di;
    integrate3D(wl, wr, N, dt, rl, rr, b, R_3D, p_3D, dR_di, dp_di);

    // Consistent poses: zero residual at nominal intrinsics.
    // With R_ItoO=I, R_GtoI0=I, R_GtoI1=R_3D, p_I1inG=p_3D the rotation residual
    // log(R_3D * R_3D^T) = 0, so Jr^{-1}(0) = I and the linear H is exact.
    Matrix3d R_GtoI0 = Matrix3d::Identity();
    Vector3d p_I0inG = Vector3d::Zero();
    Matrix3d R_GtoI1 = R_3D;
    Vector3d p_I1inG = p_3D;
    Matrix3d R_ItoO = Matrix3d::Identity();
    Vector3d p_IinO = Vector3d::Zero();

    // H_int = -[dR_di (3x3); dp_di (3x3)]  (Vec sign convention)
    MatrixXd H_int = MatrixXd::Zero(6, 3);
    H_int.block(0, 0, 3, 3) = -dR_di;
    H_int.block(3, 0, 3, 3) = -dp_di;

    for (int j = 0; j < 3; j++) {
        double rl_p = rl, rr_p = rr, b_p = b;
        double rl_m = rl, rr_m = rr, b_m = b;
        if (j == 0) {
            rl_p += eps;
            rl_m -= eps;
        } else if (j == 1) {
            rr_p += eps;
            rr_m -= eps;
        } else {
            b_p += eps;
            b_m -= eps;
        }

        Matrix3d R_p, R_m;
        Vector3d p_p, p_m;
        Matrix3d dR_dum, dp_dum;
        integrate3D(wl, wr, N, dt, rl_p, rr_p, b_p, R_p, p_p, dR_dum, dp_dum);
        integrate3D(wl, wr, N, dt, rl_m, rr_m, b_m, R_m, p_m, dR_dum, dp_dum);

        Eigen::Matrix<double, 6, 1> res_p_vec = UpdaterWheel::ComputeResidual3D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO, R_p, p_p);
        Eigen::Matrix<double, 6, 1> res_m_vec = UpdaterWheel::ComputeResidual3D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO, R_m, p_m);

        Eigen::Matrix<double, 6, 1> fd_col = -(res_p_vec - res_m_vec) / (2.0 * eps);

        for (int row = 0; row < 6; row++) {
            EXPECT_NEAR(H_int(row, j), fd_col(row), tol) << label << " row=" << row << " col=" << j;
        }
    }
}

TEST(WheelIntrinsic2D, TurningVaryingSpeed) {
    double wl[] = {2.0, 2.1, 1.9};
    double wr[] = {2.5, 2.4, 2.6};
    run2DResidualTest(wl, wr, 3, "2D TurningVaryingSpeed");
}

TEST(WheelIntrinsic2D, LargeTurn) {
    double wl[] = {1.0, 1.2, 0.8};
    double wr[] = {3.0, 2.8, 3.2};
    run2DResidualTest(wl, wr, 3, "2D LargeTurn");
}

TEST(WheelIntrinsic3D, TurningVaryingSpeed) {
    double wl[] = {2.0, 2.1, 1.9};
    double wr[] = {2.5, 2.4, 2.6};
    run3DResidualTest(wl, wr, 3, "3D TurningVaryingSpeed");
}

TEST(WheelIntrinsic3D, LargeTurn) {
    double wl[] = {1.0, 1.2, 0.8};
    double wr[] = {3.0, 2.8, 3.2};
    run3DResidualTest(wl, wr, 3, "3D LargeTurn");
}

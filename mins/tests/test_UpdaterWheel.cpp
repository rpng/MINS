// Unit tests for mins/src/update/wheel/UpdaterWheel: the OptionsWheel defaults it relies on,
// its measurement stack, and its analytical Jacobians.
#include <gtest/gtest.h>
#include <Eigen/Core>

#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsLidar.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "state/State.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "update/wheel/UpdaterWheel.h"
#include "update/wheel/WheelTypes.h"
#include "utils/quat_ops.h"

using namespace mins;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace {

/// Options for a wheel-only estimator, loaded with no yaml parser so every value is a default.
std::shared_ptr<OptionsEstimator> MakeWheelOnlyOptions(WheelType type) {
    std::shared_ptr<OptionsEstimator> op = std::make_shared<OptionsEstimator>();
    op->load();
    // The other sensors default to enabled but have no per-sensor defaults to be enabled
    // with, so State would index empty extrinsic maps while setting them up.
    op->cam->enabled = false;
    op->vicon->enabled = false;
    op->gps->enabled = false;
    op->lidar->enabled = false;
    op->wheel->enabled = true;
    op->wheel->type = type;
    return op;
}

void step2D(double dt, double wl, double wr, double rl, double rr, double b,
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

void step3D(double dt, double wl, double wr, double rl, double rr, double b,
                   Matrix3d &R_3D, Vector3d &p_3D) {
    Vector3d w(0, 0, (wr * rr - wl * rl) / b);
    Vector3d v((wr * rr + wl * rl) / 2, 0, 0);
    p_3D += R_3D.transpose() * v * dt;
    R_3D = ov_core::exp_so3(-w * dt) * R_3D;
}

void integrate2D(const double *wl, const double *wr, int N, double dt,
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

void integrate3D(const double *wl, const double *wr, int N, double dt,
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

void run2DResidualTest(const double *wl, const double *wr, int N, const char *label) {
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

void run3DResidualTest(const double *wl, const double *wr, int N, const char *label) {
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

/// Wheel reading whose two channels are time and 2*time, so interpolation is easy to predict.
WheelData MakeData(double time) {
    WheelData data;
    data.time = time;
    data.m1 = time;
    data.m2 = 2 * time;
    return data;
}

/// Updater holding a wheel-only state, fed one measurement per entry of times.
std::shared_ptr<UpdaterWheel> MakeUpdater(const std::vector<double> &times) {
    std::shared_ptr<State> state = std::make_shared<State>(MakeWheelOnlyOptions(WheelType::Wheel2DAng));
    std::shared_ptr<UpdaterWheel> updater = std::make_shared<UpdaterWheel>(state);
    for (double time : times) {
        updater->feed_measurement(MakeData(time));
    }
    return updater;
}

/// Timestamps of a selected window, so a test can compare against a plain vector.
std::vector<double> TimesOf(const std::vector<WheelData> &data_vec) {
    std::vector<double> times;
    for (const WheelData &data : data_vec) {
        times.push_back(data.time);
    }
    return times;
}

} // namespace

// ---- Options and state ----

TEST(OptionsWheelDefaults, ExtrinsicsAreIdentityWithoutAParser) {
    OptionsWheel options;
    options.load();
    ASSERT_EQ(options.extrinsics.size(), 7);
    Eigen::Matrix<double, 7, 1> identity;
    identity << 0, 0, 0, 1, 0, 0, 0;
    EXPECT_TRUE(options.extrinsics.isApprox(identity));
}

TEST(OptionsWheelDefaults, IntrinsicsAreUnitWheels) {
    OptionsWheel options;
    options.load();
    EXPECT_DOUBLE_EQ(options.intrinsics(0), 1.0);
    EXPECT_DOUBLE_EQ(options.intrinsics(1), 1.0);
    EXPECT_DOUBLE_EQ(options.intrinsics(2), 2.0);
}

TEST(WheelOnlyState, BuildsWithTheWheelCalibrationVariables) {
    // This is the regression test for the defaults above: with a zero-length extrinsics
    // vector, set_wheel_state feeds a 0x1 value to a 7x1 PoseJPL and dies.
    std::shared_ptr<State> state = std::make_shared<State>(MakeWheelOnlyOptions(WheelType::Wheel2DAng));
    ASSERT_NE(state->wheel_extrinsic, nullptr);
    ASSERT_NE(state->wheel_intrinsic, nullptr);
    ASSERT_NE(state->wheel_dt, nullptr);
    EXPECT_EQ(state->wheel_extrinsic->size(), 6);
    EXPECT_EQ(state->wheel_intrinsic->size(), 3);
    EXPECT_DOUBLE_EQ(state->wheel_dt->value()(0), 0.0);
}

TEST(WheelOnlyState, CovarianceCoversEveryCalibrationVariable) {
    std::shared_ptr<State> state = std::make_shared<State>(MakeWheelOnlyOptions(WheelType::Wheel2DAng));
    // IMU (15) + timeoffset (1) + extrinsic (6) + intrinsic (3), all three calibrated by default.
    EXPECT_EQ(state->cov.rows(), 25);
    EXPECT_EQ(state->cov.cols(), 25);
    EXPECT_TRUE(state->cov.isApprox(state->cov.transpose()));
    EXPECT_GT(state->cov.diagonal().minCoeff(), 0.0);
}

TEST(WheelOnlyState, IntrinsicCalibrationFlagSizesTheState) {
    std::shared_ptr<OptionsEstimator> op = MakeWheelOnlyOptions(WheelType::Wheel2DAng);
    EXPECT_TRUE(op->wheel->do_calib_int);
    op->wheel->do_calib_int = false;
    std::shared_ptr<State> state = std::make_shared<State>(op);
    EXPECT_EQ(state->cov.rows(), 22);
}

// ---- 2D Jacobians ----
// Numerically verifies UpdaterWheel's 2D analytical Jacobians using central finite differences.
// Perturbs each state dimension, builds H_numerical, and compares to UpdaterWheel::ComputeJacobians2D.
TEST(WheelJacobian2D, AnalyticalMatchesNumerical) {
    // Rotation cols: sign = +1 (residual changes directly with rotation error-state).
    // Position cols: sign = -1 (res = meas - h(state), so h increases means res decreases).
    const double epsilon = 1e-6;
    const double tolerance = 1e-6;

    Matrix3d R_GtoI0 = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.15));
    Vector3d p_I0inG(1.0, 0.5, 0.0);
    Matrix3d R_GtoI1 = ov_core::exp_so3(Vector3d(0.05, -0.15, 0.3));
    Vector3d p_I1inG(2.0, 1.0, 0.0);
    Matrix3d R_ItoO = ov_core::exp_so3(Vector3d(0.0, 0.0, 0.2));
    Vector3d p_IinO(0.1, 0.0, -0.2);

    // Zero-residual nominal measurement.
    Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
    Eigen::Matrix<double, 2, 3> Lambda = Eigen::Matrix<double, 2, 3>::Zero();
    Lambda.block(0, 0, 2, 2) = Eigen::Matrix2d::Identity();
    Vector3d e3(0, 0, 1);
    double th = (e3.transpose() * ov_core::log_so3(R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose()))(0);
    Eigen::Vector2d d_est = Lambda * R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG - R_GtoI0.transpose() * pOinI);
    double x = d_est(0);
    double y = d_est(1);

    const auto [H_poses, H_ext] = UpdaterWheel::ComputeJacobians2D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO);

    // --- H_poses: perturb pose0 (cols 0-5) and pose1 (cols 6-11) ---
    MatrixXd H_poses_numerical = MatrixXd::Zero(3, 12);
    for (int i = 0; i < 6; i++) {
        // pose0
        Matrix3d R0_plus = R_GtoI0, R0_minus = R_GtoI0;
        Vector3d p0_plus = p_I0inG, p0_minus = p_I0inG;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R0_plus = ov_core::exp_so3(epsilon * axis) * R_GtoI0;
            R0_minus = ov_core::exp_so3(-epsilon * axis) * R_GtoI0;
        } else {
            p0_plus(i - 3) += epsilon;
            p0_minus(i - 3) -= epsilon;
        }
        Vector3d res_plus = UpdaterWheel::ComputeResidual2D(R0_plus, p0_plus, R_GtoI1, p_I1inG, R_ItoO, p_IinO, th, x, y);
        Vector3d res_minus = UpdaterWheel::ComputeResidual2D(R0_minus, p0_minus, R_GtoI1, p_I1inG, R_ItoO, p_IinO, th, x, y);
        double sign = (i < 3) ? 1.0 : -1.0;
        H_poses_numerical.col(i) = sign * (res_plus - res_minus) / (2.0 * epsilon);

        // pose1
        Matrix3d R1_plus = R_GtoI1, R1_minus = R_GtoI1;
        Vector3d p1_plus = p_I1inG, p1_minus = p_I1inG;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R1_plus = ov_core::exp_so3(epsilon * axis) * R_GtoI1;
            R1_minus = ov_core::exp_so3(-epsilon * axis) * R_GtoI1;
        } else {
            p1_plus(i - 3) += epsilon;
            p1_minus(i - 3) -= epsilon;
        }
        res_plus = UpdaterWheel::ComputeResidual2D(R_GtoI0, p_I0inG, R1_plus, p1_plus, R_ItoO, p_IinO, th, x, y);
        res_minus = UpdaterWheel::ComputeResidual2D(R_GtoI0, p_I0inG, R1_minus, p1_minus, R_ItoO, p_IinO, th, x, y);
        H_poses_numerical.col(6 + i) = sign * (res_plus - res_minus) / (2.0 * epsilon);
    }

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 12; col++) {
            EXPECT_NEAR(H_poses(row, col), H_poses_numerical(row, col), tolerance)
                << "H_poses mismatch at row=" << row << " col=" << col;
            if (std::abs(H_poses_numerical(row, col)) > tolerance) {
                EXPECT_GT(H_poses(row, col) * H_poses_numerical(row, col), 0.0)
                    << "H_poses sign mismatch at row=" << row << " col=" << col;
            }
        }
    }

    // --- H_ext: perturb extrinsic (R_ItoO, p_IinO) ---
    MatrixXd H_ext_numerical = MatrixXd::Zero(3, 6);
    for (int i = 0; i < 6; i++) {
        Matrix3d R_plus = R_ItoO, R_minus = R_ItoO;
        Vector3d p_plus = p_IinO, p_minus = p_IinO;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R_plus = ov_core::exp_so3(epsilon * axis) * R_ItoO;
            R_minus = ov_core::exp_so3(-epsilon * axis) * R_ItoO;
        } else {
            p_plus(i - 3) += epsilon;
            p_minus(i - 3) -= epsilon;
        }
        Vector3d res_plus = UpdaterWheel::ComputeResidual2D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_plus, p_plus, th, x, y);
        Vector3d res_minus = UpdaterWheel::ComputeResidual2D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_minus, p_minus, th, x, y);
        double sign = (i < 3) ? 1.0 : -1.0;
        H_ext_numerical.col(i) = sign * (res_plus - res_minus) / (2.0 * epsilon);
    }

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 6; col++) {
            EXPECT_NEAR(H_ext(row, col), H_ext_numerical(row, col), tolerance)
                << "H_ext mismatch at row=" << row << " col=" << col;
            if (std::abs(H_ext_numerical(row, col)) > tolerance) {
                EXPECT_GT(H_ext(row, col) * H_ext_numerical(row, col), 0.0)
                    << "H_ext sign mismatch at row=" << row << " col=" << col;
            }
        }
    }
}

// ---- 3D Jacobians ----
// Numerically verifies UpdaterWheel's 3D analytical Jacobians using central finite differences.
TEST(WheelJacobian3D, AnalyticalMatchesNumerical) {
    const double epsilon = 1e-6;
    const double tolerance = 1e-6;

    Matrix3d R_GtoI0 = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.15));
    Vector3d p_I0inG(1.0, 0.5, 0.0);
    Matrix3d R_GtoI1 = ov_core::exp_so3(Vector3d(0.05, -0.15, 0.3));
    Vector3d p_I1inG(2.0, 1.0, 0.0);
    Matrix3d R_ItoO = ov_core::exp_so3(Vector3d(0.0, 0.0, 0.2));
    Vector3d p_IinO(0.1, 0.0, -0.2);

    // Zero-residual nominal measurement.
    Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
    Matrix3d R_3D = R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose();
    Vector3d p_3D = R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG - R_GtoI0.transpose() * pOinI);

    const auto [H_poses, H_ext] = UpdaterWheel::ComputeJacobians3D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO);

    // --- H_poses: perturb pose0 (cols 0-5) and pose1 (cols 6-11) ---
    MatrixXd H_poses_numerical = MatrixXd::Zero(6, 12);
    for (int i = 0; i < 6; i++) {
        // pose0
        Matrix3d R0_plus = R_GtoI0, R0_minus = R_GtoI0;
        Vector3d p0_plus = p_I0inG, p0_minus = p_I0inG;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R0_plus = ov_core::exp_so3(epsilon * axis) * R_GtoI0;
            R0_minus = ov_core::exp_so3(-epsilon * axis) * R_GtoI0;
        } else {
            p0_plus(i - 3) += epsilon;
            p0_minus(i - 3) -= epsilon;
        }
        Eigen::Matrix<double, 6, 1> res_plus = UpdaterWheel::ComputeResidual3D(R0_plus, p0_plus, R_GtoI1, p_I1inG, R_ItoO, p_IinO, R_3D, p_3D);
        Eigen::Matrix<double, 6, 1> res_minus = UpdaterWheel::ComputeResidual3D(R0_minus, p0_minus, R_GtoI1, p_I1inG, R_ItoO, p_IinO, R_3D, p_3D);
        double sign = (i < 3) ? 1.0 : -1.0;
        H_poses_numerical.col(i) = sign * (res_plus - res_minus) / (2.0 * epsilon);

        // pose1
        Matrix3d R1_plus = R_GtoI1, R1_minus = R_GtoI1;
        Vector3d p1_plus = p_I1inG, p1_minus = p_I1inG;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R1_plus = ov_core::exp_so3(epsilon * axis) * R_GtoI1;
            R1_minus = ov_core::exp_so3(-epsilon * axis) * R_GtoI1;
        } else {
            p1_plus(i - 3) += epsilon;
            p1_minus(i - 3) -= epsilon;
        }
        res_plus = UpdaterWheel::ComputeResidual3D(R_GtoI0, p_I0inG, R1_plus, p1_plus, R_ItoO, p_IinO, R_3D, p_3D);
        res_minus = UpdaterWheel::ComputeResidual3D(R_GtoI0, p_I0inG, R1_minus, p1_minus, R_ItoO, p_IinO, R_3D, p_3D);
        H_poses_numerical.col(6 + i) = sign * (res_plus - res_minus) / (2.0 * epsilon);
    }

    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 12; col++) {
            EXPECT_NEAR(H_poses(row, col), H_poses_numerical(row, col), tolerance)
                << "H_poses mismatch at row=" << row << " col=" << col;
            if (std::abs(H_poses_numerical(row, col)) > tolerance) {
                EXPECT_GT(H_poses(row, col) * H_poses_numerical(row, col), 0.0)
                    << "H_poses sign mismatch at row=" << row << " col=" << col;
            }
        }
    }

    // --- H_ext: perturb extrinsic (R_ItoO, p_IinO) ---
    MatrixXd H_ext_numerical = MatrixXd::Zero(6, 6);
    for (int i = 0; i < 6; i++) {
        Matrix3d R_plus = R_ItoO, R_minus = R_ItoO;
        Vector3d p_plus = p_IinO, p_minus = p_IinO;
        if (i < 3) {
            Vector3d axis = Vector3d::Zero();
            axis(i) = 1.0;
            R_plus = ov_core::exp_so3(epsilon * axis) * R_ItoO;
            R_minus = ov_core::exp_so3(-epsilon * axis) * R_ItoO;
        } else {
            p_plus(i - 3) += epsilon;
            p_minus(i - 3) -= epsilon;
        }
        Eigen::Matrix<double, 6, 1> res_plus = UpdaterWheel::ComputeResidual3D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_plus, p_plus, R_3D, p_3D);
        Eigen::Matrix<double, 6, 1> res_minus = UpdaterWheel::ComputeResidual3D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_minus, p_minus, R_3D, p_3D);
        double sign = (i < 3) ? 1.0 : -1.0;
        H_ext_numerical.col(i) = sign * (res_plus - res_minus) / (2.0 * epsilon);
    }

    for (int row = 0; row < 6; row++) {
        for (int col = 0; col < 6; col++) {
            EXPECT_NEAR(H_ext(row, col), H_ext_numerical(row, col), tolerance)
                << "H_ext mismatch at row=" << row << " col=" << col;
            if (std::abs(H_ext_numerical(row, col)) > tolerance) {
                EXPECT_GT(H_ext(row, col) * H_ext_numerical(row, col), 0.0)
                    << "H_ext sign mismatch at row=" << row << " col=" << col;
            }
        }
    }
}

// ---- Time offset Jacobians ----
// Numerically verifies the time-offset Jacobian (ComputeTimeOffsetJacobian2D/3D) via
// a combined central finite difference: both clone poses are shifted simultaneously
// by (exp_so3(w*eps)*R, p - v*eps), which models increasing dt by eps (backward shift).
TEST(WheelTimeOffset2D, AnalyticalMatchesNumerical) {
    const double eps = 1e-6;
    const double tol = 1e-6;

    Matrix3d R_GtoI0 = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.15));
    Vector3d p_I0inG(1.0, 0.5, 0.0);
    Matrix3d R_GtoI1 = ov_core::exp_so3(Vector3d(0.05, -0.15, 0.3));
    Vector3d p_I1inG(2.0, 1.0, 0.0);
    Matrix3d R_ItoO = ov_core::exp_so3(Vector3d(0.0, 0.0, 0.2));
    Vector3d p_IinO(0.1, 0.0, -0.2);

    // Arbitrary velocities: w in body frame, v in global frame.
    Vector3d w0(0.01, 0.02, 0.1), v0(0.5, 0.1, 0.0);
    Vector3d w1(0.02, -0.01, 0.05), v1(0.6, -0.1, 0.0);

    // Zero-residual nominal 2D measurement.
    Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
    Vector3d e3(0, 0, 1);
    Eigen::Matrix<double, 2, 3> Lambda = Eigen::Matrix<double, 2, 3>::Zero();
    Lambda.block(0, 0, 2, 2) = Eigen::Matrix2d::Identity();
    Matrix3d RO0toO1 = R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose();
    double th = e3.dot(ov_core::log_so3(RO0toO1));
    Eigen::Vector2d d_est = Lambda * R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG - R_GtoI0.transpose() * pOinI);
    double x = d_est(0), y = d_est(1);

    const auto [H_poses, H_ext] = UpdaterWheel::ComputeJacobians2D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO);
    Vector3d H_dt = UpdaterWheel::ComputeTimeOffsetJacobian2D(H_poses, w0, v0, w1, v1);

    // Combined FD: shift both poses backward in time by eps (models dt += eps).
    // R(t-eps) ~ exp_so3(w*eps)*R,  p(t-eps) ~ p - v*eps.
    Matrix3d R0p = ov_core::exp_so3(w0 * eps) * R_GtoI0;
    Vector3d p0p = p_I0inG - v0 * eps;
    Matrix3d R1p = ov_core::exp_so3(w1 * eps) * R_GtoI1;
    Vector3d p1p = p_I1inG - v1 * eps;

    Matrix3d R0m = ov_core::exp_so3(-w0 * eps) * R_GtoI0;
    Vector3d p0m = p_I0inG + v0 * eps;
    Matrix3d R1m = ov_core::exp_so3(-w1 * eps) * R_GtoI1;
    Vector3d p1m = p_I1inG + v1 * eps;

    Vector3d res_plus = UpdaterWheel::ComputeResidual2D(R0p, p0p, R1p, p1p, R_ItoO, p_IinO, th, x, y);
    Vector3d res_minus = UpdaterWheel::ComputeResidual2D(R0m, p0m, R1m, p1m, R_ItoO, p_IinO, th, x, y);
    Vector3d H_dt_numerical = (res_plus - res_minus) / (2.0 * eps);

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(H_dt(i), H_dt_numerical(i), tol)
            << "2D H_dt mismatch at row=" << i;
        if (std::abs(H_dt_numerical(i)) > tol) {
            EXPECT_GT(H_dt(i) * H_dt_numerical(i), 0.0)
                << "2D H_dt sign mismatch at row=" << i;
        }
    }
}

TEST(WheelTimeOffset3D, AnalyticalMatchesNumerical) {
    const double eps = 1e-6;
    const double tol = 1e-6;

    Matrix3d R_GtoI0 = ov_core::exp_so3(Vector3d(0.1, -0.2, 0.15));
    Vector3d p_I0inG(1.0, 0.5, 0.0);
    Matrix3d R_GtoI1 = ov_core::exp_so3(Vector3d(0.05, -0.15, 0.3));
    Vector3d p_I1inG(2.0, 1.0, 0.0);
    Matrix3d R_ItoO = ov_core::exp_so3(Vector3d(0.0, 0.0, 0.2));
    Vector3d p_IinO(0.1, 0.0, -0.2);

    Vector3d w0(0.01, 0.02, 0.1), v0(0.5, 0.1, 0.0);
    Vector3d w1(0.02, -0.01, 0.05), v1(0.6, -0.1, 0.0);

    // Zero-residual nominal 3D measurement.
    Vector3d pOinI = -R_ItoO.transpose() * p_IinO;
    Matrix3d R_3D = R_ItoO * R_GtoI1 * R_GtoI0.transpose() * R_ItoO.transpose();
    Vector3d p_3D = R_ItoO * R_GtoI0 * (p_I1inG + R_GtoI1.transpose() * pOinI - p_I0inG - R_GtoI0.transpose() * pOinI);

    const auto [H_poses, H_ext] = UpdaterWheel::ComputeJacobians3D(R_GtoI0, p_I0inG, R_GtoI1, p_I1inG, R_ItoO, p_IinO);
    Eigen::Matrix<double, 6, 1> H_dt = UpdaterWheel::ComputeTimeOffsetJacobian3D(H_poses, w0, v0, w1, v1);

    Matrix3d R0p = ov_core::exp_so3(w0 * eps) * R_GtoI0;
    Vector3d p0p = p_I0inG - v0 * eps;
    Matrix3d R1p = ov_core::exp_so3(w1 * eps) * R_GtoI1;
    Vector3d p1p = p_I1inG - v1 * eps;

    Matrix3d R0m = ov_core::exp_so3(-w0 * eps) * R_GtoI0;
    Vector3d p0m = p_I0inG + v0 * eps;
    Matrix3d R1m = ov_core::exp_so3(-w1 * eps) * R_GtoI1;
    Vector3d p1m = p_I1inG + v1 * eps;

    Eigen::Matrix<double, 6, 1> res_plus = UpdaterWheel::ComputeResidual3D(R0p, p0p, R1p, p1p, R_ItoO, p_IinO, R_3D, p_3D);
    Eigen::Matrix<double, 6, 1> res_minus = UpdaterWheel::ComputeResidual3D(R0m, p0m, R1m, p1m, R_ItoO, p_IinO, R_3D, p_3D);
    Eigen::Matrix<double, 6, 1> H_dt_numerical = (res_plus - res_minus) / (2.0 * eps);

    for (int i = 0; i < 6; i++) {
        EXPECT_NEAR(H_dt(i), H_dt_numerical(i), tol)
            << "3D H_dt mismatch at row=" << i;
        if (std::abs(H_dt_numerical(i)) > tol) {
            EXPECT_GT(H_dt(i) * H_dt_numerical(i), 0.0)
                << "3D H_dt sign mismatch at row=" << i;
        }
    }
}

// ---- Intrinsic Jacobians ----
// Tests wheel intrinsic Jacobians at the residual level for Wheel2DAng and Wheel3DAng.
// FDs ComputeResidual2D/3D w.r.t. each intrinsic (rl, rr, b), comparing to the analytical H:
//   2D: H_int = -[dth_di; dx_di; dy_di]
//   3D: H_int = -[dR_di; dp_di]
// Sign=-1 matches Vec error-state convention (same as position columns in test_UpdaterWheel2D).
// Covers turning, straight (L'Hopital branch), and varying-speed motion.

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

// Verifies ComputePhiTr3D: the 6x6 covariance transition matrix for 3D preintegration.
// State layout: [delta_R (3), delta_p (3)]. JPL convention: R_true = exp(-delta_phi) * R_hat.
//
// block(0,0): d(delta_R_new)/d(delta_R) = R_new * R_3D^T
// block(3,0): d(delta_p_new)/d(delta_R) = -R_3D^T * skew(R_3D * (new_p - p_3D))
// block(3,3): identity
TEST(WheelCovPropagation3D, PhiTrMatchesFD) {
    const double eps = 1e-6;
    const double tol = 1e-5;

    Matrix3d R_3D = ov_core::exp_so3(Vector3d(0.1, 0.2, 0.3));
    Vector3d v(1.0, 0.0, 0.0);
    double dt = 0.01;
    Vector3d p_3D = Vector3d::Zero();
    Vector3d new_p = p_3D + R_3D.transpose() * v * dt;
    Matrix3d R_new = ov_core::exp_so3(Vector3d(0, 0, -0.05)) * R_3D;

    Eigen::Matrix<double, 6, 6> Phi = UpdaterWheel::ComputePhiTr3D(R_3D, R_new, p_3D, new_p);

    // block(3,0): d(new_p)/d(delta_phi) — perturb R_3D via JPL: R_true = exp(-eps*ej)*R_3D
    for (int j = 0; j < 3; j++) {
        Vector3d ej = Vector3d::Zero();
        ej(j) = eps;
        Vector3d np_p = p_3D + (ov_core::exp_so3(-ej) * R_3D).transpose() * v * dt;
        Vector3d np_m = p_3D + (ov_core::exp_so3(ej) * R_3D).transpose() * v * dt;
        Vector3d fd = (np_p - np_m) / (2.0 * eps);
        for (int i = 0; i < 3; i++) {
            EXPECT_NEAR(Phi(3 + i, j), fd(i), tol) << "Phi_tr[" << (3 + i) << "," << j << "]";
        }
    }

    // block(3,3): identity
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_NEAR(Phi(3 + i, 3 + j), (i == j) ? 1.0 : 0.0, tol) << "Phi_tr[" << (3 + i) << "," << (3 + j) << "]";
        }
    }
}

// ---- Measurement stack ----

TEST(FeedMeasurement, KeepsEveryMeasurementInFeedOrder) {
    std::shared_ptr<UpdaterWheel> updater = MakeUpdater({0.0, 1.0, 2.0});
    EXPECT_EQ(updater->num_measurements(), 3u);
    EXPECT_EQ(updater->t_hist.size(), 3u);
    EXPECT_DOUBLE_EQ(updater->t_hist.front(), 0.0);
    EXPECT_DOUBLE_EQ(updater->t_hist.back(), 2.0);
}

TEST(FeedMeasurement, DropsMeasurementsOlderThanTheHistoryWindow) {
    // The window is 100 seconds, and the comparison is strict, so the 50 s reading survives.
    std::shared_ptr<UpdaterWheel> updater = MakeUpdater({0.0, 50.0, 150.0});
    EXPECT_EQ(updater->num_measurements(), 2u);
    double min_time, max_time;
    EXPECT_FALSE(updater->measurement_time_span(min_time, max_time));
}

TEST(FeedMeasurement, CapsTheTimeHistory) {
    std::vector<double> times;
    for (int i = 0; i < 105; i++) {
        times.push_back(0.1 * i);
    }
    std::shared_ptr<UpdaterWheel> updater = MakeUpdater(times);
    EXPECT_EQ(updater->t_hist.size(), 101u);
    // The four oldest timestamps were popped off the front.
    EXPECT_DOUBLE_EQ(updater->t_hist.front(), 0.4);
    EXPECT_DOUBLE_EQ(updater->t_hist.back(), 10.4);
}

TEST(MeasurementTimeSpan, NeedsThreeMeasurements) {
    double min_time = -1, max_time = -1;
    EXPECT_FALSE(MakeUpdater({})->measurement_time_span(min_time, max_time));
    EXPECT_FALSE(MakeUpdater({0.0, 1.0})->measurement_time_span(min_time, max_time));
    // Untouched by the failing calls above.
    EXPECT_DOUBLE_EQ(min_time, -1);
    EXPECT_DOUBLE_EQ(max_time, -1);
}

TEST(MeasurementTimeSpan, ExcludesTheFirstAndLastMeasurement) {
    double min_time, max_time;
    ASSERT_TRUE(MakeUpdater({0.0, 1.0, 2.0, 3.0})->measurement_time_span(min_time, max_time));
    EXPECT_DOUBLE_EQ(min_time, 1.0);
    EXPECT_DOUBLE_EQ(max_time, 2.0);
}

TEST(CleanupMeasurements, DropsTheOlderOnesAndCountsThem) {
    std::shared_ptr<UpdaterWheel> updater = MakeUpdater({0.0, 1.0, 2.0, 3.0});
    // Strictly older, so the measurement sitting exactly on the cut stays.
    EXPECT_EQ(updater->cleanup_measurements(2.0), 2);
    EXPECT_EQ(updater->num_measurements(), 2u);
    EXPECT_EQ(updater->cleanup_measurements(-1.0), 0);
    EXPECT_EQ(updater->num_measurements(), 2u);
    EXPECT_EQ(updater->cleanup_measurements(100.0), 2);
    EXPECT_EQ(updater->num_measurements(), 0u);
}

TEST(SelectWheelData, FailsWithAnEmptyStack) {
    std::vector<WheelData> data_vec;
    EXPECT_FALSE(MakeUpdater({})->select_wheel_data(0.0, 1.0, data_vec));
    EXPECT_TRUE(data_vec.empty());
}

TEST(SelectWheelData, FailsWhenTheWindowRunsPastTheStack) {
    std::shared_ptr<UpdaterWheel> updater = MakeUpdater({0.0, 1.0, 2.0, 3.0, 4.0});
    std::vector<WheelData> data_vec;
    // The newest measurement has to be strictly newer than time1.
    EXPECT_FALSE(updater->select_wheel_data(0.5, 4.0, data_vec));
    // And the oldest one no newer than time0.
    EXPECT_FALSE(updater->select_wheel_data(-0.5, 2.0, data_vec));
}

TEST(SelectWheelData, InterpolatesBothEndpoints) {
    std::shared_ptr<UpdaterWheel> updater = MakeUpdater({0.0, 1.0, 2.0, 3.0, 4.0});
    std::vector<WheelData> data_vec;
    ASSERT_TRUE(updater->select_wheel_data(0.5, 3.5, data_vec));
    EXPECT_EQ(TimesOf(data_vec), std::vector<double>({0.5, 1.0, 2.0, 3.0, 3.5}));
    // Both readings are linear in time, so interpolation reproduces them exactly.
    EXPECT_DOUBLE_EQ(data_vec.front().m1, 0.5);
    EXPECT_DOUBLE_EQ(data_vec.front().m2, 1.0);
    EXPECT_DOUBLE_EQ(data_vec.back().m1, 3.5);
    EXPECT_DOUBLE_EQ(data_vec.back().m2, 7.0);
}

TEST(SelectWheelData, TakesWholeMeasurementsOnAnExactHit) {
    std::shared_ptr<UpdaterWheel> updater = MakeUpdater({0.0, 1.0, 2.0, 3.0, 4.0});
    std::vector<WheelData> data_vec;
    ASSERT_TRUE(updater->select_wheel_data(1.0, 3.0, data_vec));
    EXPECT_EQ(TimesOf(data_vec), std::vector<double>({1.0, 2.0, 3.0}));
}

TEST(SelectWheelData, DropsDuplicateTimestamps) {
    // A duplicated reading would give a zero dt, and a zero dt divides into the noise covariance.
    std::shared_ptr<UpdaterWheel> updater = MakeUpdater({0.0, 1.0, 1.0, 2.0, 3.0});
    std::vector<WheelData> data_vec;
    ASSERT_TRUE(updater->select_wheel_data(0.5, 2.5, data_vec));
    EXPECT_EQ(TimesOf(data_vec), std::vector<double>({0.5, 1.0, 2.0, 2.5}));
}

TEST(SelectWheelData, CutsBothEndsOutOfOneLongMeasurement) {
    // Both bounds fall inside a single measurement interval, so the window is two splits of it.
    std::shared_ptr<UpdaterWheel> updater = MakeUpdater({0.0, 10.0, 20.0});
    std::vector<WheelData> data_vec;
    ASSERT_TRUE(updater->select_wheel_data(1.0, 2.0, data_vec));
    EXPECT_EQ(TimesOf(data_vec), std::vector<double>({1.0, 2.0}));
    EXPECT_DOUBLE_EQ(data_vec.back().m1, 2.0);
}

TEST(SelectWheelData, FailsWhenOnlyOneSampleLandsInTheWindow) {
    // Nothing bounds time1 from above, so the window never gets its second sample.
    std::shared_ptr<UpdaterWheel> updater = MakeUpdater({0.0, 10.0});
    std::vector<WheelData> data_vec;
    EXPECT_FALSE(updater->select_wheel_data(1.0, 2.0, data_vec));
}

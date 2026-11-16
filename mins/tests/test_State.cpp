/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Unit tests for the clone interpolation in mins/src/state/State.
#include <gtest/gtest.h>
#include <Eigen/Core>
#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsLidar.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/quat_ops.h"

using namespace mins;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace {

/// IMU-only options with polynomial interpolation and a clone window a bit over 0.2 s. The interpolation
/// error tables are only parsed, so they come from the simulation config.
std::shared_ptr<OptionsEstimator> MakeOptions() {
    std::shared_ptr<OptionsEstimator> op = std::make_shared<OptionsEstimator>();
    op->load(std::make_shared<ov_core::YamlParser>(MINS_CONFIG_DIR "/simulation/config.yaml"));
    op->cam->enabled = op->gps->enabled = op->wheel->enabled = op->vicon->enabled = op->lidar->enabled = false;
    op->use_imu_res = false;
    op->window_size = 0.21;
    op->lidar->do_calib_dt = false;
    return op;
}

/// Clones along a trajectory that moves and turns, so both halves of the interpolation matter.
void AddClone(const std::shared_ptr<State> &state, double time) {
    Eigen::Matrix<double, 16, 1> imu_value = Eigen::Matrix<double, 16, 1>::Zero();
    imu_value.block(0, 0, 4, 1) = ov_core::rot_2_quat(ov_core::exp_so3(Vector3d(0.1 * time, -0.2 * time, 0.5 * time)));
    imu_value.block(4, 0, 3, 1) = Vector3d(2.0 * time, time * time, 0.1 * time);
    state->imu->set_value(imu_value);
    state->imu->set_fej(imu_value);
    state->time = time;
    StateHelper::augment_clone(state);
}

}  // namespace

TEST(StateInterpolation, AKeyframeIsInterpolatedWithTheClonesStillAroundIt) {
    // A keyframe (as GPS keeps before it initializes) outlives the clones its polynomial was fit
    // on. Interpolating next to it has to use the clones that are still there.
    std::shared_ptr<State> kept = std::make_shared<State>(MakeOptions());
    kept->keyframes_candidate.push_back(0.0);
    for (int i = 0; i <= 6; i++) {
        AddClone(kept, 0.05 * i);
        StateHelper::marginalize_old_clone(kept);
    }
    // As in the filter, a new clone comes before the next interpolation. It also drops the
    // polynomials of the clones that just left.
    AddClone(kept, 0.35);
    std::vector<double> times;
    for (const auto &clone : kept->clones) {
        times.push_back(clone.first);
    }
    ASSERT_EQ(times.size(), 7u);
    ASSERT_DOUBLE_EQ(times[0], 0.0);
    ASSERT_NEAR(times[1], 0.1, 1e-9) << "the clones right after the keyframe should be gone";

    // The same clones, inserted directly.
    std::shared_ptr<State> fresh = std::make_shared<State>(MakeOptions());
    for (double time : times) {
        AddClone(fresh, time);
    }

    // Between the first two clones after the keyframe, so the keyframe is one of the nodes.
    const double t = 0.12;
    Matrix3d R_kept, R_fresh;
    Vector3d p_kept, p_fresh;
    std::vector<MatrixXd> dTdx_kept, dTdx_fresh;
    std::vector<std::shared_ptr<ov_type::Type>> order_kept, order_fresh;
    ASSERT_TRUE(kept->get_interpolated_jacobian(t, R_kept, p_kept, "LIDAR", 0, dTdx_kept, order_kept));
    ASSERT_TRUE(fresh->get_interpolated_jacobian(t, R_fresh, p_fresh, "LIDAR", 0, dTdx_fresh, order_fresh));
    EXPECT_TRUE(R_kept.isApprox(R_fresh, 1e-9));
    EXPECT_TRUE(p_kept.isApprox(p_fresh, 1e-9));
    ASSERT_EQ(dTdx_kept.size(), dTdx_fresh.size());
    for (size_t i = 0; i < dTdx_kept.size(); i++) {
        EXPECT_TRUE(dTdx_kept[i].isApprox(dTdx_fresh[i], 1e-9)) << "node " << i << "\n" << dTdx_kept[i] << "\nvs\n" << dTdx_fresh[i];
    }
}

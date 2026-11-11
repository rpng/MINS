/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Unit tests for mins/src/update/lidar: the scan container, plane fitting and neighbor search in
// LidarHelper, the scan to map transform, and UpdaterLidar end to end on a synthetic room.
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <cmath>
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
#include "types/Vec.h"
#include "update/lidar/LidarHelper.h"
#include "update/lidar/LidarTypes.h"
#include "update/lidar/PointCloud.h"
#include "update/lidar/UpdaterLidar.h"
#include "update/lidar/ikd_Tree.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/quat_ops.h"

using namespace mins;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {

/// Clone times of the test trajectory. Five clones give the cubic interpolation two polynomials.
const std::vector<double> CLONE_TIMES = {0.0, 0.1, 0.2, 0.3, 0.4};

/// The IMU position along the test trajectory. It is quadratic, so the cubic clone
/// interpolation reproduces it exactly between clones. The IMU never rotates.
Vector3d TrajectoryPosition(double time) { return Vector3d(2.0 * time, time * time, 0.1 * time); }

/// Lidar-only options with one sensor, from the simulation config (its extrinsic is rotated and
/// offset), since the interpolation error table the update reads is only parsed.
std::shared_ptr<OptionsEstimator> MakeLidarOptions() {
    std::shared_ptr<OptionsEstimator> op = std::make_shared<OptionsEstimator>();
    op->load(std::make_shared<ov_core::YamlParser>(MINS_CONFIG_DIR "/simulation/config.yaml"));
    op->cam->enabled = op->gps->enabled = op->wheel->enabled = op->vicon->enabled = false;
    op->lidar->enabled = true;
    // Interpolate between clones with the polynomial; the IMU residual needs a propagated IMU.
    op->use_imu_res = false;
    op->lidar->max_n = 1;
    return op;
}

/// Lidar-only state holding a clone at each of CLONE_TIMES along the test trajectory.
std::shared_ptr<State> MakeCloneState(bool do_calib_ext = false) {
    std::shared_ptr<OptionsEstimator> op = MakeLidarOptions();
    op->lidar->do_calib_ext = do_calib_ext;
    std::shared_ptr<State> state = std::make_shared<State>(op);
    for (double time : CLONE_TIMES) {
        // Stands in for propagation. Without it every clone is the same random variable, and a
        // relative measurement such as a scan against the map carries no information.
        const int imu = state->imu->id();
        state->cov.block(imu, imu, 3, 3).diagonal().array() += 1e-4;
        state->cov.block(imu + 3, imu + 3, 3, 3).diagonal().array() += 1e-2;
        Eigen::Matrix<double, 16, 1> imu_value = Eigen::Matrix<double, 16, 1>::Zero();
        imu_value(3) = 1;  // JPL identity quaternion
        imu_value.block(4, 0, 3, 1) = TrajectoryPosition(time);
        state->imu->set_value(imu_value);
        state->imu->set_fej(imu_value);
        state->time = time;
        StateHelper::augment_clone(state);
    }
    return state;
}

/// A room in the world frame: a floor and three walls, so every translation axis is observable.
std::vector<Vector3d> RoomInWorld() {
    const double step = 0.25;
    std::vector<Vector3d> points;
    for (double a = -3.0; a <= 6.0; a += step) {
        for (double b = -4.0; b <= 4.0; b += step) {
            points.emplace_back(a, b, -1.5);  // floor
        }
        for (double c = -1.5 + step; c <= 3.0; c += step) {
            points.emplace_back(a, 4.0, c);   // left wall
            points.emplace_back(a, -4.0, c);  // right wall
        }
    }
    for (double b = -4.0 + step; b < 4.0; b += step) {
        for (double c = -1.5 + step; c <= 3.0; c += step) {
            points.emplace_back(6.0, b, c);  // front wall
        }
    }
    return points;
}

/// The room (or other world points) as the lidar sees it with the IMU at the given world position, unrotated.
std::shared_ptr<PointCloud<PointXYZ>> ScanFrom(const std::shared_ptr<State> &state, const Vector3d &p_IinG, double time,
                                               const std::vector<Vector3d> &points_inG = RoomInWorld()) {
    const Matrix3d R_ItoL = state->lidar_extrinsic.at(0)->Rot();
    const Vector3d p_IinL = state->lidar_extrinsic.at(0)->pos();
    const Vector3d p_LinG = p_IinG - R_ItoL.transpose() * p_IinL;
    std::shared_ptr<PointCloud<PointXYZ>> cloud = std::make_shared<PointCloud<PointXYZ>>();
    for (const Vector3d &p_inG : points_inG) {
        const Vector3d p_inL = R_ItoL * (p_inG - p_LinG);
        PointXYZ point;
        point.x = p_inL(0);
        point.y = p_inL(1);
        point.z = p_inL(2);
        cloud->push_back(point);
    }
    cloud->header.stamp = (std::uint64_t)std::llround(time * 1000.0);
    cloud->header.frame_id = "0";
    return cloud;
}

std::shared_ptr<PointCloud<PointXYZ>> ScanAt(const std::shared_ptr<State> &state, double time) {
    return ScanFrom(state, TrajectoryPosition(time), time);
}

POINTCLOUD_XYZI_PTR CloudOf(const std::vector<Vector3d> &points) {
    POINTCLOUD_XYZI_PTR cloud = std::make_shared<PointCloud<PointXYZI>>();
    for (const Vector3d &p : points) {
        PointXYZI point;
        point.x = p(0);
        point.y = p(1);
        point.z = p(2);
        cloud->push_back(point);
    }
    return cloud;
}

}  // namespace

// ---- Scan container ----

TEST(LidarData, DropsPointsOutOfRangeAndStampsTheRestRelativeToTheReference) {
    std::shared_ptr<PointCloud<PointXYZ>> cloud = std::make_shared<PointCloud<PointXYZ>>();
    for (float x : {0.5f, 2.0f, 50.0f, 150.0f}) {
        PointXYZ point;
        point.x = x;
        cloud->push_back(point);
    }
    LiDARData data(10.25, 10.0, 0, cloud, 100.0, 1.0);
    ASSERT_EQ(data.size(), 2);
    EXPECT_EQ(data.p(0), Vector3d(2.0, 0.0, 0.0));
    EXPECT_EQ(data.p(1), Vector3d(50.0, 0.0, 0.0));
    EXPECT_FLOAT_EQ(data.pointcloud->points[0].intensity, 0.25f);
    EXPECT_EQ(data.pointcloud_original->size(), 2u);
}

// ---- Plane fitting ----

TEST(LidarPlane, FitsAPlaneThroughCoplanarPoints) {
    std::shared_ptr<OptionsLidar> op = MakeLidarOptions()->lidar;
    // The plane x + 2y - z = 3, sampled off any axis.
    std::vector<Vector3d> points;
    for (double u : {0.0, 0.3, 0.7}) {
        for (double v : {0.0, 0.4}) {
            points.emplace_back(u, v, u + 2.0 * v - 3.0);
        }
    }
    Vector4d plane;
    ASSERT_TRUE(LidarHelper::compute_plane(plane, CloudOf(points), op));
    const Vector4d normalized = plane / plane.head(3).norm();
    const Vector3d normal = Vector3d(1.0, 2.0, -1.0).normalized();
    EXPECT_NEAR(std::abs(normalized.head(3).dot(normal)), 1.0, 1e-9);
    // Map points are stored as float, so the fit is only good to float precision.
    for (const Vector3d &p : points) {
        EXPECT_NEAR(normalized.head(3).dot(p) + normalized(3), 0.0, 1e-6);
    }
}

TEST(LidarPlane, RejectsPointsOnALine) {
    std::shared_ptr<OptionsLidar> op = MakeLidarOptions()->lidar;
    std::vector<Vector3d> points;
    for (double u : {0.0, 0.1, 0.2, 0.3, 0.4}) {
        points.emplace_back(1.0 + u, 2.0 + 2.0 * u, 3.0 - u);
    }
    Vector4d plane;
    EXPECT_FALSE(LidarHelper::compute_plane(plane, CloudOf(points), op));
}

TEST(LidarPlane, RejectsAPointFurtherFromThePlaneThanTheThreshold) {
    std::shared_ptr<OptionsLidar> op = MakeLidarOptions()->lidar;
    std::vector<Vector3d> points = {{2, 0, 0}, {2, 0.5, 0}, {2, 0, 0.5}, {2, 0.5, 0.5}, {2, 0.25, 0.25}};
    Vector4d plane;
    ASSERT_TRUE(LidarHelper::compute_plane(plane, CloudOf(points), op));
    points.back().x() += 5.0 * op->plane_max_p2pd;
    EXPECT_FALSE(LidarHelper::compute_plane(plane, CloudOf(points), op));
}

// ---- Neighbor search ----

TEST(LidarNeighbors, FindsTheNearestMapPointsAndRefusesAFarQuery) {
    std::shared_ptr<OptionsLidar> op = MakeLidarOptions()->lidar;
    std::vector<Vector3d> grid;
    for (int i = 0; i < 10; i++) {
        for (int j = 0; j < 10; j++) {
            grid.emplace_back(0.5 * i, 0.5 * j, 0.0);
        }
    }
    std::shared_ptr<KD_TREE<PointXYZI>> tree = std::make_shared<KD_TREE<PointXYZI>>();
    tree->Build(CloudOf(grid)->points);

    POINTCLOUD_XYZI_PTR neighbors = std::make_shared<PointCloud<PointXYZI>>();
    ASSERT_TRUE(LidarHelper::get_neighbors(Vector3d(2.0, 2.0, 0.1), neighbors, tree, op));
    ASSERT_EQ((int)neighbors->size(), op->map_ngbr_num);
    for (const PointXYZI &p : neighbors->points) {
        EXPECT_LE(Vector3d(p.x - 2.0, p.y - 2.0, p.z).norm(), 0.52);
    }

    neighbors = std::make_shared<PointCloud<PointXYZI>>();
    EXPECT_FALSE(LidarHelper::get_neighbors(Vector3d(2.0, 2.0, 5.0), neighbors, tree, op));
}

// ---- Scan to map transform ----

TEST(LidarTransformToMap, PutsTheScanWhereTheAnchorScanSawTheSamePoints) {
    std::shared_ptr<State> state = MakeCloneState();
    std::shared_ptr<iKDDATA> ikd = std::make_shared<iKDDATA>(0);
    ikd->time = 0.1;
    std::shared_ptr<LiDARData> scan = std::make_shared<LiDARData>(0.3, 0.0, 0, ScanAt(state, 0.3), 100.0, 0.0);
    std::shared_ptr<LiDARData> anchor = std::make_shared<LiDARData>(0.1, 0.0, 0, ScanAt(state, 0.1), 100.0, 0.0);
    ASSERT_TRUE(LidarHelper::transform_to_map(state, scan, ikd));
    ASSERT_EQ(scan->pointcloud_in_map->size(), anchor->pointcloud->size());
    for (int i = 0; i < anchor->size(); i += 97) {
        EXPECT_LT((scan->pfinM(i) - anchor->p(i)).norm(), 1e-4) << i;
    }
}

// ---- Updater ----

TEST(UpdaterLidar, TheFirstScanBuildsTheMapAndUpdatesNothing) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterLidar updater(state);
    const MatrixXd cov_before = state->cov;
    updater.feed_measurement(ScanAt(state, 0.1));
    updater.try_update();
    EXPECT_TRUE(updater.ikd_data.at(0)->tree->initialized());
    EXPECT_DOUBLE_EQ(updater.ikd_data.at(0)->time, 0.1);
    EXPECT_TRUE(state->cov.isApprox(cov_before));
}

TEST(UpdaterLidar, TheNextScanIsMatchedAgainstTheMapAndShrinksTheCovariance) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterLidar updater(state);
    updater.feed_measurement(ScanAt(state, 0.1));
    const double trace_before = state->cov.trace();
    updater.feed_measurement(ScanAt(state, 0.3));
    updater.try_update();
    EXPECT_LT(state->cov.trace(), trace_before);
    EXPECT_DOUBLE_EQ(updater.ikd_data.at(0)->last_up_time, 0.3) << "the scan was not registered in the map";
}

TEST(UpdaterLidar, AScanTakenFurtherAlongPullsTheClonesApart) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterLidar updater(state);
    updater.feed_measurement(ScanAt(state, 0.1));
    const Vector3d offset(0.03, -0.02, 0.01);
    updater.feed_measurement(ScanFrom(state, TrajectoryPosition(0.3) + offset, 0.3));
    const Vector3d relative_before = state->clones.at(0.3)->pos() - state->clones.at(0.1)->pos();
    updater.try_update();
    const Vector3d moved = state->clones.at(0.3)->pos() - state->clones.at(0.1)->pos() - relative_before;
    for (int k = 0; k < 3; k++) {
        EXPECT_GT(moved(k) * offset(k), 0.0) << "axis " << k << " moved " << moved.transpose();
    }
    EXPECT_LT((moved - offset).norm(), (offset).norm()) << moved.transpose();
}

TEST(UpdaterLidar, AScanBetweenClonesIsUsedToo) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterLidar updater(state);
    updater.feed_measurement(ScanAt(state, 0.1));
    const double trace_before = state->cov.trace();
    updater.feed_measurement(ScanAt(state, 0.25));
    updater.try_update();
    EXPECT_LT(state->cov.trace(), trace_before);
    EXPECT_DOUBLE_EQ(updater.ikd_data.at(0)->last_up_time, 0.25);
}

TEST(UpdaterLidar, CalibratesTheExtrinsicToo) {
    std::shared_ptr<State> state = MakeCloneState(true);
    UpdaterLidar updater(state);
    const int ext = state->lidar_extrinsic.at(0)->id();
    const double ext_var_before = state->cov.block(ext, ext, 6, 6).trace();
    updater.feed_measurement(ScanAt(state, 0.1));
    updater.feed_measurement(ScanAt(state, 0.3));
    updater.try_update();
    EXPECT_LT(state->cov.block(ext, ext, 6, 6).trace(), ext_var_before);
}

TEST(UpdaterLidar, AScanTooCloseToTheNewestCloneWaits) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterLidar updater(state);
    updater.feed_measurement(ScanAt(state, 0.38));
    EXPECT_FALSE(updater.ikd_data.at(0)->tree->initialized());
}

TEST(UpdaterLidar, GivesTheLidarPoseFromTheInterpolatedIMUPose) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterLidar updater(state);
    const Matrix3d R_ItoL = state->lidar_extrinsic.at(0)->Rot();
    const Vector3d p_IinL = state->lidar_extrinsic.at(0)->pos();
    const std::pair<Matrix3d, Vector3d> pose = updater.get_pose_LinG(0, 0.15);
    EXPECT_TRUE(pose.first.isApprox(R_ItoL, 1e-9));
    EXPECT_TRUE(pose.second.isApprox(TrajectoryPosition(0.15) - R_ItoL.transpose() * p_IinL, 1e-9)) << pose.second.transpose();
}

TEST(UpdaterLidar, PropagatesAnOldMapToTheNewestClone) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterLidar updater(state);
    updater.feed_measurement(ScanAt(state, 0.1));
    updater.propagate_map_frame();
    std::shared_ptr<iKDDATA> ikd = updater.ikd_data.at(0);
    ASSERT_TRUE(ikd->tree->initialized());
    EXPECT_DOUBLE_EQ(ikd->time, 0.4);
    EXPECT_DOUBLE_EQ(ikd->last_up_time, 0.1) << "propagation is not a new scan";

    // The map now sits in the lidar frame at 0.4, so a scan from there finds every point on it.
    std::shared_ptr<LiDARData> scan = std::make_shared<LiDARData>(0.4, 0.0, 0, ScanAt(state, 0.4), 100.0, 0.0);
    for (int i = 0; i < scan->size(); i += 97) {
        POINTCLOUD_XYZI_PTR neighbors = std::make_shared<PointCloud<PointXYZI>>();
        EXPECT_TRUE(LidarHelper::get_neighbors(scan->p(i), neighbors, ikd->tree, state->op->lidar)) << i;
    }
}

TEST(UpdaterLidar, DropsAMapWhoseAnchorLeftTheCloneWindow) {
    std::shared_ptr<State> state = MakeCloneState();
    UpdaterLidar updater(state);
    updater.feed_measurement(ScanAt(state, 0.1));
    ASSERT_TRUE(updater.ikd_data.at(0)->tree->initialized());
    updater.ikd_data.at(0)->time = -1.0;
    updater.propagate_map_frame();
    EXPECT_FALSE(updater.ikd_data.at(0)->tree->initialized());
}

// ---- Bad match gate ----

TEST(UpdaterLidar, IgnoresPointsFarOffThePlaneOnEitherSide) {
    // Clone covariance and position after matching a scan, with or without stray points 0.4 m
    // above and below the floor, past the 3 * plane_max_p2pd gate.
    auto run = [](bool with_strays) {
        std::shared_ptr<State> state = MakeCloneState();
        UpdaterLidar updater(state);
        updater.feed_measurement(ScanAt(state, 0.1));
        std::vector<Vector3d> points = RoomInWorld();
        if (with_strays) {
            for (double a = 0.0; a <= 3.0; a += 0.5) {
                for (double b = -2.0; b <= 2.0; b += 0.5) {
                    points.emplace_back(a + 0.125, b + 0.125, -1.5 + 0.4);
                    points.emplace_back(a + 0.125, b + 0.125, -1.5 - 0.4);
                }
            }
        }
        updater.feed_measurement(ScanFrom(state, TrajectoryPosition(0.3), 0.3, points));
        updater.try_update();
        return std::make_pair(MatrixXd(state->cov), Vector3d(state->clones.at(0.3)->pos()));
    };
    const std::pair<MatrixXd, Vector3d> clean = run(false);
    const std::pair<MatrixXd, Vector3d> strays = run(true);
    EXPECT_TRUE(strays.first.isApprox(clean.first, 1e-9));
    EXPECT_TRUE(strays.second.isApprox(clean.second, 1e-9)) << (strays.second - clean.second).transpose();
}

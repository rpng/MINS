/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Covers the NavSatFix covariance to GPS noise conversion shared by the ROS1, ROS2 and ROS-free readers.
#include <gtest/gtest.h>
#include <cmath>

#include "update/gps/GPSTypes.h"

using namespace mins;

TEST(GPSTypes, NoiseIsTheSquareRootOfTheCovarianceDiagonal) {
    // RTK-fixed horizontal, a KAIST-like vertical; the off-diagonal terms are ignored
    double covariance[9] = {4e-4, 1.0, 1.0, 1.0, 9e-4, 1.0, 1.0, 1.0, 16.0};
    Eigen::Vector3d noise = gps_noise_from_covariance(covariance);
    EXPECT_DOUBLE_EQ(noise(0), 0.02);
    EXPECT_DOUBLE_EQ(noise(1), 0.03);
    EXPECT_DOUBLE_EQ(noise(2), 4.0);
}

TEST(GPSTypes, AMalformedVarianceGivesZeroSoTheConfiguredNoiseIsUsed) {
    double covariance[9] = {-1.0, 0, 0, 0, NAN, 0, 0, 0, 0};
    Eigen::Vector3d noise = gps_noise_from_covariance(covariance);
    EXPECT_EQ(noise, Eigen::Vector3d::Zero());
}

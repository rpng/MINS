/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

// Checks the ROS1 message decoders against messages serialized here field by field.
#include <gtest/gtest.h>
#include <cstring>
#include <string>
#include <vector>
#include "io/RosMsgs.h"
#include "update/lidar/PointCloud2Convert.h"
#include "utils/sensor_data.h"

using namespace mins;

namespace {
// Builds the ROS1 wire format: little-endian scalars, uint32 length before strings and variable arrays
struct Writer {
    std::vector<std::uint8_t> b;
    template <class T> Writer &put(T v) {
        const auto *p = reinterpret_cast<const std::uint8_t *>(&v);
        b.insert(b.end(), p, p + sizeof(T));
        return *this;
    }
    Writer &str(const std::string &s) {
        put<std::uint32_t>(static_cast<std::uint32_t>(s.size()));
        b.insert(b.end(), s.begin(), s.end());
        return *this;
    }
    Writer &header(std::uint32_t sec, std::uint32_t nsec) { return put<std::uint32_t>(7).put(sec).put(nsec).str("frame"); }
    Writer &doubles(std::initializer_list<double> v) {
        for (double x : v)
            put(x);
        return *this;
    }
    Writer &zeros(int n) {
        for (int i = 0; i < n; i++)
            put(0.0);
        return *this;
    }
};

std::vector<std::uint8_t> image_msg(const std::string &encoding, std::uint32_t h, std::uint32_t w, std::uint32_t step, const std::vector<std::uint8_t> &px) {
    Writer m;
    m.header(5, 0).put(h).put(w).str(encoding).put<std::uint8_t>(0).put(step).put<std::uint32_t>(static_cast<std::uint32_t>(px.size()));
    m.b.insert(m.b.end(), px.begin(), px.end());
    return m.b;
}
} // namespace

TEST(RosMsgs, ImuReadsStampRatesAndAccelerations) {
    Writer m;
    m.header(100, 500000000).doubles({0, 0, 0, 1}).zeros(9).doubles({0.1, 0.2, 0.3}).zeros(9).doubles({1.0, 2.0, 9.81}).zeros(9);
    auto d = rosmsg::imu(m.b);
    EXPECT_EQ(d.timestamp, 100.5);
    EXPECT_EQ(d.wm, Eigen::Vector3d(0.1, 0.2, 0.3));
    EXPECT_EQ(d.am, Eigen::Vector3d(1.0, 2.0, 9.81));
}

TEST(RosMsgs, StampIsComputedLikeRosTimeToSec) {
    Writer m;
    m.header(1630577757u, 546048415u).doubles({0, 0, 0, 1}).zeros(9).zeros(3).zeros(9).zeros(3).zeros(9);
    EXPECT_EQ(rosmsg::imu(m.b).timestamp, static_cast<double>(1630577757u) + 1e-9 * static_cast<double>(546048415u));
}

TEST(RosMsgs, RejectsTrailingBytesAndShortMessages) {
    Writer m;
    m.header(1, 0).doubles({0, 0, 0, 1}).zeros(9).zeros(3).zeros(9).zeros(3).zeros(9);
    auto longer = m.b;
    longer.push_back(0);
    EXPECT_THROW(rosmsg::imu(longer), std::runtime_error);
    auto shorter = m.b;
    shorter.resize(shorter.size() - 1);
    EXPECT_THROW(rosmsg::imu(shorter), std::runtime_error);
}

TEST(RosMsgs, Mono8ImageHonorsRowStep) {
    // 2x2 image with one byte of row padding: the padding must not leak into the pixels
    double stamp;
    cv::Mat img = rosmsg::image("sensor_msgs/Image", image_msg("mono8", 2, 2, 3, {1, 2, 99, 3, 4, 99}), stamp);
    EXPECT_EQ(stamp, 5.0);
    ASSERT_EQ(img.type(), CV_8UC1);
    EXPECT_EQ(img.at<std::uint8_t>(0, 1), 2);
    EXPECT_EQ(img.at<std::uint8_t>(1, 0), 3);
}

TEST(RosMsgs, Mono16ImageIsScaledLikeCvBridge) {
    // Expected values come from cv_bridge's own mono16 -> mono8 conversion
    std::vector<std::uint16_t> in = {255, 384, 511, 32767, 65280, 65534};
    std::vector<std::uint8_t> px(in.size() * 2);
    std::memcpy(px.data(), in.data(), px.size());
    double stamp;
    cv::Mat img = rosmsg::image("sensor_msgs/Image", image_msg("mono16", 1, 6, 12, px), stamp);
    std::vector<int> out(img.begin<std::uint8_t>(), img.end<std::uint8_t>());
    EXPECT_EQ(out, (std::vector<int>{1, 1, 2, 127, 254, 255}));
}

TEST(RosMsgs, RejectsAnUnknownImageEncoding) {
    double stamp;
    EXPECT_THROW(rosmsg::image("sensor_msgs/Image", image_msg("32FC1", 1, 1, 4, {0, 0, 0, 0}), stamp), std::runtime_error);
}

TEST(RosMsgs, BigEndian16BitImageIsByteSwapped) {
    // The same pixels as little-endian {255, 65280} must give the same gray values
    std::vector<std::uint8_t> msg = image_msg("mono16", 1, 2, 4, {0x00, 0xff, 0xff, 0x00});
    msg[msg.size() - 4 - 4 - 4 - 1] = 1; // is_bigendian sits before step, the data length and the 4 data bytes
    double stamp;
    cv::Mat img = rosmsg::image("sensor_msgs/Image", msg, stamp);
    std::vector<int> out(img.begin<std::uint8_t>(), img.end<std::uint8_t>());
    EXPECT_EQ(out, (std::vector<int>{1, 254}));
}

TEST(RosMsgs, PointCloud2FeedsFromPC2) {
    // Two points laid out x, y, z, t (float32) with 4 bytes of padding, like a driver with extra fields
    Writer m;
    m.header(3, 0).put<std::uint32_t>(1).put<std::uint32_t>(2).put<std::uint32_t>(4);
    for (auto f : {std::make_pair("x", 0u), std::make_pair("y", 4u), std::make_pair("z", 8u), std::make_pair("t", 12u)})
        m.str(f.first).put<std::uint32_t>(f.second).put<std::uint8_t>(7).put<std::uint32_t>(1);
    m.put<std::uint8_t>(0).put<std::uint32_t>(20).put<std::uint32_t>(40).put<std::uint32_t>(40);
    for (float v : {1.f, 2.f, 3.f, 0.f, 0.f, 4.f, 5.f, 6.f, 0.f, 0.f})
        m.put(v);
    m.put<std::uint8_t>(1);
    auto pc = rosmsg::pointcloud2(m.b);
    EXPECT_EQ(pc.stamp, 3.0);
    auto cloud = fromPC2(pc);
    ASSERT_EQ(cloud->points.size(), 2u);
    EXPECT_EQ(cloud->points[1].x, 4.f);
    EXPECT_EQ(cloud->points[1].z, 6.f);
}

TEST(RosMsgs, NavSatFixReadsPositionAndCovariance) {
    Writer m;
    m.header(9, 0).put<std::int8_t>(0).put<std::uint16_t>(1).doubles({36.1, 127.2, 50.5}).doubles({1, 0, 0, 0, 2, 0, 0, 0, 3}).put<std::uint8_t>(2);
    auto d = rosmsg::navsatfix(m.b);
    EXPECT_EQ(d.latitude, 36.1);
    EXPECT_EQ(d.altitude, 50.5);
    EXPECT_EQ(d.covariance[4], 2.0);
    EXPECT_EQ(d.covariance[8], 3.0);
}

TEST(RosMsgs, JointStateReadsNamesAndVelocities) {
    Writer m;
    m.header(2, 0).put<std::uint32_t>(2).str("left").str("right");
    m.put<std::uint32_t>(0).put<std::uint32_t>(2).doubles({0.5, -0.5}).put<std::uint32_t>(0);
    auto d = rosmsg::jointstate(m.b);
    EXPECT_EQ(d.name, (std::vector<std::string>{"left", "right"}));
    EXPECT_EQ(d.velocity, (std::vector<double>{0.5, -0.5}));
    EXPECT_TRUE(d.position.empty());
}

TEST(RosMsgs, OdometryReadsTheTwist) {
    Writer m;
    m.header(4, 0).str("base").zeros(3).doubles({0, 0, 0, 1}).zeros(36).doubles({1.5, 0, 0}).doubles({0, 0, 0.25}).zeros(36);
    auto d = rosmsg::odometry(m.b);
    EXPECT_EQ(d.linear(0), 1.5);
    EXPECT_EQ(d.angular(2), 0.25);
}

TEST(RosMsgs, PoseReadsPoseStampedAndTransformStamped) {
    Writer ps;
    ps.header(6, 0).doubles({1, 2, 3}).doubles({0, 0, 0.6, 0.8});
    auto a = rosmsg::pose("geometry_msgs/PoseStamped", ps.b);
    EXPECT_EQ(a.p, Eigen::Vector3d(1, 2, 3));
    EXPECT_EQ(a.q, Eigen::Vector4d(0, 0, 0.6, 0.8));

    Writer ts;
    ts.header(6, 0).str("child").doubles({4, 5, 6}).doubles({0.6, 0, 0, 0.8});
    auto b = rosmsg::pose("geometry_msgs/TransformStamped", ts.b);
    EXPECT_EQ(b.p, Eigen::Vector3d(4, 5, 6));
    EXPECT_EQ(b.q, Eigen::Vector4d(0.6, 0, 0, 0.8));
}

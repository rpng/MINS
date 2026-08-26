// Covers the parser-less OptionsWheel defaults and the wheel-only State the other tests build on.
#include <gtest/gtest.h>

#include "WheelTestFixture.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"

using namespace mins;

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
    std::shared_ptr<State> state = wheel_test::MakeWheelOnlyState(WheelType::Wheel2DAng);
    ASSERT_NE(state, nullptr);
    ASSERT_NE(state->wheel_extrinsic, nullptr);
    ASSERT_NE(state->wheel_intrinsic, nullptr);
    ASSERT_NE(state->wheel_dt, nullptr);
    EXPECT_EQ(state->wheel_extrinsic->size(), 6);
    EXPECT_EQ(state->wheel_intrinsic->size(), 3);
    EXPECT_DOUBLE_EQ(state->wheel_dt->value()(0), 0.0);
}

TEST(WheelOnlyState, CovarianceCoversEveryCalibrationVariable) {
    std::shared_ptr<State> state = wheel_test::MakeWheelOnlyState(WheelType::Wheel2DAng);
    // IMU (15) + timeoffset (1) + extrinsic (6) + intrinsic (3), all three calibrated by default.
    EXPECT_EQ(state->cov.rows(), 25);
    EXPECT_EQ(state->cov.cols(), 25);
    EXPECT_TRUE(state->cov.isApprox(state->cov.transpose()));
    EXPECT_GT(state->cov.diagonal().minCoeff(), 0.0);
}

TEST(WheelOnlyState, IntrinsicCalibrationFlagSizesTheState) {
    std::shared_ptr<OptionsEstimator> op = wheel_test::MakeWheelOnlyOptions(WheelType::Wheel2DAng);
    EXPECT_TRUE(op->wheel->do_calib_int);
    op->wheel->do_calib_int = false;
    std::shared_ptr<State> state = std::make_shared<State>(op);
    EXPECT_EQ(state->cov.rows(), 22);
}

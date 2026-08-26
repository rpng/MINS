// Covers the measurement stack of UpdaterWheel: feeding, pruning, and window selection.
#include <gtest/gtest.h>

#include "WheelTestFixture.h"
#include "update/wheel/UpdaterWheel.h"
#include "update/wheel/WheelTypes.h"

using namespace mins;

namespace {

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
    std::shared_ptr<State> state = wheel_test::MakeWheelOnlyState(WheelType::Wheel2DAng);
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

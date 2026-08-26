// Shared setup for the wheel updater tests: builds the smallest State that UpdaterWheel needs.
#ifndef MINS_TESTS_WHEELTESTFIXTURE_H
#define MINS_TESTS_WHEELTESTFIXTURE_H

#include <memory>

#include "options/OptionsCamera.h"
#include "options/OptionsEstimator.h"
#include "options/OptionsGPS.h"
#include "options/OptionsLidar.h"
#include "options/OptionsVicon.h"
#include "options/OptionsWheel.h"
#include "state/State.h"
#include "update/wheel/WheelTypes.h"

namespace mins {
namespace wheel_test {

/// Options for a wheel-only estimator, loaded with no yaml parser so every value is a default.
inline std::shared_ptr<OptionsEstimator> MakeWheelOnlyOptions(WheelType type) {
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

/// State holding only the IMU and the wheel calibration variables.
inline std::shared_ptr<State> MakeWheelOnlyState(WheelType type) {
    return std::make_shared<State>(MakeWheelOnlyOptions(type));
}

} // namespace wheel_test
} // namespace mins

#endif // MINS_TESTS_WHEELTESTFIXTURE_H

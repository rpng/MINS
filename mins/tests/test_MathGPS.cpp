// Covers the WGS-84 geodetic conversions: geodetic to ECEF, ECEF to local ENU, and the
// composition of the two. Every expectation is an analytic property of the ellipsoid rather
// than a number copied out of a previous run.
#include <gtest/gtest.h>

#include "update/gps/MathGPS.h"

namespace {

/// Somewhere with all three coordinates non-zero, so a dropped term cannot pass unnoticed.
const Eigen::Vector3d DATUM = Eigen::Vector3d(39.6837, -75.7497, 20.0);

/// Metres. The conversions are pure trigonometry on values of order 6.4e6, so a millimetre is
/// already several orders below the double precision floor of the intermediate products.
const double TOL_M = 1e-6;

} // namespace

TEST(MathGPS, EcefAtTheEquatorOnThePrimeMeridianIsTheSemimajorAxis) {
    Eigen::Vector3d ecef = MathGPS::GeodeticToEcef(Eigen::Vector3d(0, 0, 0));
    EXPECT_NEAR(ecef(0), MathGPS::a, TOL_M);
    EXPECT_NEAR(ecef(1), 0.0, TOL_M);
    EXPECT_NEAR(ecef(2), 0.0, TOL_M);
}

TEST(MathGPS, EcefAtTheNorthPoleIsTheSemiminorAxis) {
    Eigen::Vector3d ecef = MathGPS::GeodeticToEcef(Eigen::Vector3d(90, 0, 0));
    EXPECT_NEAR(ecef(0), 0.0, TOL_M);
    EXPECT_NEAR(ecef(1), 0.0, TOL_M);
    EXPECT_NEAR(ecef(2), MathGPS::b, TOL_M);
}

TEST(MathGPS, EcefLongitudeSweepsTheEquatorialCircle) {
    for (double lon : {-180.0, -90.0, 0.0, 45.0, 90.0, 179.0}) {
        Eigen::Vector3d ecef = MathGPS::GeodeticToEcef(Eigen::Vector3d(0, lon, 0));
        double lon_rad = lon * M_PI / 180.0;
        EXPECT_NEAR(ecef(0), MathGPS::a * cos(lon_rad), TOL_M) << lon;
        EXPECT_NEAR(ecef(1), MathGPS::a * sin(lon_rad), TOL_M) << lon;
        EXPECT_NEAR(ecef(2), 0.0, TOL_M) << lon;
    }
}

TEST(MathGPS, EcefHeightMovesAlongTheEllipsoidNormal) {
    // Height enters as a displacement along the surface normal, so whatever the latitude, two
    // points on the same vertical differ by exactly the height difference.
    const double height = 1234.5;
    for (double lat : {-60.0, 0.0, 12.5, 39.6837, 89.0}) {
        Eigen::Vector3d at_surface = MathGPS::GeodeticToEcef(Eigen::Vector3d(lat, -75.7497, 0));
        Eigen::Vector3d at_height = MathGPS::GeodeticToEcef(Eigen::Vector3d(lat, -75.7497, height));
        EXPECT_NEAR((at_height - at_surface).norm(), height, TOL_M) << lat;
    }
}

TEST(MathGPS, EnuIsZeroAtTheDatum) {
    Eigen::Vector3d enu = MathGPS::EcefToEnu(MathGPS::GeodeticToEcef(DATUM), DATUM);
    EXPECT_NEAR(enu.norm(), 0.0, TOL_M);
}

TEST(MathGPS, EnuAxesPointEastNorthAndUp) {
    const double step_deg = 1e-3; // about 100 m, small enough that the axes stay nearly straight

    Eigen::Vector3d east = MathGPS::GeodeticToEnu(DATUM + Eigen::Vector3d(0, step_deg, 0), DATUM);
    EXPECT_GT(east(0), 0.0);
    EXPECT_GT(std::abs(east(0)), std::abs(east(1)));

    Eigen::Vector3d north = MathGPS::GeodeticToEnu(DATUM + Eigen::Vector3d(step_deg, 0, 0), DATUM);
    EXPECT_GT(north(1), 0.0);
    EXPECT_GT(std::abs(north(1)), std::abs(north(0)));

    Eigen::Vector3d up = MathGPS::GeodeticToEnu(DATUM + Eigen::Vector3d(0, 0, 500), DATUM);
    EXPECT_NEAR(up(0), 0.0, TOL_M);
    EXPECT_NEAR(up(1), 0.0, TOL_M);
    EXPECT_NEAR(up(2), 500.0, TOL_M);
}

TEST(MathGPS, EnuIsARotationSoItPreservesDistanceFromTheDatum) {
    Eigen::Vector3d datum_ecef = MathGPS::GeodeticToEcef(DATUM);
    for (const Eigen::Vector3d &point : {Eigen::Vector3d(39.7, -75.7, 100.0),
                                         Eigen::Vector3d(-33.9, 151.2, 58.0),
                                         Eigen::Vector3d(0.0, 0.0, 0.0)}) {
        Eigen::Vector3d point_ecef = MathGPS::GeodeticToEcef(point);
        EXPECT_NEAR(MathGPS::EcefToEnu(point_ecef, DATUM).norm(), (point_ecef - datum_ecef).norm(),
                    TOL_M)
            << point.transpose();
    }
}

TEST(MathGPS, GeodeticToEnuIsTheTwoStepsComposed) {
    Eigen::Vector3d point(39.7000, -75.7000, 55.0);
    Eigen::Vector3d composed = MathGPS::EcefToEnu(MathGPS::GeodeticToEcef(point), DATUM);
    EXPECT_TRUE(MathGPS::GeodeticToEnu(point, DATUM).isApprox(composed));
}

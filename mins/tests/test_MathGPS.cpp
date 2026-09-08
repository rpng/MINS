// Covers MathGPS: the WGS-84 conversions from geodetic to ECEF to local ENU, the 4-DOF
// alignment that fits a yaw and a translation to point correspondences, and the quaternion
// product matrices. Every expectation is an analytic property of the ellipsoid or of the
// least-squares problem rather than a number copied out of a previous run.
#include <algorithm>
#include <gtest/gtest.h>

#include "update/gps/MathGPS.h"

namespace {

/// Somewhere with all three coordinates non-zero, so a dropped term cannot pass unnoticed.
const Eigen::Vector3d DATUM = Eigen::Vector3d(39.6837, -75.7497, 20.0);

/// Metres. The conversions are pure trigonometry on values of order 6.4e6, so a millimetre is
/// already several orders below the double precision floor of the intermediate products.
const double TOL_M = 1e-6;

/// The 4-DOF solver runs a quartic through complex arithmetic and so keeps fewer digits than
/// the conversions do. Still nine orders below any residual a real GPS receiver produces.
const double TOL_SOLVER = 1e-9;

/// The only rotation the 4-DOF solver can represent: the two frames share their z axis.
Eigen::Matrix3d YawRotation(double yaw_rad) {
    return Eigen::Matrix3d(Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()));
}

/// Points spread over all three axes, so a solver that quietly drops z cannot pass.
std::vector<Eigen::Vector3d> SamplePoints() {
    return {Eigen::Vector3d(1.0, 0.0, 0.5),   Eigen::Vector3d(0.0, 2.0, -1.0),
            Eigen::Vector3d(-3.0, 1.5, 2.0),  Eigen::Vector3d(4.0, -2.5, 0.0),
            Eigen::Vector3d(-1.0, -1.0, 3.0), Eigen::Vector3d(2.5, 3.5, -2.0)};
}

/// Applies the transform the solver is then asked to recover.
std::vector<Eigen::Vector3d> Transformed(const std::vector<Eigen::Vector3d> &p_inB, const Eigen::Matrix3d &R_BtoA,
                                         const Eigen::Vector3d &p_BinA) {
    std::vector<Eigen::Vector3d> p_inA;
    for (const Eigen::Vector3d &p : p_inB) {
        p_inA.push_back(R_BtoA * p + p_BinA);
    }
    return p_inA;
}

/// Recovers the yaw angle back out of a solved rotation.
double YawOf(const Eigen::Matrix3d &R) { return atan2(R(1, 0), R(0, 0)); }

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

TEST(MathGPS, FourDofRecoversAPureYawAndTranslation) {
    const Eigen::Matrix3d R_BtoA = YawRotation(35.0 * M_PI / 180.0);
    const Eigen::Vector3d p_BinA(10.0, -4.0, 2.0);
    std::vector<Eigen::Vector3d> p_inB = SamplePoints();
    std::vector<Eigen::Vector3d> p_inA = Transformed(p_inB, R_BtoA, p_BinA);

    Eigen::Matrix3d R_solved;
    Eigen::Vector3d p_solved;
    ASSERT_TRUE(MathGPS::Ransac_4Dof(p_inA, p_inB, R_solved, p_solved, 1, 1.0));
    EXPECT_LT((R_solved - R_BtoA).norm(), TOL_SOLVER);
    EXPECT_LT((p_solved - p_BinA).norm(), TOL_SOLVER);
}

TEST(MathGPS, FourDofYawIsRecoveredAllTheWayAroundTheCircle) {
    const Eigen::Vector3d p_BinA(-2.0, 7.0, 0.25);
    for (double yaw_deg : {-179.0, -120.0, -45.0, 0.0, 30.0, 90.0, 175.0}) {
        const Eigen::Matrix3d R_BtoA = YawRotation(yaw_deg * M_PI / 180.0);
        std::vector<Eigen::Vector3d> p_inB = SamplePoints();
        std::vector<Eigen::Vector3d> p_inA = Transformed(p_inB, R_BtoA, p_BinA);

        Eigen::Matrix3d R_solved;
        Eigen::Vector3d p_solved;
        ASSERT_TRUE(MathGPS::Ransac_4Dof(p_inA, p_inB, R_solved, p_solved, 1, 1.0)) << yaw_deg;
        EXPECT_NEAR(YawOf(R_solved), yaw_deg * M_PI / 180.0, TOL_SOLVER) << yaw_deg;

        // The solve is unconstrained apart from the yaw, so the shape of what comes back is
        // worth pinning too: a rotation, and one that leaves the shared z axis alone.
        EXPECT_LT((R_solved.transpose() * R_solved - Eigen::Matrix3d::Identity()).norm(), TOL_SOLVER) << yaw_deg;
        EXPECT_LT((R_solved * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ()).norm(), TOL_SOLVER) << yaw_deg;
    }
}

TEST(MathGPS, FourDofNeedsOnlyTwoCorrespondences) {
    // Two points leave a single difference vector, which is exactly enough to fix one angle.
    const Eigen::Matrix3d R_BtoA = YawRotation(60.0 * M_PI / 180.0);
    const Eigen::Vector3d p_BinA(1.0, 2.0, 3.0);
    std::vector<Eigen::Vector3d> p_inB = {Eigen::Vector3d(1.0, 0.0, 0.5), Eigen::Vector3d(-2.0, 3.0, 1.5)};
    std::vector<Eigen::Vector3d> p_inA = Transformed(p_inB, R_BtoA, p_BinA);

    Eigen::Matrix3d R_solved;
    Eigen::Vector3d p_solved;
    ASSERT_TRUE(MathGPS::Ransac_4Dof(p_inA, p_inB, R_solved, p_solved, 1, 1.0));
    EXPECT_LT((R_solved - R_BtoA).norm(), TOL_SOLVER);
    EXPECT_LT((p_solved - p_BinA).norm(), TOL_SOLVER);
}

TEST(MathGPS, FourDofRecoversTheIdentityWhenTheFramesAlreadyAlign) {
    std::vector<Eigen::Vector3d> p_inB = SamplePoints();
    std::vector<Eigen::Vector3d> p_inA = p_inB;

    Eigen::Matrix3d R_solved;
    Eigen::Vector3d p_solved;
    ASSERT_TRUE(MathGPS::Ransac_4Dof(p_inA, p_inB, R_solved, p_solved, 1, 1.0));
    EXPECT_LT((R_solved - Eigen::Matrix3d::Identity()).norm(), TOL_SOLVER);
    EXPECT_LT(p_solved.norm(), TOL_SOLVER);
}

TEST(MathGPS, FourDofMatchesTheClosedFormProcrustesYaw) {
    // Data no single yaw can fit, so the solver has to return a least-squares angle rather than
    // an exact one. Every block of A is a scaled 2-D rotation, which makes A^T A a multiple of
    // the identity, and the constrained optimum then collapses to the classic Procrustes angle:
    // atan2 of the summed cross products over the summed dot products.
    //
    // The fit takes its differences against one of the correspondences rather than against the
    // centroid, so that angle depends on which correspondence lands first, and Ransac_4Dof
    // shuffles the input before solving. Checking every candidate pivot pins the solver against
    // theory without the test having to know the shuffle.
    const Eigen::Matrix3d R_BtoA = YawRotation(20.0 * M_PI / 180.0);
    const Eigen::Vector3d p_BinA(0.5, -1.5, 4.0);
    std::vector<Eigen::Vector3d> p_inB = SamplePoints();
    std::vector<Eigen::Vector3d> p_inA = Transformed(p_inB, R_BtoA, p_BinA);
    p_inA[2] += Eigen::Vector3d(0.05, -0.03, 0.0);
    p_inA[4] += Eigen::Vector3d(-0.02, 0.04, 0.0);

    std::vector<double> yaw_per_pivot;
    for (size_t pivot = 0; pivot < p_inB.size(); pivot++) {
        double dot_sum = 0.0, cross_sum = 0.0;
        for (size_t i = 0; i < p_inB.size(); i++) {
            if (i == pivot) {
                continue;
            }
            Eigen::Vector2d u = (p_inB[i] - p_inB[pivot]).head<2>();
            Eigen::Vector2d v = (p_inA[i] - p_inA[pivot]).head<2>();
            dot_sum += u.dot(v);
            cross_sum += u(0) * v(1) - u(1) * v(0);
        }
        yaw_per_pivot.push_back(atan2(cross_sum, dot_sum));
    }

    Eigen::Matrix3d R_solved;
    Eigen::Vector3d p_solved;
    ASSERT_TRUE(MathGPS::Ransac_4Dof(p_inA, p_inB, R_solved, p_solved, 1, 1.0));

    double solved_yaw = YawOf(R_solved);
    double closest = std::abs(yaw_per_pivot[0] - solved_yaw);
    for (double yaw : yaw_per_pivot) {
        closest = std::min(closest, std::abs(yaw - solved_yaw));
    }
    EXPECT_LT(closest, TOL_SOLVER);

    // The pivot has to actually move the answer, or the check above proves nothing. The
    // measured spread is about 1.2e-2 rad, seven orders above the tolerance it is compared to.
    double spread = *std::max_element(yaw_per_pivot.begin(), yaw_per_pivot.end()) -
                    *std::min_element(yaw_per_pivot.begin(), yaw_per_pivot.end());
    EXPECT_GT(spread, 1e-3);

    // This is the only case where no yaw fits, so it is the only one where the unit-norm
    // constraint on w does any work. What comes back still has to be a rotation.
    EXPECT_LT((R_solved.transpose() * R_solved - Eigen::Matrix3d::Identity()).norm(), TOL_SOLVER);
}

TEST(MathGPS, FourDofReportsFailureWhenEveryCorrespondenceIsTheSamePoint) {
    // What a stationary receiver looks like: the yaw is unobservable, there is no direction to
    // fit, and the solve has to say so rather than hand back whatever fell out of the quartic.
    std::vector<Eigen::Vector3d> p_inB(4, Eigen::Vector3d(2.0, -1.0, 0.5));
    std::vector<Eigen::Vector3d> p_inA(4, Eigen::Vector3d(7.0, 7.0, 7.0));

    Eigen::Matrix3d R_solved;
    Eigen::Vector3d p_solved;
    EXPECT_FALSE(MathGPS::Ransac_4Dof(p_inA, p_inB, R_solved, p_solved, 1, 1.0));
}

TEST(MathGPS, QuaternionLeftAndRightMatricesBothGiveTheJplProduct) {
    Eigen::Vector4d q = Eigen::Vector4d(0.2, -0.3, 0.5, 0.8).normalized();
    Eigen::Vector4d p = Eigen::Vector4d(-0.4, 0.1, 0.2, 0.9).normalized();

    // JPL convention, so the vector part carries a minus on the cross product.
    Eigen::Vector4d expected;
    expected.head<3>() = q(3) * p.head<3>() + p(3) * q.head<3>() - q.head<3>().cross(p.head<3>());
    expected(3) = q(3) * p(3) - q.head<3>().dot(p.head<3>());

    EXPECT_LT((MathGPS::Left_q(q) * p - expected).norm(), TOL_SOLVER);
    EXPECT_LT((MathGPS::Right_q(p) * q - expected).norm(), TOL_SOLVER);
}

TEST(MathGPS, QuaternionMatricesOfTheIdentityRotationAreTheIdentity) {
    Eigen::Vector4d identity(0.0, 0.0, 0.0, 1.0);
    EXPECT_LT((MathGPS::Left_q(identity) - Eigen::Matrix4d::Identity()).norm(), TOL_SOLVER);
    EXPECT_LT((MathGPS::Right_q(identity) - Eigen::Matrix4d::Identity()).norm(), TOL_SOLVER);
}

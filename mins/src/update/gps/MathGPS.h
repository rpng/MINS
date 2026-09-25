/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2019-2026 Woosik Lee
 * Copyright (C) 2019-2026 Guoquan Huang
 * Copyright (C) 2019-2026 MINS Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef GPSCONVERSION_H
#define GPSCONVERSION_H

#include "utils/quat_ops.h"
#include <Eigen/Eigen>
#include <cmath>
#include <numeric>
#include <random>

using namespace std;
using namespace Eigen;
using namespace ov_core;

/**
 * \brief Converts lat,lon,height coordinates into ENU frame
 *
 * The implementation here is according to the paper:
 * - "Conversion of Geodetic coordinates to the Local Tangent Plane" Version 2.01.
 * - "The basic reference for this paper is J.Farrell & M.Barth 'The Global Positioning System & Inertial Navigation'"
 * - Also helpful is Wikipedia: http://en.wikipedia.org/wiki/Geodetic_datum
 * - Taken from https://gist.github.com/govert/1b373696c9a27ff4c72a
 */
class MathGPS {

public:
  // WGS-84 geodetic constants
  static constexpr double a = 6378137;      // WGS-84 Earth semimajor axis (m)
  static constexpr double b = 6356752.3142; // WGS-84 Earth semiminor axis (m)

  /**
   * Converts WGS-84 Geodetic point (lat, lon, h) to the
   * Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
   */
  static Vector3d GeodeticToEcef(Vector3d meas) {
    double lat = meas(0);
    double lon = meas(1);
    double h = meas(2);

    double f = (a - b) / a;    // Ellipsoid Flatness
    double e_sq = f * (2 - f); // Square of Eccentricenu ity

    // Convert to radians in notation consistent with the paper:
    double lambda = DegreeToRadian(lat);
    double phi = DegreeToRadian(lon);
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    Vector3d xyz_ecef;
    xyz_ecef(0) = (h + N) * cos_lambda * cos_phi;
    xyz_ecef(1) = (h + N) * cos_lambda * sin_phi;
    xyz_ecef(2) = (h + (1 - e_sq) * N) * sin_lambda;
    return xyz_ecef;
  }

  /**
   * Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
   * East-North-Up coordinates in a Local Tangent Plane that is centered at the
   * (WGS-84) Geodetic point (lat0, lon0, h0).
   */
  static Vector3d EcefToEnu(Vector3d xyz_ecef, Vector3d datum) {

    double f = (a - b) / a;    // Ellipsoid Flatness
    double e_sq = f * (2 - f); // Square of Eccentricity

    // Convert to radians in notation consistent with the paper:
    double lambda = DegreeToRadian(datum(0));
    double phi = DegreeToRadian(datum(1));
    double s = sin(lambda);
    double N = a / sqrt(1 - e_sq * s * s);

    double sin_lambda = sin(lambda);
    double cos_lambda = cos(lambda);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);

    double x0 = (datum(2) + N) * cos_lambda * cos_phi;
    double y0 = (datum(2) + N) * cos_lambda * sin_phi;
    double z0 = (datum(2) + (1 - e_sq) * N) * sin_lambda;

    double xd, yd, zd;
    xd = xyz_ecef(0) - x0;
    yd = xyz_ecef(1) - y0;
    zd = xyz_ecef(2) - z0;

    // This is the matrix multiplication
    Vector3d xyz_enu;
    xyz_enu(0) = -sin_phi * xd + cos_phi * yd;
    xyz_enu(1) = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
    xyz_enu(2) = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
    return xyz_enu;
  }

  /**
   * Converts the geodetic WGS-84 coordinated (lat, lon, h) to
   * East-North-Up coordinates in a Local Tangent Plane that is centered at the
   * (WGS-84) Geodetic point (lat0, lon0, h0).
   */
  static Vector3d GeodeticToEnu(Vector3d meas, Vector3d datum) {
    Vector3d xyz_ecef = GeodeticToEcef(meas);
    return EcefToEnu(xyz_ecef, datum);
  }

  /**
   * \brief Fits the 4-DOF transform between two z-aligned frames to point correspondences.
   *
   * The two frames share their z axis, so the fit has one rotational degree of freedom. The
   * yaw comes from a norm-constrained least squares over all of the correspondences and the
   * translation is the mean of what that rotation leaves over.
   *
   * \param[in] p_A Points expressed in frame A.
   * \param[in] p_B The same points expressed in frame B.
   * \param[out] R_BtoA Recovered rotation, about z only.
   * \param[out] p_BinA Recovered translation.
   * \param[in] inlierThresh Largest residual, in metres, that still counts as a fit.
   * \return False if no yaw could be solved for, or if the fit leaves every point beyond the
   *         threshold. The outputs are only meaningful when this returns true.
   */
  static bool Align_4Dof(const vector<Vector3d> &p_A, const vector<Vector3d> &p_B, Matrix3d &R_BtoA, Vector3d &p_BinA, double inlierThresh) {
    if (!compute_4Dof(p_A, p_B, R_BtoA, p_BinA)) {
      return false;
    }
    return count_inliers(p_A, p_B, R_BtoA, p_BinA, inlierThresh) > 0;
  }

  /**
   * \brief Fits the 4-DOF transform over random subsets of the correspondences and keeps the
   * hypothesis with the most inliers.
   *
   * \param[in] p_A Points expressed in frame A.
   * \param[in] p_B The same points expressed in frame B.
   * \param[out] R_BtoA Recovered rotation, about z only.
   * \param[out] p_BinA Recovered translation.
   * \param[in] numHypotheses How many subsets to fit.
   * \param[in] inlierThresh Largest residual, in metres, that still counts as an inlier.
   * \return False if no hypothesis produced an inlier. The outputs are only meaningful when
   *         this returns true.
   */
  static bool Ransac_4Dof(const vector<Vector3d> &p_A, const vector<Vector3d> &p_B, Matrix3d &R_BtoA, Vector3d &p_BinA, size_t numHypotheses, double inlierThresh) {
    // TODO: this is not RANSAC yet. The subset is the whole point set rather than a minimal
    // one, the generator is reseeded on every call, and the winning hypothesis is never refit
    // on its inliers, so outliers pull on every hypothesis equally. Fixing that needs outlier
    // data to test against, which the simulator does not produce yet.
    size_t sizeSubset = p_A.size();
    vector<Vector3d> suBp_A, suBp_B;
    size_t numInliers = 0;
    Matrix3d R_BtoAhyp;
    Vector3d p_BinAhyp;

    for (size_t i = 0; i < numHypotheses; i++) {
      suBp_A.clear();
      suBp_B.clear();
      get_random_subset(p_A, p_B, suBp_A, suBp_B, sizeSubset);
      if (!compute_4Dof(suBp_A, suBp_B, R_BtoAhyp, p_BinAhyp)) {
        continue;
      }

      size_t inliershyp = count_inliers(p_A, p_B, R_BtoAhyp, p_BinAhyp, inlierThresh);
      if (inliershyp > numInliers) {
        R_BtoA = R_BtoAhyp;
        p_BinA = p_BinAhyp;
        numInliers = inliershyp;
      }
    }
    return numInliers > 0;
  }

  static inline Matrix4d Left_q(Vector4d q) {
    Matrix4d L;
    L.block(0, 0, 3, 3) = q(3, 0) * Matrix3d::Identity() - skew_x(q.block(0, 0, 3, 1));
    L.block(0, 3, 4, 1) = q;
    L.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();

    return L;
  };

  static inline Matrix4d Right_q(Vector4d q) {
    Matrix4d R;
    R.block(0, 0, 3, 3) = q(3, 0) * Matrix3d::Identity() + skew_x(q.block(0, 0, 3, 1));
    R.block(0, 3, 4, 1) = q;
    R.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();

    return R;
  };

private:
  /**
   * Converts degrees to radians
   * \param angle The angle in degrees
   * \return Angle converted into radians
   */
  static double DegreeToRadian(double angle) { return M_PI * angle / 180.0; }

  // Get random subsets of given sets
  static void get_random_subset(const vector<Vector3d> &p_A, const vector<Vector3d> &p_B, vector<Vector3d> &suBp_A, vector<Vector3d> &suBp_B, size_t sizeSubset) {

    mt19937 rng(1337);
    vector<unsigned int> indices(p_A.size());
    iota(indices.begin(), indices.end(), 0);
    shuffle(indices.begin(), indices.end(), rng);
    for (size_t i = 0; i < sizeSubset; i++) {
      suBp_A.push_back(p_A[indices[i]]);
      suBp_B.push_back(p_B[indices[i]]);
    }
  }

  // Count how many correspondences the given transform explains to within the threshold
  static size_t count_inliers(const vector<Vector3d> &p_A, const vector<Vector3d> &p_B, const Matrix3d &R_BtoA, const Vector3d &p_BinA, double inlierThresh) {
    size_t inliers = 0;
    for (size_t i = 0; i < p_A.size(); i++) {
      if ((p_A[i] - (R_BtoA * p_B[i] + p_BinA)).norm() <= inlierThresh) {
        inliers++;
      }
    }
    return inliers;
  }

  // Get the full four dof transformation between z-aligned frames A and B based on point correspondences
  static bool compute_4Dof(const vector<Vector3d> &p_inA, const vector<Vector3d> &p_inB, Matrix3d &R_BtoA, Vector3d &p_BinA) {
    bool found_solution = compute_RBtoA1Dof(R_BtoA, p_inA, p_inB);

    p_BinA.setZero();
    double M = p_inA.size();

    for (size_t i = 0; i < p_inA.size(); i++) {
      p_BinA += (1.0 / M) * (p_inA[i] - R_BtoA * p_inB[i]);
    }

    return found_solution;
  }

  // Get the 1 dof yaw rotation between z-aligned frames A and B based on point correspondences
  static bool compute_RBtoA1Dof(Matrix3d &R_BtoA, const vector<Vector3d> &p_inA, const vector<Vector3d> &p_inB) {

    assert(p_inA.size() == p_inB.size());
    assert(p_inA.size() > 1);
    // Build A
    Matrix<double, -1, 2> A;
    A.resize(2 * (p_inA.size() - 1), 2);
    // Build b
    Matrix<double, -1, 1> b;
    b.resize(2 * (p_inA.size() - 1), 1);

    // Build A by subtracting out the first reading of each vector and projecting onto the xy  plane
    for (size_t i = 1; i < p_inA.size(); i++) {
      Vector2d p_inA_proj = (p_inA[i] - p_inA[0]).block(0, 0, 2, 1);
      Vector2d p_inB_proj = (p_inB[i] - p_inB[0]).block(0, 0, 2, 1);

      b.block(2 * (i - 1), 0, 2, 1) = p_inA_proj;
      A.block(2 * (i - 1), 0, 2, 2) << p_inB_proj(0), -p_inB_proj(1), p_inB_proj(1), p_inB_proj(0);
    }

    Vector2d w;
    bool found_solution = solve_QCQP(A, b, w);

    R_BtoA.setIdentity();
    R_BtoA.block(0, 0, 2, 1) = w;
    R_BtoA(0, 1) = -w(1);
    R_BtoA(1, 1) = w(0);
    return found_solution;
  }

  // Solve the 2-D quadratically constrained least squares min ||A w - b|| subject to ||w|| = 1.
  //
  // Every 2x2 block of A is a scaled rotation, so A^T A is a multiple of the identity and on the
  // unit circle the residual reduces to a constant minus 2 (A^T b) . w. The minimizer is then the
  // direction of A^T b, with no multiplier to search for.
  static bool solve_QCQP(const Matrix<double, -1, 2> &A, const Matrix<double, -1, 1> &b, Vector2d &w) {

    Vector2d ATb = A.transpose() * b;

    // Degenerate input leaves every direction equally good, and a NaN norm fails this test too
    w << 1, 0;
    if (!(ATb.norm() > 1e-12)) {
      return false;
    }

    w = ATb.normalized();
    return true;
  }
};

#endif // GPSCONVERSION_H
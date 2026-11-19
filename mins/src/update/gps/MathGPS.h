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
#include <algorithm>
#include <cmath>
#include <random>

using namespace std;
using namespace Eigen;
using namespace ov_core;

namespace mins {

/**
 * \brief Converts lat,lon,height coordinates into ENU frame, and fits 4-DOF transforms between
 * z-aligned frames.
 *
 * The geodetic implementation here is according to the paper:
 * - "Conversion of Geodetic coordinates to the Local Tangent Plane" Version 2.01.
 * - "The basic reference for this paper is J.Farrell & M.Barth 'The Global Positioning System & Inertial Navigation'"
 * - Also helpful is Wikipedia: http://en.wikipedia.org/wiki/Geodetic_datum
 * - Taken from https://gist.github.com/govert/1b373696c9a27ff4c72a
 */
namespace gps_math {

// WGS-84 geodetic constants
inline constexpr double a = 6378137;      // WGS-84 Earth semimajor axis (m)
inline constexpr double b = 6356752.3142; // WGS-84 Earth semiminor axis (m)

/**
 * Converts degrees to radians
 * \param angle The angle in degrees
 * \return Angle converted into radians
 */
inline double DegreeToRadian(double angle) { return M_PI * angle / 180.0; }

/**
 * Converts WGS-84 Geodetic point (lat, lon, h) to the
 * Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z).
 * Latitude and longitude are in degrees, height and the returned x, y, z in metres.
 */
inline Vector3d GeodeticToEcef(const Vector3d &meas) {
  double h = meas(2);

  double f = (a - b) / a;    // Ellipsoid Flatness
  double e_sq = f * (2 - f); // Square of Eccentricity

  double latitude_rad = DegreeToRadian(meas(0));
  double longitude_rad = DegreeToRadian(meas(1));
  double s = sin(latitude_rad);
  double N = a / sqrt(1 - e_sq * s * s);

  double sin_latitude = sin(latitude_rad);
  double cos_latitude = cos(latitude_rad);
  double cos_longitude = cos(longitude_rad);
  double sin_longitude = sin(longitude_rad);

  Vector3d xyz_ecef;
  xyz_ecef(0) = (h + N) * cos_latitude * cos_longitude;
  xyz_ecef(1) = (h + N) * cos_latitude * sin_longitude;
  xyz_ecef(2) = (h + (1 - e_sq) * N) * sin_latitude;
  return xyz_ecef;
}

/**
 * Converts the Earth-Centered Earth-Fixed (ECEF) coordinates (x, y, z) to
 * East-North-Up coordinates in a Local Tangent Plane that is centered at the
 * (WGS-84) Geodetic point (lat0, lon0, h0).
 * The datum is degrees, degrees, metres; everything else is metres.
 */
inline Vector3d EcefToEnu(const Vector3d &xyz_ecef, const Vector3d &datum) {

  double latitude_rad = DegreeToRadian(datum(0));
  double longitude_rad = DegreeToRadian(datum(1));

  double sin_latitude = sin(latitude_rad);
  double cos_latitude = cos(latitude_rad);
  double cos_longitude = cos(longitude_rad);
  double sin_longitude = sin(longitude_rad);

  Vector3d datum_ecef = GeodeticToEcef(datum);
  double xd = xyz_ecef(0) - datum_ecef(0);
  double yd = xyz_ecef(1) - datum_ecef(1);
  double zd = xyz_ecef(2) - datum_ecef(2);

  // This is the matrix multiplication
  Vector3d xyz_enu;
  xyz_enu(0) = -sin_longitude * xd + cos_longitude * yd;
  xyz_enu(1) = -cos_longitude * sin_latitude * xd - sin_latitude * sin_longitude * yd + cos_latitude * zd;
  xyz_enu(2) = cos_latitude * cos_longitude * xd + cos_latitude * sin_longitude * yd + sin_latitude * zd;
  return xyz_enu;
}

/**
 * Converts the geodetic WGS-84 coordinated (lat, lon, h) to
 * East-North-Up coordinates in a Local Tangent Plane that is centered at the
 * (WGS-84) Geodetic point (lat0, lon0, h0).
 * Both arguments are degrees, degrees, metres; the result is metres.
 */
inline Vector3d GeodeticToEnu(const Vector3d &meas, const Vector3d &datum) { return EcefToEnu(GeodeticToEcef(meas), datum); }

// Solve the 2-D quadratically constrained least squares min ||A w - b|| subject to ||w|| = 1.
//
// Every 2x2 block of A is a scaled rotation, so A^T A is a multiple of the identity and on the
// unit circle the residual reduces to a constant minus 2 (A^T b) . w. The minimizer is then the
// direction of A^T b, with no multiplier to search for.
inline bool solve_QCQP(const Matrix<double, -1, 2> &A, const Matrix<double, -1, 1> &b, Vector2d &w) {

  Vector2d ATb = A.transpose() * b;

  // Degenerate input leaves every direction equally good, and a NaN norm fails this test too
  w << 1, 0;
  if (!(ATb.norm() > 1e-12)) {
    return false;
  }

  w = ATb.normalized();
  return true;
}

// Get the 1 dof yaw rotation between z-aligned frames A and B based on point correspondences.
// Asserts the two sets are the same size and hold more than one point, so every caller below
// inherits that precondition.
inline bool compute_RBtoA1Dof(Matrix3d &R_BtoA, const vector<Vector3d> &p_inA, const vector<Vector3d> &p_inB) {

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

// Get the full four dof transformation between z-aligned frames A and B based on point correspondences
inline bool compute_4Dof(const vector<Vector3d> &p_inA, const vector<Vector3d> &p_inB, Matrix3d &R_BtoA, Vector3d &p_BinA) {
  bool found_solution = compute_RBtoA1Dof(R_BtoA, p_inA, p_inB);

  p_BinA.setZero();
  double M = p_inA.size();

  for (size_t i = 0; i < p_inA.size(); i++) {
    p_BinA += (1.0 / M) * (p_inA[i] - R_BtoA * p_inB[i]);
  }

  return found_solution;
}

// Flag which correspondences the given transform explains to within the threshold
inline vector<bool> find_inliers(const vector<Vector3d> &p_A, const vector<Vector3d> &p_B, const Matrix3d &R_BtoA, const Vector3d &p_BinA, double inlierThresh) {
  vector<bool> inliers(p_A.size());
  for (size_t i = 0; i < p_A.size(); i++) {
    inliers[i] = (p_A[i] - (R_BtoA * p_B[i] + p_BinA)).norm() <= inlierThresh;
  }
  return inliers;
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
inline bool Align_4Dof(const vector<Vector3d> &p_A, const vector<Vector3d> &p_B, Matrix3d &R_BtoA, Vector3d &p_BinA, double inlierThresh) {
  if (!compute_4Dof(p_A, p_B, R_BtoA, p_BinA)) {
    return false;
  }
  vector<bool> inliers = find_inliers(p_A, p_B, R_BtoA, p_BinA, inlierThresh);
  return count(inliers.begin(), inliers.end(), true) > 0;
}

/**
 * \brief Fits the 4-DOF transform between two z-aligned frames, leaving out the
 * correspondences that disagree with it.
 *
 * Each hypothesis is fit to two random correspondences, the fewest that fix the yaw. The one
 * that explains the most correspondences within the threshold is refit on just those. The
 * generator is seeded the same way on every call, so the result is repeatable.
 *
 * \param[in] p_A Points expressed in frame A.
 * \param[in] p_B The same points expressed in frame B.
 * \param[out] R_BtoA Recovered rotation, about z only.
 * \param[out] p_BinA Recovered translation.
 * \param[out] inliers Whether each correspondence is within the threshold of the returned fit.
 * \param[in] numHypotheses How many random pairs to fit.
 * \param[in] inlierThresh Largest residual, in metres, that still counts as an inlier.
 * \return False unless more than half of the correspondences are inliers. The outputs are only
 *         meaningful when this returns true.
 */
inline bool Ransac_4Dof(const vector<Vector3d> &p_A, const vector<Vector3d> &p_B, Matrix3d &R_BtoA, Vector3d &p_BinA, vector<bool> &inliers, size_t numHypotheses,
                        double inlierThresh) {
  if (p_A.size() < 2) {
    return false;
  }
  mt19937 rng(1337);
  uniform_int_distribution<size_t> pick(0, p_A.size() - 1);
  size_t most_inliers = 0;
  for (size_t h = 0; h < numHypotheses; h++) {
    size_t i = pick(rng), j = pick(rng);
    Matrix3d R_hyp;
    Vector3d p_hyp;
    if (i == j || !compute_4Dof({p_A[i], p_A[j]}, {p_B[i], p_B[j]}, R_hyp, p_hyp)) {
      continue;
    }
    vector<bool> inliers_hyp = find_inliers(p_A, p_B, R_hyp, p_hyp, inlierThresh);
    size_t count_hyp = count(inliers_hyp.begin(), inliers_hyp.end(), true);
    if (count_hyp > most_inliers) {
      most_inliers = count_hyp;
      inliers = inliers_hyp;
    }
  }
  if (2 * most_inliers <= p_A.size()) {
    return false;
  }

  // Refit on the inliers alone, then flag them again against the fit that is returned
  vector<Vector3d> inA, inB;
  for (size_t i = 0; i < p_A.size(); i++) {
    if (inliers[i]) {
      inA.push_back(p_A[i]);
      inB.push_back(p_B[i]);
    }
  }
  if (!compute_4Dof(inA, inB, R_BtoA, p_BinA)) {
    return false;
  }
  inliers = find_inliers(p_A, p_B, R_BtoA, p_BinA, inlierThresh);
  size_t count_final = count(inliers.begin(), inliers.end(), true);
  return 2 * count_final > p_A.size();
}

inline Matrix4d Left_q(const Vector4d &q) {
  Matrix4d L;
  L.block(0, 0, 3, 3) = q(3, 0) * Matrix3d::Identity() - skew_x(q.block(0, 0, 3, 1));
  L.block(0, 3, 4, 1) = q;
  L.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();

  return L;
}

inline Matrix4d Right_q(const Vector4d &q) {
  Matrix4d R;
  R.block(0, 0, 3, 3) = q(3, 0) * Matrix3d::Identity() + skew_x(q.block(0, 0, 3, 1));
  R.block(0, 3, 4, 1) = q;
  R.block(3, 0, 1, 3) = -q.block(0, 0, 3, 1).transpose();

  return R;
}

} // namespace gps_math

} // namespace mins

#endif // GPSCONVERSION_H

/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef GPSCONVERSION_H
#define GPSCONVERSION_H

#include "utils/quat_ops.h"
#include <Eigen/Eigen>
#include <cmath>
#include <complex>
#include <numeric>

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

  /// compute 4Dof ransac
  static bool Ransac_4Dof(vector<Vector3d> &p_A, vector<Vector3d> &p_B, Matrix3d &R_BtoA, Vector3d &p_BinA, size_t numHypotheses, double inlierThresh) {
    size_t sizeSubset = p_A.size();
    vector<Vector3d> suBp_A, suBp_B;
    size_t numInliers = 0;
    Matrix3d R_BtoAhyp;
    Vector3d p_BinAhyp;
    vector<unsigned int> inliershyp;

    bool found_solution = false;
    for (size_t i = 0; i < numHypotheses; i++) {
      // Generate hypothesis
      suBp_A.clear();
      suBp_B.clear();
      inliershyp.clear();
      // Get minimal solution
      get_random_subset(p_A, p_B, suBp_A, suBp_B, sizeSubset);
      found_solution = compute_4Dof(suBp_A, suBp_B, R_BtoAhyp, p_BinAhyp);
      for (size_t m = 0; m < p_A.size(); m++) {
        Vector3d p_err = p_A[m] - (R_BtoAhyp * p_B[m] + p_BinAhyp);
        if (p_err.norm() <= inlierThresh) {
          inliershyp.push_back(m);
        }
      }
      if (inliershyp.size() > numInliers) {
        R_BtoA = R_BtoAhyp;
        p_BinA = p_BinAhyp;
        numInliers = inliershyp.size();
      }
    }
    return found_solution;
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
  static void get_random_subset(vector<Vector3d> &p_A, vector<Vector3d> &p_B, vector<Vector3d> &suBp_A, vector<Vector3d> &suBp_B, size_t sizeSubset) {

    srand(1337);
    vector<unsigned int> indices(p_A.size());
    iota(indices.begin(), indices.end(), 0);
    random_shuffle(indices.begin(), indices.end());
    for (size_t i = 0; i < sizeSubset; i++) {
      suBp_A.push_back(p_A[indices[i]]);
      suBp_B.push_back(p_B[indices[i]]);
    }
  }

  // Get the full four dof transformation between z-aligned frames A and B based on point correspondences
  static bool compute_4Dof(vector<Vector3d> &p_inA, vector<Vector3d> &p_inB, Matrix3d &R_BtoA, Vector3d &p_BinA) {
    bool found_solution = compute_RBtoA1Dof(R_BtoA, p_inA, p_inB);

    p_BinA.setZero();
    double M = p_inA.size();

    for (size_t i = 0; i < p_inA.size(); i++) {
      p_BinA += (1.0 / M) * (p_inA[i] - R_BtoA * p_inB[i]);
    }

    return found_solution;
  }

  // Get the 1 dof yaw rotation between z-aligned frames A and B based on point correspondences
  static bool compute_RBtoA1Dof(Matrix3d &R_BtoA, vector<Vector3d> &p_inA, vector<Vector3d> &p_inB) {

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
    bool found_solution = solve_QCQP(A, b, w, p_inA, p_inB);

    R_BtoA.setIdentity();
    R_BtoA.block(0, 0, 2, 1) = w;
    R_BtoA(0, 1) = -w(1);
    R_BtoA(1, 1) = w(0);
    return found_solution;
  }

  // Solve a 2-D quadratically constrained quadratic program
  static bool solve_QCQP(Matrix<double, -1, 2> &A, Matrix<double, -1, 1> &b, Vector2d &w, vector<Vector3d> &p_inA, vector<Vector3d> &p_inB) {

    // Compute the coefficients for the quartic polynomial
    Matrix2d ATA = A.transpose() * A;

    double a_11 = ATA(0, 0);
    double a_12 = ATA(0, 1);
    double a_22 = ATA(1, 1);

    Matrix<double, 1, 2> bA = b.transpose() * A;

    double bA_1 = bA(0);
    double bA_2 = bA(1);

    double c_0 = -pow(-a_12 * a_12 + a_11 * a_22, 2) - bA_2 * (bA_1 * (a_11 * a_12 + a_12 * a_22) - bA_2 * (a_11 * a_11 + a_12 * a_12)) -
                 bA_1 * (bA_2 * (a_11 * a_12 + a_12 * a_22) - bA_1 * (a_11 * a_11 + a_12 * a_12));
    double c_1 = bA_2 * (2 * a_11 * bA_2 - 2 * a_12 * bA_1) - 2 * (a_11 + a_22) * (-a_12 * a_12 + a_11 * a_22) - bA_1 * (2 * a_12 * bA_2 - 2 * a_22 * bA_1);

    double c_2 = 2 * a_12 * a_12 - (a_11 + a_22) * (a_11 + a_22) - 2 * a_11 * a_22 + bA_1 * bA_1 + bA_2 * bA_2;

    double c_3 = -2 * a_11 - 2 * a_22;
    double c_4 = -1;

    complex<double> coefficients[5];
    coefficients[0] = complex<double>(c_0);
    coefficients[1] = complex<double>(c_1);
    coefficients[2] = complex<double>(c_2);
    coefficients[3] = complex<double>(c_3);
    coefficients[4] = complex<double>(c_4);

    complex<double> roots[4];

    // Based on these quartic coeffs, find each of the possible 4 roots
    solve_quartic(coefficients, roots);

    double optimal_lam = roots[0].real();
    bool found_good_lam = false;

    double Best_cost = INFINITY;
    double err = 0;
    // Find the purely real root
    for (auto &root : roots) {
      if (abs(root.imag()) <= 1e-6) {
        optimal_lam = root.real();
        found_good_lam = true;
      }
      Matrix2d ATAl = ATA;
      ATAl(0, 0) += optimal_lam;
      ATAl(1, 1) += optimal_lam;

      Vector2d w_hyp = ATAl.llt().solve(bA.transpose());

      Matrix3d R_BtoA;
      R_BtoA.setIdentity();
      R_BtoA.block(0, 0, 2, 1) = w_hyp;
      R_BtoA(0, 1) = -w_hyp(1);
      R_BtoA(1, 1) = w_hyp(0);

      double M = p_inA.size();
      Vector3d p_BinA;
      p_BinA.setZero();
      for (size_t i = 0; i < p_inA.size(); i++) {
        p_BinA += (1.0 / M) * (p_inA[i] - R_BtoA * p_inB[i]);
      }

      err = (A * w_hyp - b).norm();
      if (err < Best_cost) {
        w = w_hyp;
        Best_cost = err;
      }
    }
    // We have to have found the proper root
    return found_good_lam;
  }

  static void solve_quartic(const complex<double> coefficients[5], complex<double> roots[4]) {
    // The algorithm below was derived by solving the quartic in Mathematica, and simplifying the resulting expression by hand.
    // Quartic solver from https://github.com/sidneycadot/quartic

    const complex<double> a = coefficients[4];
    const complex<double> b = coefficients[3] / a;
    const complex<double> c = coefficients[2] / a;
    const complex<double> d = coefficients[1] / a;
    const complex<double> e = coefficients[0] / a;

    const complex<double> Q1 = c * c - 3. * b * d + 12. * e;
    const complex<double> Q2 = 2. * c * c * c - 9. * b * c * d + 27. * d * d + 27. * b * b * e - 72. * c * e;
    const complex<double> Q3 = 8. * b * c - 16. * d - 2. * b * b * b;
    const complex<double> Q4 = 3. * b * b - 8. * c;
    const complex<double> Q5 = complex_cbrt(Q2 / 2. + complex_sqrt(Q2 * Q2 / 4. - Q1 * Q1 * Q1));
    const complex<double> Q6 = (Q1 / Q5 + Q5) / 3.;
    const complex<double> Q7 = 2. * complex_sqrt(Q4 / 12. + Q6);

    roots[0] = (-b - Q7 - complex_sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)) / 4.;
    roots[1] = (-b - Q7 + complex_sqrt(4. * Q4 / 6. - 4. * Q6 - Q3 / Q7)) / 4.;
    roots[2] = (-b + Q7 - complex_sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)) / 4.;
    roots[3] = (-b + Q7 + complex_sqrt(4. * Q4 / 6. - 4. * Q6 + Q3 / Q7)) / 4.;
  }

  static inline complex<double> complex_sqrt(const complex<double> &z) { return pow(z, 1. / 2.); }

  static inline complex<double> complex_cbrt(const complex<double> &z) { return pow(z, 1. / 3.); }
};

#endif // GPSCONVERSION_H
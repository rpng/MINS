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

#ifndef MINS_PACKAGE_JABDONGSANI_H
#define MINS_PACKAGE_JABDONGSANI_H

#include "memory"
#include "vector"
#include <Eigen/Core>

using namespace std;
using namespace Eigen;
namespace ov_type {
class Type;
}
namespace mins {

/// odds and ends. Mostly used to analyze matrix
class JDSN {
public:
  static void print(MatrixXd m);
  static void print(MatrixXd m, string name);
  static void print(string name, MatrixXd m);
  static void print(MatrixXd m, string name, int precision);
  static void print(string name, MatrixXd m, int precision);
  static void print(vector<int> vec, string name);
  static void print(string name, vector<int> vec);
  static void size(MatrixXd m, string name);
  static void size(string name, MatrixXd m);
  static bool symmetric(MatrixXd m, bool verbose = false);
};

struct Dummy {
  Matrix3d R;
  Vector3d p;
  vector<MatrixXd> VM;
  vector<shared_ptr<ov_type::Type>> VT;
};

/// Return mean and standard deviation with gien vector (+ max norm difference)
static void GetStatistics(vector<double> vec, double &mean, double &std);
static void GetStatistics(vector<float> vec, double &mean, double &std);
static void GetStatistics(vector<VectorXd> vec, VectorXd &mean, MatrixXd &var, VectorXd &max);
static void GetStatistics(vector<VectorXd> vec, double &mean, double &var, double &max); // returns norm statistics

/// computes the statistics of values (residual, chi, etc) incrementally.
struct STAT {
  /// Mean value
  float mean = 0;

  /// Variance of the value
  float var = 0;

  /// Counter
  float cnt = 0;

  /// Add object. Will be added scalar form
  void add_stat(const float &val);
  void add_stat(const VectorXd &vec);
  void add_stat(const MatrixXd &mat);

  /// Returns standard deviation
  float std();

  /// Reset
  void reset();
};

} // namespace mins

#endif // MINS_PACKAGE_JABDONGSANI_H

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

#include "Jabdongsani.h"
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace mins;

void JDSN::print(string name, MatrixXd m) { print(m, name); }
void JDSN::print(MatrixXd m, string name) { m.cols() == 1 ? cout << name << ": " << m.transpose() << endl : cout << name << endl << m << endl; }

void JDSN::print(string name, MatrixXd m, int precision) { print(m, name, precision); }
void JDSN::print(MatrixXd m, string name, int precision) {
  // print in transposed way if this is a vector
  if (m.cols() == 1 && m.rows() != 1) {
    print(m.transpose(), name, precision);
    return;
  }

  size(m, name);
  // Do scientific notation if precision set <= 0
  if (precision <= 0) {
    // Print matrix
    for (int i = 0; i < m.rows(); i++) {
      for (int j = 0; j < m.cols(); j++) {
        if (m(i, j) == (int)m(i, j)) {
          printf("     %d ", (int)m(i, j));
        } else {
          printf("%s", m(i, j) < 0 ? "" : " ");
          printf("%.0e ", m(i, j));
        }
      }
      printf("\n");
    }
    printf("\n");
    return;
  }

  // find max digit if column for aligned column
  vector<int> max_digit(m.cols(), 0);
  for (int i = 0; i < m.cols(); i++) {
    for (int j = 0; j < m.rows(); j++) {
      max_digit[i] < log10(abs(m(j, i))) ? max_digit[i] = (int)log10(abs(m(j, i))) : int();
    }
  }

  // Print matrix
  for (int i = 0; i < m.rows(); i++) {
    for (int j = 0; j < m.cols(); j++) {
      for (int k = 0; k < max_digit[j] - max((int)log10(abs(m(i, j))), 0); k++)
        printf(" ");
      printf("%s%.*f ", (signbit(m(i, j)) ? "" : " "), precision, (signbit(m(i, j)) ? m(i, j) : abs(m(i, j))));
    }
    printf("\n");
  }
  printf("\n");
}

void JDSN::print(MatrixXd m) { m.cols() == 1 ? cout << m.transpose() << endl : cout << m << endl; }
void JDSN::print(string name, vector<int> vec) { print(vec, name); }
void JDSN::print(vector<int> vec, string name) {
  cout << name << endl;
  for (auto v : vec)
    cout << v << endl;
}

void JDSN::size(MatrixXd m, string name) { cout << name << ": " << m.rows() << " x " << m.cols() << endl; }
void JDSN::size(string name, MatrixXd m) { size(m, name); }

bool JDSN::symmetric(MatrixXd m, bool verbose) {
  // check square
  if (m.rows() != m.cols())
    return false;

  // Check off diagonal
  for (int i = 0; i < m.rows(); i++) {
    for (int j = 0; j < i; j++) {
      if (m(i, j) != m(j, i)) {
        if (verbose) {
          printf("Symmetry check failed at %.20f(%d,%d) != %.20f(%d,%d)\n", m(i, j), i, j, m(j, i), j, i);
        }
        return false;
      }
    }
  }

  // This is symmetric matrix
  return true;
}

void GetStatistics(vector<double> vec, double &mean, double &std) {
  float sum = 0.0, variance = 0.0;
  unsigned int i;
  for (i = 0; i < vec.size(); ++i)
    sum += vec[i];
  mean = sum / vec.size();
  for (i = 0; i < vec.size(); ++i)
    variance += pow(vec[i] - mean, 2);
  variance = variance / vec.size();
  std = sqrt(variance);
}

void GetStatistics(vector<float> vec, double &mean, double &std) {
  float sum = 0.0, variance = 0.0;
  unsigned int i;
  for (i = 0; i < vec.size(); ++i)
    sum += vec[i];
  mean = sum / vec.size();
  for (i = 0; i < vec.size(); ++i)
    variance += pow(vec[i] - mean, 2);
  variance = variance / vec.size();
  std = sqrt(variance);
}

void GetStatistics(vector<VectorXd> vec, VectorXd &mean, MatrixXd &var, VectorXd &max) {
  VectorXd sum = VectorXd::Zero(vec[0].rows());
  max = VectorXd::Zero(vec[0].rows());
  var = MatrixXd::Zero(vec[0].rows(), vec[0].rows());
  unsigned int i;
  for (i = 0; i < vec.size(); ++i) {
    sum += vec[i];
    if (max.norm() < vec[i].norm())
      max = vec[i];
  }
  mean = sum / vec.size();
  for (i = 0; i < vec.size(); ++i)
    var += (vec[i] - mean) * (vec[i] - mean).transpose();
  var = var / vec.size();
}

void GetStatistics(vector<VectorXd> vec, double &mean, double &std, double &max) {
  mean = 0;
  double var = 0;
  max = 0;
  unsigned int i;
  for (i = 0; i < vec.size(); ++i) {
    mean += vec[i].norm() / vec.size();
    if (max < vec[i].norm())
      max = vec[i].norm();
  }

  for (i = 0; i < vec.size(); ++i)
    var += pow(vec[i].norm() - mean, 2) / vec.size();
  std = sqrt(var);
}

void STAT::add_stat(const float &val) {
  cnt++;
  // compute only if we have more than 1 samples
  if (cnt > 1)
    var = (cnt - 2) / (cnt - 1) * var + 1.0 / cnt * pow((val - mean), 2);
  mean = (val + (cnt - 1) * mean) / cnt;
}

void STAT::add_stat(const VectorXd &vec) {
  for (int i = 0; i < (int)vec.size(); i++)
    add_stat(vec[i]);
}

void STAT::add_stat(const MatrixXd &mat) {
  int min_dim = mat.rows() < mat.cols() ? mat.rows() : mat.cols();
  for (int i = 0; i < min_dim; i++)
    add_stat(mat(i, i));
}

float STAT::std() { return sqrt(var); }

void STAT::reset() {
  mean = 0;
  var = 0;
  cnt = 0;
}

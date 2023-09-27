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

#ifndef MINS_UPDATERSTATISTICS
#define MINS_UPDATERSTATISTICS

#include <Eigen/Eigen>
#include <memory>

using namespace std;
using namespace Eigen;
namespace ov_type {
class Type;
}
namespace mins {
class State;
struct STAT;
typedef shared_ptr<State> StatePtr;
typedef vector<shared_ptr<ov_type::Type>> VecTypePtr;

class UpdaterStatistics {

public:
  /// Updater statistics. Perform chi2 check
  UpdaterStatistics(double chi2_mult, string type, int id = -1);

  /// Perform Chi2 test
  bool Chi2Check(StatePtr state, const VecTypePtr &H_order, const MatrixXd &H, const VectorXd &res, bool print = false);
  bool Chi2Check(StatePtr state, const VecTypePtr &H_order, const MatrixXd &H, const VectorXd &res, const MatrixXd &R, bool print = false);
  bool Chi2Check(const MatrixXd &P, const MatrixXd &H, const VectorXd &res, const MatrixXd &R, bool print = false);

  /// Print current statistics of the sensor
  void print();

private:
  /// Compute Chi2 value of given residual
  bool get_chi2(double &chi, double &thr, const MatrixXd &P, const MatrixXd &H, const VectorXd &res, const MatrixXd &R);

  /// Sensor type of this sensor
  string type;

  /// Sensor ID
  int id = -1;

  /// Chi coefficient
  double chi2_mult = 1;

  /// counters for total, rejected, and accepted measurements
  int n_rej = 0;
  int n_acp = 0;
  int n_meas = 0;

  /// Chi squared 95th percentile table
  map<int, double> chi_squared_table;
  int max_chi_size = 1000;

  /// Status of chi, res, and std value fed in
  shared_ptr<STAT> chi_stat;
  shared_ptr<STAT> res_stat;
  shared_ptr<STAT> std_stat;
};
} // namespace mins

#endif // MINS_UPDATERSTATISTICS

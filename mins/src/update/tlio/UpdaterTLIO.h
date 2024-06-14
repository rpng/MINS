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

#ifndef MINS_UPDATERODOM_H
#define MINS_UPDATERODOM_H

#include <deque>
#include <map>
#include <memory>
#include <vector>

using namespace std;

namespace mins {
class State;
class UpdaterStatistics;
struct TLIOData;

class UpdaterTLIO {

public:
  /// Odom updater
  UpdaterTLIO(shared_ptr<State> state);

  /// feed odom measurement
  void feed_measurement(const TLIOData &data);

  /// try update
  void try_update();

  /// chi stat
  std::shared_ptr<UpdaterStatistics> Chi;

  /// measurement time history
  deque<double> t_hist;

protected:
  /// perform update
  bool update(TLIOData m);

  /// stack of measurements
  std::vector<TLIOData> data_stack;

  /// state
  shared_ptr<State> state;
};

} // namespace mins

#endif // MINS_UPDATERODOM_H

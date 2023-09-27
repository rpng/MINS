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

#ifndef MINS_TIMECHECKER_H
#define MINS_TIMECHECKER_H

#include "boost/date_time/posix_time/posix_time.hpp"
#include "utils/Print_Logger.h"
#include <map>
#include <string>
#include <vector>

using namespace std;

namespace mins {

class TimeChecker {
public:
  /// Time Checker. Measures the average time between ding and dong
  TimeChecker() {}

  /// Init with name
  TimeChecker(string name) { names.insert({name, record()}); }

  /// Define prefix increment operator. This allows timechecker++ operation
  TimeChecker &operator++() {
    counter++;
    return *this;
  }

  /// Define postfix increment operator. This allows timechecker++ operation
  TimeChecker operator++(int) {
    TimeChecker temp = *this;
    (*this).counter++;
    return temp;
  }

  /// Struct carry time ding dong information
  struct record {
    boost::posix_time::ptime ding_t;
    boost::posix_time::ptime dong_t;
    bool has_prev_ding = false;
    double total_t = 0;
    double max_t = -INFINITY;
  };

  /// ding: start recording time of given name
  void ding(string name) {
    // if cannot find this name, append to the map
    if (names.find(name) == names.end())
      names.insert({name, record()});

    names.at(name).ding_t = boost::posix_time::microsec_clock::local_time();
    names.at(name).has_prev_ding = true;
  }

  /// ding: start recording time of an unique name already given
  void ding() {
    // Should have a name
    assert(names.size() == 1);
    names.begin()->second.ding_t = boost::posix_time::microsec_clock::local_time();
    names.begin()->second.has_prev_ding = true;
  }

  /// dong: end recording time of given name
  void dong(string name) {
    if (names.find(name) == names.end() || !names.at(name).has_prev_ding) {
      PRINT3("[TimeChecker] Cannot dong because no ding for this name(%s) setup!\n", name.c_str());
      return;
    }
    names.at(name).dong_t = boost::posix_time::microsec_clock::local_time();
    double dt = (double)((names.at(name).dong_t - names.at(name).ding_t).total_microseconds() * 1e-6);
    names.at(name).total_t += dt;
    if (dt > names.at(name).max_t)
      names.at(name).max_t = dt;
    names.at(name).has_prev_ding = false;
  }

  /// dong: end recording time of an unique name already given
  void dong() {
    // Should have a name
    assert(names.size() == 1);
    names.begin()->second.dong_t = boost::posix_time::microsec_clock::local_time();
    double dt = (double)((names.begin()->second.dong_t - names.begin()->second.ding_t).total_microseconds() * 1e-6);
    names.begin()->second.total_t += dt;
    if (dt > names.begin()->second.max_t)
      names.begin()->second.max_t = dt;
    names.begin()->second.has_prev_ding = false;
  }

  /// Lazy and combined version of ding-dong. This alters ding and dong
  void dingdong(string name) {
    // if cannot find this name, append to the map
    if (names.find(name) == names.end()) {
      names.insert({name, record()});
      names.at(name).ding_t = boost::posix_time::microsec_clock::local_time();
      names.at(name).has_prev_ding = true;
      return;
    }

    // Do ding if we dont have prev ding
    if (!names.at(name).has_prev_ding) {
      names.at(name).ding_t = boost::posix_time::microsec_clock::local_time();
      names.at(name).has_prev_ding = true;
      return;
    }

    // Do dong if we have prev ding
    if (names.at(name).has_prev_ding) {
      names.at(name).dong_t = boost::posix_time::microsec_clock::local_time();
      double dt = (double)((names.at(name).dong_t - names.at(name).ding_t).total_microseconds() * 1e-6);
      names.at(name).total_t += dt;
      if (dt > names.at(name).max_t)
        names.at(name).max_t = dt;
      names.at(name).has_prev_ding = false;
      return;
    }
  }

  /// ding-dong of an unique name previously defined
  void dingdong() {
    // Should have a name
    assert(names.size() == 1);
    string name = names.begin()->first;
    dingdong(name);
  }

  /// print timing analysis of given name
  void print(string name, bool in_sec = false, bool verbose = false) {
    if (names.find(name) == names.end()) {
      PRINT3("[TimeChecker] Cannot print time of this name(%s) because it never recorded!\n", name.c_str());
      return;
    }
    if (counter == 0) {
      PRINT1("[Timechecker] %s requested to print with counter 0!. Set 1.\n", name.c_str());
      counter++;
    }

    if (verbose) {
      if (in_sec) {
        PRINT2("%s: %.1fs (max %.1fs)\n", name.c_str(), names.at(name).total_t / counter, names.at(name).max_t);
      } else {
        PRINT2("%s: %dms (max %dms)\n", name.c_str(), (int)(names.at(name).total_t / counter * 1000), (int)(names.at(name).max_t * 1000));
      }
    } else {
      if (in_sec) {
        PRINT2("%s: %.1fs\n", name.c_str(), names.at(name).total_t / counter);
      } else {
        PRINT2("%s: %d ms\n", name.c_str(), (int)(names.at(name).total_t / counter * 1000));
      }
    }
  }

  /// print timing analysis of an unique name
  void print(bool in_sec = false, bool verbose = false) {
    // Should have a name
    assert(names.size() == 1);
    if (counter == 0) {
      PRINT1("[Timechecker] %s requested to print with counter 0!. Set 1.\n", names.begin()->first.c_str());
      counter++;
    }
    string name = names.begin()->first;
    if (verbose) {
      if (in_sec) {
        PRINT2("%s: %.1fs (max %.1fs)\n", name.c_str(), names.at(name).total_t / counter, names.at(name).max_t);
      } else {
        PRINT2("%s: %dms (max %dms)\n", name.c_str(), (int)(names.at(name).total_t / counter * 1000), (int)(names.at(name).max_t * 1000));
      }
    } else {
      if (in_sec) {
        PRINT2("%s: %.1fs\n", name.c_str(), names.at(name).total_t / counter);
      } else {
        PRINT2("%s: %d ms\n", name.c_str(), (int)(names.at(name).total_t / counter * 1000));
      }
    }
  }

  /// print total time the name
  void print_total(string name, bool in_sec = false) {
    if (names.find(name) == names.end()) {
      PRINT3("[TimeChecker] Cannot print total time of this name(%s) because it never recorded!\n", name.c_str());
      return;
    }

    if (in_sec) {
      PRINT2("%s: %.1fs\n", name.c_str(), names.at(name).total_t);
    } else {
      PRINT2("%s: %dms\n", name.c_str(), (int)(names.at(name).total_t * 1000));
    }
  }

  /// return total time of all names
  double get_total_sum(bool in_sec = true) {
    double total_sum = 0;
    for (auto name : names)
      total_sum += name.second.total_t;

    return in_sec ? total_sum : (int)total_sum * 1000;
  }

  /// print timing analysis of all names have
  void print_all(bool sort_by_dont_t = false, bool in_sec = false, bool verbose = false) {
    if (sort_by_dont_t) {
      for (const auto &name : sort(names))
        print(name.first, in_sec, verbose);
    } else {
      for (const auto &name : names)
        print(name.first, in_sec, verbose);
    }
  }

  /// print timing analysis of all names have in one line
  void print_all_one_line(bool in_sec = false, bool print_total = false, double process_t = -1) {
    double total = 0;
    for (const auto &info : names) {
      if (in_sec) {
        PRINT2("%s: %.1fs ", info.first.c_str(), names.at(info.first).total_t / counter);
        total += names.at(info.first).total_t / counter;
      } else {
        PRINT2("%s: %dms ", info.first.c_str(), (int)(names.at(info.first).total_t / counter * 1000));
        total += names.at(info.first).total_t / counter * 1000;
      }
    }

    if (print_total) {
      in_sec ? PRINT2("Total: %.1fs", total) : PRINT2("Total: %dms", (int)total);
      process_t > 0 ? (in_sec ? PRINT2(" / %.1fs (%.1fX)", process_t, process_t / total) : PRINT2(" / %dms (%.1fX)", (int)process_t, process_t / total)) : PRINT2("");
    }
    PRINT2("\n");
  }

  /// Names of the record
  map<string, record> names;

  /// ding dong counter
  int counter = 0;

private:
  // https://www.geeksforgeeks.org/sorting-a-map-by-value-in-c-stl/
  // Comparator function to sort pairs
  // according to second value
  static bool cmp(pair<string, record> &a, pair<string, record> &b) { return a.second.dong_t < b.second.dong_t; }

  // Function to sort the map according
  // to value in a (key-value) pairs
  vector<pair<string, record>> sort(map<string, record> &M) {

    // Declare vector of pairs
    vector<pair<string, record>> A;

    // Copy key-value pair from Map
    // to vector of pairs
    for (auto &it : M) {
      A.emplace_back(it);
    }

    // Sort using comparator function
    std::sort(A.begin(), A.end(), cmp);

    return A;
  }
};

} // namespace mins
#endif // MINS_TIMECHECKER_H

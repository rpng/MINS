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

#ifndef MINS_TYPE_4DOFPOSEJPL_H
#define MINS_TYPE_4DOFPOSEJPL_H

#include "JPLQuat_4DOF.h"
#include "types/Vec.h"
#include "utils/quat_ops.h"

namespace ov_type {

class PoseJPL_4DOF : public Type {

public:
  PoseJPL_4DOF() : Type(4) {

    // Initialize subvariables
    _q = std::shared_ptr<JPLQuat_4DOF>(new JPLQuat_4DOF());
    _p = std::shared_ptr<Vec>(new Vec(3));

    Eigen::Matrix<double, 7, 1> pose0;
    pose0.setZero();
    pose0(3) = 1.0;
    set_value(pose0);
    set_fej(pose0);
  }

  ~PoseJPL_4DOF() {}

  /**
   * @brief Sets id used to track location of variable in the filter covariance
   *
   * Note that we update the sub-variables also.
   *
   * @param new_id entry in filter covariance corresponding to this variable
   */
  void set_local_id(int new_id) override {
    _id = new_id;
    _q->set_local_id(new_id);
    _p->set_local_id(new_id + _q->size());
  }

  /**
   * @brief Update q and p using a the JPLQuat update for orientation and vector update for position
   *
   * @param dx Correction vector (orientation then position)
   */
  void update(const Eigen::VectorXd &dx) override {

    assert(dx.rows() == _size);

    double dth = dx(0, 0);
    Eigen::Matrix<double, 3, 1> dth_3;
    dth_3 << 0, 0, dth;

    Eigen::Matrix<double, 7, 1> newX = _value;

    Eigen::Matrix<double, 4, 1> dq;
    dq << .5 * dth_3, 1.0;
    dq = ov_core::quatnorm(dq);

    // Update orientation
    newX.block(0, 0, 4, 1) = ov_core::quat_multiply(dq, quat());

    // Update position
    newX.block(4, 0, 3, 1) += dx.block(1, 0, 3, 1);

    set_value(newX);
  }

  /**
   * @brief Sets the value of the estimate
   * @param new_value New value we should set
   */
  void set_value(const Eigen::MatrixXd &new_value) override {

    assert(new_value.rows() == 7);
    assert(new_value.cols() == 1);

    // Set orientation value
    _q->set_value(new_value.block(0, 0, 4, 1));

    // Set position value
    _p->set_value(new_value.block(4, 0, 3, 1));

    _value = new_value;
  }

  /**
   * @brief Sets the value of the first estimate
   * @param new_value New value we should set
   */
  void set_fej(const Eigen::MatrixXd &new_value) override {

    assert(new_value.rows() == 7);
    assert(new_value.cols() == 1);

    // Set orientation fej value
    _q->set_fej(new_value.block(0, 0, 4, 1));

    // Set position fej value
    _p->set_fej(new_value.block(4, 0, 3, 1));

    _fej = new_value;
  }

  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<JPLQuat_4DOF>(new JPLQuat_4DOF());
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }

  /**
   * @brief Used to find the components inside the Pose if needed
   * If the passed variable is a sub-variable or the current variable this will return it.
   * Otherwise it will return a nullptr, meaning that it was unable to be found.
   *
   * @param check variable to find
   */
  std::shared_ptr<Type> check_if_subvariable(const std::shared_ptr<Type> check) override {
    if (check == _q) {
      return _q;
    } else if (check == _p) {
      return _p;
    }
    return nullptr;
  }

  /// Rotation access
  Eigen::Matrix<double, 3, 3> Rot() const { return _q->Rot(); }

  /// FEJ Rotation access
  Eigen::Matrix<double, 3, 3> Rot_fej() const {
    return _q->Rot_fej();
    ;
  }

  /// Rotation access as quaternion
  Eigen::Matrix<double, 4, 1> quat() const { return _q->value(); }

  /// FEJ Rotation access as quaternion
  Eigen::Matrix<double, 4, 1> quat_fej() const { return _q->fej(); }

  /// Position access
  Eigen::Matrix<double, 3, 1> pos() const { return _p->value(); }

  // FEJ position access
  Eigen::Matrix<double, 3, 1> pos_fej() const { return _p->fej(); }

  // Quaternion type access
  std::shared_ptr<JPLQuat_4DOF> q() { return _q; }

  // Position type access
  std::shared_ptr<Vec> p() { return _p; }

protected:
  /// Subvariable containing orientation
  std::shared_ptr<JPLQuat_4DOF> _q;

  /// Subvariable containg position
  std::shared_ptr<Vec> _p;
};

} // namespace ov_type

#endif // MINS_4DOFPOSEJPL_H

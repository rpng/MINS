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

#ifndef MINS_TYPE_JPLQUAT_4DOF_H
#define MINS_TYPE_JPLQUAT_4DOF_H

#include "types/JPLQuat.h"
#include "types/Type.h"
#include "utils/quat_ops.h"

namespace ov_type {

class JPLQuat_4DOF : public Type {

public:
  JPLQuat_4DOF() : Type(1) {
    Eigen::Vector4d q0 = Eigen::Vector4d::Zero();
    q0(3) = 1.0;
    set_value(q0);
    set_fej(q0);
  }

  ~JPLQuat_4DOF() {}

  /**
   * @brief Implements update operation by left-multiplying the current
   * quaternion with a quaternion built from a small axis-angle perturbation.
   *
   * @f[
   * \bar{q}=norm\Big(\begin{bmatrix} 0.5*\mathbf{\theta_{dx}} \\ 1 \end{bmatrix}\Big) \hat{\bar{q}}
   * @f]
   *
   * @param dx Axis-angle representation of the perturbing quaternion
   */
  void update(const Eigen::VectorXd &dx) override {

    assert(dx.rows() == _size);

    // Build perturbing quaternion
    Eigen::Matrix<double, 4, 1> dq;
    dq << .5 * dx, 1.0;
    dq = ov_core::quatnorm(dq);

    // Update estimate and recompute R
    set_value(ov_core::quat_multiply(dq, _value));
  }

  /**
   * @brief Sets the value of the estimate and recomputes the internal rotation matrix
   * @param new_value New value for the quaternion estimate
   */
  void set_value(const Eigen::MatrixXd &new_value) override {

    assert(new_value.rows() == 4);
    assert(new_value.cols() == 1);

    _value = new_value;

    // compute associated rotation
    _R = ov_core::quat_2_Rot(new_value);
  }

  std::shared_ptr<Type> clone() override {
    auto Clone = std::shared_ptr<JPLQuat>(new JPLQuat());
    Clone->set_value(value());
    Clone->set_fej(fej());
    return Clone;
  }

  /**
   * @brief Sets the fej value and recomputes the fej rotation matrix
   * @param new_value New value for the quaternion estimate
   */
  void set_fej(const Eigen::MatrixXd &new_value) override {

    assert(new_value.rows() == 4);
    assert(new_value.cols() == 1);

    _fej = new_value;

    // compute associated rotation
    _Rfej = ov_core::quat_2_Rot(new_value);
  }

  /// Rotation access
  Eigen::Matrix<double, 3, 3> Rot() const { return _R; }

  /// FEJ Rotation access
  Eigen::Matrix<double, 3, 3> Rot_fej() const { return _Rfej; }

protected:
  // Stores the rotation
  Eigen::Matrix<double, 3, 3> _R;

  // Stores the first-estimate rotation
  Eigen::Matrix<double, 3, 3> _Rfej;
};
} // namespace ov_type

#endif // MINS_TYPE_JPLQUAT_H

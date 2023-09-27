/*
 * MINS: Efficient and Robust Multisensor-aided Inertial Navigation System
 * Copyright (C) 2023 Woosik Lee
 * Copyright (C) 2023 Guoquan Huang
 * Copyright (C) 2023 MINS Contributors
 *
 * This code is implemented based on:
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
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

#include "CamTypes.h"
#include "utils/Print_Logger.h"

void mins::CamLinSys::print() {
  PRINT0("Hf\n");
  print_matrix(Hf);
  PRINT0("Hx\n");
  print_matrix(Hx);
  PRINT0("R\n");
  print_matrix(R);
  PRINT0("res\n");
  print_matrix(res);
}

void mins::CamLinSys::print_matrix(Eigen::MatrixXd H) {
  // if Print level is not 0 and we do not save prints, just return. It is heavy
  if (mins::Print_Logger::current_print_level != 0 && mins::Print_Logger::pFile == NULL)
    return;

  for (int i = 0; i < H.rows(); i++) {
    for (int j = 0; j < H.cols(); j++) {
      PRINT0("%.2f ", H(i, j));
      assert(!std::isnan(H(i, j)));
    }
    printf("\n");
  }
}

void mins::CamLinSys::print_matrix(Eigen::VectorXd r) {
  // if Print level is not 0 and we do not save prints, just return. It is heavy
  if (mins::Print_Logger::current_print_level != 0 && mins::Print_Logger::pFile == NULL)
    return;

  for (int i = 0; i < r.rows(); i++) {
    PRINT0("%.2f ", r(i));
    assert(!std::isnan(r(i)));
  }
  printf("\n");
}
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

#ifndef MINS_SIMULATIONPLANE_H
#define MINS_SIMULATIONPLANE_H

#include <Eigen/Eigen>

using namespace Eigen;
namespace mins {

class SimulationPlane {

public:
  /**
   * @brief Default constructor
   * @param _pt_top_left Top left point
   * @param _pt_top_right Top right point
   * @param _pt_bottom_left Bottom left point (line start point in floorplan)
   * @param _pt_bottom_right Bottom right point (line end point in floorplan)
   */
  SimulationPlane(Vector3d &_pt_top_left, Vector3d &_pt_top_right, Vector3d &_pt_bottom_left, Vector3d &_pt_bottom_right)
      : pt_top_left(_pt_top_left), pt_top_right(_pt_top_right), pt_bottom_left(_pt_bottom_left), pt_bottom_right(_pt_bottom_right) {
    Vector3d V1 = pt_top_right - pt_top_left;
    Vector3d V2 = pt_bottom_left - pt_top_left;
    Vector3d N = V1.cross(V2);

    A = N(0);
    B = N(1);
    C = N(2);
    D = -(A * pt_top_left(0) + B * pt_top_left(1) + C * pt_top_left(2));
  }

  /**
   * @brief This will try to intersect the given ray and this plane.
   *
   * This assumes that the ray and the plane are in the same frame of reference.
   * This will return true if it is a hit, and false otherwise.
   * Given a plane in the form Ax+By+Cz+D=0 we can first solve for the intersection.
   * R(t) = R0 + Rd*t
   * A(x0 + xd*t) + B(y0 + yd*t) + (z0 + zd*t) + D = 0
   * We can inverse the above function to get the distance along this ray bearing
   * t = -(A*x0 + B*y0 + C*z0 + D) / (A*xd + B*yd + C*zd)
   *
   * @param ray Bearing ray in the frame this plane is represented in [ray_origin, ray_bearing]
   * @param point_intersection Scalar distance along the ray that makes the point lie on this plane
   * @return True if we found an intersection
   */
  bool calculate_intersection(Matrix<double, 6, 1> &ray, double &point_intersection) {

    // Intersect the ray with our plane
    point_intersection = -(A * ray(0) + B * ray(1) + C * ray(2) + D) / (A * ray(3) + B * ray(4) + C * ray(5));

    // Calculate the actual intersection 3d point
    Vector3d pt_inter = ray.head(3) + point_intersection * ray.tail(3);

    // Check the result
    Vector3d V1 = pt_top_right - pt_top_left;
    V1.normalize();
    Vector3d V2 = pt_bottom_left - pt_top_left;
    V2.normalize();
    Vector3d V3 = pt_top_right - pt_bottom_right;
    V3.normalize();
    Vector3d V4 = pt_bottom_left - pt_bottom_right;
    V4.normalize();
    Vector3d U1 = pt_inter - pt_top_left;
    U1.normalize();
    Vector3d U2 = pt_inter - pt_bottom_right;
    U2.normalize();

    return (point_intersection > 0 && U1.dot(V1) > 0 && U1.dot(V2) > 0 && U2.dot(V3) > 0 && U2.dot(V4) > 0);
  }

  double point_to_plane_distance(Vector3d p) { return abs(A * p(0) + B * p(1) + C * p(2) + D) / sqrt(A * A + B * B + C * C); }

  // Our top-left plane point
  Vector3d pt_top_left;

  // Our top-right plane point
  Vector3d pt_top_right;

  // Our top-bottom plane point
  Vector3d pt_bottom_left;

  // Our top-bottom plane point
  Vector3d pt_bottom_right;

protected:
  // Plane paramters for a general plane
  // Ax + By + Cz + D = 0
  double A, B, C, D;
};
} // namespace mins
#endif // MINS_SIMULATIONPLANE_H

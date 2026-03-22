/*
 * This file is part of NR-SLAM
 *
 * Copyright (C) 2022-2023 Juan J. Gómez Rodríguez, José M.M. Montiel and Juan
 * D. Tardós, University of Zaragoza.
 *
 * NR-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "pin_hole.h"

void PinHole::Project(const Eigen::Vector3f& landmark_position,
                      Eigen::Vector2f& pixel_position) const {
  pixel_position(0) =
      params_.fx * landmark_position(0) / landmark_position(2) + params_.cx;
  pixel_position(1) =
      params_.fy * landmark_position(1) / landmark_position(2) + params_.cy;
}

void PinHole::Unproject(const Eigen::Vector2f& pixel_position,
                        Eigen::Vector3f& projecting_ray) const {
  projecting_ray(0) = (pixel_position(0) - params_.cx) / params_.fx;
  projecting_ray(1) = (pixel_position(1) - params_.cy) / params_.fy;
  projecting_ray(2) = 1.f;
}

void PinHole::ProjectionJacobian(
    const Eigen::Vector3f& landmark_position,
    Eigen::Matrix<float, 2, 3>& projection_jacobian) const {
  projection_jacobian(0, 0) = params_.fx / landmark_position(2);
  projection_jacobian(0, 1) = 0.f;
  projection_jacobian(0, 2) = -params_.fx * landmark_position(0) /
                              (landmark_position(2) * landmark_position(2));

  projection_jacobian(1, 0) = 0.f;
  projection_jacobian(1, 1) = params_.fy / landmark_position(2);
  projection_jacobian(1, 2) = -params_.fy * landmark_position(1) /
                              (landmark_position(2) * landmark_position(2));
}

void PinHole::UnprojectionJacobian(
    const Eigen::Vector2f& pixel_position,
    Eigen::Matrix<float, 3, 2>& unprojection_jacobian) const {
  unprojection_jacobian(0, 0) = 1 / params_.fx;
  unprojection_jacobian(0, 1) = 0.f;

  unprojection_jacobian(1, 0) = 0.f;
  unprojection_jacobian(1, 1) = 1 / params_.fy;

  unprojection_jacobian(2, 0) = 0.f;
  unprojection_jacobian(2, 1) = 0.f;
}

Eigen::Matrix3f PinHole::ToIntrinsicsMatrix() const {
  Eigen::Matrix3f intrinsics_matrix = Eigen::Matrix3f::Identity();
  intrinsics_matrix(0, 0) = params_.fx;
  intrinsics_matrix(0, 2) = params_.cx;
  intrinsics_matrix(1, 1) = params_.fy;
  intrinsics_matrix(1, 2) = params_.cy;

  return intrinsics_matrix;
}
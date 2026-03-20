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

#ifndef NRSLAM_DISTORTED_PIN_HOLE_H
#define NRSLAM_DISTORTED_PIN_HOLE_H

#include <opencv2/core/mat.hpp>
#include <vector>

#include "camera_model.h"

class DistortedPinHole : public CameraModel {
 public:
  DistortedPinHole();

  DistortedPinHole(const std::vector<float> calibration_parameters);

  void Project(const Eigen::Vector3f& landmark_position,
               Eigen::Vector2f& pixel_position);

  void Unproject(const Eigen::Vector2f& pixel_position,
                 Eigen::Vector3f& projecting_ray);

  void ProjectionJacobian(const Eigen::Vector3f& landmark_position,
                          Eigen::Matrix<float, 2, 3>& projection_jacobian);

  void UnprojectionJacobian(const Eigen::Vector2f& pixel_position,
                            Eigen::Matrix<float, 3, 2>& unprojection_jacobian);

  Eigen::Matrix3f ToIntrinsicsMatrix();

 private:
  cv::Mat intrinsics_matrix_;
  cv::Mat dist_coeffs_;
  cv::Mat rvec_;
  cv::Mat tvec_;
};

#endif  // NRSLAM_DISTORTED_PIN_HOLE_H

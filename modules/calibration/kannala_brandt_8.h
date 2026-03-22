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

#ifndef NRSLAM_KANNALA_BRANDT_8_H
#define NRSLAM_KANNALA_BRANDT_8_H

#include <opencv2/opencv.hpp>

#include "calibration/camera_model.h"
#include "calibration/camera_parameters.h"

class KannalaBrandt8 : public CameraModel {
 private:
  KannalaBrandt8Parameters params_;
  const float precision_ = 1e-6;

  // Deleted: force validation through constructor
  KannalaBrandt8() = delete;

 public:
  // Construct with typed parameters
  explicit KannalaBrandt8(const KannalaBrandt8Parameters& params)
      : params_(params) {
    // Update base class for backward compatibility
    calibration_parameters_ = params_.ToVector();
  }

  // Allow construction from vector (validates)
  explicit KannalaBrandt8(const std::vector<float>& calibration_parameters)
      : params_(KannalaBrandt8Parameters::FromVector(calibration_parameters)) {
    calibration_parameters_ = params_.ToVector();
  }

  void Project(const Eigen::Vector3f& landmark_position,
               Eigen::Vector2f& pixel_position) const override;

  void Unproject(const Eigen::Vector2f& p2D,
                 Eigen::Vector3f& projecting_ray) const override;

  void ProjectionJacobian(
      const Eigen::Vector3f& landmark_position,
      Eigen::Matrix<float, 2, 3>& projection_jacobian) const override;

  void UnprojectionJacobian(
      const Eigen::Vector2f& pixel_position,
      Eigen::Matrix<float, 3, 2>& unprojection_jacobian) const override;

  Eigen::Matrix3f ToIntrinsicsMatrix() const override;

  // Access to parameters (for testing/debugging)
  const KannalaBrandt8Parameters& GetParameters() const { return params_; }
};

#endif  // NRSLAM_KANNALA_BRANDT_8_H

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

#include "distorted_pin_hole.h"

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

DistortedPinHole::DistortedPinHole(const DistortedPinholeParameters& params)
    : params_(params) {
  calibration_parameters_ = params_.ToVector();

  rvec_ = cv::Mat::eye(3, 3, CV_64F);
  tvec_ = cv::Mat::zeros(3, 1, CV_64F);

  intrinsics_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
  intrinsics_matrix_.at<double>(0, 0) = params_.fx;
  intrinsics_matrix_.at<double>(1, 1) = params_.fy;
  intrinsics_matrix_.at<double>(0, 2) = params_.cx;
  intrinsics_matrix_.at<double>(1, 2) = params_.cy;
  intrinsics_matrix_.at<double>(2, 2) = 1.;

  dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
  dist_coeffs_.at<double>(0, 0) = params_.k1;
  dist_coeffs_.at<double>(1, 0) = params_.k2;
  dist_coeffs_.at<double>(2, 0) = params_.p1;
  dist_coeffs_.at<double>(3, 0) = params_.p2;
  dist_coeffs_.at<double>(4, 0) = params_.k3;
}

DistortedPinHole::DistortedPinHole(
    const std::vector<float>& calibration_parameters)
    : DistortedPinHole(
          DistortedPinholeParameters::FromVector(calibration_parameters)) {}

void DistortedPinHole::Project(const Eigen::Vector3f& landmark_position,
                               Eigen::Vector2f& pixel_position) const {
  const cv::Mat landmark_position_cv =
      (cv::Mat_<double>(1, 3) << landmark_position(0), landmark_position(1),
       landmark_position(2));

  cv::Mat pixel_position_cv;

  cv::projectPoints(landmark_position_cv, rvec_, tvec_, intrinsics_matrix_,
                    dist_coeffs_, pixel_position_cv, cv::noArray());

  pixel_position(0) = pixel_position_cv.at<cv::Vec2d>(0, 0)[0];
  pixel_position(1) = pixel_position_cv.at<cv::Vec2d>(0, 0)[1];
}

void DistortedPinHole::Unproject(const Eigen::Vector2f& pixel_position,
                                 Eigen::Vector3f& projecting_ray) const {
  const cv::Mat pixel_position_cv =
      (cv::Mat_<double>(1, 2) << pixel_position(0), pixel_position(1));

  cv::Mat undistorted_pixel_position_cv;
  cv::undistortPoints(pixel_position_cv, undistorted_pixel_position_cv,
                      intrinsics_matrix_, dist_coeffs_);

  projecting_ray(0) = undistorted_pixel_position_cv.at<cv::Vec2d>(0, 0)[0];
  projecting_ray(1) = undistorted_pixel_position_cv.at<cv::Vec2d>(0, 0)[1];
  projecting_ray(2) = 1.f;
}

void DistortedPinHole::ProjectionJacobian(
    const Eigen::Vector3f& landmark_position,
    Eigen::Matrix<float, 2, 3>& projection_jacobian) const {
  const cv::Mat landmark_position_cv =
      (cv::Mat_<double>(1, 3) << landmark_position(0), landmark_position(1),
       landmark_position(2));

  cv::Mat pixel_position_cv;
  cv::Mat jacobian_cv;

  cv::projectPoints(landmark_position_cv, rvec_, tvec_, intrinsics_matrix_,
                    dist_coeffs_, pixel_position_cv, jacobian_cv);

  projection_jacobian(0, 0) = jacobian_cv.at<double>(0, 6);
  projection_jacobian(0, 1) = jacobian_cv.at<double>(0, 7);
  projection_jacobian(0, 2) = jacobian_cv.at<double>(0, 8);
  projection_jacobian(1, 0) = jacobian_cv.at<double>(1, 6);
  projection_jacobian(1, 1) = jacobian_cv.at<double>(1, 7);
  projection_jacobian(1, 2) = jacobian_cv.at<double>(1, 8);
}

void DistortedPinHole::UnprojectionJacobian(
    const Eigen::Vector2f& pixel_position,
    Eigen::Matrix<float, 3, 2>& unprojection_jacobian) const {
  const double delta = 1e-5;
  const double u = pixel_position(0);
  const double v = pixel_position(1);

  // Batch all four finite-difference perturbations into one undistortPoints
  // call. Row order: (u+δ,v), (u-δ,v), (u,v+δ), (u,v-δ).
  const cv::Mat perturbed = (cv::Mat_<double>(4, 2) << u + delta, v, u - delta,
                             v, u, v + delta, u, v - delta);

  cv::Mat undist;
  cv::undistortPoints(perturbed, undist, intrinsics_matrix_, dist_coeffs_);

  const cv::Vec2d xu_plus = undist.at<cv::Vec2d>(0, 0);
  const cv::Vec2d xu_minus = undist.at<cv::Vec2d>(1, 0);
  const cv::Vec2d xv_plus = undist.at<cv::Vec2d>(2, 0);
  const cv::Vec2d xv_minus = undist.at<cv::Vec2d>(3, 0);

  unprojection_jacobian(0, 0) = (xu_plus[0] - xu_minus[0]) / (2.0 * delta);
  unprojection_jacobian(1, 0) = (xu_plus[1] - xu_minus[1]) / (2.0 * delta);
  unprojection_jacobian(2, 0) = 0.0;

  unprojection_jacobian(0, 1) = (xv_plus[0] - xv_minus[0]) / (2.0 * delta);
  unprojection_jacobian(1, 1) = (xv_plus[1] - xv_minus[1]) / (2.0 * delta);
  unprojection_jacobian(2, 1) = 0.0;
}

Eigen::Matrix3f DistortedPinHole::ToIntrinsicsMatrix() const {
  Eigen::Matrix3f intrinsics_matrix = Eigen::Matrix3f::Zero();
  intrinsics_matrix(0, 0) = intrinsics_matrix_.at<double>(0, 0);
  intrinsics_matrix(0, 2) = intrinsics_matrix_.at<double>(0, 2);
  intrinsics_matrix(1, 1) = intrinsics_matrix_.at<double>(1, 1);
  intrinsics_matrix(1, 2) = intrinsics_matrix_.at<double>(1, 2);
  intrinsics_matrix(2, 2) = intrinsics_matrix_.at<double>(2, 2);
  return intrinsics_matrix;
}
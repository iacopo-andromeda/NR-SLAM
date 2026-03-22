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

#ifndef NRSLAM_FRAME_OBSERVATION_H
#define NRSLAM_FRAME_OBSERVATION_H

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <optional>

#include "absl/log/log.h"
#include "absl/status/statusor.h"
#include "utilities/landmark_status.h"

typedef long unsigned int ID;

// Single self-contained record for one keypoint observation.
// This struct ensures tight coupling of related data that must be consistent.
struct FrameObservation {
  // Keypoint data (mutable but semantically should not be modified after
  // construction)
  cv::KeyPoint keypoint;

  // Landmark position (may be zero/uninitialized if status != TRACKED_WITH_3D)
  Eigen::Vector3f landmark_position;

  // Status of this observation
  LandmarkStatus status;

  // Ground truth (if available)
  absl::StatusOr<Eigen::Vector3f> ground_truth =
      absl::InternalError("No ground truth available");

  // The MapPoint ID if this is tracked with 3D.
  // Using optional ensures we can't accidentally use uninitialized ID.
  std::optional<ID> mappoint_id;

  // Explicit feature track ID, separate from keypoint.class_id.
  // Populated by Frame::InsertObservation so callers never need to read
  // the OpenCV class_id field directly for SLAM-level semantics.
  int track_id = -1;

  // Constructor with full validation
  FrameObservation(const cv::KeyPoint& kp, const Eigen::Vector3f& lm_pos,
                   LandmarkStatus st, std::optional<ID> mp_id = std::nullopt)
      : keypoint(kp),
        landmark_position(lm_pos),
        status(st),
        mappoint_id(mp_id) {
    // Invariant: if mappoint_id exists, must be TRACKED_WITH_3D or
    // JUST_TRIANGULATED
    if (mappoint_id.has_value()) {
      if (status != TRACKED_WITH_3D && status != JUST_TRIANGULATED) {
        throw std::logic_error(
            "MapPoint ID set but status is " +
            std::to_string(static_cast<int>(status)) +
            " (expected TRACKED_WITH_3D or JUST_TRIANGULATED)");
      }
      if (mappoint_id.value() < 0) {
        throw std::invalid_argument("MapPoint ID must be non-negative");
      }
    } else {
      // If no mappoint_id, ensure status doesn't require one
      if (status == TRACKED_WITH_3D || status == JUST_TRIANGULATED) {
        throw std::logic_error(
            "MapPoint ID missing but status is "
            "TRACKED_WITH_3D/JUST_TRIANGULATED");
      }
    }
  }

  // Convenience: is this observation valid for optimization?
  bool IsOptimizable() const {
    return status == TRACKED_WITH_3D || status == JUST_TRIANGULATED;
  }

  // Convenience: is this keypoint currently tracked?
  bool IsTracked() const {
    return status == TRACKED || status == TRACKED_WITH_3D ||
           status == JUST_TRIANGULATED;
  }
};

#endif  // NRSLAM_FRAME_OBSERVATION_H

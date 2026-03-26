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

#include "mapping/mapping.h"

#include <chrono>
#include <cmath>
#include <fstream>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/log/log.h"
#include "optimization/g2o_optimization.h"
#include "utilities/geometry_toolbox.h"

using namespace std;

namespace {
struct PoseComparisonMetrics {
  float translation_error_m = 0.f;
  float rotation_error_deg = 0.f;
};

PoseComparisonMetrics ComputePoseComparison(const Sophus::SE3f& external_pose,
                                            const Sophus::SE3f& slam_pose) {
  const Sophus::SE3f delta = external_pose * slam_pose.inverse();
  PoseComparisonMetrics metrics;
  metrics.translation_error_m = delta.translation().norm();
  const Eigen::AngleAxisf aa(delta.so3().matrix());
  constexpr float kRadToDeg = 57.29577951308232f;
  metrics.rotation_error_deg = std::abs(aa.angle()) * kRadToDeg;
  return metrics;
}
}  // namespace

Mapping::Mapping(std::shared_ptr<Map> map,
                 std::shared_ptr<CameraModel> calibration,
                 const Options options, TimeProfiler* time_profiler)
    : options_(options),
      map_(map),
      calibration_(calibration),
      time_profiler_(time_profiler) {}

void Mapping::DoMapping(const Sophus::SE3f& external_camera_pose) {
  if (map_->IsEmpty()) {
    LOG(INFO) << "[POSE_CMP_MAPPING] status=map_empty has_external_pose=1";
    return;
  }

  // Get next KeyFrame to process
  auto keyframe = map_->GetNextUnmappedKeyFrame();
  const char* mapping_stage = "frame";

  if (keyframe) {
    // If the KeyFrame is valid, do KeyFrame mapping.
    KeyFrameMapping();

    // Update tracking frame with the optimized KeyFrame.
    UpdateTrackingFrameFromKeyFrame(keyframe);
    mapping_stage = "keyframe";
  } else {
    // If there is no KeyFrame to process, do Frame mapping.
    FrameMapping();
  }

  auto current_frame = map_->GetMutableLastFrame();
  if (!current_frame) {
    LOG(INFO) << "[POSE_CMP_MAPPING] stage=" << mapping_stage
              << " status=no_slam_pose";
  } else {
    const auto pose_metrics = ComputePoseComparison(
        external_camera_pose, current_frame->CameraTransformationWorld());
    LOG(INFO) << "[POSE_CMP_MAPPING] frame=" << current_frame->GetId()
              << " stage=" << mapping_stage
              << " trans_err=" << pose_metrics.translation_error_m
              << " rot_err_deg=" << pose_metrics.rotation_error_deg;
  }
}

void Mapping::KeyFrameMapping() {
  LocalDeformableBundleAdjustment(map_, calibration_, map_->GetMapScale());
}

void Mapping::FrameMapping() {
  // Triangulate new landmarks.
  LandmarkTriangulation();
}

void Mapping::LandmarkTriangulation() {
  const auto t0 = std::chrono::steady_clock::now();

  auto temporal_buffer = map_->GetTemporalBuffer();

  // Iterate over the KeyPoints tracked in the last frame
  vector<int> triangulation_candidate_ids =
      temporal_buffer->GetTriangulationCandidatesIds();

  auto current_frame = map_->GetMutableLastFrame();

  int triangulated_landmarks = 0;
  const int n_candidates = triangulation_candidate_ids.size();

  const int current_frame_id = current_frame->GetId() - 1;

  int n_rigid_triangulations = 0;
  int n_deformable_triangulations = 0;

  int n_rejected_close_features = 0;
  int n_rejected_short_track = 0;
  int n_rigid_failed_rigidity = 0;
  int n_rigid_failed_triangulation = 0;
  int n_rigid_failed_parallax = 0;
  int n_rigid_failed_negative_depth = 0;
  int n_rigid_failed_reprojection = 0;
  int n_selected_rigid = 0;
  int n_selected_deformable = 0;
  int n_selection_tie = 0;
  int n_selected_invalid_status = 0;
  int n_failed_nan = 0;
  int n_failed_index = 0;

  absl::flat_hash_map<int, absl::StatusOr<Eigen::Vector3f>>
      rigid_triangulations;
  absl::flat_hash_map<int, absl::StatusOr<Eigen::Vector3f>>
      deformable_triangulations;

  // Try to triangulate candidates.
  vector<int> candidates_triangulated;
  vector<ID> landmark_ids_triangulated;
  for (auto candidate_id : triangulation_candidate_ids) {
    // Check there are no close features.
    auto neighbour_ids =
        temporal_buffer->GetClosestMapPointsToFeature(candidate_id, 10, 20, 60);
    if (neighbour_ids.empty()) {
      n_rejected_close_features++;
      deformable_triangulations[candidate_id] =
          absl::InternalError("Close features");
      // Do not continue here: rigid triangulation can still succeed without
      // local neighbours.
    }

    if (temporal_buffer->TrackLenght(candidate_id) >= 5) {
      auto landmark_triangulated =
          DeformableLandmarkTriangulation(candidate_id);

      if (landmark_triangulated.ok()) {
        if ((*landmark_triangulated).hasNaN()) {
          deformable_triangulations[candidate_id] = absl::InternalError("NaN.");
        } else {
          deformable_triangulations[candidate_id] = landmark_triangulated;
          if (landmark_triangulated.ok()) {
            n_deformable_triangulations++;
          }
        }
      } else {
        deformable_triangulations[candidate_id] = landmark_triangulated;
      }

    } else {
      n_rejected_short_track++;
      deformable_triangulations[candidate_id] = absl::InternalError(
          "Short track: " +
          to_string(temporal_buffer->TrackLenght(candidate_id)));
    }

    // Recover feature track.
    auto candidate_track = temporal_buffer->GetFeatureTrack(candidate_id);
    if (candidate_track.size() < 2) {
      n_rigid_failed_triangulation++;
      rigid_triangulations[candidate_id] =
          absl::InternalError("Short rigid track");
      continue;
    }

    const auto& [oldest_frame_id, oldest_keypoint] = candidate_track.front();
    const auto& [newest_frame_id, newest_keypoint] = candidate_track.back();

    // Rigidity condition.
    if (!temporal_buffer->CheckRigidity(oldest_frame_id, newest_frame_id,
                                        0.004)) {
      n_rigid_failed_rigidity++;
      rigid_triangulations[candidate_id] =
          absl::InternalError("Rigidity not detected");
      continue;
    }

    // Unproject rays.
    Eigen::Vector3f oldest_ray =
        calibration_->Unproject(oldest_keypoint.pt.x, oldest_keypoint.pt.y)
            .normalized();
    Eigen::Vector3f newest_ray =
        calibration_->Unproject(newest_keypoint.pt.x, newest_keypoint.pt.y)
            .normalized();

    // Get camera poses.
    auto oldest_camera_transform_world =
        temporal_buffer->GetCameraTransformWorld(oldest_frame_id);
    auto newest_camera_transform_world =
        temporal_buffer->GetCameraTransformWorld(newest_frame_id);

    auto landmark_position_status = TriangulateMidPoint(
        oldest_ray, newest_ray, *oldest_camera_transform_world,
        *newest_camera_transform_world);

    if (!landmark_position_status.ok()) {
      n_rigid_failed_triangulation++;
      rigid_triangulations[candidate_id] = landmark_position_status;
      continue;
    }

    if ((*landmark_position_status).hasNaN()) {
      n_failed_nan++;
      rigid_triangulations[candidate_id] = absl::InternalError("NaN.");
      continue;
    }

    Eigen::Vector3f normal_1 =
        (*landmark_position_status) -
        (*newest_camera_transform_world).inverse().translation();
    Eigen::Vector3f normal_2 =
        (*landmark_position_status) -
        (*oldest_camera_transform_world).inverse().translation();
    float parallax = RaysParallax(normal_1, normal_2);

    if (parallax < calibration_->GetRadiansPerPixel() * 10.f ||
        parallax > calibration_->GetRadiansPerPixel() * 20.f) {
      n_rigid_failed_parallax++;
      rigid_triangulations[candidate_id] =
          absl::InternalError("Parallax error.");
      continue;
    }

    // Check Reprojection error.
    Eigen::Vector3f landmark_position_1 =
        (*oldest_camera_transform_world) * (*landmark_position_status);

    if (landmark_position_1.z() < 0) {
      n_rigid_failed_negative_depth++;
      rigid_triangulations[candidate_id] =
          absl::InternalError("Parallax error.");
      continue;
    }

    cv::Point2f projected_landmark_1 =
        calibration_->Project(landmark_position_1);

    if (SquaredReprojectionError(oldest_keypoint.pt, projected_landmark_1) >
        5.991) {
      n_rigid_failed_reprojection++;
      rigid_triangulations[candidate_id] =
          absl::InternalError("Parallax error.");
      continue;
    }

    Eigen::Vector3f landmark_position_2 =
        (*newest_camera_transform_world) * (*landmark_position_status);

    if (landmark_position_2.z() < 0) {
      n_rigid_failed_negative_depth++;
      rigid_triangulations[candidate_id] =
          absl::InternalError("Parallax error.");
      continue;
    }

    cv::Point2f projected_landmark_2 =
        calibration_->Project(landmark_position_2);

    if (SquaredReprojectionError(newest_keypoint.pt, projected_landmark_2) >
        5.991) {
      n_rigid_failed_reprojection++;
      rigid_triangulations[candidate_id] =
          absl::InternalError("Parallax error.");
      continue;
    }

    rigid_triangulations[candidate_id] = landmark_position_status;
    if (landmark_position_status.ok()) {
      n_rigid_triangulations++;
    }
  }

  for (auto candidate_id : triangulation_candidate_ids) {
    Eigen::Vector3f landmark_triangulated;
    if (n_rigid_triangulations > 1.5 * n_deformable_triangulations) {
      n_selected_rigid++;
      if (!rigid_triangulations.contains(candidate_id) ||
          !rigid_triangulations.at(candidate_id).ok()) {
        n_selected_invalid_status++;
        continue;
      } else {
        landmark_triangulated = *(rigid_triangulations.at(candidate_id));
      }
    } else if (n_deformable_triangulations >= 1.5 * n_rigid_triangulations) {
      n_selected_deformable++;
      if (!deformable_triangulations.contains(candidate_id) ||
          !deformable_triangulations.at(candidate_id).ok()) {
        n_selected_invalid_status++;
        continue;
      } else {
        landmark_triangulated = *(deformable_triangulations.at(candidate_id));
      }
    } else {
      n_selection_tie++;
      continue;
    }

    const auto index_in_frame = temporal_buffer->GetLandmarkIndexInFrame(
        current_frame_id, candidate_id);

    if (landmark_triangulated.hasNaN()) {
      n_failed_nan++;
      continue;
    }

    if (!index_in_frame.ok()) {
      n_failed_index++;
      continue;
    }

    const LandmarkStatus current_status =
        current_frame->LandmarkStatuses()[*index_in_frame];
    if (current_status == TRACKED_WITH_3D ||
        current_status == JUST_TRIANGULATED) {
      n_selected_invalid_status++;
      continue;
    }

    // Create MapPoint and insert it into the map.
    auto mappoint =
        map_->CreateAndInsertMapPoint(landmark_triangulated, candidate_id);

    // Insert it into the tracking.
    current_frame->AddGeometryToKeypoint(*index_in_frame, landmark_triangulated,
                                         mappoint->GetId());

    candidates_triangulated.push_back(candidate_id);

    landmark_ids_triangulated.push_back(mappoint->GetId());

    triangulated_landmarks++;
  }

  // Add newly triangulated landmarks to the regularization graph.
  auto current_mappoints_ids = current_frame->GetMapPointsIdsWithStatus(
      {TRACKED_WITH_3D, JUST_TRIANGULATED});
  for (auto landmark_id : landmark_ids_triangulated) {
    auto landmark_position_status =
        current_frame->LandmarkPosition(landmark_id);
    CHECK_OK(landmark_position_status);

    for (auto other_landmark_id : current_mappoints_ids) {
      if (landmark_id == other_landmark_id) {
        continue;
      }

      auto other_landmark_position_status =
          current_frame->LandmarkPosition(other_landmark_id);
      CHECK_OK(other_landmark_position_status);

      Eigen::Vector3f relative_position =
          *other_landmark_position_status - *landmark_position_status;

      map_->GetRegularizationGraph()->AddEdge(landmark_id, other_landmark_id,
                                              relative_position);
    }
  }

  const auto t1 = std::chrono::steady_clock::now();
  LOG(INFO)
      << "[TRIANG_METRICS] frame=" << current_frame->GetId()
      << " candidates=" << n_candidates
      << " rigid_ok=" << n_rigid_triangulations
      << " deform_ok=" << n_deformable_triangulations
      << " selected_rigid=" << n_selected_rigid
      << " selected_deform=" << n_selected_deformable
      << " selection_tie=" << n_selection_tie
      << " selected_invalid=" << n_selected_invalid_status
      << " rejected_close=" << n_rejected_close_features
      << " rejected_short_track=" << n_rejected_short_track
      << " rigid_fail_rigidity=" << n_rigid_failed_rigidity
      << " rigid_fail_triangulation=" << n_rigid_failed_triangulation
      << " rigid_fail_parallax=" << n_rigid_failed_parallax
      << " rigid_fail_depth=" << n_rigid_failed_negative_depth
      << " rigid_fail_reprojection=" << n_rigid_failed_reprojection
      << " fail_nan=" << n_failed_nan << " fail_index=" << n_failed_index
      << " triangulated=" << triangulated_landmarks << " ms_total="
      << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
}

absl::StatusOr<Eigen::Vector3f> Mapping::DeformableLandmarkTriangulation(
    const int candidate_id) {
  return DeformableTriangulation(*map_->GetTemporalBuffer(), candidate_id,
                                 calibration_);
}

void Mapping::UpdateTrackingFrameFromKeyFrame(
    std::shared_ptr<KeyFrame> keyframe) {
  auto current_frame = map_->GetMutableLastFrame();

  current_frame->SetFromKeyFrame(keyframe);
}

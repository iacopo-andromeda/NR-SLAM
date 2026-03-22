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

#include "frame.h"

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

using namespace std;

Frame::Frame() {}

Frame::Frame(const Frame& other) {
  observations_ = other.observations_;

  keypoints_ = other.keypoints_;
  landmark_positions_ = other.landmark_positions_;
  landmark_status_ = other.landmark_status_;
  landmark_ground_truth_ = other.landmark_ground_truth_;

  CHECK_EQ(landmark_positions_.size(), landmark_ground_truth_.size());

  mappoint_id_to_index_ = other.mappoint_id_to_index_;
  index_to_mappoint_id_ = other.index_to_mappoint_id_;

  camera_transformation_world_ = other.camera_transformation_world_;

  id_ = other.id_;
  legacy_dirty_ = other.legacy_dirty_;
  observations_dirty_ = other.observations_dirty_;
}

void Frame::SyncLegacyVectors() {
  if (!legacy_dirty_) {
    return;
  }

  // Populate legacy vectors from observations_.
  // NOTE: landmark_ground_truth_ is maintained directly by InsertObservation
  // and does NOT need to be rebuilt here.  Copying absl::StatusOr error
  // objects allocates heap memory per element and is the dominant per-frame
  // overhead when this function is called (e.g. from DataAssociation).
  const size_t n = observations_.size();
  keypoints_.resize(n);
  landmark_positions_.resize(n);
  landmark_status_.resize(n);
  for (size_t i = 0; i < n; i++) {
    keypoints_[i] = observations_[i].keypoint;
    landmark_positions_[i] = observations_[i].landmark_position;
    landmark_status_[i] = observations_[i].status;
  }

  legacy_dirty_ = false;
}

void Frame::SyncFromLegacyVectors() {
  if (!observations_dirty_) {
    return;
  }

  // Build observations_ from legacy vectors.
  // This should match the structure expected by FrameObservation.
  // NOTE: ground_truth is NOT copied here — it is maintained independently by
  // InsertObservation and is never read from observations_ by external code.
  // Copying absl::StatusOr error objects allocates heap memory per element.
  CHECK_EQ(keypoints_.size(), landmark_positions_.size());
  CHECK_EQ(keypoints_.size(), landmark_status_.size());

  observations_.clear();
  observations_.reserve(keypoints_.size());
  for (size_t i = 0; i < keypoints_.size(); i++) {
    std::optional<ID> mappoint_id;
    if (index_to_mappoint_id_.contains(i)) {
      mappoint_id = index_to_mappoint_id_[i];
    }

    observations_.emplace_back(keypoints_[i], landmark_positions_[i],
                               landmark_status_[i], mappoint_id);
    // Propagate the OpenCV class_id as the SLAM track_id so that
    // GetTrackIdForIndex() returns the correct value for frames that were
    // populated through the legacy-vector path (e.g. SetFromKeyFrame).
    observations_[i].track_id = keypoints_[i].class_id;
  }

  observations_dirty_ = false;
}

std::vector<FrameObservation>& Frame::Observations() {
  SyncFromLegacyVectors();
  legacy_dirty_ = true;
  return observations_;
}

const std::vector<FrameObservation>& Frame::Observations() const {
  return observations_;
}

void Frame::SetFromKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
  // Recover tracked landmarks with 3D.
  keypoints_ = keyframe->GetKeypointsWithStatus({TRACKED_WITH_3D});
  landmark_positions_ =
      keyframe->GetLandmarkPositionsWithStatus({TRACKED_WITH_3D});
  landmark_ground_truth_ =
      keyframe->GetGroundTruthWithStatus({TRACKED_WITH_3D});

  landmark_status_.resize(keypoints_.size());
  fill(landmark_status_.begin(), landmark_status_.end(), TRACKED_WITH_3D);

  vector<ID> mappoint_ids =
      keyframe->GetMapPointsIdsWithStatus({TRACKED_WITH_3D});
  mappoint_id_to_index_.clear();
  index_to_mappoint_id_.clear();
  for (int idx = 0; idx < mappoint_ids.size(); idx++) {
    mappoint_id_to_index_[mappoint_ids[idx]] = idx;
    index_to_mappoint_id_[idx] = mappoint_ids[idx];
  }

  // Recover tracked landmarks without 3D.
  vector<cv::KeyPoint> keypoints_without_3d =
      keyframe->GetKeypointsWithStatus({TRACKED});
  keypoints_.insert(keypoints_.end(), keypoints_without_3d.begin(),
                    keypoints_without_3d.end());

  for (int idx = 0; idx < keypoints_without_3d.size(); idx++) {
    landmark_positions_.push_back(Eigen::Vector3f::Zero());
    landmark_status_.push_back(TRACKED);
    landmark_ground_truth_.push_back(
        absl::InternalError("No ground truth available."));
  }

  // Copy rest of fields and sync to unified structure
  camera_transformation_world_ = keyframe->CameraTransformationWorld();

  CHECK_EQ(landmark_positions_.size(), landmark_ground_truth_.size());

  // Sync to unified observations_
  observations_dirty_ = true;
  SyncFromLegacyVectors();
  legacy_dirty_ = false;
}

std::vector<cv::KeyPoint>& Frame::Keypoints() {
  SyncLegacyVectors();
  observations_dirty_ = true;
  return keypoints_;
}

std::vector<cv::KeyPoint> Frame::GetKeypointsWithStatus(
    const absl::flat_hash_set<LandmarkStatus> statuses) const {
  vector<cv::KeyPoint> keypoints;
  keypoints.reserve(keypoints_.size());
  for (int idx = 0; idx < landmark_status_.size(); idx++) {
    if (statuses.contains(landmark_status_[idx])) {
      keypoints.push_back(keypoints_[idx]);
    }
  }

  return keypoints;
}

int Frame::CountKeypointsWithStatus(
    const absl::flat_hash_set<LandmarkStatus> statuses) const {
  int count = 0;
  for (const LandmarkStatus s : landmark_status_) {
    if (statuses.contains(s)) count++;
  }
  return count;
}

std::vector<Eigen::Vector3f>& Frame::LandmarkPositions() {
  SyncLegacyVectors();
  observations_dirty_ = true;
  return landmark_positions_;
}

absl::StatusOr<Eigen::Vector3f> Frame::LandmarkPosition(ID mappoint_id) {
  auto is_consistent_index_for_mappoint = [&](const int idx) {
    if (idx < 0 || idx >= static_cast<int>(landmark_status_.size())) {
      return false;
    }
    if (!index_to_mappoint_id_.contains(idx) ||
        index_to_mappoint_id_[idx] != mappoint_id) {
      return false;
    }
    return true;
  };

  int idx = -1;
  auto it = mappoint_id_to_index_.find(mappoint_id);
  if (it != mappoint_id_to_index_.end()) {
    idx = it->second;
  }

  if (!is_consistent_index_for_mappoint(idx)) {
    idx = -1;
    for (const auto& [index, id] : index_to_mappoint_id_) {
      if (id == mappoint_id && is_consistent_index_for_mappoint(index)) {
        idx = index;
        break;
      }
    }

    if (idx < 0) {
      mappoint_id_to_index_.erase(mappoint_id);
      return absl::InternalError("MapPoint is not present in the Frame.");
    }

    mappoint_id_to_index_[mappoint_id] = idx;
  }

  const auto status = landmark_status_[idx];
  if (status != TRACKED_WITH_3D && status != JUST_TRIANGULATED) {
    return absl::InternalError("MapPoint is not present in the Frame.");
  }

  return landmark_positions_[idx];
}

std::vector<Eigen::Vector3f> Frame::GetLandmarkPositionsWithStatus(
    const absl::flat_hash_set<LandmarkStatus> statuses) const {
  vector<Eigen::Vector3f> landmark_positions;
  landmark_positions.reserve(landmark_positions_.size());
  for (int idx = 0; idx < landmark_status_.size(); idx++) {
    if (statuses.contains(landmark_status_[idx])) {
      landmark_positions.push_back(landmark_positions_[idx]);
    }
  }

  return landmark_positions;
}

std::vector<LandmarkStatus>& Frame::LandmarkStatuses() {
  SyncLegacyVectors();
  observations_dirty_ = true;
  return landmark_status_;
}

void Frame::InsertObservation(const cv::KeyPoint& keypoint,
                              const Eigen::Vector3f& landmark_position,
                              const ID mappoint_id,
                              const LandmarkStatus status) {
  if (status == TRACKED_WITH_3D || status == JUST_TRIANGULATED) {
    if (mappoint_id_to_index_.contains(mappoint_id)) {
      const int previous_index = mappoint_id_to_index_[mappoint_id];
      index_to_mappoint_id_.erase(previous_index);
    }

    const int new_index = keypoints_.size();
    mappoint_id_to_index_[mappoint_id] = new_index;
    index_to_mappoint_id_[new_index] = mappoint_id;
  }

  keypoints_.push_back(keypoint);
  landmark_positions_.push_back(landmark_position);
  landmark_status_.push_back(status);
  landmark_ground_truth_.push_back(
      absl::InternalError("No ground truth available."));

  // Also add to unified structure
  std::optional<ID> mp_id;
  if (status == TRACKED_WITH_3D || status == JUST_TRIANGULATED) {
    mp_id = mappoint_id;
  }
  observations_.emplace_back(keypoint, landmark_position, status, mp_id);
  observations_.back().track_id = keypoint.class_id;
  observations_.back().ground_truth =
      absl::InternalError("No ground truth available.");
  legacy_dirty_ = false;
  observations_dirty_ = false;
}

void Frame::AddGeometryToKeypoint(const int idx,
                                  const Eigen::Vector3f& landmark_position,
                                  const ID mappoint_id) {
  CHECK_LT(idx, landmark_positions_.size());
  CHECK(landmark_status_[idx] != JUST_TRIANGULATED &&
        landmark_status_[idx] != TRACKED_WITH_3D);

  if (index_to_mappoint_id_.contains(idx)) {
    const ID previous_mappoint_id = index_to_mappoint_id_[idx];
    if (previous_mappoint_id != mappoint_id &&
        mappoint_id_to_index_.contains(previous_mappoint_id) &&
        mappoint_id_to_index_[previous_mappoint_id] == idx) {
      mappoint_id_to_index_.erase(previous_mappoint_id);
    }
  }

  if (mappoint_id_to_index_.contains(mappoint_id)) {
    const int previous_index = mappoint_id_to_index_[mappoint_id];
    if (previous_index != idx) {
      index_to_mappoint_id_.erase(previous_index);
    }
  }

  landmark_positions_[idx] = landmark_position;
  landmark_status_[idx] = JUST_TRIANGULATED;
  index_to_mappoint_id_[idx] = mappoint_id;
  mappoint_id_to_index_[mappoint_id] = idx;

  // Also update observations_
  CHECK_LT(idx, observations_.size());
  observations_[idx].landmark_position = landmark_position;
  observations_[idx].status = JUST_TRIANGULATED;
  observations_[idx].mappoint_id = mappoint_id;
  legacy_dirty_ = false;
  observations_dirty_ = false;
}

void Frame::Clear() {
  observations_.clear();
  keypoints_.clear();
  landmark_positions_.clear();
  landmark_status_.clear();
  landmark_ground_truth_.clear();

  mappoint_id_to_index_.clear();
  index_to_mappoint_id_.clear();
  legacy_dirty_ = false;
  observations_dirty_ = false;
}

Sophus::SE3f& Frame::MutableCameraTransformationWorld() {
  return camera_transformation_world_;
}

Sophus::SE3f Frame::CameraTransformationWorld() const {
  return camera_transformation_world_;
}

const absl::flat_hash_map<int, ID>& Frame::IndexToMapPointId() const {
  return index_to_mappoint_id_;
}

const absl::flat_hash_map<ID, int>& Frame::MapPointIdToIndex() const {
  return mappoint_id_to_index_;
}

absl::StatusOr<int> Frame::GetIndexForMapPoint(ID mappoint_id) const {
  auto it = mappoint_id_to_index_.find(mappoint_id);
  if (it == mappoint_id_to_index_.end()) {
    return absl::NotFoundError(
        absl::StrCat("MapPoint ", mappoint_id, " not in frame."));
  }
  return it->second;
}

absl::StatusOr<ID> Frame::GetMapPointIdForIndex(int idx) const {
  auto it = index_to_mappoint_id_.find(idx);
  if (it == index_to_mappoint_id_.end()) {
    return absl::NotFoundError(
        absl::StrCat("No MapPoint for frame index ", idx, "."));
  }
  return it->second;
}

absl::Status Frame::SetStatusForMapPoint(ID mappoint_id,
                                         LandmarkStatus status) {
  auto idx_or = GetIndexForMapPoint(mappoint_id);
  if (!idx_or.ok()) {
    return idx_or.status();
  }

  const int idx = *idx_or;
  if (idx < 0 || idx >= static_cast<int>(landmark_status_.size())) {
    return absl::OutOfRangeError(
        absl::StrCat("Index ", idx, " out of range for landmark_status_."));
  }
  landmark_status_[idx] = status;

  if (idx >= 0 && idx < static_cast<int>(observations_.size())) {
    observations_[idx].status = status;
  }

  legacy_dirty_ = false;
  observations_dirty_ = false;

  return absl::OkStatus();
}

int Frame::GetTrackIdForIndex(int idx) const {
  DCHECK_GE(idx, 0);
  DCHECK_LT(static_cast<size_t>(idx), keypoints_.size());
  return keypoints_[idx].class_id;
}

std::vector<ID> Frame::GetMapPointsIdsWithStatus(
    const absl::flat_hash_set<LandmarkStatus> statuses) {
  vector<ID> mappoints_ids;
  mappoints_ids.reserve(index_to_mappoint_id_.size());

  for (int idx = 0; idx < landmark_status_.size(); idx++) {
    if (statuses.contains(landmark_status_[idx]) &&
        index_to_mappoint_id_.contains(idx)) {
      const ID mappoint_id = index_to_mappoint_id_[idx];

      if (!mappoint_id_to_index_.contains(mappoint_id) ||
          mappoint_id_to_index_[mappoint_id] != idx) {
        mappoint_id_to_index_[mappoint_id] = idx;
      }

      mappoints_ids.push_back(mappoint_id);
    }
  }

  return mappoints_ids;
}

void Frame::IncreaseId() { id_++; }

int Frame::GetId() { return id_; }

std::vector<LandmarkStatus> Frame::GetLandmarkStatusesWithStatus(
    const absl::flat_hash_set<LandmarkStatus> statuses) {
  vector<LandmarkStatus> landmark_statuses;
  landmark_statuses.reserve(landmark_status_.size());

  for (int idx = 0; idx < landmark_status_.size(); idx++) {
    if (statuses.contains(landmark_status_[idx])) {
      landmark_statuses.push_back(landmark_status_[idx]);
    }
  }

  return landmark_statuses;
}

std::vector<int> Frame::GetIndexWithStatus(
    const absl::flat_hash_set<LandmarkStatus> statuses) {
  vector<int> indexes;
  indexes.reserve(landmark_status_.size());

  for (int idx = 0; idx < landmark_status_.size(); idx++) {
    if (statuses.contains(landmark_status_[idx])) {
      indexes.push_back(idx);
    }
  }

  return indexes;
}

std::vector<absl::StatusOr<Eigen::Vector3f>>& Frame::MutableGroundTruth() {
  SyncLegacyVectors();
  return landmark_ground_truth_;
}

std::vector<absl::StatusOr<Eigen::Vector3f>> Frame::GroundTruth() {
  return landmark_ground_truth_;
}

std::vector<absl::StatusOr<Eigen::Vector3f>> Frame::GetGroundTruthWithStatus(
    const absl::flat_hash_set<LandmarkStatus> statuses) {
  vector<absl::StatusOr<Eigen::Vector3f>> ground_truth;
  ground_truth.reserve(landmark_ground_truth_.size());

  for (int idx = 0; idx < landmark_status_.size(); idx++) {
    if (statuses.contains(landmark_status_[idx])) {
      ground_truth.push_back(landmark_ground_truth_[idx]);
    }
  }

  return ground_truth;
}

void Frame::SetDeformationMaginitud(const float deformation_magnitud) {
  median_deformation_magnitud_ = deformation_magnitud;
}

float Frame::GetDeformationMagnitud() { return median_deformation_magnitud_; }

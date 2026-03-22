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

#include "temporal_buffer.h"

#include <algorithm>
#include <functional>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "absl/strings/str_cat.h"

using namespace std;

TemporalBuffer::TemporalBuffer(TemporalBuffer::Options& options)
    : options_(options) {}

void TemporalBuffer::InsertSnapshotFromFrame(Frame& frame) {
  Snapshot snapshot;

  auto& keypoints = frame.Keypoints();
  auto& landmark_positions = frame.LandmarkPositions();
  auto& statuses = frame.LandmarkStatuses();

  CHECK_EQ(keypoints.size(), landmark_positions.size());
  CHECK_EQ(keypoints.size(), statuses.size());

  for (int idx = 0; idx < keypoints.size(); idx++) {
    if (statuses[idx] != TRACKED_WITH_3D && statuses[idx] != TRACKED) {
      continue;
    }

    const int keypoint_id = frame.GetTrackIdForIndex(idx);
    DCHECK_GE(keypoint_id, 0);

    snapshot.tracks[keypoint_id] = TrackRecord{keypoints[idx], statuses[idx],
                                               landmark_positions[idx], idx};
  }

  snapshot.camera_transform_world = frame.CameraTransformationWorld();
  snapshot.deformation_magnitud = frame.GetDeformationMagnitud();
  snapshot.frame_id = frame.GetId();

  if (buffer_.size() > options_.max_buffer_size) {
    // Pop oldest snapshot
    int oldest_frame_id = buffer_.begin()->first;
    buffer_.erase(oldest_frame_id);
  }

  buffer_[frame.GetId()] = snapshot;

  PruneTrackHistoryByAge(options_.max_track_lookback_frames);

  const int stale_age_frames = std::max(3, options_.max_buffer_size / 2);
  PruneStaleTracks(stale_age_frames);
}

absl::btree_map<ID, TemporalBuffer::Snapshot>& TemporalBuffer::GetRawBuffer() {
  return buffer_;
}

std::vector<int> TemporalBuffer::GetTriangulationCandidatesIds() {
  const TemporalBuffer::Snapshot& last_snapshot = buffer_.rbegin()->second;

  vector<int> candidate_ids;

  for (const auto& [keypoint_id, track] : last_snapshot.tracks) {
    if (track.status == TRACKED) {
      candidate_ids.push_back(keypoint_id);
    }
  }

  return candidate_ids;
}

int TemporalBuffer::TrackLenght(const int keypoint_id) {
  int track_lenght = 0;
  for (const auto& [frame_id, snapshot] : buffer_) {
    if (snapshot.tracks.contains(keypoint_id)) {
      track_lenght++;
    }
  }

  return track_lenght;
}

std::vector<Sophus::SE3f> TemporalBuffer::GetLatestCameraPoses() {
  vector<Sophus::SE3f> latest_poses;

  for (const auto& [frame_id, snapshot] : buffer_) {
    latest_poses.push_back(snapshot.camera_transform_world);
  }

  return latest_poses;
}

std::vector<int> TemporalBuffer::GetClosestMapPointsToFeature(
    const int keypoint_id, const int num_neighbors,
    const int min_image_distance, const int max_image_distance) {
  const TemporalBuffer::Snapshot& last_snapshot = buffer_.rbegin()->second;

  if (num_neighbors <= 0) {
    return std::vector<int>();
  }

  auto keypoint_it = last_snapshot.tracks.find(keypoint_id);
  if (keypoint_it == last_snapshot.tracks.end()) {
    return std::vector<int>();
  }

  const cv::KeyPoint& keypoint = keypoint_it->second.keypoint;

  vector<pair<float, int>> distances;
  distances.reserve(last_snapshot.tracks.size());
  for (const auto& [neighbor_keypoint_id, neighbor_track] :
       last_snapshot.tracks) {
    if (neighbor_keypoint_id == keypoint_id) {
      continue;
    }

    if (neighbor_track.status != TRACKED_WITH_3D) {
      continue;
    }

    // Compute distance between KeyPoints
    float distance = cv::norm(keypoint.pt - neighbor_track.keypoint.pt);

    if (distance > max_image_distance) {
      continue;
    }

    if (distance < min_image_distance) {
      return std::vector<int>();
    }

    distances.push_back(make_pair(distance, neighbor_keypoint_id));
  }

  // Sort neighbors by distance
  sort(distances.begin(), distances.end());

  // Recover the K closest points
  vector<int> closest_mappoint_ids_;
  const size_t max_neighbors =
      std::min(distances.size(), static_cast<size_t>(num_neighbors));
  closest_mappoint_ids_.reserve(max_neighbors);
  for (size_t idx = 0; idx < max_neighbors; ++idx) {
    closest_mappoint_ids_.push_back(distances[idx].second);
  }

  return closest_mappoint_ids_;
}

vector<pair<float, int>> TemporalBuffer::GetClosestMapPointsToLocation(
    const Eigen::Vector3f location, const int keypoint_id,
    const int num_neighbors) {
  const TemporalBuffer::Snapshot& last_snapshot = buffer_.rbegin()->second;

  if (num_neighbors <= 0) {
    return {};
  }

  vector<pair<float, int>> distances;
  distances.reserve(last_snapshot.tracks.size());

  for (const auto& [neighbor_keypoint_id, neighbor_track] :
       last_snapshot.tracks) {
    if (neighbor_keypoint_id == keypoint_id) {
      continue;
    }

    if (neighbor_track.status != TRACKED_WITH_3D) {
      continue;
    }

    // Compute distance between KeyPoints
    float distance = (neighbor_track.landmark_position - location).norm();

    distances.push_back(make_pair(distance, neighbor_keypoint_id));
  }

  // Sort neighbors by distance
  sort(distances.begin(), distances.end());

  const size_t max_neighbors =
      std::min(distances.size(), static_cast<size_t>(num_neighbors));
  return vector<pair<float, int>>(distances.begin(),
                                  distances.begin() + max_neighbors);
}

std::vector<std::pair<ID, cv::KeyPoint>> TemporalBuffer::GetFeatureTrack(
    const int keypoint_id) {
  vector<pair<ID, cv::KeyPoint>> keypoint_track;

  for (const auto& [frame_id, snapshot] : buffer_) {
    auto it = snapshot.tracks.find(keypoint_id);
    if (it != snapshot.tracks.end()) {
      keypoint_track.push_back(make_pair(frame_id, it->second.keypoint));
    }
  }

  return keypoint_track;
}

int TemporalBuffer::PruneStaleTracks(const int max_stale_frames) {
  if (buffer_.empty()) {
    return 0;
  }

  const int latest_frame_id = buffer_.rbegin()->first;

  absl::flat_hash_map<int, int> keypoint_last_seen_frame;
  for (const auto& [frame_id, snapshot] : buffer_) {
    for (const auto& [keypoint_id, track] : snapshot.tracks) {
      keypoint_last_seen_frame[keypoint_id] = frame_id;
    }
  }

  absl::flat_hash_set<int> stale_keypoint_ids;
  for (const auto& [keypoint_id, last_seen_frame] : keypoint_last_seen_frame) {
    if (latest_frame_id - last_seen_frame > max_stale_frames) {
      stale_keypoint_ids.insert(keypoint_id);
    }
  }

  if (stale_keypoint_ids.empty()) {
    return 0;
  }

  int removed_entries = 0;
  for (auto& [frame_id, snapshot] : buffer_) {
    for (const int keypoint_id : stale_keypoint_ids) {
      removed_entries += snapshot.tracks.erase(keypoint_id);
    }
  }

  return removed_entries;
}

int TemporalBuffer::PruneTrackHistoryByAge(
    const int max_track_lookback_frames) {
  if (buffer_.empty()) {
    return 0;
  }

  if (max_track_lookback_frames < 1) {
    return 0;
  }

  const int latest_frame_id = buffer_.rbegin()->first;

  int removed_entries = 0;
  for (auto& [frame_id, snapshot] : buffer_) {
    std::vector<int> keypoints_to_remove;
    keypoints_to_remove.reserve(snapshot.tracks.size());

    for (const auto& [keypoint_id, track] : snapshot.tracks) {
      if (latest_frame_id - frame_id > max_track_lookback_frames) {
        keypoints_to_remove.push_back(keypoint_id);
      }
    }

    for (const int keypoint_id : keypoints_to_remove) {
      removed_entries += snapshot.tracks.erase(keypoint_id);
    }
  }

  return removed_entries;
}

absl::StatusOr<std::reference_wrapper<const TemporalBuffer::Snapshot>>
TemporalBuffer::GetSnapshot(const int frame_id) const {
  auto it = buffer_.find(frame_id);
  if (it == buffer_.end()) {
    return absl::NotFoundError(
        absl::StrCat("Frame Id ", frame_id, " not found in the buffer"));
  }
  return std::cref(it->second);
}

absl::StatusOr<Sophus::SE3f> TemporalBuffer::GetCameraTransformWorld(
    const int frame_id) {
  auto snapshot_or = GetSnapshot(frame_id);
  if (!snapshot_or.ok()) return snapshot_or.status();
  return snapshot_or->get().camera_transform_world;
}

absl::StatusOr<Eigen::Vector3f> TemporalBuffer::GetLandmarkPosition(
    const int frame_id, const int keypoint_id) {
  auto snapshot_or = GetSnapshot(frame_id);
  if (!snapshot_or.ok()) return snapshot_or.status();

  const Snapshot& snapshot = snapshot_or->get();
  auto track_it = snapshot.tracks.find(keypoint_id);
  if (track_it == snapshot.tracks.end()) {
    return absl::NotFoundError(
        absl::StrCat("KeyPoint Id ", keypoint_id, " not found in snapshot"));
  }

  return track_it->second.landmark_position;
}

absl::StatusOr<int> TemporalBuffer::GetLandmarkIndexInFrame(
    const int frame_id, const int keypoint_id) {
  auto snapshot_or = GetSnapshot(frame_id);
  if (!snapshot_or.ok()) return snapshot_or.status();

  const Snapshot& snapshot = snapshot_or->get();
  auto track_it = snapshot.tracks.find(keypoint_id);
  if (track_it == snapshot.tracks.end()) {
    return absl::NotFoundError(
        absl::StrCat("KeyPoint Id ", keypoint_id, " not found in snapshot"));
  }

  return track_it->second.frame_index;
}

bool TemporalBuffer::CheckRigidity(const int first_frame_id,
                                   const int last_frame_id,
                                   const float rigidity_th) {
  bool to_return = true;
  for (int frame_id = first_frame_id; frame_id <= last_frame_id; frame_id++) {
    auto it = buffer_.find(frame_id);
    if (it == buffer_.end()) {
      continue;
    }

    if (it->second.deformation_magnitud > rigidity_th) {
      to_return = false;
    }
  }

  return to_return;
}

absl::Status TemporalBuffer::Validate() const {
  for (const auto& [frame_id, snapshot] : buffer_) {
    if (snapshot.frame_id != static_cast<int>(frame_id)) {
      return absl::InternalError(
          absl::StrCat("Snapshot frame_id ", snapshot.frame_id,
                       " does not match buffer key ", frame_id));
    }
    for (const auto& [keypoint_id, track] : snapshot.tracks) {
      if (track.frame_index < 0) {
        return absl::InternalError(absl::StrCat(
            "Track for keypoint ", keypoint_id, " in frame ", frame_id,
            " has invalid frame_index ", track.frame_index));
      }
    }
  }
  return absl::OkStatus();
}

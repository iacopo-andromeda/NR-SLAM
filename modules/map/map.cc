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

#include "map/map.h"

#include "absl/log/log.h"

using namespace std;

Map::Map(Map::Options& options) : options_(options) {
  RegularizationGraph::Options regularization_graph_options;
  regularization_graph_options.weight_sigma = 10.5f;
  regularization_graph_options.streching_th = 1.1f;
  regularization_graph_ =
      make_shared<RegularizationGraph>(regularization_graph_options, this);

  TemporalBuffer::Options temporal_buffer_options;
  temporal_buffer_options.max_buffer_size = options.max_temporal_buffer_size;
  temporal_buffer_options.max_track_lookback_frames =
      options.triangulation_track_lookback_frames;
  temporal_buffer_ = make_shared<TemporalBuffer>(temporal_buffer_options);
}

void Map::InsertKeyFrame(std::shared_ptr<KeyFrame> keyframe) {
  keyframes_mutex_.Lock();

  keyframes_[keyframe->GetId()] = keyframe;

  unmapped_keyframes_.push_back(keyframe->GetId());

  keyframes_mutex_.Unlock();
}

void Map::InsertMapPoint(std::shared_ptr<MapPoint> mappoint) {
  mappoints_[mappoint->GetId()] = mappoint;
}

std::shared_ptr<MapPoint> Map::FindNearbyMapPoint(
    const Eigen::Vector3f& position) const {
  if (options_.min_mappoint_distance <= 0.0f || mappoints_.empty()) {
    return nullptr;
  }

  const float min_distance_sq =
      options_.min_mappoint_distance * options_.min_mappoint_distance;

  std::shared_ptr<MapPoint> nearest_mappoint = nullptr;
  float nearest_distance_sq = min_distance_sq;

  for (const auto& [mappoint_id, mappoint] : mappoints_) {
    (void)mappoint_id;
    const float distance_sq =
        (mappoint->GetLastWorldPosition() - position).squaredNorm();
    if (distance_sq <= nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest_mappoint = mappoint;
    }
  }

  return nearest_mappoint;
}

std::shared_ptr<MapPoint> Map::CreateAndInsertMapPoint(
    const Eigen::Vector3f& position, const int keypoint_id) {
  if (auto nearby = FindNearbyMapPoint(position)) {
    return nearby;
  }

  auto mappoint = make_shared<MapPoint>(position, keypoint_id);
  InsertMapPoint(mappoint);

  return mappoint;
}

void Map::RemoveMapPoint(ID id) {
  regularization_graph_->RemoveVertex(id);
  mappoints_.erase(id);
}

std::shared_ptr<KeyFrame> Map::GetNextUnmappedKeyFrame() {
  if (unmapped_keyframes_.empty()) {
    return nullptr;
  } else {
    auto keyframe = GetKeyFrame(unmapped_keyframes_.front());

    unmapped_keyframes_.pop_back();

    return keyframe;
  }
}

absl::btree_map<ID, std::shared_ptr<KeyFrame>> Map::GetKeyFrames() {
  keyframes_mutex_.Lock();

  auto keyframes = keyframes_;

  keyframes_mutex_.Unlock();

  return keyframes;
}

absl::flat_hash_map<ID, std::shared_ptr<MapPoint>>& Map::GetMapPoints() {
  return mappoints_;
}

std::shared_ptr<MapPoint> Map::GetMapPoint(ID id) {
  if (mappoints_.contains(id) == 0) {
    return nullptr;
  } else
    return mappoints_[id];
}

std::shared_ptr<KeyFrame> Map::GetKeyFrame(ID id) {
  if (keyframes_.count(id) == 0) {
    return nullptr;
  } else
    return keyframes_[id];
}

void Map::SetLastFrame(std::shared_ptr<Frame> frame) {
  last_frame_mutex_.Lock();

  last_frame_ = frame;

  temporal_buffer_->InsertSnapshotFromFrame(*last_frame_);

  frame_to_render_ = *frame;

  last_frame_mutex_.Unlock();

  frame->IncreaseId();
}

Frame Map::GetLastFrame() {
  last_frame_mutex_.Lock();

  Frame latest_frame = frame_to_render_;

  last_frame_mutex_.Unlock();

  return latest_frame;
}

std::shared_ptr<Frame> Map::GetMutableLastFrame() { return last_frame_; }

bool Map::IsEmpty() { return keyframes_.empty() && mappoints_.empty(); }

// TODO: we should do this with a Keyframe
void Map::InitializeRegularizationGraph(const float sigma) {
  if (last_frame_ && last_frame_->Keypoints().empty()) {
    LOG(FATAL) << "At least one Frame must be inserted before initializing the "
                  "regularization graph.";
  }

  regularization_graph_->SetSigma(sigma);

  vector<Eigen::Vector3f>& landmark_positions =
      last_frame_->LandmarkPositions();

  for (int idx = 0; idx < landmark_positions.size(); idx++) {
    if (!last_frame_->IndexToMapPointId().contains(idx)) {
      continue;
    }

    const ID mappoint_id_1 = last_frame_->IndexToMapPointId().at(idx);

    for (int other_idx = idx + 1; other_idx < landmark_positions.size();
         other_idx++) {
      if (!last_frame_->IndexToMapPointId().contains(other_idx)) {
        continue;
      }

      const ID mappoint_id_2 = last_frame_->IndexToMapPointId().at(other_idx);

      Eigen::Vector3f relative_position =
          landmark_positions[other_idx] - landmark_positions[idx];

      regularization_graph_->AddEdge(mappoint_id_1, mappoint_id_2,
                                     relative_position);
    }
  }
}

std::shared_ptr<RegularizationGraph> Map::GetRegularizationGraph() {
  return regularization_graph_;
}

std::shared_ptr<TemporalBuffer> Map::GetTemporalBuffer() {
  return temporal_buffer_;
}

void Map::SetAllMappointsToNonActive() {
  for (const auto& mappoint : mappoints_) {
    mappoint.second->IsActive() = false;
  }
}

void Map::SetMapScale(const float scale) { map_scale_ = scale; }

float Map::GetMapScale() { return map_scale_; }

void Map::RebaseWorldFrame(const Sophus::SE3f& world_old_from_world_new) {
  const Sophus::SE3f world_new_from_world_old =
      world_old_from_world_new.inverse();

  // MapPoints (world positions).
  for (auto& [mappoint_id, mappoint] : mappoints_) {
    (void)mappoint_id;
    Eigen::Vector3f position_old = mappoint->GetLastWorldPosition();
    Eigen::Vector3f position_new = world_new_from_world_old * position_old;
    mappoint->SetLastWorldPosition(position_new);
  }

  // KeyFrames (camera poses + cached landmark positions).
  for (auto& [keyframe_id, keyframe] : keyframes_) {
    (void)keyframe_id;
    keyframe->CameraTransformationWorld() =
        keyframe->CameraTransformationWorld() * world_old_from_world_new;

    auto& keyframe_landmarks = keyframe->LandmarkPositions();
    for (auto& landmark_position : keyframe_landmarks) {
      if (landmark_position.hasNaN()) {
        continue;
      }
      landmark_position = world_new_from_world_old * landmark_position;
    }
  }

  // Last frame (camera pose + cached landmark positions).
  if (last_frame_) {
    last_frame_->MutableCameraTransformationWorld() =
        last_frame_->CameraTransformationWorld() * world_old_from_world_new;

    auto& frame_landmarks = last_frame_->LandmarkPositions();
    for (auto& landmark_position : frame_landmarks) {
      if (landmark_position.hasNaN()) {
        continue;
      }
      landmark_position = world_new_from_world_old * landmark_position;
    }

    frame_to_render_ = *last_frame_;
  }

  // Temporal buffer snapshots.
  auto& raw_buffer = temporal_buffer_->GetRawBuffer();
  for (auto& [snapshot_id, snapshot] : raw_buffer) {
    (void)snapshot_id;
    snapshot.camera_transform_world =
        snapshot.camera_transform_world * world_old_from_world_new;

    for (auto& [track_id, track_record] : snapshot.tracks) {
      (void)track_id;
      if (track_record.landmark_position.hasNaN()) {
        continue;
      }
      track_record.landmark_position =
          world_new_from_world_old * track_record.landmark_position;
    }
  }

  // Regularization edge vectors are world-frame relative vectors and must be
  // rotated to stay consistent after world-frame re-expression.
  regularization_graph_->RotateRelativeVectors(
      world_new_from_world_old.so3().matrix());
}

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

#ifndef NRSLAM_TEMPORAL_BUFFER_H
#define NRSLAM_TEMPORAL_BUFFER_H

#include <opencv2/opencv.hpp>

#include "absl/container/btree_map.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "map/frame.h"
#include "sophus/se3.hpp"

typedef long unsigned int ID;

class TemporalBuffer {
 public:
  struct Options {
    int max_buffer_size = 40;
    int max_track_lookback_frames = 5;
  };

  struct TrackRecord {
    cv::KeyPoint keypoint;
    LandmarkStatus status;
    Eigen::Vector3f landmark_position;
    int frame_index = -1;
  };

  TemporalBuffer() = delete;

  TemporalBuffer(Options& options);

  struct Snapshot {
    // Per-keypoint track data. Keeping all related fields in a single record
    // prevents the snapshot from drifting out of sync across multiple maps.
    absl::flat_hash_map<int, TrackRecord> tracks;

    // Camera pose.
    Sophus::SE3f camera_transform_world;

    // TODO: we should add index inside the frame to make things easier when
    // inserting in the frame after triangulation
    int frame_id;

    float deformation_magnitud;
  };

  void InsertSnapshotFromFrame(Frame& frame);

  absl::btree_map<ID, Snapshot>& GetRawBuffer();

  std::vector<int> GetTriangulationCandidatesIds();

  int TrackLenght(const int keypoint_id);

  std::vector<Sophus::SE3f> GetLatestCameraPoses();

  std::vector<int> GetClosestMapPointsToFeature(const int keypoint_id,
                                                const int num_neighbors,
                                                const int min_image_distance,
                                                const int max_image_distance);

  std::vector<std::pair<float, int>> GetClosestMapPointsToLocation(
      const Eigen::Vector3f location, const int keypoint_id,
      const int num_neighbors);

  std::vector<std::pair<ID, cv::KeyPoint>> GetFeatureTrack(
      const int keypoint_id);

  int PruneStaleTracks(const int max_stale_frames);

  absl::StatusOr<Sophus::SE3f> GetCameraTransformWorld(const int frame_id);

  absl::StatusOr<Eigen::Vector3f> GetLandmarkPosition(const int frame_id,
                                                      const int keypoint_id);

  absl::StatusOr<int> GetLandmarkIndexInFrame(const int frame_id,
                                              const int keypoint_id);

  bool CheckRigidity(const int first_frame_id, const int last_frame_id,
                     const float rigidity_th);

  // Checks that all buffer entries are internally consistent.
  // Returns an error if any invariant is violated.
  absl::Status Validate() const;

 private:
  // Returns a const reference to the Snapshot for frame_id, or NotFoundError.
  absl::StatusOr<std::reference_wrapper<const Snapshot>> GetSnapshot(
      int frame_id) const;

  int PruneTrackHistoryByAge(const int max_track_lookback_frames);

  absl::btree_map<ID, Snapshot> buffer_;

  Options options_;
};

#endif  // NRSLAM_TEMPORAL_BUFFER_H

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

#ifndef NRSLAM_TRACKING_H
#define NRSLAM_TRACKING_H

#include "absl/container/flat_hash_set.h"
#include "calibration/camera_model.h"
#include "features/feature.h"
#include "map/frame.h"
#include "map/map.h"
#include "matching/lucas_kanade_tracker.h"
#include "tracking/monocular_map_initializer.h"
#include "utilities/time_profiler.h"
#include "visualization/image_visualizer.h"

class Tracking {
 public:
  struct Options {
    int klt_window_size = 21;
    int klt_max_level = 3;
    int klt_max_iters = 50;
    float klt_epsilon = 0.01;
    float klt_min_eig_th = 1e-4;
    float klt_min_SSIM = 0.7;

    int images_to_insert_keyframe = 5;

    int stale_mappoint_max_age_frames = 30;

    int lost_bootstrap_frame_stride = 1;

    int min_tracked_points_abort = 10;
    int lost_recovery_grace_frames = 5;
    int lost_recovery_min_tracked_points = 3;

    // Monocular re/bootstrap initializer
    int monocular_init_klt_window_size = 21;
    int monocular_init_klt_max_level = 4;
    int monocular_init_klt_max_iters = 10;
    float monocular_init_klt_epsilon = 0.0001f;
    float monocular_init_klt_min_eig_th = 0.0001f;
    float monocular_init_klt_min_SSIM = 0.5f;
    int monocular_init_rigid_max_features = 4000;
    float monocular_init_rigid_min_parallax = 0.999f;
    float monocular_init_rigid_epipolar_threshold = 0.005f;

    // Shi-Tomasi feature extraction
    int feature_max_corners = 1000;
    float feature_quality_level = 0.1f;
    float feature_min_distance = 7.0f;
  };

  enum TrackingStatus { NOT_INITIALIZED, TRACKING, LOST };

  Tracking() = delete;

  Tracking(const Options options, std::shared_ptr<Map> map,
           std::shared_ptr<CameraModel> calibration,
           std::shared_ptr<ImageVisualizer> image_visualizer,
           TimeProfiler* time_profiler);

  void TrackImage(const cv::Mat& im,
                  const absl::flat_hash_map<std::string, cv::Mat>& masks,
                  const Sophus::SE3f& external_camera_pose);

  TrackingStatus GetTrackingStatus() const;

  // Rebases tracker-owned cached world-frame state after the map world frame
  // changes. The transform follows the same convention as
  // Map::RebaseWorldFrame: world_old_from_world_new.
  void RebaseWorldFrame(const Sophus::SE3f& world_old_from_world_new);

  void Reset();

 private:
  void ExtractFeatures(const cv::Mat& im, const cv::Mat& mask,
                       const std::vector<cv::KeyPoint>& extracted_keypoints,
                       std::vector<cv::KeyPoint>& new_keypoints);

  void MonocularMapInitialization(const cv::Mat& im_left, const cv::Mat& mask);

  absl::flat_hash_set<ID> TrackCameraAndDeformation(const cv::Mat& im,
                                                    const cv::Mat& mask);

  void DataAssociation(const cv::Mat& im, const cv::Mat& mask);

  void CameraPoseEstimation();

  absl::flat_hash_set<ID> CameraPoseAndDeformationEstimation();

  void KeyFrameInsertion(
      const cv::Mat& im,
      const absl::flat_hash_map<std::string, cv::Mat>& masks);

  bool NeedNewKeyFrame();

  void CreateNewKeyFrame(
      const cv::Mat& im,
      const absl::flat_hash_map<std::string, cv::Mat>& masks);

  void ExtractFeaturesInFrame(const cv::Mat& im, const cv::Mat& mask,
                              Frame& frame);

  void SetKLTReference(const cv::Mat& im, Frame& frame, const cv::Mat& mask);

  void PointReuse(const cv::Mat& im, const cv::Mat& mask,
                  absl::flat_hash_set<ID> lost_mappoint_ids);

  void UpdateTriangulatedPoints();

  Options options_;

  std::shared_ptr<Map> map_;

  std::shared_ptr<CameraModel> calibration_;

  std::shared_ptr<Feature> feature_extractor_;

  LucasKanadeTracker klt_tracker_;

  std::shared_ptr<Frame> current_frame_;

  Sophus::SE3f motion_model_;

  std::shared_ptr<ImageVisualizer> image_visualizer_;

  int n_images_from_last_keyframe_ = 0;

  std::unique_ptr<MonocularMapInitializer> monocular_map_initializer_;

  TrackingStatus tracking_status_;

  Sophus::SE3f previous_camera_transform_world_;

  absl::flat_hash_map<ID, int> mappoint_last_seen_frame_;

  // Camera pose recorded the moment the system transitions to LOST, used as
  // the reference-frame world pose during re-bootstrap triangulation.
  Sophus::SE3f pose_at_lost_entry_;

  int recovery_grace_frames_remaining_ = 0;

  TimeProfiler* time_profiler_;
};

#endif  // NRSLAM_TRACKING_H

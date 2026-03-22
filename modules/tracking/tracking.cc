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

#include "tracking.h"

#include <algorithm>
#include <chrono>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "features/shi_tomasi_cv.h"
#include "optimization/g2o_optimization.h"
#include "utilities/dbscan.h"
#include "utilities/geometry_toolbox.h"
#include "utilities/statistics_toolbox.h"

using namespace std;

Tracking::Tracking(const Tracking::Options options, std::shared_ptr<Map> map,
                   std::shared_ptr<CameraModel> calibration,
                   std::shared_ptr<ImageVisualizer> image_visualizer,
                   TimeProfiler* time_profiler)
    : options_(options),
      map_(map),
      calibration_(calibration),
      image_visualizer_(image_visualizer),
      tracking_status_(NOT_INITIALIZED),
      time_profiler_(time_profiler) {
  ShiTomasiCV::Options shi_tomasi_options;
  shi_tomasi_options.maxCorners = options_.feature_max_corners;
  shi_tomasi_options.qualityLevel = options_.feature_quality_level;
  shi_tomasi_options.minDistance = options_.feature_min_distance;

  feature_extractor_ = make_shared<ShiTomasiCV>(shi_tomasi_options);

  klt_tracker_ = LucasKanadeTracker(
      cv::Size(options_.klt_window_size, options_.klt_window_size),
      options_.klt_max_level, options_.klt_max_iters, options_.klt_epsilon,
      options_.klt_min_eig_th);

  current_frame_ = make_shared<Frame>();

  MonocularMapInitializer::Options monocular_map_initializer_options;
  monocular_map_initializer_options.klt_window_size = 21;
  monocular_map_initializer_options.klt_max_level = 4;
  monocular_map_initializer_options.klt_max_iters = 10;
  monocular_map_initializer_options.klt_epsilon = 0.0001;
  monocular_map_initializer_options.klt_min_eig_th = 0.0001;
  monocular_map_initializer_options.klt_min_SSIM = 0.5;

  monocular_map_initializer_options.rigid_initializer_max_features = 4000;
  monocular_map_initializer_options.rigid_initializer_min_sample_set_size = 8;
  monocular_map_initializer_options.rigid_initializer_min_parallax = 0.999;
  monocular_map_initializer_options.rigid_initializer_epipolar_threshold =
      0.005;

  monocular_map_initializer_ = make_unique<MonocularMapInitializer>(
      monocular_map_initializer_options, feature_extractor_, calibration_,
      image_visualizer_);
}

void Tracking::TrackImage(
    const cv::Mat& im, const absl::flat_hash_map<std::string, cv::Mat>& masks) {
  map_->SetAllMappointsToNonActive();

  static uint64_t frame_seq = 0;
  const uint64_t frame_id = frame_seq++;
  const auto t_frame_start = std::chrono::steady_clock::now();

  const bool map_is_empty = map_->IsEmpty();

  if (map_is_empty) {
    MonocularMapInitialization(im, masks.at("Global"));
  } else {
    // NOTE: the LOST branch is handled below; fall into normal tracking only
    // when we're already in TRACKING state.
  }

  if (!map_is_empty && tracking_status_ == LOST) {
    if (options_.lost_bootstrap_frame_stride > 1 &&
        (frame_id % options_.lost_bootstrap_frame_stride) != 0) {
      LOG(INFO) << "[LOST] Bootstrap stride skip: frame=" << frame_id
                << " stride=" << options_.lost_bootstrap_frame_stride;
      return;
    }

    // ----- LOST mode: rebuild 3D structure via monocular re-bootstrap -----
    const bool bootstrap_ok = LostModeBootstrap(im, masks.at("Global"));
    if (bootstrap_ok) {
      tracking_status_ = TRACKING;
      recovery_grace_frames_remaining_ = options_.lost_recovery_grace_frames;
      LOG(WARNING) << "[LOST] Bootstrap succeeded, resuming normal tracking";
    }
  } else if (!map_is_empty && tracking_status_ != LOST) {
    // ----- Normal tracking -----------------------------------------------
    const auto t0 = std::chrono::steady_clock::now();
    UpdateTriangulatedPoints();
    const auto t1 = std::chrono::steady_clock::now();

    absl::flat_hash_set<ID> lost_mappoint_ids =
        TrackCameraAndDeformation(im, masks.at("Global"));
    const auto t2 = std::chrono::steady_clock::now();

    const int tracked_before_reuse =
        current_frame_->CountKeypointsWithStatus({TRACKED_WITH_3D});

    PointReuse(im, masks.at("Global"), lost_mappoint_ids);
    const auto t3 = std::chrono::steady_clock::now();

    MarkTrackedMapPointsSeen(static_cast<int>(frame_id));
    PruneStaleMapPointCache(static_cast<int>(frame_id));  // seeds new MPs too

    const int tracked_after_reuse =
        current_frame_->CountKeypointsWithStatus({TRACKED_WITH_3D});

    const int reuse_gain = tracked_after_reuse - tracked_before_reuse;
    const bool in_recovery_grace = (recovery_grace_frames_remaining_ > 0);
    const int min_points_abort = in_recovery_grace
                                     ? options_.lost_recovery_min_tracked_points
                                     : options_.min_tracked_points_abort;
    const bool tracking_abort = (tracked_after_reuse < min_points_abort);

    LOG(INFO) << "[TRACK_METRICS] frame=" << frame_id
              << " tracked_before_reuse=" << tracked_before_reuse
              << " tracked_after_reuse=" << tracked_after_reuse
              << " reuse_gain=" << reuse_gain
              << " min_points_abort=" << min_points_abort
              << " recovery_grace_remaining="
              << recovery_grace_frames_remaining_
              << " abort=" << (tracking_abort ? 1 : 0)
              << " ms_update_triangulated="
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                     .count()
              << " ms_track_deformation="
              << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
                     .count()
              << " ms_point_reuse="
              << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2)
                     .count();

    if (tracking_abort) {
      LOG(WARNING) << "Tracking frame abort: tracked_3d=" << tracked_after_reuse
                   << " is below minimum " << min_points_abort
                   << " after point reuse";
      tracking_status_ = LOST;
      recovery_grace_frames_remaining_ = 0;
      pose_at_lost_entry_ = current_frame_->CameraTransformationWorld();
      ResetMonocularInitializer();
      current_frame_->Clear();
      ExtractFeaturesInFrame(im, masks.at("Global"), *current_frame_);
      SetKLTReference(im, *current_frame_, masks.at("Global"));
      map_->SetLastFrame(current_frame_);

      LOG(WARNING)
          << "Tracking switched to LOST; re-seeded features for recovery";
      return;
    }

    if (recovery_grace_frames_remaining_ > 0) {
      recovery_grace_frames_remaining_--;
    }

    tracking_status_ = TRACKING;

    KeyFrameInsertion(im, masks);
    const auto t4 = std::chrono::steady_clock::now();

    map_->SetLastFrame(current_frame_);

    LOG(INFO) << "[TRACK_METRICS] frame=" << frame_id
              << " ms_keyframe_insertion="
              << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
                     .count()
              << " ms_total="
              << std::chrono::duration_cast<std::chrono::milliseconds>(
                     t4 - t_frame_start)
                     .count();

    if (image_visualizer_) {
      image_visualizer_->DrawCurrentFrame(*current_frame_);
      image_visualizer_->DrawRegularizationGraph(
          *current_frame_, *(map_->GetRegularizationGraph()));
      image_visualizer_->DrawFeatures(current_frame_->Keypoints());
    }
  }
}

Tracking::TrackingStatus Tracking::GetTrackingStatus() const {
  return tracking_status_;
}

void Tracking::ExtractFeatures(
    const cv::Mat& im, const cv::Mat& mask,
    const std::vector<cv::KeyPoint>& extracted_keypoints,
    std::vector<cv::KeyPoint>& new_keypoints) {
  cv::Mat augmented_mask;
  feature_extractor_->AugmentMask(mask, extracted_keypoints, augmented_mask);
  // Extract features.
  feature_extractor_->Extract(im, augmented_mask, new_keypoints);

  // // Mask out points.
  // vector<cv::KeyPoint> masked_keypoints;
  // for (size_t i = 0; i < keypoints.size(); i++) {
  //   if (!mask.at<uchar>(keypoints[i].pt)) {
  //     continue;
  //   } else {
  //     masked_keypoints.push_back(keypoints[i]);
  //   }
  // }

  // keypoints = masked_keypoints;
}

void Tracking::MonocularMapInitialization(const cv::Mat& im,
                                          const cv::Mat& mask) {
  auto initialization_status =
      monocular_map_initializer_->ProcessNewImage(im, mask);

  if (!initialization_status.ok()) {
    LOG(WARNING) << "Monocular initialization deferred: "
                 << initialization_status.status().message();
    return;
  }

  auto initialization_results = *initialization_status;

  vector<float> depths;
  for (int idx = 0; idx < initialization_results.current_keypoints.size();
       idx++) {
    Eigen::Vector3f current_landmark_position =
        initialization_results.current_landmark_positions[idx];
    depths.push_back(current_landmark_position.z());
  }

  const int median_idx = depths.size() / 2;
  nth_element(depths.begin(), depths.begin() + median_idx, depths.end());
  const float median_depth = depths[median_idx];
  const float scale = 3.f / median_depth;
  map_->SetMapScale(scale);

  float sigma = Sigma(depths);
  float sigma_scaled = sigma * scale;

  Frame reference_frame;
  for (int idx = 0; idx < initialization_results.current_keypoints.size();
       idx++) {
    cv::KeyPoint reference_keypoint =
        initialization_results.reference_keypoints[idx];
    cv::KeyPoint current_keypoint =
        initialization_results.current_keypoints[idx];

    Eigen::Vector3f reference_landmark_position =
        initialization_results.reference_landmark_positions[idx] * scale;
    Eigen::Vector3f current_landmark_position =
        initialization_results.current_landmark_positions[idx] * scale;

    ID mappoint_id = map_->CreateAndInsertMapPoint(reference_landmark_position,
                                                   reference_keypoint.class_id)
                         ->GetId();

    reference_frame.InsertObservation(reference_keypoint,
                                      reference_landmark_position, mappoint_id,
                                      TRACKED_WITH_3D);

    current_frame_->InsertObservation(current_keypoint,
                                      current_landmark_position, mappoint_id,
                                      TRACKED_WITH_3D);
  }
  reference_frame.MutableCameraTransformationWorld() = Sophus::SE3f();
  initialization_results.camera_transform_world.translation() =
      initialization_results.camera_transform_world.translation() * scale;
  current_frame_->MutableCameraTransformationWorld() =
      initialization_results.camera_transform_world;

  // Create KeyFrames from the frames.
  auto first_keyframe = make_shared<KeyFrame>(reference_frame);
  auto current_keyframe = make_shared<KeyFrame>(*current_frame_);

  // Insert KeyFrame in the map.
  map_->InsertKeyFrame(first_keyframe);
  map_->InsertKeyFrame(current_keyframe);

  map_->SetLastFrame(current_frame_);

  // Initialize regularization graph.
  map_->InitializeRegularizationGraph(sigma_scaled * 3);

  // Set reference image to the KLT tracker.
  klt_tracker_.SetReferenceImage(im, current_frame_->Keypoints());

  // Save MapPoint photometric information
  for (const auto& [mappoint_id, idx] : current_frame_->MapPointIdToIndex()) {
    LucasKanadeTracker::PhotometricInformation photometric_information =
        klt_tracker_.GetPhotometricInformationOfPoint(idx);

    map_->GetMapPoint(mappoint_id)
        ->SetPhotometricInformation(photometric_information);
  }

  tracking_status_ = TRACKING;
}

absl::flat_hash_set<ID> Tracking::TrackCameraAndDeformation(
    const cv::Mat& im, const cv::Mat& mask) {
  int tracked_before_klt =
      current_frame_->CountKeypointsWithStatus({TRACKED_WITH_3D});
  LOG(INFO) << "[KLT Tracking] Points before KLT: " << tracked_before_klt;

  const auto t_klt_start = std::chrono::steady_clock::now();
  DataAssociation(im, mask);

  int tracked_after_klt =
      current_frame_->CountKeypointsWithStatus({TRACKED_WITH_3D});
  int bad_features = 0, out_of_bounds = 0, bad_displacement = 0, low_ssim = 0;

  const auto& statuses = current_frame_->LandmarkStatuses();
  for (auto status : statuses) {
    if (status == BAD_FEATURE)
      bad_features++;
    else if (status == OUT_IMAGE_BOUNDARIES)
      out_of_bounds++;
    else if (status == BAD)
      bad_displacement++;
  }

  int total_failed = tracked_before_klt - tracked_after_klt;
  low_ssim = total_failed - bad_features - out_of_bounds - bad_displacement;
  if (low_ssim < 0) low_ssim = 0;

  const double survival_ratio =
      (tracked_before_klt > 0)
          ? static_cast<double>(tracked_after_klt) / tracked_before_klt
          : 0.0;

  LOG(INFO) << "[KLT Tracking] Results - Tracked: " << tracked_after_klt
            << " (was " << tracked_before_klt << ")"
            << " survival=" << survival_ratio
            << " | Failures: BadFeature=" << bad_features
            << " OutOfBounds=" << out_of_bounds
            << " BadDisp=" << bad_displacement << " LowSSIM=" << low_ssim
            << " KLT params: window=" << options_.klt_window_size
            << " levels=" << options_.klt_max_level
            << " iters=" << options_.klt_max_iters
            << " minSSIM=" << options_.klt_min_SSIM
            << " minEigTh=" << options_.klt_min_eig_th;

  const auto t_klt_done = std::chrono::steady_clock::now();
  CameraPoseEstimation();
  const auto t_pose_only_done = std::chrono::steady_clock::now();
  auto lost_ids = CameraPoseAndDeformationEstimation();
  const auto t_deform_done = std::chrono::steady_clock::now();

  LOG(INFO) << "[TRACK_PHASE_TIMING]"
            << " ms_klt="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   t_klt_done - t_klt_start)
                   .count()
            << " ms_pose_only="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   t_pose_only_done - t_klt_done)
                   .count()
            << " ms_deform_opt="
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   t_deform_done - t_pose_only_done)
                   .count();

  return lost_ids;
}

void Tracking::DataAssociation(const cv::Mat& im, const cv::Mat& mask) {
  klt_tracker_.Track(im, current_frame_->Keypoints(),
                     current_frame_->LandmarkStatuses(), true,
                     options_.klt_min_SSIM, mask);
}

void Tracking::CameraPoseEstimation() {
  // Apply motion model to get a first seed of the current camera pose.
  current_frame_->MutableCameraTransformationWorld() =
      motion_model_ * current_frame_->CameraTransformationWorld();

  previous_camera_transform_world_ =
      current_frame_->CameraTransformationWorld();

  // Do optimization.
  CameraPoseOptimization(*current_frame_, calibration_,
                         previous_camera_transform_world_);
}

absl::flat_hash_set<ID> Tracking::CameraPoseAndDeformationEstimation() {
  int tracked_3d = current_frame_->CountKeypointsWithStatus({TRACKED_WITH_3D});
  LOG(INFO) << "[Deformation Optimization] Starting with tracked_3d="
            << tracked_3d;

  // Do optimization.
  auto lost_mappoint_ids = CameraPoseAndDeformationOptimization(
      *current_frame_, map_, calibration_, previous_camera_transform_world_,
      map_->GetMapScale());

  // Update motion model.
  motion_model_ = current_frame_->CameraTransformationWorld() *
                  map_->GetLastFrame().CameraTransformationWorld().inverse();

  return lost_mappoint_ids;
}

void Tracking::KeyFrameInsertion(
    const cv::Mat& im, const absl::flat_hash_map<std::string, cv::Mat>& masks) {
  if (NeedNewKeyFrame()) {
    CreateNewKeyFrame(im, masks);
  }
}

bool Tracking::NeedNewKeyFrame() {
  if (n_images_from_last_keyframe_ >= options_.images_to_insert_keyframe) {
    n_images_from_last_keyframe_ = 0;
    return true;
  } else {
    n_images_from_last_keyframe_++;
    return false;
  }
}

void Tracking::CreateNewKeyFrame(
    const cv::Mat& im, const absl::flat_hash_map<std::string, cv::Mat>& masks) {
  // Extract new features.
  ExtractFeaturesInFrame(im, masks.at("Global"), *current_frame_);

  // Create Keyframe from the current frame.
  auto keyframe = make_shared<KeyFrame>(*current_frame_);

  // Insert KeyFrame in the map.
  map_->InsertKeyFrame(keyframe);

  // Update current frame.
  current_frame_->SetFromKeyFrame(keyframe);

  // Set new klt reference.
  // Depending on the type of sequence, the mask type used can be different.
  // SetKLTReference(im, *current_frame_, masks.at("BorderFilter"));
  SetKLTReference(im, *current_frame_, masks.at("Global"));
  // SetKLTReference(im, *current_frame_, cv::Mat());
  // SetKLTReference(im, *current_frame_, masks.at("PredefinedFilter"));
}

void Tracking::ExtractFeaturesInFrame(const cv::Mat& im, const cv::Mat& mask,
                                      Frame& frame) {
  vector<cv::KeyPoint> tracked_keypoints =
      frame.GetKeypointsWithStatus({TRACKED_WITH_3D, TRACKED});
  LOG(INFO) << "Extracting new features, already tracked keypoints: "
            << tracked_keypoints.size();
  vector<cv::KeyPoint> new_keypoints;
  ExtractFeatures(im, mask, tracked_keypoints, new_keypoints);
  LOG(INFO) << "Extracted " << new_keypoints.size() << " new keypoints";

  for (int idx = 0; idx < new_keypoints.size(); idx++) {
    frame.InsertObservation(new_keypoints[idx], Eigen::Vector3f::Zero(), 0,
                            TRACKED);
  }
}

void Tracking::SetKLTReference(const cv::Mat& im, Frame& frame,
                               const cv::Mat& mask) {
  klt_tracker_.SetReferenceImage(im, frame.Keypoints(), mask);

  // Update MapPoints photometric information.
  for (const auto& [mappoint_id, idx] : frame.MapPointIdToIndex()) {
    LucasKanadeTracker::PhotometricInformation photometric_information =
        klt_tracker_.GetPhotometricInformationOfPoint(idx);

    map_->GetMapPoint(mappoint_id)
        ->SetPhotometricInformation(photometric_information);
  }
}

void Tracking::PointReuse(const cv::Mat& im, const cv::Mat& mask,
                          absl::flat_hash_set<ID> lost_mappoint_ids) {
  const auto& all_mappoints = map_->GetMapPoints();
  const int frame_id = current_frame_->GetId();
  for (const auto& [mappoint_id, mappoint] : all_mappoints) {
    if (IsMapPointStale(mappoint_id, frame_id)) {
      continue;
    }

    if (!current_frame_->LandmarkPosition(mappoint_id).ok()) {
      // Project mappoint into the camera and check if it lies inside the image.
      Eigen::Vector3f landmark_position_seed = mappoint->GetLastWorldPosition();
      Eigen::Vector3f landmark_camera_position =
          current_frame_->CameraTransformationWorld() * landmark_position_seed;

      if (landmark_camera_position.z() < 0) {
        continue;
      }

      cv::Point2f projected_landmark =
          calibration_->Project(landmark_camera_position);

      if (projected_landmark.x >= 0 && projected_landmark.x < im.cols &&
          projected_landmark.y >= 0 && projected_landmark.y < im.rows) {
        lost_mappoint_ids.insert(mappoint_id);
      }
    }
  }

  if (lost_mappoint_ids.empty()) {
    return;
  }

  // Project candidates into the image.
  Frame frame_with_only_candidates;

  LucasKanadeTracker klt(
      cv::Size(options_.klt_window_size, options_.klt_window_size),
      options_.klt_max_level, options_.klt_max_iters, options_.klt_epsilon,
      options_.klt_min_eig_th);

  int candidates_in_image = 0;
  vector<cv::KeyPoint> keypoint_seeds;
  for (const auto& mappoint_id : lost_mappoint_ids) {
    if (IsMapPointStale(mappoint_id, frame_id)) {
      continue;
    }

    auto mappoint = map_->GetMapPoint(mappoint_id);
    if (!mappoint) {
      continue;
    }

    Eigen::Vector3f landmark_position_seed = mappoint->GetLastWorldPosition();
    Eigen::Vector3f landmark_camera_position =
        current_frame_->CameraTransformationWorld() * landmark_position_seed;
    cv::Point2f projected_landmark =
        calibration_->Project(landmark_camera_position);

    if (isnan(projected_landmark.x) || isnan(projected_landmark.y)) {
      LOG(FATAL) << "NaN found!";
    }

    if (projected_landmark.x >= 0 && projected_landmark.x < im.cols &&
        projected_landmark.y >= 0 && projected_landmark.y < im.rows) {
      cv::KeyPoint keypoint(projected_landmark, 1);
      frame_with_only_candidates.InsertObservation(
          keypoint, landmark_position_seed, mappoint_id, TRACKED_WITH_3D);

      // Set photometric information in the KLT.
      LucasKanadeTracker::PhotometricInformation photometric_information =
          map_->GetMapPoint(mappoint_id)->GetPhotometricInformation();
      klt.InsertPhotometricInformation(keypoint, photometric_information);

      keypoint_seeds.push_back(keypoint);

      candidates_in_image++;
    }
  }

  if (candidates_in_image == 0) {
    return;
  }

  klt.Track(im, frame_with_only_candidates.Keypoints(),
            frame_with_only_candidates.LandmarkStatuses(), true, 0.75, mask);

  auto& tracked_candidate_keypoints = frame_with_only_candidates.Keypoints();
  auto& tracked_candidate_landmarks =
      frame_with_only_candidates.LandmarkPositions();
  auto& tracked_candidate_statuses =
      frame_with_only_candidates.LandmarkStatuses();
  const auto& candidate_index_to_mappoint_id =
      frame_with_only_candidates.IndexToMapPointId();

  int reused_landmarks = 0;
  int reproj_rejected = 0;
  int updated_existing = 0;
  int inserted_new = 0;

  int klt_tracked = 0;
  for (int idx = 0; idx < tracked_candidate_keypoints.size(); idx++) {
    if (tracked_candidate_statuses[idx] != TRACKED_WITH_3D) {
      continue;
    }

    auto mappoint_id_it = candidate_index_to_mappoint_id.find(idx);
    if (mappoint_id_it == candidate_index_to_mappoint_id.end()) {
      continue;
    }

    klt_tracked++;
    cv::KeyPoint keypoint = tracked_candidate_keypoints[idx];
    Eigen::Vector3f landmark_position = tracked_candidate_landmarks[idx];
    ID mappoint_id = mappoint_id_it->second;

    keypoint.class_id = map_->GetMapPoint(mappoint_id)->GetKeyPointId();

    Eigen::Vector3f landmark_camera_position =
        current_frame_->CameraTransformationWorld() * landmark_position;
    cv::Point2f projected_landmark =
        calibration_->Project(landmark_camera_position);

    if (SquaredReprojectionError(projected_landmark, keypoint.pt) > 5.99) {
      reproj_rejected++;
      continue;
    }

    if (current_frame_->MapPointIdToIndex().contains(mappoint_id)) {
      const int idx_in_frame =
          current_frame_->MapPointIdToIndex().at(mappoint_id);

      current_frame_->Keypoints()[idx_in_frame] = keypoint;
      current_frame_->LandmarkPositions()[idx_in_frame] = landmark_position;
      current_frame_->LandmarkStatuses()[idx_in_frame] = TRACKED_WITH_3D;
      updated_existing++;
    } else {
      current_frame_->InsertObservation(keypoint, landmark_position,
                                        mappoint_id, TRACKED_WITH_3D);

      LucasKanadeTracker::PhotometricInformation photometric_information =
          map_->GetMapPoint(mappoint_id)->GetPhotometricInformation();
      klt_tracker_.InsertPhotometricInformation(keypoint,
                                                photometric_information);
      inserted_new++;
    }

    reused_landmarks++;
  }

  LOG(INFO) << "[POINT_REUSE] candidates_in_image=" << candidates_in_image
            << " klt_tracked=" << klt_tracked
            << " reproj_rejected=" << reproj_rejected
            << " reused=" << reused_landmarks
            << " updated_existing=" << updated_existing
            << " inserted_new=" << inserted_new;
}

void Tracking::UpdateTriangulatedPoints() {
  auto indices = current_frame_->GetIndexWithStatus({JUST_TRIANGULATED});

  for (auto index : indices) {
    LucasKanadeTracker::PhotometricInformation photometric_information =
        klt_tracker_.GetPhotometricInformationOfPoint(index);

    auto mappoint_id_or = current_frame_->GetMapPointIdForIndex(index);
    if (!mappoint_id_or.ok()) {
      continue;
    }
    ID mappoint_id = *mappoint_id_or;
    map_->GetMapPoint(mappoint_id)
        ->SetPhotometricInformation(photometric_information);

    // Update landmark status.
    current_frame_->LandmarkStatuses()[index] = TRACKED_WITH_3D;
  }
}

void Tracking::ResetMonocularInitializer() {
  MonocularMapInitializer::Options monocular_map_initializer_options;
  monocular_map_initializer_options.klt_window_size = 21;
  monocular_map_initializer_options.klt_max_level = 4;
  monocular_map_initializer_options.klt_max_iters = 10;
  monocular_map_initializer_options.klt_epsilon = 0.0001;
  monocular_map_initializer_options.klt_min_eig_th = 0.0001;
  monocular_map_initializer_options.klt_min_SSIM = 0.5;
  monocular_map_initializer_options.rigid_initializer_max_features = 4000;
  monocular_map_initializer_options.rigid_initializer_min_sample_set_size = 8;
  monocular_map_initializer_options.rigid_initializer_min_parallax = 0.999;
  monocular_map_initializer_options.rigid_initializer_epipolar_threshold =
      0.005;
  monocular_map_initializer_ = make_unique<MonocularMapInitializer>(
      monocular_map_initializer_options, feature_extractor_, calibration_,
      image_visualizer_);
  LOG(INFO) << "[LOST] MonocularMapInitializer reset for re-bootstrap";
}

bool Tracking::LostModeBootstrap(const cv::Mat& im, const cv::Mat& mask) {
  auto init_status = monocular_map_initializer_->ProcessNewImage(im, mask);
  if (!init_status.ok()) {
    LOG(INFO) << "[LOST] Bootstrap pending: " << init_status.status().message();
    return false;
  }

  auto& r = *init_status;

  // Compute per-frame scale the same way as initial map setup (target median
  // depth = 3.0 world units).
  vector<float> depths;
  depths.reserve(r.current_landmark_positions.size());
  for (const auto& pos : r.current_landmark_positions) {
    depths.push_back(pos.z());
  }
  if (depths.empty()) return false;

  const int med_idx = depths.size() / 2;
  nth_element(depths.begin(), depths.begin() + med_idx, depths.end());
  const float median_depth = depths[med_idx];
  if (median_depth <= 0.f) {
    LOG(WARNING) << "[LOST] Bootstrap skipped: non-positive median depth";
    return false;
  }
  const float scale = 3.f / median_depth;
  map_->SetMapScale(scale);
  float sigma = Sigma(depths);

  // Scale the relative translation (essential matrix gives unit-less
  // direction).
  r.camera_transform_world.translation() *= scale;

  // Build the new current frame from bootstrap results.
  // Landmark positions from the monocular initializer are in "init-world" =
  // reference-camera coordinates. We compose with pose_at_lost_entry_ to get
  // actual world coordinates.
  //   P_world  = pose_at_lost_entry_.inverse() * P_init_world
  //   T_current_world = T_current_init_world * pose_at_lost_entry_
  current_frame_->Clear();

  const int n_bootstrap_points =
      std::min(static_cast<int>(r.current_keypoints.size()),
               static_cast<int>(r.reference_landmark_positions.size()));
  for (int idx = 0; idx < n_bootstrap_points; idx++) {
    Eigen::Vector3f p_init_world = r.reference_landmark_positions[idx] * scale;
    Eigen::Vector3f p_world = pose_at_lost_entry_.inverse() * p_init_world;

    ID mappoint_id = map_->CreateAndInsertMapPoint(
                             p_world, r.reference_keypoints[idx].class_id)
                         ->GetId();

    current_frame_->InsertObservation(r.current_keypoints[idx], p_world,
                                      mappoint_id, TRACKED_WITH_3D);
  }

  // Set the new current-frame camera pose in actual world coordinates.
  current_frame_->MutableCameraTransformationWorld() =
      r.camera_transform_world * pose_at_lost_entry_;

  // Insert a new keyframe so mapping can continue normally.
  auto keyframe = make_shared<KeyFrame>(*current_frame_);
  map_->InsertKeyFrame(keyframe);
  current_frame_->SetFromKeyFrame(keyframe);

  // Commit the frame and initialise the regularisation graph with the new
  // batch of points so deformation optimisation has edges from the start.
  map_->SetLastFrame(current_frame_);
  map_->InitializeRegularizationGraph((sigma * scale) * 3.f);

  // Set the KLT reference on the new active keypoints.
  klt_tracker_.SetReferenceImage(im, current_frame_->Keypoints());
  for (const auto& [mp_id, frame_idx] : current_frame_->MapPointIdToIndex()) {
    auto ph = klt_tracker_.GetPhotometricInformationOfPoint(frame_idx);
    map_->GetMapPoint(mp_id)->SetPhotometricInformation(ph);
  }

  // Seed the LRU cache for the new map points.
  const int frame_id = static_cast<int>(current_frame_->GetId());
  MarkTrackedMapPointsSeen(frame_id);

  // Reset the keyframe insertion counter so we don't immediately insert
  // another keyframe on the very next tracking frame.
  n_images_from_last_keyframe_ = 0;

  LOG(WARNING) << "[LOST] Bootstrap completed: triangulated="
               << n_bootstrap_points << " new map points";
  return true;
}

void Tracking::MarkTrackedMapPointsSeen(const int frame_id) {
  const auto tracked_mappoint_ids = current_frame_->GetMapPointsIdsWithStatus(
      {TRACKED_WITH_3D, JUST_TRIANGULATED});
  for (const ID mappoint_id : tracked_mappoint_ids) {
    mappoint_last_seen_frame_[mappoint_id] = frame_id;
  }
}

void Tracking::SeedMapPointLastSeen(const int frame_id) {
  const auto& all_mappoints = map_->GetMapPoints();
  for (const auto& [mappoint_id, mappoint] : all_mappoints) {
    if (!mappoint_last_seen_frame_.contains(mappoint_id)) {
      mappoint_last_seen_frame_[mappoint_id] = frame_id;
    }
  }
}

void Tracking::PruneStaleMapPointCache(const int frame_id) {
  // Single pass: seed new map points AND collect stale ones.
  // This replaces the separate SeedMapPointLastSeen O(M) scan that previously
  // ran every frame before this function.
  std::vector<ID> stale_mappoint_ids;
  stale_mappoint_ids.reserve(mappoint_last_seen_frame_.size());

  for (const auto& [mappoint_id, mappoint] : map_->GetMapPoints()) {
    auto it = mappoint_last_seen_frame_.find(mappoint_id);
    if (it == mappoint_last_seen_frame_.end()) {
      // New map point: seed its last-seen timestamp.
      mappoint_last_seen_frame_[mappoint_id] = frame_id;
    } else if (frame_id - it->second > options_.stale_mappoint_max_age_frames) {
      stale_mappoint_ids.push_back(mappoint_id);
    }
  }

  int removed_from_map = 0;
  for (const ID mappoint_id : stale_mappoint_ids) {
    if (current_frame_->MapPointIdToIndex().contains(mappoint_id)) {
      continue;
    }

    map_->RemoveMapPoint(mappoint_id);
    mappoint_last_seen_frame_.erase(mappoint_id);
    removed_from_map++;
  }

  if (removed_from_map > 0) {
    LOG(INFO) << "[MAP_LRU] removed_stale_mappoints=" << removed_from_map
              << " frame=" << frame_id;
  }
}

bool Tracking::IsMapPointStale(const ID mappoint_id, const int frame_id) const {
  if (!mappoint_last_seen_frame_.contains(mappoint_id)) {
    return false;
  }

  return frame_id - mappoint_last_seen_frame_.at(mappoint_id) >
         options_.stale_mappoint_max_age_frames;
}

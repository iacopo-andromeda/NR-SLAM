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

#include "tracking/tracking.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "features/shi_tomasi_cv.h"
#include "optimization/g2o_optimization.h"
#include "utilities/dbscan.h"
#include "utilities/geometry_toolbox.h"
#include "utilities/statistics_toolbox.h"

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

MonocularMapInitializer::Options BuildMonocularInitializerOptions(
    const Tracking::Options& options) {
  MonocularMapInitializer::Options monocular_map_initializer_options;
  monocular_map_initializer_options.klt_window_size =
      options.monocular_init_klt_window_size;
  monocular_map_initializer_options.klt_max_level =
      options.monocular_init_klt_max_level;
  monocular_map_initializer_options.klt_max_iters =
      options.monocular_init_klt_max_iters;
  monocular_map_initializer_options.klt_epsilon =
      options.monocular_init_klt_epsilon;
  monocular_map_initializer_options.klt_min_eig_th =
      options.monocular_init_klt_min_eig_th;
  monocular_map_initializer_options.klt_min_SSIM =
      options.monocular_init_klt_min_SSIM;

  monocular_map_initializer_options.rigid_initializer_max_features =
      options.monocular_init_rigid_max_features;
  monocular_map_initializer_options.rigid_initializer_min_parallax =
      options.monocular_init_rigid_min_parallax;
  monocular_map_initializer_options.rigid_initializer_epipolar_threshold =
      options.monocular_init_rigid_epipolar_threshold;
  return monocular_map_initializer_options;
}
}  // namespace

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

  MonocularMapInitializer::Options monocular_map_initializer_options =
      BuildMonocularInitializerOptions(options_);

  monocular_map_initializer_ = make_unique<MonocularMapInitializer>(
      monocular_map_initializer_options, feature_extractor_, calibration_,
      image_visualizer_);
}

void Tracking::TrackImage(
    const cv::Mat& im, const absl::flat_hash_map<std::string, cv::Mat>& masks,
    const Sophus::SE3f& external_camera_pose) {
  map_->SetAllMappointsToNonActive();

  if (map_->IsEmpty()) {
    // If map is not initialized, perform map initialization.

    // For stereo experiment purposes.
    // StereoMapInitialization(im, additional_im, masks.at("Global"), im_clahe);

    // Depending on the type of sequence, the mask type used can be different.
    MonocularMapInitialization(im, masks.at("Global"));
    // MonocularMapInitialization(im, masks.at("PredefinedFilter"), im_clahe);
  } else {
    // Update photometric info for points triangulated by the mapping in the
    // last frame.
    UpdateTriangulatedPoints();

    // Otherwise perform normal tracking.
    // Depending on the type of sequence, the mask type used can be different.
    // absl::flat_hash_set<ID> lost_mappoint_ids = TrackCameraAndDeformation(im,
    // masks.at("BorderFilter"));
    absl::flat_hash_set<ID> lost_mappoint_ids =
        TrackCameraAndDeformation(im, masks.at("Global"));

    // Point reuse.
    PointReuse(im, masks.at("Global"), lost_mappoint_ids);

    if (current_frame_->GetKeypointsWithStatus({TRACKED_WITH_3D}).size() <
        options_.min_tracked_points_abort) {
      LOG(WARNING) << "Too few tracked points, aborting tracking";
      Clear();
      return;
    }

    // KeyFrame insertion.
    KeyFrameInsertion(im, masks);

    // Insert frame to the temporal buffer.
    map_->SetLastFrame(current_frame_);

    // Draw current frame.
    if (image_visualizer_) {
      image_visualizer_->DrawCurrentFrame(*current_frame_);
      image_visualizer_->DrawRegularizationGraph(
          *current_frame_, *(map_->GetRegularizationGraph()));
      image_visualizer_->DrawFeatures(current_frame_->Keypoints());
    }
  }
}

void Tracking::Clear() {
  tracking_status_ = NOT_INITIALIZED;
  monocular_map_initializer_->Clear();
  current_frame_ = make_shared<Frame>();
  map_->Clear();

  // klt will be cleared at MonocularMapInitialization
  LOG(INFO) << "Tracker reset: " << map_->IsEmpty();
}

Tracking::TrackingStatus Tracking::GetTrackingStatus() const {
  return tracking_status_;
}

void Tracking::ExtractFeatures(
    const cv::Mat& im, const cv::Mat& mask,
    const std::vector<cv::KeyPoint>& extracted_keypoints,
    std::vector<cv::KeyPoint>& new_keypoints) {
  // Extract features.
  feature_extractor_->Extract(im, mask, new_keypoints);

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
  reference_frame.MutableCameraTransformationWorld() =
      Sophus::SE3f();  // problem: The reference frame's keypoints come from
                       // different timestamps, so the reference frame's camera
                       // pose is not exactly identity. The code does some
                       // tricks with camera_transform_world

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
  DataAssociation(im, mask);

  CameraPoseEstimation();

  auto lost_ids = CameraPoseAndDeformationEstimation();

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
  } else {
    n_images_from_last_keyframe_++;
  }
}

bool Tracking::NeedNewKeyFrame() {
  if (n_images_from_last_keyframe_ >= options_.images_to_insert_keyframe) {
    return true;
  } else {
    return false;
  }
}

void Tracking::CreateNewKeyFrame(
    const cv::Mat& im, const absl::flat_hash_map<std::string, cv::Mat>& masks) {
  // Extract new features.
  ExtractFeaturesInFrame(
      im, masks.at("Global"),
      *current_frame_);  // TODO: likely issue. why do we need to re-extract
                         // features? they should already be in the
                         // current_frame

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

  n_images_from_last_keyframe_ = 0;
}

void Tracking::ExtractFeaturesInFrame(const cv::Mat& im, const cv::Mat& mask,
                                      Frame& frame) {
  vector<cv::KeyPoint> keypoints =
      frame.GetKeypointsWithStatus({TRACKED_WITH_3D, TRACKED});

  LOG(INFO) << "Extracting new features, already tracked keypoints: "
            << keypoints.size();
  feature_extractor_->Extract(im, mask, keypoints);

  LOG(INFO) << "Extracted " << keypoints.size() << " new keypoints";

  for (int idx = 0; idx < keypoints.size(); idx++) {
    frame.InsertObservation(keypoints[idx], Eigen::Vector3f::Zero(), 0,
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
  // Simulates an extra image tracking step, only with the mappoint
  // candidates that are currently not tracked but could be in the current
  // image. This is done to find more matches and increase tracking robustness.

  // 1. Project all non-active mappoints into the image and find those that lie
  // inside the image. These are the candidates for reuse.
  const auto& all_mappoints = map_->GetMapPoints();
  for (const auto& [mappoint_id, mappoint] : all_mappoints) {
    if (!current_frame_->LandmarkPosition(mappoint_id).ok()) {
      // Project mappoint into the camera and check if it lies inside the image.
      Eigen::Vector3f landmark_position_seed = mappoint->GetLastWorldPosition();
      Eigen::Vector3f landmark_camera_position =
          current_frame_->CameraTransformationWorld() * landmark_position_seed;

      if (landmark_camera_position.z() < 0) {
        // Landmark is behind the camera, skip it.
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

  // 2. Create a temporary frame with only the candidate keypoints, and set the
  // photometric information in the KLT for those keypoints using the
  // information stored in the map.
  int candidates_in_image = 0;
  vector<cv::KeyPoint> keypoint_seeds;
  Frame frame_with_only_candidates;
  LucasKanadeTracker klt(
      cv::Size(options_.klt_window_size, options_.klt_window_size),
      options_.klt_max_level, options_.klt_max_iters, options_.klt_epsilon,
      options_.klt_min_eig_th);
  for (const auto& mappoint_id : lost_mappoint_ids) {
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
      LOG(FATAL) << "NaN found! error in the calibration";
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
      // this is used in order to be able to later track points "from the past"

      keypoint_seeds.push_back(keypoint);

      candidates_in_image++;
    }
  }

  if (candidates_in_image == 0) {
    return;
  }

  // 3. Track the candidates with the KLT.
  klt.Track(im, frame_with_only_candidates.Keypoints(),
            frame_with_only_candidates.LandmarkStatuses(), true,
            options_.klt_min_SSIM, mask);

  // 4. For the candidates that are successfully tracked, check their
  // reprojection error. If the reprojection error is low, insert them into the
  // current frame as tracked with 3D. This way we can reuse points that are not
  // tracked by the normal tracking process, but are still visible in the
  // current image.  Frame frame_with_only_candidates;
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

    ID mappoint_id = current_frame_->IndexToMapPointId().at(index);
    map_->GetMapPoint(mappoint_id)
        ->SetPhotometricInformation(photometric_information);

    // Update landmark status.
    current_frame_->LandmarkStatuses()[index] = TRACKED_WITH_3D;
  }
}

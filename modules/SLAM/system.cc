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

#include "system.h"

#include <chrono>

#include "absl/log/log.h"
#include "utilities/landmark_status.h"

using namespace std;

System::System(const string settings_file_path) {
  // Output welcome message
  LOG(INFO).NoPrefix() << "NR-SLAM Copyright (C) Copyright (C) 2022-2023 Juan "
                          "J. Gómez Rodríguez, José M.M. Montiel and Juan D. "
                          "Tardós, University of Zaragoza.";
  LOG(INFO).NoPrefix() << "This program comes with ABSOLUTELY NO WARRANTY;";
  LOG(INFO).NoPrefix()
      << "This is free software, and you are welcome to redistribute it";
  LOG(INFO).NoPrefix() << "under certain conditions. See LICENSE.txt.";

  settings_ = make_unique<Settings>(settings_file_path);
  LOG(INFO) << *settings_;

  // Initialize image processing stuff
  clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));
  masker_ = settings_->getMasker();

  // Create map
  Map::Options map_options;
  map_options.max_temporal_buffer_size = 20;
  map_options.triangulation_track_lookback_frames =
      settings_->GetTriangulationTrackLookbackFrames();
  map_options.min_mappoint_distance = settings_->GetMinMapPointDistance();
  map_ = make_shared<Map>(map_options);

  // StereoLucasKanade::Options stereo_matcher_options;
  // stereo_matcher_options.klt_window_size = 21;
  // stereo_matcher_options.klt_max_level = 4;
  // stereo_matcher_options.klt_max_iters = 10;
  // stereo_matcher_options.klt_epsilon = 0.0001;
  // stereo_matcher_options.klt_min_eig_th = 0.0001;
  // stereo_matcher_options.klt_min_SSIM = 0.5;

  stereo_pattern_matcher_ = make_shared<StereoPatternMatching>(
      settings_->getCalibration(), settings_->getBf());

  // Initialize map visualizer and launch it in a different thread
  MapVisualizer::Options map_visualizer_options;
  map_visualizer_options.camera_size_ = settings_->GetCameraSize();
  map_visualizer_options.initial_left_view_ =
      settings_->GetLeftMapVisualizationView().matrix();
  map_visualizer_options.initial_right_view_ =
      settings_->GetRightMapVisualizationView().matrix();
  map_visualizer_options.render_save_path = settings_->GetMapVisualizerPath();

  map_visualizer_ = make_unique<MapVisualizer>(map_visualizer_options, map_);

  map_visualizer_thread_ =
      make_unique<thread>(&MapVisualizer::Run, map_visualizer_.get());

  // Initialize image visualizer.
  ImageVisualizer::Options image_visualizer_options;
  image_visualizer_options.wait_for_user_button = !settings_->GetAutoplay();
  image_visualizer_options.image_save_path =
      settings_->GetImageVisualizerPath();
  image_visualizer_ = make_shared<ImageVisualizer>(image_visualizer_options);

  // Initialize Tracking.
  Tracking::Options tracking_options;
  tracking_options.klt_window_size = settings_->GetKltWindowSize();
  tracking_options.klt_max_level = settings_->GetKltMaxLevel();
  tracking_options.klt_max_iters = settings_->GetKltMaxIters();
  tracking_options.klt_epsilon = settings_->GetKltEpsilon();
  tracking_options.klt_min_eig_th = settings_->GetKltMinEigTh();
  tracking_options.klt_min_SSIM = settings_->GetKltMinSSIM();
  tracking_options.images_to_insert_keyframe =
      settings_->GetImagesToInsertKeyframe();
  tracking_options.stale_mappoint_max_age_frames =
      settings_->GetStaleMapPointMaxAgeFrames();
  tracking_options.min_tracked_points_abort =
      settings_->GetMinTrackedPointsAbort();
  tracking_options.lost_recovery_grace_frames =
      settings_->GetLostRecoveryGraceFrames();
  tracking_options.lost_recovery_min_tracked_points =
      settings_->GetLostRecoveryMinTrackedPoints();
  tracking_options.feature_max_corners = settings_->GetFeatureMaxCorners();
  tracking_options.feature_quality_level = settings_->GetFeatureQualityLevel();
  tracking_options.feature_min_distance = settings_->GetFeatureMinDistance();
  tracking_options.lost_bootstrap_frame_stride =
      settings_->GetLostBootstrapFrameStride();

  // Time profiler.
  time_profiler_ = make_unique<TimeProfiler>();

  tracker_ =
      make_unique<Tracking>(tracking_options, map_, settings_->getCalibration(),
                            image_visualizer_, time_profiler_.get());

  // Initialize Mapping.
  Mapping::Options mapping_options;
  mapper_ = make_unique<Mapping>(map_, settings_->getCalibration(),
                                 mapping_options, time_profiler_.get());

  // Initialize frame evaluator.
  FrameEvaluator::Options frame_evaluator_options;
  frame_evaluator_options.results_file_path = settings_->GetEvaluationPath();
  frame_evaluator_options.precomputed_depth_ = true;
  frame_evaluator_ = make_unique<FrameEvaluator>(
      frame_evaluator_options, settings_->getCalibration(),
      stereo_pattern_matcher_, map_visualizer_.get());

  // Initialize performance logger.
  perf_log_csv_path_ = "slam_performance.csv";
  perf_logger_ = make_unique<PerformanceLogger>();
  LOG(INFO) << "PerformanceLogger ready. CSV will be saved to: "
            << perf_log_csv_path_;
}

System::~System() {
  // Print and save performance summary before shutting down.
  if (perf_logger_) {
    perf_logger_->PrintSummary();
    perf_logger_->SaveToCSV(perf_log_csv_path_);
  }

  // Send signal to the visualizer to finish
  map_visualizer_->SetFinish();

  // Wait until is done
  map_visualizer_thread_->join();
}

void System::TrackImage(const cv::Mat& im) {
  const auto t_frame_start = std::chrono::steady_clock::now();
  const uint64_t frame_id = frame_counter_++;

  // Preprocess image.
  cv::Mat im_gray;
  cv::Mat processed_image = ImageProcessing(im, im_gray);

  // Insert image in the image visualizer.
  if (image_visualizer_)
    image_visualizer_->SetCurrentImage(im, processed_image);

  // Generate image mask.
  auto masks = masker_->GetAllMasks(processed_image);

  // --- Tracking ---
  const auto t_tracking_start = std::chrono::steady_clock::now();
  tracker_->TrackImage(processed_image, masks);
  const auto t_tracking_end = std::chrono::steady_clock::now();

  // --- Mapping ---
  const auto t_mapping_start = std::chrono::steady_clock::now();
  mapper_->DoMapping();
  const auto t_mapping_end = std::chrono::steady_clock::now();

  // Draw images.
  if (image_visualizer_) image_visualizer_->UpdateWindows();

  const auto t_frame_end = std::chrono::steady_clock::now();

  // --- Collect per-frame performance stats ---
  auto to_ms = [](auto a, auto b) -> double {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };

  PerformanceLogger::FrameStats stats;
  stats.frame_id = frame_id;

  // Tracking status string.
  const auto tracking_status = tracker_->GetTrackingStatus();
  switch (tracking_status) {
    case Tracking::TRACKING:
      stats.tracking_status = "TRACKING";
      break;
    case Tracking::LOST:
      stats.tracking_status = "LOST";
      break;
    case Tracking::NOT_INITIALIZED:
      stats.tracking_status = "NOT_INITIALIZED";
      break;
  }

  // Feature counts are only meaningful for frames that finished in TRACKING.
  // In LOST/NOT_INITIALIZED, map_->GetMutableLastFrame() may refer to an older
  // frame and would otherwise skew per-frame means with stale values.
  if (tracking_status == Tracking::TRACKING) {
    if (auto last_frame = map_->GetMutableLastFrame()) {
      stats.n_keypoints = static_cast<int>(last_frame->Keypoints().size());
      stats.n_tracked_3d = static_cast<int>(
          last_frame->GetKeypointsWithStatus({TRACKED_WITH_3D}).size());
    }
  } else {
    stats.n_keypoints = 0;
    stats.n_tracked_3d = 0;
  }
  stats.n_map_points = static_cast<int>(map_->GetMapPoints().size());

  stats.ms_tracking = to_ms(t_tracking_start, t_tracking_end);
  stats.ms_mapping = to_ms(t_mapping_start, t_mapping_end);
  stats.ms_total = to_ms(t_frame_start, t_frame_end);
  stats.ms_other = stats.ms_total - stats.ms_tracking - stats.ms_mapping;
  if (stats.ms_other < 0.0) {
    stats.ms_other = 0.0;
  }

  LOG(INFO) << "[PERF] frame=" << frame_id
            << " status=" << stats.tracking_status
            << " kps=" << stats.n_keypoints << " kps_3d=" << stats.n_tracked_3d
            << " map_pts=" << stats.n_map_points
            << " ms_track=" << stats.ms_tracking
            << " ms_map=" << stats.ms_mapping << " ms_other=" << stats.ms_other
            << " ms_total=" << stats.ms_total;

  perf_logger_->LogFrame(stats);
}

// void System::TrackImageWithStereo(const cv::Mat& im_left,
//                                   const cv::Mat& im_right) {
//   // Preprocess images.
//   cv::Mat im_gray_left, im_gray_right;
//   cv::Mat processed_image_left = ImageProcessing(im_left, im_gray_left);
//   cv::Mat processed_image_right = ImageProcessing(im_right, im_gray_right);

//   // Insert image in the image visualizer.
//   image_visualizer_->SetCurrentImage(im_left, processed_image_left);

//   // Generate image mask.
//   auto masks = masker_->GetAllMasks(im_gray_left);

//   // Perform tracking.
//   tracker_->TrackImage(im_gray_left, masks, im_gray_right,
//                        processed_image_left);

//   // Perform mapping.
//   mapper_->DoMapping();

//   // Evaluate reconstruction.
//   if (false && tracker_->GetTrackingStatus() == Tracking::TRACKING) {
//     frame_evaluator_->EvaluateFrameReconstruction(
//         *(map_->GetMutableLastFrame()), im_gray_left, im_gray_right);
//     frame_evaluator_->SaveResultsToFile();
//   }

//   // Draw images.
//   image_visualizer_->UpdateWindows();
// }

// void System::TrackImageWithDepth(const cv::Mat& im_left,
//                                  const cv::Mat& im_depth) {
//   // Preprocess images.
//   cv::Mat im_gray_left, im_gray_right;
//   cv::Mat processed_image_left = ImageProcessing(im_left, im_gray_left);

//   // Insert image in the image visualizer.
//   image_visualizer_->SetCurrentImage(im_left, processed_image_left);

//   // Generate image mask.
//   auto masks = masker_->GetAllMasks(im_gray_left);

//   // Perform tracking.
//   tracker_->TrackImage(im_gray_left, masks, im_gray_right,
//                        processed_image_left);

//   // Perform mapping.
//   mapper_->DoMapping();

//   // Evaluate reconstruction.
//   if (true && tracker_->GetTrackingStatus() == Tracking::TRACKING) {
//     frame_evaluator_->EvaluateFrameReconstruction(
//         *(map_->GetMutableLastFrame()), im_gray_left, im_depth);
//     frame_evaluator_->SaveResultsToFile();
//   }

//   // Draw images.
//   image_visualizer_->UpdateWindows();
// }

cv::Mat System::ImageProcessing(const cv::Mat& im, cv::Mat& im_gray) {
  cv::Mat processed_image;

  // Convert to grayscale.
  cv::cvtColor(im, processed_image, cv::COLOR_RGB2GRAY);

  im_gray = processed_image.clone();

  // Apply Clahe to the image.
  clahe_->apply(processed_image, processed_image);

  return processed_image;
}

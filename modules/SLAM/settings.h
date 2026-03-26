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

#ifndef NRSLAM_SETTINGS_H
#define NRSLAM_SETTINGS_H

#include <memory>

#include "calibration/camera_model.h"
#include "masking/masker.h"
#include "sophus/se3.hpp"

class Settings {
 public:
  /*
   * Default constructor: sets everything to default values
   */
  Settings();

  /*
   * Constructor reading parameters from file
   */
  Settings(const std::string& configFile);

  /*
   * Ostream operator overloading to dump settings to the terminal
   */
  friend std::ostream& operator<<(std::ostream& output, const Settings& D);

  // Getter methods
  std::shared_ptr<CameraModel> getCalibration();
  float getBf();

  std::shared_ptr<Masker> getMasker();

  Sophus::SE3f GetLeftMapVisualizationView();
  Sophus::SE3f GetRightMapVisualizationView();

  float GetCameraSize();

  bool GetAutoplay();

  std::string GetMapVisualizerPath();

  std::string GetImageVisualizerPath();

  std::string GetEvaluationPath();

  // Override the three output directories derived from a single root.
  // Equivalent to setting MapVisualizer.save_path, ImageVisualizer.save_path,
  // and Evaluation.save_path in the YAML, but takes precedence over them.
  void OverrideOutputDir(const std::string& dir);

  int GetLostBootstrapFrameStride();

  int GetTriangulationTrackLookbackFrames();

  float GetMinMapPointDistance();

  // KLT / Tracking parameters
  int GetKltWindowSize();
  int GetKltMaxLevel();
  int GetKltMaxIters();
  float GetKltEpsilon();
  float GetKltMinEigTh();
  float GetKltMinSSIM();

  // Keyframe & map-point parameters
  int GetImagesToInsertKeyframe();
  int GetStaleMapPointMaxAgeFrames();
  int GetMinTrackedPointsAbort();
  int GetLostRecoveryGraceFrames();
  int GetLostRecoveryMinTrackedPoints();

  // Monocular map initializer parameters
  int GetMonocularInitKltWindowSize();
  int GetMonocularInitKltMaxLevel();
  int GetMonocularInitKltMaxIters();
  float GetMonocularInitKltEpsilon();
  float GetMonocularInitKltMinEigTh();
  float GetMonocularInitKltMinSSIM();
  int GetMonocularInitRigidMaxFeatures();
  float GetMonocularInitRigidMinParallax();
  float GetMonocularInitRigidEpipolarThreshold();

  // Feature extraction parameters
  int GetFeatureMaxCorners();
  float GetFeatureQualityLevel();
  float GetFeatureMinDistance();

 private:
  // Camera parameters
  std::shared_ptr<CameraModel>
      calibration_;  // Geometric calibration with projection and unprojection
                     // functions
  float bf_;         // baseline times fx

  std::shared_ptr<Masker> masker_;

  Sophus::SE3f left_map_view_, right_map_view_;

  float camera_size_;

  bool autoplay_;

  std::string map_visualizer_save_path_;

  std::string image_visualizer_save_path_;

  std::string evaluation_save_path_;

  int lost_bootstrap_frame_stride_ = 1;
  int triangulation_track_lookback_frames_ = 5;
  float min_mappoint_distance_ = 0.02f;

  // KLT / Tracking defaults (match system.cc hardcoded values)
  int klt_window_size_ = 21;
  int klt_max_level_ = 4;
  int klt_max_iters_ = 10;
  float klt_epsilon_ = 0.0001f;
  float klt_min_eig_th_ = 0.0001f;
  float klt_min_SSIM_ = 0.65f;

  // Keyframe & map-point defaults
  int images_to_insert_keyframe_ = 5;
  int stale_mappoint_max_age_frames_ = 30;
  int min_tracked_points_abort_ = 10;
  int lost_recovery_grace_frames_ = 5;
  int lost_recovery_min_tracked_points_ = 3;

  // Monocular map initializer defaults (matching previous hardcoded values in
  // tracking.cc)
  int monocular_init_klt_window_size_ = 21;
  int monocular_init_klt_max_level_ = 4;
  int monocular_init_klt_max_iters_ = 10;
  float monocular_init_klt_epsilon_ = 0.0001f;
  float monocular_init_klt_min_eig_th_ = 0.0001f;
  float monocular_init_klt_min_SSIM_ = 0.5f;
  int monocular_init_rigid_max_features_ = 4000;
  float monocular_init_rigid_min_parallax_ = 0.999f;
  float monocular_init_rigid_epipolar_threshold_ = 0.005f;

  // Feature extraction defaults
  int feature_max_corners_ = 1000;
  float feature_quality_level_ = 0.1f;
  float feature_min_distance_ = 7.0f;

  template <typename T>
  T readParameter(cv::FileStorage& fSettings, const std::string& name,
                  bool& found, const bool required = true) {
    cv::FileNode node = fSettings[name];
    if (node.empty()) {
      if (required) {
        std::cerr << name << " required parameter does not exist, aborting..."
                  << std::endl;
        exit(-1);
      } else {
        std::cerr << name << " optional parameter does not exist..."
                  << std::endl;
        found = false;
        return T();
      }

    } else {
      found = true;
      return (T)node;
    }
  }
};

#endif  // NRSLAM_SETTINGS_H

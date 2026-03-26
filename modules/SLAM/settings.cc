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

#include "SLAM/settings.h"

#include "absl/log/log.h"
#include "calibration/camera_parameters.h"
#include "calibration/distorted_pin_hole.h"
#include "calibration/kannala_brandt_8.h"
#include "calibration/pin_hole.h"
#include "utilities/types_conversions.h"

using namespace std;

template <>
Sophus::SE3f Settings::readParameter<Sophus::SE3f>(cv::FileStorage& fSettings,
                                                   const std::string& name,
                                                   bool& found,
                                                   const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      LOG(ERROR) << name << " required parameter does not exist, aborting...";
      exit(-1);
    } else {
      LOG(WARNING) << name << " optional parameter does not exist, aborting...";
      found = false;
      return Sophus::SE3f();
    }
  } else {
    found = true;
    cv::Mat cvT = node.mat();

    // Convert to Sophus
    Sophus::SE3f sT = cvToSophus(cvT);

    return sT;
  }
}

template <>
bool Settings::readParameter<bool>(cv::FileStorage& fSettings,
                                   const std::string& name, bool& found,
                                   const bool required) {
  cv::FileNode node = fSettings[name];
  if (node.empty()) {
    if (required) {
      LOG(ERROR) << name << " required parameter does not exist, aborting...";
      exit(-1);
    } else {
      LOG(WARNING) << name << " optional parameter does not exist, aborting...";
      found = false;
      return false;
    }
  } else {
    found = true;
    int value = (int)node;

    // Convert to bool
    bool return_value = (bool)value;

    return return_value;
  }
}

Settings::Settings() {}

Settings::Settings(const std::string& configFile) {
  // Open settings file
  cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
  if (!fSettings.isOpened()) {
    LOG(ERROR) << "[ERROR]: could not open configuration file at: "
               << configFile;
    exit(-1);
  }

  // Read camera model
  string cameraModel = (string)fSettings["Camera.model"];
  if (cameraModel == "PinHole") {
    // Read camera calibration
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    calibration_ = std::make_shared<PinHole>(PinholeParameters{fx, fy, cx, cy});
  } else if (cameraModel == "KannalaBrandt8") {
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    float k0 = fSettings["Camera.k0"];
    float k1 = fSettings["Camera.k1"];
    float k2 = fSettings["Camera.k2"];
    float k3 = fSettings["Camera.k3"];

    calibration_ = std::make_shared<KannalaBrandt8>(
        KannalaBrandt8Parameters{fx, fy, cx, cy, k0, k1, k2, k3});
  } else if (cameraModel == "DistortedPinHole") {
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    // 5 distortion coefficients
    float k1 = fSettings["Camera.k1"];
    float k2 = fSettings["Camera.k2"];
    float p1 = fSettings["Camera.p1"];
    float p2 = fSettings["Camera.p2"];
    float k3 = fSettings["Camera.k3"];

    calibration_ = std::make_shared<DistortedPinHole>(
        DistortedPinholeParameters{fx, fy, cx, cy, k1, k2, p1, p2, k3});
  } else {
    LOG(ERROR) << "Error: " << cameraModel << " not known";
    exit(-1);
  }

  bf_ = fSettings["Stereo.bf"];

  // Camera drawing size.
  camera_size_ = (float)fSettings["Visualization.cameraSize"];

  // Read filter for mask generation
  string filterFile = (string)fSettings["Masking.filterFile"];
  masker_ = std::make_shared<Masker>();
  masker_->loadFromTxt(filterFile);
  LOG(INFO) << masker_->printFilters();

  // Read Map Visualization initial view points
  bool found;
  left_map_view_ = readParameter<Sophus::SE3f>(
      fSettings, "MapVisualizer.left_view", found, true);
  if (!found) {
    LOG(ERROR) << "Parameter MapVisualizer.left_view not found";
    exit(-1);
  }

  right_map_view_ = readParameter<Sophus::SE3f>(
      fSettings, "MapVisualizer.right_view", found, true);
  if (!found) {
    LOG(ERROR) << "Parameter MapVisualizer.right_view not found";
    exit(-1);
  }

  autoplay_ = readParameter<bool>(fSettings, "System.autoplay", found, true);
  if (!found) {
    LOG(ERROR) << "Parameter System.autoplay not found";
    exit(-1);
  }

  map_visualizer_save_path_ =
      readParameter<string>(fSettings, "MapVisualizer.save_path", found, false);
  if (!found) {
    LOG(WARNING)
        << "MapVisualizer.save_path not in settings; set via --output_dir";
  }

  image_visualizer_save_path_ = readParameter<string>(
      fSettings, "ImageVisualizer.save_path", found, false);
  if (!found) {
    LOG(WARNING)
        << "ImageVisualizer.save_path not in settings; set via --output_dir";
  }

  evaluation_save_path_ =
      readParameter<string>(fSettings, "Evaluation.save_path", found, false);
  if (!found) {
    LOG(WARNING)
        << "Evaluation.save_path not in settings; set via --output_dir";
  }

  cv::FileNode lost_stride_node =
      fSettings["Tracking.lost_bootstrap_frame_stride"];
  if (!lost_stride_node.empty()) {
    const int loaded_stride = static_cast<int>(lost_stride_node);
    if (loaded_stride > 0) {
      lost_bootstrap_frame_stride_ = loaded_stride;
    } else {
      LOG(WARNING) << "Tracking.lost_bootstrap_frame_stride must be >= 1. "
                   << "Using default value 1.";
      lost_bootstrap_frame_stride_ = 1;
    }
  }

  cv::FileNode triangulation_lookback_node =
      fSettings["Tracking.triangulation_track_lookback_frames"];
  if (!triangulation_lookback_node.empty()) {
    const int loaded_lookback = static_cast<int>(triangulation_lookback_node);
    if (loaded_lookback > 0) {
      triangulation_track_lookback_frames_ = loaded_lookback;
    } else {
      LOG(WARNING)
          << "Tracking.triangulation_track_lookback_frames must be >= 1. "
          << "Using default value 5.";
      triangulation_track_lookback_frames_ = 5;
    }
  }

  cv::FileNode min_mappoint_distance_node =
      fSettings["Tracking.min_mappoint_distance"];
  if (!min_mappoint_distance_node.empty()) {
    const float loaded_min_distance =
        static_cast<float>(min_mappoint_distance_node);
    if (loaded_min_distance >= 0.f) {
      min_mappoint_distance_ = loaded_min_distance;
    } else {
      LOG(WARNING) << "Tracking.min_mappoint_distance must be >= 0. "
                   << "Using default value 0.02.";
      min_mappoint_distance_ = 0.02f;
    }
  }

  // --- Optional tracking / KLT parameters ---
  auto read_int_opt = [&](const char* key, int& dest) {
    cv::FileNode n = fSettings[key];
    if (!n.empty()) {
      int v = static_cast<int>(n);
      if (v > 0) dest = v;
    }
  };
  auto read_float_opt = [&](const char* key, float& dest) {
    cv::FileNode n = fSettings[key];
    if (!n.empty()) {
      float v = static_cast<float>(n);
      if (v > 0.f) dest = v;
    }
  };
  auto read_float_nonneg = [&](const char* key, float& dest) {
    cv::FileNode n = fSettings[key];
    if (!n.empty()) {
      float v = static_cast<float>(n);
      if (v >= 0.f) dest = v;
    }
  };

  read_int_opt("Tracking.klt_window_size", klt_window_size_);
  read_int_opt("Tracking.klt_max_level", klt_max_level_);
  read_int_opt("Tracking.klt_max_iters", klt_max_iters_);
  read_float_nonneg("Tracking.klt_epsilon", klt_epsilon_);
  read_float_nonneg("Tracking.klt_min_eig_th", klt_min_eig_th_);
  read_float_nonneg("Tracking.klt_min_SSIM", klt_min_SSIM_);
  read_int_opt("Tracking.images_to_insert_keyframe",
               images_to_insert_keyframe_);
  read_int_opt("Tracking.stale_mappoint_max_age_frames",
               stale_mappoint_max_age_frames_);
  read_int_opt("Tracking.min_tracked_points_abort", min_tracked_points_abort_);
  read_int_opt("Tracking.lost_recovery_grace_frames",
               lost_recovery_grace_frames_);
  read_int_opt("Tracking.lost_recovery_min_tracked_points",
               lost_recovery_min_tracked_points_);
  read_int_opt("Tracking.monocular_initializer.klt_window_size",
               monocular_init_klt_window_size_);
  read_int_opt("Tracking.monocular_initializer.klt_max_level",
               monocular_init_klt_max_level_);
  read_int_opt("Tracking.monocular_initializer.klt_max_iters",
               monocular_init_klt_max_iters_);
  read_float_nonneg("Tracking.monocular_initializer.klt_epsilon",
                    monocular_init_klt_epsilon_);
  read_float_nonneg("Tracking.monocular_initializer.klt_min_eig_th",
                    monocular_init_klt_min_eig_th_);
  read_float_nonneg("Tracking.monocular_initializer.klt_min_SSIM",
                    monocular_init_klt_min_SSIM_);
  read_int_opt("Tracking.monocular_initializer.rigid_max_features",
               monocular_init_rigid_max_features_);
  read_float_opt("Tracking.monocular_initializer.rigid_min_parallax",
                 monocular_init_rigid_min_parallax_);
  read_float_opt("Tracking.monocular_initializer.rigid_epipolar_threshold",
                 monocular_init_rigid_epipolar_threshold_);
  read_int_opt("Features.max_corners", feature_max_corners_);
  read_float_opt("Features.quality_level", feature_quality_level_);
  read_float_opt("Features.min_distance", feature_min_distance_);
}

ostream& operator<<(std::ostream& output, const Settings& settings) {
  output << "SLAM settings: " << endl;

  output << "\t-Camera parameters: [ ";
  output << settings.calibration_->GetParameter(0) << " , "
         << settings.calibration_->GetParameter(1) << " , ";
  output << settings.calibration_->GetParameter(2) << " , "
         << settings.calibration_->GetParameter(3) << " ]" << endl;

  output << "\t-Visualization settings:" << endl;
  output << "\t\t-[MapVisualizer] camera size: " << settings.camera_size_
         << endl;

  return output;
}

std::shared_ptr<CameraModel> Settings::getCalibration() { return calibration_; }

float Settings::getBf() { return bf_; }

std::shared_ptr<Masker> Settings::getMasker() { return masker_; }

Sophus::SE3f Settings::GetLeftMapVisualizationView() { return left_map_view_; }

Sophus::SE3f Settings::GetRightMapVisualizationView() {
  return right_map_view_;
}

float Settings::GetCameraSize() { return camera_size_; }

bool Settings::GetAutoplay() { return autoplay_; }

std::string Settings::GetMapVisualizerPath() {
  return map_visualizer_save_path_;
}

std::string Settings::GetImageVisualizerPath() {
  return image_visualizer_save_path_;
}

std::string Settings::GetEvaluationPath() { return evaluation_save_path_; }

void Settings::OverrideOutputDir(const std::string& dir) {
  map_visualizer_save_path_ = dir + "/" + map_visualizer_save_path_;
  image_visualizer_save_path_ = dir + "/" + image_visualizer_save_path_;
  evaluation_save_path_ = dir + "/" + evaluation_save_path_;
}

int Settings::GetLostBootstrapFrameStride() {
  return lost_bootstrap_frame_stride_;
}

int Settings::GetTriangulationTrackLookbackFrames() {
  return triangulation_track_lookback_frames_;
}

float Settings::GetMinMapPointDistance() { return min_mappoint_distance_; }

int Settings::GetKltWindowSize() { return klt_window_size_; }
int Settings::GetKltMaxLevel() { return klt_max_level_; }
int Settings::GetKltMaxIters() { return klt_max_iters_; }
float Settings::GetKltEpsilon() { return klt_epsilon_; }
float Settings::GetKltMinEigTh() { return klt_min_eig_th_; }
float Settings::GetKltMinSSIM() { return klt_min_SSIM_; }
int Settings::GetImagesToInsertKeyframe() { return images_to_insert_keyframe_; }
int Settings::GetStaleMapPointMaxAgeFrames() {
  return stale_mappoint_max_age_frames_;
}
int Settings::GetMinTrackedPointsAbort() { return min_tracked_points_abort_; }
int Settings::GetLostRecoveryGraceFrames() {
  return lost_recovery_grace_frames_;
}
int Settings::GetLostRecoveryMinTrackedPoints() {
  return lost_recovery_min_tracked_points_;
}
int Settings::GetMonocularInitKltWindowSize() {
  return monocular_init_klt_window_size_;
}
int Settings::GetMonocularInitKltMaxLevel() {
  return monocular_init_klt_max_level_;
}
int Settings::GetMonocularInitKltMaxIters() {
  return monocular_init_klt_max_iters_;
}
float Settings::GetMonocularInitKltEpsilon() {
  return monocular_init_klt_epsilon_;
}
float Settings::GetMonocularInitKltMinEigTh() {
  return monocular_init_klt_min_eig_th_;
}
float Settings::GetMonocularInitKltMinSSIM() {
  return monocular_init_klt_min_SSIM_;
}
int Settings::GetMonocularInitRigidMaxFeatures() {
  return monocular_init_rigid_max_features_;
}
float Settings::GetMonocularInitRigidMinParallax() {
  return monocular_init_rigid_min_parallax_;
}
float Settings::GetMonocularInitRigidEpipolarThreshold() {
  return monocular_init_rigid_epipolar_threshold_;
}
int Settings::GetFeatureMaxCorners() { return feature_max_corners_; }
float Settings::GetFeatureQualityLevel() { return feature_quality_level_; }
float Settings::GetFeatureMinDistance() { return feature_min_distance_; }

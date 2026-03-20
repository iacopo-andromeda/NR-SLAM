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

#ifndef NRSLAM_SHI_TOMASI_CV_H
#define NRSLAM_SHI_TOMASI_CV_H

#include <fstream>
#include <opencv2/core/core.hpp>

#include "feature.h"
#include "opencv2/opencv.hpp"

class ShiTomasiCV : public Feature {
 public:
  struct Options {
    int maxCorners;
    double qualityLevel;
    double minDistance;
    int blockSize = 3;
    bool useHarrisDetector = false;
    double k = 0.04;
  };

  ShiTomasiCV() = delete;

  ShiTomasiCV(Options& options);

  ~ShiTomasiCV();

  // Extracts corners following the Shi-Tomasi corner detection algorithm
  void Extract(const cv::Mat& im, const cv::Mat& mask,
               std::vector<cv::KeyPoint>& vKeys) override;

 private:
  Options options_;
  int n_seen_ = 0;
  unsigned int next_feature_id_ = 0;
};

#endif  // NRSLAM_SHI_TOMASI_CV_H

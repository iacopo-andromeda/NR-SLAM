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

#include "shi_tomasi_cv.h"

#include <array>
#include <opencv2/core/core.hpp>
#include <unordered_set>

#include "absl/log/log.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace std::chrono;

ShiTomasiCV::ShiTomasiCV(Options& options) : options_(options) {}

ShiTomasiCV::~ShiTomasiCV() {}

void ShiTomasiCV::Extract(const cv::Mat& im, const cv::Mat& mask,
                          std::vector<cv::KeyPoint>& keypoints) {
  std::vector<cv::Point2f> corners;
  // cv::imshow("Shi-Tomasi", im);
  // cv::waitKey(0);
  // cv::destroyAllWindows();

  std::string image_file_path =
      "/tmp/debug/shi_tomasi_debug_" + to_string(n_seen_++) + ".png";
  cv::imwrite(image_file_path, im);

  cv::goodFeaturesToTrack(im, corners, options_.maxCorners,
                          options_.qualityLevel, options_.minDistance, mask,
                          options_.blockSize, options_.useHarrisDetector,
                          options_.k);

  std::set<std::array<int, 2>> unique_corners;
  for (auto& c : keypoints) {
    unique_corners.insert({(int)round(c.pt.x), (int)round(c.pt.y)});
  }
  keypoints.clear();

  for (const auto& c : corners) {
    if (unique_corners.find({(int)round(c.x), (int)round(c.y)}) ==
        unique_corners.end()) {
      keypoints.emplace_back(c, 1.0, -1, 0, 0, next_feature_id_++);
    }
  }

  LOG(INFO) << "Previously extracted keypoints: " << unique_corners.size()
            << ", new keypoints: " << keypoints.size();
}

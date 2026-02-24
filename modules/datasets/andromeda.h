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

#ifndef NRSLAM_ANDROMEDA_H
#define NRSLAM_ANDROMEDA_H

#include <opencv2/opencv.hpp>
#include <string>

#include "absl/status/statusor.h"

class Andromeda {
 public:
  // Loads the dataset stored at path. If the video has not been previously
  // split, it splits it. Otherwise just loads the images.
  Andromeda(const std::string& image_directory);

  // Retrieves the ith image in the sequence.
  absl::StatusOr<cv::Mat> GetImage(const int idx);

 private:
  // Splits a given video found at videoPath and splits it into single images
  // stored at path.

  std::vector<std::string> images_names_;  // Vector with the image paths
};

#endif  // NRSLAM_ANDROMEDA_H

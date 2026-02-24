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

#include "andromeda.h"

#include <sys/stat.h>

#include <boost/filesystem.hpp>
#include <filesystem>
#include <fstream>

#include "absl/log/log.h"

using namespace std;

Andromeda::Andromeda(const std::string& image_directory) {
  auto path = std::filesystem::path(image_directory);

  // Ensure names are sorted
  std::map<std::string, std::string> names_map;
  for (const auto& entry :
       std::filesystem::recursive_directory_iterator(path)) {
    if (entry.path().extension() == ".png") {
      names_map[entry.path().filename().string()] = entry.path().string();
    }
  }

  // Save names in vector
  images_names_.clear();
  for (const auto& name : names_map) {
    images_names_.push_back(name.second);
  }
}

absl::StatusOr<cv::Mat> Andromeda::GetImage(const int idx) {
  if (idx >= images_names_.size()) {
    return absl::InternalError("Image index out boundaries.");
  }

  return cv::imread(images_names_[idx], cv::IMREAD_UNCHANGED);
}

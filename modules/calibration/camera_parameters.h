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

#ifndef NRSLAM_CAMERA_PARAMETERS_H
#define NRSLAM_CAMERA_PARAMETERS_H

#include <stdexcept>
#include <string>
#include <vector>

#include "absl/log/log.h"

// Type-safe parameter sets for each camera model

struct PinholeParameters {
  float fx, fy, cx, cy;

  // Convert to vector for compatibility with legacy code
  std::vector<float> ToVector() const { return {fx, fy, cx, cy}; }

  // Create from vector with validation
  static PinholeParameters FromVector(const std::vector<float>& v) {
    if (v.size() != 4) {
      throw std::invalid_argument(
          "PinholeParameters expects exactly 4 parameters, got " +
          std::to_string(v.size()));
    }
    return {v[0], v[1], v[2], v[3]};
  }

  // Pretty printing for logging
  std::string ToString() const {
    return "PinHole(fx=" + std::to_string(fx) + ", fy=" + std::to_string(fy) +
           ", cx=" + std::to_string(cx) + ", cy=" + std::to_string(cy) + ")";
  }
};

struct DistortedPinholeParameters {
  float fx, fy, cx, cy, k1, k2, p1, p2;

  std::vector<float> ToVector() const {
    return {fx, fy, cx, cy, k1, k2, p1, p2};
  }

  static DistortedPinholeParameters FromVector(const std::vector<float>& v) {
    if (v.size() != 8) {
      throw std::invalid_argument(
          "DistortedPinholeParameters expects exactly 8 parameters, got " +
          std::to_string(v.size()));
    }
    return {v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]};
  }

  std::string ToString() const {
    return "DistortedPinHole(fx=" + std::to_string(fx) +
           ", fy=" + std::to_string(fy) + ", cx=" + std::to_string(cx) +
           ", cy=" + std::to_string(cy) + ", k1=" + std::to_string(k1) +
           ", k2=" + std::to_string(k2) + ", p1=" + std::to_string(p1) +
           ", p2=" + std::to_string(p2) + ")";
  }
};

struct KannalaBrandt8Parameters {
  float fx, fy, cx, cy, k0, k1, k2, k3;

  std::vector<float> ToVector() const {
    return {fx, fy, cx, cy, k0, k1, k2, k3};
  }

  static KannalaBrandt8Parameters FromVector(const std::vector<float>& v) {
    if (v.size() != 8) {
      throw std::invalid_argument(
          "KannalaBrandt8Parameters expects exactly 8 parameters, got " +
          std::to_string(v.size()));
    }
    return {v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]};
  }

  std::string ToString() const {
    return "KannalaBrandt8(fx=" + std::to_string(fx) +
           ", fy=" + std::to_string(fy) + ", cx=" + std::to_string(cx) +
           ", cy=" + std::to_string(cy) + ", k0=" + std::to_string(k0) +
           ", k1=" + std::to_string(k1) + ", k2=" + std::to_string(k2) +
           ", k3=" + std::to_string(k3) + ")";
  }
};

#endif  // NRSLAM_CAMERA_PARAMETERS_H

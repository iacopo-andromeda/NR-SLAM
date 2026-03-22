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

#ifndef NRSLAM_PERFORMANCE_LOGGER_H
#define NRSLAM_PERFORMANCE_LOGGER_H

#include <cstdint>
#include <string>
#include <vector>

// Per-frame performance stats for the NR-SLAM system.
// Records tracking status, feature counts, and wall-clock timing for every
// processed frame, then provides CSV export and a human-readable summary.
class PerformanceLogger {
 public:
  struct FrameStats {
    uint64_t frame_id = 0;

    // "NOT_INITIALIZED", "TRACKING", or "LOST"
    std::string tracking_status;

    // Total keypoints observed in the current frame.
    int n_keypoints = 0;

    // Keypoints that have an associated 3-D map-point (TRACKED_WITH_3D).
    int n_tracked_3d = 0;

    // Number of map-points active in the global map at this frame.
    int n_map_points = 0;

    // Wall-clock time spent inside tracker_->TrackImage() [ms].
    double ms_tracking = 0.0;

    // Wall-clock time spent inside mapper_->DoMapping() [ms].
    double ms_mapping = 0.0;

    // Total wall-clock time for this frame (preprocessing included) [ms].
    double ms_total = 0.0;
  };

  PerformanceLogger() = default;

  // Append statistics for one processed frame.
  void LogFrame(const FrameStats& stats);

  // Write every per-frame record as a CSV file (creates or overwrites `path`).
  void SaveToCSV(const std::string& path) const;

  // Print a human-readable summary of the whole run to LOG(INFO).
  void PrintSummary() const;

 private:
  std::vector<FrameStats> frames_;
};

#endif  // NRSLAM_PERFORMANCE_LOGGER_H

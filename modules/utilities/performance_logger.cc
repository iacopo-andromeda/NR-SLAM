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

#include "performance_logger.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <numeric>

#include "absl/log/log.h"

void PerformanceLogger::LogFrame(const FrameStats& stats) {
  frames_.push_back(stats);
}

void PerformanceLogger::SaveToCSV(const std::string& path) const {
  std::ofstream f(path);
  if (!f.is_open()) {
    LOG(ERROR) << "PerformanceLogger: cannot open output file: " << path;
    return;
  }

  f << "frame_id,tracking_status,n_keypoints,n_tracked_3d,n_map_points,"
       "ms_tracking,ms_mapping,ms_total\n";

  for (const auto& s : frames_) {
    f << s.frame_id << "," << s.tracking_status << "," << s.n_keypoints << ","
      << s.n_tracked_3d << "," << s.n_map_points << "," << std::fixed
      << std::setprecision(2) << s.ms_tracking << "," << s.ms_mapping << ","
      << s.ms_total << "\n";
  }

  f.close();
  LOG(INFO) << "PerformanceLogger: saved " << frames_.size()
            << " frame records to " << path;
}

void PerformanceLogger::PrintSummary() const {
  if (frames_.empty()) {
    LOG(INFO) << "[PERF SUMMARY] No frames logged.";
    return;
  }

  const int total_frames = static_cast<int>(frames_.size());
  int tracking_frames = 0, lost_frames = 0, init_frames = 0;
  double sum_keypoints = 0, sum_tracked_3d = 0, sum_map_points = 0;
  double sum_ms_tracking = 0, sum_ms_mapping = 0, sum_ms_total = 0;
  double max_ms_total = 0;
  std::vector<double> ms_total_vec;
  ms_total_vec.reserve(total_frames);

  for (const auto& s : frames_) {
    if (s.tracking_status == "TRACKING")
      tracking_frames++;
    else if (s.tracking_status == "LOST")
      lost_frames++;
    else
      init_frames++;

    sum_keypoints += s.n_keypoints;
    sum_tracked_3d += s.n_tracked_3d;
    sum_map_points += s.n_map_points;
    sum_ms_tracking += s.ms_tracking;
    sum_ms_mapping += s.ms_mapping;
    sum_ms_total += s.ms_total;
    if (s.ms_total > max_ms_total) max_ms_total = s.ms_total;
    ms_total_vec.push_back(s.ms_total);
  }

  // Median frame time.
  std::sort(ms_total_vec.begin(), ms_total_vec.end());
  const double median_ms_total = ms_total_vec[total_frames / 2];

  const double mean_fps =
      (sum_ms_total > 0.0) ? (1000.0 * total_frames / sum_ms_total) : 0.0;

  auto pct = [&](int n) -> double { return (100.0 * n) / total_frames; };
  auto mean = [&](double sum) -> double { return sum / total_frames; };

  LOG(INFO).NoPrefix() << "";
  LOG(INFO).NoPrefix() << "========== NR-SLAM PERFORMANCE SUMMARY ==========";
  LOG(INFO).NoPrefix() << "Total frames processed    : " << total_frames;
  LOG(INFO).NoPrefix() << "";
  LOG(INFO).NoPrefix() << "--- Tracking Status Breakdown ---";
  LOG(INFO).NoPrefix() << "  TRACKING              : " << tracking_frames
                       << "  (" << std::fixed << std::setprecision(1)
                       << pct(tracking_frames) << "%)";
  LOG(INFO).NoPrefix() << "  LOST                  : " << lost_frames << "  ("
                       << pct(lost_frames) << "%)";
  LOG(INFO).NoPrefix() << "  NOT_INITIALIZED       : " << init_frames << "  ("
                       << pct(init_frames) << "%)";
  LOG(INFO).NoPrefix() << "";
  LOG(INFO).NoPrefix() << "--- Feature Counts (mean per frame) ---";
  LOG(INFO).NoPrefix() << "  Total keypoints       : " << std::fixed
                       << std::setprecision(1) << mean(sum_keypoints);
  LOG(INFO).NoPrefix() << "  Tracked with 3-D      : " << mean(sum_tracked_3d);
  LOG(INFO).NoPrefix() << "  Active map points     : " << mean(sum_map_points);
  LOG(INFO).NoPrefix() << "";
  LOG(INFO).NoPrefix() << "--- Timing (mean per frame) ---";
  LOG(INFO).NoPrefix() << "  Tracking              : " << std::fixed
                       << std::setprecision(2) << mean(sum_ms_tracking)
                       << " ms";
  LOG(INFO).NoPrefix() << "  Mapping               : " << mean(sum_ms_mapping)
                       << " ms";
  LOG(INFO).NoPrefix() << "  Total                 : " << mean(sum_ms_total)
                       << " ms";
  LOG(INFO).NoPrefix() << "  Median frame time     : " << median_ms_total
                       << " ms";
  LOG(INFO).NoPrefix() << "  Max frame time        : " << max_ms_total << " ms";
  LOG(INFO).NoPrefix() << "  Mean throughput       : " << std::fixed
                       << std::setprecision(1) << mean_fps << " fps";
  LOG(INFO).NoPrefix() << "=================================================";
  LOG(INFO).NoPrefix() << "";
}

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

#include <cstdint>
#include <fstream>
#include <mutex>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <vector>

#include "SLAM/system.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"
#include "absl/log/initialize.h"
#include "absl/log/log.h"
#include "absl/log/log_entry.h"
#include "absl/log/log_sink.h"
#include "absl/log/log_sink_registry.h"
#include "cpp_bag_reader/bag_reader.hpp"

using namespace std;

ABSL_FLAG(std::string, dataset_path,
          "/home/galactus/Documents/robot-bags/rosbag2_13-02-2026_08-57-17",
          "Path to the video dataset");
ABSL_FLAG(std::string, settings_path, "", "Path to the settings file");
ABSL_FLAG(uint64_t, starting_frame, 1771001959417433296ULL,
          "Start bound. Interpreted per --range_mode.");
ABSL_FLAG(uint64_t, end_frame, 1771002836540509598ULL,
          "End bound. Interpreted per --range_mode. 0 means no end bound.");
ABSL_FLAG(std::string, range_mode, "timestamp_ns",
          "How to interpret start/end: message_index or timestamp_ns");
ABSL_FLAG(uint64_t, max_images, 200,
          "Maximum number of main-topic images to process. 0 means unlimited.");
ABSL_FLAG(std::string, log_file, "slam_run.log",
          "Path of the log file where all LOG() output is saved");

// ---------------------------------------------------------------------------
// File sink: mirrors every absl log entry to a text file.
// ---------------------------------------------------------------------------
class FileSink : public absl::LogSink {
 public:
  explicit FileSink(const std::string& path) {
    file_.open(path, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
      LOG(ERROR) << "FileSink: could not open log file: " << path;
    }
  }

  ~FileSink() override {
    if (file_.is_open()) file_.close();
  }

  void Send(const absl::LogEntry& entry) override {
    if (!file_.is_open()) return;
    std::lock_guard<std::mutex> lock(mu_);
    file_ << entry.text_message_with_prefix_and_newline();
    file_.flush();
  }

 private:
  std::ofstream file_;
  std::mutex mu_;
};

int main(int argc, char** argv) {
  constexpr char kMainTopic[] = "/image_raw/compressed";
  constexpr char kSecondaryTopic[] = "/robot/state";

  // Parse command line arguments
  absl::ParseCommandLine(argc, argv);

  // Process command arguments
  string dataset_path = absl::GetFlag(FLAGS_dataset_path);
  if (dataset_path.empty()) {
    LOG(ERROR) << "Must specify an input dataset path." << endl;
    return -1;
  }
  string settings_path = absl::GetFlag(FLAGS_settings_path);
  if (settings_path.empty()) {
    LOG(ERROR) << "Must specify an input settings file." << endl;
    return -1;
  }

  const uint64_t starting_frame = absl::GetFlag(FLAGS_starting_frame);
  const uint64_t end_frame = absl::GetFlag(FLAGS_end_frame);
  const uint64_t max_images = absl::GetFlag(FLAGS_max_images);
  const string range_mode = absl::GetFlag(FLAGS_range_mode);
  const bool use_timestamp_mode = (range_mode == "timestamp_ns");
  if (!use_timestamp_mode && range_mode != "message_index") {
    LOG(ERROR) << "Invalid --range_mode='" << range_mode
               << "'. Valid values: message_index, timestamp_ns";
    return -1;
  }

  if (end_frame > 0 && end_frame <= starting_frame) {
    LOG(ERROR)
        << "Invalid range: end_frame must be greater than starting_frame "
        << "when end_frame > 0.";
    return -1;
  }

  // Register the file log sink so that every LOG(...) call is also written to
  // the log file (in addition to stderr).
  const string log_file_path = absl::GetFlag(FLAGS_log_file);
  FileSink file_sink(log_file_path);
  absl::AddLogSink(&file_sink);

  LOG(INFO) << "NR-SLAM andromeda launcher";
  LOG(INFO) << "  dataset_path   : " << dataset_path;
  LOG(INFO) << "  settings_path  : " << settings_path;
  LOG(INFO) << "  frames         : [" << starting_frame << ", " << end_frame
            << ")";
  LOG(INFO) << "  range_mode     : " << range_mode;
  LOG(INFO) << "  max_images     : " << max_images;
  LOG(INFO) << "  main_topic     : " << kMainTopic;
  LOG(INFO) << "  secondary_topic: " << kSecondaryTopic;
  LOG(INFO) << "  log_file       : " << log_file_path;

  cpp_bag_reader::BagReaderIterator iterator;
  const uint64_t open_start_ts = use_timestamp_mode ? starting_frame : 0;
  const uint64_t open_end_ts = use_timestamp_mode ? end_frame : 0;
  if (!iterator.open(dataset_path, open_start_ts, open_end_ts)) {
    LOG(ERROR) << "Could not open bag at '" << dataset_path
               << "': " << iterator.last_error();
    absl::RemoveLogSink(&file_sink);
    return -1;
  }

  // Create SLAM system.
  System SLAM(settings_path);

  uint64_t main_topic_count = 0;
  uint64_t tracked_image_count = 0;
  uint64_t secondary_topic_count = 0;
  uint64_t failed_decode_count = 0;

  for (const auto& message : iterator) {
    if (use_timestamp_mode && end_frame > 0 &&
        message.timestamp_ns >= end_frame) {
      break;
    }

    if (message.topic_name == kSecondaryTopic) {
      ++secondary_topic_count;
      continue;
    }

    if (message.topic_name != kMainTopic) {
      continue;
    }

    const uint64_t current_main_index = main_topic_count;
    ++main_topic_count;

    if (!use_timestamp_mode) {
      if (current_main_index < starting_frame) {
        continue;
      }
      if (end_frame > 0 && current_main_index >= end_frame) {
        break;
      }
    }

    sensor_msgs::msg::CompressedImage compressed;
    if (!cpp_bag_reader::deserialize_message(message, compressed)) {
      LOG(WARNING) << "Failed to deserialize compressed image at ts="
                   << message.timestamp_ns;
      ++failed_decode_count;
      continue;
    }

    cv::Mat encoded(1, static_cast<int>(compressed.data.size()), CV_8UC1,
                    compressed.data.data());
    cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image.empty()) {
      LOG(WARNING) << "Failed to decode compressed image at ts="
                   << message.timestamp_ns;
      ++failed_decode_count;
      continue;
    }

    LOG(INFO) << "Processing main topic message idx=" << current_main_index
              << " ts=" << message.timestamp_ns;
    SLAM.TrackImage(image);
    ++tracked_image_count;

    if (max_images > 0 && tracked_image_count >= max_images) {
      LOG(INFO) << "Reached max_images=" << max_images
                << ". Stopping processing.";
      break;
    }
  }

  if (!iterator.last_error().empty()) {
    LOG(ERROR) << "Bag reader error: " << iterator.last_error();
    absl::RemoveLogSink(&file_sink);
    return -1;
  }

  LOG(INFO) << "Finished processing bag.";
  LOG(INFO) << "  main_topic_messages_seen : " << main_topic_count;
  LOG(INFO) << "  images_tracked           : " << tracked_image_count;
  LOG(INFO) << "  secondary_topic_messages : " << secondary_topic_count;
  LOG(INFO) << "  decode_failures          : " << failed_decode_count;

  // Unregister the sink before it goes out of scope (required by absl).
  absl::RemoveLogSink(&file_sink);

  return 0;
}
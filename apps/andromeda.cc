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

#include "datasets/andromeda.h"

#include <fstream>
#include <mutex>

#include "SLAM/system.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/log/check.h"
#include "absl/log/initialize.h"
#include "absl/log/log.h"
#include "absl/log/log_entry.h"
#include "absl/log/log_sink.h"
#include "absl/log/log_sink_registry.h"

using namespace std;

ABSL_FLAG(std::string, dataset_path, "", "Path to the video dataset");
ABSL_FLAG(std::string, settings_path, "", "Path to the settings file");
ABSL_FLAG(int, starting_frame, 0, "First frame of the dataset to process");
ABSL_FLAG(int, end_frame, 0, "Last frame of the dataset to process");
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

  int starting_frame = absl::GetFlag(FLAGS_starting_frame);
  int end_frame = absl::GetFlag(FLAGS_end_frame);

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
  LOG(INFO) << "  log_file       : " << log_file_path;

  Andromeda dataset(dataset_path);

  // Create SLAM system.
  System SLAM(settings_path);

  for (int idx = starting_frame; idx < end_frame; idx++) {
    LOG(INFO) << "Processing image " << idx;
    auto image = dataset.GetImage(ImageIndex{idx});
    CHECK_OK(image);

    SLAM.TrackImage(*image);
  }

  // Unregister the sink before it goes out of scope (required by absl).
  absl::RemoveLogSink(&file_sink);

  return 0;
}
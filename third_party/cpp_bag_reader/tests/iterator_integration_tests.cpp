#include <cstdlib>
#include <filesystem>
#include <string>

#include "cpp_bag_reader/bag_reader.hpp"
#include "m31_interfaces/msg/robot_state.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

int main(int argc, char* argv[]) {
  std::string bag_path;
  uint64_t start_timestamp_ns = 0;
  uint64_t end_timestamp_ns = 0;

  if (argc >= 4) {
    bag_path = argv[1];
    start_timestamp_ns = std::stoull(argv[2]);
    end_timestamp_ns = std::stoull(argv[3]);
  } else {
    const char* env_bag_path = std::getenv("CPP_BAG_READER_TEST_BAG");
    if (env_bag_path == nullptr) {
      return 77;
    }
    bag_path = env_bag_path;

    const char* env_start_ns = std::getenv("CPP_BAG_READER_TEST_START_NS");
    const char* env_end_ns = std::getenv("CPP_BAG_READER_TEST_END_NS");
    if (env_start_ns != nullptr) {
      start_timestamp_ns = std::stoull(env_start_ns);
    }
    if (env_end_ns != nullptr) {
      end_timestamp_ns = std::stoull(env_end_ns);
    }
  }

  if (!std::filesystem::exists(bag_path)) {
    return 77;
  }

  cpp_bag_reader::BagReaderIterator iterator;
  if (!iterator.open(bag_path, start_timestamp_ns, end_timestamp_ns)) {
    return 1;
  }

  uint64_t message_count = 0;
  uint64_t compressed_seen = 0;
  uint64_t compressed_failed = 0;
  uint64_t robot_state_seen = 0;
  uint64_t robot_state_failed = 0;

  for (const auto& message : iterator) {
    message_count++;

    if (start_timestamp_ns > 0 && message.timestamp_ns < start_timestamp_ns) {
      return 2;
    }
    if (end_timestamp_ns > 0 && message.timestamp_ns > end_timestamp_ns) {
      return 3;
    }

    if (message.topic_name == "/image_raw/compressed") {
      compressed_seen++;
      sensor_msgs::msg::CompressedImage deserialized;
      if (!cpp_bag_reader::deserialize_message(message, deserialized)) {
        compressed_failed++;
      }
    }

    if (message.topic_name == "/robot/state") {
      robot_state_seen++;
      m31_interfaces::msg::RobotState deserialized;
      if (!cpp_bag_reader::deserialize_message(message, deserialized)) {
        robot_state_failed++;
      }
    }
  }

  if (!iterator.last_error().empty()) {
    return 4;
  }

  if (message_count == 0) {
    return 5;
  }

  if (compressed_seen == 0 || robot_state_seen == 0) {
    return 6;
  }

  if (compressed_failed > 0 || robot_state_failed > 0) {
    return 7;
  }

  return 0;
}

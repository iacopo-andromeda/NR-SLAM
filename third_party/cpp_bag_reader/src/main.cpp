#include <cstdint>
#include <string>

#include "cpp_bag_reader/bag_reader.hpp"
#include "m31_interfaces/msg/robot_state.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

int main(int argc, char* argv[]) {
  if (argc < 4) {
    return 1;
  }

  const std::string bag_path = argv[1];
  const uint64_t start_timestamp_ns = std::stoull(argv[2]);
  const uint64_t end_timestamp_ns = std::stoull(argv[3]);

  cpp_bag_reader::BagReaderIterator iterator;
  if (!iterator.open(bag_path, start_timestamp_ns, end_timestamp_ns)) {
    return 2;
  }

  uint64_t compressed_seen = 0;
  uint64_t compressed_failed = 0;
  uint64_t robot_state_seen = 0;
  uint64_t robot_state_failed = 0;

  for (const auto& message : iterator) {
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
    return 3;
  }

  if (compressed_seen == 0 || robot_state_seen == 0) {
    return 4;
  }

  if (compressed_failed > 0 || robot_state_failed > 0) {
    return 5;
  }

  return 0;
}

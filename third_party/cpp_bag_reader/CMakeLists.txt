cmake_minimum_required(VERSION 3.10)
project(cpp_bag_reader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

# Find ROS2 packages
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs QUIET)
find_package(m31_interfaces QUIET)

# Create reusable library
add_library(cpp_bag_reader src/bag_reader.cpp)

# Link libraries
target_link_libraries(cpp_bag_reader
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rclcpp::rclcpp
)

# Include directories
target_include_directories(cpp_bag_reader
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(sensor_msgs_FOUND AND m31_interfaces_FOUND)
  # Create thin CLI wrapper
  add_executable(bag_reader src/main.cpp)
  target_link_libraries(bag_reader cpp_bag_reader)

  add_executable(cpp_bag_reader_iterator_unit_tests
    tests/iterator_unit_tests.cpp
  )
  target_link_libraries(cpp_bag_reader_iterator_unit_tests
    cpp_bag_reader
  )
  add_test(
    NAME cpp_bag_reader_iterator_unit_tests
    COMMAND cpp_bag_reader_iterator_unit_tests
  )

  add_executable(cpp_bag_reader_iterator_integration_tests
    tests/iterator_integration_tests.cpp
  )
  target_link_libraries(cpp_bag_reader_iterator_integration_tests
    cpp_bag_reader
  )
  add_test(
    NAME cpp_bag_reader_iterator_integration_tests
    COMMAND cpp_bag_reader_iterator_integration_tests
      /home/galactus/Documents/M31/local_dev/rosbag2_13-02-2026_08-57-17
      1771001959417433296
      1771002836540509598
  )
  set_tests_properties(cpp_bag_reader_iterator_integration_tests PROPERTIES SKIP_RETURN_CODE 77)
else()
  message(STATUS "cpp_bag_reader: sensor_msgs or m31_interfaces not found; skipping bag_reader CLI and tests")
endif()

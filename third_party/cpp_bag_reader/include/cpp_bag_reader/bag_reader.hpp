#pragma once

#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <string>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcutils/types/uint8_array.h"

namespace cpp_bag_reader {

enum class ReadStatus {
  kOk,
  kEndOfBag,
  kNotOpen,
  kError,
};

struct BagMessage {
  std::string topic_name;
  uint64_t timestamp_ns{0};
  std::shared_ptr<rcutils_uint8_array_t> serialized_data;
};

class BagReaderIterator {
 public:
  class EndSentinel {};

  class Iterator {
   public:
    using iterator_category = std::input_iterator_tag;
    using value_type = BagMessage;
    using difference_type = std::ptrdiff_t;
    using pointer = const BagMessage*;
    using reference = const BagMessage&;

    Iterator() = default;
    explicit Iterator(BagReaderIterator* owner);

    reference operator*() const;
    pointer operator->() const;
    Iterator& operator++();
    Iterator operator++(int);
    bool operator==(const Iterator& other) const;
    bool operator!=(const Iterator& other) const;
    bool operator==(EndSentinel) const;
    bool operator!=(EndSentinel) const;

   private:
    void advance();

    BagReaderIterator* owner_{nullptr};
    BagMessage current_{};
    bool at_end_{true};
  };

  using iterator = Iterator;
  using const_iterator = Iterator;
  using sentinel = EndSentinel;

  BagReaderIterator();
  ~BagReaderIterator();
  BagReaderIterator(BagReaderIterator&&) noexcept;
  BagReaderIterator& operator=(BagReaderIterator&&) noexcept;

  BagReaderIterator(const BagReaderIterator&) = delete;
  BagReaderIterator& operator=(const BagReaderIterator&) = delete;

  bool open(const std::string& bag_path, uint64_t start_timestamp_ns,
            uint64_t end_timestamp_ns, const std::string& storage_id = "mcap");
  void close();
  bool is_open() const;
  ReadStatus next(BagMessage& message);
  const std::string& last_error() const;

  Iterator begin();
  Iterator begin() const;
  Iterator cbegin() const;

  EndSentinel end() const;
  EndSentinel cend() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

template <typename MessageT>
bool deserialize_message(const BagMessage& bag_message, MessageT& output,
                         std::string* error_message = nullptr) {
  if (!bag_message.serialized_data) {
    if (error_message != nullptr) {
      *error_message = "Serialized data is null";
    }
    return false;
  }

  try {
    rclcpp::SerializedMessage serialized_message(*bag_message.serialized_data);
    rclcpp::Serialization<MessageT> serializer;
    serializer.deserialize_message(&serialized_message, &output);
    return true;
  } catch (const std::exception& e) {
    if (error_message != nullptr) {
      *error_message = e.what();
    }
    return false;
  }
}

}  // namespace cpp_bag_reader

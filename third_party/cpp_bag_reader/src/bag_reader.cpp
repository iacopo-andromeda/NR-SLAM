#include "cpp_bag_reader/bag_reader.hpp"

#include <exception>
#include <memory>

#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace cpp_bag_reader {

class BagReaderIterator::Impl {
 public:
  std::unique_ptr<rosbag2_cpp::Reader> reader;
  uint64_t start_timestamp_ns{0};
  uint64_t end_timestamp_ns{0};
  bool open{false};
  std::string last_error;
};

BagReaderIterator::BagReaderIterator() : impl_(std::make_unique<Impl>()) {}

BagReaderIterator::~BagReaderIterator() = default;

BagReaderIterator::BagReaderIterator(BagReaderIterator&&) noexcept = default;

BagReaderIterator& BagReaderIterator::operator=(BagReaderIterator&&) noexcept =
    default;

bool BagReaderIterator::open(const std::string& bag_path,
                             uint64_t start_timestamp_ns,
                             uint64_t end_timestamp_ns,
                             const std::string& storage_id) {
  impl_->last_error.clear();
  impl_->start_timestamp_ns = start_timestamp_ns;
  impl_->end_timestamp_ns = end_timestamp_ns;

  rosbag2_storage::StorageOptions storage_options{};
  storage_options.uri = bag_path;
  storage_options.storage_id = storage_id;

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  impl_->reader = std::make_unique<rosbag2_cpp::Reader>();
  try {
    impl_->reader->open(storage_options, converter_options);
    impl_->open = true;
    return true;
  } catch (const std::exception& e) {
    impl_->open = false;
    impl_->reader.reset();
    impl_->last_error = std::string("Failed to open bag: ") + e.what();
    return false;
  }
}

void BagReaderIterator::close() {
  impl_->open = false;
  impl_->reader.reset();
}

bool BagReaderIterator::is_open() const { return impl_->open; }

ReadStatus BagReaderIterator::next(BagMessage& message) {
  if (!impl_->open || !impl_->reader) {
    impl_->last_error = "Reader is not open";
    return ReadStatus::kNotOpen;
  }

  try {
    while (impl_->reader->has_next()) {
      auto bag_message = impl_->reader->read_next();
      const uint64_t msg_timestamp = bag_message->recv_timestamp;

      if (impl_->start_timestamp_ns > 0 &&
          msg_timestamp < impl_->start_timestamp_ns) {
        continue;
      }
      if (impl_->end_timestamp_ns > 0 &&
          msg_timestamp > impl_->end_timestamp_ns) {
        continue;
      }

      message.topic_name = bag_message->topic_name;
      message.timestamp_ns = msg_timestamp;
      message.serialized_data = bag_message->serialized_data;
      return ReadStatus::kOk;
    }

    return ReadStatus::kEndOfBag;
  } catch (const std::exception& e) {
    impl_->last_error = std::string("Failed while reading bag: ") + e.what();
    return ReadStatus::kError;
  }
}

const std::string& BagReaderIterator::last_error() const {
  return impl_->last_error;
}

BagReaderIterator::Iterator::Iterator(BagReaderIterator* owner)
    : owner_(owner) {
  advance();
}

BagReaderIterator::Iterator::reference BagReaderIterator::Iterator::operator*()
    const {
  return current_;
}

BagReaderIterator::Iterator::pointer BagReaderIterator::Iterator::operator->()
    const {
  return &current_;
}

BagReaderIterator::Iterator& BagReaderIterator::Iterator::operator++() {
  advance();
  return *this;
}

BagReaderIterator::Iterator BagReaderIterator::Iterator::operator++(int) {
  Iterator previous(*this);
  advance();
  return previous;
}

bool BagReaderIterator::Iterator::operator==(const Iterator& other) const {
  if (at_end_ && other.at_end_) {
    return true;
  }

  return owner_ == other.owner_ && at_end_ == other.at_end_ &&
         current_.topic_name == other.current_.topic_name &&
         current_.timestamp_ns == other.current_.timestamp_ns;
}

bool BagReaderIterator::Iterator::operator!=(const Iterator& other) const {
  return !(*this == other);
}

bool BagReaderIterator::Iterator::operator==(EndSentinel) const {
  return at_end_;
}

bool BagReaderIterator::Iterator::operator!=(EndSentinel) const {
  return !at_end_;
}

void BagReaderIterator::Iterator::advance() {
  if (owner_ == nullptr) {
    at_end_ = true;
    return;
  }

  const ReadStatus status = owner_->next(current_);
  at_end_ = (status != ReadStatus::kOk);
}

BagReaderIterator::Iterator BagReaderIterator::begin() {
  return Iterator(this);
}

BagReaderIterator::Iterator BagReaderIterator::begin() const {
  return Iterator(const_cast<BagReaderIterator*>(this));
}

BagReaderIterator::Iterator BagReaderIterator::cbegin() const {
  return begin();
}

BagReaderIterator::EndSentinel BagReaderIterator::end() const {
  return EndSentinel{};
}

BagReaderIterator::EndSentinel BagReaderIterator::cend() const {
  return EndSentinel{};
}

}  // namespace cpp_bag_reader

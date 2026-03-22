#include <string>
#include <type_traits>

#include "cpp_bag_reader/bag_reader.hpp"

static_assert(
    std::is_same<cpp_bag_reader::BagReaderIterator::iterator::iterator_category,
                 std::input_iterator_tag>::value,
    "Iterator category must be std::input_iterator_tag");
static_assert(
    std::is_same<cpp_bag_reader::BagReaderIterator::iterator::value_type,
                 cpp_bag_reader::BagMessage>::value,
    "Iterator value_type must be BagMessage");

int main() {
  cpp_bag_reader::BagReaderIterator iterator;

  cpp_bag_reader::BagMessage message;
  const cpp_bag_reader::ReadStatus status = iterator.next(message);
  if (status != cpp_bag_reader::ReadStatus::kNotOpen) {
    return 1;
  }
  if (iterator.last_error().empty()) {
    return 2;
  }

  auto begin_it = iterator.begin();
  if (begin_it != iterator.end()) {
    return 3;
  }
  if (iterator.last_error().empty()) {
    return 4;
  }

  const cpp_bag_reader::BagReaderIterator& const_iterator_ref = iterator;
  auto cbegin_it = const_iterator_ref.cbegin();
  if (cbegin_it != const_iterator_ref.cend()) {
    return 5;
  }

  return 0;
}

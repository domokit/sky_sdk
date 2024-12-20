// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_CONTAINER_UTILS_H_
#define BASE_CONTAINER_UTILS_H_

#include <set>
#include <vector>

namespace base {

template <class T, class Allocator, class Predicate>
size_t EraseIf(std::vector<T, Allocator>& container, Predicate pred) {
  auto it = std::remove_if(container.begin(), container.end(), pred);
  size_t removed = std::distance(it, container.end());
  container.erase(it, container.end());
  return removed;
}

template <typename Container, typename Value>
bool Contains(const Container& container, const Value& value) {
  return container.find(value) != container.end();
}

template <typename T>
bool Contains(const std::vector<T>& container, const T& value) {
  return std::find(container.begin(), container.end(), value) !=
         container.end();
}

}  // namespace base

#endif  // BASE_CONTAINER_UTILS_H_

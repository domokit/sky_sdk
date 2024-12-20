// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_BASE_COMPARABLE_H_
#define FLUTTER_IMPELLER_BASE_COMPARABLE_H_

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <type_traits>

namespace impeller {

struct UniqueID {
  size_t id;

  UniqueID();

  constexpr bool operator==(const UniqueID& other) const {
    return id == other.id;
  }
};

class ComparableBase {};

template <class Type>
class Comparable : ComparableBase {
 public:
  virtual std::size_t GetHash() const = 0;

  virtual bool IsEqual(const Type& other) const = 0;
};

template <
    class ComparableType,
    class = std::enable_if_t<std::is_base_of_v<ComparableBase, ComparableType>>>
struct ComparableHash {
  std::size_t operator()(const ComparableType& object) const {
    return object.GetHash();
  }
};

template <
    class ComparableType,
    class = std::enable_if_t<std::is_base_of_v<ComparableBase, ComparableType>>>
struct ComparableEqual {
  bool operator()(const ComparableType& lhs, const ComparableType& rhs) const {
    return lhs.IsEqual(rhs);
  }
};

template <
    class ComparableType,
    class = std::enable_if_t<std::is_base_of_v<ComparableBase, ComparableType>>>
bool DeepComparePointer(const std::shared_ptr<ComparableType>& lhs,
                        const std::shared_ptr<ComparableType>& rhs) {
  if (lhs == rhs) {
    return true;
  }

  if (lhs && rhs) {
    return lhs->IsEqual(*rhs);
  }

  return false;
}

template <
    class Key,
    class ComparableType,
    class = std::enable_if_t<std::is_base_of_v<ComparableBase, ComparableType>>>
bool DeepCompareMap(const std::map<Key, std::shared_ptr<ComparableType>>& lhs,
                    const std::map<Key, std::shared_ptr<ComparableType>>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }

  for (auto i = lhs.begin(), j = rhs.begin(); i != lhs.end(); i++, j++) {
    if (i->first != j->first) {
      return false;
    }

    if (!DeepComparePointer(i->second, j->second)) {
      return false;
    }
  }

  return true;
}

}  // namespace impeller

namespace std {

template <>
struct hash<impeller::UniqueID> {
  constexpr std::size_t operator()(const impeller::UniqueID& id) {
    return id.id;
  }
};

template <>
struct less<impeller::UniqueID> {
  constexpr bool operator()(const impeller::UniqueID& lhs,
                            const impeller::UniqueID& rhs) const {
    return lhs.id < rhs.id;
  }
};

}  // namespace std

#endif  // FLUTTER_IMPELLER_BASE_COMPARABLE_H_

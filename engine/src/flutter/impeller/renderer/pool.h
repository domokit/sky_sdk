// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_POOL_H_
#define FLUTTER_IMPELLER_RENDERER_POOL_H_

#include <cstdint>
#include <memory>
#include <mutex>

namespace impeller {

/// @brief A thread-safe pool with a limited byte size.
/// @tparam T The type that the pool will contain.
template <typename T>
class Pool {
 public:
  explicit Pool(uint32_t limit_bytes) : limit_bytes_(limit_bytes) {}

  std::shared_ptr<T> Grab() {
    std::scoped_lock lock(mutex_);
    if (pool_.empty()) {
      return T::Create();
    }
    std::shared_ptr<T> result = std::move(pool_.back());
    pool_.pop_back();
    size_ -= result->GetSize();
    return result;
  }

  void Recycle(std::shared_ptr<T> object) {
    std::scoped_lock lock(mutex_);
    size_t object_size = object->GetSize();
    if (size_ + object_size <= limit_bytes_ &&
        object_size < (limit_bytes_ / 2)) {
      object->Reset();
      size_ += object_size;
      pool_.emplace_back(std::move(object));
    }
  }

  uint32_t GetSize() const {
    std::scoped_lock lock(mutex_);
    return size_;
  }

 private:
  std::vector<std::shared_ptr<T>> pool_;
  const uint32_t limit_bytes_;
  uint32_t size_ = 0;
  // Note: This would perform better as a lockless ring buffer.
  mutable std::mutex mutex_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_POOL_H_

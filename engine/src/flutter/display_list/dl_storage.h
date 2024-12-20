// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_STORAGE_H_
#define FLUTTER_DISPLAY_LIST_DL_STORAGE_H_

#include <memory>

#include "flutter/fml/logging.h"

namespace flutter {

// Manages a buffer allocated with malloc.
class DisplayListStorage {
 public:
  static const constexpr size_t kDLPageSize = 4096u;

  DisplayListStorage() = default;
  DisplayListStorage(DisplayListStorage&&);

  /// Returns a pointer to the base of the storage.
  uint8_t* base() { return ptr_.get(); }
  const uint8_t* base() const { return ptr_.get(); }

  /// Returns the currently allocated size
  size_t size() const { return used_; }

  /// Returns the maximum currently allocated space
  size_t capacity() const { return allocated_; }

  /// Ensures the indicated number of bytes are available and returns
  /// a pointer to that memory within the storage while also invalidating
  /// any other outstanding pointers into the storage.
  uint8_t* allocate(size_t needed);

  /// Trims the storage to the currently allocated size and invalidates
  /// any outstanding pointers into the storage.
  void trim() { realloc(used_); }

  /// Resets the storage and allocation of the object to an empty state
  void reset();

  DisplayListStorage& operator=(DisplayListStorage&& other);

 private:
  void realloc(size_t count);

  struct FreeDeleter {
    void operator()(uint8_t* p) { std::free(p); }
  };
  std::unique_ptr<uint8_t, FreeDeleter> ptr_;

  size_t used_ = 0u;
  size_t allocated_ = 0u;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_STORAGE_H_

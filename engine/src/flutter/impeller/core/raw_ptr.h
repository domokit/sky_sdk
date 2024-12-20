// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_CORE_RAW_PTR_H_
#define FLUTTER_IMPELLER_CORE_RAW_PTR_H_

#include <memory>

namespace impeller {

/// @brief A wrapper around a raw ptr that adds additional unopt mode only
/// checks.
template <typename T>
class raw_ptr {
 public:
  explicit raw_ptr(const std::shared_ptr<T>& ptr)
      : ptr_(ptr.get())
#if !NDEBUG
        ,
        weak_ptr_(ptr)
#endif
  {
  }

  raw_ptr() : ptr_(nullptr) {}

  T* operator->() {
#if !NDEBUG
    FML_CHECK(weak_ptr_.lock());
#endif
    return ptr_;
  }

  const T* operator->() const {
#if !NDEBUG
    FML_CHECK(weak_ptr_.lock());
#endif
    return ptr_;
  }

  T* get() {
#if !NDEBUG
    FML_CHECK(weak_ptr_.lock());
#endif
    return ptr_;
  }

  T& operator*() {
#if !NDEBUG
    FML_CHECK(weak_ptr_.lock());
#endif
    return *ptr_;
  }

  const T& operator*() const {
#if !NDEBUG
    FML_CHECK(weak_ptr_.lock());
#endif
    return *ptr_;
  }

  template <class U>
  inline bool operator==(raw_ptr<U> const& other) const {
    return ptr_ == other.ptr_;
  }

  template <class U>
  inline bool operator!=(raw_ptr<U> const& other) const {
    return !(*this == other);
  }

  explicit operator bool() const { return !!ptr_; }

 private:
  T* ptr_;
#if !NDEBUG
  std::weak_ptr<T> weak_ptr_;
#endif
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_CORE_RAW_PTR_H_

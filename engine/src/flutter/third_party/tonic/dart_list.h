// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_DART_LIST_H_
#define LIB_TONIC_DART_LIST_H_

#include <cstddef>

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/converter/dart_converter.h"

namespace tonic {

class DartList {
 public:
  DartList(DartList&& other);

  void Set(size_t index, Dart_Handle value);
  Dart_Handle Get(size_t index);

  template <class T>
  void Set(size_t index, T value) {
    Set(index, DartConverter<T>::ToDart(value));
  }

  template <class T>
  T Get(size_t index) {
    return DartConverter<T>::FromDart(Get(index));
  }

  Dart_Handle dart_handle() const { return dart_handle_; }
  size_t size() const { return size_; }
  bool is_valid() const { return is_valid_; }

  explicit operator bool() const { return is_valid_; }

 private:
  explicit DartList(Dart_Handle list);
  friend struct DartConverter<DartList>;

  DartList();
  Dart_Handle dart_handle_;
  size_t size_;
  bool is_valid_;

  DartList(const DartList& other) = delete;
};

template <>
struct DartConverter<DartList> {
  static DartList FromArguments(Dart_NativeArguments args,
                                int index,
                                Dart_Handle& exception);
};

}  // namespace tonic

#endif  // LIB_TONIC_DART_LIST_H_

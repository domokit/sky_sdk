// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_TYPED_DATA_DART_BYTE_DATA_H_
#define LIB_TONIC_TYPED_DATA_DART_BYTE_DATA_H_

#include <vector>

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/converter/dart_converter.h"

namespace tonic {

class DartByteData {
 public:
  static const size_t kExternalSizeThreshold;
  static Dart_Handle Create(const void* data, size_t length);

  explicit DartByteData(Dart_Handle list);
  DartByteData(DartByteData&& other);
  DartByteData();
  ~DartByteData();

  const void* data() const { return data_; }
  void* data() { return data_; }
  size_t length_in_bytes() const { return length_in_bytes_; }
  Dart_Handle dart_handle() const { return dart_handle_; }

  std::vector<char> Copy() const;
  void Release() const;

  explicit operator bool() const { return data_ != nullptr; }

 private:
  mutable void* data_;
  intptr_t length_in_bytes_;
  Dart_Handle dart_handle_;

  DartByteData(const DartByteData& other) = delete;
  DartByteData(const void* data, size_t length);
};

template <>
struct DartConverter<DartByteData> {
  using FfiType = Dart_Handle;

  static void SetReturnValue(Dart_NativeArguments args, DartByteData val);
  static DartByteData FromArguments(Dart_NativeArguments args,
                                    int index,
                                    Dart_Handle& exception);

  static DartByteData FromFfi(FfiType val) { return DartByteData(val); }
  static FfiType ToFfi(DartByteData val) { return val.dart_handle(); }
};

}  // namespace tonic

#endif  // LIB_TONIC_TYPED_DATA_DART_BYTE_DATA_H_

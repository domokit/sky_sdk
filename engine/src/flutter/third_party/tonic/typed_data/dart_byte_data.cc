// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/typed_data/dart_byte_data.h"

#include <cstring>

#include "tonic/logging/dart_error.h"

namespace tonic {

namespace {

void FreeFinalizer(void* isolate_callback_data, void* peer) {
  free(peer);
}

}  // anonymous namespace

// For large objects it is more efficient to use an external typed data object
// with a buffer allocated outside the Dart heap.
const size_t DartByteData::kExternalSizeThreshold = 1000;

Dart_Handle DartByteData::Create(const void* data, size_t length) {
  if (length < kExternalSizeThreshold) {
    auto handle = DartByteData{data, length}.dart_handle();
    // The destructor should release the typed data.
    return handle;
  } else {
    void* buf = ::malloc(length);
    TONIC_DCHECK(buf);
    ::memcpy(buf, data, length);
    return Dart_NewExternalTypedDataWithFinalizer(
        Dart_TypedData_kByteData, buf, length, buf, length, FreeFinalizer);
  }
}

DartByteData::DartByteData()
    : data_(nullptr), length_in_bytes_(0), dart_handle_(nullptr) {}

DartByteData::DartByteData(const void* data, size_t length)
    : data_(nullptr),
      length_in_bytes_(0),
      dart_handle_(Dart_NewTypedData(Dart_TypedData_kByteData, length)) {
  if (!Dart_IsError(dart_handle_)) {
    Dart_TypedData_Type type;
    auto acquire_result = Dart_TypedDataAcquireData(dart_handle_, &type, &data_,
                                                    &length_in_bytes_);

    if (!Dart_IsError(acquire_result)) {
      ::memcpy(data_, data, length_in_bytes_);
    }
  }
}

DartByteData::DartByteData(Dart_Handle list)
    : data_(nullptr), length_in_bytes_(0), dart_handle_(list) {
  if (Dart_IsNull(list))
    return;

  Dart_TypedData_Type type;
  Dart_TypedDataAcquireData(list, &type, &data_, &length_in_bytes_);
  TONIC_DCHECK(!CheckAndHandleError(list));
  if (type != Dart_TypedData_kByteData)
    Dart_ThrowException(ToDart("Non-genuine ByteData passed to engine."));
}

DartByteData::DartByteData(DartByteData&& other)
    : data_(other.data_),
      length_in_bytes_(other.length_in_bytes_),
      dart_handle_(other.dart_handle_) {
  other.data_ = nullptr;
  other.dart_handle_ = nullptr;
}

DartByteData::~DartByteData() {
  Release();
}

std::vector<char> DartByteData::Copy() const {
  const char* ptr = static_cast<const char*>(data_);
  return std::vector<char>(ptr, ptr + length_in_bytes_);
}

void DartByteData::Release() const {
  if (data_) {
    Dart_TypedDataReleaseData(dart_handle_);
    data_ = nullptr;
  }
}

DartByteData DartConverter<DartByteData>::FromArguments(
    Dart_NativeArguments args,
    int index,
    Dart_Handle& exception) {
  Dart_Handle data = Dart_GetNativeArgument(args, index);
  TONIC_DCHECK(!CheckAndHandleError(data));
  return DartByteData(data);
}

void DartConverter<DartByteData>::SetReturnValue(Dart_NativeArguments args,
                                                 DartByteData val) {
  Dart_SetReturnValue(args, val.dart_handle());
}

}  // namespace tonic

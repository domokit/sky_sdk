// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_TYPED_DATA_TYPED_LIST_H_
#define LIB_TONIC_TYPED_DATA_TYPED_LIST_H_

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/converter/dart_converter.h"

namespace tonic {

// A simple wrapper around Dart TypedData objects. It uses
// Dart_TypedDataAcquireData to obtain a raw pointer to the data, which is
// released when this object is destroyed.
//
// This is designed to be used with DartConverter only.
template <Dart_TypedData_Type kTypeName, typename ElemType>
class TypedList {
 public:
  explicit TypedList(Dart_Handle list);
  TypedList(TypedList<kTypeName, ElemType>&& other);
  TypedList();
  ~TypedList();

  ElemType& at(intptr_t i) {
    TONIC_CHECK(0 <= i);
    TONIC_CHECK(i < num_elements_);
    return data_[i];
  }
  const ElemType& at(intptr_t i) const {
    TONIC_CHECK(0 <= i);
    TONIC_CHECK(i < num_elements_);
    return data_[i];
  }

  ElemType& operator[](intptr_t i) { return at(i); }
  const ElemType& operator[](intptr_t i) const { return at(i); }

  const ElemType* data() const { return data_; }
  intptr_t num_elements() const { return num_elements_; }
  Dart_Handle dart_handle() const { return dart_handle_; }

  void Release();

 private:
  ElemType* data_;
  intptr_t num_elements_;
  Dart_Handle dart_handle_;
};

template <Dart_TypedData_Type kTypeName, typename ElemType>
struct DartConverter<TypedList<kTypeName, ElemType>> {
  using NativeType = TypedList<kTypeName, ElemType>;
  using FfiType = Dart_Handle;
  static constexpr const char* kFfiRepresentation = "Handle";
  static constexpr const char* kDartRepresentation = "Object";
  static constexpr bool kAllowedInLeafCall = false;

  static void SetReturnValue(Dart_NativeArguments args, NativeType val);
  static NativeType FromArguments(Dart_NativeArguments args,
                                  int index,
                                  Dart_Handle& exception);
  static Dart_Handle ToDart(const ElemType* buffer, unsigned int length);

  static NativeType FromFfi(FfiType val) { return NativeType(val); }
  static FfiType ToFfi(NativeType val) {
    Dart_Handle handle = val.dart_handle();
    val.Release();
    return handle;
  }
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

#define TONIC_TYPED_DATA_FOREACH(F) \
  F(Int8, int8_t)                   \
  F(Uint8, uint8_t)                 \
  F(Int16, int16_t)                 \
  F(Uint16, uint16_t)               \
  F(Int32, int32_t)                 \
  F(Uint32, uint32_t)               \
  F(Int64, int64_t)                 \
  F(Uint64, uint64_t)               \
  F(Float32, float)                 \
  F(Float64, double)

#define TONIC_TYPED_DATA_DECLARE(name, type)                     \
  using name##List = TypedList<Dart_TypedData_k##name, type>;    \
  extern template class TypedList<Dart_TypedData_k##name, type>; \
  extern template struct DartConverter<name##List>;

TONIC_TYPED_DATA_FOREACH(TONIC_TYPED_DATA_DECLARE)

#undef TONIC_TYPED_DATA_DECLARE

}  // namespace tonic

#endif  // LIB_TONIC_TYPED_DATA_TYPED_LIST_H_

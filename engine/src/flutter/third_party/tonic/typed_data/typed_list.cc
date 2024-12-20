// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/typed_data/typed_list.h"

#include <cstring>

#include "tonic/logging/dart_error.h"

namespace tonic {

template <Dart_TypedData_Type kTypeName, typename ElemType>
TypedList<kTypeName, ElemType>::TypedList()
    : data_(nullptr), num_elements_(0), dart_handle_(nullptr) {}

template <Dart_TypedData_Type kTypeName, typename ElemType>
TypedList<kTypeName, ElemType>::TypedList(Dart_Handle list)
    : data_(nullptr), num_elements_(0), dart_handle_(list) {
  if (Dart_IsNull(list))
    return;

  Dart_TypedData_Type type;
  Dart_TypedDataAcquireData(list, &type, reinterpret_cast<void**>(&data_),
                            &num_elements_);
  TONIC_DCHECK(!CheckAndHandleError(list));
  if (type != kTypeName)
    Dart_ThrowException(ToDart("Non-genuine TypedData passed to engine."));
}

template <Dart_TypedData_Type kTypeName, typename ElemType>
TypedList<kTypeName, ElemType>::TypedList(
    TypedList<kTypeName, ElemType>&& other)
    : data_(other.data_),
      num_elements_(other.num_elements_),
      dart_handle_(other.dart_handle_) {
  other.data_ = nullptr;
  other.num_elements_ = 0;
  other.dart_handle_ = nullptr;
}

template <Dart_TypedData_Type kTypeName, typename ElemType>
TypedList<kTypeName, ElemType>::~TypedList() {
  Release();
}

template <Dart_TypedData_Type kTypeName, typename ElemType>
void TypedList<kTypeName, ElemType>::Release() {
  if (data_) {
    Dart_TypedDataReleaseData(dart_handle_);
    data_ = nullptr;
    num_elements_ = 0;
    dart_handle_ = nullptr;
  }
}

template <Dart_TypedData_Type kTypeName, typename ElemType>
TypedList<kTypeName, ElemType>
DartConverter<TypedList<kTypeName, ElemType>>::FromArguments(
    Dart_NativeArguments args,
    int index,
    Dart_Handle& exception) {
  Dart_Handle list = Dart_GetNativeArgument(args, index);
  TONIC_DCHECK(!CheckAndHandleError(list));
  return TypedList<kTypeName, ElemType>(list);
}

template <Dart_TypedData_Type kTypeName, typename ElemType>
void DartConverter<TypedList<kTypeName, ElemType>>::SetReturnValue(
    Dart_NativeArguments args,
    TypedList<kTypeName, ElemType> val) {
  Dart_Handle result = val.dart_handle();
  val.Release();  // Must release acquired typed data before calling Dart API.
  Dart_SetReturnValue(args, result);
}

template <Dart_TypedData_Type kTypeName, typename ElemType>
Dart_Handle DartConverter<TypedList<kTypeName, ElemType>>::ToDart(
    const ElemType* buffer,
    unsigned int length) {
  const intptr_t buffer_length = static_cast<intptr_t>(length);
  Dart_Handle array = Dart_NewTypedData(kTypeName, buffer_length);
  TONIC_DCHECK(!CheckAndHandleError(array));
  {
    Dart_TypedData_Type type;
    void* data = nullptr;
    intptr_t data_length = 0;
    Dart_TypedDataAcquireData(array, &type, &data, &data_length);
    TONIC_CHECK(type == kTypeName);
    TONIC_CHECK(data);
    TONIC_CHECK(data_length == buffer_length);
    std::memmove(data, buffer, data_length * sizeof(ElemType));
    Dart_TypedDataReleaseData(array);
  }
  return array;
}

#define TONIC_TYPED_DATA_DEFINE(name, type)               \
  template class TypedList<Dart_TypedData_k##name, type>; \
  template struct DartConverter<name##List>;

TONIC_TYPED_DATA_FOREACH(TONIC_TYPED_DATA_DEFINE)

#undef TONIC_TYPED_DATA_DEFINE

}  // namespace tonic

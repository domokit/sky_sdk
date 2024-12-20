// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_WIN_VARIANT_UTIL_H_
#define BASE_WIN_VARIANT_UTIL_H_

#include "base/logging.h"

namespace base {
namespace win {
namespace internal {

// Returns true if a VARIANT of type |self| can be assigned to a
// variant of type |other|.
// Does not allow converting unsigned <-> signed or converting between
// different sized types, but does allow converting IDispatch* -> IUnknown*.
constexpr bool VarTypeIsConvertibleTo(VARTYPE self, VARTYPE other) {
  // IDispatch inherits from IUnknown, so it's safe to
  // upcast a VT_DISPATCH into an IUnknown*.
  return (self == other) || (self == VT_DISPATCH && other == VT_UNKNOWN);
}

// VartypeToNativeType contains the underlying |Type| and offset to the
// VARIANT union member related to the |ElementVartype| for simple types.
template <VARTYPE ElementVartype>
struct VartypeToNativeType final {};

template <>
struct VartypeToNativeType<VT_BOOL> final {
  using Type = VARIANT_BOOL;
  static constexpr VARIANT_BOOL VARIANT::*kMemberOffset = &VARIANT::boolVal;
};

template <>
struct VartypeToNativeType<VT_I1> final {
  using Type = int8_t;
  static constexpr CHAR VARIANT::*kMemberOffset = &VARIANT::cVal;
};

template <>
struct VartypeToNativeType<VT_UI1> final {
  using Type = uint8_t;
  static constexpr BYTE VARIANT::*kMemberOffset = &VARIANT::bVal;
};

template <>
struct VartypeToNativeType<VT_I2> final {
  using Type = int16_t;
  static constexpr SHORT VARIANT::*kMemberOffset = &VARIANT::iVal;
};

template <>
struct VartypeToNativeType<VT_UI2> final {
  using Type = uint16_t;
  static constexpr USHORT VARIANT::*kMemberOffset = &VARIANT::uiVal;
};

template <>
struct VartypeToNativeType<VT_I4> final {
  using Type = int32_t;
  static constexpr LONG VARIANT::*kMemberOffset = &VARIANT::lVal;
};

template <>
struct VartypeToNativeType<VT_UI4> final {
  using Type = uint32_t;
  static constexpr ULONG VARIANT::*kMemberOffset = &VARIANT::ulVal;
};

template <>
struct VartypeToNativeType<VT_I8> final {
  using Type = int64_t;
  static constexpr LONGLONG VARIANT::*kMemberOffset = &VARIANT::llVal;
};

template <>
struct VartypeToNativeType<VT_UI8> final {
  using Type = uint64_t;
  static constexpr ULONGLONG VARIANT::*kMemberOffset = &VARIANT::ullVal;
};

template <>
struct VartypeToNativeType<VT_R4> final {
  using Type = float;
  static constexpr FLOAT VARIANT::*kMemberOffset = &VARIANT::fltVal;
};

template <>
struct VartypeToNativeType<VT_R8> final {
  using Type = double;
  static constexpr DOUBLE VARIANT::*kMemberOffset = &VARIANT::dblVal;
};

template <>
struct VartypeToNativeType<VT_DATE> final {
  using Type = DATE;
  static constexpr DATE VARIANT::*kMemberOffset = &VARIANT::date;
};

template <>
struct VartypeToNativeType<VT_BSTR> final {
  using Type = BSTR;
  static constexpr BSTR VARIANT::*kMemberOffset = &VARIANT::bstrVal;
};

template <>
struct VartypeToNativeType<VT_UNKNOWN> final {
  using Type = IUnknown*;
  static constexpr IUnknown* VARIANT::*kMemberOffset = &VARIANT::punkVal;
};

template <>
struct VartypeToNativeType<VT_DISPATCH> final {
  using Type = IDispatch*;
  static constexpr IDispatch* VARIANT::*kMemberOffset = &VARIANT::pdispVal;
};

// VariantUtil contains the underlying |Type| and helper methods
// related to the |ElementVartype| for simple types.
template <VARTYPE ElementVartype>
struct VariantUtil final {
  using Type = typename VartypeToNativeType<ElementVartype>::Type;
  static constexpr bool IsConvertibleTo(VARTYPE vartype) {
    return VarTypeIsConvertibleTo(ElementVartype, vartype);
  }
  static constexpr bool IsConvertibleFrom(VARTYPE vartype) {
    return VarTypeIsConvertibleTo(vartype, ElementVartype);
  }
  // Get the associated VARIANT union member value.
  // Returns the value owned by the VARIANT without affecting the lifetime
  // of managed contents.
  // e.g. Does not affect IUnknown* reference counts or allocate a BSTR.
  static Type RawGet(const VARIANT& var) {
    BASE_DCHECK(IsConvertibleFrom(V_VT(&var)));
    return var.*VartypeToNativeType<ElementVartype>::kMemberOffset;
  }
  // Set the associated VARIANT union member value.
  // The caller is responsible for handling the lifetime of managed contents.
  // e.g. Incrementing IUnknown* reference counts or allocating a BSTR.
  static void RawSet(VARIANT* var, Type value) {
    BASE_DCHECK(IsConvertibleTo(V_VT(var)));
    var->*VartypeToNativeType<ElementVartype>::kMemberOffset = value;
  }
};

}  // namespace internal
}  // namespace win
}  // namespace base

#endif  // BASE_WIN_VARIANT_UTIL_H_

// Copyright (c) 2010 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/win/scoped_variant.h"

#include <propvarutil.h>
#include <wrl/client.h>

#include <algorithm>
#include <functional>

#include "base/logging.h"
#include "base/numerics/ranges.h"
#include "base/win/variant_util.h"

namespace base {
namespace win {

// Global, const instance of an empty variant.
const VARIANT ScopedVariant::kEmptyVariant = {{{VT_EMPTY}}};

ScopedVariant::ScopedVariant(ScopedVariant&& var) {
  var_.vt = VT_EMPTY;
  Reset(var.Release());
}

ScopedVariant::~ScopedVariant() {
  static_assert(sizeof(ScopedVariant) == sizeof(VARIANT), "ScopedVariantSize");
  ::VariantClear(&var_);
}

ScopedVariant::ScopedVariant(const wchar_t* str) {
  var_.vt = VT_EMPTY;
  Set(str);
}

ScopedVariant::ScopedVariant(const wchar_t* str, UINT length) {
  var_.vt = VT_BSTR;
  var_.bstrVal = ::SysAllocStringLen(str, length);
}

ScopedVariant::ScopedVariant(long value, VARTYPE vt) {
  var_.vt = vt;
  var_.lVal = value;
}

ScopedVariant::ScopedVariant(int value) {
  var_.vt = VT_I4;
  var_.lVal = value;
}

ScopedVariant::ScopedVariant(bool value) {
  var_.vt = VT_BOOL;
  var_.boolVal = value ? VARIANT_TRUE : VARIANT_FALSE;
}

ScopedVariant::ScopedVariant(double value, VARTYPE vt) {
  BASE_DCHECK(vt == VT_R8 || vt == VT_DATE);
  var_.vt = vt;
  var_.dblVal = value;
}

ScopedVariant::ScopedVariant(IDispatch* dispatch) {
  var_.vt = VT_EMPTY;
  Set(dispatch);
}

ScopedVariant::ScopedVariant(IUnknown* unknown) {
  var_.vt = VT_EMPTY;
  Set(unknown);
}

ScopedVariant::ScopedVariant(SAFEARRAY* safearray) {
  var_.vt = VT_EMPTY;
  Set(safearray);
}

ScopedVariant::ScopedVariant(const VARIANT& var) {
  var_.vt = VT_EMPTY;
  Set(var);
}

void ScopedVariant::Reset(const VARIANT& var) {
  if (&var != &var_) {
    ::VariantClear(&var_);
    var_ = var;
  }
}

VARIANT ScopedVariant::Release() {
  VARIANT var = var_;
  var_.vt = VT_EMPTY;
  return var;
}

void ScopedVariant::Swap(ScopedVariant& var) {
  VARIANT tmp = var_;
  var_ = var.var_;
  var.var_ = tmp;
}

VARIANT* ScopedVariant::Receive() {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "variant leak. type: " << var_.vt;
  return &var_;
}

VARIANT ScopedVariant::Copy() const {
  VARIANT ret = {{{VT_EMPTY}}};
  ::VariantCopy(&ret, &var_);
  return ret;
}

int ScopedVariant::Compare(const VARIANT& other, bool ignore_case) const {
  BASE_DCHECK(!V_ISARRAY(&var_))
      << "Comparison is not supported when |this| owns a SAFEARRAY";
  BASE_DCHECK(!V_ISARRAY(&other))
      << "Comparison is not supported when |other| owns a SAFEARRAY";

  const bool this_is_empty = var_.vt == VT_EMPTY || var_.vt == VT_NULL;
  const bool other_is_empty = other.vt == VT_EMPTY || other.vt == VT_NULL;

  // 1. VT_NULL and VT_EMPTY is always considered less-than any other VARTYPE.
  if (this_is_empty)
    return other_is_empty ? 0 : -1;
  if (other_is_empty)
    return 1;

  // 2. If both VARIANTS have either VT_UNKNOWN or VT_DISPATCH even if the
  //    VARTYPEs do not match, the address of its IID_IUnknown is compared to
  //    guarantee a logical ordering even though it is not a meaningful order.
  //    e.g. (a.Compare(b) != b.Compare(a)) unless (a == b).
  const bool this_is_unknown = var_.vt == VT_UNKNOWN || var_.vt == VT_DISPATCH;
  const bool other_is_unknown =
      other.vt == VT_UNKNOWN || other.vt == VT_DISPATCH;
  if (this_is_unknown && other_is_unknown) {
    // https://docs.microsoft.com/en-us/windows/win32/com/rules-for-implementing-queryinterface
    // Query IID_IUnknown to determine whether the two variants point
    // to the same instance of an object
    Microsoft::WRL::ComPtr<IUnknown> this_unknown;
    Microsoft::WRL::ComPtr<IUnknown> other_unknown;
    V_UNKNOWN(&var_)->QueryInterface(IID_PPV_ARGS(&this_unknown));
    V_UNKNOWN(&other)->QueryInterface(IID_PPV_ARGS(&other_unknown));
    if (this_unknown.Get() == other_unknown.Get())
      return 0;
    // std::less for any pointer type yields a strict total order even if the
    // built-in operator< does not.
    return std::less<>{}(this_unknown.Get(), other_unknown.Get()) ? -1 : 1;
  }

  // 3. If the VARTYPEs do not match, then the value of the VARTYPE is compared.
  if (V_VT(&var_) != V_VT(&other))
    return (V_VT(&var_) < V_VT(&other)) ? -1 : 1;

  const VARTYPE shared_vartype = V_VT(&var_);
  // 4. Comparing VT_BSTR values is a lexicographical comparison of the contents
  //    of the BSTR, taking into account |ignore_case|.
  if (shared_vartype == VT_BSTR) {
    ULONG flags = ignore_case ? NORM_IGNORECASE : 0;
    HRESULT hr =
        ::VarBstrCmp(V_BSTR(&var_), V_BSTR(&other), LOCALE_USER_DEFAULT, flags);
    BASE_DCHECK(SUCCEEDED(hr) && hr != VARCMP_NULL)
        << "unsupported variant comparison: " << var_.vt << " and " << other.vt;

    switch (hr) {
      case VARCMP_LT:
        return -1;
      case VARCMP_GT:
      case VARCMP_NULL:
        return 1;
      default:
        return 0;
    }
  }

  // 5. Otherwise returns the lexicographical comparison of the values held by
  //    the two VARIANTS that share the same VARTYPE.
  return ::VariantCompare(var_, other);
}

void ScopedVariant::Set(const wchar_t* str) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_BSTR;
  var_.bstrVal = ::SysAllocString(str);
}

void ScopedVariant::Set(int8_t i8) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_I1;
  var_.cVal = i8;
}

void ScopedVariant::Set(uint8_t ui8) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_UI1;
  var_.bVal = ui8;
}

void ScopedVariant::Set(int16_t i16) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_I2;
  var_.iVal = i16;
}

void ScopedVariant::Set(uint16_t ui16) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_UI2;
  var_.uiVal = ui16;
}

void ScopedVariant::Set(int32_t i32) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_I4;
  var_.lVal = i32;
}

void ScopedVariant::Set(uint32_t ui32) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_UI4;
  var_.ulVal = ui32;
}

void ScopedVariant::Set(int64_t i64) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_I8;
  var_.llVal = i64;
}

void ScopedVariant::Set(uint64_t ui64) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_UI8;
  var_.ullVal = ui64;
}

void ScopedVariant::Set(float r32) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_R4;
  var_.fltVal = r32;
}

void ScopedVariant::Set(double r64) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_R8;
  var_.dblVal = r64;
}

void ScopedVariant::SetDate(DATE date) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_DATE;
  var_.date = date;
}

void ScopedVariant::Set(IDispatch* disp) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_DISPATCH;
  var_.pdispVal = disp;
  if (disp)
    disp->AddRef();
}

void ScopedVariant::Set(bool b) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_BOOL;
  var_.boolVal = b ? VARIANT_TRUE : VARIANT_FALSE;
}

void ScopedVariant::Set(IUnknown* unk) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  var_.vt = VT_UNKNOWN;
  var_.punkVal = unk;
  if (unk)
    unk->AddRef();
}

void ScopedVariant::Set(SAFEARRAY* array) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  if (SUCCEEDED(::SafeArrayGetVartype(array, &var_.vt))) {
    var_.vt |= VT_ARRAY;
    var_.parray = array;
  } else {
    BASE_DCHECK(!array) << "Unable to determine safearray vartype";
    var_.vt = VT_EMPTY;
  }
}

void ScopedVariant::Set(const VARIANT& var) {
  BASE_DCHECK(!IsLeakableVarType(var_.vt)) << "leaking variant: " << var_.vt;
  if (FAILED(::VariantCopy(&var_, &var))) {
    BASE_DLOG() << "Error: VariantCopy failed";
    var_.vt = VT_EMPTY;
  }
}

ScopedVariant& ScopedVariant::operator=(ScopedVariant&& var) {
  if (var.ptr() != &var_)
    Reset(var.Release());
  return *this;
}

ScopedVariant& ScopedVariant::operator=(const VARIANT& var) {
  if (&var != &var_) {
    VariantClear(&var_);
    Set(var);
  }
  return *this;
}

bool ScopedVariant::IsLeakableVarType(VARTYPE vt) {
  bool leakable = false;
  switch (vt & VT_TYPEMASK) {
    case VT_BSTR:
    case VT_DISPATCH:
    // we treat VT_VARIANT as leakable to err on the safe side.
    case VT_VARIANT:
    case VT_UNKNOWN:
    case VT_SAFEARRAY:

    // very rarely used stuff (if ever):
    case VT_VOID:
    case VT_PTR:
    case VT_CARRAY:
    case VT_USERDEFINED:
    case VT_LPSTR:
    case VT_LPWSTR:
    case VT_RECORD:
    case VT_INT_PTR:
    case VT_UINT_PTR:
    case VT_FILETIME:
    case VT_BLOB:
    case VT_STREAM:
    case VT_STORAGE:
    case VT_STREAMED_OBJECT:
    case VT_STORED_OBJECT:
    case VT_BLOB_OBJECT:
    case VT_VERSIONED_STREAM:
    case VT_BSTR_BLOB:
      leakable = true;
      break;
  }

  if (!leakable && (vt & VT_ARRAY) != 0) {
    leakable = true;
  }

  return leakable;
}

}  // namespace win
}  // namespace base

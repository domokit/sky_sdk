// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/win/variant_vector.h"

#include "base/win/scoped_safearray.h"
#include "base/win/scoped_variant.h"

namespace base {
namespace win {

namespace {

// Lexicographical comparison between the contents of |vector| and |safearray|.
template <VARTYPE ElementVartype>
int CompareAgainstSafearray(const std::vector<ScopedVariant>& vector,
                            const ScopedSafearray& safearray,
                            bool ignore_case) {
  std::optional<ScopedSafearray::LockScope<ElementVartype>> lock_scope =
      safearray.CreateLockScope<ElementVartype>();
  // If we fail to create a lock scope, then arbitrarily treat |this| as
  // greater. This should only happen when the SAFEARRAY fails to be locked,
  // so we cannot compare the contents of the SAFEARRAY.
  if (!lock_scope)
    return 1;

  // Create a temporary VARIANT which does not own its contents, and is
  // populated with values from the |lock_scope| so it can be compared against.
  VARIANT non_owning_temp;
  V_VT(&non_owning_temp) = ElementVartype;

  auto vector_iter = vector.begin();
  auto scope_iter = lock_scope->begin();
  for (; vector_iter != vector.end() && scope_iter != lock_scope->end();
       ++vector_iter, ++scope_iter) {
    internal::VariantUtil<ElementVartype>::RawSet(&non_owning_temp,
                                                  *scope_iter);
    int compare_result = vector_iter->Compare(non_owning_temp, ignore_case);
    // If there is a difference in values, return the difference.
    if (compare_result)
      return compare_result;
  }
  // There are more elements in |vector|, so |vector| is
  // greater than |safearray|.
  if (vector_iter != vector.end())
    return 1;
  // There are more elements in |safearray|, so |vector| is
  // less than |safearray|.
  if (scope_iter != lock_scope->end())
    return -1;
  return 0;
}

}  // namespace

VariantVector::VariantVector() = default;

VariantVector::VariantVector(VariantVector&& other)
    : vartype_(std::exchange(other.vartype_, VT_EMPTY)),
      vector_(std::move(other.vector_)) {}

VariantVector& VariantVector::operator=(VariantVector&& other) {
  BASE_DCHECK(this != &other);
  vartype_ = std::exchange(other.vartype_, VT_EMPTY);
  vector_ = std::move(other.vector_);
  return *this;
}

VariantVector::~VariantVector() {
  Reset();
}

bool VariantVector::operator==(const VariantVector& other) const {
  return !Compare(other);
}

bool VariantVector::operator!=(const VariantVector& other) const {
  return !VariantVector::operator==(other);
}

void VariantVector::Reset() {
  vector_.clear();
  vartype_ = VT_EMPTY;
}

VARIANT VariantVector::ReleaseAsScalarVariant() {
  ScopedVariant scoped_variant;

  if (!Empty()) {
    BASE_DCHECK(Size() == 1U);
    scoped_variant = std::move(vector_[0]);
    Reset();
  }

  return scoped_variant.Release();
}

VARIANT VariantVector::ReleaseAsSafearrayVariant() {
  ScopedVariant scoped_variant;

  switch (Type()) {
    case VT_EMPTY:
      break;
    case VT_BOOL:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_BOOL>());
      break;
    case VT_I1:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_I1>());
      break;
    case VT_UI1:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_UI1>());
      break;
    case VT_I2:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_I2>());
      break;
    case VT_UI2:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_UI2>());
      break;
    case VT_I4:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_I4>());
      break;
    case VT_UI4:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_UI4>());
      break;
    case VT_I8:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_I8>());
      break;
    case VT_UI8:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_UI8>());
      break;
    case VT_R4:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_R4>());
      break;
    case VT_R8:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_R8>());
      break;
    case VT_DATE:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_DATE>());
      break;
    case VT_BSTR:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_BSTR>());
      break;
    case VT_DISPATCH:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_DISPATCH>());
      break;
    case VT_UNKNOWN:
      scoped_variant.Set(CreateAndPopulateSafearray<VT_UNKNOWN>());
      break;
    // The default case shouldn't be reachable, but if we added support for more
    // VARTYPEs to base::win::internal::VariantUtil<> and they were inserted
    // into a VariantVector then it would be possible to reach the default case
    // for those new types until implemented.
    //
    // Because the switch is against VARTYPE (unsigned short) and not VARENUM,
    // removing the default case will not result in build warnings/errors if
    // there are missing cases. It is important that this uses VARTYPE rather
    // than VARENUM, because in the future we may want to support complex
    // VARTYPES. For example a value within VT_TYPEMASK that's joined something
    // outside the typemask like VT_ARRAY or VT_BYREF.
    default:
      BASE_UNREACHABLE();
      break;
  }

  // CreateAndPopulateSafearray handles resetting |this| to VT_EMPTY because it
  // transfers ownership of each element to the SAFEARRAY.
  return scoped_variant.Release();
}

int VariantVector::Compare(const VARIANT& other, bool ignore_case) const {
  // If the element variant types are different, compare against the types.
  if (Type() != (V_VT(&other) & VT_TYPEMASK))
    return (Type() < (V_VT(&other) & VT_TYPEMASK)) ? (-1) : 1;

  // Both have an empty variant type so they are the same.
  if (Type() == VT_EMPTY)
    return 0;

  int compare_result = 0;
  if (V_ISARRAY(&other)) {
    compare_result = Compare(V_ARRAY(&other), ignore_case);
  } else {
    compare_result = vector_[0].Compare(other, ignore_case);
    // If the first element is equal to |other|, and |vector_|
    // has more than one element, then |vector_| is greater.
    if (!compare_result && Size() > 1)
      compare_result = 1;
  }
  return compare_result;
}

int VariantVector::Compare(const VariantVector& other, bool ignore_case) const {
  // If the element variant types are different, compare against the types.
  if (Type() != other.Type())
    return (Type() < other.Type()) ? (-1) : 1;

  // Both have an empty variant type so they are the same.
  if (Type() == VT_EMPTY)
    return 0;

  auto iter1 = vector_.begin();
  auto iter2 = other.vector_.begin();
  for (; (iter1 != vector_.end()) && (iter2 != other.vector_.end());
       ++iter1, ++iter2) {
    int compare_result = iter1->Compare(*iter2, ignore_case);
    if (compare_result)
      return compare_result;
  }
  // There are more elements in |this|, so |this| is greater than |other|.
  if (iter1 != vector_.end())
    return 1;
  // There are more elements in |other|, so |this| is less than |other|.
  if (iter2 != other.vector_.end())
    return -1;
  return 0;
}

int VariantVector::Compare(SAFEARRAY* safearray, bool ignore_case) const {
  VARTYPE safearray_vartype;
  // If we fail to get the element variant type for the SAFEARRAY, then
  // arbitrarily treat |this| as greater.
  if (FAILED(SafeArrayGetVartype(safearray, &safearray_vartype)))
    return 1;

  // If the element variant types are different, compare against the types.
  if (Type() != safearray_vartype)
    return (Type() < safearray_vartype) ? (-1) : 1;

  ScopedSafearray scoped_safearray(safearray);
  int compare_result = 0;
  switch (Type()) {
    case VT_BOOL:
      compare_result = CompareAgainstSafearray<VT_BOOL>(
          vector_, scoped_safearray, ignore_case);
      break;
    case VT_I1:
      compare_result = CompareAgainstSafearray<VT_I1>(vector_, scoped_safearray,
                                                      ignore_case);
      break;
    case VT_UI1:
      compare_result = CompareAgainstSafearray<VT_UI1>(
          vector_, scoped_safearray, ignore_case);
      break;
    case VT_I2:
      compare_result = CompareAgainstSafearray<VT_I2>(vector_, scoped_safearray,
                                                      ignore_case);
      break;
    case VT_UI2:
      compare_result = CompareAgainstSafearray<VT_UI2>(
          vector_, scoped_safearray, ignore_case);
      break;
    case VT_I4:
      compare_result = CompareAgainstSafearray<VT_I4>(vector_, scoped_safearray,
                                                      ignore_case);
      break;
    case VT_UI4:
      compare_result = CompareAgainstSafearray<VT_UI4>(
          vector_, scoped_safearray, ignore_case);
      break;
    case VT_I8:
      compare_result = CompareAgainstSafearray<VT_I8>(vector_, scoped_safearray,
                                                      ignore_case);
      break;
    case VT_UI8:
      compare_result = CompareAgainstSafearray<VT_UI8>(
          vector_, scoped_safearray, ignore_case);
      break;
    case VT_R4:
      compare_result = CompareAgainstSafearray<VT_R4>(vector_, scoped_safearray,
                                                      ignore_case);
      break;
    case VT_R8:
      compare_result = CompareAgainstSafearray<VT_R8>(vector_, scoped_safearray,
                                                      ignore_case);
      break;
    case VT_DATE:
      compare_result = CompareAgainstSafearray<VT_DATE>(
          vector_, scoped_safearray, ignore_case);
      break;
    case VT_BSTR:
      compare_result = CompareAgainstSafearray<VT_BSTR>(
          vector_, scoped_safearray, ignore_case);
      break;
    case VT_DISPATCH:
      compare_result = CompareAgainstSafearray<VT_DISPATCH>(
          vector_, scoped_safearray, ignore_case);
      break;
    case VT_UNKNOWN:
      compare_result = CompareAgainstSafearray<VT_UNKNOWN>(
          vector_, scoped_safearray, ignore_case);
      break;
    // The default case shouldn't be reachable, but if we added support for more
    // VARTYPEs to base::win::internal::VariantUtil<> and they were inserted
    // into a VariantVector then it would be possible to reach the default case
    // for those new types until implemented.
    //
    // Because the switch is against VARTYPE (unsigned short) and not VARENUM,
    // removing the default case will not result in build warnings/errors if
    // there are missing cases. It is important that this uses VARTYPE rather
    // than VARENUM, because in the future we may want to support complex
    // VARTYPES. For example a value within VT_TYPEMASK that's joined something
    // outside the typemask like VT_ARRAY or VT_BYREF.
    default:
      compare_result = 1;
      BASE_UNREACHABLE();
      break;
  }

  scoped_safearray.Release();
  return compare_result;
}

template <VARTYPE ElementVartype>
SAFEARRAY* VariantVector::CreateAndPopulateSafearray() {
  BASE_DCHECK(!Empty());

  ScopedSafearray scoped_safearray(
      SafeArrayCreateVector(ElementVartype, 0, Size()));
  if (!scoped_safearray.Get()) {
    constexpr size_t kElementSize =
        sizeof(typename internal::VariantUtil<ElementVartype>::Type);
    std::abort();
  }

  std::optional<ScopedSafearray::LockScope<ElementVartype>> lock_scope =
      scoped_safearray.CreateLockScope<ElementVartype>();
  BASE_DCHECK(lock_scope);

  for (size_t i = 0; i < Size(); ++i) {
    VARIANT element = vector_[i].Release();
    (*lock_scope)[i] = internal::VariantUtil<ElementVartype>::RawGet(element);
  }
  Reset();

  return scoped_safearray.Release();
}

}  // namespace win
}  // namespace base

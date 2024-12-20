// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_WIN_VARIANT_VECTOR_H_
#define BASE_WIN_VARIANT_VECTOR_H_

#include <objbase.h>
#include <oleauto.h>

#include <type_traits>
#include <utility>
#include <vector>

#include "base/base_export.h"
#include "base/logging.h"
#include "base/win/scoped_variant.h"
#include "base/win/variant_util.h"

namespace base {
namespace win {

// This class has RAII semantics and is used to build a vector for a specific
// OLE VARTYPE, and handles converting the data to a VARIANT or VARIANT
// SAFEARRAY. It can be populated similarly to a STL vector<T>, but without the
// compile-time requirement of knowing what element type the VariantVector will
// store. The VariantVector only allows one variant type to be stored at a time.
//
// This class can release ownership of its contents to a VARIANT, and will
// automatically allocate + populate a SAFEARRAY as needed or when explicitly
// requesting that the results be released as a SAFEARRAY.
class BASE_EXPORT VariantVector final {
 public:
  VariantVector();
  VariantVector(VariantVector&& other);
  VariantVector& operator=(VariantVector&& other);
  VariantVector(const VariantVector&) = delete;
  VariantVector& operator=(const VariantVector&) = delete;
  ~VariantVector();

  bool operator==(const VariantVector& other) const;
  bool operator!=(const VariantVector& other) const;

  // Returns the variant type for data stored in the VariantVector.
  VARTYPE Type() const { return vartype_; }

  // Returns the number of elements in the VariantVector.
  size_t Size() const { return vector_.size(); }

  // Returns whether or not there are any elements.
  bool Empty() const { return vector_.empty(); }

  // Resets VariantVector to its default state, releasing any managed content.
  void Reset();

  // Helper template method for selecting the correct |Insert| call based
  // on the underlying type that is expected for a VARTYPE.
  template <VARTYPE ExpectedVartype,
            std::enable_if_t<ExpectedVartype != VT_BOOL, int> = 0>
  void Insert(typename internal::VariantUtil<ExpectedVartype>::Type value) {
    if (vartype_ == VT_EMPTY)
      vartype_ = ExpectedVartype;
    AssertVartype<ExpectedVartype>();
    ScopedVariant scoped_variant;
    scoped_variant.Set(value);
    vector_.push_back(std::move(scoped_variant));
  }

  // Specialize VT_BOOL to accept a bool type instead of VARIANT_BOOL,
  // this is to make calling insert with VT_BOOL safer.
  template <VARTYPE ExpectedVartype,
            std::enable_if_t<ExpectedVartype == VT_BOOL, int> = 0>
  void Insert(bool value) {
    if (vartype_ == VT_EMPTY)
      vartype_ = ExpectedVartype;
    AssertVartype<ExpectedVartype>();
    ScopedVariant scoped_variant;
    scoped_variant.Set(value);
    vector_.push_back(std::move(scoped_variant));
  }

  // Specialize VT_DATE because ScopedVariant has a separate SetDate method,
  // this is because VT_R8 and VT_DATE share the same underlying type.
  template <>
  void Insert<VT_DATE>(typename internal::VariantUtil<VT_DATE>::Type value) {
    if (vartype_ == VT_EMPTY)
      vartype_ = VT_DATE;
    AssertVartype<VT_DATE>();
    ScopedVariant scoped_variant;
    scoped_variant.SetDate(value);
    vector_.push_back(std::move(scoped_variant));
  }

  // Populates a VARIANT based on what is stored, transferring ownership
  // of managed contents.
  // This is only valid when the VariantVector is empty or has a single element.
  // The VariantVector is then reset.
  VARIANT ReleaseAsScalarVariant();

  // Populates a VARIANT as a SAFEARRAY, even if there is only one element.
  // The VariantVector is then reset.
  VARIANT ReleaseAsSafearrayVariant();

  // Lexicographical comparison between a VariantVector and a VARIANT.
  // The return value is 0 if the variants are equal, 1 if this object is
  // greater than |other|, -1 if it is smaller.
  int Compare(const VARIANT& other, bool ignore_case = false) const;

  // Lexicographical comparison between a VariantVector and a SAFEARRAY.
  int Compare(SAFEARRAY* safearray, bool ignore_case = false) const;

  // Lexicographical comparison between two VariantVectors.
  int Compare(const VariantVector& other, bool ignore_case = false) const;

 private:
  // Returns true if the current |vartype_| is compatible with |ExpectedVartype|
  // for inserting into |vector_|.
  template <VARTYPE ExpectedVartype>
  void AssertVartype() const {
    BASE_DCHECK(
        internal::VariantUtil<ExpectedVartype>::IsConvertibleTo(vartype_))
        << "Type mismatch, " << ExpectedVartype << " is not convertible to "
        << Type();
  }

  // Creates a SAFEARRAY and populates it with the values held by each VARIANT
  // in |vector_|, transferring ownership to the new SAFEARRAY.
  // The VariantVector is reset when successful.
  template <VARTYPE ElementVartype>
  SAFEARRAY* CreateAndPopulateSafearray();

  VARTYPE vartype_ = VT_EMPTY;
  std::vector<ScopedVariant> vector_;
};

}  // namespace win
}  // namespace base

#endif  // BASE_WIN_VARIANT_VECTOR_H_

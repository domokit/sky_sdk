// Copyright (c) 2011 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_WIN_SCOPED_VARIANT_H_
#define BASE_WIN_SCOPED_VARIANT_H_

#include <windows.h>

#include <oleauto.h>

#include <cstdint>

#include "base/base_export.h"
#include "base/macros.h"

namespace base {
namespace win {

// Scoped VARIANT class for automatically freeing a COM VARIANT at the
// end of a scope.  Additionally provides a few functions to make the
// encapsulated VARIANT easier to use.
// Instead of inheriting from VARIANT, we take the containment approach
// in order to have more control over the usage of the variant and guard
// against memory leaks.
class BASE_EXPORT ScopedVariant {
 public:
  // Declaration of a global variant variable that's always VT_EMPTY
  static const VARIANT kEmptyVariant;

  // Default constructor.
  ScopedVariant() {
    // This is equivalent to what VariantInit does, but less code.
    var_.vt = VT_EMPTY;
  }

  // Constructor to create a new VT_BSTR VARIANT.
  // NOTE: Do not pass a BSTR to this constructor expecting ownership to
  // be transferred
  explicit ScopedVariant(const wchar_t* str);

  // Creates a new VT_BSTR variant of a specified length.
  ScopedVariant(const wchar_t* str, UINT length);

  // Creates a new integral type variant and assigns the value to
  // VARIANT.lVal (32 bit sized field).
  explicit ScopedVariant(long value, VARTYPE vt = VT_I4);

  // Creates a new integral type variant for the int type and assigns the value
  // to VARIANT.lVal (32 bit sized field).
  explicit ScopedVariant(int value);

  // Creates a new boolean (VT_BOOL) variant and assigns the value to
  // VARIANT.boolVal.
  explicit ScopedVariant(bool value);

  // Creates a new double-precision type variant.  |vt| must be either VT_R8
  // or VT_DATE.
  explicit ScopedVariant(double value, VARTYPE vt = VT_R8);

  // VT_DISPATCH
  explicit ScopedVariant(IDispatch* dispatch);

  // VT_UNKNOWN
  explicit ScopedVariant(IUnknown* unknown);

  // SAFEARRAY
  explicit ScopedVariant(SAFEARRAY* safearray);

  // Copies the variant.
  explicit ScopedVariant(const VARIANT& var);

  // Moves the wrapped variant into another ScopedVariant.
  ScopedVariant(ScopedVariant&& var);

  ~ScopedVariant();

  inline VARTYPE type() const { return var_.vt; }

  // Give ScopedVariant ownership over an already allocated VARIANT.
  void Reset(const VARIANT& var = kEmptyVariant);

  // Releases ownership of the VARIANT to the caller.
  VARIANT Release();

  // Swap two ScopedVariant's.
  void Swap(ScopedVariant& var);

  // Returns a copy of the variant.
  VARIANT Copy() const;

  // The return value is 0 if the variants are equal, 1 if this object is
  // greater than |other|, -1 if it is smaller.
  // Comparison with an array VARIANT is not supported.
  // 1. VT_NULL and VT_EMPTY is always considered less-than any other VARTYPE.
  // 2. If both VARIANTS have either VT_UNKNOWN or VT_DISPATCH even if the
  //    VARTYPEs do not match, the address of its IID_IUnknown is compared to
  //    guarantee a logical ordering even though it is not a meaningful order.
  //    e.g. (a.Compare(b) != b.Compare(a)) unless (a == b).
  // 3. If the VARTYPEs do not match, then the value of the VARTYPE is compared.
  // 4. Comparing VT_BSTR values is a lexicographical comparison of the contents
  //    of the BSTR, taking into account |ignore_case|.
  // 5. Otherwise returns the lexicographical comparison of the values held by
  //    the two VARIANTS that share the same VARTYPE.
  int Compare(const VARIANT& other, bool ignore_case = false) const;

  // Retrieves the pointer address.
  // Used to receive a VARIANT as an out argument (and take ownership).
  // The function DCHECKs on the current value being empty/null.
  // Usage: GetVariant(var.receive());
  VARIANT* Receive();

  void Set(const wchar_t* str);

  // Setters for simple types.
  void Set(int8_t i8);
  void Set(uint8_t ui8);
  void Set(int16_t i16);
  void Set(uint16_t ui16);
  void Set(int32_t i32);
  void Set(uint32_t ui32);
  void Set(int64_t i64);
  void Set(uint64_t ui64);
  void Set(float r32);
  void Set(double r64);
  void Set(bool b);

  // Creates a copy of |var| and assigns as this instance's value.
  // Note that this is different from the Reset() method that's used to
  // free the current value and assume ownership.
  void Set(const VARIANT& var);

  // COM object setters
  void Set(IDispatch* disp);
  void Set(IUnknown* unk);

  // SAFEARRAY support
  void Set(SAFEARRAY* array);

  // Special setter for DATE since DATE is a double and we already have
  // a setter for double.
  void SetDate(DATE date);

  // Allows const access to the contained variant without DCHECKs etc.
  // This support is necessary for the V_XYZ (e.g. V_BSTR) set of macros to
  // work properly but still doesn't allow modifications since we want control
  // over that.
  const VARIANT* ptr() const { return &var_; }

  // Moves the ScopedVariant to another instance.
  ScopedVariant& operator=(ScopedVariant&& var);

  // Like other scoped classes (e.g. scoped_refptr, ScopedBstr,
  // Microsoft::WRL::ComPtr) we support the assignment operator for the type we
  // wrap.
  ScopedVariant& operator=(const VARIANT& var);

  // A hack to pass a pointer to the variant where the accepting
  // function treats the variant as an input-only, read-only value
  // but the function prototype requires a non const variant pointer.
  // There's no DCHECK or anything here.  Callers must know what they're doing.
  VARIANT* AsInput() const {
    // The nature of this function is const, so we declare
    // it as such and cast away the constness here.
    return const_cast<VARIANT*>(&var_);
  }

  // Allows the ScopedVariant instance to be passed to functions either by value
  // or by const reference.
  operator const VARIANT&() const { return var_; }

  // Used as a debug check to see if we're leaking anything.
  static bool IsLeakableVarType(VARTYPE vt);

 protected:
  VARIANT var_;

 private:
  // Comparison operators for ScopedVariant are not supported at this point.
  // Use the Compare method instead.
  bool operator==(const ScopedVariant& var) const;
  bool operator!=(const ScopedVariant& var) const;
  BASE_DISALLOW_COPY_AND_ASSIGN(ScopedVariant);
};

}  // namespace win
}  // namespace base

#endif  // BASE_WIN_SCOPED_VARIANT_H_

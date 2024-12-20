// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/win/variant_vector.h"

#include <windows.foundation.h>
#include <wrl/client.h>

#include <cstddef>

#include "base/stl_util.h"
#include "base/test/gtest_util.h"
#include "base/win/dispatch_stub.h"
#include "base/win/scoped_safearray.h"
#include "testing/gtest/include/gtest/gtest.h"

using base::win::test::DispatchStub;

namespace base {
namespace win {

TEST(VariantVectorTest, InitiallyEmpty) {
  VariantVector vector;
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_TRUE(vector.Empty());
}

TEST(VariantVectorTest, MoveConstructor) {
  VariantVector vector1;
  Microsoft::WRL::ComPtr<IDispatch> dispatch =
      Microsoft::WRL::Make<DispatchStub>();
  vector1.Insert<VT_DISPATCH>(dispatch.Get());
  EXPECT_EQ(vector1.Type(), VT_DISPATCH);
  EXPECT_EQ(vector1.Size(), 1U);

  VariantVector vector2(std::move(vector1));
  EXPECT_EQ(vector1.Type(), VT_EMPTY);
  EXPECT_EQ(vector1.Size(), 0U);
  EXPECT_EQ(vector2.Type(), VT_DISPATCH);
  EXPECT_EQ(vector2.Size(), 1U);
  // |dispatch| should have been transferred to |vector2|.
  EXPECT_EQ(dispatch.Reset(), 1U);
}

TEST(VariantVectorTest, MoveAssignOperator) {
  VariantVector vector1, vector2;
  Microsoft::WRL::ComPtr<IDispatch> dispatch1 =
      Microsoft::WRL::Make<DispatchStub>();
  Microsoft::WRL::ComPtr<IDispatch> dispatch2 =
      Microsoft::WRL::Make<DispatchStub>();
  vector1.Insert<VT_DISPATCH>(dispatch1.Get());
  vector2.Insert<VT_UNKNOWN>(dispatch2.Get());
  EXPECT_EQ(vector1.Type(), VT_DISPATCH);
  EXPECT_EQ(vector1.Size(), 1U);
  EXPECT_EQ(vector2.Type(), VT_UNKNOWN);
  EXPECT_EQ(vector2.Size(), 1U);
  vector1 = std::move(vector2);
  EXPECT_EQ(vector1.Type(), VT_UNKNOWN);
  EXPECT_EQ(vector1.Size(), 1U);
  EXPECT_EQ(vector2.Type(), VT_EMPTY);
  EXPECT_EQ(vector2.Size(), 0U);
  // |dispatch1| should have been released during the move.
  EXPECT_EQ(dispatch1.Reset(), 0U);
  // |dispatch2| should have been transferred to |vector1|.
  EXPECT_EQ(dispatch2.Reset(), 1U);

  // Indirectly move |vector1| into itself.
  VariantVector& reference_to_vector1 = vector1;
  EXPECT_DCHECK_DEATH(vector1 = std::move(reference_to_vector1));
}

TEST(VariantVectorTest, Insert) {
  VariantVector vector;
  vector.Insert<VT_I4>(123);
  EXPECT_EQ(vector.Type(), VT_I4);
  // The first insert sets the type to VT_I4, and attempting to insert
  // unrelated types will silently fail in release builds but DCHECKs
  // in debug builds.
  EXPECT_DCHECK_DEATH(vector.Insert<VT_UI4>(1U));
  EXPECT_DCHECK_DEATH(vector.Insert<VT_R8>(100.0));
  EXPECT_EQ(vector.Type(), VT_I4);
  EXPECT_EQ(vector.Size(), 1U);
  EXPECT_FALSE(vector.Empty());
}

TEST(VariantVectorTest, InsertCanUpcastDispatchToUnknown) {
  Microsoft::WRL::ComPtr<IDispatch> dispatch =
      Microsoft::WRL::Make<DispatchStub>();
  Microsoft::WRL::ComPtr<IDispatch> unknown;
  dispatch.CopyTo(&unknown);

  VariantVector vector;
  vector.Insert<VT_UNKNOWN>(unknown.Get());
  vector.Insert<VT_UNKNOWN>(dispatch.Get());
  vector.Insert<VT_DISPATCH>(dispatch.Get());
  EXPECT_EQ(vector.Type(), VT_UNKNOWN);
  EXPECT_EQ(vector.Size(), 3U);
}

TEST(VariantVectorTest, InsertCannotDowncastUnknownToDispatch) {
  Microsoft::WRL::ComPtr<IDispatch> dispatch =
      Microsoft::WRL::Make<DispatchStub>();
  Microsoft::WRL::ComPtr<IDispatch> unknown;
  dispatch.CopyTo(&unknown);

  VariantVector vector;
  vector.Insert<VT_DISPATCH>(dispatch.Get());
  // The first insert sets the type to VT_DISPATCH, and attempting to
  // explicitly insert VT_UNKNOWN will silently fail in release builds
  // but DCHECKs in debug builds.
  EXPECT_DCHECK_DEATH(vector.Insert<VT_UNKNOWN>(unknown.Get()));
  EXPECT_DCHECK_DEATH(vector.Insert<VT_UNKNOWN>(dispatch.Get()));
  EXPECT_EQ(vector.Type(), VT_DISPATCH);
  EXPECT_EQ(vector.Size(), 1U);
}

TEST(VariantVectorTest, Reset) {
  VariantVector vector;
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  vector.Insert<VT_I4>(123);
  vector.Insert<VT_I4>(456);
  EXPECT_EQ(vector.Type(), VT_I4);
  EXPECT_EQ(vector.Size(), 2U);
  vector.Reset();
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
}

TEST(VariantVectorTest, ResetWithManagedContents) {
  VariantVector vector;
  // Test that managed contents are released when cleared.
  Microsoft::WRL::ComPtr<IUnknown> unknown1 =
      Microsoft::WRL::Make<DispatchStub>();
  Microsoft::WRL::ComPtr<IUnknown> unknown2;
  unknown1.CopyTo(&unknown2);
  vector.Insert<VT_UNKNOWN>(unknown1.Get());
  EXPECT_EQ(vector.Type(), VT_UNKNOWN);
  EXPECT_EQ(vector.Size(), 1U);
  // There are now 3 references to the value owned by |unknown1|.
  // Remove ownership from |unknown2| should reduce the count to 2.
  EXPECT_EQ(unknown2.Reset(), 2U);
  // Resetting the VariantVector will reduce the count to 1.
  vector.Reset();
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  // Then resetting |unknown1| should reduce to 0.
  EXPECT_EQ(unknown1.Reset(), 0U);
}

TEST(VariantVectorTest, ScopeWithManagedContents) {
  Microsoft::WRL::ComPtr<IUnknown> unknown1 =
      Microsoft::WRL::Make<DispatchStub>();
  {
    VariantVector vector;
    vector.Insert<VT_UNKNOWN>(unknown1.Get());
    EXPECT_EQ(vector.Type(), VT_UNKNOWN);
    EXPECT_EQ(vector.Size(), 1U);

    Microsoft::WRL::ComPtr<IUnknown> unknown2;
    unknown1.CopyTo(&unknown2);
    // There are now 3 references to the value owned by |unknown1|.
    // Remove ownership from |unknown2| should reduce the count to 2.
    EXPECT_EQ(unknown2.Reset(), 2U);
  }
  // The VariantVector going out of scope will reduce the count to 1.
  // Then resetting |unknown1| should reduce to 0.
  EXPECT_EQ(unknown1.Reset(), 0U);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantEmpty) {
  VariantVector vector;
  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(variant.type(), VT_EMPTY);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleBool) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set(true);
  vector.Insert<VT_BOOL>(true);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleI1) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set((int8_t)34);
  vector.Insert<VT_I1>(34);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleUI1) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set((uint8_t)35U);
  vector.Insert<VT_UI1>(35U);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleI2) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set((int16_t)8738);
  vector.Insert<VT_I2>(8738);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleUI2) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set((uint16_t)8739U);
  vector.Insert<VT_UI2>(8739U);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleI4) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set((int32_t)572662306);
  vector.Insert<VT_I4>(572662306);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleUI4) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set((uint32_t)572662307U);
  vector.Insert<VT_UI4>(572662307U);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleI8) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set((int64_t)2459565876494606882);
  vector.Insert<VT_I8>(2459565876494606882);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleUI8) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set((uint64_t)2459565876494606883U);
  vector.Insert<VT_UI8>(2459565876494606883U);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleR4) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set(3.14159f);
  vector.Insert<VT_R4>(3.14159f);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleR8) {
  VariantVector vector;
  ScopedVariant expected_variant;

  expected_variant.Set(6.28318);
  vector.Insert<VT_R8>(6.28318);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleDate) {
  VariantVector vector;
  ScopedVariant expected_variant;

  SYSTEMTIME sys_time;
  ::GetSystemTime(&sys_time);
  DATE date;
  ::SystemTimeToVariantTime(&sys_time, &date);
  expected_variant.SetDate(date);
  vector.Insert<VT_DATE>(date);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleBstr) {
  VariantVector vector;
  ScopedVariant expected_variant;

  wchar_t test_string[] = L"Test string for BSTRs.";
  expected_variant.Set(test_string);
  vector.Insert<VT_BSTR>(test_string);
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleUnknown) {
  VariantVector vector;
  ScopedVariant expected_variant;

  Microsoft::WRL::ComPtr<IUnknown> unknown =
      Microsoft::WRL::Make<DispatchStub>();
  expected_variant.Set(unknown.Get());
  vector.Insert<VT_UNKNOWN>(unknown.Get());
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantSingleDispatch) {
  VariantVector vector;
  ScopedVariant expected_variant;

  Microsoft::WRL::ComPtr<IDispatch> dispatch =
      Microsoft::WRL::Make<DispatchStub>();
  expected_variant.Set(dispatch.Get());
  vector.Insert<VT_DISPATCH>(dispatch.Get());
  EXPECT_EQ(vector.Type(), expected_variant.type());
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsScalarVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.Compare(expected_variant), 0);
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleBool) {
  constexpr VARTYPE kVariantType = VT_BOOL;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(true);
  vector.Insert<kVariantType>(false);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleI1) {
  constexpr VARTYPE kVariantType = VT_I1;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(34);
  vector.Insert<kVariantType>(52);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleUI1) {
  constexpr VARTYPE kVariantType = VT_UI1;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(34U);
  vector.Insert<kVariantType>(52U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleI2) {
  constexpr VARTYPE kVariantType = VT_I2;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(8738);
  vector.Insert<kVariantType>(8758);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleUI2) {
  constexpr VARTYPE kVariantType = VT_UI2;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(8739U);
  vector.Insert<kVariantType>(8759U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleI4) {
  constexpr VARTYPE kVariantType = VT_I4;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(572662306);
  vector.Insert<kVariantType>(572662307);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleUI4) {
  constexpr VARTYPE kVariantType = VT_UI4;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(578662306U);
  vector.Insert<kVariantType>(578662307U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleI8) {
  constexpr VARTYPE kVariantType = VT_I8;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(2459565876494606882);
  vector.Insert<kVariantType>(2459565876494606883);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleUI8) {
  constexpr VARTYPE kVariantType = VT_UI8;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(2459565876494606883U);
  vector.Insert<kVariantType>(2459565876494606884U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleR4) {
  constexpr VARTYPE kVariantType = VT_R4;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(3.14159f);
  vector.Insert<kVariantType>(6.28318f);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleR8) {
  constexpr VARTYPE kVariantType = VT_R8;
  VariantVector vector;
  ScopedVariant variant;

  vector.Insert<kVariantType>(6.28318);
  vector.Insert<kVariantType>(3.14159);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleDate) {
  constexpr VARTYPE kVariantType = VT_DATE;
  VariantVector vector;
  ScopedVariant variant;

  SYSTEMTIME sys_time;
  ::GetSystemTime(&sys_time);
  DATE date;
  ::SystemTimeToVariantTime(&sys_time, &date);

  vector.Insert<kVariantType>(date);
  vector.Insert<kVariantType>(date);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleBstr) {
  constexpr VARTYPE kVariantType = VT_BSTR;
  VariantVector vector;
  ScopedVariant variant;

  wchar_t some_text[] = L"some text";
  wchar_t more_text[] = L"more text";
  vector.Insert<kVariantType>(some_text);
  vector.Insert<kVariantType>(more_text);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleUnknown) {
  constexpr VARTYPE kVariantType = VT_UNKNOWN;
  VariantVector vector;
  ScopedVariant variant;

  Microsoft::WRL::ComPtr<IUnknown> unknown1 =
      Microsoft::WRL::Make<DispatchStub>();
  Microsoft::WRL::ComPtr<IUnknown> unknown2 =
      Microsoft::WRL::Make<DispatchStub>();

  vector.Insert<kVariantType>(unknown1.Get());
  vector.Insert<kVariantType>(unknown2.Get());
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsScalarVariantMultipleDispatch) {
  constexpr VARTYPE kVariantType = VT_DISPATCH;
  VariantVector vector;
  ScopedVariant variant;

  Microsoft::WRL::ComPtr<IDispatch> dispatch1 =
      Microsoft::WRL::Make<DispatchStub>();
  Microsoft::WRL::ComPtr<IDispatch> dispatch2 =
      Microsoft::WRL::Make<DispatchStub>();

  vector.Insert<kVariantType>(dispatch1.Get());
  vector.Insert<kVariantType>(dispatch2.Get());
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);
  EXPECT_DCHECK_DEATH(vector.ReleaseAsScalarVariant());
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantEmpty) {
  VariantVector vector;
  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(variant.type(), VT_EMPTY);
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleBool) {
  constexpr VARTYPE kVariantType = VT_BOOL;
  VariantVector vector;

  vector.Insert<kVariantType>(true);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), VARIANT_TRUE);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleI1) {
  constexpr VARTYPE kVariantType = VT_I1;
  VariantVector vector;

  vector.Insert<kVariantType>(34);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), 34);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleUI1) {
  constexpr VARTYPE kVariantType = VT_UI1;
  VariantVector vector;

  vector.Insert<kVariantType>(34U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), 34U);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleI2) {
  constexpr VARTYPE kVariantType = VT_I2;
  VariantVector vector;

  vector.Insert<kVariantType>(8738);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), 8738);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleUI2) {
  constexpr VARTYPE kVariantType = VT_UI2;
  VariantVector vector;

  vector.Insert<kVariantType>(8739U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), 8739U);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleI4) {
  constexpr VARTYPE kVariantType = VT_I4;
  VariantVector vector;

  vector.Insert<kVariantType>(572662306);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), 572662306);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleUI4) {
  constexpr VARTYPE kVariantType = VT_UI4;
  VariantVector vector;

  vector.Insert<kVariantType>(578662306U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), 578662306U);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleI8) {
  constexpr VARTYPE kVariantType = VT_I8;
  VariantVector vector;

  vector.Insert<kVariantType>(2459565876494606882);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), 2459565876494606882);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleUI8) {
  constexpr VARTYPE kVariantType = VT_UI8;
  VariantVector vector;

  vector.Insert<kVariantType>(2459565876494606883U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), 2459565876494606883U);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleR4) {
  constexpr VARTYPE kVariantType = VT_R4;
  VariantVector vector;

  vector.Insert<kVariantType>(3.14159f);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), 3.14159f);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleR8) {
  constexpr VARTYPE kVariantType = VT_R8;
  VariantVector vector;

  vector.Insert<kVariantType>(6.28318);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), 6.28318);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleDate) {
  constexpr VARTYPE kVariantType = VT_DATE;
  VariantVector vector;

  SYSTEMTIME sys_time;
  ::GetSystemTime(&sys_time);
  DATE date;
  ::SystemTimeToVariantTime(&sys_time, &date);

  vector.Insert<kVariantType>(date);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), date);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleBstr) {
  constexpr VARTYPE kVariantType = VT_BSTR;
  VariantVector vector;

  wchar_t some_text[] = L"some text";
  vector.Insert<kVariantType>(some_text);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_STREQ(lock_scope->at(0), some_text);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleUnknown) {
  constexpr VARTYPE kVariantType = VT_UNKNOWN;
  VariantVector vector;

  Microsoft::WRL::ComPtr<IUnknown> unknown =
      Microsoft::WRL::Make<DispatchStub>();

  vector.Insert<kVariantType>(unknown.Get());
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), unknown.Get());
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantSingleDispatch) {
  constexpr VARTYPE kVariantType = VT_DISPATCH;
  VariantVector vector;

  Microsoft::WRL::ComPtr<IDispatch> dispatch =
      Microsoft::WRL::Make<DispatchStub>();

  vector.Insert<kVariantType>(dispatch.Get());
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 1U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 1U);
  EXPECT_EQ(lock_scope->at(0), dispatch.Get());
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleBool) {
  constexpr VARTYPE kVariantType = VT_BOOL;
  VariantVector vector;

  vector.Insert<kVariantType>(true);
  vector.Insert<kVariantType>(false);
  vector.Insert<kVariantType>(true);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 3U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 3U);
  EXPECT_EQ(lock_scope->at(0), VARIANT_TRUE);
  EXPECT_EQ(lock_scope->at(1), VARIANT_FALSE);
  EXPECT_EQ(lock_scope->at(2), VARIANT_TRUE);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleI1) {
  constexpr VARTYPE kVariantType = VT_I1;
  VariantVector vector;

  vector.Insert<kVariantType>(34);
  vector.Insert<kVariantType>(52);
  vector.Insert<kVariantType>(12);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 3U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 3U);
  EXPECT_EQ(lock_scope->at(0), 34);
  EXPECT_EQ(lock_scope->at(1), 52);
  EXPECT_EQ(lock_scope->at(2), 12);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleUI1) {
  constexpr VARTYPE kVariantType = VT_UI1;
  VariantVector vector;

  vector.Insert<kVariantType>(34U);
  vector.Insert<kVariantType>(52U);
  vector.Insert<kVariantType>(12U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 3U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 3U);
  EXPECT_EQ(lock_scope->at(0), 34U);
  EXPECT_EQ(lock_scope->at(1), 52U);
  EXPECT_EQ(lock_scope->at(2), 12U);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleI2) {
  constexpr VARTYPE kVariantType = VT_I2;
  VariantVector vector;

  vector.Insert<kVariantType>(8738);
  vector.Insert<kVariantType>(8758);
  vector.Insert<kVariantType>(42);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 3U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 3U);
  EXPECT_EQ(lock_scope->at(0), 8738);
  EXPECT_EQ(lock_scope->at(1), 8758);
  EXPECT_EQ(lock_scope->at(2), 42);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleUI2) {
  constexpr VARTYPE kVariantType = VT_UI2;
  VariantVector vector;

  vector.Insert<kVariantType>(8739U);
  vector.Insert<kVariantType>(8759U);
  vector.Insert<kVariantType>(42U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 3U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 3U);
  EXPECT_EQ(lock_scope->at(0), 8739U);
  EXPECT_EQ(lock_scope->at(1), 8759U);
  EXPECT_EQ(lock_scope->at(2), 42U);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleI4) {
  constexpr VARTYPE kVariantType = VT_I4;
  VariantVector vector;

  vector.Insert<kVariantType>(572662306);
  vector.Insert<kVariantType>(572662307);
  vector.Insert<kVariantType>(572662308);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 3U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 3U);
  EXPECT_EQ(lock_scope->at(0), 572662306);
  EXPECT_EQ(lock_scope->at(1), 572662307);
  EXPECT_EQ(lock_scope->at(2), 572662308);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleUI4) {
  constexpr VARTYPE kVariantType = VT_UI4;
  VariantVector vector;

  vector.Insert<kVariantType>(578662306U);
  vector.Insert<kVariantType>(578662307U);
  vector.Insert<kVariantType>(578662308U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 3U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 3U);
  EXPECT_EQ(lock_scope->at(0), 578662306U);
  EXPECT_EQ(lock_scope->at(1), 578662307U);
  EXPECT_EQ(lock_scope->at(2), 578662308U);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleI8) {
  constexpr VARTYPE kVariantType = VT_I8;
  VariantVector vector;

  vector.Insert<kVariantType>(2459565876494606882);
  vector.Insert<kVariantType>(2459565876494606883);
  vector.Insert<kVariantType>(2459565876494606884);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 3U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 3U);
  EXPECT_EQ(lock_scope->at(0), 2459565876494606882);
  EXPECT_EQ(lock_scope->at(1), 2459565876494606883);
  EXPECT_EQ(lock_scope->at(2), 2459565876494606884);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleUI8) {
  constexpr VARTYPE kVariantType = VT_UI8;
  VariantVector vector;

  vector.Insert<kVariantType>(2459565876494606883U);
  vector.Insert<kVariantType>(2459565876494606884U);
  vector.Insert<kVariantType>(2459565876494606885U);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 3U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 3U);
  EXPECT_EQ(lock_scope->at(0), 2459565876494606883U);
  EXPECT_EQ(lock_scope->at(1), 2459565876494606884U);
  EXPECT_EQ(lock_scope->at(2), 2459565876494606885U);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleR4) {
  constexpr VARTYPE kVariantType = VT_R4;
  VariantVector vector;

  vector.Insert<kVariantType>(3.14159f);
  vector.Insert<kVariantType>(6.28318f);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 2U);
  EXPECT_EQ(lock_scope->at(0), 3.14159f);
  EXPECT_EQ(lock_scope->at(1), 6.28318f);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleR8) {
  constexpr VARTYPE kVariantType = VT_R8;
  VariantVector vector;

  vector.Insert<kVariantType>(6.28318);
  vector.Insert<kVariantType>(3.14159);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 2U);
  EXPECT_EQ(lock_scope->at(0), 6.28318);
  EXPECT_EQ(lock_scope->at(1), 3.14159);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleDate) {
  constexpr VARTYPE kVariantType = VT_DATE;
  VariantVector vector;
  SYSTEMTIME sys_time;
  ::GetSystemTime(&sys_time);
  DATE date;
  ::SystemTimeToVariantTime(&sys_time, &date);

  vector.Insert<kVariantType>(date);
  vector.Insert<kVariantType>(date);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 2U);
  EXPECT_EQ(lock_scope->at(0), date);
  EXPECT_EQ(lock_scope->at(1), date);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleBstr) {
  constexpr VARTYPE kVariantType = VT_BSTR;
  VariantVector vector;
  wchar_t some_text[] = L"some text";
  wchar_t more_text[] = L"more text";
  vector.Insert<kVariantType>(some_text);
  vector.Insert<kVariantType>(more_text);
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 2U);
  EXPECT_STREQ(lock_scope->at(0), some_text);
  EXPECT_STREQ(lock_scope->at(1), more_text);
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleUnknown) {
  constexpr VARTYPE kVariantType = VT_UNKNOWN;
  VariantVector vector;

  Microsoft::WRL::ComPtr<IUnknown> unknown1 =
      Microsoft::WRL::Make<DispatchStub>();
  Microsoft::WRL::ComPtr<IUnknown> unknown2 =
      Microsoft::WRL::Make<DispatchStub>();

  vector.Insert<kVariantType>(unknown1.Get());
  vector.Insert<kVariantType>(unknown2.Get());
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 2U);
  EXPECT_EQ(lock_scope->at(0), unknown1.Get());
  EXPECT_EQ(lock_scope->at(1), unknown2.Get());
  safearray.Release();
}

TEST(VariantVectorTest, ReleaseAsSafearrayVariantMultipleDispatch) {
  constexpr VARTYPE kVariantType = VT_DISPATCH;
  VariantVector vector;

  Microsoft::WRL::ComPtr<IDispatch> dispatch1 =
      Microsoft::WRL::Make<DispatchStub>();
  Microsoft::WRL::ComPtr<IDispatch> dispatch2 =
      Microsoft::WRL::Make<DispatchStub>();

  vector.Insert<kVariantType>(dispatch1.Get());
  vector.Insert<kVariantType>(dispatch2.Get());
  EXPECT_EQ(vector.Type(), kVariantType);
  EXPECT_EQ(vector.Size(), 2U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);
  EXPECT_EQ(variant.type(), VT_ARRAY | kVariantType);

  ScopedSafearray safearray(V_ARRAY(variant.ptr()));
  base::Optional<ScopedSafearray::LockScope<kVariantType>> lock_scope =
      safearray.CreateLockScope<kVariantType>();
  ASSERT_TRUE(lock_scope.has_value());
  ASSERT_EQ(lock_scope->size(), 2U);
  EXPECT_EQ(lock_scope->at(0), dispatch1.Get());
  EXPECT_EQ(lock_scope->at(1), dispatch2.Get());
  safearray.Release();
}

TEST(VariantVectorTest, CompareVariant) {
  VariantVector vector;
  ScopedVariant variant;
  EXPECT_EQ(vector.Compare(variant), 0);

  vector.Insert<VT_I4>(123);
  EXPECT_EQ(vector.Type(), VT_I4);
  EXPECT_EQ(vector.Size(), 1U);

  variant.Set(123);
  EXPECT_EQ(vector.Compare(variant), 0);
  variant.Set(4);
  EXPECT_EQ(vector.Compare(variant), 1);
  variant.Set(567);
  EXPECT_EQ(vector.Compare(variant), -1);
  // Because the types do not match and VT_I4 is less-than VT_R8,
  // |vector| compares as less-than |variant|, even though the value
  // in |vector| is greater.
  variant.Set(1.0);
  EXPECT_EQ(variant.type(), VT_R8);
  EXPECT_LT(vector.Type(), variant.type());
  EXPECT_EQ(vector.Compare(variant), -1);

  vector.Insert<VT_I4>(456);
  EXPECT_EQ(vector.Size(), 2U);

  // The first element of |vector| is equal to |variant|, but |vector|
  // has more than one element so it is greater-than |variant|.
  variant.Set(123);
  EXPECT_EQ(vector.Compare(variant), 1);
  // The first element of |vector| is greater-than |variant|.
  variant.Set(5);
  EXPECT_EQ(vector.Compare(variant), 1);
  // The first element of |vector| is less-than |variant|.
  variant.Set(1000);
  EXPECT_EQ(vector.Compare(variant), -1);
}

TEST(VariantVectorTest, CompareSafearray) {
  VariantVector vector;
  vector.Insert<VT_I4>(123);
  vector.Insert<VT_I4>(456);
  EXPECT_EQ(vector.Type(), VT_I4);
  EXPECT_EQ(vector.Size(), 2U);

  ScopedVariant variant(vector.ReleaseAsSafearrayVariant());
  EXPECT_EQ(variant.type(), VT_ARRAY | VT_I4);
  EXPECT_EQ(vector.Type(), VT_EMPTY);
  EXPECT_EQ(vector.Size(), 0U);

  // Because |vector| is now empty, it will compare as less-than the array.
  EXPECT_EQ(vector.Compare(V_ARRAY(variant.ptr())), -1);
  EXPECT_EQ(vector.Compare(variant), -1);

  vector.Insert<VT_I4>(123);
  EXPECT_EQ(vector.Type(), VT_I4);
  EXPECT_EQ(vector.Size(), 1U);
  // |vector| has fewer elements than |variant|.
  EXPECT_EQ(vector.Compare(V_ARRAY(variant.ptr())), -1);
  EXPECT_EQ(vector.Compare(variant), -1);

  vector.Insert<VT_I4>(456);
  EXPECT_EQ(vector.Type(), VT_I4);
  EXPECT_EQ(vector.Size(), 2U);
  // |vector| is now equal to |variant|.
  EXPECT_EQ(vector.Compare(V_ARRAY(variant.ptr())), 0);
  EXPECT_EQ(vector.Compare(variant), 0);

  vector.Insert<VT_I4>(789);
  EXPECT_EQ(vector.Type(), VT_I4);
  EXPECT_EQ(vector.Size(), 3U);
  // |vector| contains |variant| but has more elements so
  // |vector| is now greater-than |variant|.
  EXPECT_EQ(vector.Compare(V_ARRAY(variant.ptr())), 1);
  EXPECT_EQ(vector.Compare(variant), 1);

  vector.Reset();
  vector.Insert<VT_I4>(456);
  EXPECT_EQ(vector.Type(), VT_I4);
  EXPECT_EQ(vector.Size(), 1U);
  // |vector| has fewer elements than |variant|, but the first element in
  // |vector| compares as greater-than the first element in |variant|.
  EXPECT_EQ(vector.Compare(V_ARRAY(variant.ptr())), 1);
  EXPECT_EQ(vector.Compare(variant), 1);

  vector.Reset();
  vector.Insert<VT_R8>(0.0);
  vector.Insert<VT_R8>(0.0);
  EXPECT_EQ(vector.Type(), VT_R8);
  // Because the types do not match and VT_R8 is greater-than VT_I4,
  // |vector| compares as greater-than |variant|, even though the values
  // in |vector| are less-than the values in |variant|.
  EXPECT_GT(VT_R8, VT_I4);
  EXPECT_EQ(vector.Compare(V_ARRAY(variant.ptr())), 1);
  EXPECT_EQ(vector.Compare(variant), 1);

  vector.Reset();
  vector.Insert<VT_I2>(1000);
  vector.Insert<VT_I2>(1000);
  EXPECT_EQ(vector.Type(), VT_I2);
  // Because the types do not match and VT_I2 is less-than VT_I4,
  // |vector| compares as less-than |variant|, even though the values
  // in |vector| are greater-than the values in |variant|.
  EXPECT_LT(VT_I2, VT_I4);
  EXPECT_EQ(vector.Compare(V_ARRAY(variant.ptr())), -1);
  EXPECT_EQ(vector.Compare(variant), -1);
}

TEST(VariantVectorTest, CompareVariantVector) {
  VariantVector vector1, vector2;
  EXPECT_EQ(vector1.Compare(vector2), 0);
  EXPECT_EQ(vector1, vector2);

  vector1.Insert<VT_I4>(1);
  EXPECT_EQ(vector1.Compare(vector2), 1);
  EXPECT_EQ(vector2.Compare(vector1), -1);
  EXPECT_NE(vector1, vector2);

  vector2.Insert<VT_I4>(1);
  EXPECT_EQ(vector1.Compare(vector2), 0);
  EXPECT_EQ(vector2.Compare(vector1), 0);
  EXPECT_EQ(vector1, vector2);

  vector1.Insert<VT_I4>(1);
  EXPECT_EQ(vector1.Compare(vector2), 1);
  EXPECT_EQ(vector2.Compare(vector1), -1);
  EXPECT_NE(vector1, vector2);

  vector2.Insert<VT_I4>(2);
  EXPECT_EQ(vector1.Compare(vector2), -1);
  EXPECT_EQ(vector2.Compare(vector1), 1);
  EXPECT_NE(vector1, vector2);

  vector1.Reset();
  vector1.Insert<VT_I4>(10);
  vector2.Reset();
  vector2.Insert<VT_R8>(5.0);
  // Because the types do not match and VT_I4 is less-than VT_R8,
  // |vector1| compares as less-than |vector2|, even though the value
  // in |vector1| is greater.
  EXPECT_LT(vector1.Type(), vector2.Type());
  EXPECT_EQ(vector1.Compare(vector2), -1);
  EXPECT_EQ(vector2.Compare(vector1), 1);
  EXPECT_NE(vector1, vector2);
}

}  // namespace win
}  // namespace base

// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/win/scoped_safearray.h"
#include "base/test/gtest_util.h"

#include <array>
#include <cstddef>
#include <vector>

#include "gtest/gtest.h"

namespace base {
namespace win {

namespace {

static constexpr std::array<int, 5> kInputValues = {0, 1, 2, 1, 0};

static void PopulateScopedSafearrayOfInts(ScopedSafearray& scoped_safe_array) {
  // TODO(crbug.com/1082005): Create a safer alternative to SAFEARRAY methods.
  scoped_safe_array.Reset(SafeArrayCreateVector(
      /*vartype=*/VT_I4, /*lower_bound=*/2,
      /*element_count=*/kInputValues.size()));
  ASSERT_NE(scoped_safe_array.Get(), nullptr);
  ASSERT_EQ(SafeArrayGetDim(scoped_safe_array.Get()), 1U);
  ASSERT_EQ(scoped_safe_array.GetCount(), kInputValues.size());

  int* int_array;
  ASSERT_HRESULT_SUCCEEDED(SafeArrayAccessData(
      scoped_safe_array.Get(), reinterpret_cast<void**>(&int_array)));
  for (size_t i = 0; i < kInputValues.size(); ++i)
    int_array[i] = kInputValues[i];
  ASSERT_HRESULT_SUCCEEDED(SafeArrayUnaccessData(scoped_safe_array.Get()));
}

}  // namespace

TEST(ScopedSafearrayTest, ScopedSafearrayMethods) {
  ScopedSafearray empty_safe_array;
  EXPECT_EQ(empty_safe_array.Get(), nullptr);
  EXPECT_EQ(empty_safe_array.Release(), nullptr);
  EXPECT_NE(empty_safe_array.Receive(), nullptr);

  SAFEARRAY* safe_array = SafeArrayCreateVector(
      VT_R8 /* element type */, 0 /* lower bound */, 4 /* elements */);
  ScopedSafearray scoped_safe_array(safe_array);
  EXPECT_EQ(scoped_safe_array.Get(), safe_array);
  EXPECT_EQ(scoped_safe_array.Release(), safe_array);
  EXPECT_NE(scoped_safe_array.Receive(), nullptr);

  // The Release() call should have set the internal pointer to nullptr
  EXPECT_EQ(scoped_safe_array.Get(), nullptr);

  scoped_safe_array.Reset(safe_array);
  EXPECT_EQ(scoped_safe_array.Get(), safe_array);

  ScopedSafearray moved_safe_array(std::move(scoped_safe_array));
  EXPECT_EQ(moved_safe_array.Get(), safe_array);
  EXPECT_EQ(moved_safe_array.Release(), safe_array);
  EXPECT_NE(moved_safe_array.Receive(), nullptr);

  // std::move should have cleared the values of scoped_safe_array
  EXPECT_EQ(scoped_safe_array.Get(), nullptr);
  EXPECT_EQ(scoped_safe_array.Release(), nullptr);
  EXPECT_NE(scoped_safe_array.Receive(), nullptr);

  scoped_safe_array.Reset(safe_array);
  EXPECT_EQ(scoped_safe_array.Get(), safe_array);

  ScopedSafearray assigment_moved_safe_array = std::move(scoped_safe_array);
  EXPECT_EQ(assigment_moved_safe_array.Get(), safe_array);
  EXPECT_EQ(assigment_moved_safe_array.Release(), safe_array);
  EXPECT_NE(assigment_moved_safe_array.Receive(), nullptr);

  // The move-assign operator= should have cleared the values of
  // scoped_safe_array
  EXPECT_EQ(scoped_safe_array.Get(), nullptr);
  EXPECT_EQ(scoped_safe_array.Release(), nullptr);
  EXPECT_NE(scoped_safe_array.Receive(), nullptr);

  // Calling Receive() will free the existing reference
  ScopedSafearray safe_array_received(SafeArrayCreateVector(
      VT_R8 /* element type */, 0 /* lower bound */, 4 /* elements */));
  EXPECT_NE(safe_array_received.Receive(), nullptr);
  EXPECT_EQ(safe_array_received.Get(), nullptr);
}

TEST(ScopedSafearrayTest, ScopedSafearrayMoveConstructor) {
  ScopedSafearray first;
  PopulateScopedSafearrayOfInts(first);
  EXPECT_NE(first.Get(), nullptr);
  EXPECT_EQ(first.GetCount(), kInputValues.size());

  SAFEARRAY* safearray = first.Get();
  ScopedSafearray second(std::move(first));
  EXPECT_EQ(first.Get(), nullptr);
  EXPECT_EQ(second.Get(), safearray);
}

TEST(ScopedSafearrayTest, ScopedSafearrayMoveAssignOperator) {
  ScopedSafearray first, second;
  PopulateScopedSafearrayOfInts(first);
  EXPECT_NE(first.Get(), nullptr);
  EXPECT_EQ(first.GetCount(), kInputValues.size());

  SAFEARRAY* safearray = first.Get();
  second = std::move(first);
  EXPECT_EQ(first.Get(), nullptr);
  EXPECT_EQ(second.Get(), safearray);

  // Indirectly move |second| into itself.
  ScopedSafearray& reference_to_second = second;
  second = std::move(reference_to_second);
  EXPECT_EQ(second.GetCount(), kInputValues.size());
  EXPECT_EQ(second.Get(), safearray);
}

TEST(ScopedSafearrayTest, ScopedSafearrayCast) {
  SAFEARRAY* safe_array = SafeArrayCreateVector(
      VT_R8 /* element type */, 1 /* lower bound */, 5 /* elements */);
  ScopedSafearray scoped_safe_array(safe_array);
  EXPECT_EQ(SafeArrayGetDim(scoped_safe_array.Get()), 1U);

  LONG lower_bound;
  EXPECT_HRESULT_SUCCEEDED(
      SafeArrayGetLBound(scoped_safe_array.Get(), 1, &lower_bound));
  EXPECT_EQ(lower_bound, 1);

  LONG upper_bound;
  EXPECT_HRESULT_SUCCEEDED(
      SafeArrayGetUBound(scoped_safe_array.Get(), 1, &upper_bound));
  EXPECT_EQ(upper_bound, 5);

  VARTYPE variable_type;
  EXPECT_HRESULT_SUCCEEDED(
      SafeArrayGetVartype(scoped_safe_array.Get(), &variable_type));
  EXPECT_EQ(variable_type, VT_R8);
}

TEST(ScopedSafearrayTest, InitiallyEmpty) {
  ScopedSafearray empty_safe_array;
  EXPECT_EQ(empty_safe_array.Get(), nullptr);
  EXPECT_DCHECK_DEATH(empty_safe_array.GetCount());
}

TEST(ScopedSafearrayTest, ScopedSafearrayGetCount) {
  // TODO(crbug.com/1082005): Create a safer alternative to SAFEARRAY methods.
  ScopedSafearray scoped_safe_array(SafeArrayCreateVector(
      /*vartype=*/VT_I4, /*lower_bound=*/2, /*element_count=*/5));
  ASSERT_NE(scoped_safe_array.Get(), nullptr);
  EXPECT_EQ(SafeArrayGetDim(scoped_safe_array.Get()), 1U);

  LONG lower_bound;
  EXPECT_HRESULT_SUCCEEDED(
      SafeArrayGetLBound(scoped_safe_array.Get(), 1, &lower_bound));
  EXPECT_EQ(lower_bound, 2);

  LONG upper_bound;
  EXPECT_HRESULT_SUCCEEDED(
      SafeArrayGetUBound(scoped_safe_array.Get(), 1, &upper_bound));
  EXPECT_EQ(upper_bound, 6);

  EXPECT_EQ(scoped_safe_array.GetCount(), 5U);
}

TEST(ScopedSafearrayTest, ScopedSafearrayInitialLockScope) {
  ScopedSafearray scoped_safe_array;
  std::optional<ScopedSafearray::LockScope<VT_I4>> lock_scope =
      scoped_safe_array.CreateLockScope<VT_I4>();
  EXPECT_FALSE(lock_scope.has_value());
}

TEST(ScopedSafearrayTest, ScopedSafearrayLockScopeMoveConstructor) {
  ScopedSafearray scoped_safe_array;
  PopulateScopedSafearrayOfInts(scoped_safe_array);

  std::optional<ScopedSafearray::LockScope<VT_I4>> first =
      scoped_safe_array.CreateLockScope<VT_I4>();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->Type(), VT_I4);
  EXPECT_EQ(first->size(), kInputValues.size());

  ScopedSafearray::LockScope<VT_I4> second(std::move(*first));
  EXPECT_EQ(first->Type(), VT_EMPTY);
  EXPECT_EQ(first->size(), 0U);
  EXPECT_EQ(second.Type(), VT_I4);
  EXPECT_EQ(second.size(), kInputValues.size());
}

TEST(ScopedSafearrayTest, ScopedSafearrayLockScopeMoveAssignOperator) {
  ScopedSafearray scoped_safe_array;
  PopulateScopedSafearrayOfInts(scoped_safe_array);

  std::optional<ScopedSafearray::LockScope<VT_I4>> first =
      scoped_safe_array.CreateLockScope<VT_I4>();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->Type(), VT_I4);
  EXPECT_EQ(first->size(), kInputValues.size());

  ScopedSafearray::LockScope<VT_I4> second;
  second = std::move(*first);
  EXPECT_EQ(first->Type(), VT_EMPTY);
  EXPECT_EQ(first->size(), 0U);
  EXPECT_EQ(second.Type(), VT_I4);
  EXPECT_EQ(second.size(), kInputValues.size());

  // Indirectly move |second| into itself.
  ScopedSafearray::LockScope<VT_I4>& reference_to_second = second;
  EXPECT_DCHECK_DEATH(second = std::move(reference_to_second));
}

TEST(ScopedSafearrayTest, ScopedSafearrayLockScopeTypeMismatch) {
  ScopedSafearray scoped_safe_array;
  PopulateScopedSafearrayOfInts(scoped_safe_array);

  {
    std::optional<ScopedSafearray::LockScope<VT_BSTR>> invalid_lock_scope =
        scoped_safe_array.CreateLockScope<VT_BSTR>();
    EXPECT_FALSE(invalid_lock_scope.has_value());
  }

  {
    std::optional<ScopedSafearray::LockScope<VT_UI4>> invalid_lock_scope =
        scoped_safe_array.CreateLockScope<VT_UI4>();
    EXPECT_FALSE(invalid_lock_scope.has_value());
  }
}

TEST(ScopedSafearrayTest, ScopedSafearrayLockScopeRandomAccess) {
  ScopedSafearray scoped_safe_array;
  PopulateScopedSafearrayOfInts(scoped_safe_array);

  std::optional<ScopedSafearray::LockScope<VT_I4>> lock_scope =
      scoped_safe_array.CreateLockScope<VT_I4>();
  ASSERT_TRUE(lock_scope.has_value());
  EXPECT_EQ(lock_scope->Type(), VT_I4);
  EXPECT_EQ(lock_scope->size(), kInputValues.size());
  for (size_t i = 0; i < kInputValues.size(); ++i) {
    EXPECT_EQ(lock_scope->at(i), kInputValues[i]);
    EXPECT_EQ((*lock_scope)[i], kInputValues[i]);
  }
}

TEST(ScopedSafearrayTest, ScopedSafearrayLockScopeIterator) {
  ScopedSafearray scoped_safe_array;
  PopulateScopedSafearrayOfInts(scoped_safe_array);

  std::optional<ScopedSafearray::LockScope<VT_I4>> lock_scope =
      scoped_safe_array.CreateLockScope<VT_I4>();

  std::vector<int> unpacked_vector(lock_scope->begin(), lock_scope->end());
  ASSERT_EQ(unpacked_vector.size(), kInputValues.size());
  for (size_t i = 0; i < kInputValues.size(); ++i)
    EXPECT_EQ(unpacked_vector[i], kInputValues[i]);
}

}  // namespace win
}  // namespace base

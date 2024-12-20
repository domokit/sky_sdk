// Copyright (c) 2010 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/win/scoped_bstr.h"

#include <cstddef>

#include "gtest/gtest.h"

namespace base {
namespace win {

namespace {

constexpr wchar_t kTestString1[] = L"123";
constexpr wchar_t kTestString2[] = L"456789";
constexpr size_t test1_len = std::size(kTestString1) - 1;
constexpr size_t test2_len = std::size(kTestString2) - 1;

}  // namespace

TEST(ScopedBstrTest, Empty) {
  ScopedBstr b;
  EXPECT_EQ(nullptr, b.Get());
  EXPECT_EQ(0u, b.Length());
  EXPECT_EQ(0u, b.ByteLength());
  b.Reset(nullptr);
  EXPECT_EQ(nullptr, b.Get());
  EXPECT_EQ(nullptr, b.Release());
  ScopedBstr b2;
  b.Swap(b2);
  EXPECT_EQ(nullptr, b.Get());
}

TEST(ScopedBstrTest, Basic) {
  ScopedBstr b(kTestString1);
  EXPECT_EQ(test1_len, b.Length());
  EXPECT_EQ(test1_len * sizeof(kTestString1[0]), b.ByteLength());
}

namespace {

void CreateTestString1(BSTR* ret) {
  *ret = SysAllocString(kTestString1);
}

}  // namespace

TEST(ScopedBstrTest, Swap) {
  ScopedBstr b1(kTestString1);
  ScopedBstr b2;
  b1.Swap(b2);
  EXPECT_EQ(test1_len, b2.Length());
  EXPECT_EQ(0u, b1.Length());
  EXPECT_STREQ(kTestString1, b2.Get());

  BSTR tmp = b2.Release();
  EXPECT_NE(nullptr, tmp);
  EXPECT_STREQ(kTestString1, tmp);
  EXPECT_EQ(nullptr, b2.Get());
  SysFreeString(tmp);
}

TEST(ScopedBstrTest, OutParam) {
  ScopedBstr b;
  CreateTestString1(b.Receive());
  EXPECT_STREQ(kTestString1, b.Get());
}

TEST(ScopedBstrTest, AllocateBytesAndSetByteLen) {
  constexpr size_t num_bytes = 100;
  ScopedBstr b;
  EXPECT_NE(nullptr, b.AllocateBytes(num_bytes));
  EXPECT_EQ(num_bytes, b.ByteLength());
  EXPECT_EQ(num_bytes / sizeof(kTestString1[0]), b.Length());

  lstrcpy(b.Get(), kTestString1);
  EXPECT_EQ(test1_len, static_cast<size_t>(lstrlen(b.Get())));
  EXPECT_EQ(num_bytes / sizeof(kTestString1[0]), b.Length());

  b.SetByteLen(lstrlen(b.Get()) * sizeof(kTestString2[0]));
  EXPECT_EQ(b.Length(), static_cast<size_t>(lstrlen(b.Get())));
}

TEST(ScopedBstrTest, AllocateAndSetByteLen) {
  ScopedBstr b;
  EXPECT_NE(nullptr, b.Allocate(kTestString2));
  EXPECT_EQ(test2_len, b.Length());

  b.SetByteLen((test2_len - 1) * sizeof(kTestString2[0]));
  EXPECT_EQ(test2_len - 1, b.Length());
}

}  // namespace win
}  // namespace base

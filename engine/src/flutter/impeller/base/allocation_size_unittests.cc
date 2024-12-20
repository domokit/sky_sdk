// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"
#include "impeller/base/allocation_size.h"

namespace impeller::testing {

TEST(AllocationSizeTest, CanCreateTypedAllocations) {
  auto bytes = Bytes{1024};
  ASSERT_EQ(bytes.GetByteSize(), 1024u);

  auto kilobytes = KiloBytes{5};
  ASSERT_EQ(kilobytes.GetByteSize(), 5u * 1e3);

  auto megabytes = MegaBytes{5};
  ASSERT_EQ(megabytes.GetByteSize(), 5u * 1e6);

  auto gigabytes = GigaBytes{5};
  ASSERT_EQ(gigabytes.GetByteSize(), 5u * 1e9);

  auto kibibytes = KibiBytes{1};
  ASSERT_EQ(kibibytes.GetByteSize(), 1024u);

  auto mebibytes = MebiBytes{1};
  ASSERT_EQ(mebibytes.GetByteSize(), 1048576u);

  auto gigibytes = GibiBytes{1};
  ASSERT_EQ(gigibytes.GetByteSize(), 1073741824u);
}

TEST(AllocationSizeTest, CanCreateTypedAllocationsWithLiterals) {
  using namespace allocation_size_literals;
  ASSERT_EQ((1024_bytes).GetByteSize(), 1024u);
  ASSERT_EQ((5_kb).GetByteSize(), 5u * 1e3);
  ASSERT_EQ((5_mb).GetByteSize(), 5u * 1e6);
  ASSERT_EQ((5_gb).GetByteSize(), 5u * 1e9);
  ASSERT_EQ((1_kib).GetByteSize(), 1024u);
  ASSERT_EQ((1_mib).GetByteSize(), 1048576u);
  ASSERT_EQ((1_gib).GetByteSize(), 1073741824u);
}

TEST(AllocationSizeTest, CanConvert) {
  using namespace allocation_size_literals;
  ASSERT_EQ((5_gb).ConvertTo<MegaBytes>().GetSize(), 5000u);
}

TEST(AllocationSizeTest, ConversionsAreNonTruncating) {
  using namespace allocation_size_literals;
  ASSERT_DOUBLE_EQ((1500_bytes).ConvertTo<KiloBytes>().GetSize(), 1.5);
  ASSERT_EQ((1500_bytes).ConvertTo<KiloBytes>().GetByteSize(), 1500u);
}

TEST(AllocationSizeTest, CanGetFloatValues) {
  using namespace allocation_size_literals;
  ASSERT_DOUBLE_EQ((1500_bytes).ConvertTo<KiloBytes>().GetSize(), 1.5);
}

TEST(AllocationSizeTest, RelationalOperatorsAreFunctional) {
  using namespace allocation_size_literals;

  auto a = 1500_bytes;
  auto b = 2500_bytes;
  auto c = 0_bytes;

  ASSERT_TRUE(a != b);
  ASSERT_FALSE(a == b);
  ASSERT_TRUE(b > a);
  ASSERT_TRUE(b >= a);
  ASSERT_TRUE(a < b);
  ASSERT_TRUE(a <= b);
  ASSERT_TRUE(a);
  ASSERT_FALSE(c);
}

TEST(AllocationSizeTest, CanCast) {
  using namespace allocation_size_literals;
  {
    auto a = KiloBytes{1500_bytes};
    ASSERT_DOUBLE_EQ(a.GetSize(), 1.5);
  }
  {
    auto a = KiloBytes{Bytes{1500}};
    ASSERT_DOUBLE_EQ(a.GetSize(), 1.5);
  }

  ASSERT_DOUBLE_EQ(MebiBytes{Bytes{4194304}}.GetSize(), 4);
}

TEST(AllocationSizeTest, CanPerformSimpleArithmetic) {
  using namespace allocation_size_literals;
  {
    auto a = 100_bytes;
    auto b = 200_bytes;
    ASSERT_EQ((a + b).GetByteSize(), 300u);
  }
  {
    auto a = 100_bytes;
    a += 200_bytes;
    ASSERT_EQ(a.GetByteSize(), 300u);
  }
  {
    auto a = 100_bytes;
    a -= 50_bytes;
    ASSERT_EQ(a.GetByteSize(), 50u);
  }
}

TEST(AllocationSizeTest, CanConstructWithArith) {
  {
    Bytes a(1u);
    ASSERT_EQ(a.GetByteSize(), 1u);
  }
  {
    Bytes a(1.5);
    ASSERT_EQ(a.GetByteSize(), 2u);
  }
  {
    Bytes a(1.5f);
    ASSERT_EQ(a.GetByteSize(), 2u);
  }
}

}  // namespace impeller::testing

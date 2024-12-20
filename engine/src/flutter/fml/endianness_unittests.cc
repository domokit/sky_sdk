// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/endianness.h"

#include "flutter/testing/testing.h"

namespace fml {
namespace testing {

TEST(EndiannessTest, ByteSwap) {
  ASSERT_EQ(ByteSwap<int16_t>(0x1122), 0x2211);
  ASSERT_EQ(ByteSwap<int32_t>(0x11223344), 0x44332211);
  ASSERT_EQ(ByteSwap<uint64_t>(0x1122334455667788), 0x8877665544332211);
}

TEST(EndiannessTest, BigEndianToArch) {
#if FML_ARCH_CPU_LITTLE_ENDIAN
  uint32_t expected = 0x44332211;
#else
  uint32_t expected = 0x11223344;
#endif
  ASSERT_EQ(BigEndianToArch(0x11223344u), expected);
}

TEST(EndiannessTest, LittleEndianToArch) {
#if FML_ARCH_CPU_LITTLE_ENDIAN
  uint32_t expected = 0x11223344;
#else
  uint32_t expected = 0x44332211;
#endif
  ASSERT_EQ(LittleEndianToArch(0x11223344u), expected);
}

}  // namespace testing
}  // namespace fml

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/string_conversion.h"

#include "gtest/gtest.h"

namespace fml {
namespace testing {

TEST(StringConversion, Utf16ToUtf16Empty) {
  EXPECT_EQ(Utf8ToUtf16(""), u"");
}

TEST(StringConversion, Utf8ToUtf16Ascii) {
  EXPECT_EQ(Utf8ToUtf16("abc123"), u"abc123");
}

TEST(StringConversion, Utf8ToUtf16Unicode) {
  EXPECT_EQ(Utf8ToUtf16("\xe2\x98\x83"), u"\x2603");
}

TEST(StringConversion, Utf16ToUtf8Empty) {
  EXPECT_EQ(Utf16ToUtf8(u""), "");
}

TEST(StringConversion, Utf16ToUtf8Ascii) {
  EXPECT_EQ(Utf16ToUtf8(u"abc123"), "abc123");
}

TEST(StringConversion, Utf16ToUtf8Unicode) {
  EXPECT_EQ(Utf16ToUtf8(u"\x2603"), "\xe2\x98\x83");
}

}  // namespace testing
}  // namespace fml

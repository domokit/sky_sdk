// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/platform/win/wstring_conversion.h"

#include "gtest/gtest.h"

namespace fml {
namespace testing {

TEST(StringConversion, Utf8ToWideStringEmpty) {
  EXPECT_EQ(Utf8ToWideString(""), L"");
}

TEST(StringConversion, Utf8ToWideStringAscii) {
  EXPECT_EQ(Utf8ToWideString("abc123"), L"abc123");
}

TEST(StringConversion, Utf8ToWideStringUnicode) {
  EXPECT_EQ(Utf8ToWideString("\xe2\x98\x83"), L"\x2603");
}

TEST(StringConversion, WideStringToUtf8Empty) {
  EXPECT_EQ(WideStringToUtf8(L""), "");
}

TEST(StringConversion, WideStringToUtf8Ascii) {
  EXPECT_EQ(WideStringToUtf8(L"abc123"), "abc123");
}

TEST(StringConversion, WideStringToUtf8Unicode) {
  EXPECT_EQ(WideStringToUtf8(L"\x2603"), "\xe2\x98\x83");
}

TEST(StringConversion, WideStringToUtf16Empty) {
  EXPECT_EQ(WideStringToUtf16(L""), u"");
}

TEST(StringConversion, WideStringToUtf16Ascii) {
  EXPECT_EQ(WideStringToUtf16(L"abc123"), u"abc123");
}

TEST(StringConversion, WideStringToUtf16Unicode) {
  EXPECT_EQ(WideStringToUtf16(L"\xe2\x98\x83"), u"\xe2\x98\x83");
}

TEST(StringConversion, Utf16ToWideStringEmpty) {
  EXPECT_EQ(Utf16ToWideString(u""), L"");
}

TEST(StringConversion, Utf16ToWideStringAscii) {
  EXPECT_EQ(Utf16ToWideString(u"abc123"), L"abc123");
}

TEST(StringConversion, Utf16ToWideStringUtf8Unicode) {
  EXPECT_EQ(Utf16ToWideString(u"\xe2\x98\x83"), L"\xe2\x98\x83");
}

}  // namespace testing
}  // namespace fml

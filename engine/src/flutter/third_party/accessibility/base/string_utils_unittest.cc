// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "string_utils.h"

#include <cerrno>
#include <cstddef>
#include <string>

#include "base/logging.h"
#include "gtest/gtest.h"

namespace base {

TEST(StringUtilsTest, StringPrintfEmpty) {
  EXPECT_EQ("", base::StringPrintf("%s", ""));
}

TEST(StringUtilsTest, StringPrintfMisc) {
  EXPECT_EQ("123hello w", StringPrintf("%3d%2s %1c", 123, "hello", 'w'));
}
// Test that StringPrintf and StringAppendV do not change errno.
TEST(StringUtilsTest, StringPrintfErrno) {
  errno = 1;
  EXPECT_EQ("", StringPrintf("%s", ""));
  EXPECT_EQ(1, errno);
}

TEST(StringUtilsTest, canASCIIToUTF16) {
  std::string ascii = "abcdefg";
  EXPECT_EQ(ASCIIToUTF16(ascii).compare(u"abcdefg"), 0);
}

TEST(StringUtilsTest, canUTF8ToUTF16) {
  std::string utf8 = "äåè";
  EXPECT_EQ(UTF8ToUTF16(utf8).compare(u"äåè"), 0);
}

TEST(StringUtilsTest, canUTF16ToUTF8) {
  std::u16string utf16 = u"äåè";
  EXPECT_EQ(UTF16ToUTF8(utf16).compare("äåè"), 0);
}

TEST(StringUtilsTest, canNumberToString16) {
  EXPECT_EQ(NumberToString16(1.123f), std::u16string(u"1.123"));
}

TEST(StringUtilsTest, numberToStringSimplifiesOutput) {
  EXPECT_STREQ(NumberToString(0.0).c_str(), "0");
  EXPECT_STREQ(NumberToString(0.0f).c_str(), "0");
  EXPECT_STREQ(NumberToString(1.123).c_str(), "1.123");
  EXPECT_STREQ(NumberToString(1.123f).c_str(), "1.123");
  EXPECT_STREQ(NumberToString(-1.123).c_str(), "-1.123");
  EXPECT_STREQ(NumberToString(-1.123f).c_str(), "-1.123");
  EXPECT_STREQ(NumberToString(1.00001).c_str(), "1.00001");
  EXPECT_STREQ(NumberToString(1.00001f).c_str(), "1.00001");
  EXPECT_STREQ(NumberToString(1000.000001).c_str(), "1000.000001");
  EXPECT_STREQ(NumberToString(10.00001f).c_str(), "10.00001");
  EXPECT_STREQ(NumberToString(1.0 + 1e-8).c_str(), "1.00000001");
  EXPECT_STREQ(NumberToString(1.0f + 1e-8f).c_str(), "1");
  EXPECT_STREQ(NumberToString(1e-6).c_str(), "0.000001");
  EXPECT_STREQ(NumberToString(1e-6f).c_str(), "0.000001");
  EXPECT_STREQ(NumberToString(1e-8).c_str(), "1e-8");
  EXPECT_STREQ(NumberToString(1e-8f).c_str(), "1e-8");
  EXPECT_STREQ(NumberToString(100.0).c_str(), "100");
  EXPECT_STREQ(NumberToString(100.0f).c_str(), "100");
  EXPECT_STREQ(NumberToString(-1.0 - 1e-7).c_str(), "-1.0000001");
  EXPECT_STREQ(NumberToString(-1.0f - 1e-7f).c_str(), "-1.0000001");
  EXPECT_STREQ(NumberToString(0.00000012345678).c_str(), "1.2345678e-7");
  // Difference in output is due to differences in double and float precision.
  EXPECT_STREQ(NumberToString(0.00000012345678f).c_str(), "1.2345679e-7");
  EXPECT_STREQ(NumberToString(-0.00000012345678).c_str(), "-1.2345678e-7");
  // Difference in output is due to differences in double and float precision.
  EXPECT_STREQ(NumberToString(-0.00000012345678f).c_str(), "-1.2345679e-7");
  EXPECT_STREQ(NumberToString(static_cast<unsigned int>(11)).c_str(), "11");
  EXPECT_STREQ(NumberToString(static_cast<int32_t>(-23)).c_str(), "-23");
}

}  // namespace base

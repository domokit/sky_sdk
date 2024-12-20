// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/base64.h"

#include "fml/logging.h"
#include "gtest/gtest.h"

#include <string>

namespace flutter {
namespace testing {

TEST(Base64, EncodeStrings) {
  auto test = [](const std::string& input, const std::string& output) {
    char buffer[256];
    size_t len = Base64::Encode(input.c_str(), input.length(), &buffer);
    FML_CHECK(len <= 256);
    ASSERT_EQ(len, Base64::EncodedSize(input.length()));
    std::string actual(buffer, len);
    ASSERT_STREQ(actual.c_str(), output.c_str());
  };
  // Some arbitrary strings
  test("apple", "YXBwbGU=");
  test("BANANA", "QkFOQU5B");
  test("Cherry Pie", "Q2hlcnJ5IFBpZQ==");
  test("fLoCcInAuCiNiHiLiPiLiFiCaTiOn",
       "ZkxvQ2NJbkF1Q2lOaUhpTGlQaUxpRmlDYVRpT24=");
  test("", "");
}

TEST(Base64, EncodeBytes) {
  auto test = [](const uint8_t input[], size_t num, const std::string& output) {
    char buffer[512];
    size_t len = Base64::Encode(input, num, &buffer);
    FML_CHECK(len <= 512);
    ASSERT_EQ(len, Base64::EncodedSize(num));
    std::string actual(buffer, len);
    ASSERT_STREQ(actual.c_str(), output.c_str());
  };
  // Some arbitrary raw bytes
  uint8_t e[] = {0x02, 0x71, 0x82, 0x81, 0x82, 0x84, 0x59};
  test(e, sizeof(e), "AnGCgYKEWQ==");

  uint8_t pi[] = {0x03, 0x24, 0x3F, 0x6A, 0x88, 0x85};
  test(pi, sizeof(pi), "AyQ/aoiF");

  uint8_t bytes[256];
  for (int i = 0; i < 256; i++) {
    bytes[i] = i;
  }
  test(bytes, sizeof(bytes),
       "AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8gIS"
       "IjJCUmJygpKissLS4vMDEyMzQ1Njc4OTo7PD0+P0BBQkNERUZHSElKS0xNTk9QUVJTVFV"
       "WV1hZWltcXV5fYGFiY2RlZmdoaWprbG1ub3BxcnN0dXZ3eHl6e3x9fn+AgYKDhIWGh4iJ"
       "iouMjY6PkJGSk5SVlpeYmZqbnJ2en6ChoqOkpaanqKmqq6ytrq+wsbKztLW2t7i5uru8v"
       "b6/wMHCw8TFxsfIycrLzM3Oz9DR0tPU1dbX2Nna29zd3t/g4eLj5OXm5+jp6uvs7e7v8P"
       "Hy8/T19vf4+fr7/P3+/w==");
}

TEST(Base64, DecodeStringsSuccess) {
  auto test = [](const std::string& input, const std::string& output) {
    char buffer[256];
    size_t len = 0;
    auto err = Base64::Decode(input.c_str(), input.length(), &buffer, &len);
    ASSERT_EQ(err, Base64::Error::kNone);
    FML_CHECK(len <= 256);
    std::string actual(buffer, len);
    ASSERT_STREQ(actual.c_str(), output.c_str());
  };
  // Some arbitrary strings
  test("ZGF0ZQ==", "date");
  test("RWdncGxhbnQ=", "Eggplant");
  test("RmlzaCAmIENoaXBz", "Fish & Chips");
  test("U3VQZVJjQWxJZlJhR2lMaVN0SWNFeFBpQWxJZE9jSW9Vcw==",
       "SuPeRcAlIfRaGiLiStIcExPiAlIdOcIoUs");

  // Spaces are ignored
  test("Y X Bwb GU=", "apple");
}

TEST(Base64, DecodeStringsHasErrors) {
  auto test = [](const std::string& input, Base64::Error expectedError) {
    char buffer[256];
    size_t len = 0;
    auto err = Base64::Decode(input.c_str(), input.length(), &buffer, &len);
    ASSERT_EQ(err, expectedError) << input;
  };

  test("Nuts&Bolts", Base64::Error::kBadChar);
  test("Error!", Base64::Error::kBadChar);
  test(":", Base64::Error::kBadChar);

  test("RmlzaCAmIENoaXBz=", Base64::Error::kBadPadding);
  // Some cases of bad padding may be ignored due to an internal optimization
  // test("ZGF0ZQ=", Base64::Error::kBadPadding);
  // test("RWdncGxhbnQ", Base64::Error::kBadPadding);
}

TEST(Base64, DecodeBytes) {
  auto test = [](const std::string& input, const uint8_t output[], size_t num) {
    char buffer[256];
    size_t len = 0;
    auto err = Base64::Decode(input.c_str(), input.length(), &buffer, &len);
    ASSERT_EQ(err, Base64::Error::kNone);
    FML_CHECK(len <= 256);
    ASSERT_EQ(num, len) << input;
    for (int i = 0; i < static_cast<int>(len); i++) {
      ASSERT_EQ(uint8_t(buffer[i]), output[i]) << input << i;
    }
  };
  // Some arbitrary raw bytes, same as the byte output above
  uint8_t e[] = {0x02, 0x71, 0x82, 0x81, 0x82, 0x84, 0x59};
  test("AnGCgYKEWQ==", e, sizeof(e));

  uint8_t pi[] = {0x03, 0x24, 0x3F, 0x6A, 0x88, 0x85};
  test("AyQ/aoiF", pi, sizeof(pi));
}

}  // namespace testing
}  // namespace flutter
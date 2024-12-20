// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_message_codec.h"

#include <map>
#include <vector>

#include "flutter/shell/platform/common/client_wrapper/testing/test_codec_extensions.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace flutter {

namespace {

class MockStandardCodecSerializer : public StandardCodecSerializer {
 public:
  MOCK_METHOD(void,
              WriteValue,
              (const EncodableValue& value, ByteStreamWriter* stream),
              (const, override));
  MOCK_METHOD(EncodableValue,
              ReadValueOfType,
              (uint8_t type, ByteStreamReader* stream),
              (const, override));
};
}  // namespace

// Validates round-trip encoding and decoding of |value|, and checks that the
// encoded value matches |expected_encoding|.
//
// If testing with CustomEncodableValues, |serializer| must be provided to
// handle the encoding/decoding, and |custom_comparator| must be provided to
// validate equality since CustomEncodableValue doesn't define a useful ==.
static void CheckEncodeDecode(
    const EncodableValue& value,
    const std::vector<uint8_t>& expected_encoding,
    const StandardCodecSerializer* serializer = nullptr,
    const std::function<bool(const EncodableValue& a, const EncodableValue& b)>&
        custom_comparator = nullptr) {
  const StandardMessageCodec& codec =
      StandardMessageCodec::GetInstance(serializer);
  auto encoded = codec.EncodeMessage(value);
  ASSERT_TRUE(encoded);
  EXPECT_EQ(*encoded, expected_encoding);

  auto decoded = codec.DecodeMessage(*encoded);
  if (custom_comparator) {
    EXPECT_TRUE(custom_comparator(value, *decoded));
  } else {
    EXPECT_EQ(value, *decoded);
  }
}

// Validates round-trip encoding and decoding of |value|, and checks that the
// encoded value has the given prefix and length.
//
// This should be used only for Map, where asserting the order of the elements
// in a test is undesirable.
static void CheckEncodeDecodeWithEncodePrefix(
    const EncodableValue& value,
    const std::vector<uint8_t>& expected_encoding_prefix,
    size_t expected_encoding_length) {
  EXPECT_TRUE(std::holds_alternative<EncodableMap>(value));
  const StandardMessageCodec& codec = StandardMessageCodec::GetInstance();
  auto encoded = codec.EncodeMessage(value);
  ASSERT_TRUE(encoded);

  EXPECT_EQ(encoded->size(), expected_encoding_length);
  ASSERT_GT(encoded->size(), expected_encoding_prefix.size());
  EXPECT_TRUE(std::equal(
      encoded->begin(), encoded->begin() + expected_encoding_prefix.size(),
      expected_encoding_prefix.begin(), expected_encoding_prefix.end()));

  auto decoded = codec.DecodeMessage(*encoded);

  EXPECT_EQ(value, *decoded);
}

TEST(StandardMessageCodec, GetInstanceCachesInstance) {
  const StandardMessageCodec& codec_a =
      StandardMessageCodec::GetInstance(nullptr);
  const StandardMessageCodec& codec_b =
      StandardMessageCodec::GetInstance(nullptr);
  EXPECT_EQ(&codec_a, &codec_b);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeNull) {
  std::vector<uint8_t> bytes = {0x00};
  CheckEncodeDecode(EncodableValue(), bytes);
}

TEST(StandardMessageCodec, CanDecodeEmptyBytesAsNullWithoutCallingSerializer) {
  std::vector<uint8_t> bytes = {};
  const MockStandardCodecSerializer serializer;
  const StandardMessageCodec& codec =
      StandardMessageCodec::GetInstance(&serializer);

  auto decoded = codec.DecodeMessage(bytes);

  EXPECT_EQ(EncodableValue(), *decoded);
  EXPECT_CALL(serializer, ReadValueOfType(::testing::_, ::testing::_)).Times(0);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeTrue) {
  std::vector<uint8_t> bytes = {0x01};
  CheckEncodeDecode(EncodableValue(true), bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeFalse) {
  std::vector<uint8_t> bytes = {0x02};
  CheckEncodeDecode(EncodableValue(false), bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeInt32) {
  std::vector<uint8_t> bytes = {0x03, 0x78, 0x56, 0x34, 0x12};
  CheckEncodeDecode(EncodableValue(0x12345678), bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeInt64) {
  std::vector<uint8_t> bytes = {0x04, 0xef, 0xcd, 0xab, 0x90,
                                0x78, 0x56, 0x34, 0x12};
  CheckEncodeDecode(EncodableValue(INT64_C(0x1234567890abcdef)), bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeDouble) {
  std::vector<uint8_t> bytes = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x18, 0x2d, 0x44, 0x54, 0xfb, 0x21, 0x09, 0x40};
  CheckEncodeDecode(EncodableValue(3.14159265358979311599796346854), bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeString) {
  std::vector<uint8_t> bytes = {0x07, 0x0b, 0x68, 0x65, 0x6c, 0x6c, 0x6f,
                                0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64};
  CheckEncodeDecode(EncodableValue(u8"hello world"), bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeStringWithNonAsciiCodePoint) {
  std::vector<uint8_t> bytes = {0x07, 0x05, 0x68, 0xe2, 0x98, 0xba, 0x77};
  CheckEncodeDecode(EncodableValue(u8"h\u263Aw"), bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeStringWithNonBMPCodePoint) {
  std::vector<uint8_t> bytes = {0x07, 0x06, 0x68, 0xf0, 0x9f, 0x98, 0x82, 0x77};
  CheckEncodeDecode(EncodableValue(u8"h\U0001F602w"), bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeEmptyString) {
  std::vector<uint8_t> bytes = {0x07, 0x00};
  CheckEncodeDecode(EncodableValue(u8""), bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeList) {
  std::vector<uint8_t> bytes = {
      0x0c, 0x05, 0x00, 0x07, 0x05, 0x68, 0x65, 0x6c, 0x6c, 0x6f, 0x06,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x85, 0xeb, 0x51, 0xb8, 0x1e,
      0x09, 0x40, 0x03, 0x2f, 0x00, 0x00, 0x00, 0x0c, 0x02, 0x03, 0x2a,
      0x00, 0x00, 0x00, 0x07, 0x06, 0x6e, 0x65, 0x73, 0x74, 0x65, 0x64,
  };
  EncodableValue value(EncodableList{
      EncodableValue(),
      EncodableValue("hello"),
      EncodableValue(3.14),
      EncodableValue(47),
      EncodableValue(EncodableList{
          EncodableValue(42),
          EncodableValue("nested"),
      }),
  });
  CheckEncodeDecode(value, bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeEmptyList) {
  std::vector<uint8_t> bytes = {0x0c, 0x00};
  CheckEncodeDecode(EncodableValue(EncodableList{}), bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeMap) {
  std::vector<uint8_t> bytes_prefix = {0x0d, 0x04};
  EncodableValue value(EncodableMap{
      {EncodableValue("a"), EncodableValue(3.14)},
      {EncodableValue("b"), EncodableValue(47)},
      {EncodableValue(), EncodableValue()},
      {EncodableValue(3.14), EncodableValue(EncodableList{
                                 EncodableValue("nested"),
                             })},
  });
  CheckEncodeDecodeWithEncodePrefix(value, bytes_prefix, 48);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeByteArray) {
  std::vector<uint8_t> bytes = {0x08, 0x04, 0xba, 0x5e, 0xba, 0x11};
  EncodableValue value(std::vector<uint8_t>{0xba, 0x5e, 0xba, 0x11});
  CheckEncodeDecode(value, bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeInt32Array) {
  std::vector<uint8_t> bytes = {0x09, 0x03, 0x00, 0x00, 0x78, 0x56, 0x34, 0x12,
                                0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00};
  EncodableValue value(std::vector<int32_t>{0x12345678, -1, 0});
  CheckEncodeDecode(value, bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeInt64Array) {
  std::vector<uint8_t> bytes = {0x0a, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0xef, 0xcd, 0xab, 0x90, 0x78, 0x56, 0x34, 0x12,
                                0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  EncodableValue value(std::vector<int64_t>{0x1234567890abcdef, -1});
  CheckEncodeDecode(value, bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeFloat32Array) {
  std::vector<uint8_t> bytes = {0x0e, 0x02, 0x00, 0x00, 0xd8, 0x0f,
                                0x49, 0x40, 0x00, 0x00, 0x7a, 0x44};
  EncodableValue value(std::vector<float>{3.1415920257568359375f, 1000.0f});
  CheckEncodeDecode(value, bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeFloat64Array) {
  std::vector<uint8_t> bytes = {0x0b, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                0x18, 0x2d, 0x44, 0x54, 0xfb, 0x21, 0x09, 0x40,
                                0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x8f, 0x40};
  EncodableValue value(
      std::vector<double>{3.14159265358979311599796346854, 1000.0});
  CheckEncodeDecode(value, bytes);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeSimpleCustomType) {
  std::vector<uint8_t> bytes = {0x80, 0x09, 0x00, 0x00, 0x00,
                                0x10, 0x00, 0x00, 0x00};
  auto point_comparator = [](const EncodableValue& a, const EncodableValue& b) {
    const Point& a_point =
        std::any_cast<Point>(std::get<CustomEncodableValue>(a));
    const Point& b_point =
        std::any_cast<Point>(std::get<CustomEncodableValue>(b));
    return a_point == b_point;
  };
  CheckEncodeDecode(CustomEncodableValue(Point(9, 16)), bytes,
                    &PointExtensionSerializer::GetInstance(), point_comparator);
}

TEST(StandardMessageCodec, CanEncodeAndDecodeVariableLengthCustomType) {
  std::vector<uint8_t> bytes = {
      0x81,                                      // custom type
      0x06, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,  // data
      0x07, 0x04,                                // string type and length
      0x74, 0x65, 0x73, 0x74                     // string characters
  };
  auto some_data_comparator = [](const EncodableValue& a,
                                 const EncodableValue& b) {
    const SomeData& data_a =
        std::any_cast<SomeData>(std::get<CustomEncodableValue>(a));
    const SomeData& data_b =
        std::any_cast<SomeData>(std::get<CustomEncodableValue>(b));
    return data_a.data() == data_b.data() && data_a.label() == data_b.label();
  };
  CheckEncodeDecode(CustomEncodableValue(
                        SomeData("test", {0x00, 0x01, 0x02, 0x03, 0x04, 0x05})),
                    bytes, &SomeDataExtensionSerializer::GetInstance(),
                    some_data_comparator);
}

}  // namespace flutter

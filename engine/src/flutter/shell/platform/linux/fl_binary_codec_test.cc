// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_codec.h"
#include "flutter/shell/platform/linux/testing/fl_test.h"
#include "gtest/gtest.h"

// Encodes a message using a FlBinaryCodec. Return a hex string with the encoded
// binary output.
static gchar* encode_message(FlValue* value) {
  g_autoptr(FlBinaryCodec) codec = fl_binary_codec_new();
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_message_codec_encode_message(FL_MESSAGE_CODEC(codec), value, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  return bytes_to_hex_string(message);
}

// Encodes a message using a FlBinaryCodec. Expect the given error.
static void encode_message_error(FlValue* value, GQuark domain, int code) {
  g_autoptr(FlBinaryCodec) codec = fl_binary_codec_new();
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_message_codec_encode_message(FL_MESSAGE_CODEC(codec), value, &error);
  EXPECT_EQ(message, nullptr);
  EXPECT_TRUE(g_error_matches(error, domain, code));
}

// Decodes a message using a FlBinaryCodec. The binary data is given in the form
// of a hex string.
static FlValue* decode_message(const char* hex_string) {
  g_autoptr(FlBinaryCodec) codec = fl_binary_codec_new();
  g_autoptr(GBytes) data = hex_string_to_bytes(hex_string);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlValue) value =
      fl_message_codec_decode_message(FL_MESSAGE_CODEC(codec), data, &error);
  EXPECT_EQ(error, nullptr);
  EXPECT_NE(value, nullptr);
  return fl_value_ref(value);
}

TEST(FlBinaryCodecTest, EncodeData) {
  uint8_t data[] = {0x00, 0x01, 0x02, 0xFD, 0xFE, 0xFF};
  g_autoptr(FlValue) value = fl_value_new_uint8_list(data, 6);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "000102fdfeff");
}

TEST(FlBinaryCodecTest, EncodeEmpty) {
  g_autoptr(FlValue) value = fl_value_new_uint8_list(nullptr, 0);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "");
}

TEST(FlBinaryCodecTest, EncodeNullptr) {
  encode_message_error(nullptr, FL_MESSAGE_CODEC_ERROR,
                       FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE);
}

TEST(FlBinaryCodecTest, EncodeUnknownType) {
  g_autoptr(FlValue) value = fl_value_new_null();
  encode_message_error(value, FL_MESSAGE_CODEC_ERROR,
                       FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE);
}

TEST(FlBinaryCodecTest, DecodeData) {
  g_autoptr(FlValue) value = decode_message("000102fdfeff");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_UINT8_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(6));
  EXPECT_EQ(fl_value_get_uint8_list(value)[0], 0x00);
  EXPECT_EQ(fl_value_get_uint8_list(value)[1], 0x01);
  EXPECT_EQ(fl_value_get_uint8_list(value)[2], 0x02);
  EXPECT_EQ(fl_value_get_uint8_list(value)[3], 0xFD);
  EXPECT_EQ(fl_value_get_uint8_list(value)[4], 0xFE);
  EXPECT_EQ(fl_value_get_uint8_list(value)[5], 0xFF);
}

TEST(FlBinaryCodecTest, DecodeEmpty) {
  g_autoptr(FlValue) value = decode_message("");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_UINT8_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlBinaryCodecTest, EncodeDecode) {
  g_autoptr(FlBinaryCodec) codec = fl_binary_codec_new();

  uint8_t data[] = {0x00, 0x01, 0x02, 0xFD, 0xFE, 0xFF};
  g_autoptr(FlValue) input = fl_value_new_uint8_list(data, 6);

  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_message_codec_encode_message(FL_MESSAGE_CODEC(codec), input, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  g_autoptr(FlValue) output =
      fl_message_codec_decode_message(FL_MESSAGE_CODEC(codec), message, &error);
  EXPECT_EQ(error, nullptr);
  EXPECT_NE(output, nullptr);

  ASSERT_TRUE(fl_value_equal(input, output));
}

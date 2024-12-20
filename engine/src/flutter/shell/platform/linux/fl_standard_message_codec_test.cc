// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_message_codec.h"
#include "flutter/shell/platform/linux/testing/fl_test.h"
#include "gtest/gtest.h"

// NOTE(robert-ancell) These test cases assumes a little-endian architecture.
// These tests will need to be updated if tested on a big endian architecture.

// Encodes a message using the supplied codec. Return a hex string with
// the encoded binary output.
static gchar* encode_message_with_codec(FlValue* value, FlMessageCodec* codec) {
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_message_codec_encode_message(codec, value, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  return bytes_to_hex_string(message);
}

// Encodes a message using a FlStandardMessageCodec. Return a hex string with
// the encoded binary output.
static gchar* encode_message(FlValue* value) {
  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  return encode_message_with_codec(value, FL_MESSAGE_CODEC(codec));
}

// Decodes a message using the supplied codec. The binary data is given in
// the form of a hex string.
static FlValue* decode_message_with_codec(const char* hex_string,
                                          FlMessageCodec* codec) {
  g_autoptr(GBytes) data = hex_string_to_bytes(hex_string);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlValue) value =
      fl_message_codec_decode_message(codec, data, &error);
  EXPECT_EQ(error, nullptr);
  EXPECT_NE(value, nullptr);
  return fl_value_ref(value);
}

// Decodes a message using a FlStandardMessageCodec. The binary data is given in
// the form of a hex string.
static FlValue* decode_message(const char* hex_string) {
  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  return decode_message_with_codec(hex_string, FL_MESSAGE_CODEC(codec));
}

// Decodes a message using a FlStandardMessageCodec. The binary data is given in
// the form of a hex string. Expect the given error.
static void decode_error_value(const char* hex_string,
                               GQuark domain,
                               gint code) {
  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  g_autoptr(GBytes) data = hex_string_to_bytes(hex_string);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlValue) value =
      fl_message_codec_decode_message(FL_MESSAGE_CODEC(codec), data, &error);
  EXPECT_TRUE(value == nullptr);
  EXPECT_TRUE(g_error_matches(error, domain, code));
}

TEST(FlStandardMessageCodecTest, EncodeNullptr) {
  g_autofree gchar* hex_string = encode_message(nullptr);
  EXPECT_STREQ(hex_string, "00");
}

TEST(FlStandardMessageCodecTest, EncodeNull) {
  g_autoptr(FlValue) value = fl_value_new_null();
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "00");
}

TEST(FlStandardMessageCodecTest, DecodeNull) {
  // Regression test for https://github.com/flutter/flutter/issues/128704.

  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();
  g_autoptr(GBytes) data = g_bytes_new(nullptr, 0);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlValue) value =
      fl_message_codec_decode_message(FL_MESSAGE_CODEC(codec), data, &error);

  EXPECT_FALSE(value == nullptr);
  EXPECT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_NULL);
}

static gchar* encode_bool(gboolean value) {
  g_autoptr(FlValue) v = fl_value_new_bool(value);
  return encode_message(v);
}

TEST(FlStandardMessageCodecTest, EncodeBoolFalse) {
  g_autofree gchar* hex_string = encode_bool(FALSE);
  EXPECT_STREQ(hex_string, "02");
}

TEST(FlStandardMessageCodecTest, EncodeBoolTrue) {
  g_autofree gchar* hex_string = encode_bool(TRUE);
  EXPECT_STREQ(hex_string, "01");
}

static gchar* encode_int(int64_t value) {
  g_autoptr(FlValue) v = fl_value_new_int(value);
  return encode_message(v);
}

TEST(FlStandardMessageCodecTest, EncodeIntZero) {
  g_autofree gchar* hex_string = encode_int(0);
  EXPECT_STREQ(hex_string, "0300000000");
}

TEST(FlStandardMessageCodecTest, EncodeIntOne) {
  g_autofree gchar* hex_string = encode_int(1);
  EXPECT_STREQ(hex_string, "0301000000");
}

TEST(FlStandardMessageCodecTest, EncodeInt32) {
  g_autofree gchar* hex_string = encode_int(0x01234567);
  EXPECT_STREQ(hex_string, "0367452301");
}

TEST(FlStandardMessageCodecTest, EncodeInt32Min) {
  g_autofree gchar* hex_string = encode_int(G_MININT32);
  EXPECT_STREQ(hex_string, "0300000080");
}

TEST(FlStandardMessageCodecTest, EncodeInt32Max) {
  g_autofree gchar* hex_string = encode_int(G_MAXINT32);
  EXPECT_STREQ(hex_string, "03ffffff7f");
}

TEST(FlStandardMessageCodecTest, EncodeInt64) {
  g_autofree gchar* hex_string = encode_int(0x0123456789abcdef);
  EXPECT_STREQ(hex_string, "04efcdab8967452301");
}

TEST(FlStandardMessageCodecTest, EncodeInt64Min) {
  g_autofree gchar* hex_string = encode_int(G_MININT64);
  EXPECT_STREQ(hex_string, "040000000000000080");
}

TEST(FlStandardMessageCodecTest, EncodeInt64Max) {
  g_autofree gchar* hex_string = encode_int(G_MAXINT64);
  EXPECT_STREQ(hex_string, "04ffffffffffffff7f");
}

TEST(FlStandardMessageCodecTest, DecodeIntZero) {
  g_autoptr(FlValue) value = decode_message("0300000000");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), 0);
}

TEST(FlStandardMessageCodecTest, DecodeIntOne) {
  g_autoptr(FlValue) value = decode_message("0301000000");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), 1);
}

TEST(FlStandardMessageCodecTest, DecodeInt32) {
  g_autoptr(FlValue) value = decode_message("0367452301");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), 0x01234567);
}

TEST(FlStandardMessageCodecTest, DecodeInt32Min) {
  g_autoptr(FlValue) value = decode_message("0300000080");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), G_MININT32);
}

TEST(FlStandardMessageCodecTest, DecodeInt32Max) {
  g_autoptr(FlValue) value = decode_message("03ffffff7f");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), G_MAXINT32);
}

TEST(FlStandardMessageCodecTest, DecodeInt64) {
  g_autoptr(FlValue) value = decode_message("04efcdab8967452301");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), 0x0123456789abcdef);
}

TEST(FlStandardMessageCodecTest, DecodeInt64Min) {
  g_autoptr(FlValue) value = decode_message("040000000000000080");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), G_MININT64);
}

TEST(FlStandardMessageCodecTest, DecodeInt64Max) {
  g_autoptr(FlValue) value = decode_message("04ffffffffffffff7f");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), G_MAXINT64);
}

TEST(FlStandardMessageCodecTest, DecodeInt32NoData) {
  decode_error_value("03", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeIntShortData1) {
  decode_error_value("0367", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeIntShortData2) {
  decode_error_value("03674523", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeInt64NoData) {
  decode_error_value("04", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeInt64ShortData1) {
  decode_error_value("04ef", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeInt64ShortData2) {
  decode_error_value("04efcdab89674523", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

static gchar* encode_float(double value) {
  g_autoptr(FlValue) v = fl_value_new_float(value);
  return encode_message(v);
}

TEST(FlStandardMessageCodecTest, EncodeFloatZero) {
  g_autofree gchar* hex_string = encode_float(0);
  EXPECT_STREQ(hex_string, "06000000000000000000000000000000");
}

TEST(FlStandardMessageCodecTest, EncodeFloatOne) {
  g_autofree gchar* hex_string = encode_float(1);
  EXPECT_STREQ(hex_string, "0600000000000000000000000000f03f");
}

TEST(FlStandardMessageCodecTest, EncodeFloatMinusOne) {
  g_autofree gchar* hex_string = encode_float(-1);
  EXPECT_STREQ(hex_string, "0600000000000000000000000000f0bf");
}

TEST(FlStandardMessageCodecTest, EncodeFloatHalf) {
  g_autofree gchar* hex_string = encode_float(0.5);
  EXPECT_STREQ(hex_string, "0600000000000000000000000000e03f");
}

TEST(FlStandardMessageCodecTest, EncodeFloatFraction) {
  g_autofree gchar* hex_string = encode_float(M_PI);
  EXPECT_STREQ(hex_string, "0600000000000000182d4454fb210940");
}

TEST(FlStandardMessageCodecTest, EncodeFloatMinusZero) {
  g_autofree gchar* hex_string = encode_float(-0.0);
  EXPECT_STREQ(hex_string, "06000000000000000000000000000080");
}

TEST(FlStandardMessageCodecTest, EncodeFloatNaN) {
  g_autofree gchar* hex_string = encode_float(NAN);
  EXPECT_STREQ(hex_string, "0600000000000000000000000000f87f");
}

TEST(FlStandardMessageCodecTest, EncodeFloatInfinity) {
  g_autofree gchar* hex_string = encode_float(INFINITY);
  EXPECT_STREQ(hex_string, "0600000000000000000000000000f07f");
}

TEST(FlStandardMessageCodecTest, DecodeFloatZero) {
  g_autoptr(FlValue) value = decode_message("06000000000000000000000000000000");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_float(value), 0.0);
}

TEST(FlStandardMessageCodecTest, DecodeFloatOne) {
  g_autoptr(FlValue) value = decode_message("0600000000000000000000000000f03f");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_float(value), 1.0);
}

TEST(FlStandardMessageCodecTest, DecodeFloatMinusOne) {
  g_autoptr(FlValue) value = decode_message("0600000000000000000000000000f0bf");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_float(value), -1.0);
}

TEST(FlStandardMessageCodecTest, DecodeFloatHalf) {
  g_autoptr(FlValue) value = decode_message("0600000000000000000000000000e03f");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_float(value), 0.5);
}

TEST(FlStandardMessageCodecTest, DecodeFloatPi) {
  g_autoptr(FlValue) value = decode_message("0600000000000000182d4454fb210940");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_float(value), M_PI);
}

TEST(FlStandardMessageCodecTest, DecodeFloatMinusZero) {
  g_autoptr(FlValue) value = decode_message("06000000000000000000000000000080");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_float(value), -0.0);
}

TEST(FlStandardMessageCodecTest, DecodeFloatNaN) {
  g_autoptr(FlValue) value = decode_message("0600000000000000000000000000f87f");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_TRUE(isnan(fl_value_get_float(value)));
}

TEST(FlStandardMessageCodecTest, DecodeFloatInfinity) {
  g_autoptr(FlValue) value = decode_message("0600000000000000000000000000f07f");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_TRUE(isinf(fl_value_get_float(value)));
}

TEST(FlStandardMessageCodecTest, DecodeFloatNoData) {
  decode_error_value("060000000000000000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeFloatShortData1) {
  decode_error_value("060000000000000000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeFloatShortData2) {
  decode_error_value("060000000000000000000000000000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

static gchar* encode_string(const gchar* value) {
  g_autoptr(FlValue) v = fl_value_new_string(value);
  return encode_message(v);
}

TEST(FlStandardMessageCodecTest, EncodeStringEmpty) {
  g_autofree gchar* hex_string = encode_string("");
  EXPECT_STREQ(hex_string, "0700");
}

TEST(FlStandardMessageCodecTest, EncodeStringHello) {
  g_autofree gchar* hex_string = encode_string("hello");
  EXPECT_STREQ(hex_string, "070568656c6c6f");
}

TEST(FlStandardMessageCodecTest, EncodeStringEmptySized) {
  g_autoptr(FlValue) value = fl_value_new_string_sized(nullptr, 0);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "0700");
}

TEST(FlStandardMessageCodecTest, EncodeStringHelloSized) {
  g_autoptr(FlValue) value = fl_value_new_string_sized("Hello World", 5);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "070548656c6c6f");
}

TEST(FlStandardMessageCodecTest, DecodeStringEmpty) {
  g_autoptr(FlValue) value = decode_message("0700");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(value), "");
}

TEST(FlStandardMessageCodecTest, DecodeStringHello) {
  g_autoptr(FlValue) value = decode_message("070568656c6c6f");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(value), "hello");
}

TEST(FlStandardMessageCodecTest, DecodeStringNoData) {
  decode_error_value("07", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeStringLengthNoData) {
  decode_error_value("0705", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeStringShortData1) {
  decode_error_value("070568", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeStringShortData2) {
  decode_error_value("070568656c6c", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, EncodeUint8ListEmpty) {
  g_autoptr(FlValue) value = fl_value_new_uint8_list(nullptr, 0);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "0800");
}

TEST(FlStandardMessageCodecTest, EncodeUint8List) {
  uint8_t data[] = {0, 1, 2, 3, 4};
  g_autoptr(FlValue) value = fl_value_new_uint8_list(data, 5);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "08050001020304");
}

TEST(FlStandardMessageCodecTest, DecodeUint8ListEmpty) {
  g_autoptr(FlValue) value = decode_message("0800");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_UINT8_LIST);
  EXPECT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlStandardMessageCodecTest, DecodeUint8List) {
  g_autoptr(FlValue) value = decode_message("08050001020304");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_UINT8_LIST);
  EXPECT_EQ(fl_value_get_length(value), static_cast<size_t>(5));
  const uint8_t* data = fl_value_get_uint8_list(value);
  EXPECT_EQ(data[0], 0);
  EXPECT_EQ(data[1], 1);
  EXPECT_EQ(data[2], 2);
  EXPECT_EQ(data[3], 3);
  EXPECT_EQ(data[4], 4);
}

TEST(FlStandardMessageCodecTest, DecodeUint8ListNoData) {
  decode_error_value("08", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeUint8ListLengthNoData) {
  decode_error_value("0805", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeUint8ListShortData1) {
  decode_error_value("080500", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeUint8ListShortData2) {
  decode_error_value("080500010203", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, EncodeInt32ListEmpty) {
  g_autoptr(FlValue) value = fl_value_new_int32_list(nullptr, 0);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "09000000");
}

TEST(FlStandardMessageCodecTest, EncodeInt32List) {
  int32_t data[] = {0, -1, 2, -3, 4};
  g_autoptr(FlValue) value = fl_value_new_int32_list(data, 5);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "0905000000000000ffffffff02000000fdffffff04000000");
}

TEST(FlStandardMessageCodecTest, DecodeInt32ListEmpty) {
  g_autoptr(FlValue) value = decode_message("09000000");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT32_LIST);
  EXPECT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlStandardMessageCodecTest, DecodeInt32List) {
  g_autoptr(FlValue) value =
      decode_message("0905000000000000ffffffff02000000fdffffff04000000");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT32_LIST);
  const int32_t* data = fl_value_get_int32_list(value);
  EXPECT_EQ(data[0], 0);
  EXPECT_EQ(data[1], -1);
  EXPECT_EQ(data[2], 2);
  EXPECT_EQ(data[3], -3);
  EXPECT_EQ(data[4], 4);
}

TEST(FlStandardMessageCodecTest, DecodeInt32ListNoData) {
  decode_error_value("09", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeInt32ListLengthNoData) {
  decode_error_value("09050000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeInt32ListShortData1) {
  decode_error_value("0905000000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeInt32ListShortData2) {
  decode_error_value("090500000000ffffffff02000000fdffffff040000",
                     FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, EncodeInt64ListEmpty) {
  g_autoptr(FlValue) value = fl_value_new_int64_list(nullptr, 0);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "0a00000000000000");
}

TEST(FlStandardMessageCodecTest, EncodeInt64List) {
  int64_t data[] = {0, -1, 2, -3, 4};
  g_autoptr(FlValue) value = fl_value_new_int64_list(data, 5);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(
      hex_string,
      "0a050000000000000000000000000000ffffffffffffffff0200000000000000fdffffff"
      "ffffffff0400000000000000");
}

TEST(FlStandardMessageCodecTest, DecodeInt64ListEmpty) {
  g_autoptr(FlValue) value = decode_message("0a00000000000000");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT64_LIST);
  EXPECT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlStandardMessageCodecTest, DecodeInt64List) {
  g_autoptr(FlValue) value = decode_message(
      "0a050000000000000000000000000000ffffffffffffffff0200000000000000fdffffff"
      "ffffffff0400000000000000");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT64_LIST);
  const int64_t* data = fl_value_get_int64_list(value);
  EXPECT_EQ(data[0], 0);
  EXPECT_EQ(data[1], -1);
  EXPECT_EQ(data[2], 2);
  EXPECT_EQ(data[3], -3);
  EXPECT_EQ(data[4], 4);
}

TEST(FlStandardMessageCodecTest, DecodeInt64ListNoData) {
  decode_error_value("0a", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeInt64ListLengthNoData) {
  decode_error_value("0a05000000000000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeInt64ListShortData1) {
  decode_error_value("0a0500000000000000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeInt64ListShortData2) {
  decode_error_value(
      "0a050000000000000000000000000000ffffffffffffffff0200000000000000fdffffff"
      "ffffffff0400"
      "0000000000",
      FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, EncodeFloat32ListEmpty) {
  g_autoptr(FlValue) value = fl_value_new_float32_list(nullptr, 0);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "0e000000");
}

TEST(FlStandardMessageCodecTest, EncodeFloat32List) {
  float data[] = {0.0f, -0.5f, 0.25f, -0.125f, 0.00625f};
  g_autoptr(FlValue) value = fl_value_new_float32_list(data, 5);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "0e05000000000000000000bf0000803e000000becdcccc3b");
}

TEST(FlStandardMessageCodecTest, DecodeFloat32ListEmpty) {
  g_autoptr(FlValue) value = decode_message("0e000000");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT32_LIST);
  EXPECT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlStandardMessageCodecTest, DecodeFloat32List) {
  g_autoptr(FlValue) value =
      decode_message("0e05000000000000000000bf0000803e000000becdcccc3b");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT32_LIST);
  const float* data = fl_value_get_float32_list(value);
  EXPECT_FLOAT_EQ(data[0], 0.0f);
  EXPECT_FLOAT_EQ(data[1], -0.5f);
  EXPECT_FLOAT_EQ(data[2], 0.25f);
  EXPECT_FLOAT_EQ(data[3], -0.125f);
  EXPECT_FLOAT_EQ(data[4], 0.00625f);
}

TEST(FlStandardMessageCodecTest, DecodeFloat32ListNoData) {
  decode_error_value("0e", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeFloat32ListLengthNoData) {
  decode_error_value("0e050000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeFloat32ListShortData1) {
  decode_error_value("0e05000000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeFloat32ListShortData2) {
  decode_error_value("0e05000000000000000000bf0000803e000000becdcccc",
                     FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, EncodeFloatListEmpty) {
  g_autoptr(FlValue) value = fl_value_new_float_list(nullptr, 0);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "0b00000000000000");
}

TEST(FlStandardMessageCodecTest, EncodeFloatList) {
  double data[] = {0, -0.5, 0.25, -0.125, 0.00625};
  g_autoptr(FlValue) value = fl_value_new_float_list(data, 5);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(
      hex_string,
      "0b050000000000000000000000000000000000000000e0bf000000000000d03f00000000"
      "0000c0bf9a9999999999793f");
}

TEST(FlStandardMessageCodecTest, DecodeFloatListEmpty) {
  g_autoptr(FlValue) value = decode_message("0b00000000000000");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT_LIST);
  EXPECT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlStandardMessageCodecTest, DecodeFloatList) {
  g_autoptr(FlValue) value = decode_message(
      "0b050000000000000000000000000000000000000000e0bf000000000000d03f00000000"
      "0000c0bf9a9999999999793f");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT_LIST);
  const double* data = fl_value_get_float_list(value);
  EXPECT_FLOAT_EQ(data[0], 0.0);
  EXPECT_FLOAT_EQ(data[1], -0.5);
  EXPECT_FLOAT_EQ(data[2], 0.25);
  EXPECT_FLOAT_EQ(data[3], -0.125);
  EXPECT_FLOAT_EQ(data[4], 0.00625);
}

TEST(FlStandardMessageCodecTest, DecodeFloatListNoData) {
  decode_error_value("0b", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeFloatListLengthNoData) {
  decode_error_value("0b05000000000000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeFloatListShortData1) {
  decode_error_value("0b0500000000000000", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeFloatListShortData2) {
  decode_error_value(
      "0b050000000000000000000000000000000000000000e0bf000000000000d03f00000000"
      "0000c0bf9a99"
      "9999999979",
      FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, EncodeListEmpty) {
  g_autoptr(FlValue) value = fl_value_new_list();
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "0c00");
}

TEST(FlStandardMessageCodecTest, EncodeListTypes) {
  g_autoptr(FlValue) value = fl_value_new_list();
  fl_value_append_take(value, fl_value_new_null());
  fl_value_append_take(value, fl_value_new_bool(TRUE));
  fl_value_append_take(value, fl_value_new_int(42));
  fl_value_append_take(value, fl_value_new_float(M_PI));
  fl_value_append_take(value, fl_value_new_string("hello"));
  fl_value_append_take(value, fl_value_new_list());
  fl_value_append_take(value, fl_value_new_map());
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(
      hex_string,
      "0c070001032a00000006000000000000182d4454fb210940070568656c6c6f0c000d00");
}

TEST(FlStandardMessageCodecTest, EncodeListNested) {
  g_autoptr(FlValue) even_numbers = fl_value_new_list();
  g_autoptr(FlValue) odd_numbers = fl_value_new_list();
  for (int i = 0; i < 10; i++) {
    if (i % 2 == 0) {
      fl_value_append_take(even_numbers, fl_value_new_int(i));
    } else {
      fl_value_append_take(odd_numbers, fl_value_new_int(i));
    }
  }
  g_autoptr(FlValue) value = fl_value_new_list();
  fl_value_append(value, even_numbers);
  fl_value_append(value, odd_numbers);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string,
               "0c020c05030000000003020000000304000000030600000003080000000c"
               "0503010000000303000000030500000003070000000309000000");
}

TEST(FlStandardMessageCodecTest, DecodeListEmpty) {
  g_autoptr(FlValue) value = decode_message("0c00");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_LIST);
  EXPECT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlStandardMessageCodecTest, DecodeListTypes) {
  g_autoptr(FlValue) value = decode_message(
      "0c070001032a00000006000000000000182d4454fb210940070568656c6c6f0c000d00");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(7));
  ASSERT_EQ(fl_value_get_type(fl_value_get_list_value(value, 0)),
            FL_VALUE_TYPE_NULL);
  ASSERT_EQ(fl_value_get_type(fl_value_get_list_value(value, 1)),
            FL_VALUE_TYPE_BOOL);
  EXPECT_TRUE(fl_value_get_bool(fl_value_get_list_value(value, 1)));
  ASSERT_EQ(fl_value_get_type(fl_value_get_list_value(value, 2)),
            FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(fl_value_get_list_value(value, 2)), 42);
  ASSERT_EQ(fl_value_get_type(fl_value_get_list_value(value, 3)),
            FL_VALUE_TYPE_FLOAT);
  EXPECT_FLOAT_EQ(fl_value_get_float(fl_value_get_list_value(value, 3)), M_PI);
  ASSERT_EQ(fl_value_get_type(fl_value_get_list_value(value, 4)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_list_value(value, 4)), "hello");
  ASSERT_EQ(fl_value_get_type(fl_value_get_list_value(value, 5)),
            FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(fl_value_get_list_value(value, 5)),
            static_cast<size_t>(0));
  ASSERT_EQ(fl_value_get_type(fl_value_get_list_value(value, 6)),
            FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(fl_value_get_list_value(value, 6)),
            static_cast<size_t>(0));
}

TEST(FlStandardMessageCodecTest, DecodeListNested) {
  g_autoptr(FlValue) value = decode_message(
      "0c020c05030000000003020000000304000000030600000003080000000c"
      "0503010000000303000000030500000003070000000309000000");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(2));
  FlValue* even_list = fl_value_get_list_value(value, 0);
  ASSERT_EQ(fl_value_get_type(even_list), FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(even_list), static_cast<size_t>(5));
  FlValue* odd_list = fl_value_get_list_value(value, 1);
  ASSERT_EQ(fl_value_get_type(odd_list), FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(odd_list), static_cast<size_t>(5));
  for (int i = 0; i < 5; i++) {
    FlValue* v = fl_value_get_list_value(even_list, i);
    ASSERT_EQ(fl_value_get_type(v), FL_VALUE_TYPE_INT);
    EXPECT_EQ(fl_value_get_int(v), i * 2);

    v = fl_value_get_list_value(odd_list, i);
    ASSERT_EQ(fl_value_get_type(v), FL_VALUE_TYPE_INT);
    EXPECT_EQ(fl_value_get_int(v), i * 2 + 1);
  }
}

TEST(FlStandardMessageCodecTest, DecodeListNoData) {
  decode_error_value("0c", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeListLengthNoData) {
  decode_error_value("0c07", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeListShortData1) {
  decode_error_value("0c0700", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeListShortData2) {
  decode_error_value(
      "0c070001032a00000006000000000000182d4454fb210940070568656c6c6f0c000d",
      FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, EncodeDecodeLargeList) {
  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();

  g_autoptr(FlValue) value = fl_value_new_list();
  for (int i = 0; i < 65535; i++) {
    fl_value_append_take(value, fl_value_new_int(i));
  }

  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_message_codec_encode_message(FL_MESSAGE_CODEC(codec), value, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  g_autoptr(FlValue) decoded_value =
      fl_message_codec_decode_message(FL_MESSAGE_CODEC(codec), message, &error);
  EXPECT_EQ(error, nullptr);
  EXPECT_NE(value, nullptr);

  ASSERT_TRUE(fl_value_equal(value, decoded_value));
}

TEST(FlStandardMessageCodecTest, EncodeMapEmpty) {
  g_autoptr(FlValue) value = fl_value_new_map();
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string, "0d00");
}

TEST(FlStandardMessageCodecTest, EncodeMapKeyTypes) {
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_take(value, fl_value_new_null(), fl_value_new_string("null"));
  fl_value_set_take(value, fl_value_new_bool(TRUE),
                    fl_value_new_string("bool"));
  fl_value_set_take(value, fl_value_new_int(42), fl_value_new_string("int"));
  fl_value_set_take(value, fl_value_new_float(M_PI),
                    fl_value_new_string("float"));
  fl_value_set_take(value, fl_value_new_string("hello"),
                    fl_value_new_string("string"));
  fl_value_set_take(value, fl_value_new_list(), fl_value_new_string("list"));
  fl_value_set_take(value, fl_value_new_map(), fl_value_new_string("map"));
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string,
               "0d070007046e756c6c010704626f6f6c032a0000000703696e7406000000000"
               "0182d4454fb2109400705666c6f6174070568656c6c6f0706737472696e670c"
               "0007046c6973740d0007036d6170");
}

TEST(FlStandardMessageCodecTest, EncodeMapValueTypes) {
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_take(value, fl_value_new_string("null"), fl_value_new_null());
  fl_value_set_take(value, fl_value_new_string("bool"),
                    fl_value_new_bool(TRUE));
  fl_value_set_take(value, fl_value_new_string("int"), fl_value_new_int(42));
  fl_value_set_take(value, fl_value_new_string("float"),
                    fl_value_new_float(M_PI));
  fl_value_set_take(value, fl_value_new_string("string"),
                    fl_value_new_string("hello"));
  fl_value_set_take(value, fl_value_new_string("list"), fl_value_new_list());
  fl_value_set_take(value, fl_value_new_string("map"), fl_value_new_map());
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string,
               "0d0707046e756c6c000704626f6f6c010703696e74032a0000000705666c6f6"
               "17406000000000000182d4454fb2109400706737472696e67070568656c6c6f"
               "07046c6973740c0007036d61700d00");
}

TEST(FlStandardMessageCodecTest, EncodeMapNested) {
  g_autoptr(FlValue) str_to_int = fl_value_new_map();
  g_autoptr(FlValue) int_to_str = fl_value_new_map();
  const char* numbers[] = {"zero", "one", "two", "three", nullptr};
  for (int i = 0; numbers[i] != nullptr; i++) {
    fl_value_set_take(str_to_int, fl_value_new_string(numbers[i]),
                      fl_value_new_int(i));
    fl_value_set_take(int_to_str, fl_value_new_int(i),
                      fl_value_new_string(numbers[i]));
  }
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_string(value, "str-to-int", str_to_int);
  fl_value_set_string(value, "int-to-str", int_to_str);
  g_autofree gchar* hex_string = encode_message(value);
  EXPECT_STREQ(hex_string,
               "0d02070a7374722d746f2d696e740d0407047a65726f030000000007036f6e6"
               "50301000000070374776f0302000000070574687265650303000000070a696e"
               "742d746f2d7374720d04030000000007047a65726f030100000007036f6e650"
               "302000000070374776f030300000007057468726565");
}

TEST(FlStandardMessageCodecTest, DecodeMapEmpty) {
  g_autoptr(FlValue) value = decode_message("0d00");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlStandardMessageCodecTest, DecodeMapKeyTypes) {
  g_autoptr(FlValue) value = decode_message(
      "0d070007046e756c6c010704626f6f6c032a0000000703696e74060000000000182d4454"
      "fb2109400705666c6f6174070568656c6c6f0706737472696e670c0007046c6973740d00"
      "07036d6170");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(7));

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 0)),
            FL_VALUE_TYPE_NULL);
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 0)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_value(value, 0)), "null");

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 1)),
            FL_VALUE_TYPE_BOOL);
  EXPECT_TRUE(fl_value_get_bool(fl_value_get_map_key(value, 1)));
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 1)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_value(value, 1)), "bool");

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 2)),
            FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(fl_value_get_map_key(value, 2)), 42);
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 2)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_value(value, 2)), "int");

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 3)),
            FL_VALUE_TYPE_FLOAT);
  EXPECT_FLOAT_EQ(fl_value_get_float(fl_value_get_map_key(value, 3)), M_PI);
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 3)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_value(value, 3)), "float");

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 4)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 4)), "hello");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 4)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_value(value, 4)), "string");

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 5)),
            FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(fl_value_get_map_key(value, 5)),
            static_cast<size_t>(0));
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 5)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_value(value, 5)), "list");

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 6)),
            FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(fl_value_get_map_key(value, 6)),
            static_cast<size_t>(0));
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 6)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_value(value, 6)), "map");
}

TEST(FlStandardMessageCodecTest, DecodeMapValueTypes) {
  g_autoptr(FlValue) value = decode_message(
      "0d0707046e756c6c000704626f6f6c010703696e74032a0000000705666c6f6174060000"
      "00000000182d4454fb2109400706737472696e67070568656c6c6f07046c6973740c0007"
      "036d61700d00");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(7));

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 0)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 0)), "null");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 0)),
            FL_VALUE_TYPE_NULL);

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 1)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 1)), "bool");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 1)),
            FL_VALUE_TYPE_BOOL);
  EXPECT_TRUE(fl_value_get_bool(fl_value_get_map_value(value, 1)));

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 2)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 2)), "int");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 2)),
            FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(fl_value_get_map_value(value, 2)), 42);

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 3)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 3)), "float");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 3)),
            FL_VALUE_TYPE_FLOAT);
  EXPECT_FLOAT_EQ(fl_value_get_float(fl_value_get_map_value(value, 3)), M_PI);

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 4)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 4)), "string");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 4)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_value(value, 4)), "hello");

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 5)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 5)), "list");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 5)),
            FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(fl_value_get_map_value(value, 5)),
            static_cast<size_t>(0));

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 6)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 6)), "map");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 6)),
            FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(fl_value_get_map_value(value, 6)),
            static_cast<size_t>(0));
}

TEST(FlStandardMessageCodecTest, DecodeMapNested) {
  g_autoptr(FlValue) value = decode_message(
      "0d02070a7374722d746f2d696e740d0407047a65726f030000000007036f6e6503010000"
      "00070374776f0302000000070574687265650303000000070a696e742d746f2d7374720d"
      "04030000000007047a65726f030100000007036f6e650302000000070374776f03030000"
      "0007057468726565");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(2));

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 0)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 0)),
               "str-to-int");
  FlValue* str_to_int = fl_value_get_map_value(value, 0);
  ASSERT_EQ(fl_value_get_type(str_to_int), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(str_to_int), static_cast<size_t>(4));

  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 1)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 1)),
               "int-to-str");
  FlValue* int_to_str = fl_value_get_map_value(value, 1);
  ASSERT_EQ(fl_value_get_type(int_to_str), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(int_to_str), static_cast<size_t>(4));

  const char* numbers[] = {"zero", "one", "two", "three", nullptr};
  for (int i = 0; numbers[i] != nullptr; i++) {
    ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(str_to_int, i)),
              FL_VALUE_TYPE_STRING);
    EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(str_to_int, i)),
                 numbers[i]);

    ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(str_to_int, i)),
              FL_VALUE_TYPE_INT);
    EXPECT_EQ(fl_value_get_int(fl_value_get_map_value(str_to_int, i)), i);

    ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(int_to_str, i)),
              FL_VALUE_TYPE_INT);
    EXPECT_EQ(fl_value_get_int(fl_value_get_map_key(int_to_str, i)), i);

    ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(int_to_str, i)),
              FL_VALUE_TYPE_STRING);
    EXPECT_STREQ(fl_value_get_string(fl_value_get_map_value(int_to_str, i)),
                 numbers[i]);
  }
}

TEST(FlStandardMessageCodecTest, DecodeMapNoData) {
  decode_error_value("0d", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeMapLengthNoData) {
  decode_error_value("0d07", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeMapShortData1) {
  decode_error_value("0d0707", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, DecodeMapShortData2) {
  decode_error_value(
      "0d0707046e756c6c000704626f6f6c010703696e74032a0000000705666c6f6174060000"
      "00000000182d4454fb2109400706737472696e67070568656c6c6f07046c6973740c0007"
      "036d61700d",
      FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMessageCodecTest, EncodeDecodeLargeMap) {
  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();

  g_autoptr(FlValue) value = fl_value_new_map();
  for (int i = 0; i < 512; i++) {
    g_autofree gchar* key = g_strdup_printf("key%d", i);
    fl_value_set_string_take(value, key, fl_value_new_int(i));
  }

  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_message_codec_encode_message(FL_MESSAGE_CODEC(codec), value, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  g_autoptr(FlValue) decoded_value =
      fl_message_codec_decode_message(FL_MESSAGE_CODEC(codec), message, &error);
  EXPECT_EQ(error, nullptr);
  EXPECT_NE(value, nullptr);

  ASSERT_TRUE(fl_value_equal(value, decoded_value));
}

TEST(FlStandardMessageCodecTest, DecodeUnknownType) {
  decode_error_value("0f", FL_MESSAGE_CODEC_ERROR,
                     FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE);
}

G_DECLARE_FINAL_TYPE(FlTestStandardMessageCodec,
                     fl_test_standard_message_codec,
                     FL,
                     TEST_STANDARD_MESSAGE_CODEC,
                     FlStandardMessageCodec)

struct _FlTestStandardMessageCodec {
  FlStandardMessageCodec parent_instance;
};

G_DEFINE_TYPE(FlTestStandardMessageCodec,
              fl_test_standard_message_codec,
              fl_standard_message_codec_get_type())

static gboolean write_custom_value1(FlStandardMessageCodec* codec,
                                    GByteArray* buffer,
                                    FlValue* value,
                                    GError** error) {
  const gchar* text =
      static_cast<const gchar*>(fl_value_get_custom_value(value));
  size_t length = strlen(text);

  uint8_t type = 128;
  g_byte_array_append(buffer, &type, sizeof(uint8_t));
  fl_standard_message_codec_write_size(codec, buffer, length);
  g_byte_array_append(buffer, reinterpret_cast<const uint8_t*>(text), length);
  return TRUE;
}

static gboolean write_custom_value2(FlStandardMessageCodec* codec,
                                    GByteArray* buffer,
                                    FlValue* value,
                                    GError** error) {
  uint8_t type = 129;
  g_byte_array_append(buffer, &type, sizeof(uint8_t));
  return TRUE;
}

static gboolean fl_test_standard_message_codec_write_value(
    FlStandardMessageCodec* codec,
    GByteArray* buffer,
    FlValue* value,
    GError** error) {
  if (fl_value_get_type(value) == FL_VALUE_TYPE_CUSTOM &&
      fl_value_get_custom_type(value) == 128) {
    return write_custom_value1(codec, buffer, value, error);
  } else if (fl_value_get_type(value) == FL_VALUE_TYPE_CUSTOM &&
             fl_value_get_custom_type(value) == 129) {
    return write_custom_value2(codec, buffer, value, error);
  } else {
    return FL_STANDARD_MESSAGE_CODEC_CLASS(
               fl_test_standard_message_codec_parent_class)
        ->write_value(codec, buffer, value, error);
  }
}

static FlValue* read_custom_value1(FlStandardMessageCodec* codec,
                                   GBytes* buffer,
                                   size_t* offset,
                                   GError** error) {
  uint32_t length;
  if (!fl_standard_message_codec_read_size(codec, buffer, offset, &length,
                                           error)) {
    return nullptr;
  }
  if (*offset + length > g_bytes_get_size(buffer)) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA, "Unexpected end of data");
    return nullptr;
  }
  FlValue* value = fl_value_new_custom(
      128,
      g_strndup(static_cast<const gchar*>(g_bytes_get_data(buffer, nullptr)) +
                    *offset,
                length),
      g_free);
  *offset += length;

  return value;
}

static FlValue* read_custom_value2(FlStandardMessageCodec* codec,
                                   GBytes* buffer,
                                   size_t* offset,
                                   GError** error) {
  return fl_value_new_custom(129, nullptr, nullptr);
}

static FlValue* fl_test_standard_message_codec_read_value_of_type(
    FlStandardMessageCodec* codec,
    GBytes* buffer,
    size_t* offset,
    int type,
    GError** error) {
  if (type == 128) {
    return read_custom_value1(codec, buffer, offset, error);
  } else if (type == 129) {
    return read_custom_value2(codec, buffer, offset, error);
  } else {
    return FL_STANDARD_MESSAGE_CODEC_CLASS(
               fl_test_standard_message_codec_parent_class)
        ->read_value_of_type(codec, buffer, offset, type, error);
  }
}

static void fl_test_standard_message_codec_class_init(
    FlTestStandardMessageCodecClass* klass) {
  FL_STANDARD_MESSAGE_CODEC_CLASS(klass)->write_value =
      fl_test_standard_message_codec_write_value;
  FL_STANDARD_MESSAGE_CODEC_CLASS(klass)->read_value_of_type =
      fl_test_standard_message_codec_read_value_of_type;
}

static void fl_test_standard_message_codec_init(
    FlTestStandardMessageCodec* self) {
  // The following line suppresses a warning for unused function
  FL_IS_TEST_STANDARD_MESSAGE_CODEC(self);
}

static FlTestStandardMessageCodec* fl_test_standard_message_codec_new() {
  return FL_TEST_STANDARD_MESSAGE_CODEC(
      g_object_new(fl_test_standard_message_codec_get_type(), nullptr));
}

TEST(FlStandardMessageCodecTest, DecodeCustomType) {
  g_autoptr(FlTestStandardMessageCodec) codec =
      fl_test_standard_message_codec_new();
  g_autoptr(FlValue) value =
      decode_message_with_codec("800568656c6c6f", FL_MESSAGE_CODEC(codec));
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_CUSTOM);
  ASSERT_EQ(fl_value_get_custom_type(value), 128);
  EXPECT_STREQ(static_cast<const gchar*>(fl_value_get_custom_value(value)),
               "hello");
}

TEST(FlStandardMessageCodecTest, DecodeCustomTypes) {
  g_autoptr(FlTestStandardMessageCodec) codec =
      fl_test_standard_message_codec_new();
  g_autoptr(FlValue) value = decode_message_with_codec("0c02800568656c6c6f81",
                                                       FL_MESSAGE_CODEC(codec));
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_LIST);
  EXPECT_EQ(fl_value_get_length(value), static_cast<size_t>(2));
  FlValue* value1 = fl_value_get_list_value(value, 0);
  ASSERT_EQ(fl_value_get_type(value1), FL_VALUE_TYPE_CUSTOM);
  ASSERT_EQ(fl_value_get_custom_type(value1), 128);
  EXPECT_STREQ(static_cast<const gchar*>(fl_value_get_custom_value(value1)),
               "hello");
  FlValue* value2 = fl_value_get_list_value(value, 1);
  ASSERT_EQ(fl_value_get_type(value2), FL_VALUE_TYPE_CUSTOM);
  ASSERT_EQ(fl_value_get_custom_type(value2), 129);
}

TEST(FlStandardMessageCodecTest, EncodeCustomType) {
  g_autoptr(FlValue) value = fl_value_new_custom(128, "hello", nullptr);
  g_autoptr(FlTestStandardMessageCodec) codec =
      fl_test_standard_message_codec_new();
  g_autofree gchar* hex_string =
      encode_message_with_codec(value, FL_MESSAGE_CODEC(codec));
  EXPECT_STREQ(hex_string, "800568656c6c6f");
}

TEST(FlStandardMessageCodecTest, EncodeCustomTypes) {
  g_autoptr(FlValue) value = fl_value_new_list();
  fl_value_append_take(value, fl_value_new_custom(128, "hello", nullptr));
  fl_value_append_take(value, fl_value_new_custom(129, nullptr, nullptr));
  g_autoptr(FlTestStandardMessageCodec) codec =
      fl_test_standard_message_codec_new();
  g_autofree gchar* hex_string =
      encode_message_with_codec(value, FL_MESSAGE_CODEC(codec));
  EXPECT_STREQ(hex_string, "0c02800568656c6c6f81");
}

TEST(FlStandardMessageCodecTest, EncodeDecode) {
  g_autoptr(FlStandardMessageCodec) codec = fl_standard_message_codec_new();

  g_autoptr(FlValue) input = fl_value_new_list();
  fl_value_append_take(input, fl_value_new_null());
  fl_value_append_take(input, fl_value_new_bool(TRUE));
  fl_value_append_take(input, fl_value_new_int(42));
  fl_value_append_take(input, fl_value_new_float(M_PI));
  fl_value_append_take(input, fl_value_new_string("hello"));
  fl_value_append_take(input, fl_value_new_list());
  fl_value_append_take(input, fl_value_new_map());

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

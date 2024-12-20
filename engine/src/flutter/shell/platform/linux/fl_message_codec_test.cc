// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_message_codec.h"
#include "gtest/gtest.h"

G_DECLARE_FINAL_TYPE(FlTestCodec, fl_test_codec, FL, TEST_CODEC, FlMessageCodec)

// Implement the FlMessageCodec API for the following tests to check it works as
// expected.
struct _FlTestCodec {
  FlMessageCodec parent_instance;
};

G_DEFINE_TYPE(FlTestCodec, fl_test_codec, fl_message_codec_get_type())

// Implements FlMessageCodec::encode_message.
static GBytes* fl_test_codec_encode_message(FlMessageCodec* codec,
                                            FlValue* value,
                                            GError** error) {
  EXPECT_TRUE(FL_IS_TEST_CODEC(codec));

  if (fl_value_get_type(value) == FL_VALUE_TYPE_INT) {
    char c = '0' + fl_value_get_int(value);
    return g_bytes_new(&c, 1);
  } else {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "ERROR");
    return nullptr;
  }
}

// Implements FlMessageCodec::decode_message.
static FlValue* fl_test_codec_decode_message(FlMessageCodec* codec,
                                             GBytes* message,
                                             GError** error) {
  EXPECT_TRUE(FL_IS_TEST_CODEC(codec));

  size_t data_length;
  const uint8_t* data =
      static_cast<const uint8_t*>(g_bytes_get_data(message, &data_length));
  if (data_length < 1) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA, "No data");
    return FALSE;
  }
  if (data_length > 1) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                FL_MESSAGE_CODEC_ERROR_ADDITIONAL_DATA,
                "FL_MESSAGE_CODEC_ERROR_ADDITIONAL_DATA");
    return FALSE;
  }

  return fl_value_new_int(data[0] - '0');
}

static void fl_test_codec_class_init(FlTestCodecClass* klass) {
  FL_MESSAGE_CODEC_CLASS(klass)->encode_message = fl_test_codec_encode_message;
  FL_MESSAGE_CODEC_CLASS(klass)->decode_message = fl_test_codec_decode_message;
}

static void fl_test_codec_init(FlTestCodec* self) {}

static FlTestCodec* fl_test_codec_new() {
  return FL_TEST_CODEC(g_object_new(fl_test_codec_get_type(), nullptr));
}

TEST(FlMessageCodecTest, EncodeMessage) {
  g_autoptr(FlTestCodec) codec = fl_test_codec_new();

  g_autoptr(FlValue) value = fl_value_new_int(1);
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_message_codec_encode_message(FL_MESSAGE_CODEC(codec), value, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);
  EXPECT_EQ(g_bytes_get_size(message), static_cast<gsize>(1));
  EXPECT_EQ(static_cast<const uint8_t*>(g_bytes_get_data(message, nullptr))[0],
            '1');
}

TEST(FlMessageCodecTest, EncodeMessageError) {
  g_autoptr(FlTestCodec) codec = fl_test_codec_new();

  g_autoptr(FlValue) value = fl_value_new_null();
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message =
      fl_message_codec_encode_message(FL_MESSAGE_CODEC(codec), value, &error);
  EXPECT_EQ(message, nullptr);
  EXPECT_TRUE(g_error_matches(error, FL_MESSAGE_CODEC_ERROR,
                              FL_MESSAGE_CODEC_ERROR_FAILED));
}

TEST(FlMessageCodecTest, DecodeMessageEmpty) {
  g_autoptr(FlTestCodec) codec = fl_test_codec_new();
  g_autoptr(GBytes) message = g_bytes_new(nullptr, 0);

  g_autoptr(GError) error = nullptr;
  g_autoptr(FlValue) value =
      fl_message_codec_decode_message(FL_MESSAGE_CODEC(codec), message, &error);
  EXPECT_EQ(value, nullptr);
  EXPECT_TRUE(g_error_matches(error, FL_MESSAGE_CODEC_ERROR,
                              FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA));
}

TEST(FlMessageCodecTest, DecodeMessage) {
  g_autoptr(FlTestCodec) codec = fl_test_codec_new();
  uint8_t data[] = {'1'};
  g_autoptr(GBytes) message = g_bytes_new(data, 1);

  g_autoptr(GError) error = nullptr;
  g_autoptr(FlValue) value =
      fl_message_codec_decode_message(FL_MESSAGE_CODEC(codec), message, &error);
  EXPECT_NE(value, nullptr);
  EXPECT_EQ(error, nullptr);

  ASSERT_TRUE(fl_value_get_type(value) == FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), 1);
}

TEST(FlMessageCodecTest, DecodeMessageExtraData) {
  g_autoptr(FlTestCodec) codec = fl_test_codec_new();
  uint8_t data[] = {'1', '2'};
  g_autoptr(GBytes) message = g_bytes_new(data, 2);

  g_autoptr(GError) error = nullptr;
  g_autoptr(FlValue) value =
      fl_message_codec_decode_message(FL_MESSAGE_CODEC(codec), message, &error);
  EXPECT_EQ(value, nullptr);
  EXPECT_TRUE(g_error_matches(error, FL_MESSAGE_CODEC_ERROR,
                              FL_MESSAGE_CODEC_ERROR_ADDITIONAL_DATA));
}

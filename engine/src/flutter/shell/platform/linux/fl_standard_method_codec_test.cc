// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"
#include "flutter/shell/platform/linux/fl_method_codec_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_message_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_message_codec.h"
#include "flutter/shell/platform/linux/testing/fl_test.h"
#include "gtest/gtest.h"

G_DECLARE_FINAL_TYPE(FlTestMethodMessageCodec,
                     fl_test_method_message_codec,
                     FL,
                     TEST_METHOD_MESSAGE_CODEC,
                     FlStandardMessageCodec)

struct _FlTestMethodMessageCodec {
  FlStandardMessageCodec parent_instance;
};

G_DEFINE_TYPE(FlTestMethodMessageCodec,
              fl_test_method_message_codec,
              fl_standard_message_codec_get_type())

static gboolean write_custom_value(FlStandardMessageCodec* codec,
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

static gboolean fl_test_method_message_codec_write_value(
    FlStandardMessageCodec* codec,
    GByteArray* buffer,
    FlValue* value,
    GError** error) {
  if (fl_value_get_type(value) == FL_VALUE_TYPE_CUSTOM &&
      fl_value_get_custom_type(value) == 128) {
    return write_custom_value(codec, buffer, value, error);
  } else {
    return FL_STANDARD_MESSAGE_CODEC_CLASS(
               fl_test_method_message_codec_parent_class)
        ->write_value(codec, buffer, value, error);
  }
}

static FlValue* read_custom_value(FlStandardMessageCodec* codec,
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

static FlValue* fl_test_method_message_codec_read_value_of_type(
    FlStandardMessageCodec* codec,
    GBytes* buffer,
    size_t* offset,
    int type,
    GError** error) {
  if (type == 128) {
    return read_custom_value(codec, buffer, offset, error);
  } else {
    return FL_STANDARD_MESSAGE_CODEC_CLASS(
               fl_test_method_message_codec_parent_class)
        ->read_value_of_type(codec, buffer, offset, type, error);
  }
}

static void fl_test_method_message_codec_class_init(
    FlTestMethodMessageCodecClass* klass) {
  FL_STANDARD_MESSAGE_CODEC_CLASS(klass)->write_value =
      fl_test_method_message_codec_write_value;
  FL_STANDARD_MESSAGE_CODEC_CLASS(klass)->read_value_of_type =
      fl_test_method_message_codec_read_value_of_type;
}

static void fl_test_method_message_codec_init(FlTestMethodMessageCodec* self) {
  // The following line suppresses a warning for unused function
  FL_IS_TEST_METHOD_MESSAGE_CODEC(self);
}

static FlTestMethodMessageCodec* fl_test_method_message_codec_new() {
  return FL_TEST_METHOD_MESSAGE_CODEC(
      g_object_new(fl_test_method_message_codec_get_type(), nullptr));
}

// NOTE(robert-ancell) These test cases assumes a little-endian architecture.
// These tests will need to be updated if tested on a big endian architecture.

// Encodes a method call using StandardMethodCodec to a hex string.
static gchar* encode_method_call(const gchar* name, FlValue* args) {
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message = fl_method_codec_encode_method_call(
      FL_METHOD_CODEC(codec), name, args, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  return bytes_to_hex_string(message);
}

// Encodes a success envelope response using StandardMethodCodec to a hex
// string.
static gchar* encode_success_envelope(FlValue* result) {
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message = fl_method_codec_encode_success_envelope(
      FL_METHOD_CODEC(codec), result, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  return bytes_to_hex_string(message);
}

// Encodes a error envelope response using StandardMethodCodec to a hex string.
static gchar* encode_error_envelope(const gchar* error_code,
                                    const gchar* error_message,
                                    FlValue* details) {
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message = fl_method_codec_encode_error_envelope(
      FL_METHOD_CODEC(codec), error_code, error_message, details, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  return bytes_to_hex_string(message);
}

// Decodes a method call using StandardMethodCodec with a hex string.
static void decode_method_call(const char* hex_string,
                               gchar** name,
                               FlValue** args) {
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(GBytes) message = hex_string_to_bytes(hex_string);
  g_autoptr(GError) error = nullptr;
  gboolean result = fl_method_codec_decode_method_call(
      FL_METHOD_CODEC(codec), message, name, args, &error);
  EXPECT_TRUE(result);
  EXPECT_EQ(error, nullptr);
}

// Decodes a method call using StandardMethodCodec. Expect the given error.
static void decode_error_method_call(const char* hex_string,
                                     GQuark domain,
                                     gint code) {
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(GBytes) message = hex_string_to_bytes(hex_string);
  g_autoptr(GError) error = nullptr;
  g_autofree gchar* name = nullptr;
  g_autoptr(FlValue) args = nullptr;
  gboolean result = fl_method_codec_decode_method_call(
      FL_METHOD_CODEC(codec), message, &name, &args, &error);
  EXPECT_FALSE(result);
  EXPECT_EQ(name, nullptr);
  EXPECT_EQ(args, nullptr);
  EXPECT_TRUE(g_error_matches(error, domain, code));
}

// Decodes a response using StandardMethodCodec. Expect the response is a
// result.
static void decode_response_with_success(const char* hex_string,
                                         FlValue* result) {
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(GBytes) message = hex_string_to_bytes(hex_string);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlMethodResponse) response =
      fl_method_codec_decode_response(FL_METHOD_CODEC(codec), message, &error);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(error, nullptr);
  EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
  EXPECT_TRUE(fl_value_equal(fl_method_success_response_get_result(
                                 FL_METHOD_SUCCESS_RESPONSE(response)),
                             result));
}

// Decodes a response using StandardMethodCodec. Expect the response contains
// the given error.
static void decode_response_with_error(const char* hex_string,
                                       const gchar* code,
                                       const gchar* error_message,
                                       FlValue* details) {
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(GBytes) message = hex_string_to_bytes(hex_string);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlMethodResponse) response =
      fl_method_codec_decode_response(FL_METHOD_CODEC(codec), message, &error);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(error, nullptr);
  EXPECT_TRUE(FL_IS_METHOD_ERROR_RESPONSE(response));
  EXPECT_STREQ(
      fl_method_error_response_get_code(FL_METHOD_ERROR_RESPONSE(response)),
      code);
  if (error_message == nullptr) {
    EXPECT_EQ(fl_method_error_response_get_message(
                  FL_METHOD_ERROR_RESPONSE(response)),
              nullptr);
  } else {
    EXPECT_STREQ(fl_method_error_response_get_message(
                     FL_METHOD_ERROR_RESPONSE(response)),
                 error_message);
  }
  if (details == nullptr) {
    EXPECT_EQ(fl_method_error_response_get_details(
                  FL_METHOD_ERROR_RESPONSE(response)),
              nullptr);
  } else {
    EXPECT_TRUE(fl_value_equal(fl_method_error_response_get_details(
                                   FL_METHOD_ERROR_RESPONSE(response)),
                               details));
  }
}

static void decode_error_response(const char* hex_string,
                                  GQuark domain,
                                  gint code) {
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(GBytes) message = hex_string_to_bytes(hex_string);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlMethodResponse) response =
      fl_method_codec_decode_response(FL_METHOD_CODEC(codec), message, &error);
  EXPECT_EQ(response, nullptr);
  EXPECT_TRUE(g_error_matches(error, domain, code));
}

TEST(FlStandardMethodCodecTest, EncodeMethodCallNullptrArgs) {
  g_autofree gchar* hex_string = encode_method_call("hello", nullptr);
  EXPECT_STREQ(hex_string, "070568656c6c6f00");
}

TEST(FlStandardMethodCodecTest, EncodeMethodCallNullArgs) {
  g_autoptr(FlValue) value = fl_value_new_null();
  g_autofree gchar* hex_string = encode_method_call("hello", value);
  EXPECT_STREQ(hex_string, "070568656c6c6f00");
}

TEST(FlStandardMethodCodecTest, EncodeMethodCallStringArgs) {
  g_autoptr(FlValue) args = fl_value_new_string("world");
  g_autofree gchar* hex_string = encode_method_call("hello", args);
  EXPECT_STREQ(hex_string, "070568656c6c6f0705776f726c64");
}

TEST(FlStandardMethodCodecTest, EncodeMethodCallListArgs) {
  g_autoptr(FlValue) args = fl_value_new_list();
  fl_value_append_take(args, fl_value_new_string("count"));
  fl_value_append_take(args, fl_value_new_int(42));
  g_autofree gchar* hex_string = encode_method_call("hello", args);
  EXPECT_STREQ(hex_string, "070568656c6c6f0c020705636f756e74032a000000");
}

TEST(FlStandardMethodCodecTest, DecodeMethodCallNullArgs) {
  g_autofree gchar* name = nullptr;
  g_autoptr(FlValue) args = nullptr;
  decode_method_call("070568656c6c6f00", &name, &args);
  EXPECT_STREQ(name, "hello");
  ASSERT_EQ(fl_value_get_type(args), FL_VALUE_TYPE_NULL);
}

TEST(FlStandardMethodCodecTest, DecodeMethodCallStringArgs) {
  g_autofree gchar* name = nullptr;
  g_autoptr(FlValue) args = nullptr;
  decode_method_call("070568656c6c6f0705776f726c64", &name, &args);
  EXPECT_STREQ(name, "hello");
  ASSERT_EQ(fl_value_get_type(args), FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(args), "world");
}

TEST(FlStandardMethodCodecTest, DecodeMethodCallListArgs) {
  g_autofree gchar* name = nullptr;
  g_autoptr(FlValue) args = nullptr;
  decode_method_call("070568656c6c6f0c020705636f756e74032a000000", &name,
                     &args);
  EXPECT_STREQ(name, "hello");
  ASSERT_EQ(fl_value_get_type(args), FL_VALUE_TYPE_LIST);
  EXPECT_EQ(fl_value_get_length(args), static_cast<size_t>(2));

  FlValue* arg0 = fl_value_get_list_value(args, 0);
  ASSERT_EQ(fl_value_get_type(arg0), FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(arg0), "count");

  FlValue* arg1 = fl_value_get_list_value(args, 1);
  ASSERT_EQ(fl_value_get_type(arg1), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(arg1), 42);
}

TEST(FlStandardMethodCodecTest, DecodeMethodCallNoData) {
  decode_error_method_call("", FL_MESSAGE_CODEC_ERROR,
                           FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMethodCodecTest, DecodeMethodCallNullMethodName) {
  decode_error_method_call("000000", FL_MESSAGE_CODEC_ERROR,
                           FL_MESSAGE_CODEC_ERROR_FAILED);
}

TEST(FlStandardMethodCodecTest, DecodeMethodCallMissingArgs) {
  decode_error_method_call("070568656c6c6f", FL_MESSAGE_CODEC_ERROR,
                           FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMethodCodecTest, EncodeSuccessEnvelopeNullptr) {
  g_autofree gchar* hex_string = encode_success_envelope(nullptr);
  EXPECT_STREQ(hex_string, "0000");
}

TEST(FlStandardMethodCodecTest, EncodeSuccessEnvelopeNull) {
  g_autoptr(FlValue) result = fl_value_new_null();
  g_autofree gchar* hex_string = encode_success_envelope(result);
  EXPECT_STREQ(hex_string, "0000");
}

TEST(FlStandardMethodCodecTest, EncodeSuccessEnvelopeString) {
  g_autoptr(FlValue) result = fl_value_new_string("hello");
  g_autofree gchar* hex_string = encode_success_envelope(result);
  EXPECT_STREQ(hex_string, "00070568656c6c6f");
}

TEST(FlStandardMethodCodecTest, EncodeSuccessEnvelopeList) {
  g_autoptr(FlValue) result = fl_value_new_list();
  fl_value_append_take(result, fl_value_new_string("count"));
  fl_value_append_take(result, fl_value_new_int(42));
  g_autofree gchar* hex_string = encode_success_envelope(result);
  EXPECT_STREQ(hex_string, "000c020705636f756e74032a000000");
}

TEST(FlStandardMethodCodecTest, EncodeErrorEnvelopeEmptyCode) {
  g_autofree gchar* hex_string = encode_error_envelope("", nullptr, nullptr);
  EXPECT_STREQ(hex_string, "0107000000");
}

TEST(FlStandardMethodCodecTest, EncodeErrorEnvelopeNonMessageOrDetails) {
  g_autofree gchar* hex_string =
      encode_error_envelope("error", nullptr, nullptr);
  EXPECT_STREQ(hex_string, "0107056572726f720000");
}

TEST(FlStandardMethodCodecTest, EncodeErrorEnvelopeMessage) {
  g_autofree gchar* hex_string =
      encode_error_envelope("error", "message", nullptr);
  EXPECT_STREQ(hex_string, "0107056572726f7207076d65737361676500");
}

TEST(FlStandardMethodCodecTest, EncodeErrorEnvelopeDetails) {
  g_autoptr(FlValue) details = fl_value_new_list();
  fl_value_append_take(details, fl_value_new_string("count"));
  fl_value_append_take(details, fl_value_new_int(42));
  g_autofree gchar* hex_string =
      encode_error_envelope("error", nullptr, details);
  EXPECT_STREQ(hex_string, "0107056572726f72000c020705636f756e74032a000000");
}

TEST(FlStandardMethodCodecTest, EncodeErrorEnvelopeMessageAndDetails) {
  g_autoptr(FlValue) details = fl_value_new_list();
  fl_value_append_take(details, fl_value_new_string("count"));
  fl_value_append_take(details, fl_value_new_int(42));
  g_autofree gchar* hex_string =
      encode_error_envelope("error", "message", details);
  EXPECT_STREQ(
      hex_string,
      "0107056572726f7207076d6573736167650c020705636f756e74032a000000");
}

TEST(FlStandardMethodCodecTest, DecodeResponseSuccessNull) {
  g_autoptr(FlValue) result = fl_value_new_null();
  decode_response_with_success("0000", result);
}

TEST(FlStandardMethodCodecTest, DecodeResponseSuccessString) {
  g_autoptr(FlValue) result = fl_value_new_string("hello");
  decode_response_with_success("00070568656c6c6f", result);
}

TEST(FlStandardMethodCodecTest, DecodeResponseSuccessList) {
  g_autoptr(FlValue) result = fl_value_new_list();
  fl_value_append_take(result, fl_value_new_string("count"));
  fl_value_append_take(result, fl_value_new_int(42));
  decode_response_with_success("000c020705636f756e74032a000000", result);
}

TEST(FlStandardMethodCodecTest, DecodeResponseErrorEmptyCode) {
  decode_response_with_error("0107000000", "", nullptr, nullptr);
}

TEST(FlStandardMethodCodecTest, DecodeResponseErrorNoMessageOrDetails) {
  decode_response_with_error("0107056572726f720000", "error", nullptr, nullptr);
}

TEST(FlStandardMethodCodecTest, DecodeResponseErrorMessage) {
  decode_response_with_error("0107056572726f7207076d65737361676500", "error",
                             "message", nullptr);
}

TEST(FlStandardMethodCodecTest, DecodeResponseErrorDetails) {
  g_autoptr(FlValue) details = fl_value_new_list();
  fl_value_append_take(details, fl_value_new_string("count"));
  fl_value_append_take(details, fl_value_new_int(42));
  decode_response_with_error("0107056572726f72000c020705636f756e74032a000000",
                             "error", nullptr, details);
}

TEST(FlStandardMethodCodecTest, DecodeResponseErrorMessageAndDetails) {
  g_autoptr(FlValue) details = fl_value_new_list();
  fl_value_append_take(details, fl_value_new_string("count"));
  fl_value_append_take(details, fl_value_new_int(42));
  decode_response_with_error(
      "0107056572726f7207076d6573736167650c020705636f756e74032a000000", "error",
      "message", details);
}

TEST(FlStandardMethodCodecTest, DecodeResponseSuccessNoData) {
  decode_error_response("00", FL_MESSAGE_CODEC_ERROR,
                        FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMethodCodecTest, DecodeResponseSuccessExtraData) {
  decode_error_response("000000", FL_MESSAGE_CODEC_ERROR,
                        FL_MESSAGE_CODEC_ERROR_FAILED);
}

TEST(FlStandardMethodCodecTest, DecodeResponseErrorNoData) {
  decode_error_response("01", FL_MESSAGE_CODEC_ERROR,
                        FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMethodCodecTest, DecodeResponseErrorMissingMessageAndDetails) {
  decode_error_response("0107056572726f72", FL_MESSAGE_CODEC_ERROR,
                        FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMethodCodecTest, DecodeResponseErrorMissingDetails) {
  decode_error_response("0107056572726f7200", FL_MESSAGE_CODEC_ERROR,
                        FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA);
}

TEST(FlStandardMethodCodecTest, DecodeResponseErrorExtraData) {
  decode_error_response("0107056572726f72000000", FL_MESSAGE_CODEC_ERROR,
                        FL_MESSAGE_CODEC_ERROR_FAILED);
}

TEST(FlStandardMethodCodecTest, DecodeResponseNotImplemented) {
  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  g_autoptr(GBytes) message = g_bytes_new(nullptr, 0);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlMethodResponse) response =
      fl_method_codec_decode_response(FL_METHOD_CODEC(codec), message, &error);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(error, nullptr);
  EXPECT_TRUE(FL_IS_METHOD_NOT_IMPLEMENTED_RESPONSE(response));
}

TEST(FlStandardMethodCodecTest, DecodeResponseUnknownEnvelope) {
  decode_error_response("02", FL_MESSAGE_CODEC_ERROR,
                        FL_MESSAGE_CODEC_ERROR_FAILED);
}

TEST(FlStandardMethodCodecTest, CustomMessageCodec) {
  g_autoptr(FlTestMethodMessageCodec) message_codec =
      fl_test_method_message_codec_new();
  g_autoptr(FlStandardMethodCodec) codec =
      fl_standard_method_codec_new_with_message_codec(
          FL_STANDARD_MESSAGE_CODEC(message_codec));

  g_autoptr(GError) error = nullptr;
  g_autoptr(FlValue) value = fl_value_new_custom(128, "hello", nullptr);
  g_autoptr(GBytes) message = fl_method_codec_encode_success_envelope(
      FL_METHOD_CODEC(codec), value, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);
  g_autofree gchar* hex_string = bytes_to_hex_string(message);
  EXPECT_STREQ(hex_string, "00800568656c6c6f");

  g_autoptr(FlMethodResponse) response =
      fl_method_codec_decode_response(FL_METHOD_CODEC(codec), message, &error);
  EXPECT_NE(response, nullptr);
  EXPECT_EQ(error, nullptr);
  EXPECT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
  FlValue* result = fl_method_success_response_get_result(
      FL_METHOD_SUCCESS_RESPONSE(response));
  ASSERT_EQ(fl_value_get_type(result), FL_VALUE_TYPE_CUSTOM);
  ASSERT_EQ(fl_value_get_custom_type(result), 128);
  EXPECT_STREQ(static_cast<const gchar*>(fl_value_get_custom_value(result)),
               "hello");
}

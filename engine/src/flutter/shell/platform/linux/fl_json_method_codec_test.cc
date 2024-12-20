// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_method_codec.h"

#include <cstring>

#include "flutter/shell/platform/linux/fl_method_codec_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_message_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_message_codec.h"
#include "gtest/gtest.h"

// Converts a binary blob to a string.
static gchar* message_to_text(GBytes* message) {
  size_t data_length;
  const gchar* data =
      static_cast<const gchar*>(g_bytes_get_data(message, &data_length));
  return g_strndup(data, data_length);
}

// Converts a string to a binary blob.
static GBytes* text_to_message(const gchar* text) {
  return g_bytes_new(text, strlen(text));
}

// Encodes a method call using JsonMethodCodec to a UTF-8 string.
static gchar* encode_method_call(const gchar* name, FlValue* args) {
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message = fl_method_codec_encode_method_call(
      FL_METHOD_CODEC(codec), name, args, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  return message_to_text(message);
}

// Encodes a success envelope response using JsonMethodCodec to a UTF-8 string.
static gchar* encode_success_envelope(FlValue* result) {
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message = fl_method_codec_encode_success_envelope(
      FL_METHOD_CODEC(codec), result, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  return message_to_text(message);
}

// Encodes a error envelope response using JsonMethodCodec to a UTF8 string.
static gchar* encode_error_envelope(const gchar* error_code,
                                    const gchar* error_message,
                                    FlValue* details) {
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  g_autoptr(GError) error = nullptr;
  g_autoptr(GBytes) message = fl_method_codec_encode_error_envelope(
      FL_METHOD_CODEC(codec), error_code, error_message, details, &error);
  EXPECT_NE(message, nullptr);
  EXPECT_EQ(error, nullptr);

  return message_to_text(message);
}

// Decodes a method call using JsonMethodCodec with a UTF8 string.
static void decode_method_call(const char* text, gchar** name, FlValue** args) {
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  g_autoptr(GBytes) data = text_to_message(text);
  g_autoptr(GError) error = nullptr;
  gboolean result = fl_method_codec_decode_method_call(
      FL_METHOD_CODEC(codec), data, name, args, &error);
  EXPECT_TRUE(result);
  EXPECT_EQ(error, nullptr);
}

// Decodes a method call using JsonMethodCodec. Expect the given error.
static void decode_error_method_call(const char* text,
                                     GQuark domain,
                                     gint code) {
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  g_autoptr(GBytes) data = text_to_message(text);
  g_autoptr(GError) error = nullptr;
  g_autofree gchar* name = nullptr;
  g_autoptr(FlValue) args = nullptr;
  gboolean result = fl_method_codec_decode_method_call(
      FL_METHOD_CODEC(codec), data, &name, &args, &error);
  EXPECT_FALSE(result);
  EXPECT_EQ(name, nullptr);
  EXPECT_EQ(args, nullptr);
  EXPECT_TRUE(g_error_matches(error, domain, code));
}

// Decodes a response using JsonMethodCodec. Expect the response is a result.
static void decode_response_with_success(const char* text, FlValue* result) {
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  g_autoptr(GBytes) message = text_to_message(text);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlMethodResponse) response =
      fl_method_codec_decode_response(FL_METHOD_CODEC(codec), message, &error);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(error, nullptr);
  ASSERT_TRUE(FL_IS_METHOD_SUCCESS_RESPONSE(response));
  EXPECT_TRUE(fl_value_equal(fl_method_success_response_get_result(
                                 FL_METHOD_SUCCESS_RESPONSE(response)),
                             result));
}

// Decodes a response using JsonMethodCodec. Expect the response contains the
// given error.
static void decode_response_with_error(const char* text,
                                       const gchar* code,
                                       const gchar* error_message,
                                       FlValue* details) {
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  g_autoptr(GBytes) message = text_to_message(text);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlMethodResponse) response =
      fl_method_codec_decode_response(FL_METHOD_CODEC(codec), message, &error);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(error, nullptr);
  ASSERT_TRUE(FL_IS_METHOD_ERROR_RESPONSE(response));
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

// Decode a response using JsonMethodCodec. Expect the given error.
static void decode_error_response(const char* text, GQuark domain, gint code) {
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  g_autoptr(GBytes) message = text_to_message(text);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlMethodResponse) response =
      fl_method_codec_decode_response(FL_METHOD_CODEC(codec), message, &error);
  EXPECT_EQ(response, nullptr);
  EXPECT_TRUE(g_error_matches(error, domain, code));
}

TEST(FlJsonMethodCodecTest, EncodeMethodCallNullptrArgs) {
  g_autofree gchar* text = encode_method_call("hello", nullptr);
  EXPECT_STREQ(text, "{\"method\":\"hello\",\"args\":null}");
}

TEST(FlJsonMethodCodecTest, EncodeMethodCallNullArgs) {
  g_autoptr(FlValue) value = fl_value_new_null();
  g_autofree gchar* text = encode_method_call("hello", value);
  EXPECT_STREQ(text, "{\"method\":\"hello\",\"args\":null}");
}

TEST(FlJsonMethodCodecTest, EncodeMethodCallStringArgs) {
  g_autoptr(FlValue) args = fl_value_new_string("world");
  g_autofree gchar* text = encode_method_call("hello", args);
  EXPECT_STREQ(text, "{\"method\":\"hello\",\"args\":\"world\"}");
}

TEST(FlJsonMethodCodecTest, EncodeMethodCallListArgs) {
  g_autoptr(FlValue) args = fl_value_new_list();
  fl_value_append_take(args, fl_value_new_string("count"));
  fl_value_append_take(args, fl_value_new_int(42));
  g_autofree gchar* text = encode_method_call("hello", args);
  EXPECT_STREQ(text, "{\"method\":\"hello\",\"args\":[\"count\",42]}");
}

TEST(FlJsonMethodCodecTest, DecodeMethodCallNoArgs) {
  g_autofree gchar* name = nullptr;
  g_autoptr(FlValue) args = nullptr;
  decode_method_call("{\"method\":\"hello\"}", &name, &args);
  EXPECT_STREQ(name, "hello");
  ASSERT_EQ(args, nullptr);
}

TEST(FlJsonMethodCodecTest, DecodeMethodCallNullArgs) {
  g_autofree gchar* name = nullptr;
  g_autoptr(FlValue) args = nullptr;
  decode_method_call("{\"method\":\"hello\",\"args\":null}", &name, &args);
  EXPECT_STREQ(name, "hello");
  ASSERT_EQ(fl_value_get_type(args), FL_VALUE_TYPE_NULL);
}

TEST(FlJsonMethodCodecTest, DecodeMethodCallStringArgs) {
  g_autofree gchar* name = nullptr;
  g_autoptr(FlValue) args = nullptr;
  decode_method_call("{\"method\":\"hello\",\"args\":\"world\"}", &name, &args);
  EXPECT_STREQ(name, "hello");
  ASSERT_EQ(fl_value_get_type(args), FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(args), "world");
}

TEST(FlJsonMethodCodecTest, DecodeMethodCallListArgs) {
  g_autofree gchar* name = nullptr;
  g_autoptr(FlValue) args = nullptr;
  decode_method_call("{\"method\":\"hello\",\"args\":[\"count\",42]}", &name,
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

TEST(FlJsonMethodCodecTest, DecodeMethodCallNoData) {
  decode_error_method_call("", FL_JSON_MESSAGE_CODEC_ERROR,
                           FL_JSON_MESSAGE_CODEC_ERROR_INVALID_JSON);
}

TEST(FlJsonMethodCodecTest, DecodeMethodCallNoMethodOrArgs) {
  decode_error_method_call("{}", FL_MESSAGE_CODEC_ERROR,
                           FL_MESSAGE_CODEC_ERROR_FAILED);
}

TEST(FlJsonMethodCodecTest, DecodeMethodCallInvalidJson) {
  decode_error_method_call("X", FL_JSON_MESSAGE_CODEC_ERROR,
                           FL_JSON_MESSAGE_CODEC_ERROR_INVALID_JSON);
}

TEST(FlJsonMethodCodecTest, DecodeMethodCallWrongType) {
  decode_error_method_call("42", FL_MESSAGE_CODEC_ERROR,
                           FL_MESSAGE_CODEC_ERROR_FAILED);
}

TEST(FlJsonMethodCodecTest, DecodeMethodCallNoMethod) {
  decode_error_method_call("{\"args\":\"world\"}", FL_MESSAGE_CODEC_ERROR,
                           FL_MESSAGE_CODEC_ERROR_FAILED);
}

TEST(FlJsonMethodCodecTest, DecodeMethodCallNoTerminator) {
  decode_error_method_call("{\"method\":\"hello\",\"args\":\"world\"",
                           FL_JSON_MESSAGE_CODEC_ERROR,
                           FL_JSON_MESSAGE_CODEC_ERROR_INVALID_JSON);
}

TEST(FlJsonMethodCodecTest, DecodeMethodCallExtraData) {
  decode_error_method_call("{\"method\":\"hello\"}XXX",
                           FL_JSON_MESSAGE_CODEC_ERROR,
                           FL_JSON_MESSAGE_CODEC_ERROR_INVALID_JSON);
}

TEST(FlJsonMethodCodecTest, EncodeSuccessEnvelopeNullptr) {
  g_autofree gchar* text = encode_success_envelope(nullptr);
  EXPECT_STREQ(text, "[null]");
}

TEST(FlJsonMethodCodecTest, EncodeSuccessEnvelopeNull) {
  g_autoptr(FlValue) result = fl_value_new_null();
  g_autofree gchar* text = encode_success_envelope(result);
  EXPECT_STREQ(text, "[null]");
}

TEST(FlJsonMethodCodecTest, EncodeSuccessEnvelopeString) {
  g_autoptr(FlValue) result = fl_value_new_string("hello");
  g_autofree gchar* text = encode_success_envelope(result);
  EXPECT_STREQ(text, "[\"hello\"]");
}

TEST(FlJsonMethodCodecTest, EncodeSuccessEnvelopeList) {
  g_autoptr(FlValue) result = fl_value_new_list();
  fl_value_append_take(result, fl_value_new_string("count"));
  fl_value_append_take(result, fl_value_new_int(42));
  g_autofree gchar* text = encode_success_envelope(result);
  EXPECT_STREQ(text, "[[\"count\",42]]");
}

TEST(FlJsonMethodCodecTest, EncodeErrorEnvelopeEmptyCode) {
  g_autofree gchar* text = encode_error_envelope("", nullptr, nullptr);
  EXPECT_STREQ(text, "[\"\",null,null]");
}

TEST(FlJsonMethodCodecTest, EncodeErrorEnvelopeNonMessageOrDetails) {
  g_autofree gchar* text = encode_error_envelope("error", nullptr, nullptr);
  EXPECT_STREQ(text, "[\"error\",null,null]");
}

TEST(FlJsonMethodCodecTest, EncodeErrorEnvelopeMessage) {
  g_autofree gchar* text = encode_error_envelope("error", "message", nullptr);
  EXPECT_STREQ(text, "[\"error\",\"message\",null]");
}

TEST(FlJsonMethodCodecTest, EncodeErrorEnvelopeDetails) {
  g_autoptr(FlValue) details = fl_value_new_list();
  fl_value_append_take(details, fl_value_new_string("count"));
  fl_value_append_take(details, fl_value_new_int(42));
  g_autofree gchar* text = encode_error_envelope("error", nullptr, details);
  EXPECT_STREQ(text, "[\"error\",null,[\"count\",42]]");
}

TEST(FlJsonMethodCodecTest, EncodeErrorEnvelopeMessageAndDetails) {
  g_autoptr(FlValue) details = fl_value_new_list();
  fl_value_append_take(details, fl_value_new_string("count"));
  fl_value_append_take(details, fl_value_new_int(42));
  g_autofree gchar* text = encode_error_envelope("error", "message", details);
  EXPECT_STREQ(text, "[\"error\",\"message\",[\"count\",42]]");
}

TEST(FlJsonMethodCodecTest, DecodeResponseSuccessNull) {
  g_autoptr(FlValue) result = fl_value_new_null();
  decode_response_with_success("[null]", result);
}

TEST(FlJsonMethodCodecTest, DecodeResponseSuccessString) {
  g_autoptr(FlValue) result = fl_value_new_string("hello");
  decode_response_with_success("[\"hello\"]", result);
}

TEST(FlJsonMethodCodecTest, DecodeResponseSuccessList) {
  g_autoptr(FlValue) result = fl_value_new_list();
  fl_value_append_take(result, fl_value_new_string("count"));
  fl_value_append_take(result, fl_value_new_int(42));
  decode_response_with_success("[[\"count\",42]]", result);
}

TEST(FlJsonMethodCodecTest, DecodeResponseErrorEmptyCode) {
  decode_response_with_error("[\"\",null,null]", "", nullptr, nullptr);
}

TEST(FlJsonMethodCodecTest, DecodeResponseErrorNoMessageOrDetails) {
  decode_response_with_error("[\"error\",null,null]", "error", nullptr,
                             nullptr);
}

TEST(FlJsonMethodCodecTest, DecodeResponseErrorMessage) {
  decode_response_with_error("[\"error\",\"message\",null]", "error", "message",
                             nullptr);
}

TEST(FlJsonMethodCodecTest, DecodeResponseErrorDetails) {
  g_autoptr(FlValue) details = fl_value_new_list();
  fl_value_append_take(details, fl_value_new_string("count"));
  fl_value_append_take(details, fl_value_new_int(42));
  decode_response_with_error("[\"error\",null,[\"count\",42]]", "error",
                             nullptr, details);
}

TEST(FlJsonMethodCodecTest, DecodeResponseErrorMessageAndDetails) {
  g_autoptr(FlValue) details = fl_value_new_list();
  fl_value_append_take(details, fl_value_new_string("count"));
  fl_value_append_take(details, fl_value_new_int(42));
  decode_response_with_error("[\"error\",\"message\",[\"count\",42]]", "error",
                             "message", details);
}

TEST(FlJsonMethodCodecTest, DecodeResponseNotImplemented) {
  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  g_autoptr(GBytes) message = g_bytes_new(nullptr, 0);
  g_autoptr(GError) error = nullptr;
  g_autoptr(FlMethodResponse) response =
      fl_method_codec_decode_response(FL_METHOD_CODEC(codec), message, &error);
  ASSERT_NE(response, nullptr);
  EXPECT_EQ(error, nullptr);
  EXPECT_TRUE(FL_IS_METHOD_NOT_IMPLEMENTED_RESPONSE(response));
}

TEST(FlJsonMethodCodecTest, DecodeResponseNoTerminator) {
  decode_error_response("[42", FL_JSON_MESSAGE_CODEC_ERROR,
                        FL_JSON_MESSAGE_CODEC_ERROR_INVALID_JSON);
}

TEST(FlJsonMethodCodecTest, DecodeResponseInvalidJson) {
  decode_error_response("X", FL_JSON_MESSAGE_CODEC_ERROR,
                        FL_JSON_MESSAGE_CODEC_ERROR_INVALID_JSON);
}

TEST(FlJsonMethodCodecTest, DecodeResponseMissingDetails) {
  decode_error_response("[\"error\",\"message\"]", FL_MESSAGE_CODEC_ERROR,
                        FL_MESSAGE_CODEC_ERROR_FAILED);
}

TEST(FlJsonMethodCodecTest, DecodeResponseExtraDetails) {
  decode_error_response("[\"error\",\"message\",true,42]",
                        FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED);
}

TEST(FlJsonMethodCodecTest, DecodeResponseSuccessExtraData) {
  decode_error_response("[null]X", FL_JSON_MESSAGE_CODEC_ERROR,
                        FL_JSON_MESSAGE_CODEC_ERROR_INVALID_JSON);
}

TEST(FlJsonMethodCodecTest, DecodeResponseErrorExtraData) {
  decode_error_response("[\"error\",null,null]X", FL_JSON_MESSAGE_CODEC_ERROR,
                        FL_JSON_MESSAGE_CODEC_ERROR_INVALID_JSON);
}

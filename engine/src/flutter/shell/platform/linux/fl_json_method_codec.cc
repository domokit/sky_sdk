// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_method_codec.h"

#include <gmodule.h>

#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_message_codec.h"

static constexpr char kMethodKey[] = "method";
static constexpr char kArgsKey[] = "args";

struct _FlJsonMethodCodec {
  FlMethodCodec parent_instance;

  FlJsonMessageCodec* codec;
};

G_DEFINE_TYPE(FlJsonMethodCodec,
              fl_json_method_codec,
              fl_method_codec_get_type())

static void fl_json_method_codec_dispose(GObject* object) {
  FlJsonMethodCodec* self = FL_JSON_METHOD_CODEC(object);

  g_clear_object(&self->codec);

  G_OBJECT_CLASS(fl_json_method_codec_parent_class)->dispose(object);
}

// Implements FlMethodCodec::encode_method_call.
static GBytes* fl_json_method_codec_encode_method_call(FlMethodCodec* codec,
                                                       const gchar* name,
                                                       FlValue* args,
                                                       GError** error) {
  FlJsonMethodCodec* self = FL_JSON_METHOD_CODEC(codec);

  g_autoptr(FlValue) message = fl_value_new_map();
  fl_value_set_take(message, fl_value_new_string(kMethodKey),
                    fl_value_new_string(name));
  fl_value_set_take(message, fl_value_new_string(kArgsKey),
                    args != nullptr ? fl_value_ref(args) : fl_value_new_null());

  return fl_message_codec_encode_message(FL_MESSAGE_CODEC(self->codec), message,
                                         error);
}

// Implements FlMethodCodec::decode_method_call.
static gboolean fl_json_method_codec_decode_method_call(FlMethodCodec* codec,
                                                        GBytes* message,
                                                        gchar** name,
                                                        FlValue** args,
                                                        GError** error) {
  FlJsonMethodCodec* self = FL_JSON_METHOD_CODEC(codec);

  g_autoptr(FlValue) value = fl_message_codec_decode_message(
      FL_MESSAGE_CODEC(self->codec), message, error);
  if (value == nullptr) {
    return FALSE;
  }

  if (fl_value_get_type(value) != FL_VALUE_TYPE_MAP) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "Expected JSON map in method response, got %d instead",
                fl_value_get_type(value));
    return FALSE;
  }

  FlValue* method_value = fl_value_lookup_string(value, kMethodKey);
  if (method_value == nullptr) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "Missing JSON method field in method response");
    return FALSE;
  }
  if (fl_value_get_type(method_value) != FL_VALUE_TYPE_STRING) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "Expected JSON string for method name, got %d instead",
                fl_value_get_type(method_value));
    return FALSE;
  }
  FlValue* args_value = fl_value_lookup_string(value, kArgsKey);

  *name = g_strdup(fl_value_get_string(method_value));
  *args = args_value != nullptr ? fl_value_ref(args_value) : nullptr;

  return TRUE;
}

// Implements FlMethodCodec::encode_success_envelope.
static GBytes* fl_json_method_codec_encode_success_envelope(
    FlMethodCodec* codec,
    FlValue* result,
    GError** error) {
  FlJsonMethodCodec* self = FL_JSON_METHOD_CODEC(codec);

  g_autoptr(FlValue) message = fl_value_new_list();
  fl_value_append_take(
      message, result != nullptr ? fl_value_ref(result) : fl_value_new_null());

  return fl_message_codec_encode_message(FL_MESSAGE_CODEC(self->codec), message,
                                         error);
}

// Implements FlMethodCodec::encode_error_envelope.
static GBytes* fl_json_method_codec_encode_error_envelope(
    FlMethodCodec* codec,
    const gchar* code,
    const gchar* error_message,
    FlValue* details,
    GError** error) {
  FlJsonMethodCodec* self = FL_JSON_METHOD_CODEC(codec);

  g_autoptr(FlValue) message = fl_value_new_list();
  fl_value_append_take(message, fl_value_new_string(code));
  fl_value_append_take(message, error_message != nullptr
                                    ? fl_value_new_string(error_message)
                                    : fl_value_new_null());
  fl_value_append_take(message, details != nullptr ? fl_value_ref(details)
                                                   : fl_value_new_null());

  return fl_message_codec_encode_message(FL_MESSAGE_CODEC(self->codec), message,
                                         error);
}

// Implements FlMethodCodec::decode_response.
static FlMethodResponse* fl_json_method_codec_decode_response(
    FlMethodCodec* codec,
    GBytes* message,
    GError** error) {
  FlJsonMethodCodec* self = FL_JSON_METHOD_CODEC(codec);

  g_autoptr(FlValue) value = fl_message_codec_decode_message(
      FL_MESSAGE_CODEC(self->codec), message, error);
  if (value == nullptr) {
    return nullptr;
  }

  if (fl_value_get_type(value) != FL_VALUE_TYPE_LIST) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "Expected JSON list in method response, got %d instead",
                fl_value_get_type(value));
    return nullptr;
  }

  size_t length = fl_value_get_length(value);
  if (length == 1) {
    return FL_METHOD_RESPONSE(
        fl_method_success_response_new(fl_value_get_list_value(value, 0)));
  } else if (length == 3) {
    FlValue* code_value = fl_value_get_list_value(value, 0);
    if (fl_value_get_type(code_value) != FL_VALUE_TYPE_STRING) {
      g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                  "Error code wrong type");
      return nullptr;
    }
    const gchar* code = fl_value_get_string(code_value);

    FlValue* message_value = fl_value_get_list_value(value, 1);
    if (fl_value_get_type(message_value) != FL_VALUE_TYPE_STRING &&
        fl_value_get_type(message_value) != FL_VALUE_TYPE_NULL) {
      g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                  "Error message wrong type");
      return nullptr;
    }
    const gchar* message =
        fl_value_get_type(message_value) == FL_VALUE_TYPE_STRING
            ? fl_value_get_string(message_value)
            : nullptr;

    FlValue* args = fl_value_get_list_value(value, 2);
    if (fl_value_get_type(args) == FL_VALUE_TYPE_NULL) {
      args = nullptr;
    }

    return FL_METHOD_RESPONSE(
        fl_method_error_response_new(code, message, args));
  } else {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "Got response envelope of length %zi, expected 1 (success) or "
                "3 (error)",
                length);
    return nullptr;
  }
}

static void fl_json_method_codec_class_init(FlJsonMethodCodecClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_json_method_codec_dispose;
  FL_METHOD_CODEC_CLASS(klass)->encode_method_call =
      fl_json_method_codec_encode_method_call;
  FL_METHOD_CODEC_CLASS(klass)->decode_method_call =
      fl_json_method_codec_decode_method_call;
  FL_METHOD_CODEC_CLASS(klass)->encode_success_envelope =
      fl_json_method_codec_encode_success_envelope;
  FL_METHOD_CODEC_CLASS(klass)->encode_error_envelope =
      fl_json_method_codec_encode_error_envelope;
  FL_METHOD_CODEC_CLASS(klass)->decode_response =
      fl_json_method_codec_decode_response;
}

static void fl_json_method_codec_init(FlJsonMethodCodec* self) {
  self->codec = fl_json_message_codec_new();
}

G_MODULE_EXPORT FlJsonMethodCodec* fl_json_method_codec_new() {
  return static_cast<FlJsonMethodCodec*>(
      g_object_new(fl_json_method_codec_get_type(), nullptr));
}

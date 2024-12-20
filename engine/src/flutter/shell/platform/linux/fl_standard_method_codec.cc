// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"

#include <gmodule.h>

#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_message_codec.h"

// See lib/src/services/message_codecs.dart in Flutter source for description of
// encoding.

// Envelope codes.
static constexpr guint8 kEnvelopeTypeSuccess = 0;
static constexpr guint8 kEnvelopeTypeError = 1;

struct _FlStandardMethodCodec {
  FlMethodCodec parent_instance;

  FlStandardMessageCodec* message_codec;
};

enum { PROP_MESSAGE_CODEC = 1, PROP_LAST };

G_DEFINE_TYPE(FlStandardMethodCodec,
              fl_standard_method_codec,
              fl_method_codec_get_type())

static void fl_standard_method_codec_set_property(GObject* object,
                                                  guint prop_id,
                                                  const GValue* value,
                                                  GParamSpec* pspec) {
  FlStandardMethodCodec* self = FL_STANDARD_METHOD_CODEC(object);

  switch (prop_id) {
    case PROP_MESSAGE_CODEC:
      g_set_object(&self->message_codec,
                   FL_STANDARD_MESSAGE_CODEC(g_value_get_object(value)));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void fl_standard_method_codec_get_property(GObject* object,
                                                  guint prop_id,
                                                  GValue* value,
                                                  GParamSpec* pspec) {
  FlStandardMethodCodec* self = FL_STANDARD_METHOD_CODEC(object);

  switch (prop_id) {
    case PROP_MESSAGE_CODEC:
      g_value_set_object(value, self->message_codec);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void fl_standard_method_codec_dispose(GObject* object) {
  FlStandardMethodCodec* self = FL_STANDARD_METHOD_CODEC(object);

  g_clear_object(&self->message_codec);

  G_OBJECT_CLASS(fl_standard_method_codec_parent_class)->dispose(object);
}

// Implements FlMethodCodec::encode_method_call.
static GBytes* fl_standard_method_codec_encode_method_call(FlMethodCodec* codec,
                                                           const gchar* name,
                                                           FlValue* args,
                                                           GError** error) {
  FlStandardMethodCodec* self = FL_STANDARD_METHOD_CODEC(codec);

  g_autoptr(GByteArray) buffer = g_byte_array_new();
  g_autoptr(FlValue) name_value = fl_value_new_string(name);
  if (!fl_standard_message_codec_write_value(self->message_codec, buffer,
                                             name_value, error)) {
    return nullptr;
  }
  if (!fl_standard_message_codec_write_value(self->message_codec, buffer, args,
                                             error)) {
    return nullptr;
  }

  return g_byte_array_free_to_bytes(
      static_cast<GByteArray*>(g_steal_pointer(&buffer)));
}

// Implements FlMethodCodec::decode_method_call.
static gboolean fl_standard_method_codec_decode_method_call(
    FlMethodCodec* codec,
    GBytes* message,
    gchar** name,
    FlValue** args,
    GError** error) {
  FlStandardMethodCodec* self = FL_STANDARD_METHOD_CODEC(codec);

  size_t offset = 0;
  g_autoptr(FlValue) name_value = fl_standard_message_codec_read_value(
      self->message_codec, message, &offset, error);
  if (name_value == nullptr) {
    return FALSE;
  }
  if (fl_value_get_type(name_value) != FL_VALUE_TYPE_STRING) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "Method call name wrong type");
    return FALSE;
  }

  g_autoptr(FlValue) args_value = fl_standard_message_codec_read_value(
      self->message_codec, message, &offset, error);
  if (args_value == nullptr) {
    return FALSE;
  }

  if (offset != g_bytes_get_size(message)) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "Unexpected extra data");
    return FALSE;
  }

  *name = g_strdup(fl_value_get_string(name_value));
  *args = fl_value_ref(args_value);

  return TRUE;
}

// Implements FlMethodCodec::encode_success_envelope.
static GBytes* fl_standard_method_codec_encode_success_envelope(
    FlMethodCodec* codec,
    FlValue* result,
    GError** error) {
  FlStandardMethodCodec* self = FL_STANDARD_METHOD_CODEC(codec);

  g_autoptr(GByteArray) buffer = g_byte_array_new();
  guint8 type = kEnvelopeTypeSuccess;
  g_byte_array_append(buffer, &type, 1);
  if (!fl_standard_message_codec_write_value(self->message_codec, buffer,
                                             result, error)) {
    return nullptr;
  }

  return g_byte_array_free_to_bytes(
      static_cast<GByteArray*>(g_steal_pointer(&buffer)));
}

// Implements FlMethodCodec::encode_error_envelope.
static GBytes* fl_standard_method_codec_encode_error_envelope(
    FlMethodCodec* codec,
    const gchar* code,
    const gchar* message,
    FlValue* details,
    GError** error) {
  FlStandardMethodCodec* self = FL_STANDARD_METHOD_CODEC(codec);

  g_autoptr(GByteArray) buffer = g_byte_array_new();
  guint8 type = kEnvelopeTypeError;
  g_byte_array_append(buffer, &type, 1);
  g_autoptr(FlValue) code_value = fl_value_new_string(code);
  if (!fl_standard_message_codec_write_value(self->message_codec, buffer,
                                             code_value, error)) {
    return nullptr;
  }
  g_autoptr(FlValue) message_value =
      message != nullptr ? fl_value_new_string(message) : nullptr;
  if (!fl_standard_message_codec_write_value(self->message_codec, buffer,
                                             message_value, error)) {
    return nullptr;
  }
  if (!fl_standard_message_codec_write_value(self->message_codec, buffer,
                                             details, error)) {
    return nullptr;
  }

  return g_byte_array_free_to_bytes(
      static_cast<GByteArray*>(g_steal_pointer(&buffer)));
}

// Implements FlMethodCodec::encode_decode_response.
static FlMethodResponse* fl_standard_method_codec_decode_response(
    FlMethodCodec* codec,
    GBytes* message,
    GError** error) {
  FlStandardMethodCodec* self = FL_STANDARD_METHOD_CODEC(codec);

  if (g_bytes_get_size(message) == 0) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA, "Empty response");
    return nullptr;
  }

  // First byte is response type.
  const guint8* data =
      static_cast<const guint8*>(g_bytes_get_data(message, nullptr));
  guint8 type = data[0];
  size_t offset = 1;

  g_autoptr(FlMethodResponse) response = nullptr;
  if (type == kEnvelopeTypeError) {
    g_autoptr(FlValue) code = fl_standard_message_codec_read_value(
        self->message_codec, message, &offset, error);
    if (code == nullptr) {
      return nullptr;
    }
    if (fl_value_get_type(code) != FL_VALUE_TYPE_STRING) {
      g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                  "Error code wrong type");
      return nullptr;
    }

    g_autoptr(FlValue) error_message = fl_standard_message_codec_read_value(
        self->message_codec, message, &offset, error);
    if (error_message == nullptr) {
      return nullptr;
    }
    if (fl_value_get_type(error_message) != FL_VALUE_TYPE_STRING &&
        fl_value_get_type(error_message) != FL_VALUE_TYPE_NULL) {
      g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                  "Error message wrong type");
      return nullptr;
    }

    g_autoptr(FlValue) details = fl_standard_message_codec_read_value(
        self->message_codec, message, &offset, error);
    if (details == nullptr) {
      return nullptr;
    }

    response = FL_METHOD_RESPONSE(fl_method_error_response_new(
        fl_value_get_string(code),
        fl_value_get_type(error_message) == FL_VALUE_TYPE_STRING
            ? fl_value_get_string(error_message)
            : nullptr,
        fl_value_get_type(details) != FL_VALUE_TYPE_NULL ? details : nullptr));
  } else if (type == kEnvelopeTypeSuccess) {
    g_autoptr(FlValue) result = fl_standard_message_codec_read_value(
        self->message_codec, message, &offset, error);

    if (result == nullptr) {
      return nullptr;
    }

    response = FL_METHOD_RESPONSE(fl_method_success_response_new(result));
  } else {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "Unknown envelope type %02x", type);
    return nullptr;
  }

  if (offset != g_bytes_get_size(message)) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "Unexpected extra data");
    return nullptr;
  }

  return FL_METHOD_RESPONSE(g_object_ref(response));
}

static void fl_standard_method_codec_class_init(
    FlStandardMethodCodecClass* klass) {
  G_OBJECT_CLASS(klass)->set_property = fl_standard_method_codec_set_property;
  G_OBJECT_CLASS(klass)->get_property = fl_standard_method_codec_get_property;
  G_OBJECT_CLASS(klass)->dispose = fl_standard_method_codec_dispose;

  FL_METHOD_CODEC_CLASS(klass)->encode_method_call =
      fl_standard_method_codec_encode_method_call;
  FL_METHOD_CODEC_CLASS(klass)->decode_method_call =
      fl_standard_method_codec_decode_method_call;
  FL_METHOD_CODEC_CLASS(klass)->encode_success_envelope =
      fl_standard_method_codec_encode_success_envelope;
  FL_METHOD_CODEC_CLASS(klass)->encode_error_envelope =
      fl_standard_method_codec_encode_error_envelope;
  FL_METHOD_CODEC_CLASS(klass)->decode_response =
      fl_standard_method_codec_decode_response;

  g_object_class_install_property(
      G_OBJECT_CLASS(klass), PROP_MESSAGE_CODEC,
      g_param_spec_object(
          "message-codec", "message-codec", "Message codec to use",
          fl_message_codec_get_type(),
          static_cast<GParamFlags>(G_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY |
                                   G_PARAM_STATIC_STRINGS)));
}

static void fl_standard_method_codec_init(FlStandardMethodCodec* self) {}

G_MODULE_EXPORT FlStandardMethodCodec* fl_standard_method_codec_new() {
  g_autoptr(FlStandardMessageCodec) message_codec =
      fl_standard_message_codec_new();
  return fl_standard_method_codec_new_with_message_codec(message_codec);
}

G_MODULE_EXPORT FlStandardMethodCodec*
fl_standard_method_codec_new_with_message_codec(
    FlStandardMessageCodec* message_codec) {
  return FL_STANDARD_METHOD_CODEC(
      g_object_new(fl_standard_method_codec_get_type(), "message-codec",
                   message_codec, nullptr));
}

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_string_codec.h"

#include <gmodule.h>

#include <cstring>

G_DEFINE_QUARK(fl_string_codec_error_quark, fl_string_codec_error)

struct _FlStringCodec {
  FlMessageCodec parent_instance;
};

G_DEFINE_TYPE(FlStringCodec, fl_string_codec, fl_message_codec_get_type())

// Implements FlMessageCodec::encode_message.
static GBytes* fl_string_codec_encode_message(FlMessageCodec* codec,
                                              FlValue* value,
                                              GError** error) {
  if (fl_value_get_type(value) != FL_VALUE_TYPE_STRING) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE,
                "Only string values supported");
    return nullptr;
  }

  const gchar* text = fl_value_get_string(value);
  return g_bytes_new(text, strlen(text));
}

// Implements FlMessageCodec::decode_message.
static FlValue* fl_string_codec_decode_message(FlMessageCodec* codec,
                                               GBytes* message,
                                               GError** error) {
  gsize data_length;
  const gchar* data =
      static_cast<const gchar*>(g_bytes_get_data(message, &data_length));
  return fl_value_new_string_sized(data, data_length);
}

static void fl_string_codec_class_init(FlStringCodecClass* klass) {
  FL_MESSAGE_CODEC_CLASS(klass)->encode_message =
      fl_string_codec_encode_message;
  FL_MESSAGE_CODEC_CLASS(klass)->decode_message =
      fl_string_codec_decode_message;
}

static void fl_string_codec_init(FlStringCodec* self) {}

G_MODULE_EXPORT FlStringCodec* fl_string_codec_new() {
  return static_cast<FlStringCodec*>(
      g_object_new(fl_string_codec_get_type(), nullptr));
}

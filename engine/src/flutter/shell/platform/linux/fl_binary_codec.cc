// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_codec.h"

#include <gmodule.h>

G_DEFINE_QUARK(fl_binary_codec_error_quark, fl_binary_codec_error)

struct _FlBinaryCodec {
  FlMessageCodec parent_instance;
};

G_DEFINE_TYPE(FlBinaryCodec, fl_binary_codec, fl_message_codec_get_type())

// Implements FlMessageCodec::encode_message.
static GBytes* fl_binary_codec_encode_message(FlMessageCodec* codec,
                                              FlValue* value,
                                              GError** error) {
  if (fl_value_get_type(value) != FL_VALUE_TYPE_UINT8_LIST) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE,
                "Only uint8[] values supported");
    return nullptr;
  }

  return g_bytes_new(fl_value_get_uint8_list(value),
                     fl_value_get_length(value));
}

// Implements FlMessageCodec::decode_message.
static FlValue* fl_binary_codec_decode_message(FlMessageCodec* codec,
                                               GBytes* message,
                                               GError** error) {
  gsize data_length;
  const uint8_t* data =
      static_cast<const uint8_t*>(g_bytes_get_data(message, &data_length));
  return fl_value_new_uint8_list(data, data_length);
}

static void fl_binary_codec_class_init(FlBinaryCodecClass* klass) {
  FL_MESSAGE_CODEC_CLASS(klass)->encode_message =
      fl_binary_codec_encode_message;
  FL_MESSAGE_CODEC_CLASS(klass)->decode_message =
      fl_binary_codec_decode_message;
}

static void fl_binary_codec_init(FlBinaryCodec* self) {}

G_MODULE_EXPORT FlBinaryCodec* fl_binary_codec_new() {
  return static_cast<FlBinaryCodec*>(
      g_object_new(fl_binary_codec_get_type(), nullptr));
}

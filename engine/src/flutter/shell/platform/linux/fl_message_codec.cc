// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_message_codec.h"

#include <gmodule.h>

G_DEFINE_QUARK(fl_message_codec_error_quark, fl_message_codec_error)

G_DEFINE_TYPE(FlMessageCodec, fl_message_codec, G_TYPE_OBJECT)

static void fl_message_codec_class_init(FlMessageCodecClass* klass) {}

static void fl_message_codec_init(FlMessageCodec* self) {}

G_MODULE_EXPORT GBytes* fl_message_codec_encode_message(FlMessageCodec* self,
                                                        FlValue* message,
                                                        GError** error) {
  g_return_val_if_fail(FL_IS_MESSAGE_CODEC(self), nullptr);

  // If the user provided NULL, then make a temporary FlValue object for this to
  // make it simpler for the subclasses.
  g_autoptr(FlValue) null_value = nullptr;
  if (message == nullptr) {
    null_value = fl_value_new_null();
    message = null_value;
  }

  return FL_MESSAGE_CODEC_GET_CLASS(self)->encode_message(self, message, error);
}

G_MODULE_EXPORT FlValue* fl_message_codec_decode_message(FlMessageCodec* self,
                                                         GBytes* message,
                                                         GError** error) {
  g_return_val_if_fail(FL_IS_MESSAGE_CODEC(self), nullptr);
  g_return_val_if_fail(message != nullptr, nullptr);

  return FL_MESSAGE_CODEC_GET_CLASS(self)->decode_message(self, message, error);
}

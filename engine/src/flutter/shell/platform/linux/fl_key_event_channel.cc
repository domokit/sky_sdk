// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_key_event_channel.h"

#include "flutter/shell/platform/linux/public/flutter_linux/fl_basic_message_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_message_codec.h"

static constexpr char kChannelName[] = "flutter/keyevent";
static constexpr char kTypeKey[] = "type";
static constexpr char kTypeValueUp[] = "keyup";
static constexpr char kTypeValueDown[] = "keydown";
static constexpr char kKeymapKey[] = "keymap";
static constexpr char kKeyCodeKey[] = "keyCode";
static constexpr char kScanCodeKey[] = "scanCode";
static constexpr char kModifiersKey[] = "modifiers";
static constexpr char kToolkitKey[] = "toolkit";
static constexpr char kSpecifiedLogicalKey[] = "specifiedLogicalKey";
static constexpr char kUnicodeScalarValuesKey[] = "unicodeScalarValues";

static constexpr char kGtkToolkit[] = "gtk";
static constexpr char kLinuxKeymap[] = "linux";

static constexpr int64_t kUnicodeScalarValuesUnset = 0;
static constexpr int64_t kSpecifiedLogicalKeyUnset = 0;

struct _FlKeyEventChannel {
  GObject parent_instance;

  FlBasicMessageChannel* channel;
};

G_DEFINE_TYPE(FlKeyEventChannel, fl_key_event_channel, G_TYPE_OBJECT)

static void fl_key_event_channel_dispose(GObject* object) {
  FlKeyEventChannel* self = FL_KEY_EVENT_CHANNEL(object);

  g_clear_object(&self->channel);

  G_OBJECT_CLASS(fl_key_event_channel_parent_class)->dispose(object);
}

static void fl_key_event_channel_class_init(FlKeyEventChannelClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_key_event_channel_dispose;
}

static void fl_key_event_channel_init(FlKeyEventChannel* self) {}

FlKeyEventChannel* fl_key_event_channel_new(FlBinaryMessenger* messenger) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(messenger), nullptr);

  FlKeyEventChannel* self = FL_KEY_EVENT_CHANNEL(
      g_object_new(fl_key_event_channel_get_type(), nullptr));

  g_autoptr(FlJsonMessageCodec) codec = fl_json_message_codec_new();
  self->channel = fl_basic_message_channel_new(messenger, kChannelName,
                                               FL_MESSAGE_CODEC(codec));

  return self;
}

void fl_key_event_channel_send(FlKeyEventChannel* self,
                               FlKeyEventType type,
                               int64_t scan_code,
                               int64_t key_code,
                               int64_t modifiers,
                               int64_t unicode_scalar_values,
                               int64_t specified_logical_key,
                               GCancellable* cancellable,
                               GAsyncReadyCallback callback,
                               gpointer user_data) {
  g_return_if_fail(FL_IS_KEY_EVENT_CHANNEL(self));

  const gchar* type_string;
  switch (type) {
    case FL_KEY_EVENT_TYPE_KEYUP:
      type_string = kTypeValueUp;
      break;
    case FL_KEY_EVENT_TYPE_KEYDOWN:
      type_string = kTypeValueDown;
      break;
    default:
      g_assert_not_reached();
  }

  g_autoptr(FlValue) message = fl_value_new_map();
  fl_value_set_string_take(message, kTypeKey, fl_value_new_string(type_string));
  fl_value_set_string_take(message, kKeymapKey,
                           fl_value_new_string(kLinuxKeymap));
  fl_value_set_string_take(message, kScanCodeKey, fl_value_new_int(scan_code));
  fl_value_set_string_take(message, kToolkitKey,
                           fl_value_new_string(kGtkToolkit));
  fl_value_set_string_take(message, kKeyCodeKey, fl_value_new_int(key_code));
  fl_value_set_string_take(message, kModifiersKey, fl_value_new_int(modifiers));
  if (unicode_scalar_values != kUnicodeScalarValuesUnset) {
    fl_value_set_string_take(message, kUnicodeScalarValuesKey,
                             fl_value_new_int(unicode_scalar_values));
  }
  if (specified_logical_key != kSpecifiedLogicalKeyUnset) {
    fl_value_set_string_take(message, kSpecifiedLogicalKey,
                             fl_value_new_int(specified_logical_key));
  }
  fl_basic_message_channel_send(self->channel, message, cancellable, callback,
                                user_data);
}

gboolean fl_key_event_channel_send_finish(GObject* object,
                                          GAsyncResult* result,
                                          gboolean* handled,
                                          GError** error) {
  g_autoptr(FlValue) message = fl_basic_message_channel_send_finish(
      FL_BASIC_MESSAGE_CHANNEL(object), result, error);
  if (message == nullptr) {
    return FALSE;
  }

  FlValue* handled_value = fl_value_lookup_string(message, "handled");
  *handled = fl_value_get_bool(handled_value);

  return TRUE;
}

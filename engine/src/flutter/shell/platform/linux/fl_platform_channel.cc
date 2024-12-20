// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_platform_channel.h"

#include <cstring>

#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_method_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"

static constexpr char kChannelName[] = "flutter/platform";
static constexpr char kBadArgumentsError[] = "Bad Arguments";
static constexpr char kGetClipboardDataMethod[] = "Clipboard.getData";
static constexpr char kSetClipboardDataMethod[] = "Clipboard.setData";
static constexpr char kClipboardHasStringsMethod[] = "Clipboard.hasStrings";
static constexpr char kExitApplicationMethod[] = "System.exitApplication";
static constexpr char kRequestAppExitMethod[] = "System.requestAppExit";
static constexpr char kInitializationCompleteMethod[] =
    "System.initializationComplete";
static constexpr char kPlaySoundMethod[] = "SystemSound.play";
static constexpr char kSystemNavigatorPopMethod[] = "SystemNavigator.pop";
static constexpr char kTextKey[] = "text";
static constexpr char kValueKey[] = "value";

static constexpr char kExitTypeKey[] = "type";
static constexpr char kExitTypeCancelable[] = "cancelable";
static constexpr char kExitTypeRequired[] = "required";

static constexpr char kExitResponseKey[] = "response";
static constexpr char kExitResponseCancel[] = "cancel";
static constexpr char kExitResponseExit[] = "exit";

struct _FlPlatformChannel {
  GObject parent_instance;

  FlMethodChannel* channel;

  // Handlers for incoming method calls.
  FlPlatformChannelVTable* vtable;

  // User data to pass to method call handlers.
  gpointer user_data;
};

G_DEFINE_TYPE(FlPlatformChannel, fl_platform_channel, G_TYPE_OBJECT)

static FlMethodResponse* clipboard_set_data(FlPlatformChannel* self,
                                            FlMethodCall* method_call) {
  FlValue* args = fl_method_call_get_args(method_call);

  if (fl_value_get_type(args) != FL_VALUE_TYPE_MAP) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Argument map missing or malformed", nullptr));
  }

  FlValue* text_value = fl_value_lookup_string(args, kTextKey);
  if (text_value == nullptr ||
      fl_value_get_type(text_value) != FL_VALUE_TYPE_STRING) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Missing clipboard text", nullptr));
  }
  const gchar* text = fl_value_get_string(text_value);

  return self->vtable->clipboard_set_data(method_call, text, self->user_data);
}

static FlMethodResponse* clipboard_get_data(FlPlatformChannel* self,
                                            FlMethodCall* method_call) {
  FlValue* args = fl_method_call_get_args(method_call);

  if (fl_value_get_type(args) != FL_VALUE_TYPE_STRING) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Expected string", nullptr));
  }
  const gchar* format = fl_value_get_string(args);

  return self->vtable->clipboard_get_data(method_call, format, self->user_data);
}

static FlMethodResponse* clipboard_has_strings(FlPlatformChannel* self,
                                               FlMethodCall* method_call) {
  return self->vtable->clipboard_has_strings(method_call, self->user_data);
}

// Get the exit response from a System.requestAppExit method call.
FlPlatformChannelExitResponse get_exit_response(FlMethodResponse* response) {
  if (response == nullptr) {
    return FL_PLATFORM_CHANNEL_EXIT_RESPONSE_EXIT;
  }

  g_autoptr(GError) error = nullptr;
  FlValue* result = fl_method_response_get_result(response, &error);
  if (result == nullptr) {
    g_warning("Error returned from System.requestAppExit: %s", error->message);
    return FL_PLATFORM_CHANNEL_EXIT_RESPONSE_EXIT;
  }
  if (fl_value_get_type(result) != FL_VALUE_TYPE_MAP) {
    g_warning("System.requestAppExit result argument map missing or malformed");
    return FL_PLATFORM_CHANNEL_EXIT_RESPONSE_EXIT;
  }

  FlValue* response_value = fl_value_lookup_string(result, kExitResponseKey);
  if (fl_value_get_type(response_value) != FL_VALUE_TYPE_STRING) {
    g_warning("Invalid response from System.requestAppExit");
    return FL_PLATFORM_CHANNEL_EXIT_RESPONSE_EXIT;
  }
  const char* response_string = fl_value_get_string(response_value);

  if (strcmp(response_string, kExitResponseCancel) == 0) {
    return FL_PLATFORM_CHANNEL_EXIT_RESPONSE_CANCEL;
  } else if (strcmp(response_string, kExitResponseExit) == 0) {
    return FL_PLATFORM_CHANNEL_EXIT_RESPONSE_EXIT;
  }

  // If something went wrong, then just exit.
  return FL_PLATFORM_CHANNEL_EXIT_RESPONSE_EXIT;
}

static FlMethodResponse* system_exit_application(FlPlatformChannel* self,
                                                 FlMethodCall* method_call) {
  FlValue* args = fl_method_call_get_args(method_call);
  if (fl_value_get_type(args) != FL_VALUE_TYPE_MAP) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Argument map missing or malformed", nullptr));
  }

  FlValue* type_value = fl_value_lookup_string(args, kExitTypeKey);
  if (type_value == nullptr ||
      fl_value_get_type(type_value) != FL_VALUE_TYPE_STRING) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Missing type argument", nullptr));
  }
  const char* type_string = fl_value_get_string(type_value);
  FlPlatformChannelExitType type;
  if (strcmp(type_string, kExitTypeCancelable) == 0) {
    type = FL_PLATFORM_CHANNEL_EXIT_TYPE_CANCELABLE;
  } else if (strcmp(type_string, kExitTypeRequired) == 0) {
    type = FL_PLATFORM_CHANNEL_EXIT_TYPE_REQUIRED;
  } else {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Invalid exit type", nullptr));
  }

  return self->vtable->system_exit_application(method_call, type,
                                               self->user_data);
}

static FlMethodResponse* system_initialization_complete(
    FlPlatformChannel* self) {
  self->vtable->system_initialization_complete(self->user_data);
  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

static FlMethodResponse* system_sound_play(FlPlatformChannel* self,
                                           FlValue* args) {
  if (fl_value_get_type(args) != FL_VALUE_TYPE_STRING) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Expected string", nullptr));
  }
  const gchar* type = fl_value_get_string(args);

  self->vtable->system_sound_play(type, self->user_data);

  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

static FlMethodResponse* system_navigator_pop(FlPlatformChannel* self) {
  self->vtable->system_navigator_pop(self->user_data);
  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

static void method_call_cb(FlMethodChannel* channel,
                           FlMethodCall* method_call,
                           gpointer user_data) {
  FlPlatformChannel* self = FL_PLATFORM_CHANNEL(user_data);

  const gchar* method = fl_method_call_get_name(method_call);
  FlValue* args = fl_method_call_get_args(method_call);

  g_autoptr(FlMethodResponse) response = nullptr;
  if (strcmp(method, kSetClipboardDataMethod) == 0) {
    response = clipboard_set_data(self, method_call);
  } else if (strcmp(method, kGetClipboardDataMethod) == 0) {
    response = clipboard_get_data(self, method_call);
  } else if (strcmp(method, kClipboardHasStringsMethod) == 0) {
    response = clipboard_has_strings(self, method_call);
  } else if (strcmp(method, kExitApplicationMethod) == 0) {
    response = system_exit_application(self, method_call);
  } else if (strcmp(method, kInitializationCompleteMethod) == 0) {
    response = system_initialization_complete(self);
  } else if (strcmp(method, kPlaySoundMethod) == 0) {
    response = system_sound_play(self, args);
  } else if (strcmp(method, kSystemNavigatorPopMethod) == 0) {
    response = system_navigator_pop(self);
  } else {
    response = FL_METHOD_RESPONSE(fl_method_not_implemented_response_new());
  }

  if (response != nullptr) {
    g_autoptr(GError) error = nullptr;
    if (!fl_method_call_respond(method_call, response, &error)) {
      g_warning("Failed to send method call response: %s", error->message);
    }
  }
}

static void fl_platform_channel_dispose(GObject* object) {
  FlPlatformChannel* self = FL_PLATFORM_CHANNEL(object);

  g_clear_object(&self->channel);

  G_OBJECT_CLASS(fl_platform_channel_parent_class)->dispose(object);
}

static void fl_platform_channel_class_init(FlPlatformChannelClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_platform_channel_dispose;
}

static void fl_platform_channel_init(FlPlatformChannel* self) {}

FlPlatformChannel* fl_platform_channel_new(FlBinaryMessenger* messenger,
                                           FlPlatformChannelVTable* vtable,
                                           gpointer user_data) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(messenger), nullptr);
  g_return_val_if_fail(vtable != nullptr, nullptr);

  FlPlatformChannel* self = FL_PLATFORM_CHANNEL(
      g_object_new(fl_platform_channel_get_type(), nullptr));

  self->vtable = vtable;
  self->user_data = user_data;

  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  self->channel =
      fl_method_channel_new(messenger, kChannelName, FL_METHOD_CODEC(codec));
  fl_method_channel_set_method_call_handler(self->channel, method_call_cb, self,
                                            nullptr);

  return self;
}

void fl_platform_channel_system_request_app_exit(FlPlatformChannel* self,
                                                 FlPlatformChannelExitType type,
                                                 GCancellable* cancellable,
                                                 GAsyncReadyCallback callback,
                                                 gpointer user_data) {
  g_return_if_fail(FL_IS_PLATFORM_CHANNEL(self));

  g_autoptr(FlValue) args = fl_value_new_map();
  const gchar* type_string;
  switch (type) {
    case FL_PLATFORM_CHANNEL_EXIT_TYPE_CANCELABLE:
      type_string = kExitTypeCancelable;
      break;
    case FL_PLATFORM_CHANNEL_EXIT_TYPE_REQUIRED:
      type_string = kExitTypeRequired;
      break;
    default:
      g_assert_not_reached();
  }
  fl_value_set_string_take(args, kExitTypeKey,
                           fl_value_new_string(type_string));
  fl_method_channel_invoke_method(self->channel, kRequestAppExitMethod, args,
                                  cancellable, callback, user_data);
}

gboolean fl_platform_channel_system_request_app_exit_finish(
    GObject* object,
    GAsyncResult* result,
    FlPlatformChannelExitResponse* exit_response,
    GError** error) {
  g_autoptr(FlMethodResponse) response = fl_method_channel_invoke_method_finish(
      FL_METHOD_CHANNEL(object), result, error);
  if (response == nullptr) {
    return FALSE;
  }

  *exit_response = get_exit_response(response);

  return TRUE;
}

void fl_platform_channel_respond_clipboard_get_data(FlMethodCall* method_call,
                                                    const gchar* text) {
  g_autoptr(FlValue) result = nullptr;
  if (text != nullptr) {
    result = fl_value_new_map();
    fl_value_set_string_take(result, kTextKey, fl_value_new_string(text));
  }

  g_autoptr(FlMethodResponse) response =
      FL_METHOD_RESPONSE(fl_method_success_response_new(result));

  g_autoptr(GError) error = nullptr;
  if (!fl_method_call_respond(method_call, response, &error)) {
    g_warning("Failed to send response to %s: %s", kGetClipboardDataMethod,
              error->message);
  }
}

void fl_platform_channel_respond_clipboard_has_strings(
    FlMethodCall* method_call,
    gboolean has_strings) {
  g_autoptr(FlValue) result = fl_value_new_map();
  fl_value_set_string_take(result, kValueKey, fl_value_new_bool(has_strings));

  g_autoptr(FlMethodResponse) response =
      FL_METHOD_RESPONSE(fl_method_success_response_new(result));

  g_autoptr(GError) error = nullptr;
  if (!fl_method_call_respond(method_call, response, &error)) {
    g_warning("Failed to send response to %s: %s", kClipboardHasStringsMethod,
              error->message);
  }
}

void fl_platform_channel_respond_system_exit_application(
    FlMethodCall* method_call,
    FlPlatformChannelExitResponse exit_response) {
  g_autoptr(FlMethodResponse) response =
      fl_platform_channel_make_system_request_app_exit_response(exit_response);
  g_autoptr(GError) error = nullptr;
  if (!fl_method_call_respond(method_call, response, &error)) {
    g_warning("Failed to send response to System.exitApplication: %s",
              error->message);
  }
}

FlMethodResponse* fl_platform_channel_make_system_request_app_exit_response(
    FlPlatformChannelExitResponse exit_response) {
  g_autoptr(FlValue) exit_result = fl_value_new_map();
  const gchar* exit_response_string;
  switch (exit_response) {
    case FL_PLATFORM_CHANNEL_EXIT_RESPONSE_CANCEL:
      exit_response_string = kExitResponseCancel;
      break;
    case FL_PLATFORM_CHANNEL_EXIT_RESPONSE_EXIT:
      exit_response_string = kExitResponseExit;
      break;
    default:
      g_assert_not_reached();
  }
  fl_value_set_string_take(exit_result, kExitResponseKey,
                           fl_value_new_string(exit_response_string));
  return FL_METHOD_RESPONSE(fl_method_success_response_new(exit_result));
}

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_windowing_channel.h"

#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_method_codec.h"

static constexpr char kChannelName[] = "flutter/windowing";
static constexpr char kBadArgumentsError[] = "Bad Arguments";

static constexpr char kCreateRegularMethod[] = "createRegular";
static constexpr char kModifyRegularMethod[] = "modifyRegular";
static constexpr char kDestroyWindowMethod[] = "destroyWindow";

static constexpr char kSizeKey[] = "size";
static constexpr char kMinSizeKey[] = "minSize";
static constexpr char kMaxSizeKey[] = "maxSize";
static constexpr char kTitleKey[] = "title";
static constexpr char kStateKey[] = "state";
static constexpr char kViewIdKey[] = "viewId";

struct _FlWindowingChannel {
  GObject parent_instance;

  FlMethodChannel* channel;

  // Handlers for incoming method calls.
  FlWindowingChannelVTable* vtable;

  // User data to pass to method call handlers.
  gpointer user_data;
};

G_DEFINE_TYPE(FlWindowingChannel, fl_windowing_channel, G_TYPE_OBJECT)

// Returns TRUE if [args] is a valid size argument.
static gboolean is_valid_size_argument(FlValue* value) {
  return fl_value_get_type(value) == FL_VALUE_TYPE_LIST &&
         fl_value_get_length(value) == 2 &&
         fl_value_get_type(fl_value_get_list_value(value, 0)) ==
             FL_VALUE_TYPE_FLOAT &&
         fl_value_get_type(fl_value_get_list_value(value, 1)) ==
             FL_VALUE_TYPE_FLOAT;
}

static FlWindowingSize* parse_size_value(FlValue* value) {
  FlWindowingSize* size = g_new0(FlWindowingSize, 1);
  size->width = fl_value_get_float(fl_value_get_list_value(value, 0));
  size->height = fl_value_get_float(fl_value_get_list_value(value, 1));
  return size;
}

static gboolean parse_window_state_value(FlValue* value, FlWindowState* state) {
  if (fl_value_get_type(value) != FL_VALUE_TYPE_STRING) {
    return FALSE;
  }

  const gchar* text = fl_value_get_string(value);
  if (strcmp(text, "WindowState.restored") == 0) {
    *state = FL_WINDOW_STATE_RESTORED;
    return TRUE;
  } else if (strcmp(text, "WindowState.maximized") == 0) {
    *state = FL_WINDOW_STATE_MAXIMIZED;
    return TRUE;
  } else if (strcmp(text, "WindowState.minimized") == 0) {
    *state = FL_WINDOW_STATE_MINIMIZED;
    return TRUE;
  }

  return FALSE;
}

static const gchar* window_state_to_string(FlWindowState state) {
  switch (state) {
    case FL_WINDOW_STATE_UNDEFINED:
      return nullptr;
    case FL_WINDOW_STATE_RESTORED:
      return "WindowState.restored";
    case FL_WINDOW_STATE_MAXIMIZED:
      return "WindowState.maximized";
    case FL_WINDOW_STATE_MINIMIZED:
      return "WindowState.minimized";
  }

  return nullptr;
}

// Called when a regular window should be created.
static FlMethodResponse* create_regular(FlWindowingChannel* self,
                                        FlValue* args) {
  if (fl_value_get_type(args) != FL_VALUE_TYPE_MAP) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Argument map missing or malformed", nullptr));
  }

  FlValue* size_value = fl_value_lookup_string(args, kSizeKey);
  if (size_value == nullptr || !is_valid_size_argument(size_value)) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Missing/invalid size argument", nullptr));
  }
  g_autofree FlWindowingSize* size = parse_size_value(size_value);

  FlValue* min_size_value = fl_value_lookup_string(args, kMinSizeKey);
  g_autofree FlWindowingSize* min_size = nullptr;
  if (min_size_value != nullptr) {
    if (!is_valid_size_argument(min_size_value)) {
      return FL_METHOD_RESPONSE(fl_method_error_response_new(
          kBadArgumentsError, "Invalid minSize argument", nullptr));
    }
    min_size = parse_size_value(min_size_value);
  }

  FlValue* max_size_value = fl_value_lookup_string(args, kMaxSizeKey);
  g_autofree FlWindowingSize* max_size = nullptr;
  if (max_size_value != nullptr) {
    if (!is_valid_size_argument(max_size_value)) {
      return FL_METHOD_RESPONSE(fl_method_error_response_new(
          kBadArgumentsError, "Invalid maxSize argument", nullptr));
    }
    max_size = parse_size_value(max_size_value);
  }

  FlValue* title_value = fl_value_lookup_string(args, kTitleKey);
  const gchar* title = nullptr;
  if (title_value != nullptr) {
    if (fl_value_get_type(title_value) != FL_VALUE_TYPE_STRING) {
      return FL_METHOD_RESPONSE(fl_method_error_response_new(
          kBadArgumentsError, "Invalid title argument", nullptr));
    }
    title = fl_value_get_string(title_value);
  }
  FlWindowState state = FL_WINDOW_STATE_UNDEFINED;
  FlValue* state_value = fl_value_lookup_string(args, kStateKey);
  if (state_value != nullptr) {
    if (!parse_window_state_value(state_value, &state)) {
      return FL_METHOD_RESPONSE(fl_method_error_response_new(
          kBadArgumentsError, "Invalid state argument", nullptr));
    }
  }

  return self->vtable->create_regular(size, min_size, max_size, title, state,
                                      self->user_data);
}

// Called when a regular window should be created.
static FlMethodResponse* modify_regular(FlWindowingChannel* self,
                                        FlValue* args) {
  if (fl_value_get_type(args) != FL_VALUE_TYPE_MAP) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Argument map missing or malformed", nullptr));
  }

  FlValue* view_id_value = fl_value_lookup_string(args, kViewIdKey);
  if (view_id_value == nullptr ||
      fl_value_get_type(view_id_value) != FL_VALUE_TYPE_INT) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Missing/invalid viewId argument", nullptr));
  }
  int64_t view_id = fl_value_get_int(view_id_value);

  g_autofree FlWindowingSize* size = nullptr;
  FlValue* size_value = fl_value_lookup_string(args, kSizeKey);
  if (size_value != nullptr) {
    if (!is_valid_size_argument(size_value)) {
      return FL_METHOD_RESPONSE(fl_method_error_response_new(
          kBadArgumentsError, "Invalid size argument", nullptr));
    }
    size = parse_size_value(size_value);
  }
  FlValue* title_value = fl_value_lookup_string(args, kTitleKey);
  const gchar* title = nullptr;
  if (title_value != nullptr) {
    if (fl_value_get_type(title_value) != FL_VALUE_TYPE_STRING) {
      return FL_METHOD_RESPONSE(fl_method_error_response_new(
          kBadArgumentsError, "Invalid title argument", nullptr));
    }
    title = fl_value_get_string(title_value);
  }
  FlWindowState state = FL_WINDOW_STATE_UNDEFINED;
  FlValue* state_value = fl_value_lookup_string(args, kStateKey);
  if (state_value != nullptr) {
    if (!parse_window_state_value(state_value, &state)) {
      return FL_METHOD_RESPONSE(fl_method_error_response_new(
          kBadArgumentsError, "Invalid state argument", nullptr));
    }
  }

  return self->vtable->modify_regular(view_id, size, title, state,
                                      self->user_data);
}

// Called when a window should be destroyed.
static FlMethodResponse* destroy_window(FlWindowingChannel* self,
                                        FlValue* args) {
  if (fl_value_get_type(args) != FL_VALUE_TYPE_MAP) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Argument map missing or malformed", nullptr));
  }

  FlValue* view_id_value = fl_value_lookup_string(args, kViewIdKey);
  if (view_id_value == nullptr ||
      fl_value_get_type(view_id_value) != FL_VALUE_TYPE_INT) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Missing/invalid viewId argument", nullptr));
  }
  int64_t view_id = fl_value_get_int(view_id_value);

  return self->vtable->destroy_window(view_id, self->user_data);
}

// Called when a method call is received from Flutter.
static void method_call_cb(FlMethodChannel* channel,
                           FlMethodCall* method_call,
                           gpointer user_data) {
  FlWindowingChannel* self = FL_WINDOWING_CHANNEL(user_data);

  const gchar* method = fl_method_call_get_name(method_call);
  FlValue* args = fl_method_call_get_args(method_call);
  g_autoptr(FlMethodResponse) response = nullptr;

  if (strcmp(method, kCreateRegularMethod) == 0) {
    response = create_regular(self, args);
  } else if (strcmp(method, kModifyRegularMethod) == 0) {
    response = modify_regular(self, args);
  } else if (strcmp(method, kDestroyWindowMethod) == 0) {
    response = destroy_window(self, args);
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

static void fl_windowing_channel_dispose(GObject* object) {
  FlWindowingChannel* self = FL_WINDOWING_CHANNEL(object);

  g_clear_object(&self->channel);

  G_OBJECT_CLASS(fl_windowing_channel_parent_class)->dispose(object);
}

static void fl_windowing_channel_class_init(FlWindowingChannelClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_windowing_channel_dispose;
}

static void fl_windowing_channel_init(FlWindowingChannel* self) {}

FlWindowingChannel* fl_windowing_channel_new(FlBinaryMessenger* messenger,
                                             FlWindowingChannelVTable* vtable,
                                             gpointer user_data) {
  FlWindowingChannel* self = FL_WINDOWING_CHANNEL(
      g_object_new(fl_windowing_channel_get_type(), nullptr));

  self->vtable = vtable;
  self->user_data = user_data;

  g_autoptr(FlStandardMethodCodec) codec = fl_standard_method_codec_new();
  self->channel =
      fl_method_channel_new(messenger, kChannelName, FL_METHOD_CODEC(codec));
  fl_method_channel_set_method_call_handler(self->channel, method_call_cb, self,
                                            nullptr);

  return self;
}

FlMethodResponse* fl_windowing_channel_make_create_regular_response(
    int64_t view_id,
    FlWindowingSize* size,
    FlWindowState state) {
  g_autoptr(FlValue) result = fl_value_new_map();
  fl_value_set_string_take(result, kViewIdKey, fl_value_new_int(view_id));
  g_autoptr(FlValue) size_value = fl_value_new_list();
  fl_value_append_take(size_value, fl_value_new_float(size->width));
  fl_value_append_take(size_value, fl_value_new_float(size->height));
  fl_value_set_string(result, kSizeKey, size_value);
  fl_value_set_string_take(result, kStateKey,
                           fl_value_new_string(window_state_to_string(state)));
  return FL_METHOD_RESPONSE(fl_method_success_response_new(result));
}

FlMethodResponse* fl_windowing_channel_make_modify_regular_response() {
  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

FlMethodResponse* fl_windowing_channel_make_destroy_window_response() {
  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

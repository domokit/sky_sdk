// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_text_input_channel.h"

#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_method_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_channel.h"

static constexpr char kChannelName[] = "flutter/textinput";

static constexpr char kBadArgumentsError[] = "Bad Arguments";

static constexpr char kSetClientMethod[] = "TextInput.setClient";
static constexpr char kShowMethod[] = "TextInput.show";
static constexpr char kSetEditingStateMethod[] = "TextInput.setEditingState";
static constexpr char kClearClientMethod[] = "TextInput.clearClient";
static constexpr char kHideMethod[] = "TextInput.hide";
static constexpr char kUpdateEditingStateMethod[] =
    "TextInputClient.updateEditingState";
static constexpr char kUpdateEditingStateWithDeltasMethod[] =
    "TextInputClient.updateEditingStateWithDeltas";
static constexpr char kPerformActionMethod[] = "TextInputClient.performAction";
static constexpr char kSetEditableSizeAndTransform[] =
    "TextInput.setEditableSizeAndTransform";
static constexpr char kSetMarkedTextRect[] = "TextInput.setMarkedTextRect";

static constexpr char kInputActionKey[] = "inputAction";
static constexpr char kTextInputTypeKey[] = "inputType";
static constexpr char kEnableDeltaModel[] = "enableDeltaModel";
static constexpr char kTextInputTypeNameKey[] = "name";
static constexpr char kTextKey[] = "text";
static constexpr char kSelectionBaseKey[] = "selectionBase";
static constexpr char kSelectionExtentKey[] = "selectionExtent";
static constexpr char kSelectionAffinityKey[] = "selectionAffinity";
static constexpr char kSelectionIsDirectionalKey[] = "selectionIsDirectional";
static constexpr char kComposingBaseKey[] = "composingBase";
static constexpr char kComposingExtentKey[] = "composingExtent";

static constexpr char kTransform[] = "transform";

static constexpr char kMultilineInputType[] = "TextInputType.multiline";
static constexpr char kNoneInputType[] = "TextInputType.none";

static constexpr char kTextAffinityUpstream[] = "TextAffinity.upstream";
static constexpr char kTextAffinityDownstream[] = "TextAffinity.downstream";

struct _FlTextInputChannel {
  GObject parent_instance;

  FlMethodChannel* channel;

  FlTextInputChannelVTable* vtable;

  gpointer user_data;
};

G_DEFINE_TYPE(FlTextInputChannel, fl_text_input_channel, G_TYPE_OBJECT)

static const gchar* text_affinity_to_string(FlTextAffinity affinity) {
  switch (affinity) {
    case FL_TEXT_AFFINITY_UPSTREAM:
      return kTextAffinityUpstream;
    case FL_TEXT_AFFINITY_DOWNSTREAM:
      return kTextAffinityDownstream;
    default:
      g_assert_not_reached();
  }
}

// Called when the input method client is set up.
static FlMethodResponse* set_client(FlTextInputChannel* self, FlValue* args) {
  if (fl_value_get_type(args) != FL_VALUE_TYPE_LIST ||
      fl_value_get_length(args) < 2) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Expected 2-element list", nullptr));
  }

  int64_t client_id = fl_value_get_int(fl_value_get_list_value(args, 0));
  FlValue* config_value = fl_value_get_list_value(args, 1);
  const gchar* input_action = nullptr;
  FlValue* input_action_value =
      fl_value_lookup_string(config_value, kInputActionKey);
  if (fl_value_get_type(input_action_value) == FL_VALUE_TYPE_STRING) {
    input_action = fl_value_get_string(input_action_value);
  }

  FlValue* enable_delta_model_value =
      fl_value_lookup_string(config_value, kEnableDeltaModel);
  gboolean enable_delta_model = fl_value_get_bool(enable_delta_model_value);

  // Reset the input type, then set only if appropriate.
  FlTextInputType input_type = FL_TEXT_INPUT_TYPE_TEXT;
  FlValue* input_type_value =
      fl_value_lookup_string(config_value, kTextInputTypeKey);
  if (fl_value_get_type(input_type_value) == FL_VALUE_TYPE_MAP) {
    FlValue* input_type_name_value =
        fl_value_lookup_string(input_type_value, kTextInputTypeNameKey);
    if (fl_value_get_type(input_type_name_value) == FL_VALUE_TYPE_STRING) {
      const gchar* input_type_name = fl_value_get_string(input_type_name_value);
      if (g_strcmp0(input_type_name, kMultilineInputType) == 0) {
        input_type = FL_TEXT_INPUT_TYPE_MULTILINE;
      } else if (g_strcmp0(input_type_name, kNoneInputType) == 0) {
        input_type = FL_TEXT_INPUT_TYPE_NONE;
      }
    }
  }

  self->vtable->set_client(client_id, input_action, enable_delta_model,
                           input_type, self->user_data);

  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

// Hides the input method.
static FlMethodResponse* hide(FlTextInputChannel* self) {
  self->vtable->hide(self->user_data);
  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

// Shows the input method.
static FlMethodResponse* show(FlTextInputChannel* self) {
  self->vtable->show(self->user_data);
  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

// Updates the editing state from Flutter.
static FlMethodResponse* set_editing_state(FlTextInputChannel* self,
                                           FlValue* args) {
  const gchar* text =
      fl_value_get_string(fl_value_lookup_string(args, kTextKey));
  int64_t selection_base =
      fl_value_get_int(fl_value_lookup_string(args, kSelectionBaseKey));
  int64_t selection_extent =
      fl_value_get_int(fl_value_lookup_string(args, kSelectionExtentKey));
  int64_t composing_base =
      fl_value_get_int(fl_value_lookup_string(args, kComposingBaseKey));
  int64_t composing_extent =
      fl_value_get_int(fl_value_lookup_string(args, kComposingExtentKey));

  self->vtable->set_editing_state(text, selection_base, selection_extent,
                                  composing_base, composing_extent,
                                  self->user_data);

  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

// Called when the input method client is complete.
static FlMethodResponse* clear_client(FlTextInputChannel* self) {
  self->vtable->clear_client(self->user_data);
  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

// Handles updates to the EditableText size and position from the framework.
//
// On changes to the size or position of the RenderObject underlying the
// EditableText, this update may be triggered. It provides an updated size and
// transform from the local coordinate system of the EditableText to root
// Flutter coordinate system.
static FlMethodResponse* set_editable_size_and_transform(
    FlTextInputChannel* self,
    FlValue* args) {
  FlValue* transform_value = fl_value_lookup_string(args, kTransform);
  if (fl_value_get_length(transform_value) != 16) {
    return FL_METHOD_RESPONSE(fl_method_error_response_new(
        kBadArgumentsError, "Invalid transform", nullptr));
  }

  double transform[16];
  for (size_t i = 0; i < 16; i++) {
    transform[i] =
        fl_value_get_float(fl_value_get_list_value(transform_value, i));
  }
  self->vtable->set_editable_size_and_transform(transform, self->user_data);

  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

// Handles updates to the composing rect from the framework.
//
// On changes to the state of the EditableText in the framework, this update
// may be triggered. It provides an updated rect for the composing region in
// local coordinates of the EditableText. In the case where there is no
// composing region, the cursor rect is sent.
static FlMethodResponse* set_marked_text_rect(FlTextInputChannel* self,
                                              FlValue* args) {
  double x = fl_value_get_float(fl_value_lookup_string(args, "x"));
  double y = fl_value_get_float(fl_value_lookup_string(args, "y"));
  double width = fl_value_get_float(fl_value_lookup_string(args, "width"));
  double height = fl_value_get_float(fl_value_lookup_string(args, "height"));

  self->vtable->set_marked_text_rect(x, y, width, height, self->user_data);

  return FL_METHOD_RESPONSE(fl_method_success_response_new(nullptr));
}

// Called when a method call is received from Flutter.
static void method_call_cb(FlMethodChannel* channel,
                           FlMethodCall* method_call,
                           gpointer user_data) {
  FlTextInputChannel* self = FL_TEXT_INPUT_CHANNEL(user_data);

  const gchar* method = fl_method_call_get_name(method_call);
  FlValue* args = fl_method_call_get_args(method_call);

  g_autoptr(FlMethodResponse) response = nullptr;
  if (strcmp(method, kSetClientMethod) == 0) {
    response = set_client(self, args);
  } else if (strcmp(method, kShowMethod) == 0) {
    response = show(self);
  } else if (strcmp(method, kSetEditingStateMethod) == 0) {
    response = set_editing_state(self, args);
  } else if (strcmp(method, kClearClientMethod) == 0) {
    response = clear_client(self);
  } else if (strcmp(method, kHideMethod) == 0) {
    response = hide(self);
  } else if (strcmp(method, kSetEditableSizeAndTransform) == 0) {
    response = set_editable_size_and_transform(self, args);
  } else if (strcmp(method, kSetMarkedTextRect) == 0) {
    response = set_marked_text_rect(self, args);
  } else {
    response = FL_METHOD_RESPONSE(fl_method_not_implemented_response_new());
  }

  g_autoptr(GError) error = nullptr;
  if (!fl_method_call_respond(method_call, response, &error)) {
    g_warning("Failed to send method call response: %s", error->message);
  }
}

static void fl_text_input_channel_dispose(GObject* object) {
  FlTextInputChannel* self = FL_TEXT_INPUT_CHANNEL(object);

  g_clear_object(&self->channel);

  G_OBJECT_CLASS(fl_text_input_channel_parent_class)->dispose(object);
}

static void fl_text_input_channel_class_init(FlTextInputChannelClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_text_input_channel_dispose;
}

static void fl_text_input_channel_init(FlTextInputChannel* self) {}

FlTextInputChannel* fl_text_input_channel_new(FlBinaryMessenger* messenger,
                                              FlTextInputChannelVTable* vtable,
                                              gpointer user_data) {
  g_return_val_if_fail(FL_IS_BINARY_MESSENGER(messenger), nullptr);
  g_return_val_if_fail(vtable != nullptr, nullptr);

  FlTextInputChannel* self = FL_TEXT_INPUT_CHANNEL(
      g_object_new(fl_text_input_channel_get_type(), nullptr));

  self->vtable = vtable;
  self->user_data = user_data;

  g_autoptr(FlJsonMethodCodec) codec = fl_json_method_codec_new();
  self->channel =
      fl_method_channel_new(messenger, kChannelName, FL_METHOD_CODEC(codec));
  fl_method_channel_set_method_call_handler(self->channel, method_call_cb, self,
                                            nullptr);

  return self;
}

void fl_text_input_channel_update_editing_state(
    FlTextInputChannel* self,
    int64_t client_id,
    const gchar* text,
    int64_t selection_base,
    int64_t selection_extent,
    FlTextAffinity selection_affinity,
    gboolean selection_is_directional,
    int64_t composing_base,
    int64_t composing_extent,
    GCancellable* cancellable,
    GAsyncReadyCallback callback,
    gpointer user_data) {
  g_return_if_fail(FL_IS_TEXT_INPUT_CHANNEL(self));

  g_autoptr(FlValue) args = fl_value_new_list();
  fl_value_append_take(args, fl_value_new_int(client_id));
  g_autoptr(FlValue) value = fl_value_new_map();

  fl_value_set_string_take(value, kTextKey, fl_value_new_string(text));
  fl_value_set_string_take(value, kSelectionBaseKey,
                           fl_value_new_int(selection_base));
  fl_value_set_string_take(value, kSelectionExtentKey,
                           fl_value_new_int(selection_extent));
  fl_value_set_string_take(
      value, kSelectionAffinityKey,
      fl_value_new_string(text_affinity_to_string(selection_affinity)));
  fl_value_set_string_take(value, kSelectionIsDirectionalKey,
                           fl_value_new_bool(selection_is_directional));
  fl_value_set_string_take(value, kComposingBaseKey,
                           fl_value_new_int(composing_base));
  fl_value_set_string_take(value, kComposingExtentKey,
                           fl_value_new_int(composing_extent));

  fl_value_append(args, value);

  fl_method_channel_invoke_method(self->channel, kUpdateEditingStateMethod,
                                  args, cancellable, callback, user_data);
}

gboolean fl_text_input_channel_update_editing_state_finish(GObject* object,
                                                           GAsyncResult* result,
                                                           GError** error) {
  g_autoptr(FlMethodResponse) response = fl_method_channel_invoke_method_finish(
      FL_METHOD_CHANNEL(object), result, error);
  if (response == nullptr) {
    return FALSE;
  }
  return fl_method_response_get_result(response, error) != nullptr;
}

void fl_text_input_channel_update_editing_state_with_deltas(
    FlTextInputChannel* self,
    int64_t client_id,
    const gchar* old_text,
    const gchar* delta_text,
    int64_t delta_start,
    int64_t delta_end,
    int64_t selection_base,
    int64_t selection_extent,
    FlTextAffinity selection_affinity,
    gboolean selection_is_directional,
    int64_t composing_base,
    int64_t composing_extent,
    GCancellable* cancellable,
    GAsyncReadyCallback callback,
    gpointer user_data) {
  g_return_if_fail(FL_IS_TEXT_INPUT_CHANNEL(self));

  g_autoptr(FlValue) args = fl_value_new_list();
  fl_value_append_take(args, fl_value_new_int(client_id));

  g_autoptr(FlValue) deltaValue = fl_value_new_map();
  fl_value_set_string_take(deltaValue, "oldText",
                           fl_value_new_string(old_text));
  fl_value_set_string_take(deltaValue, "deltaText",
                           fl_value_new_string(delta_text));
  fl_value_set_string_take(deltaValue, "deltaStart",
                           fl_value_new_int(delta_start));
  fl_value_set_string_take(deltaValue, "deltaEnd", fl_value_new_int(delta_end));
  fl_value_set_string_take(deltaValue, "selectionBase",
                           fl_value_new_int(selection_base));
  fl_value_set_string_take(deltaValue, "selectionExtent",
                           fl_value_new_int(selection_extent));
  fl_value_set_string_take(
      deltaValue, "selectionAffinity",
      fl_value_new_string(text_affinity_to_string(selection_affinity)));
  fl_value_set_string_take(deltaValue, "selectionIsDirectional",
                           fl_value_new_bool(selection_is_directional));
  fl_value_set_string_take(deltaValue, "composingBase",
                           fl_value_new_int(composing_base));
  fl_value_set_string_take(deltaValue, "composingExtent",
                           fl_value_new_int(composing_extent));

  g_autoptr(FlValue) deltas = fl_value_new_list();
  fl_value_append(deltas, deltaValue);
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_string(value, "deltas", deltas);

  fl_value_append(args, value);

  fl_method_channel_invoke_method(self->channel,
                                  kUpdateEditingStateWithDeltasMethod, args,
                                  cancellable, callback, user_data);
}

gboolean fl_text_input_channel_update_editing_state_with_deltas_finish(
    GObject* object,
    GAsyncResult* result,
    GError** error) {
  g_autoptr(FlMethodResponse) response = fl_method_channel_invoke_method_finish(
      FL_METHOD_CHANNEL(object), result, error);
  if (response == nullptr) {
    return FALSE;
  }
  return fl_method_response_get_result(response, error) != nullptr;
}

void fl_text_input_channel_perform_action(FlTextInputChannel* self,
                                          int64_t client_id,
                                          const gchar* input_action,
                                          GCancellable* cancellable,
                                          GAsyncReadyCallback callback,
                                          gpointer user_data) {
  g_return_if_fail(FL_IS_TEXT_INPUT_CHANNEL(self));

  g_autoptr(FlValue) args = fl_value_new_list();
  fl_value_append_take(args, fl_value_new_int(client_id));
  fl_value_append_take(args, fl_value_new_string(input_action));

  fl_method_channel_invoke_method(self->channel, kPerformActionMethod, args,
                                  cancellable, callback, user_data);
}

gboolean fl_text_input_channel_perform_action_finish(GObject* object,
                                                     GAsyncResult* result,
                                                     GError** error) {
  g_autoptr(FlMethodResponse) response = fl_method_channel_invoke_method_finish(
      FL_METHOD_CHANNEL(object), result, error);
  if (response == nullptr) {
    return FALSE;
  }
  return fl_method_response_get_result(response, error) != nullptr;
}

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_keyboard_manager.h"

#include <array>
#include <cinttypes>
#include <memory>
#include <string>

#include "flutter/shell/platform/linux/fl_key_channel_responder.h"
#include "flutter/shell/platform/linux/fl_key_embedder_responder.h"
#include "flutter/shell/platform/linux/fl_keyboard_layout.h"
#include "flutter/shell/platform/linux/key_mapping.h"

// Turn on this flag to print complete layout data when switching IMEs. The data
// is used in unit tests.
#define DEBUG_PRINT_LAYOUT

namespace {

static bool is_eascii(uint16_t character) {
  return character < 256;
}

#ifdef DEBUG_PRINT_LAYOUT
// Prints layout entries that will be parsed by `MockLayoutData`.
void debug_format_layout_data(std::string& debug_layout_data,
                              uint16_t keycode,
                              uint16_t clue1,
                              uint16_t clue2) {
  if (keycode % 4 == 0) {
    debug_layout_data.append("    ");
  }

  constexpr int kBufferSize = 30;
  char buffer[kBufferSize];
  buffer[0] = 0;
  buffer[kBufferSize - 1] = 0;

  snprintf(buffer, kBufferSize, "0x%04x, 0x%04x, ", clue1, clue2);
  debug_layout_data.append(buffer);

  if (keycode % 4 == 3) {
    snprintf(buffer, kBufferSize, " // 0x%02x", keycode);
    debug_layout_data.append(buffer);
  }
}
#endif

}  // namespace

typedef struct {
  // Event being handled.
  FlKeyEvent* event;

  // TRUE if the embedder has responded.
  gboolean embedder_responded;

  // TRUE if the channel has responded.
  gboolean channel_responded;

  // TRUE if this event is to be redispatched;
  gboolean redispatch;

  // TRUE if either the embedder of channel handled this event (or both).
  gboolean handled;
} HandleEventData;

static HandleEventData* handle_event_data_new(FlKeyEvent* event) {
  HandleEventData* data =
      static_cast<HandleEventData*>(g_new0(HandleEventData, 1));
  data->event = FL_KEY_EVENT(g_object_ref(event));
  return data;
}

static void handle_event_data_free(HandleEventData* data) {
  g_object_unref(data->event);
  g_free(data);
}

struct _FlKeyboardManager {
  GObject parent_instance;

  FlKeyboardManagerLookupKeyHandler lookup_key_handler;
  gpointer lookup_key_handler_user_data;

  // Key events that have been redispatched.
  GPtrArray* redispatched_key_events;

  FlKeyEmbedderResponder* key_embedder_responder;

  FlKeyChannelResponder* key_channel_responder;

  // Record the derived layout.
  //
  // It is cleared when the platform reports a layout switch. Each entry,
  // which corresponds to a group, is only initialized on the arrival of the
  // first event for that group that has a goal keycode.
  FlKeyboardLayout* derived_layout;

  // A static map from keycodes to all layout goals.
  //
  // It is set up when the manager is initialized and is not changed ever after.
  std::unique_ptr<std::map<uint16_t, const LayoutGoal*>> keycode_to_goals;

  // A static map from logical keys to all mandatory layout goals.
  //
  // It is set up when the manager is initialized and is not changed ever after.
  std::unique_ptr<std::map<uint64_t, const LayoutGoal*>>
      logical_to_mandatory_goals;

  GdkKeymap* keymap;
  gulong keymap_keys_changed_cb_id;  // Signal connection ID for
                                     // keymap-keys-changed

  GCancellable* cancellable;
};

G_DEFINE_TYPE(FlKeyboardManager, fl_keyboard_manager, G_TYPE_OBJECT);

static gboolean event_is_redispatched(FlKeyboardManager* self,
                                      FlKeyEvent* event) {
  guint32 time = fl_key_event_get_time(event);
  gboolean is_press = !!fl_key_event_get_is_press(event);
  guint16 keycode = fl_key_event_get_keycode(event);
  for (guint i = 0; i < self->redispatched_key_events->len; i++) {
    FlKeyEvent* e =
        FL_KEY_EVENT(g_ptr_array_index(self->redispatched_key_events, i));
    if (fl_key_event_get_time(e) == time &&
        !!fl_key_event_get_is_press(e) == is_press &&
        fl_key_event_get_keycode(e) == keycode) {
      g_ptr_array_remove_index(self->redispatched_key_events, i);
      return TRUE;
    }
  }

  return FALSE;
}

static void keymap_keys_changed_cb(FlKeyboardManager* self) {
  g_clear_object(&self->derived_layout);
  self->derived_layout = fl_keyboard_layout_new();
}

static void complete_handle_event(FlKeyboardManager* self, GTask* task) {
  HandleEventData* data =
      static_cast<HandleEventData*>(g_task_get_task_data(task));

  // Waiting for responses.
  if (!data->embedder_responded || !data->channel_responded) {
    return;
  }

  data->redispatch = !data->handled;
  g_task_return_boolean(task, TRUE);
}

static void responder_handle_embedder_event_cb(GObject* object,
                                               GAsyncResult* result,
                                               gpointer user_data) {
  g_autoptr(GTask) task = G_TASK(user_data);
  FlKeyboardManager* self = FL_KEYBOARD_MANAGER(g_task_get_source_object(task));

  HandleEventData* data =
      static_cast<HandleEventData*>(g_task_get_task_data(G_TASK(task)));
  data->embedder_responded = TRUE;

  g_autoptr(GError) error = nullptr;
  gboolean handled;
  if (!fl_key_embedder_responder_handle_event_finish(
          FL_KEY_EMBEDDER_RESPONDER(object), result, &handled, &error)) {
    if (!g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED)) {
      g_warning("Failed to handle key event in embedder: %s", error->message);
    }
    handled = FALSE;
  }
  if (handled) {
    data->handled = TRUE;
  }

  complete_handle_event(self, task);
}

static void responder_handle_channel_event_cb(GObject* object,
                                              GAsyncResult* result,
                                              gpointer user_data) {
  g_autoptr(GTask) task = G_TASK(user_data);
  FlKeyboardManager* self = FL_KEYBOARD_MANAGER(g_task_get_source_object(task));

  HandleEventData* data =
      static_cast<HandleEventData*>(g_task_get_task_data(G_TASK(task)));
  data->channel_responded = TRUE;

  g_autoptr(GError) error = nullptr;
  gboolean handled;
  if (!fl_key_channel_responder_handle_event_finish(
          FL_KEY_CHANNEL_RESPONDER(object), result, &handled, &error)) {
    if (!g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED)) {
      g_warning("Failed to handle key event in platform: %s", error->message);
    }
    handled = FALSE;
  }
  if (handled) {
    data->handled = TRUE;
  }

  complete_handle_event(self, task);
}

static uint16_t convert_key_to_char(FlKeyboardManager* self,
                                    guint keycode,
                                    gint group,
                                    gint level) {
  GdkKeymapKey key = {keycode, group, level};
  constexpr int kBmpMax = 0xD7FF;
  guint origin;
  if (self->lookup_key_handler != nullptr) {
    origin = self->lookup_key_handler(&key, self->lookup_key_handler_user_data);
  } else {
    origin = gdk_keymap_lookup_key(self->keymap, &key);
  }
  return origin < kBmpMax ? origin : 0xFFFF;
}

// Make sure that Flutter has derived the layout for the group of the event,
// if the event contains a goal keycode.
static void guarantee_layout(FlKeyboardManager* self, FlKeyEvent* event) {
  guint8 group = fl_key_event_get_group(event);
  if (fl_keyboard_layout_has_group(self->derived_layout, group)) {
    return;
  }
  if (self->keycode_to_goals->find(fl_key_event_get_keycode(event)) ==
      self->keycode_to_goals->end()) {
    return;
  }

  // Clone all mandatory goals. Each goal is removed from this cloned map when
  // fulfilled, and the remaining ones will be assigned to a default position.
  std::map<uint64_t, const LayoutGoal*> remaining_mandatory_goals =
      *self->logical_to_mandatory_goals;

#ifdef DEBUG_PRINT_LAYOUT
  std::string debug_layout_data;
  for (uint16_t keycode = 0; keycode < 128; keycode += 1) {
    std::vector<uint16_t> this_key_clues = {
        convert_key_to_char(self, keycode, group, 0),
        convert_key_to_char(self, keycode, group, 1),  // Shift
    };
    debug_format_layout_data(debug_layout_data, keycode, this_key_clues[0],
                             this_key_clues[1]);
  }
#endif

  // It's important to only traverse layout goals instead of all keycodes.
  // Some key codes outside of the standard keyboard also gives alpha-numeric
  // letters, and will therefore take over mandatory goals from standard
  // keyboard keys if they come first. Example: French keyboard digit 1.
  for (const LayoutGoal& keycode_goal : layout_goals) {
    uint16_t keycode = keycode_goal.keycode;
    std::vector<uint16_t> this_key_clues = {
        convert_key_to_char(self, keycode, group, 0),
        convert_key_to_char(self, keycode, group, 1),  // Shift
    };

    // The logical key should be the first available clue from below:
    //
    //  - Mandatory goal, if it matches any clue. This ensures that all alnum
    //    keys can be found somewhere.
    //  - US layout, if neither clue of the key is EASCII. This ensures that
    //    there are no non-latin logical keys.
    //  - A value derived on the fly from keycode & keyval.
    for (uint16_t clue : this_key_clues) {
      auto matching_goal = remaining_mandatory_goals.find(clue);
      if (matching_goal != remaining_mandatory_goals.end()) {
        // Found a key that produces a mandatory char. Use it.
        g_return_if_fail(fl_keyboard_layout_get_logical_key(
                             self->derived_layout, group, keycode) == 0);
        fl_keyboard_layout_set_logical_key(self->derived_layout, group, keycode,
                                           clue);
        remaining_mandatory_goals.erase(matching_goal);
        break;
      }
    }
    bool has_any_eascii =
        is_eascii(this_key_clues[0]) || is_eascii(this_key_clues[1]);
    // See if any produced char meets the requirement as a logical key.
    if (fl_keyboard_layout_get_logical_key(self->derived_layout, group,
                                           keycode) == 0 &&
        !has_any_eascii) {
      auto found_us_layout = self->keycode_to_goals->find(keycode);
      if (found_us_layout != self->keycode_to_goals->end()) {
        fl_keyboard_layout_set_logical_key(
            self->derived_layout, group, keycode,
            found_us_layout->second->logical_key);
      }
    }
  }

  // Ensure all mandatory goals are assigned.
  for (const auto mandatory_goal_iter : remaining_mandatory_goals) {
    const LayoutGoal* goal = mandatory_goal_iter.second;
    fl_keyboard_layout_set_logical_key(self->derived_layout, group,
                                       goal->keycode, goal->logical_key);
  }
}

static void fl_keyboard_manager_dispose(GObject* object) {
  FlKeyboardManager* self = FL_KEYBOARD_MANAGER(object);

  g_cancellable_cancel(self->cancellable);

  self->keycode_to_goals.reset();
  self->logical_to_mandatory_goals.reset();

  g_clear_pointer(&self->redispatched_key_events, g_ptr_array_unref);
  g_clear_object(&self->key_embedder_responder);
  g_clear_object(&self->key_channel_responder);
  g_clear_object(&self->derived_layout);
  if (self->keymap_keys_changed_cb_id != 0) {
    g_signal_handler_disconnect(self->keymap, self->keymap_keys_changed_cb_id);
    self->keymap_keys_changed_cb_id = 0;
  }
  g_clear_object(&self->cancellable);

  G_OBJECT_CLASS(fl_keyboard_manager_parent_class)->dispose(object);
}

static void fl_keyboard_manager_class_init(FlKeyboardManagerClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_keyboard_manager_dispose;
}

static void fl_keyboard_manager_init(FlKeyboardManager* self) {
  self->redispatched_key_events =
      g_ptr_array_new_with_free_func(g_object_unref);
  self->derived_layout = fl_keyboard_layout_new();

  self->keycode_to_goals =
      std::make_unique<std::map<uint16_t, const LayoutGoal*>>();
  self->logical_to_mandatory_goals =
      std::make_unique<std::map<uint64_t, const LayoutGoal*>>();
  for (const LayoutGoal& goal : layout_goals) {
    (*self->keycode_to_goals)[goal.keycode] = &goal;
    if (goal.mandatory) {
      (*self->logical_to_mandatory_goals)[goal.logical_key] = &goal;
    }
  }

  self->keymap = gdk_keymap_get_for_display(gdk_display_get_default());
  self->keymap_keys_changed_cb_id = g_signal_connect_swapped(
      self->keymap, "keys-changed", G_CALLBACK(keymap_keys_changed_cb), self);
  self->cancellable = g_cancellable_new();
}

FlKeyboardManager* fl_keyboard_manager_new(FlEngine* engine) {
  FlKeyboardManager* self = FL_KEYBOARD_MANAGER(
      g_object_new(fl_keyboard_manager_get_type(), nullptr));

  self->key_embedder_responder = fl_key_embedder_responder_new(engine);
  self->key_channel_responder =
      fl_key_channel_responder_new(fl_engine_get_binary_messenger(engine));

  return self;
}

void fl_keyboard_manager_add_redispatched_event(FlKeyboardManager* self,
                                                FlKeyEvent* event) {
  g_return_if_fail(FL_IS_KEYBOARD_MANAGER(self));

  g_ptr_array_add(self->redispatched_key_events, g_object_ref(event));
}

void fl_keyboard_manager_handle_event(FlKeyboardManager* self,
                                      FlKeyEvent* event,
                                      GCancellable* cancellable,
                                      GAsyncReadyCallback callback,
                                      gpointer user_data) {
  g_return_if_fail(FL_IS_KEYBOARD_MANAGER(self));
  g_return_if_fail(event != nullptr);

  g_autoptr(GTask) task = g_task_new(self, cancellable, callback, user_data);

  guarantee_layout(self, event);

  g_task_set_task_data(
      task, handle_event_data_new(event),
      reinterpret_cast<GDestroyNotify>(handle_event_data_free));

  if (event_is_redispatched(self, event)) {
    HandleEventData* data =
        static_cast<HandleEventData*>(g_task_get_task_data(task));
    data->handled = TRUE;
    g_task_return_boolean(task, TRUE);
    return;
  }

  uint64_t specified_logical_key = fl_keyboard_layout_get_logical_key(
      self->derived_layout, fl_key_event_get_group(event),
      fl_key_event_get_keycode(event));
  fl_key_embedder_responder_handle_event(
      self->key_embedder_responder, event, specified_logical_key,
      self->cancellable, responder_handle_embedder_event_cb,
      g_object_ref(task));
  fl_key_channel_responder_handle_event(
      self->key_channel_responder, event, specified_logical_key,
      self->cancellable, responder_handle_channel_event_cb, g_object_ref(task));
}

gboolean fl_keyboard_manager_handle_event_finish(
    FlKeyboardManager* self,
    GAsyncResult* result,
    FlKeyEvent** redispatched_event,
    GError** error) {
  g_return_val_if_fail(FL_IS_KEYBOARD_MANAGER(self), FALSE);
  g_return_val_if_fail(g_task_is_valid(result, self), FALSE);

  HandleEventData* data =
      static_cast<HandleEventData*>(g_task_get_task_data(G_TASK(result)));
  if (redispatched_event != nullptr && data->redispatch) {
    *redispatched_event = FL_KEY_EVENT(g_object_ref(data->event));
  }

  return g_task_propagate_boolean(G_TASK(result), error);
}

void fl_keyboard_manager_sync_modifier_if_needed(FlKeyboardManager* self,
                                                 guint state,
                                                 double event_time) {
  g_return_if_fail(FL_IS_KEYBOARD_MANAGER(self));
  fl_key_embedder_responder_sync_modifiers_if_needed(
      self->key_embedder_responder, state, event_time);
}

GHashTable* fl_keyboard_manager_get_pressed_state(FlKeyboardManager* self) {
  g_return_val_if_fail(FL_IS_KEYBOARD_MANAGER(self), nullptr);
  return fl_key_embedder_responder_get_pressed_state(
      self->key_embedder_responder);
}

void fl_keyboard_manager_set_lookup_key_handler(
    FlKeyboardManager* self,
    FlKeyboardManagerLookupKeyHandler lookup_key_handler,
    gpointer user_data) {
  g_return_if_fail(FL_IS_KEYBOARD_MANAGER(self));
  self->lookup_key_handler = lookup_key_handler;
  self->lookup_key_handler_user_data = user_data;
}

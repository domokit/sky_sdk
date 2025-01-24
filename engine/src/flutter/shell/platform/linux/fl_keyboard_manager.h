// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_MANAGER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_MANAGER_H_

#include <gdk/gdk.h>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/fl_key_event.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlKeyboardManager,
                     fl_keyboard_manager,
                     FL,
                     KEYBOARD_MANAGER,
                     GObject);

/**
 * FlKeyboardManager:
 *
 * Processes keyboard events and cooperate with `TextInputManager`.
 *
 * A keyboard event goes through a few sections, each can choose to handle the
 * event, and only unhandled events can move to the next section:
 *
 * - Keyboard: Dispatch to the embedder responder and the channel responder
 *   simultaneously. After both responders have responded (asynchronously), the
 *   event is considered handled if either responder handles it.
 * - Text input: Events are sent to IM filter (usually owned by
 *   `TextInputManager`) and are handled synchronously.
 * - Redispatching: Events are inserted back to the system for redispatching.
 */

/**
 * fl_keyboard_manager_new:
 * @engine: an #FlEngine.
 *
 * Create a new #FlKeyboardManager.
 *
 * Returns: a new #FlKeyboardManager.
 */
FlKeyboardManager* fl_keyboard_manager_new(FlEngine* engine);

/** fl_keyboard_manager_add_redispatched_event:
 * @manager: an #FlKeyboardManager.
 * @event: an event that will be handled by the manager in the future.
 *
 * Add an event that will be redispatched and handled by the manager in the
 * future. When that event is received it will be ignored.
 */
void fl_keyboard_manager_add_redispatched_event(FlKeyboardManager* manager,
                                                FlKeyEvent* event);

/**
 * fl_keyboard_manager_handle_event:
 * @manager: an #FlKeyboardManager.
 * @event: the event to be dispatched. It is usually a wrap of a GdkEventKey.
 * This event will be managed and released by #FlKeyboardManager.
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the view is
 * added.
 * @user_data: (closure): user data to pass to @callback.
 *
 * Make the manager process a system key event. This might eventually send
 * messages to the framework, trigger text input effects, or redispatch the
 * event back to the system.
 */
void fl_keyboard_manager_handle_event(FlKeyboardManager* manager,
                                      FlKeyEvent* event,
                                      GCancellable* cancellable,
                                      GAsyncReadyCallback callback,
                                      gpointer user_data);

/**
 * fl_keyboard_manager_handle_event_finish:
 * @manager: an #FlKeyboardManager.
 * @result: a #GAsyncResult.
 * @redispatched_event: FIXME
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_keyboard_manager_handle_event().
 *
 * Returns: %TRUE on success.
 */
gboolean fl_keyboard_manager_handle_event_finish(
    FlKeyboardManager* manager,
    GAsyncResult* result,
    FlKeyEvent** redispatched_event,
    GError** error);

/**
 * fl_keyboard_manager_sync_modifier_if_needed:
 * @manager: the #FlKeyboardManager self.
 * @state: the state of the modifiers mask.
 * @event_time: the time attribute of the incoming GDK event.
 *
 * If needed, synthesize modifier keys up and down event by comparing their
 * current pressing states with the given modifiers mask.
 */
void fl_keyboard_manager_sync_modifier_if_needed(FlKeyboardManager* manager,
                                                 guint state,
                                                 double event_time);

/**
 * fl_keyboard_manager_get_pressed_state:
 * @manager: the #FlKeyboardManager self.
 *
 * Returns the keyboard pressed state. The hash table contains one entry per
 * pressed keys, mapping from the logical key to the physical key.*
 */
GHashTable* fl_keyboard_manager_get_pressed_state(FlKeyboardManager* manager);

typedef guint (*FlKeyboardManagerLookupKeyHandler)(const GdkKeymapKey* key,
                                                   gpointer user_data);

/**
 * fl_keyboard_manager_set_lookup_key_handler:
 * @manager: the #FlKeyboardManager self.
 *
 * Set the handler for key lookup, for testing purposes only.
 */
void fl_keyboard_manager_set_lookup_key_handler(
    FlKeyboardManager* manager,
    FlKeyboardManagerLookupKeyHandler lookup_key_handler,
    gpointer user_data);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_KEYBOARD_MANAGER_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EMBEDDER_RESPONDER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EMBEDDER_RESPONDER_H_

#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/fl_key_event.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlKeyEmbedderResponder,
                     fl_key_embedder_responder,
                     FL,
                     KEY_EMBEDDER_RESPONDER,
                     GObject);

/**
 * FlKeyEmbedderResponder:
 *
 * A #FlKeyResponder that handles events by sending the converted events
 * through the embedder API.
 *
 * This class communicates with the HardwareKeyboard API in the framework.
 */

/**
 * fl_key_embedder_responder_new:
 * @engine: The #FlEngine, whose the embedder API will be used to send
 * the event.
 *
 * Creates a new #FlKeyEmbedderResponder.
 *
 * Returns: a new #FlKeyEmbedderResponder.
 */
FlKeyEmbedderResponder* fl_key_embedder_responder_new(FlEngine* engine);

/**
 * fl_key_embedder_responder_handle_event:
 * @responder: the #FlKeyEmbedderResponder self.
 * @event: the event to be handled. Must not be null. The object is managed by
 * callee and must not be assumed available after this function.
 * @specified_logical_key:
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the view is
 * added.
 * @user_data: (closure): user data to pass to @callback.
 *
 * Let the responder handle an event, expecting the responder to report whether
 * to handle the event.
 */
void fl_key_embedder_responder_handle_event(FlKeyEmbedderResponder* responder,
                                            FlKeyEvent* event,
                                            uint64_t specified_logical_key,
                                            GCancellable* cancellable,
                                            GAsyncReadyCallback callback,
                                            gpointer user_data);

/**
 * fl_key_embedder_responder_handle_event_finish:
 * @responder: an #FlKeyEmbedderResponder.
 * @result: a #GAsyncResult.
 * @handled: location to write if this event was handled by the embedder.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_key_embedder_responder_handle_event().
 *
 * Returns %TRUE on success.
 */
gboolean fl_key_embedder_responder_handle_event_finish(
    FlKeyEmbedderResponder* responder,
    GAsyncResult* result,
    gboolean* handled,
    GError** error);

/**
 * fl_key_embedder_responder_sync_modifiers_if_needed:
 * @responder: the #FlKeyEmbedderResponder self.
 * @state: the state of the modifiers mask.
 * @event_time: the time attribute of the incoming GDK event.
 *
 * If needed, synthesize modifier keys up and down event by comparing their
 * current pressing states with the given modifiers mask.
 */
void fl_key_embedder_responder_sync_modifiers_if_needed(
    FlKeyEmbedderResponder* responder,
    guint state,
    double event_time);

/**
 * fl_key_embedder_responder_get_pressed_state:
 * @responder: the #FlKeyEmbedderResponder self.
 *
 * Returns the keyboard pressed state. The hash table contains one entry per
 * pressed keys, mapping from the logical key to the physical key.
 */
GHashTable* fl_key_embedder_responder_get_pressed_state(
    FlKeyEmbedderResponder* responder);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EMBEDDER_RESPONDER_H_

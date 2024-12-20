// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EMBEDDER_RESPONDER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_KEY_EMBEDDER_RESPONDER_H_

#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/fl_key_event.h"

// The signature of a function that FlKeyEmbedderResponder calls on every key
// event.
//
// The `user_data` is opaque data managed by the object that creates
// FlKeyEmbedderResponder, and is registered along with this callback
// via `fl_key_embedder_responder_new`.
//
// The `callback_user_data` is opaque data managed by FlKeyEmbedderResponder.
// Instances of the EmbedderSendKeyEvent callback are required to invoke
// `callback` with the `callback_user_data` parameter after the `event` has been
// processed.
typedef void (*EmbedderSendKeyEvent)(const FlutterKeyEvent* event,
                                     FlutterKeyEventCallback callback,
                                     void* callback_user_data,
                                     void* send_key_event_user_data);

/**
 * FlKeyEmbedderResponderAsyncCallback:
 * @event: whether the event has been handled.
 * @user_data: the same value as user_data sent by
 * #fl_key_responder_handle_event.
 *
 * The signature for a callback with which a #FlKeyEmbedderResponder
 *asynchronously reports whether the responder handles the event.
 **/
typedef void (*FlKeyEmbedderResponderAsyncCallback)(bool handled,
                                                    gpointer user_data);

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
 * @send_key_event: a function that is called on every key event.
 * @send_key_event_user_data: an opaque pointer that will be sent back as the
 * last argument of send_key_event, created and managed by the object that holds
 * FlKeyEmbedderResponder.
 *
 * Returns: a new #FlKeyEmbedderResponder.
 */
FlKeyEmbedderResponder* fl_key_embedder_responder_new(
    EmbedderSendKeyEvent send_key_event,
    void* send_key_event_user_data);

/**
 * fl_key_embedder_responder_handle_event:
 * @responder: the #FlKeyEmbedderResponder self.
 * @event: the event to be handled. Must not be null. The object is managed by
 * callee and must not be assumed available after this function.
 * @specified_logical_key:
 * @callback: the callback to report the result. It should be called exactly
 * once. Must not be null.
 * @user_data: a value that will be sent back in the callback. Can be null.
 *
 * Let the responder handle an event, expecting the responder to report
 *  whether to handle the event. The result will be reported by invoking
 * `callback` exactly once, which might happen after
 * `fl_key_embedder_responder_handle_event` or during it.
 */
void fl_key_embedder_responder_handle_event(
    FlKeyEmbedderResponder* responder,
    FlKeyEvent* event,
    uint64_t specified_logical_key,
    FlKeyEmbedderResponderAsyncCallback callback,
    gpointer user_data);

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

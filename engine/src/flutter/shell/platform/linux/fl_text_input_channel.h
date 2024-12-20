// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXT_INPUT_CHANNEL_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXT_INPUT_CHANNEL_H_

#include <gtk/gtk.h>

#include "flutter/shell/platform/linux/public/flutter_linux/fl_binary_messenger.h"

G_BEGIN_DECLS

typedef enum {
  FL_TEXT_INPUT_TYPE_TEXT,
  // Send newline when multi-line and enter is pressed.
  FL_TEXT_INPUT_TYPE_MULTILINE,
  // The input method is not shown at all.
  FL_TEXT_INPUT_TYPE_NONE,
} FlTextInputType;

typedef enum {
  FL_TEXT_AFFINITY_UPSTREAM,
  FL_TEXT_AFFINITY_DOWNSTREAM,
} FlTextAffinity;

G_DECLARE_FINAL_TYPE(FlTextInputChannel,
                     fl_text_input_channel,
                     FL,
                     TEXT_INPUT_CHANNEL,
                     GObject);

/**
 * FlTextInputChannel:
 *
 * #FlTextInputChannel is a channel that implements the shell side
 * of SystemChannels.textInput from the Flutter services library.
 */

typedef struct {
  void (*set_client)(int64_t client_id,
                     const gchar* input_action,
                     gboolean enable_delta_model,
                     FlTextInputType input_type,
                     gpointer user_data);
  void (*hide)(gpointer user_data);
  void (*show)(gpointer user_data);
  void (*set_editing_state)(const gchar* text,
                            int64_t selection_base,
                            int64_t selection_extent,
                            int64_t composing_base,
                            int64_t composing_extent,
                            gpointer user_data);
  void (*clear_client)(gpointer user_data);
  void (*set_editable_size_and_transform)(double* transform,
                                          gpointer user_data);
  void (*set_marked_text_rect)(double x,
                               double y,
                               double width,
                               double height,
                               gpointer user_data);
} FlTextInputChannelVTable;

/**
 * fl_text_input_channel_new:
 * @messenger: an #FlBinaryMessenger.
 * @vtable: callbacks for incoming method calls.
 * @user_data: data to pass in callbacks.
 *
 * Creates a new channel that implements SystemChannels.textInput from the
 * Flutter services library.
 *
 * Returns: a new #FlTextInputChannel.
 */
FlTextInputChannel* fl_text_input_channel_new(FlBinaryMessenger* messenger,
                                              FlTextInputChannelVTable* vtable,
                                              gpointer user_data);

/**
 * fl_text_input_channel_update_editing_state:
 * @channel: an #FlTextInputChannel.
 * @client_id:
 * @text:
 * @selection_base:
 * @selection_extent:
 * @selection_affinity:
 * @selection_is_directional:
 * @composing_base:
 * @composing_extent:
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the method
 * returns.
 * @user_data: (closure): user data to pass to @callback.
 */
void fl_text_input_channel_update_editing_state(
    FlTextInputChannel* channel,
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
    gpointer user_data);

/**
 * fl_text_input_channel_update_editing_state_finish:
 * @object:
 * @result: a #GAsyncResult.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_text_input_channel_update_editing_state().
 *
 * Returns: %TRUE on success.
 */
gboolean fl_text_input_channel_update_editing_state_finish(GObject* object,
                                                           GAsyncResult* result,
                                                           GError** error);

/**
 * fl_text_input_channel_update_editing_state_with_deltas:
 * @channel: an #FlTextInputChannel.
 * @client_id:
 * @old_text:
 * @delta_text:
 * @delta_start:
 * @delta_end:
 * @selection_base:
 * @selection_extent:
 * @selection_affinity:
 * @selection_is_directional:
 * @composing_base:
 * @composing_extent:
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the method
 * returns.
 * @user_data: (closure): user data to pass to @callback.
 */
void fl_text_input_channel_update_editing_state_with_deltas(
    FlTextInputChannel* channel,
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
    gpointer user_data);

/**
 * fl_text_input_channel_update_editing_state_with_deltas_finish:
 * @object:
 * @result: a #GAsyncResult.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with
 * fl_text_input_channel_update_editing_state_with_deltas().
 *
 * Returns: %TRUE on success.
 */
gboolean fl_text_input_channel_update_editing_state_with_deltas_finish(
    GObject* object,
    GAsyncResult* result,
    GError** error);

/**
 * fl_text_input_channel_perform_action:
 * @channel: an #FlTextInputChannel.
 * @client_id:
 * @input_action: action to perform.
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the method
 * returns.
 * @user_data: (closure): user data to pass to @callback.
 */
void fl_text_input_channel_perform_action(FlTextInputChannel* channel,
                                          int64_t client_id,
                                          const gchar* input_action,
                                          GCancellable* cancellable,
                                          GAsyncReadyCallback callback,
                                          gpointer user_data);

/**
 * fl_text_input_channel_perform_action_finish:
 * @object:
 * @result: a #GAsyncResult.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_text_input_channel_perform_action().
 *
 * Returns: %TRUE on success.
 */
gboolean fl_text_input_channel_perform_action_finish(GObject* object,
                                                     GAsyncResult* result,
                                                     GError** error);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_TEXT_INPUT_CHANNEL_H_

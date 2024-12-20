// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_ENGINE_PRIVATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_ENGINE_PRIVATE_H_

#include <glib-object.h>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/fl_mouse_cursor_handler.h"
#include "flutter/shell/platform/linux/fl_renderer.h"
#include "flutter/shell/platform/linux/fl_task_runner.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_dart_project.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

/**
 * FlEngineError:
 * Errors for #FlEngine objects to set on failures.
 */

typedef enum {
  FL_ENGINE_ERROR_FAILED,
} FlEngineError;

GQuark fl_engine_error_quark(void) G_GNUC_CONST;

/**
 * FlEnginePlatformMessageHandler:
 * @engine: an #FlEngine.
 * @channel: channel message received on.
 * @message: message content received from Dart.
 * @response_handle: a handle to respond to the message with.
 * @user_data: (closure): data provided when registering this handler.
 *
 * Function called when platform messages are received.
 *
 * Returns: %TRUE if message has been accepted.
 */
typedef gboolean (*FlEnginePlatformMessageHandler)(
    FlEngine* engine,
    const gchar* channel,
    GBytes* message,
    const FlutterPlatformMessageResponseHandle* response_handle,
    gpointer user_data);

/**
 * FlEngineUpdateSemanticsHandler:
 * @engine: an #FlEngine.
 * @node: semantic node information.
 * @user_data: (closure): data provided when registering this handler.
 *
 * Function called when semantics node updates are received.
 */
typedef void (*FlEngineUpdateSemanticsHandler)(
    FlEngine* engine,
    const FlutterSemanticsUpdate2* update,
    gpointer user_data);

/**
 * fl_engine_new_with_renderer:
 * @project: an #FlDartProject.
 * @renderer: an #FlRenderer.
 *
 * Creates new Flutter engine.
 *
 * Returns: a new #FlEngine.
 */
FlEngine* fl_engine_new_with_renderer(FlDartProject* project,
                                      FlRenderer* renderer);

/**
 * fl_engine_get_renderer:
 * @engine: an #FlEngine.
 *
 * Gets the renderer used by this engine.
 *
 * Returns: an #FlRenderer.
 */
FlRenderer* fl_engine_get_renderer(FlEngine* engine);

/**
 * fl_engine_start:
 * @engine: an #FlEngine.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Starts the Flutter engine.
 *
 * Returns: %TRUE on success.
 */
gboolean fl_engine_start(FlEngine* engine, GError** error);

/**
 * fl_engine_get_embedder_api:
 * @engine: an #FlEngine.
 *
 * Gets the embedder API proc table, allowing modificiations for unit testing.
 *
 * Returns: a mutable pointer to the embedder API proc table.
 */
FlutterEngineProcTable* fl_engine_get_embedder_api(FlEngine* engine);

/**
 * fl_engine_add_view:
 * @engine: an #FlEngine.
 * @width: width of view in pixels.
 * @height: height of view in pixels.
 * @pixel_ratio: scale factor for view.
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the view is
 * added.
 * @user_data: (closure): user data to pass to @callback.
 *
 * Asynchronously add a new view. The returned view ID should not be used until
 * this function completes.
 *
 * Returns: the ID for the view.
 */
FlutterViewId fl_engine_add_view(FlEngine* engine,
                                 size_t width,
                                 size_t height,
                                 double pixel_ratio,
                                 GCancellable* cancellable,
                                 GAsyncReadyCallback callback,
                                 gpointer user_data);

/**
 * fl_engine_add_view_finish:
 * @engine: an #FlEngine.
 * @result: a #GAsyncResult.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_engine_add_view().
 *
 * Returns: %TRUE on success.
 */
gboolean fl_engine_add_view_finish(FlEngine* engine,
                                   GAsyncResult* result,
                                   GError** error);

/**
 * fl_engine_remove_view:
 * @engine: an #FlEngine.
 * @view_id: ID to remove.
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the view is
 * added.
 * @user_data: (closure): user data to pass to @callback.
 *
 * Removes a view previously added with fl_engine_add_view().
 */
void fl_engine_remove_view(FlEngine* engine,
                           FlutterViewId view_id,
                           GCancellable* cancellable,
                           GAsyncReadyCallback callback,
                           gpointer user_data);

/**
 * fl_engine_remove_view_finish:
 * @engine: an #FlEngine.
 * @result: a #GAsyncResult.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_engine_remove_view().
 *
 * Returns: %TRUE on succcess.
 */
gboolean fl_engine_remove_view_finish(FlEngine* engine,
                                      GAsyncResult* result,
                                      GError** error);

/**
 * fl_engine_set_platform_message_handler:
 * @engine: an #FlEngine.
 * @handler: function to call when a platform message is received.
 * @user_data: (closure): user data to pass to @handler.
 * @destroy_notify: (allow-none): a function which gets called to free
 * @user_data, or %NULL.
 *
 * Registers the function called when a platform message is received. Call
 * fl_engine_send_platform_message_response() with the response to this message.
 * Ownership of #FlutterPlatformMessageResponseHandle is
 * transferred to the caller, and the message must be responded to avoid
 * memory leaks.
 */
void fl_engine_set_platform_message_handler(
    FlEngine* engine,
    FlEnginePlatformMessageHandler handler,
    gpointer user_data,
    GDestroyNotify destroy_notify);

/**
 * fl_engine_set_update_semantics_handler:
 * @engine: an #FlEngine.
 * @handler: function to call when a semantics update is received.
 * @user_data: (closure): user data to pass to @handler.
 * @destroy_notify: (allow-none): a function which gets called to free
 * @user_data, or %NULL.
 *
 * Registers the function called when a semantics update is received.
 */
void fl_engine_set_update_semantics_handler(
    FlEngine* engine,
    FlEngineUpdateSemanticsHandler handler,
    gpointer user_data,
    GDestroyNotify destroy_notify);

/**
 * fl_engine_send_window_metrics_event:
 * @engine: an #FlEngine.
 * @view_id: the view that the event occured on.
 * @width: width of the window in pixels.
 * @height: height of the window in pixels.
 * @pixel_ratio: scale factor for window.
 *
 * Sends a window metrics event to the engine.
 */
void fl_engine_send_window_metrics_event(FlEngine* engine,
                                         FlutterViewId view_id,
                                         size_t width,
                                         size_t height,
                                         double pixel_ratio);

/**
 * fl_engine_send_mouse_pointer_event:
 * @engine: an #FlEngine.
 * @view_id: the view that the event occured on.
 * @phase: mouse phase.
 * @timestamp: time when event occurred in microseconds.
 * @x: x location of mouse cursor.
 * @y: y location of mouse cursor.
 * @device_kind: kind of pointing device.
 * @scroll_delta_x: x offset of scroll.
 * @scroll_delta_y: y offset of scroll.
 * @buttons: buttons that are pressed.
 *
 * Sends a mouse pointer event to the engine.
 */
void fl_engine_send_mouse_pointer_event(FlEngine* engine,
                                        FlutterViewId view_id,
                                        FlutterPointerPhase phase,
                                        size_t timestamp,
                                        double x,
                                        double y,
                                        FlutterPointerDeviceKind device_kind,
                                        double scroll_delta_x,
                                        double scroll_delta_y,
                                        int64_t buttons);

/**
 * fl_engine_send_touch_up_event:
 * @engine: an #FlEngine.
 * @view_id: the view that the event occured on.
 * @timestamp: time when event occurred in microseconds.
 * @x: x location of mouse cursor.
 * @y: y location of mouse cursor.
 * @device: device id.
 *
 * Sends a touch up event to the engine.
 */
void fl_engine_send_touch_up_event(FlEngine* engine,
                                   FlutterViewId view_id,
                                   size_t timestamp,
                                   double x,
                                   double y,
                                   int32_t device);

/**
 * fl_engine_send_touch_down_event:
 * @engine: an #FlEngine.
 * @view_id: the view that the event occured on.
 * @timestamp: time when event occurred in microseconds.
 * @x: x location of mouse cursor.
 * @y: y location of mouse cursor.
 * @device: device id.
 *
 * Sends a touch down event to the engine.
 */
void fl_engine_send_touch_down_event(FlEngine* engine,
                                     FlutterViewId view_id,
                                     size_t timestamp,
                                     double x,
                                     double y,
                                     int32_t device);
/**
 * fl_engine_send_touch_move_event:
 * @engine: an #FlEngine.
 * @view_id: the view that the event occured on.
 * @timestamp: time when event occurred in microseconds.
 * @x: x location of mouse cursor.
 * @y: y location of mouse cursor.
 * @device: device id.
 *
 * Sends a touch move event to the engine.
 */
void fl_engine_send_touch_move_event(FlEngine* engine,
                                     FlutterViewId view_id,
                                     size_t timestamp,
                                     double x,
                                     double y,
                                     int32_t device);

/**
 * fl_engine_send_touch_add_event:
 * @engine: an #FlEngine.
 * @view_id: the view that the event occured on.
 * @timestamp: time when event occurred in microseconds.
 * @x: x location of mouse cursor.
 * @y: y location of mouse cursor.
 * @device: device id.
 *
 * Sends a touch add event to the engine.
 */
void fl_engine_send_touch_add_event(FlEngine* engine,
                                    FlutterViewId view_id,
                                    size_t timestamp,
                                    double x,
                                    double y,
                                    int32_t device);

/**
 * fl_engine_send_touch_remove_event:
 * @engine: an #FlEngine.
 * @view_id: the view that the event occured on.
 * @timestamp: time when event occurred in microseconds.
 * @x: x location of mouse cursor.
 * @y: y location of mouse cursor.
 * @device: device id.
 *
 * Sends a touch remove event to the engine.
 */
void fl_engine_send_touch_remove_event(FlEngine* engine,
                                       FlutterViewId view_id,
                                       size_t timestamp,
                                       double x,
                                       double y,
                                       int32_t device);

/**
 * fl_engine_send_pointer_pan_zoom_event:
 * @engine: an #FlEngine.
 * @view_id: the view that the event occured on.
 * @timestamp: time when event occurred in microseconds.
 * @x: x location of mouse cursor.
 * @y: y location of mouse cursor.
 * @phase: mouse phase.
 * @pan_x: x offset of the pan/zoom in pixels.
 * @pan_y: y offset of the pan/zoom in pixels.
 * @scale: scale of the pan/zoom.
 * @rotation: rotation of the pan/zoom in radians.
 *
 * Sends a pan/zoom pointer event to the engine.
 */
void fl_engine_send_pointer_pan_zoom_event(FlEngine* engine,
                                           FlutterViewId view_id,
                                           size_t timestamp,
                                           double x,
                                           double y,
                                           FlutterPointerPhase phase,
                                           double pan_x,
                                           double pan_y,
                                           double scale,
                                           double rotation);

/**
 * fl_engine_send_key_event:
 * @engine: an #FlEngine.
 * @event: key event to send.
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the request is
 * satisfied.
 * @user_data: (closure): user data to pass to @callback.
 *
 * Send a key event to the engine.
 */
void fl_engine_send_key_event(FlEngine* engine,
                              const FlutterKeyEvent* event,
                              GCancellable* cancellable,
                              GAsyncReadyCallback callback,
                              gpointer user_data);

/**
 * fl_engine_send_key_event_finish:
 * @engine: an #FlEngine.
 * @result: a #GAsyncResult.
 * @handled: location to write if this event was handled by the engine.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_engine_send_key_event().
 *
 * Returns: %TRUE on success.
 */
gboolean fl_engine_send_key_event_finish(FlEngine* engine,
                                         GAsyncResult* result,
                                         gboolean* handled,
                                         GError** error);

/**
 * fl_engine_dispatch_semantics_action:
 * @engine: an #FlEngine.
 * @id: the semantics action identifier.
 * @action: the action being dispatched.
 * @data: (allow-none): data associated with the action.
 */
void fl_engine_dispatch_semantics_action(FlEngine* engine,
                                         uint64_t id,
                                         FlutterSemanticsAction action,
                                         GBytes* data);

/**
 * fl_engine_send_platform_message_response:
 * @engine: an #FlEngine.
 * @handle: handle that was provided in #FlEnginePlatformMessageHandler.
 * @response: (allow-none): response to send or %NULL for an empty response.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Responds to a platform message.
 *
 * Returns: %TRUE on success.
 */
gboolean fl_engine_send_platform_message_response(
    FlEngine* engine,
    const FlutterPlatformMessageResponseHandle* handle,
    GBytes* response,
    GError** error);

/**
 * fl_engine_send_platform_message:
 * @engine: an #FlEngine.
 * @channel: channel to send to.
 * @message: (allow-none): message buffer to send or %NULL for an empty message
 * @cancellable: (allow-none): a #GCancellable or %NULL.
 * @callback: (scope async): a #GAsyncReadyCallback to call when the request is
 * satisfied.
 * @user_data: (closure): user data to pass to @callback.
 *
 * Asynchronously sends a platform message.
 */
void fl_engine_send_platform_message(FlEngine* engine,
                                     const gchar* channel,
                                     GBytes* message,
                                     GCancellable* cancellable,
                                     GAsyncReadyCallback callback,
                                     gpointer user_data);

/**
 * fl_engine_send_platform_message_finish:
 * @engine: an #FlEngine.
 * @result: a #GAsyncResult.
 * @error: (allow-none): #GError location to store the error occurring, or %NULL
 * to ignore.
 *
 * Completes request started with fl_engine_send_platform_message().
 *
 * Returns: message response on success or %NULL on error.
 */
GBytes* fl_engine_send_platform_message_finish(FlEngine* engine,
                                               GAsyncResult* result,
                                               GError** error);

/**
 * fl_engine_get_task_runner:
 * @engine: an #FlEngine.
 * @result: a #FlTaskRunner.
 *
 * Returns: task runner responsible for scheduling Flutter tasks.
 */
FlTaskRunner* fl_engine_get_task_runner(FlEngine* engine);

/**
 * fl_engine_execute_task:
 * @engine: an #FlEngine.
 * @task: a #FlutterTask to execute.
 *
 * Executes given Flutter task.
 */
void fl_engine_execute_task(FlEngine* engine, FlutterTask* task);

/**
 * fl_engine_mark_texture_frame_available:
 * @engine: an #FlEngine.
 * @texture_id: the identifier of the texture whose frame has been updated.
 *
 * Tells the Flutter engine that a new texture frame is available for the given
 * texture.
 *
 * Returns: %TRUE on success.
 */
gboolean fl_engine_mark_texture_frame_available(FlEngine* engine,
                                                int64_t texture_id);

/**
 * fl_engine_register_external_texture:
 * @engine: an #FlEngine.
 * @texture_id: the identifier of the texture that is available.
 *
 * Tells the Flutter engine that a new external texture is available.
 *
 * Returns: %TRUE on success.
 */
gboolean fl_engine_register_external_texture(FlEngine* engine,
                                             int64_t texture_id);

/**
 * fl_engine_unregister_external_texture:
 * @engine: an #FlEngine.
 * @texture_id: the identifier of the texture that is not available anymore.
 *
 * Tells the Flutter engine that an existing external texture is not available
 * anymore.
 *
 * Returns: %TRUE on success.
 */
gboolean fl_engine_unregister_external_texture(FlEngine* engine,
                                               int64_t texture_id);

/**
 * fl_engine_update_accessibility_features:
 * @engine: an #FlEngine.
 * @flags: the features to enable in the accessibility tree.
 *
 * Tells the Flutter engine to update the flags on the accessibility tree.
 */
void fl_engine_update_accessibility_features(FlEngine* engine, int32_t flags);

/**
 * fl_engine_request_app_exit:
 * @engine: an #FlEngine.
 *
 * Request the application exits.
 */
void fl_engine_request_app_exit(FlEngine* engine);

/**
 * fl_engine_get_mouse_cursor_handler:
 * @engine: an #FlEngine.
 *
 * Gets the mouse cursor handler used by this engine.
 *
 * Returns: a #FlMouseCursorHandler.
 */
FlMouseCursorHandler* fl_engine_get_mouse_cursor_handler(FlEngine* engine);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_ENGINE_PRIVATE_H_

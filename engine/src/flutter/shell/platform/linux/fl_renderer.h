// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERER_H_

#include <gtk/gtk.h>

#include "flutter/shell/platform/embedder/embedder.h"

#include "flutter/shell/platform/linux/fl_renderable.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

/**
 * FlRendererError:
 * Errors for #FlRenderer objects to set on failures.
 */

typedef enum {
  FL_RENDERER_ERROR_FAILED,
} FlRendererError;

GQuark fl_renderer_error_quark(void) G_GNUC_CONST;

G_DECLARE_DERIVABLE_TYPE(FlRenderer, fl_renderer, FL, RENDERER, GObject)

/**
 * FlRenderer:
 *
 * #FlRenderer is an abstract class that allows Flutter to draw pixels.
 */

struct _FlRendererClass {
  GObjectClass parent_class;

  /**
   * Virtual method called when Flutter needs to make the OpenGL context
   * current.
   * @renderer: an #FlRenderer.
   */
  void (*make_current)(FlRenderer* renderer);

  /**
   * Virtual method called when Flutter needs to make the OpenGL resource
   * context current.
   * @renderer: an #FlRenderer.
   */
  void (*make_resource_current)(FlRenderer* renderer);

  /**
   * Virtual method called when Flutter needs to clear the OpenGL context.
   * @renderer: an #FlRenderer.
   */
  void (*clear_current)(FlRenderer* renderer);

  /**
   * Virtual method called when Flutter wants to get the refresh rate of the
   * renderer.
   * @renderer: an #FlRenderer.
   *
   * Returns: The refresh rate of the display in Hz. If the refresh rate is
   * not available, returns -1.0.
   */
  gdouble (*get_refresh_rate)(FlRenderer* renderer);
};

/**
 * fl_renderer_set_engine:
 * @renderer: an #FlRenderer.
 * @engine: an #FlEngine.
 *
 * Called when the renderer is connected to an engine.
 */
void fl_renderer_set_engine(FlRenderer* renderer, FlEngine* engine);

/**
 * fl_renderer_add_renderable:
 * @renderer: an #FlRenderer.
 * @view_id: the ID of the view.
 * @renderable: object that is to be rendered on.
 *
 * Add a view to render on.
 */
void fl_renderer_add_renderable(FlRenderer* renderer,
                                FlutterViewId view_id,
                                FlRenderable* renderable);

/**
 * fl_renderer_remove_view:
 * @renderer: an #FlRenderer.
 * @view_id: the ID of the view.
 *
 * Remove a view from the renderer.
 */
void fl_renderer_remove_view(FlRenderer* renderer, FlutterViewId view_id);

/**
 * fl_renderer_get_proc_address:
 * @renderer: an #FlRenderer.
 * @name: a function name.
 *
 * Gets the rendering API function that matches the given name.
 *
 * Returns: a function pointer.
 */
void* fl_renderer_get_proc_address(FlRenderer* renderer, const char* name);

/**
 * fl_renderer_make_current:
 * @renderer: an #FlRenderer.
 *
 * Makes the rendering context current.
 */
void fl_renderer_make_current(FlRenderer* renderer);

/**
 * fl_renderer_make_resource_current:
 * @renderer: an #FlRenderer.
 *
 * Makes the resource rendering context current.
 */
void fl_renderer_make_resource_current(FlRenderer* renderer);

/**
 * fl_renderer_clear_current:
 * @renderer: an #FlRenderer.
 *
 * Clears the current rendering context.
 */
void fl_renderer_clear_current(FlRenderer* renderer);

/**
 * fl_renderer_get_fbo:
 * @renderer: an #FlRenderer.
 *
 * Gets the frame buffer object to render to.
 *
 * Returns: a frame buffer object index.
 */
guint32 fl_renderer_get_fbo(FlRenderer* renderer);

/**
 * fl_renderer_create_backing_store:
 * @renderer: an #FlRenderer.
 * @config: backing store config.
 * @backing_store_out: saves created backing store.
 *
 * Obtain a backing store for a specific #FlutterLayer.
 *
 * Returns %TRUE if successful.
 */
gboolean fl_renderer_create_backing_store(
    FlRenderer* renderer,
    const FlutterBackingStoreConfig* config,
    FlutterBackingStore* backing_store_out);

/**
 * fl_renderer_collect_backing_store:
 * @renderer: an #FlRenderer.
 * @backing_store: backing store to be released.
 *
 * A callback invoked by the engine to release the backing store. The
 * embedder may collect any resources associated with the backing store.
 *
 * Returns %TRUE if successful.
 */
gboolean fl_renderer_collect_backing_store(
    FlRenderer* renderer,
    const FlutterBackingStore* backing_store);

/**
 * fl_renderer_present_layers:
 * @renderer: an #FlRenderer.
 * @view_id: view to present.
 * @layers: layers to be composited.
 * @layers_count: number of layers.
 *
 * Callback invoked by the engine to composite the contents of each layer
 * onto the screen.
 *
 * Returns %TRUE if successful.
 */
gboolean fl_renderer_present_layers(FlRenderer* renderer,
                                    FlutterViewId view_id,
                                    const FlutterLayer** layers,
                                    size_t layers_count);

/**
 * fl_renderer_wait_for_frame:
 * @renderer: an #FlRenderer.
 * @target_width: width of frame being waited for
 * @target_height: height of frame being waited for
 *
 * Holds the thread until frame with requested dimensions is presented.
 * While waiting for frame Flutter platform and raster tasks are being
 * processed.
 */
void fl_renderer_wait_for_frame(FlRenderer* renderer,
                                int target_width,
                                int target_height);

/**
 * fl_renderer_setup:
 * @renderer: an #FlRenderer.
 *
 * Creates OpenGL resources required before rendering. Requires an active OpenGL
 * context.
 */
void fl_renderer_setup(FlRenderer* renderer);

/**
 * fl_renderer_render:
 * @renderer: an #FlRenderer.
 * @view_id: view to render.
 * @width: width of the window in pixels.
 * @height: height of the window in pixels.
 * @background_color: color to use for background.
 *
 * Performs OpenGL commands to render current Flutter view.
 */
void fl_renderer_render(FlRenderer* renderer,
                        FlutterViewId view_id,
                        int width,
                        int height,
                        const GdkRGBA* background_color);

/**
 * fl_renderer_cleanup:
 *
 * Removes OpenGL resources used for rendering. Requires an active OpenGL
 * context.
 */
void fl_renderer_cleanup(FlRenderer* renderer);

/**
 * fl_renderer_get_refresh_rate:
 * @renderer: an #FlRenderer.
 *
 * Returns: The refresh rate of the display in Hz. If the refresh rate is
 * not available, returns -1.0.
 */
gdouble fl_renderer_get_refresh_rate(FlRenderer* renderer);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_RENDERER_H_

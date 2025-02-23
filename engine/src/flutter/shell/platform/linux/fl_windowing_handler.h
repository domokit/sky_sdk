// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_WINDOWING_HANDLER_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_WINDOWING_HANDLER_H_

#include <gtk/gtk.h>

#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_view.h"

G_BEGIN_DECLS

G_DECLARE_DERIVABLE_TYPE(FlWindowingHandler,
                         fl_windowing_handler,
                         FL,
                         WINDOWING_HANDLER,
                         GObject);

struct _FlWindowingHandlerClass {
  GObjectClass parent_class;

  GtkWindow* (*create_window)(FlWindowingHandler* handler, FlView* view);
};

/**
 * FlWindowingHandler:
 *
 * #FlWindowingHandler is a handler that implements the shell side
 * of SystemChannels.windowing from the Flutter services library.
 */

/**
 * fl_windowing_handler_new:
 * @engine: an #FlEngine.
 *
 * Creates a new handler that implements SystemChannels.windowing from the
 * Flutter services library.
 *
 * Returns: a new #FlWindowingHandler
 */
FlWindowingHandler* fl_windowing_handler_new(FlEngine* engine);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_WINDOWING_HANDLER_H_

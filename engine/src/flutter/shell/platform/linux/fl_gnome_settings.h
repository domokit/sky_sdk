// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_GNOME_SETTINGS_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_GNOME_SETTINGS_H_

#include "flutter/shell/platform/linux/fl_settings.h"

G_BEGIN_DECLS

G_DECLARE_FINAL_TYPE(FlGnomeSettings,
                     fl_gnome_settings,
                     FL,
                     GNOME_SETTINGS,
                     GObject);

/**
 * fl_gnome_settings_new:
 *
 * Creates a new settings instance for GNOME.
 *
 * Returns: a new #FlSettings.
 */
FlSettings* fl_gnome_settings_new();

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_GNOME_SETTINGS_H_

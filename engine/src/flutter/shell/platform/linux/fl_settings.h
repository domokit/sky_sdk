// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_SETTINGS_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_SETTINGS_H_

#include <glib-object.h>

G_BEGIN_DECLS

G_DECLARE_INTERFACE(FlSettings, fl_settings, FL, SETTINGS, GObject)

/**
 * FlClockFormat:
 * @FL_CLOCK_FORMAT_12H: 12-hour clock format.
 * @FL_CLOCK_FORMAT_24H: 24-hour clock format.
 *
 * Available clock formats.
 */
typedef enum {
  FL_CLOCK_FORMAT_12H,
  FL_CLOCK_FORMAT_24H,
} FlClockFormat;

/**
 * FlColorScheme:
 * @FL_COLOR_SCHEME_LIGHT: Prefer light theme.
 * @FL_COLOR_SCHEME_DARK: Prefer dark theme.
 *
 * Available color schemes.
 */
typedef enum {
  FL_COLOR_SCHEME_LIGHT,
  FL_COLOR_SCHEME_DARK,
} FlColorScheme;

/**
 * FlSettings:
 * #FlSettings is and object that provides desktop settings.
 */
struct _FlSettingsInterface {
  GTypeInterface parent;
  FlClockFormat (*get_clock_format)(FlSettings* settings);
  FlColorScheme (*get_color_scheme)(FlSettings* settings);
  gboolean (*get_enable_animations)(FlSettings* settings);
  gboolean (*get_high_contrast)(FlSettings* settings);
  gdouble (*get_text_scaling_factor)(FlSettings* settings);
};

/**
 * fl_settings_new:
 *
 * Creates a new settings instance.
 *
 * Returns: a new #FlSettings.
 */
FlSettings* fl_settings_new();

/**
 * fl_settings_get_clock_format:
 * @settings: an #FlSettings.
 *
 * Whether the clock displays in 24-hour or 12-hour format.
 *
 * This corresponds to `org.gnome.desktop.interface.clock-format` in GNOME.
 *
 * Returns: an #FlClockFormat.
 */
FlClockFormat fl_settings_get_clock_format(FlSettings* settings);

/**
 * fl_settings_get_color_scheme:
 * @settings: an #FlSettings.
 *
 * The preferred color scheme for the user interface.
 *
 * This corresponds to `org.gnome.desktop.interface.color-scheme` in GNOME.
 *
 * Returns: an #FlColorScheme.
 */
FlColorScheme fl_settings_get_color_scheme(FlSettings* settings);

/**
 * fl_settings_get_enable_animations:
 * @settings: an #FlSettings.
 *
 * Whether animations should be enabled.
 *
 * This corresponds to `org.gnome.desktop.interface.enable-animations` in GNOME.
 *
 * Returns: %TRUE if animations are enabled.
 */
gboolean fl_settings_get_enable_animations(FlSettings* settings);

/**
 * fl_settings_get_high_contrast:
 * @settings: an #FlSettings.
 *
 * Whether to use high contrast theme.
 *
 * This corresponds to `org.gnome.desktop.a11y.interface.high-contrast` in
 * GNOME.
 *
 * Returns: %TRUE if high contrast is used.
 */
gboolean fl_settings_get_high_contrast(FlSettings* settings);

/**
 * fl_settings_get_text_scaling_factor:
 * @settings: an #FlSettings.
 *
 * Factor used to enlarge or reduce text display, without changing font size.
 *
 * This corresponds to `org.gnome.desktop.interface.text-scaling-factor` in
 * GNOME.
 *
 * Returns: a floating point number.
 */
gdouble fl_settings_get_text_scaling_factor(FlSettings* settings);

/**
 * fl_settings_emit_changed:
 * @settings: an #FlSettings.
 *
 * Emits the "changed" signal. Used by FlSettings implementations to notify when
 * the desktop settings have changed.
 */
void fl_settings_emit_changed(FlSettings* settings);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_SETTINGS_H_

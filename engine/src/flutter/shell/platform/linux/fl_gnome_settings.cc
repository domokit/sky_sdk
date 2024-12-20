// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_gnome_settings.h"

#include <gio/gio.h>
#include <glib.h>

static constexpr char kDesktopInterfaceSchema[] = "org.gnome.desktop.interface";
static constexpr char kDesktopTextScalingFactorKey[] = "text-scaling-factor";
static constexpr char kDesktopClockFormatKey[] = "clock-format";
static constexpr char kDesktopGtkThemeKey[] = "gtk-theme";

static constexpr char kClockFormat12Hour[] = "12h";
static constexpr char kGtkThemeDarkSuffix[] = "-dark";
static constexpr char kInterfaceSettings[] = "interface-settings";

struct _FlGnomeSettings {
  GObject parent_instance;

  GSettings* interface_settings;
};

enum { PROP_0, PROP_INTERFACE_SETTINGS, PROP_LAST };

static void fl_gnome_settings_iface_init(FlSettingsInterface* iface);

G_DEFINE_TYPE_WITH_CODE(FlGnomeSettings,
                        fl_gnome_settings,
                        G_TYPE_OBJECT,
                        G_IMPLEMENT_INTERFACE(fl_settings_get_type(),
                                              fl_gnome_settings_iface_init))

static FlClockFormat fl_gnome_settings_get_clock_format(FlSettings* settings) {
  FlGnomeSettings* self = FL_GNOME_SETTINGS(settings);

  FlClockFormat clock_format = FL_CLOCK_FORMAT_24H;

  if (self->interface_settings != nullptr) {
    g_autofree gchar* value =
        g_settings_get_string(self->interface_settings, kDesktopClockFormatKey);
    if (g_strcmp0(value, kClockFormat12Hour) == 0) {
      clock_format = FL_CLOCK_FORMAT_12H;
    }
  }
  return clock_format;
}

static FlColorScheme fl_gnome_settings_get_color_scheme(FlSettings* settings) {
  FlGnomeSettings* self = FL_GNOME_SETTINGS(settings);

  FlColorScheme color_scheme = FL_COLOR_SCHEME_LIGHT;

  if (self->interface_settings != nullptr) {
    // check whether org.gnome.desktop.interface.gtk-theme ends with "-dark"
    g_autofree gchar* value =
        g_settings_get_string(self->interface_settings, kDesktopGtkThemeKey);
    if (g_str_has_suffix(value, kGtkThemeDarkSuffix)) {
      color_scheme = FL_COLOR_SCHEME_DARK;
    }
  }
  return color_scheme;
}

static gboolean fl_gnome_settings_get_enable_animations(FlSettings* settings) {
  return TRUE;
}

static gboolean fl_gnome_settings_get_high_contrast(FlSettings* settings) {
  return FALSE;
}

static gdouble fl_gnome_settings_get_text_scaling_factor(FlSettings* settings) {
  FlGnomeSettings* self = FL_GNOME_SETTINGS(settings);

  gdouble scaling_factor = 1.0;

  if (self->interface_settings != nullptr) {
    scaling_factor = g_settings_get_double(self->interface_settings,
                                           kDesktopTextScalingFactorKey);
  }
  return scaling_factor;
}

static void fl_gnome_settings_set_interface_settings(FlGnomeSettings* self,
                                                     GSettings* settings) {
  g_return_if_fail(G_IS_SETTINGS(settings));

  g_signal_connect_object(settings, "changed::clock-format",
                          G_CALLBACK(fl_settings_emit_changed), self,
                          G_CONNECT_SWAPPED);
  g_signal_connect_object(settings, "changed::gtk-theme",
                          G_CALLBACK(fl_settings_emit_changed), self,
                          G_CONNECT_SWAPPED);
  g_signal_connect_object(settings, "changed::text-scaling-factor",
                          G_CALLBACK(fl_settings_emit_changed), self,
                          G_CONNECT_SWAPPED);

  self->interface_settings = G_SETTINGS(g_object_ref(settings));
}

static void fl_gnome_settings_set_property(GObject* object,
                                           guint prop_id,
                                           const GValue* value,
                                           GParamSpec* pspec) {
  FlGnomeSettings* self = FL_GNOME_SETTINGS(object);
  switch (prop_id) {
    case PROP_INTERFACE_SETTINGS:
      fl_gnome_settings_set_interface_settings(
          self, G_SETTINGS(g_value_get_object(value)));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void fl_gnome_settings_dispose(GObject* object) {
  FlGnomeSettings* self = FL_GNOME_SETTINGS(object);

  g_clear_object(&self->interface_settings);

  G_OBJECT_CLASS(fl_gnome_settings_parent_class)->dispose(object);
}

static void fl_gnome_settings_class_init(FlGnomeSettingsClass* klass) {
  GObjectClass* object_class = G_OBJECT_CLASS(klass);
  object_class->dispose = fl_gnome_settings_dispose;
  object_class->set_property = fl_gnome_settings_set_property;

  g_object_class_install_property(
      object_class, PROP_INTERFACE_SETTINGS,
      g_param_spec_object(
          kInterfaceSettings, kInterfaceSettings, kDesktopInterfaceSchema,
          g_settings_get_type(),
          static_cast<GParamFlags>(G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY |
                                   G_PARAM_STATIC_STRINGS)));
}

static void fl_gnome_settings_iface_init(FlSettingsInterface* iface) {
  iface->get_clock_format = fl_gnome_settings_get_clock_format;
  iface->get_color_scheme = fl_gnome_settings_get_color_scheme;
  iface->get_enable_animations = fl_gnome_settings_get_enable_animations;
  iface->get_high_contrast = fl_gnome_settings_get_high_contrast;
  iface->get_text_scaling_factor = fl_gnome_settings_get_text_scaling_factor;
}

static void fl_gnome_settings_init(FlGnomeSettings* self) {}

static GSettings* create_settings(const gchar* schema_id) {
  GSettings* settings = nullptr;
  GSettingsSchemaSource* source = g_settings_schema_source_get_default();
  if (source != nullptr) {
    g_autoptr(GSettingsSchema) schema =
        g_settings_schema_source_lookup(source, schema_id, TRUE);
    if (schema != nullptr) {
      settings = g_settings_new_full(schema, nullptr, nullptr);
    }
  }
  return settings;
}

FlSettings* fl_gnome_settings_new() {
  g_autoptr(GSettings) interface_settings =
      create_settings(kDesktopInterfaceSchema);
  return FL_SETTINGS(g_object_new(fl_gnome_settings_get_type(),
                                  kInterfaceSettings, interface_settings,
                                  nullptr));
}

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_settings_portal.h"

#include <gio/gio.h>
#include <glib.h>

static constexpr char kPortalName[] = "org.freedesktop.portal.Desktop";
static constexpr char kPortalPath[] = "/org/freedesktop/portal/desktop";
static constexpr char kPortalSettings[] = "org.freedesktop.portal.Settings";

struct FlSetting {
  const gchar* ns;
  const gchar* key;
  const GVariantType* type;
};

static constexpr char kXdgAppearance[] = "org.freedesktop.appearance";
static const FlSetting kColorScheme = {
    kXdgAppearance,
    "color-scheme",
    G_VARIANT_TYPE_UINT32,
};

static constexpr char kGnomeA11yInterface[] =
    "org.gnome.desktop.a11y.interface";
static const FlSetting kHighContrast = {
    kGnomeA11yInterface,
    "high-contrast",
    G_VARIANT_TYPE_BOOLEAN,
};

static constexpr char kGnomeDesktopInterface[] = "org.gnome.desktop.interface";
static const FlSetting kClockFormat = {
    kGnomeDesktopInterface,
    "clock-format",
    G_VARIANT_TYPE_STRING,
};
static const FlSetting kEnableAnimations = {
    kGnomeDesktopInterface,
    "enable-animations",
    G_VARIANT_TYPE_BOOLEAN,
};
static const FlSetting kGtkTheme = {
    kGnomeDesktopInterface,
    "gtk-theme",
    G_VARIANT_TYPE_STRING,
};
static const FlSetting kTextScalingFactor = {
    kGnomeDesktopInterface,
    "text-scaling-factor",
    G_VARIANT_TYPE_DOUBLE,
};

static const FlSetting kAllSettings[] = {
    kClockFormat, kColorScheme,  kEnableAnimations,
    kGtkTheme,    kHighContrast, kTextScalingFactor,
};

static constexpr char kClockFormat12Hour[] = "12h";
static constexpr char kGtkThemeDarkSuffix[] = "-dark";

typedef enum {
  COLOR_SCHEME_DEFAULT,
  COLOR_SCHEME_PREFER_DARK,
  COLOR_SCHEME_PREFER_LIGHT
} ColorScheme;

struct _FlSettingsPortal {
  GObject parent_instance;

  GDBusProxy* dbus_proxy;
  GVariantDict* values;
};

static void fl_settings_portal_iface_init(FlSettingsInterface* iface);

G_DEFINE_TYPE_WITH_CODE(FlSettingsPortal,
                        fl_settings_portal,
                        G_TYPE_OBJECT,
                        G_IMPLEMENT_INTERFACE(fl_settings_get_type(),
                                              fl_settings_portal_iface_init))

static gchar* format_key(const FlSetting* setting) {
  return g_strconcat(setting->ns, "::", setting->key, nullptr);
}

static gboolean get_value(FlSettingsPortal* portal,
                          const FlSetting* setting,
                          GVariant** value) {
  g_autofree const gchar* key = format_key(setting);
  *value = g_variant_dict_lookup_value(portal->values, key, setting->type);
  return *value != nullptr;
}

static void set_value(FlSettingsPortal* portal,
                      const FlSetting* setting,
                      GVariant* value) {
  g_autofree const gchar* key = format_key(setting);

  // ignore redundant changes from multiple XDG desktop portal backends
  g_autoptr(GVariant) old_value =
      g_variant_dict_lookup_value(portal->values, key, nullptr);
  if (old_value != nullptr && value != nullptr &&
      g_variant_equal(old_value, value)) {
    return;
  }

  g_variant_dict_insert_value(portal->values, key, value);
  fl_settings_emit_changed(FL_SETTINGS(portal));
}

// Based on
// https://gitlab.gnome.org/GNOME/Initiatives/-/wikis/Dark-Style-Preference#other
static gboolean settings_portal_read(GDBusProxy* proxy,
                                     const gchar* ns,
                                     const gchar* key,
                                     GVariant** out) {
  g_autoptr(GError) error = nullptr;
  g_autoptr(GVariant) value =
      g_dbus_proxy_call_sync(proxy, "Read", g_variant_new("(ss)", ns, key),
                             G_DBUS_CALL_FLAGS_NONE, G_MAXINT, nullptr, &error);

  if (error) {
    if (error->domain == G_DBUS_ERROR &&
        error->code == G_DBUS_ERROR_SERVICE_UNKNOWN) {
      g_debug("XDG desktop portal unavailable: %s", error->message);
      return FALSE;
    }

    if (error->domain == G_DBUS_ERROR &&
        error->code == G_DBUS_ERROR_UNKNOWN_METHOD) {
      g_debug("XDG desktop portal settings unavailable: %s", error->message);
      return FALSE;
    }

    g_critical("Failed to read XDG desktop portal settings: %s",
               error->message);
    return FALSE;
  }

  g_autoptr(GVariant) child = nullptr;
  g_variant_get(value, "(v)", &child);
  g_variant_get(child, "v", out);

  return TRUE;
}

static void settings_portal_changed_cb(GDBusProxy* proxy,
                                       const char* sender_name,
                                       const char* signal_name,
                                       GVariant* parameters,
                                       gpointer user_data) {
  FlSettingsPortal* portal = FL_SETTINGS_PORTAL(user_data);
  if (g_strcmp0(signal_name, "SettingChanged")) {
    return;
  }

  FlSetting setting;
  g_autoptr(GVariant) value = nullptr;
  g_variant_get(parameters, "(&s&sv)", &setting.ns, &setting.key, &value);
  set_value(portal, &setting, value);
}

static FlClockFormat fl_settings_portal_get_clock_format(FlSettings* settings) {
  FlSettingsPortal* self = FL_SETTINGS_PORTAL(settings);

  FlClockFormat clock_format = FL_CLOCK_FORMAT_24H;

  g_autoptr(GVariant) value = nullptr;
  if (get_value(self, &kClockFormat, &value)) {
    const gchar* clock_format_str = g_variant_get_string(value, nullptr);
    if (g_strcmp0(clock_format_str, kClockFormat12Hour) == 0) {
      clock_format = FL_CLOCK_FORMAT_12H;
    }
  }

  return clock_format;
}

static FlColorScheme fl_settings_portal_get_color_scheme(FlSettings* settings) {
  FlSettingsPortal* self = FL_SETTINGS_PORTAL(settings);

  FlColorScheme color_scheme = FL_COLOR_SCHEME_LIGHT;

  g_autoptr(GVariant) value = nullptr;
  if (get_value(self, &kColorScheme, &value)) {
    if (g_variant_get_uint32(value) == COLOR_SCHEME_PREFER_DARK) {
      color_scheme = FL_COLOR_SCHEME_DARK;
    }
  } else if (get_value(self, &kGtkTheme, &value)) {
    const gchar* gtk_theme_str = g_variant_get_string(value, nullptr);
    if (g_str_has_suffix(gtk_theme_str, kGtkThemeDarkSuffix)) {
      color_scheme = FL_COLOR_SCHEME_DARK;
    }
  }

  return color_scheme;
}

static gboolean fl_settings_portal_get_enable_animations(FlSettings* settings) {
  FlSettingsPortal* self = FL_SETTINGS_PORTAL(settings);

  gboolean enable_animations = true;

  g_autoptr(GVariant) value = nullptr;
  if (get_value(self, &kEnableAnimations, &value)) {
    enable_animations = g_variant_get_boolean(value);
  }

  return enable_animations;
}

static gboolean fl_settings_portal_get_high_contrast(FlSettings* settings) {
  FlSettingsPortal* self = FL_SETTINGS_PORTAL(settings);

  gboolean high_contrast = false;

  g_autoptr(GVariant) value = nullptr;
  if (get_value(self, &kHighContrast, &value)) {
    high_contrast = g_variant_get_boolean(value);
  }

  return high_contrast;
}

static gdouble fl_settings_portal_get_text_scaling_factor(
    FlSettings* settings) {
  FlSettingsPortal* self = FL_SETTINGS_PORTAL(settings);

  gdouble scaling_factor = 1.0;

  g_autoptr(GVariant) value = nullptr;
  if (get_value(self, &kTextScalingFactor, &value)) {
    scaling_factor = g_variant_get_double(value);
  }

  return scaling_factor;
}

static void fl_settings_portal_dispose(GObject* object) {
  FlSettingsPortal* self = FL_SETTINGS_PORTAL(object);

  g_clear_object(&self->dbus_proxy);
  g_clear_pointer(&self->values, g_variant_dict_unref);

  G_OBJECT_CLASS(fl_settings_portal_parent_class)->dispose(object);
}

static void fl_settings_portal_class_init(FlSettingsPortalClass* klass) {
  GObjectClass* object_class = G_OBJECT_CLASS(klass);
  object_class->dispose = fl_settings_portal_dispose;
}

static void fl_settings_portal_iface_init(FlSettingsInterface* iface) {
  iface->get_clock_format = fl_settings_portal_get_clock_format;
  iface->get_color_scheme = fl_settings_portal_get_color_scheme;
  iface->get_enable_animations = fl_settings_portal_get_enable_animations;
  iface->get_high_contrast = fl_settings_portal_get_high_contrast;
  iface->get_text_scaling_factor = fl_settings_portal_get_text_scaling_factor;
}

static void fl_settings_portal_init(FlSettingsPortal* self) {}

FlSettingsPortal* fl_settings_portal_new() {
  g_autoptr(GVariantDict) values = g_variant_dict_new(nullptr);
  return fl_settings_portal_new_with_values(values);
}

FlSettingsPortal* fl_settings_portal_new_with_values(GVariantDict* values) {
  g_return_val_if_fail(values != nullptr, nullptr);
  FlSettingsPortal* portal =
      FL_SETTINGS_PORTAL(g_object_new(fl_settings_portal_get_type(), nullptr));
  portal->values = g_variant_dict_ref(values);
  return portal;
}

gboolean fl_settings_portal_start(FlSettingsPortal* self, GError** error) {
  g_return_val_if_fail(FL_IS_SETTINGS_PORTAL(self), false);
  g_return_val_if_fail(self->dbus_proxy == nullptr, false);

  self->dbus_proxy = g_dbus_proxy_new_for_bus_sync(
      G_BUS_TYPE_SESSION, G_DBUS_PROXY_FLAGS_NONE, nullptr, kPortalName,
      kPortalPath, kPortalSettings, nullptr, error);

  if (self->dbus_proxy == nullptr) {
    return FALSE;
  }

  for (const FlSetting setting : kAllSettings) {
    g_autoptr(GVariant) value = nullptr;
    if (settings_portal_read(self->dbus_proxy, setting.ns, setting.key,
                             &value)) {
      set_value(self, &setting, value);
    }
  }

  g_signal_connect_object(self->dbus_proxy, "g-signal",
                          G_CALLBACK(settings_portal_changed_cb), self,
                          static_cast<GConnectFlags>(0));

  return true;
}

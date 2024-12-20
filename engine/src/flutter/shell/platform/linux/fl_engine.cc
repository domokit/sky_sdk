// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

#include <gmodule.h>

#include <cstring>

#include "flutter/shell/platform/common/engine_switches.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/fl_binary_messenger_private.h"
#include "flutter/shell/platform/linux/fl_dart_project_private.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/fl_pixel_buffer_texture_private.h"
#include "flutter/shell/platform/linux/fl_platform_handler.h"
#include "flutter/shell/platform/linux/fl_plugin_registrar_private.h"
#include "flutter/shell/platform/linux/fl_renderer.h"
#include "flutter/shell/platform/linux/fl_renderer_gdk.h"
#include "flutter/shell/platform/linux/fl_renderer_headless.h"
#include "flutter/shell/platform/linux/fl_settings_handler.h"
#include "flutter/shell/platform/linux/fl_texture_gl_private.h"
#include "flutter/shell/platform/linux/fl_texture_registrar_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_plugin_registry.h"

// Unique number associated with platform tasks.
static constexpr size_t kPlatformTaskRunnerIdentifier = 1;

// Use different device ID for mouse and pan/zoom events, since we can't
// differentiate the actual device (mouse v.s. trackpad)
static constexpr int32_t kMousePointerDeviceId = 0;
static constexpr int32_t kPointerPanZoomDeviceId = 1;

struct _FlEngine {
  GObject parent_instance;

  // Thread the GLib main loop is running on.
  GThread* thread;

  // The project this engine is running.
  FlDartProject* project;

  // Renders the Flutter app.
  FlRenderer* renderer;

  // Messenger used to send and receive platform messages.
  FlBinaryMessenger* binary_messenger;

  // Implements the flutter/settings channel.
  FlSettingsHandler* settings_handler;

  // Implements the flutter/platform channel.
  FlPlatformHandler* platform_handler;

  // Implements the flutter/mousecursor channel.
  FlMouseCursorHandler* mouse_cursor_handler;

  // Manages textures rendered by native code.
  FlTextureRegistrar* texture_registrar;

  // Schedules tasks to be run on the appropriate thread.
  FlTaskRunner* task_runner;

  // Ahead of time data used to make engine run faster.
  FlutterEngineAOTData aot_data;

  // The Flutter engine.
  FLUTTER_API_SYMBOL(FlutterEngine) engine;

  // Function table for engine API, used to intercept engine calls for testing
  // purposes.
  FlutterEngineProcTable embedder_api;

  // Next ID to use for a view.
  FlutterViewId next_view_id;

  // Function to call when a platform message is received.
  FlEnginePlatformMessageHandler platform_message_handler;
  gpointer platform_message_handler_data;
  GDestroyNotify platform_message_handler_destroy_notify;

  // Function to call when a semantic node is received.
  FlEngineUpdateSemanticsHandler update_semantics_handler;
  gpointer update_semantics_handler_data;
  GDestroyNotify update_semantics_handler_destroy_notify;
};

G_DEFINE_QUARK(fl_engine_error_quark, fl_engine_error)

static void fl_engine_plugin_registry_iface_init(
    FlPluginRegistryInterface* iface);

enum { SIGNAL_ON_PRE_ENGINE_RESTART, LAST_SIGNAL };

static guint fl_engine_signals[LAST_SIGNAL];

G_DEFINE_TYPE_WITH_CODE(
    FlEngine,
    fl_engine,
    G_TYPE_OBJECT,
    G_IMPLEMENT_INTERFACE(fl_plugin_registry_get_type(),
                          fl_engine_plugin_registry_iface_init))

enum { PROP_0, PROP_BINARY_MESSENGER, PROP_LAST };

// Parse a locale into its components.
static void parse_locale(const gchar* locale,
                         gchar** language,
                         gchar** territory,
                         gchar** codeset,
                         gchar** modifier) {
  gchar* l = g_strdup(locale);

  // Locales are in the form "language[_territory][.codeset][@modifier]"
  gchar* match = strrchr(l, '@');
  if (match != nullptr) {
    if (modifier != nullptr) {
      *modifier = g_strdup(match + 1);
    }
    *match = '\0';
  } else if (modifier != nullptr) {
    *modifier = nullptr;
  }

  match = strrchr(l, '.');
  if (match != nullptr) {
    if (codeset != nullptr) {
      *codeset = g_strdup(match + 1);
    }
    *match = '\0';
  } else if (codeset != nullptr) {
    *codeset = nullptr;
  }

  match = strrchr(l, '_');
  if (match != nullptr) {
    if (territory != nullptr) {
      *territory = g_strdup(match + 1);
    }
    *match = '\0';
  } else if (territory != nullptr) {
    *territory = nullptr;
  }

  if (language != nullptr) {
    *language = l;
  }
}

static void view_added_cb(const FlutterAddViewResult* result) {
  g_autoptr(GTask) task = G_TASK(result->user_data);

  if (result->added) {
    g_task_return_boolean(task, TRUE);
  } else {
    g_task_return_new_error(task, fl_engine_error_quark(),
                            FL_ENGINE_ERROR_FAILED, "Failed to add view");
  }
}

static void view_removed_cb(const FlutterRemoveViewResult* result) {
  g_autoptr(GTask) task = G_TASK(result->user_data);

  if (result->removed) {
    g_task_return_boolean(task, TRUE);
  } else {
    g_task_return_new_error(task, fl_engine_error_quark(),
                            FL_ENGINE_ERROR_FAILED, "Failed to remove view");
  }
}

static void free_locale(FlutterLocale* locale) {
  free(const_cast<gchar*>(locale->language_code));
  free(const_cast<gchar*>(locale->country_code));
  free(locale);
}

// Passes locale information to the Flutter engine.
static void setup_locales(FlEngine* self) {
  const gchar* const* languages = g_get_language_names();
  g_autoptr(GPtrArray) locales_array = g_ptr_array_new_with_free_func(
      reinterpret_cast<GDestroyNotify>(free_locale));
  for (int i = 0; languages[i] != nullptr; i++) {
    g_autofree gchar* locale_string = g_strstrip(g_strdup(languages[i]));

    // Ignore empty locales, caused by settings like `LANGUAGE=pt_BR:`
    if (strcmp(locale_string, "") == 0) {
      continue;
    }

    g_autofree gchar* language = nullptr;
    g_autofree gchar* territory = nullptr;
    parse_locale(locale_string, &language, &territory, nullptr, nullptr);

    // Ignore duplicate locales, caused by settings like `LANGUAGE=C` (returns
    // two "C") or `LANGUAGE=en:en`
    gboolean has_locale = FALSE;
    for (guint j = 0; !has_locale && j < locales_array->len; j++) {
      FlutterLocale* locale =
          reinterpret_cast<FlutterLocale*>(g_ptr_array_index(locales_array, j));
      has_locale = g_strcmp0(locale->language_code, language) == 0 &&
                   g_strcmp0(locale->country_code, territory) == 0;
    }
    if (has_locale) {
      continue;
    }

    FlutterLocale* locale =
        static_cast<FlutterLocale*>(g_malloc0(sizeof(FlutterLocale)));
    g_ptr_array_add(locales_array, locale);
    locale->struct_size = sizeof(FlutterLocale);
    locale->language_code =
        reinterpret_cast<const gchar*>(g_steal_pointer(&language));
    locale->country_code =
        reinterpret_cast<const gchar*>(g_steal_pointer(&territory));
    locale->script_code = nullptr;
    locale->variant_code = nullptr;
  }
  FlutterLocale** locales =
      reinterpret_cast<FlutterLocale**>(locales_array->pdata);
  FlutterEngineResult result = self->embedder_api.UpdateLocales(
      self->engine, const_cast<const FlutterLocale**>(locales),
      locales_array->len);
  if (result != kSuccess) {
    g_warning("Failed to set up Flutter locales");
  }
}

// Called when engine needs a backing store for a specific #FlutterLayer.
static bool compositor_create_backing_store_callback(
    const FlutterBackingStoreConfig* config,
    FlutterBackingStore* backing_store_out,
    void* user_data) {
  g_return_val_if_fail(FL_IS_RENDERER(user_data), false);
  return fl_renderer_create_backing_store(FL_RENDERER(user_data), config,
                                          backing_store_out);
}

// Called when the backing store is to be released.
static bool compositor_collect_backing_store_callback(
    const FlutterBackingStore* backing_store,
    void* user_data) {
  g_return_val_if_fail(FL_IS_RENDERER(user_data), false);
  return fl_renderer_collect_backing_store(FL_RENDERER(user_data),
                                           backing_store);
}

// Called when embedder should composite contents of each layer onto the screen.
static bool compositor_present_view_callback(
    const FlutterPresentViewInfo* info) {
  g_return_val_if_fail(FL_IS_RENDERER(info->user_data), false);
  return fl_renderer_present_layers(FL_RENDERER(info->user_data), info->view_id,
                                    info->layers, info->layers_count);
}

// Flutter engine rendering callbacks.

static void* fl_engine_gl_proc_resolver(void* user_data, const char* name) {
  FlEngine* self = static_cast<FlEngine*>(user_data);
  return fl_renderer_get_proc_address(self->renderer, name);
}

static bool fl_engine_gl_make_current(void* user_data) {
  FlEngine* self = static_cast<FlEngine*>(user_data);
  fl_renderer_make_current(self->renderer);
  return true;
}

static bool fl_engine_gl_clear_current(void* user_data) {
  FlEngine* self = static_cast<FlEngine*>(user_data);
  fl_renderer_clear_current(self->renderer);
  return true;
}

static uint32_t fl_engine_gl_get_fbo(void* user_data) {
  FlEngine* self = static_cast<FlEngine*>(user_data);
  return fl_renderer_get_fbo(self->renderer);
}

static bool fl_engine_gl_present(void* user_data) {
  // No action required, as this is handled in
  // compositor_present_view_callback.
  return true;
}

static bool fl_engine_gl_make_resource_current(void* user_data) {
  FlEngine* self = static_cast<FlEngine*>(user_data);
  fl_renderer_make_resource_current(self->renderer);
  return true;
}

// Called by the engine to retrieve an external texture.
static bool fl_engine_gl_external_texture_frame_callback(
    void* user_data,
    int64_t texture_id,
    size_t width,
    size_t height,
    FlutterOpenGLTexture* opengl_texture) {
  FlEngine* self = static_cast<FlEngine*>(user_data);
  if (!self->texture_registrar) {
    return false;
  }

  FlTexture* texture =
      fl_texture_registrar_lookup_texture(self->texture_registrar, texture_id);
  if (texture == nullptr) {
    g_warning("Unable to find texture %" G_GINT64_FORMAT, texture_id);
    return false;
  }

  gboolean result;
  g_autoptr(GError) error = nullptr;
  if (FL_IS_TEXTURE_GL(texture)) {
    result = fl_texture_gl_populate(FL_TEXTURE_GL(texture), width, height,
                                    opengl_texture, &error);
  } else if (FL_IS_PIXEL_BUFFER_TEXTURE(texture)) {
    result =
        fl_pixel_buffer_texture_populate(FL_PIXEL_BUFFER_TEXTURE(texture),
                                         width, height, opengl_texture, &error);
  } else {
    g_warning("Unsupported texture type %" G_GINT64_FORMAT, texture_id);
    return false;
  }

  if (!result) {
    g_warning("%s", error->message);
    return false;
  }

  return true;
}

// Called by the engine to determine if it is on the GTK thread.
static bool fl_engine_runs_task_on_current_thread(void* user_data) {
  FlEngine* self = static_cast<FlEngine*>(user_data);
  return self->thread == g_thread_self();
}

// Called when the engine has a task to perform in the GTK thread.
static void fl_engine_post_task(FlutterTask task,
                                uint64_t target_time_nanos,
                                void* user_data) {
  FlEngine* self = static_cast<FlEngine*>(user_data);

  fl_task_runner_post_task(self->task_runner, task, target_time_nanos);
}

// Called when a platform message is received from the engine.
static void fl_engine_platform_message_cb(const FlutterPlatformMessage* message,
                                          void* user_data) {
  FlEngine* self = FL_ENGINE(user_data);

  gboolean handled = FALSE;
  if (self->platform_message_handler != nullptr) {
    g_autoptr(GBytes) data =
        g_bytes_new(message->message, message->message_size);
    handled = self->platform_message_handler(
        self, message->channel, data, message->response_handle,
        self->platform_message_handler_data);
  }

  if (!handled) {
    fl_engine_send_platform_message_response(self, message->response_handle,
                                             nullptr, nullptr);
  }
}

// Called when a semantic node update is received from the engine.
static void fl_engine_update_semantics_cb(const FlutterSemanticsUpdate2* update,
                                          void* user_data) {
  FlEngine* self = FL_ENGINE(user_data);

  if (self->update_semantics_handler != nullptr) {
    self->update_semantics_handler(self, update,
                                   self->update_semantics_handler_data);
  }
}

// Called right before the engine is restarted.
//
// This method should reset states to as if the engine has just been started,
// which usually indicates the user has requested a hot restart (Shift-R in the
// Flutter CLI.)
static void fl_engine_on_pre_engine_restart_cb(void* user_data) {
  FlEngine* self = FL_ENGINE(user_data);

  g_signal_emit(self, fl_engine_signals[SIGNAL_ON_PRE_ENGINE_RESTART], 0);
}

// Called when a response to a sent platform message is received from the
// engine.
static void fl_engine_platform_message_response_cb(const uint8_t* data,
                                                   size_t data_length,
                                                   void* user_data) {
  g_autoptr(GTask) task = G_TASK(user_data);
  g_task_return_pointer(task, g_bytes_new(data, data_length),
                        reinterpret_cast<GDestroyNotify>(g_bytes_unref));
}

// Implements FlPluginRegistry::get_registrar_for_plugin.
static FlPluginRegistrar* fl_engine_get_registrar_for_plugin(
    FlPluginRegistry* registry,
    const gchar* name) {
  FlEngine* self = FL_ENGINE(registry);

  return fl_plugin_registrar_new(nullptr, self->binary_messenger,
                                 self->texture_registrar);
}

static void fl_engine_plugin_registry_iface_init(
    FlPluginRegistryInterface* iface) {
  iface->get_registrar_for_plugin = fl_engine_get_registrar_for_plugin;
}

static void fl_engine_set_property(GObject* object,
                                   guint prop_id,
                                   const GValue* value,
                                   GParamSpec* pspec) {
  FlEngine* self = FL_ENGINE(object);
  switch (prop_id) {
    case PROP_BINARY_MESSENGER:
      g_set_object(&self->binary_messenger,
                   FL_BINARY_MESSENGER(g_value_get_object(value)));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void fl_engine_dispose(GObject* object) {
  FlEngine* self = FL_ENGINE(object);

  if (self->engine != nullptr) {
    self->embedder_api.Shutdown(self->engine);
    self->engine = nullptr;
  }

  if (self->aot_data != nullptr) {
    self->embedder_api.CollectAOTData(self->aot_data);
    self->aot_data = nullptr;
  }

  fl_binary_messenger_shutdown(self->binary_messenger);
  fl_texture_registrar_shutdown(self->texture_registrar);

  g_clear_object(&self->project);
  g_clear_object(&self->renderer);
  g_clear_object(&self->texture_registrar);
  g_clear_object(&self->binary_messenger);
  g_clear_object(&self->settings_handler);
  g_clear_object(&self->platform_handler);
  g_clear_object(&self->mouse_cursor_handler);
  g_clear_object(&self->task_runner);

  if (self->platform_message_handler_destroy_notify) {
    self->platform_message_handler_destroy_notify(
        self->platform_message_handler_data);
  }
  self->platform_message_handler_data = nullptr;
  self->platform_message_handler_destroy_notify = nullptr;

  if (self->update_semantics_handler_destroy_notify) {
    self->update_semantics_handler_destroy_notify(
        self->update_semantics_handler_data);
  }
  self->update_semantics_handler_data = nullptr;
  self->update_semantics_handler_destroy_notify = nullptr;

  G_OBJECT_CLASS(fl_engine_parent_class)->dispose(object);
}

static void fl_engine_class_init(FlEngineClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = fl_engine_dispose;
  G_OBJECT_CLASS(klass)->set_property = fl_engine_set_property;

  g_object_class_install_property(
      G_OBJECT_CLASS(klass), PROP_BINARY_MESSENGER,
      g_param_spec_object(
          "binary-messenger", "messenger", "Binary messenger",
          fl_binary_messenger_get_type(),
          static_cast<GParamFlags>(G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY |
                                   G_PARAM_STATIC_STRINGS)));

  fl_engine_signals[SIGNAL_ON_PRE_ENGINE_RESTART] = g_signal_new(
      "on-pre-engine-restart", fl_engine_get_type(), G_SIGNAL_RUN_LAST, 0,
      nullptr, nullptr, nullptr, G_TYPE_NONE, 0);
}

static void fl_engine_init(FlEngine* self) {
  self->thread = g_thread_self();

  self->embedder_api.struct_size = sizeof(FlutterEngineProcTable);
  if (FlutterEngineGetProcAddresses(&self->embedder_api) != kSuccess) {
    g_warning("Failed get get engine function pointers");
  }

  // Implicit view is 0, so start at 1.
  self->next_view_id = 1;

  self->texture_registrar = fl_texture_registrar_new(self);
}

FlEngine* fl_engine_new_with_renderer(FlDartProject* project,
                                      FlRenderer* renderer) {
  g_return_val_if_fail(FL_IS_DART_PROJECT(project), nullptr);
  g_return_val_if_fail(FL_IS_RENDERER(renderer), nullptr);

  FlEngine* self = FL_ENGINE(g_object_new(fl_engine_get_type(), nullptr));
  self->project = FL_DART_PROJECT(g_object_ref(project));
  self->renderer = FL_RENDERER(g_object_ref(renderer));
  self->binary_messenger = fl_binary_messenger_new(self);

  fl_renderer_set_engine(self->renderer, self);

  return self;
}

G_MODULE_EXPORT FlEngine* fl_engine_new(FlDartProject* project) {
  g_autoptr(FlRendererGdk) renderer = fl_renderer_gdk_new();
  return fl_engine_new_with_renderer(project, FL_RENDERER(renderer));
}

G_MODULE_EXPORT FlEngine* fl_engine_new_headless(FlDartProject* project) {
  g_autoptr(FlRendererHeadless) renderer = fl_renderer_headless_new();
  return fl_engine_new_with_renderer(project, FL_RENDERER(renderer));
}

FlRenderer* fl_engine_get_renderer(FlEngine* self) {
  g_return_val_if_fail(FL_IS_ENGINE(self), nullptr);
  return self->renderer;
}

gboolean fl_engine_start(FlEngine* self, GError** error) {
  g_return_val_if_fail(FL_IS_ENGINE(self), FALSE);

  self->task_runner = fl_task_runner_new(self);

  FlutterRendererConfig config = {};
  config.type = kOpenGL;
  config.open_gl.struct_size = sizeof(FlutterOpenGLRendererConfig);
  config.open_gl.gl_proc_resolver = fl_engine_gl_proc_resolver;
  config.open_gl.make_current = fl_engine_gl_make_current;
  config.open_gl.clear_current = fl_engine_gl_clear_current;
  config.open_gl.fbo_callback = fl_engine_gl_get_fbo;
  config.open_gl.present = fl_engine_gl_present;
  config.open_gl.make_resource_current = fl_engine_gl_make_resource_current;
  config.open_gl.gl_external_texture_frame_callback =
      fl_engine_gl_external_texture_frame_callback;

  FlutterTaskRunnerDescription platform_task_runner = {};
  platform_task_runner.struct_size = sizeof(FlutterTaskRunnerDescription);
  platform_task_runner.user_data = self;
  platform_task_runner.runs_task_on_current_thread_callback =
      fl_engine_runs_task_on_current_thread;
  platform_task_runner.post_task_callback = fl_engine_post_task;
  platform_task_runner.identifier = kPlatformTaskRunnerIdentifier;

  FlutterCustomTaskRunners custom_task_runners = {};
  custom_task_runners.struct_size = sizeof(FlutterCustomTaskRunners);
  custom_task_runners.platform_task_runner = &platform_task_runner;
  custom_task_runners.render_task_runner = &platform_task_runner;

  g_autoptr(GPtrArray) command_line_args =
      g_ptr_array_new_with_free_func(g_free);
  g_ptr_array_insert(command_line_args, 0, g_strdup("flutter"));
  for (const auto& env_switch : flutter::GetSwitchesFromEnvironment()) {
    g_ptr_array_add(command_line_args, g_strdup(env_switch.c_str()));
  }

  gchar** dart_entrypoint_args =
      fl_dart_project_get_dart_entrypoint_arguments(self->project);

  FlutterProjectArgs args = {};
  args.struct_size = sizeof(FlutterProjectArgs);
  args.assets_path = fl_dart_project_get_assets_path(self->project);
  args.icu_data_path = fl_dart_project_get_icu_data_path(self->project);
  args.command_line_argc = command_line_args->len;
  args.command_line_argv =
      reinterpret_cast<const char* const*>(command_line_args->pdata);
  args.platform_message_callback = fl_engine_platform_message_cb;
  args.update_semantics_callback2 = fl_engine_update_semantics_cb;
  args.custom_task_runners = &custom_task_runners;
  args.shutdown_dart_vm_when_done = true;
  args.on_pre_engine_restart_callback = fl_engine_on_pre_engine_restart_cb;
  args.dart_entrypoint_argc =
      dart_entrypoint_args != nullptr ? g_strv_length(dart_entrypoint_args) : 0;
  args.dart_entrypoint_argv =
      reinterpret_cast<const char* const*>(dart_entrypoint_args);

  FlutterCompositor compositor = {};
  compositor.struct_size = sizeof(FlutterCompositor);
  compositor.user_data = self->renderer;
  compositor.create_backing_store_callback =
      compositor_create_backing_store_callback;
  compositor.collect_backing_store_callback =
      compositor_collect_backing_store_callback;
  compositor.present_view_callback = compositor_present_view_callback;
  args.compositor = &compositor;

  if (self->embedder_api.RunsAOTCompiledDartCode()) {
    FlutterEngineAOTDataSource source = {};
    source.type = kFlutterEngineAOTDataSourceTypeElfPath;
    source.elf_path = fl_dart_project_get_aot_library_path(self->project);
    if (self->embedder_api.CreateAOTData(&source, &self->aot_data) !=
        kSuccess) {
      g_set_error(error, fl_engine_error_quark(), FL_ENGINE_ERROR_FAILED,
                  "Failed to create AOT data");
      return FALSE;
    }
    args.aot_data = self->aot_data;
  }

  FlutterEngineResult result = self->embedder_api.Initialize(
      FLUTTER_ENGINE_VERSION, &config, &args, self, &self->engine);
  if (result != kSuccess) {
    g_set_error(error, fl_engine_error_quark(), FL_ENGINE_ERROR_FAILED,
                "Failed to initialize Flutter engine");
    return FALSE;
  }

  result = self->embedder_api.RunInitialized(self->engine);
  if (result != kSuccess) {
    g_set_error(error, fl_engine_error_quark(), FL_ENGINE_ERROR_FAILED,
                "Failed to run Flutter engine");
    return FALSE;
  }

  setup_locales(self);

  g_autoptr(FlSettings) settings = fl_settings_new();
  self->settings_handler = fl_settings_handler_new(self);
  fl_settings_handler_start(self->settings_handler, settings);

  self->platform_handler = fl_platform_handler_new(self->binary_messenger);
  self->mouse_cursor_handler =
      fl_mouse_cursor_handler_new(self->binary_messenger);

  result = self->embedder_api.UpdateSemanticsEnabled(self->engine, TRUE);
  if (result != kSuccess) {
    g_warning("Failed to enable accessibility features on Flutter engine");
  }

  gdouble refresh_rate = fl_renderer_get_refresh_rate(self->renderer);
  // FlutterEngineDisplay::refresh_rate expects 0 if the refresh rate is
  // unknown.
  if (refresh_rate <= 0.0) {
    refresh_rate = 0.0;
  }
  FlutterEngineDisplay display = {};
  display.struct_size = sizeof(FlutterEngineDisplay);
  display.display_id = 0;
  display.single_display = true;
  display.refresh_rate = refresh_rate;

  result = self->embedder_api.NotifyDisplayUpdate(
      self->engine, kFlutterEngineDisplaysUpdateTypeStartup, &display, 1);
  if (result != kSuccess) {
    g_warning("Failed to notify display update to Flutter engine: %d", result);
  }

  return TRUE;
}

FlutterEngineProcTable* fl_engine_get_embedder_api(FlEngine* self) {
  return &(self->embedder_api);
}

FlutterViewId fl_engine_add_view(FlEngine* self,
                                 size_t width,
                                 size_t height,
                                 double pixel_ratio,
                                 GCancellable* cancellable,
                                 GAsyncReadyCallback callback,
                                 gpointer user_data) {
  g_return_val_if_fail(FL_IS_ENGINE(self), -1);

  g_autoptr(GTask) task = g_task_new(self, cancellable, callback, user_data);

  FlutterViewId view_id = self->next_view_id;
  self->next_view_id++;

  FlutterWindowMetricsEvent metrics;
  metrics.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics.width = width;
  metrics.height = height;
  metrics.pixel_ratio = pixel_ratio;
  metrics.view_id = view_id;
  FlutterAddViewInfo info;
  info.struct_size = sizeof(FlutterAddViewInfo);
  info.view_id = view_id;
  info.view_metrics = &metrics;
  info.user_data = g_object_ref(task);
  info.add_view_callback = view_added_cb;
  FlutterEngineResult result = self->embedder_api.AddView(self->engine, &info);
  if (result != kSuccess) {
    g_task_return_new_error(task, fl_engine_error_quark(),
                            FL_ENGINE_ERROR_FAILED, "AddView returned %d",
                            result);
    // This would have been done in the callback, but that won't occur now.
    g_object_unref(task);
  }

  return view_id;
}

gboolean fl_engine_add_view_finish(FlEngine* self,
                                   GAsyncResult* result,
                                   GError** error) {
  g_return_val_if_fail(FL_IS_ENGINE(self), FALSE);
  return g_task_propagate_boolean(G_TASK(result), error);
}

void fl_engine_remove_view(FlEngine* self,
                           FlutterViewId view_id,
                           GCancellable* cancellable,
                           GAsyncReadyCallback callback,
                           gpointer user_data) {
  g_return_if_fail(FL_IS_ENGINE(self));

  g_autoptr(GTask) task = g_task_new(self, cancellable, callback, user_data);

  FlutterRemoveViewInfo info;
  info.struct_size = sizeof(FlutterRemoveViewInfo);
  info.view_id = view_id;
  info.user_data = g_object_ref(task);
  info.remove_view_callback = view_removed_cb;
  FlutterEngineResult result =
      self->embedder_api.RemoveView(self->engine, &info);
  if (result != kSuccess) {
    g_task_return_new_error(task, fl_engine_error_quark(),
                            FL_ENGINE_ERROR_FAILED, "RemoveView returned %d",
                            result);
    // This would have been done in the callback, but that won't occur now.
    g_object_unref(task);
  }
}

gboolean fl_engine_remove_view_finish(FlEngine* self,
                                      GAsyncResult* result,
                                      GError** error) {
  g_return_val_if_fail(FL_IS_ENGINE(self), FALSE);
  return g_task_propagate_boolean(G_TASK(result), error);
}

void fl_engine_set_platform_message_handler(
    FlEngine* self,
    FlEnginePlatformMessageHandler handler,
    gpointer user_data,
    GDestroyNotify destroy_notify) {
  g_return_if_fail(FL_IS_ENGINE(self));
  g_return_if_fail(handler != nullptr);

  if (self->platform_message_handler_destroy_notify) {
    self->platform_message_handler_destroy_notify(
        self->platform_message_handler_data);
  }

  self->platform_message_handler = handler;
  self->platform_message_handler_data = user_data;
  self->platform_message_handler_destroy_notify = destroy_notify;
}

void fl_engine_set_update_semantics_handler(
    FlEngine* self,
    FlEngineUpdateSemanticsHandler handler,
    gpointer user_data,
    GDestroyNotify destroy_notify) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->update_semantics_handler_destroy_notify) {
    self->update_semantics_handler_destroy_notify(
        self->update_semantics_handler_data);
  }

  self->update_semantics_handler = handler;
  self->update_semantics_handler_data = user_data;
  self->update_semantics_handler_destroy_notify = destroy_notify;
}

// Note: This function can be called from any thread.
gboolean fl_engine_send_platform_message_response(
    FlEngine* self,
    const FlutterPlatformMessageResponseHandle* handle,
    GBytes* response,
    GError** error) {
  g_return_val_if_fail(FL_IS_ENGINE(self), FALSE);
  g_return_val_if_fail(handle != nullptr, FALSE);

  if (self->engine == nullptr) {
    g_set_error(error, fl_engine_error_quark(), FL_ENGINE_ERROR_FAILED,
                "No engine to send response to");
    return FALSE;
  }

  gsize data_length = 0;
  const uint8_t* data = nullptr;
  if (response != nullptr) {
    data =
        static_cast<const uint8_t*>(g_bytes_get_data(response, &data_length));
  }
  FlutterEngineResult result = self->embedder_api.SendPlatformMessageResponse(
      self->engine, handle, data, data_length);

  if (result != kSuccess) {
    g_set_error(error, fl_engine_error_quark(), FL_ENGINE_ERROR_FAILED,
                "Failed to send platform message response");
    return FALSE;
  }

  return TRUE;
}

void fl_engine_send_platform_message(FlEngine* self,
                                     const gchar* channel,
                                     GBytes* message,
                                     GCancellable* cancellable,
                                     GAsyncReadyCallback callback,
                                     gpointer user_data) {
  g_return_if_fail(FL_IS_ENGINE(self));

  GTask* task = nullptr;
  FlutterPlatformMessageResponseHandle* response_handle = nullptr;
  if (callback != nullptr) {
    task = g_task_new(self, cancellable, callback, user_data);

    if (self->engine == nullptr) {
      g_task_return_new_error(task, fl_engine_error_quark(),
                              FL_ENGINE_ERROR_FAILED, "No engine to send to");
      return;
    }

    FlutterEngineResult result =
        self->embedder_api.PlatformMessageCreateResponseHandle(
            self->engine, fl_engine_platform_message_response_cb, task,
            &response_handle);
    if (result != kSuccess) {
      g_task_return_new_error(task, fl_engine_error_quark(),
                              FL_ENGINE_ERROR_FAILED,
                              "Failed to create response handle");
      g_object_unref(task);
      return;
    }
  } else if (self->engine == nullptr) {
    return;
  }

  FlutterPlatformMessage fl_message = {};
  fl_message.struct_size = sizeof(fl_message);
  fl_message.channel = channel;
  fl_message.message =
      message != nullptr
          ? static_cast<const uint8_t*>(g_bytes_get_data(message, nullptr))
          : nullptr;
  fl_message.message_size = message != nullptr ? g_bytes_get_size(message) : 0;
  fl_message.response_handle = response_handle;
  FlutterEngineResult result =
      self->embedder_api.SendPlatformMessage(self->engine, &fl_message);

  if (result != kSuccess && task != nullptr) {
    g_task_return_new_error(task, fl_engine_error_quark(),
                            FL_ENGINE_ERROR_FAILED,
                            "Failed to send platform messages");
    g_object_unref(task);
  }

  if (response_handle != nullptr) {
    self->embedder_api.PlatformMessageReleaseResponseHandle(self->engine,
                                                            response_handle);
  }
}

GBytes* fl_engine_send_platform_message_finish(FlEngine* self,
                                               GAsyncResult* result,
                                               GError** error) {
  g_return_val_if_fail(FL_IS_ENGINE(self), FALSE);
  g_return_val_if_fail(g_task_is_valid(result, self), FALSE);

  return static_cast<GBytes*>(g_task_propagate_pointer(G_TASK(result), error));
}

void fl_engine_send_window_metrics_event(FlEngine* self,
                                         FlutterViewId view_id,
                                         size_t width,
                                         size_t height,
                                         double pixel_ratio) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->engine == nullptr) {
    return;
  }

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(FlutterWindowMetricsEvent);
  event.width = width;
  event.height = height;
  event.pixel_ratio = pixel_ratio;
  event.view_id = view_id;
  self->embedder_api.SendWindowMetricsEvent(self->engine, &event);
}

void fl_engine_send_mouse_pointer_event(FlEngine* self,
                                        FlutterViewId view_id,
                                        FlutterPointerPhase phase,
                                        size_t timestamp,
                                        double x,
                                        double y,
                                        FlutterPointerDeviceKind device_kind,
                                        double scroll_delta_x,
                                        double scroll_delta_y,
                                        int64_t buttons) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->engine == nullptr) {
    return;
  }

  FlutterPointerEvent fl_event = {};
  fl_event.struct_size = sizeof(fl_event);
  fl_event.phase = phase;
  fl_event.timestamp = timestamp;
  fl_event.x = x;
  fl_event.y = y;
  if (scroll_delta_x != 0 || scroll_delta_y != 0) {
    fl_event.signal_kind = kFlutterPointerSignalKindScroll;
  }
  fl_event.scroll_delta_x = scroll_delta_x;
  fl_event.scroll_delta_y = scroll_delta_y;
  fl_event.device_kind = device_kind;
  fl_event.buttons = buttons;
  fl_event.device = kMousePointerDeviceId;
  fl_event.view_id = view_id;
  self->embedder_api.SendPointerEvent(self->engine, &fl_event, 1);
}

void fl_engine_send_touch_up_event(FlEngine* self,
                                   FlutterViewId view_id,
                                   size_t timestamp,
                                   double x,
                                   double y,
                                   int32_t device) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->engine == nullptr) {
    return;
  }

  FlutterPointerEvent event;
  event.timestamp = timestamp;
  event.x = x;
  event.y = y;
  event.device_kind = kFlutterPointerDeviceKindTouch;
  event.device = device;
  event.buttons = 0;
  event.view_id = view_id;
  event.phase = FlutterPointerPhase::kUp;
  event.struct_size = sizeof(event);

  self->embedder_api.SendPointerEvent(self->engine, &event, 1);
}

void fl_engine_send_touch_down_event(FlEngine* self,
                                     FlutterViewId view_id,
                                     size_t timestamp,
                                     double x,
                                     double y,
                                     int32_t device) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->engine == nullptr) {
    return;
  }

  FlutterPointerEvent event;
  event.timestamp = timestamp;
  event.x = x;
  event.y = y;
  event.device_kind = kFlutterPointerDeviceKindTouch;
  event.device = device;
  event.buttons = FlutterPointerMouseButtons::kFlutterPointerButtonMousePrimary;
  event.view_id = view_id;
  event.phase = FlutterPointerPhase::kDown;
  event.struct_size = sizeof(event);

  self->embedder_api.SendPointerEvent(self->engine, &event, 1);
}

void fl_engine_send_touch_move_event(FlEngine* self,
                                     FlutterViewId view_id,
                                     size_t timestamp,
                                     double x,
                                     double y,
                                     int32_t device) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->engine == nullptr) {
    return;
  }

  FlutterPointerEvent event;
  event.timestamp = timestamp;
  event.x = x;
  event.y = y;
  event.device_kind = kFlutterPointerDeviceKindTouch;
  event.device = device;
  event.buttons = FlutterPointerMouseButtons::kFlutterPointerButtonMousePrimary;
  event.view_id = view_id;
  event.phase = FlutterPointerPhase::kMove;
  event.struct_size = sizeof(event);

  self->embedder_api.SendPointerEvent(self->engine, &event, 1);
}

void fl_engine_send_touch_add_event(FlEngine* self,
                                    FlutterViewId view_id,
                                    size_t timestamp,
                                    double x,
                                    double y,
                                    int32_t device) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->engine == nullptr) {
    return;
  }

  FlutterPointerEvent event;
  event.timestamp = timestamp;
  event.x = x;
  event.y = y;
  event.device_kind = kFlutterPointerDeviceKindTouch;
  event.device = device;
  event.buttons = 0;
  event.view_id = view_id;
  event.phase = FlutterPointerPhase::kAdd;
  event.struct_size = sizeof(event);

  self->embedder_api.SendPointerEvent(self->engine, &event, 1);
}

void fl_engine_send_touch_remove_event(FlEngine* self,
                                       FlutterViewId view_id,
                                       size_t timestamp,
                                       double x,
                                       double y,
                                       int32_t device) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->engine == nullptr) {
    return;
  }

  FlutterPointerEvent event;
  event.timestamp = timestamp;
  event.x = x;
  event.y = y;
  event.device_kind = kFlutterPointerDeviceKindTouch;
  event.device = device;
  event.buttons = 0;
  event.view_id = view_id;
  event.phase = FlutterPointerPhase::kRemove;
  event.struct_size = sizeof(event);

  self->embedder_api.SendPointerEvent(self->engine, &event, 1);
}

void fl_engine_send_pointer_pan_zoom_event(FlEngine* self,
                                           FlutterViewId view_id,
                                           size_t timestamp,
                                           double x,
                                           double y,
                                           FlutterPointerPhase phase,
                                           double pan_x,
                                           double pan_y,
                                           double scale,
                                           double rotation) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->engine == nullptr) {
    return;
  }

  FlutterPointerEvent fl_event = {};
  fl_event.struct_size = sizeof(fl_event);
  fl_event.timestamp = timestamp;
  fl_event.x = x;
  fl_event.y = y;
  fl_event.phase = phase;
  fl_event.pan_x = pan_x;
  fl_event.pan_y = pan_y;
  fl_event.scale = scale;
  fl_event.rotation = rotation;
  fl_event.device = kPointerPanZoomDeviceId;
  fl_event.device_kind = kFlutterPointerDeviceKindTrackpad;
  fl_event.view_id = view_id;
  self->embedder_api.SendPointerEvent(self->engine, &fl_event, 1);
}

static void send_key_event_cb(bool handled, void* user_data) {
  g_autoptr(GTask) task = G_TASK(user_data);
  gboolean* return_value = g_new0(gboolean, 1);
  *return_value = handled;
  g_task_return_pointer(task, return_value, g_free);
}

void fl_engine_send_key_event(FlEngine* self,
                              const FlutterKeyEvent* event,
                              GCancellable* cancellable,
                              GAsyncReadyCallback callback,
                              gpointer user_data) {
  g_return_if_fail(FL_IS_ENGINE(self));

  g_autoptr(GTask) task = g_task_new(self, cancellable, callback, user_data);

  if (self->engine == nullptr) {
    g_task_return_new_error(task, fl_engine_error_quark(),
                            FL_ENGINE_ERROR_FAILED, "No engine");
    return;
  }

  if (self->embedder_api.SendKeyEvent(self->engine, event, send_key_event_cb,
                                      g_object_ref(task)) != kSuccess) {
    g_task_return_new_error(task, fl_engine_error_quark(),
                            FL_ENGINE_ERROR_FAILED, "Failed to send key event");
    g_object_unref(task);
  }
}

gboolean fl_engine_send_key_event_finish(FlEngine* self,
                                         GAsyncResult* result,
                                         gboolean* handled,
                                         GError** error) {
  g_return_val_if_fail(FL_IS_ENGINE(self), FALSE);
  g_return_val_if_fail(g_task_is_valid(result, self), FALSE);

  g_autofree gboolean* return_value =
      static_cast<gboolean*>(g_task_propagate_pointer(G_TASK(result), error));
  if (return_value == nullptr) {
    return FALSE;
  }

  *handled = *return_value;
  return TRUE;
}

void fl_engine_dispatch_semantics_action(FlEngine* self,
                                         uint64_t id,
                                         FlutterSemanticsAction action,
                                         GBytes* data) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->engine == nullptr) {
    return;
  }

  const uint8_t* action_data = nullptr;
  size_t action_data_length = 0;
  if (data != nullptr) {
    action_data = static_cast<const uint8_t*>(
        g_bytes_get_data(data, &action_data_length));
  }

  self->embedder_api.DispatchSemanticsAction(self->engine, id, action,
                                             action_data, action_data_length);
}

gboolean fl_engine_mark_texture_frame_available(FlEngine* self,
                                                int64_t texture_id) {
  g_return_val_if_fail(FL_IS_ENGINE(self), FALSE);
  return self->embedder_api.MarkExternalTextureFrameAvailable(
             self->engine, texture_id) == kSuccess;
}

gboolean fl_engine_register_external_texture(FlEngine* self,
                                             int64_t texture_id) {
  g_return_val_if_fail(FL_IS_ENGINE(self), FALSE);
  return self->embedder_api.RegisterExternalTexture(self->engine, texture_id) ==
         kSuccess;
}

gboolean fl_engine_unregister_external_texture(FlEngine* self,
                                               int64_t texture_id) {
  g_return_val_if_fail(FL_IS_ENGINE(self), FALSE);
  return self->embedder_api.UnregisterExternalTexture(self->engine,
                                                      texture_id) == kSuccess;
}

G_MODULE_EXPORT FlBinaryMessenger* fl_engine_get_binary_messenger(
    FlEngine* self) {
  g_return_val_if_fail(FL_IS_ENGINE(self), nullptr);
  return self->binary_messenger;
}

FlTaskRunner* fl_engine_get_task_runner(FlEngine* self) {
  g_return_val_if_fail(FL_IS_ENGINE(self), nullptr);
  return self->task_runner;
}

void fl_engine_execute_task(FlEngine* self, FlutterTask* task) {
  g_return_if_fail(FL_IS_ENGINE(self));
  self->embedder_api.RunTask(self->engine, task);
}

G_MODULE_EXPORT FlTextureRegistrar* fl_engine_get_texture_registrar(
    FlEngine* self) {
  g_return_val_if_fail(FL_IS_ENGINE(self), nullptr);
  return self->texture_registrar;
}

void fl_engine_update_accessibility_features(FlEngine* self, int32_t flags) {
  g_return_if_fail(FL_IS_ENGINE(self));

  if (self->engine == nullptr) {
    return;
  }

  self->embedder_api.UpdateAccessibilityFeatures(
      self->engine, static_cast<FlutterAccessibilityFeature>(flags));
}

void fl_engine_request_app_exit(FlEngine* self) {
  g_return_if_fail(FL_IS_ENGINE(self));
  fl_platform_handler_request_app_exit(self->platform_handler);
}

FlMouseCursorHandler* fl_engine_get_mouse_cursor_handler(FlEngine* self) {
  g_return_val_if_fail(FL_IS_ENGINE(self), nullptr);
  return self->mouse_cursor_handler;
}

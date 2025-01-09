// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Included first as it collides with the X11 headers.
#include "gtest/gtest.h"

#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/linux/fl_engine_private.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_message_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_string_codec.h"

// MOCK_ENGINE_PROC is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

// Checks sending window metrics events works.
TEST(FlEngineTest, WindowMetrics) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  bool called = false;
  fl_engine_get_embedder_api(engine)->SendWindowMetricsEvent = MOCK_ENGINE_PROC(
      SendWindowMetricsEvent,
      ([&called](auto engine, const FlutterWindowMetricsEvent* event) {
        called = true;
        EXPECT_EQ(event->view_id, 1);
        EXPECT_EQ(event->width, static_cast<size_t>(3840));
        EXPECT_EQ(event->height, static_cast<size_t>(2160));
        EXPECT_EQ(event->pixel_ratio, 2.0);

        return kSuccess;
      }));

  fl_engine_send_window_metrics_event(engine, 1, 3840, 2160, 2.0);

  EXPECT_TRUE(called);
}

// Checks sending mouse pointer events works.
TEST(FlEngineTest, MousePointer) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&called](auto engine, const FlutterPointerEvent* events,
                 size_t events_count) {
        called = true;
        EXPECT_EQ(events_count, static_cast<size_t>(1));
        EXPECT_EQ(events[0].view_id, 1);
        EXPECT_EQ(events[0].phase, kDown);
        EXPECT_EQ(events[0].timestamp, static_cast<size_t>(1234567890));
        EXPECT_EQ(events[0].x, 800);
        EXPECT_EQ(events[0].y, 600);
        EXPECT_EQ(events[0].device, static_cast<int32_t>(0));
        EXPECT_EQ(events[0].signal_kind, kFlutterPointerSignalKindScroll);
        EXPECT_EQ(events[0].scroll_delta_x, 1.2);
        EXPECT_EQ(events[0].scroll_delta_y, -3.4);
        EXPECT_EQ(events[0].device_kind, kFlutterPointerDeviceKindMouse);
        EXPECT_EQ(events[0].buttons, kFlutterPointerButtonMouseSecondary);

        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);
  fl_engine_send_mouse_pointer_event(engine, 1, kDown, 1234567890, 800, 600,
                                     kFlutterPointerDeviceKindMouse, 1.2, -3.4,
                                     kFlutterPointerButtonMouseSecondary);

  EXPECT_TRUE(called);
}

// Checks sending pan/zoom events works.
TEST(FlEngineTest, PointerPanZoom) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->SendPointerEvent = MOCK_ENGINE_PROC(
      SendPointerEvent,
      ([&called](auto engine, const FlutterPointerEvent* events,
                 size_t events_count) {
        called = true;
        EXPECT_EQ(events_count, static_cast<size_t>(1));
        EXPECT_EQ(events[0].view_id, 1);
        EXPECT_EQ(events[0].phase, kPanZoomUpdate);
        EXPECT_EQ(events[0].timestamp, static_cast<size_t>(1234567890));
        EXPECT_EQ(events[0].x, 800);
        EXPECT_EQ(events[0].y, 600);
        EXPECT_EQ(events[0].device, static_cast<int32_t>(1));
        EXPECT_EQ(events[0].signal_kind, kFlutterPointerSignalKindNone);
        EXPECT_EQ(events[0].pan_x, 1.5);
        EXPECT_EQ(events[0].pan_y, 2.5);
        EXPECT_EQ(events[0].scale, 3.5);
        EXPECT_EQ(events[0].rotation, 4.5);
        EXPECT_EQ(events[0].device_kind, kFlutterPointerDeviceKindTrackpad);
        EXPECT_EQ(events[0].buttons, 0);

        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);
  fl_engine_send_pointer_pan_zoom_event(engine, 1, 1234567890, 800, 600,
                                        kPanZoomUpdate, 1.5, 2.5, 3.5, 4.5);

  EXPECT_TRUE(called);
}

// Checks dispatching a semantics action works.
TEST(FlEngineTest, DispatchSemanticsAction) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->DispatchSemanticsAction =
      MOCK_ENGINE_PROC(
          DispatchSemanticsAction,
          ([&called](auto engine, uint64_t id, FlutterSemanticsAction action,
                     const uint8_t* data, size_t data_length) {
            EXPECT_EQ(id, static_cast<uint64_t>(42));
            EXPECT_EQ(action, kFlutterSemanticsActionTap);
            EXPECT_EQ(data_length, static_cast<size_t>(4));
            EXPECT_EQ(data[0], 't');
            EXPECT_EQ(data[1], 'e');
            EXPECT_EQ(data[2], 's');
            EXPECT_EQ(data[3], 't');
            called = true;

            return kSuccess;
          }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);
  g_autoptr(GBytes) data = g_bytes_new_static("test", 4);
  fl_engine_dispatch_semantics_action(engine, 42, kFlutterSemanticsActionTap,
                                      data);

  EXPECT_TRUE(called);
}

// Checks sending platform messages works.
TEST(FlEngineTest, PlatformMessage) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  FlutterEngineSendPlatformMessageFnPtr old_handler =
      fl_engine_get_embedder_api(engine)->SendPlatformMessage;
  fl_engine_get_embedder_api(engine)->SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      ([&called, old_handler](auto engine,
                              const FlutterPlatformMessage* message) {
        if (strcmp(message->channel, "test") != 0) {
          return old_handler(engine, message);
        }

        called = true;

        EXPECT_EQ(message->message_size, static_cast<size_t>(4));
        EXPECT_EQ(message->message[0], 't');
        EXPECT_EQ(message->message[1], 'e');
        EXPECT_EQ(message->message[2], 's');
        EXPECT_EQ(message->message[3], 't');

        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);
  g_autoptr(GBytes) message = g_bytes_new_static("test", 4);
  fl_engine_send_platform_message(engine, "test", message, nullptr, nullptr,
                                  nullptr);

  EXPECT_TRUE(called);
}

// Checks sending platform message responses works.
TEST(FlEngineTest, PlatformMessageResponse) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->SendPlatformMessageResponse =
      MOCK_ENGINE_PROC(
          SendPlatformMessageResponse,
          ([&called](auto engine,
                     const FlutterPlatformMessageResponseHandle* handle,
                     const uint8_t* data, size_t data_length) {
            called = true;

            EXPECT_EQ(
                handle,
                reinterpret_cast<const FlutterPlatformMessageResponseHandle*>(
                    42));
            EXPECT_EQ(data_length, static_cast<size_t>(4));
            EXPECT_EQ(data[0], 't');
            EXPECT_EQ(data[1], 'e');
            EXPECT_EQ(data[2], 's');
            EXPECT_EQ(data[3], 't');

            return kSuccess;
          }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);
  g_autoptr(GBytes) response = g_bytes_new_static("test", 4);
  EXPECT_TRUE(fl_engine_send_platform_message_response(
      engine, reinterpret_cast<const FlutterPlatformMessageResponseHandle*>(42),
      response, &error));
  EXPECT_EQ(error, nullptr);

  EXPECT_TRUE(called);
}

// Checks settings handler sends settings on startup.
TEST(FlEngineTest, SettingsHandler) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      ([&called](auto engine, const FlutterPlatformMessage* message) {
        called = true;

        EXPECT_STREQ(message->channel, "flutter/settings");

        g_autoptr(FlJsonMessageCodec) codec = fl_json_message_codec_new();
        g_autoptr(GBytes) data =
            g_bytes_new(message->message, message->message_size);
        g_autoptr(GError) error = nullptr;
        g_autoptr(FlValue) settings = fl_message_codec_decode_message(
            FL_MESSAGE_CODEC(codec), data, &error);
        EXPECT_NE(settings, nullptr);
        EXPECT_EQ(error, nullptr);

        FlValue* text_scale_factor =
            fl_value_lookup_string(settings, "textScaleFactor");
        EXPECT_NE(text_scale_factor, nullptr);
        EXPECT_EQ(fl_value_get_type(text_scale_factor), FL_VALUE_TYPE_FLOAT);

        FlValue* always_use_24hr_format =
            fl_value_lookup_string(settings, "alwaysUse24HourFormat");
        EXPECT_NE(always_use_24hr_format, nullptr);
        EXPECT_EQ(fl_value_get_type(always_use_24hr_format),
                  FL_VALUE_TYPE_BOOL);

        FlValue* platform_brightness =
            fl_value_lookup_string(settings, "platformBrightness");
        EXPECT_NE(platform_brightness, nullptr);
        EXPECT_EQ(fl_value_get_type(platform_brightness), FL_VALUE_TYPE_STRING);

        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  EXPECT_TRUE(called);
}

void on_pre_engine_restart_cb(FlEngine* engine, gpointer user_data) {
  int* count = reinterpret_cast<int*>(user_data);
  *count += 1;
}

// Checks restarting the engine invokes the correct callback.
TEST(FlEngineTest, OnPreEngineRestart) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  OnPreEngineRestartCallback callback;
  void* callback_user_data;

  bool called = false;
  fl_engine_get_embedder_api(engine)->Initialize = MOCK_ENGINE_PROC(
      Initialize, ([&callback, &callback_user_data, &called](
                       size_t version, const FlutterRendererConfig* config,
                       const FlutterProjectArgs* args, void* user_data,
                       FLUTTER_API_SYMBOL(FlutterEngine) * engine_out) {
        called = true;
        callback = args->on_pre_engine_restart_callback;
        callback_user_data = user_data;

        return kSuccess;
      }));
  fl_engine_get_embedder_api(engine)->RunInitialized =
      MOCK_ENGINE_PROC(RunInitialized, ([](auto engine) { return kSuccess; }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  EXPECT_TRUE(called);
  EXPECT_NE(callback, nullptr);

  // The following call has no effect but should not crash.
  callback(callback_user_data);

  int count = 0;

  // Set handler so that:
  //
  //  * When the engine restarts, count += 1;
  //  * When the engine is freed, count += 10.
  g_signal_connect(engine, "on-pre-engine-restart",
                   G_CALLBACK(on_pre_engine_restart_cb), &count);

  callback(callback_user_data);
  EXPECT_EQ(count, 1);
}

TEST(FlEngineTest, DartEntrypointArgs) {
  GPtrArray* args_array = g_ptr_array_new();
  g_ptr_array_add(args_array, const_cast<char*>("arg_one"));
  g_ptr_array_add(args_array, const_cast<char*>("arg_two"));
  g_ptr_array_add(args_array, const_cast<char*>("arg_three"));
  g_ptr_array_add(args_array, nullptr);
  gchar** args = reinterpret_cast<gchar**>(g_ptr_array_free(args_array, false));

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  fl_dart_project_set_dart_entrypoint_arguments(project, args);
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->Initialize = MOCK_ENGINE_PROC(
      Initialize, ([&called, &set_args = args](
                       size_t version, const FlutterRendererConfig* config,
                       const FlutterProjectArgs* args, void* user_data,
                       FLUTTER_API_SYMBOL(FlutterEngine) * engine_out) {
        called = true;
        EXPECT_NE(set_args, args->dart_entrypoint_argv);
        EXPECT_EQ(args->dart_entrypoint_argc, 3);

        return kSuccess;
      }));
  fl_engine_get_embedder_api(engine)->RunInitialized =
      MOCK_ENGINE_PROC(RunInitialized, ([](auto engine) { return kSuccess; }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  EXPECT_TRUE(called);
}

TEST(FlEngineTest, Locales) {
  g_autofree gchar* initial_language = g_strdup(g_getenv("LANGUAGE"));
  g_setenv("LANGUAGE", "de:en_US", TRUE);
  g_autoptr(FlDartProject) project = fl_dart_project_new();

  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->UpdateLocales = MOCK_ENGINE_PROC(
      UpdateLocales, ([&called](auto engine, const FlutterLocale** locales,
                                size_t locales_count) {
        called = true;

        EXPECT_EQ(locales_count, static_cast<size_t>(4));

        EXPECT_STREQ(locales[0]->language_code, "de");
        EXPECT_STREQ(locales[0]->country_code, nullptr);
        EXPECT_STREQ(locales[0]->script_code, nullptr);
        EXPECT_STREQ(locales[0]->variant_code, nullptr);

        EXPECT_STREQ(locales[1]->language_code, "en");
        EXPECT_STREQ(locales[1]->country_code, "US");
        EXPECT_STREQ(locales[1]->script_code, nullptr);
        EXPECT_STREQ(locales[1]->variant_code, nullptr);

        EXPECT_STREQ(locales[2]->language_code, "en");
        EXPECT_STREQ(locales[2]->country_code, nullptr);
        EXPECT_STREQ(locales[2]->script_code, nullptr);
        EXPECT_STREQ(locales[2]->variant_code, nullptr);

        EXPECT_STREQ(locales[3]->language_code, "C");
        EXPECT_STREQ(locales[3]->country_code, nullptr);
        EXPECT_STREQ(locales[3]->script_code, nullptr);
        EXPECT_STREQ(locales[3]->variant_code, nullptr);

        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  EXPECT_TRUE(called);

  if (initial_language) {
    g_setenv("LANGUAGE", initial_language, TRUE);
  } else {
    g_unsetenv("LANGUAGE");
  }
}

TEST(FlEngineTest, CLocale) {
  g_autofree gchar* initial_language = g_strdup(g_getenv("LANGUAGE"));
  g_setenv("LANGUAGE", "C", TRUE);
  g_autoptr(FlDartProject) project = fl_dart_project_new();

  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->UpdateLocales = MOCK_ENGINE_PROC(
      UpdateLocales, ([&called](auto engine, const FlutterLocale** locales,
                                size_t locales_count) {
        called = true;

        EXPECT_EQ(locales_count, static_cast<size_t>(1));

        EXPECT_STREQ(locales[0]->language_code, "C");
        EXPECT_STREQ(locales[0]->country_code, nullptr);
        EXPECT_STREQ(locales[0]->script_code, nullptr);
        EXPECT_STREQ(locales[0]->variant_code, nullptr);

        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  EXPECT_TRUE(called);

  if (initial_language) {
    g_setenv("LANGUAGE", initial_language, TRUE);
  } else {
    g_unsetenv("LANGUAGE");
  }
}

TEST(FlEngineTest, DuplicateLocale) {
  g_autofree gchar* initial_language = g_strdup(g_getenv("LANGUAGE"));
  g_setenv("LANGUAGE", "en:en", TRUE);
  g_autoptr(FlDartProject) project = fl_dart_project_new();

  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->UpdateLocales = MOCK_ENGINE_PROC(
      UpdateLocales, ([&called](auto engine, const FlutterLocale** locales,
                                size_t locales_count) {
        called = true;

        EXPECT_EQ(locales_count, static_cast<size_t>(2));

        EXPECT_STREQ(locales[0]->language_code, "en");
        EXPECT_STREQ(locales[0]->country_code, nullptr);
        EXPECT_STREQ(locales[0]->script_code, nullptr);
        EXPECT_STREQ(locales[0]->variant_code, nullptr);

        EXPECT_STREQ(locales[1]->language_code, "C");
        EXPECT_STREQ(locales[1]->country_code, nullptr);
        EXPECT_STREQ(locales[1]->script_code, nullptr);
        EXPECT_STREQ(locales[1]->variant_code, nullptr);

        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  EXPECT_TRUE(called);

  if (initial_language) {
    g_setenv("LANGUAGE", initial_language, TRUE);
  } else {
    g_unsetenv("LANGUAGE");
  }
}

TEST(FlEngineTest, EmptyLocales) {
  g_autofree gchar* initial_language = g_strdup(g_getenv("LANGUAGE"));
  g_setenv("LANGUAGE", "de:: :en_US", TRUE);
  g_autoptr(FlDartProject) project = fl_dart_project_new();

  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->UpdateLocales = MOCK_ENGINE_PROC(
      UpdateLocales, ([&called](auto engine, const FlutterLocale** locales,
                                size_t locales_count) {
        called = true;

        EXPECT_EQ(locales_count, static_cast<size_t>(4));

        EXPECT_STREQ(locales[0]->language_code, "de");
        EXPECT_STREQ(locales[0]->country_code, nullptr);
        EXPECT_STREQ(locales[0]->script_code, nullptr);
        EXPECT_STREQ(locales[0]->variant_code, nullptr);

        EXPECT_STREQ(locales[1]->language_code, "en");
        EXPECT_STREQ(locales[1]->country_code, "US");
        EXPECT_STREQ(locales[1]->script_code, nullptr);
        EXPECT_STREQ(locales[1]->variant_code, nullptr);

        EXPECT_STREQ(locales[2]->language_code, "en");
        EXPECT_STREQ(locales[2]->country_code, nullptr);
        EXPECT_STREQ(locales[2]->script_code, nullptr);
        EXPECT_STREQ(locales[2]->variant_code, nullptr);

        EXPECT_STREQ(locales[3]->language_code, "C");
        EXPECT_STREQ(locales[3]->country_code, nullptr);
        EXPECT_STREQ(locales[3]->script_code, nullptr);
        EXPECT_STREQ(locales[3]->variant_code, nullptr);

        return kSuccess;
      }));

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  EXPECT_TRUE(called);

  if (initial_language) {
    g_setenv("LANGUAGE", initial_language, TRUE);
  } else {
    g_unsetenv("LANGUAGE");
  }
}

static void add_view_cb(GObject* object,
                        GAsyncResult* result,
                        gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  gboolean r = fl_engine_add_view_finish(FL_ENGINE(object), result, &error);
  EXPECT_TRUE(r);
  EXPECT_EQ(error, nullptr);

  g_main_loop_quit(static_cast<GMainLoop*>(user_data));
}

TEST(FlEngineTest, AddView) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->AddView = MOCK_ENGINE_PROC(
      AddView, ([&called](auto engine, const FlutterAddViewInfo* info) {
        called = true;
        EXPECT_EQ(info->view_metrics->width, 123u);
        EXPECT_EQ(info->view_metrics->height, 456u);
        EXPECT_EQ(info->view_metrics->pixel_ratio, 2.0);

        FlutterAddViewResult result;
        result.struct_size = sizeof(FlutterAddViewResult);
        result.added = true;
        result.user_data = info->user_data;
        info->add_view_callback(&result);

        return kSuccess;
      }));

  FlutterViewId view_id =
      fl_engine_add_view(engine, 123, 456, 2.0, nullptr, add_view_cb, loop);
  EXPECT_GT(view_id, 0);
  EXPECT_TRUE(called);

  // Blocks here until add_view_cb is called.
  g_main_loop_run(loop);
}

static void add_view_error_cb(GObject* object,
                              GAsyncResult* result,
                              gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  gboolean r = fl_engine_add_view_finish(FL_ENGINE(object), result, &error);
  EXPECT_FALSE(r);
  EXPECT_NE(error, nullptr);

  g_main_loop_quit(static_cast<GMainLoop*>(user_data));
}

TEST(FlEngineTest, AddViewError) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  fl_engine_get_embedder_api(engine)->AddView = MOCK_ENGINE_PROC(
      AddView, ([](auto engine, const FlutterAddViewInfo* info) {
        FlutterAddViewResult result;
        result.struct_size = sizeof(FlutterAddViewResult);
        result.added = false;
        result.user_data = info->user_data;
        info->add_view_callback(&result);

        return kSuccess;
      }));

  FlutterViewId view_id = fl_engine_add_view(engine, 123, 456, 2.0, nullptr,
                                             add_view_error_cb, loop);
  EXPECT_GT(view_id, 0);

  // Blocks here until add_view_error_cb is called.
  g_main_loop_run(loop);
}

static void add_view_engine_error_cb(GObject* object,
                                     GAsyncResult* result,
                                     gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  gboolean r = fl_engine_add_view_finish(FL_ENGINE(object), result, &error);
  EXPECT_FALSE(r);
  EXPECT_NE(error, nullptr);

  g_main_loop_quit(static_cast<GMainLoop*>(user_data));
}

TEST(FlEngineTest, AddViewEngineError) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  fl_engine_get_embedder_api(engine)->AddView = MOCK_ENGINE_PROC(
      AddView, ([](auto engine, const FlutterAddViewInfo* info) {
        return kInvalidArguments;
      }));

  FlutterViewId view_id = fl_engine_add_view(engine, 123, 456, 2.0, nullptr,
                                             add_view_engine_error_cb, loop);
  EXPECT_GT(view_id, 0);

  // Blocks here until remove_view_engine_error_cb is called.
  g_main_loop_run(loop);
}

static void remove_view_cb(GObject* object,
                           GAsyncResult* result,
                           gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  gboolean r = fl_engine_remove_view_finish(FL_ENGINE(object), result, &error);
  EXPECT_TRUE(r);
  EXPECT_EQ(error, nullptr);

  g_main_loop_quit(static_cast<GMainLoop*>(user_data));
}

TEST(FlEngineTest, RemoveView) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  bool called = false;
  fl_engine_get_embedder_api(engine)->RemoveView = MOCK_ENGINE_PROC(
      RemoveView, ([&called](auto engine, const FlutterRemoveViewInfo* info) {
        called = true;
        EXPECT_EQ(info->view_id, 123);

        FlutterRemoveViewResult result;
        result.struct_size = sizeof(FlutterRemoveViewResult);
        result.removed = true;
        result.user_data = info->user_data;
        info->remove_view_callback(&result);

        return kSuccess;
      }));

  fl_engine_remove_view(engine, 123, nullptr, remove_view_cb, loop);
  EXPECT_TRUE(called);

  // Blocks here until remove_view_cb is called.
  g_main_loop_run(loop);
}

static void remove_view_error_cb(GObject* object,
                                 GAsyncResult* result,
                                 gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  gboolean r = fl_engine_remove_view_finish(FL_ENGINE(object), result, &error);
  EXPECT_FALSE(r);
  EXPECT_NE(error, nullptr);

  g_main_loop_quit(static_cast<GMainLoop*>(user_data));
}

TEST(FlEngineTest, RemoveViewError) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  fl_engine_get_embedder_api(engine)->RemoveView = MOCK_ENGINE_PROC(
      RemoveView, ([](auto engine, const FlutterRemoveViewInfo* info) {
        FlutterRemoveViewResult result;
        result.struct_size = sizeof(FlutterRemoveViewResult);
        result.removed = false;
        result.user_data = info->user_data;
        info->remove_view_callback(&result);

        return kSuccess;
      }));

  fl_engine_remove_view(engine, 123, nullptr, remove_view_error_cb, loop);

  // Blocks here until remove_view_error_cb is called.
  g_main_loop_run(loop);
}

static void remove_view_engine_error_cb(GObject* object,
                                        GAsyncResult* result,
                                        gpointer user_data) {
  g_autoptr(GError) error = nullptr;
  gboolean r = fl_engine_remove_view_finish(FL_ENGINE(object), result, &error);
  EXPECT_FALSE(r);
  EXPECT_NE(error, nullptr);

  g_main_loop_quit(static_cast<GMainLoop*>(user_data));
}

TEST(FlEngineTest, RemoveViewEngineError) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  fl_engine_get_embedder_api(engine)->RemoveView = MOCK_ENGINE_PROC(
      RemoveView, ([](auto engine, const FlutterRemoveViewInfo* info) {
        return kInvalidArguments;
      }));

  fl_engine_remove_view(engine, 123, nullptr, remove_view_engine_error_cb,
                        loop);

  // Blocks here until remove_view_engine_error_cb is called.
  g_main_loop_run(loop);
}

TEST(FlEngineTest, SendKeyEvent) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  bool called;
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&called](auto engine, const FlutterKeyEvent* event,
                 FlutterKeyEventCallback callback, void* user_data) {
        called = true;
        EXPECT_EQ(event->timestamp, 1234);
        EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
        EXPECT_EQ(event->physical, static_cast<uint64_t>(42));
        EXPECT_EQ(event->logical, static_cast<uint64_t>(123));
        EXPECT_TRUE(event->synthesized);
        EXPECT_EQ(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
        callback(TRUE, user_data);
        return kSuccess;
      }));

  FlutterKeyEvent event = {.struct_size = sizeof(FlutterKeyEvent),
                           .timestamp = 1234,
                           .type = kFlutterKeyEventTypeUp,
                           .physical = 42,
                           .logical = 123,
                           .character = nullptr,
                           .synthesized = true,
                           .device_type = kFlutterKeyEventDeviceTypeKeyboard};
  fl_engine_send_key_event(
      engine, &event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        g_autoptr(GError) error = nullptr;
        EXPECT_TRUE(fl_engine_send_key_event_finish(FL_ENGINE(object), result,
                                                    &handled, &error));
        EXPECT_EQ(error, nullptr);
        EXPECT_TRUE(handled);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
  EXPECT_TRUE(called);
}

TEST(FlEngineTest, SendKeyEventNotHandled) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  bool called;
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&called](auto engine, const FlutterKeyEvent* event,
                 FlutterKeyEventCallback callback, void* user_data) {
        called = true;
        callback(FALSE, user_data);
        return kSuccess;
      }));

  FlutterKeyEvent event = {.struct_size = sizeof(FlutterKeyEvent),
                           .timestamp = 1234,
                           .type = kFlutterKeyEventTypeUp,
                           .physical = 42,
                           .logical = 123,
                           .character = nullptr,
                           .synthesized = true,
                           .device_type = kFlutterKeyEventDeviceTypeKeyboard};
  fl_engine_send_key_event(
      engine, &event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        g_autoptr(GError) error = nullptr;
        EXPECT_TRUE(fl_engine_send_key_event_finish(FL_ENGINE(object), result,
                                                    &handled, &error));
        EXPECT_EQ(error, nullptr);
        EXPECT_FALSE(handled);
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
  EXPECT_TRUE(called);
}

TEST(FlEngineTest, SendKeyEventError) {
  g_autoptr(GMainLoop) loop = g_main_loop_new(nullptr, 0);

  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(GError) error = nullptr;
  EXPECT_TRUE(fl_engine_start(engine, &error));
  EXPECT_EQ(error, nullptr);

  bool called;
  fl_engine_get_embedder_api(engine)->SendKeyEvent = MOCK_ENGINE_PROC(
      SendKeyEvent,
      ([&called](auto engine, const FlutterKeyEvent* event,
                 FlutterKeyEventCallback callback, void* user_data) {
        called = true;
        return kInvalidArguments;
      }));

  FlutterKeyEvent event = {.struct_size = sizeof(FlutterKeyEvent),
                           .timestamp = 1234,
                           .type = kFlutterKeyEventTypeUp,
                           .physical = 42,
                           .logical = 123,
                           .character = nullptr,
                           .synthesized = true,
                           .device_type = kFlutterKeyEventDeviceTypeKeyboard};
  fl_engine_send_key_event(
      engine, &event, nullptr,
      [](GObject* object, GAsyncResult* result, gpointer user_data) {
        gboolean handled;
        g_autoptr(GError) error = nullptr;
        EXPECT_FALSE(fl_engine_send_key_event_finish(FL_ENGINE(object), result,
                                                     &handled, &error));
        EXPECT_NE(error, nullptr);
        EXPECT_STREQ(error->message, "Failed to send key event");
        g_main_loop_quit(static_cast<GMainLoop*>(user_data));
      },
      loop);

  g_main_loop_run(loop);
  EXPECT_TRUE(called);
}

// NOLINTEND(clang-analyzer-core.StackAddressEscape)

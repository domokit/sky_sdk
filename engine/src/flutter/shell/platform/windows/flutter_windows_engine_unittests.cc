// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/flutter_windows_engine.h"

#include "flutter/fml/logging.h"
#include "flutter/fml/macros.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"
#include "flutter/shell/platform/windows/public/flutter_windows.h"
#include "flutter/shell/platform/windows/testing/egl/mock_manager.h"
#include "flutter/shell/platform/windows/testing/engine_modifier.h"
#include "flutter/shell/platform/windows/testing/flutter_windows_engine_builder.h"
#include "flutter/shell/platform/windows/testing/mock_platform_view_manager.h"
#include "flutter/shell/platform/windows/testing/mock_window_binding_handler.h"
#include "flutter/shell/platform/windows/testing/mock_windows_proc_table.h"
#include "flutter/shell/platform/windows/testing/test_keyboard.h"
#include "flutter/shell/platform/windows/testing/windows_test.h"
#include "flutter/shell/platform/windows/testing/windows_test_config_builder.h"
#include "flutter/third_party/accessibility/ax/platform/ax_platform_node_win.h"
#include "fml/synchronization/waitable_event.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

// winbase.h defines GetCurrentTime as a macro.
#undef GetCurrentTime

namespace flutter {
namespace testing {

using ::testing::NiceMock;
using ::testing::Return;

class FlutterWindowsEngineTest : public WindowsTest {};

// The engine can be run without any views.
TEST_F(FlutterWindowsEngineTest, RunHeadless) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();

  EngineModifier modifier(engine.get());
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };

  ASSERT_TRUE(engine->Run());
  ASSERT_EQ(engine->view(kImplicitViewId), nullptr);
  ASSERT_EQ(engine->view(123), nullptr);
}

TEST_F(FlutterWindowsEngineTest, RunDoesExpectedInitialization) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.AddDartEntrypointArgument("arg1");
  builder.AddDartEntrypointArgument("arg2");

  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  // The engine should be run with expected configuration values.
  bool run_called = false;
  modifier.embedder_api().Run = MOCK_ENGINE_PROC(
      Run, ([&run_called, engine_instance = engine.get()](
                size_t version, const FlutterRendererConfig* config,
                const FlutterProjectArgs* args, void* user_data,
                FLUTTER_API_SYMBOL(FlutterEngine) * engine_out) {
        run_called = true;
        *engine_out = reinterpret_cast<FLUTTER_API_SYMBOL(FlutterEngine)>(1);

        EXPECT_EQ(version, FLUTTER_ENGINE_VERSION);
        EXPECT_NE(config, nullptr);
        // We have an EGL manager, so this should be using OpenGL.
        EXPECT_EQ(config->type, kOpenGL);
        EXPECT_EQ(user_data, engine_instance);
        // Spot-check arguments.
        EXPECT_NE(args->assets_path, nullptr);
        EXPECT_NE(args->icu_data_path, nullptr);
        EXPECT_EQ(args->dart_entrypoint_argc, 2U);
        EXPECT_EQ(strcmp(args->dart_entrypoint_argv[0], "arg1"), 0);
        EXPECT_EQ(strcmp(args->dart_entrypoint_argv[1], "arg2"), 0);
        EXPECT_NE(args->platform_message_callback, nullptr);
        EXPECT_NE(args->custom_task_runners, nullptr);
        EXPECT_NE(args->custom_task_runners->thread_priority_setter, nullptr);
        EXPECT_EQ(args->custom_dart_entrypoint, nullptr);
        EXPECT_NE(args->vsync_callback, nullptr);
        EXPECT_EQ(args->update_semantics_callback, nullptr);
        EXPECT_NE(args->update_semantics_callback2, nullptr);
        EXPECT_EQ(args->update_semantics_node_callback, nullptr);
        EXPECT_EQ(args->update_semantics_custom_action_callback, nullptr);

        args->custom_task_runners->thread_priority_setter(
            FlutterThreadPriority::kRaster);
        EXPECT_EQ(GetThreadPriority(GetCurrentThread()),
                  THREAD_PRIORITY_ABOVE_NORMAL);
        return kSuccess;
      }));
  // Accessibility updates must do nothing when the embedder engine is mocked
  modifier.embedder_api().UpdateAccessibilityFeatures = MOCK_ENGINE_PROC(
      UpdateAccessibilityFeatures,
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine,
         FlutterAccessibilityFeature flags) { return kSuccess; });

  // It should send locale info.
  bool update_locales_called = false;
  modifier.embedder_api().UpdateLocales = MOCK_ENGINE_PROC(
      UpdateLocales,
      ([&update_locales_called](auto engine, const FlutterLocale** locales,
                                size_t locales_count) {
        update_locales_called = true;

        EXPECT_GT(locales_count, 0);
        EXPECT_NE(locales, nullptr);

        return kSuccess;
      }));

  // And it should send initial settings info.
  bool settings_message_sent = false;
  modifier.embedder_api().SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      ([&settings_message_sent](auto engine, auto message) {
        if (std::string(message->channel) == std::string("flutter/settings")) {
          settings_message_sent = true;
        }

        return kSuccess;
      }));

  // And it should send display info.
  bool notify_display_update_called = false;
  modifier.SetFrameInterval(16600000);  // 60 fps.
  modifier.embedder_api().NotifyDisplayUpdate = MOCK_ENGINE_PROC(
      NotifyDisplayUpdate,
      ([&notify_display_update_called, engine_instance = engine.get()](
           FLUTTER_API_SYMBOL(FlutterEngine) raw_engine,
           const FlutterEngineDisplaysUpdateType update_type,
           const FlutterEngineDisplay* embedder_displays,
           size_t display_count) {
        EXPECT_EQ(update_type, kFlutterEngineDisplaysUpdateTypeStartup);
        EXPECT_EQ(display_count, 1);

        FlutterEngineDisplay display = embedder_displays[0];

        EXPECT_EQ(display.display_id, 0);
        EXPECT_EQ(display.single_display, true);
        EXPECT_EQ(std::floor(display.refresh_rate), 60.0);

        notify_display_update_called = true;
        return kSuccess;
      }));

  // Set the EGL manager to !nullptr to test ANGLE rendering.
  modifier.SetEGLManager(std::make_unique<egl::MockManager>());

  engine->Run();

  EXPECT_TRUE(run_called);
  EXPECT_TRUE(update_locales_called);
  EXPECT_TRUE(settings_message_sent);
  EXPECT_TRUE(notify_display_update_called);

  // Ensure that deallocation doesn't call the actual Shutdown with the bogus
  // engine pointer that the overridden Run returned.
  modifier.embedder_api().Shutdown = [](auto engine) { return kSuccess; };
  modifier.ReleaseEGLManager();
}

TEST_F(FlutterWindowsEngineTest, ConfiguresFrameVsync) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());
  bool on_vsync_called = false;

  modifier.embedder_api().GetCurrentTime =
      MOCK_ENGINE_PROC(GetCurrentTime, ([]() -> uint64_t { return 1; }));
  modifier.embedder_api().OnVsync = MOCK_ENGINE_PROC(
      OnVsync,
      ([&on_vsync_called, engine_instance = engine.get()](
           FLUTTER_API_SYMBOL(FlutterEngine) engine, intptr_t baton,
           uint64_t frame_start_time_nanos, uint64_t frame_target_time_nanos) {
        EXPECT_EQ(baton, 1);
        EXPECT_EQ(frame_start_time_nanos, 16600000);
        EXPECT_EQ(frame_target_time_nanos, 33200000);
        on_vsync_called = true;
        return kSuccess;
      }));
  modifier.SetStartTime(0);
  modifier.SetFrameInterval(16600000);

  engine->OnVsync(1);

  EXPECT_TRUE(on_vsync_called);
}

TEST_F(FlutterWindowsEngineTest, RunWithoutANGLEUsesSoftware) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  modifier.embedder_api().NotifyDisplayUpdate =
      MOCK_ENGINE_PROC(NotifyDisplayUpdate,
                       ([engine_instance = engine.get()](
                            FLUTTER_API_SYMBOL(FlutterEngine) raw_engine,
                            const FlutterEngineDisplaysUpdateType update_type,
                            const FlutterEngineDisplay* embedder_displays,
                            size_t display_count) { return kSuccess; }));

  // The engine should be run with expected configuration values.
  bool run_called = false;
  modifier.embedder_api().Run = MOCK_ENGINE_PROC(
      Run, ([&run_called, engine_instance = engine.get()](
                size_t version, const FlutterRendererConfig* config,
                const FlutterProjectArgs* args, void* user_data,
                FLUTTER_API_SYMBOL(FlutterEngine) * engine_out) {
        run_called = true;
        *engine_out = reinterpret_cast<FLUTTER_API_SYMBOL(FlutterEngine)>(1);
        // We don't have an EGL Manager, so we should be using software.
        EXPECT_EQ(config->type, kSoftware);
        return kSuccess;
      }));
  // Accessibility updates must do nothing when the embedder engine is mocked
  modifier.embedder_api().UpdateAccessibilityFeatures = MOCK_ENGINE_PROC(
      UpdateAccessibilityFeatures,
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine,
         FlutterAccessibilityFeature flags) { return kSuccess; });

  // Stub out UpdateLocales and SendPlatformMessage as we don't have a fully
  // initialized engine instance.
  modifier.embedder_api().UpdateLocales = MOCK_ENGINE_PROC(
      UpdateLocales, ([](auto engine, const FlutterLocale** locales,
                         size_t locales_count) { return kSuccess; }));
  modifier.embedder_api().SendPlatformMessage =
      MOCK_ENGINE_PROC(SendPlatformMessage,
                       ([](auto engine, auto message) { return kSuccess; }));

  // Set the EGL manager to nullptr to test software fallback path.
  modifier.SetEGLManager(nullptr);

  engine->Run();

  EXPECT_TRUE(run_called);

  // Ensure that deallocation doesn't call the actual Shutdown with the bogus
  // engine pointer that the overridden Run returned.
  modifier.embedder_api().Shutdown = [](auto engine) { return kSuccess; };
}

TEST_F(FlutterWindowsEngineTest, RunWithoutANGLEOnImpellerFailsToStart) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetSwitches({"--enable-impeller=true"});
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  modifier.embedder_api().NotifyDisplayUpdate =
      MOCK_ENGINE_PROC(NotifyDisplayUpdate,
                       ([engine_instance = engine.get()](
                            FLUTTER_API_SYMBOL(FlutterEngine) raw_engine,
                            const FlutterEngineDisplaysUpdateType update_type,
                            const FlutterEngineDisplay* embedder_displays,
                            size_t display_count) { return kSuccess; }));

  // Accessibility updates must do nothing when the embedder engine is mocked
  modifier.embedder_api().UpdateAccessibilityFeatures = MOCK_ENGINE_PROC(
      UpdateAccessibilityFeatures,
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine,
         FlutterAccessibilityFeature flags) { return kSuccess; });

  // Stub out UpdateLocales and SendPlatformMessage as we don't have a fully
  // initialized engine instance.
  modifier.embedder_api().UpdateLocales = MOCK_ENGINE_PROC(
      UpdateLocales, ([](auto engine, const FlutterLocale** locales,
                         size_t locales_count) { return kSuccess; }));
  modifier.embedder_api().SendPlatformMessage =
      MOCK_ENGINE_PROC(SendPlatformMessage,
                       ([](auto engine, auto message) { return kSuccess; }));

  // Set the EGL manager to nullptr to test software fallback path.
  modifier.SetEGLManager(nullptr);

  EXPECT_FALSE(engine->Run());
}

TEST_F(FlutterWindowsEngineTest, SendPlatformMessageWithoutResponse) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  const char* channel = "test";
  const std::vector<uint8_t> test_message = {1, 2, 3, 4};

  // Without a response, SendPlatformMessage should be a simple pass-through.
  bool called = false;
  modifier.embedder_api().SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage, ([&called, test_message](auto engine, auto message) {
        called = true;
        EXPECT_STREQ(message->channel, "test");
        EXPECT_EQ(message->message_size, test_message.size());
        EXPECT_EQ(memcmp(message->message, test_message.data(),
                         message->message_size),
                  0);
        EXPECT_EQ(message->response_handle, nullptr);
        return kSuccess;
      }));

  engine->SendPlatformMessage(channel, test_message.data(), test_message.size(),
                              nullptr, nullptr);
  EXPECT_TRUE(called);
}

TEST_F(FlutterWindowsEngineTest, PlatformMessageRoundTrip) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("hiPlatformChannels");

  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };

  auto binary_messenger =
      std::make_unique<BinaryMessengerImpl>(engine->messenger());

  engine->Run();
  bool did_call_callback = false;
  bool did_call_reply = false;
  bool did_call_dart_reply = false;
  std::string channel = "hi";
  binary_messenger->SetMessageHandler(
      channel,
      [&did_call_callback, &did_call_dart_reply](
          const uint8_t* message, size_t message_size, BinaryReply reply) {
        if (message_size == 5) {
          EXPECT_EQ(message[0], static_cast<uint8_t>('h'));
          char response[] = {'b', 'y', 'e'};
          reply(reinterpret_cast<uint8_t*>(response), 3);
          did_call_callback = true;
        } else {
          EXPECT_EQ(message_size, 3);
          EXPECT_EQ(message[0], static_cast<uint8_t>('b'));
          did_call_dart_reply = true;
        }
      });
  char payload[] = {'h', 'e', 'l', 'l', 'o'};
  binary_messenger->Send(
      channel, reinterpret_cast<uint8_t*>(payload), 5,
      [&did_call_reply](const uint8_t* reply, size_t reply_size) {
        EXPECT_EQ(reply_size, 5);
        EXPECT_EQ(reply[0], static_cast<uint8_t>('h'));
        did_call_reply = true;
      });
  // Rely on timeout mechanism in CI.
  while (!did_call_callback || !did_call_reply || !did_call_dart_reply) {
    engine->task_runner()->ProcessTasks();
  }
}

TEST_F(FlutterWindowsEngineTest, PlatformMessageRespondOnDifferentThread) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("hiPlatformChannels");

  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();

  EngineModifier modifier(engine.get());
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };

  auto binary_messenger =
      std::make_unique<BinaryMessengerImpl>(engine->messenger());

  engine->Run();
  bool did_call_callback = false;
  bool did_call_reply = false;
  bool did_call_dart_reply = false;
  std::string channel = "hi";
  std::unique_ptr<std::thread> reply_thread;
  binary_messenger->SetMessageHandler(
      channel,
      [&did_call_callback, &did_call_dart_reply, &reply_thread](
          const uint8_t* message, size_t message_size, BinaryReply reply) {
        if (message_size == 5) {
          EXPECT_EQ(message[0], static_cast<uint8_t>('h'));
          reply_thread.reset(new std::thread([reply = std::move(reply)]() {
            char response[] = {'b', 'y', 'e'};
            reply(reinterpret_cast<uint8_t*>(response), 3);
          }));
          did_call_callback = true;
        } else {
          EXPECT_EQ(message_size, 3);
          EXPECT_EQ(message[0], static_cast<uint8_t>('b'));
          did_call_dart_reply = true;
        }
      });
  char payload[] = {'h', 'e', 'l', 'l', 'o'};
  binary_messenger->Send(
      channel, reinterpret_cast<uint8_t*>(payload), 5,
      [&did_call_reply](const uint8_t* reply, size_t reply_size) {
        EXPECT_EQ(reply_size, 5);
        EXPECT_EQ(reply[0], static_cast<uint8_t>('h'));
        did_call_reply = true;
      });
  // Rely on timeout mechanism in CI.
  while (!did_call_callback || !did_call_reply || !did_call_dart_reply) {
    engine->task_runner()->ProcessTasks();
  }
  ASSERT_TRUE(reply_thread);
  reply_thread->join();
}

TEST_F(FlutterWindowsEngineTest, SendPlatformMessageWithResponse) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  const char* channel = "test";
  const std::vector<uint8_t> test_message = {1, 2, 3, 4};
  auto* dummy_response_handle =
      reinterpret_cast<FlutterPlatformMessageResponseHandle*>(5);
  const FlutterDesktopBinaryReply reply_handler = [](auto... args) {};
  void* reply_user_data = reinterpret_cast<void*>(6);

  // When a response is requested, a handle should be created, passed as part
  // of the message, and then released.
  bool create_response_handle_called = false;
  modifier.embedder_api().PlatformMessageCreateResponseHandle =
      MOCK_ENGINE_PROC(
          PlatformMessageCreateResponseHandle,
          ([&create_response_handle_called, &reply_handler, reply_user_data,
            dummy_response_handle](auto engine, auto reply, auto user_data,
                                   auto response_handle) {
            create_response_handle_called = true;
            EXPECT_EQ(reply, reply_handler);
            EXPECT_EQ(user_data, reply_user_data);
            EXPECT_NE(response_handle, nullptr);
            *response_handle = dummy_response_handle;
            return kSuccess;
          }));
  bool release_response_handle_called = false;
  modifier.embedder_api().PlatformMessageReleaseResponseHandle =
      MOCK_ENGINE_PROC(
          PlatformMessageReleaseResponseHandle,
          ([&release_response_handle_called, dummy_response_handle](
               auto engine, auto response_handle) {
            release_response_handle_called = true;
            EXPECT_EQ(response_handle, dummy_response_handle);
            return kSuccess;
          }));
  bool send_message_called = false;
  modifier.embedder_api().SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage, ([&send_message_called, test_message,
                             dummy_response_handle](auto engine, auto message) {
        send_message_called = true;
        EXPECT_STREQ(message->channel, "test");
        EXPECT_EQ(message->message_size, test_message.size());
        EXPECT_EQ(memcmp(message->message, test_message.data(),
                         message->message_size),
                  0);
        EXPECT_EQ(message->response_handle, dummy_response_handle);
        return kSuccess;
      }));

  engine->SendPlatformMessage(channel, test_message.data(), test_message.size(),
                              reply_handler, reply_user_data);
  EXPECT_TRUE(create_response_handle_called);
  EXPECT_TRUE(release_response_handle_called);
  EXPECT_TRUE(send_message_called);
}

TEST_F(FlutterWindowsEngineTest, DispatchSemanticsAction) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  bool called = false;
  std::string message = "Hello";
  modifier.embedder_api().DispatchSemanticsAction = MOCK_ENGINE_PROC(
      DispatchSemanticsAction,
      ([&called, &message](auto engine, auto target, auto action, auto data,
                           auto data_length) {
        called = true;
        EXPECT_EQ(target, 42);
        EXPECT_EQ(action, kFlutterSemanticsActionDismiss);
        EXPECT_EQ(memcmp(data, message.c_str(), message.size()), 0);
        EXPECT_EQ(data_length, message.size());
        return kSuccess;
      }));

  auto data = fml::MallocMapping::Copy(message.c_str(), message.size());
  engine->DispatchSemanticsAction(42, kFlutterSemanticsActionDismiss,
                                  std::move(data));
  EXPECT_TRUE(called);
}

TEST_F(FlutterWindowsEngineTest, SetsThreadPriority) {
  WindowsPlatformThreadPrioritySetter(FlutterThreadPriority::kBackground);
  EXPECT_EQ(GetThreadPriority(GetCurrentThread()),
            THREAD_PRIORITY_BELOW_NORMAL);

  WindowsPlatformThreadPrioritySetter(FlutterThreadPriority::kDisplay);
  EXPECT_EQ(GetThreadPriority(GetCurrentThread()),
            THREAD_PRIORITY_ABOVE_NORMAL);

  WindowsPlatformThreadPrioritySetter(FlutterThreadPriority::kRaster);
  EXPECT_EQ(GetThreadPriority(GetCurrentThread()),
            THREAD_PRIORITY_ABOVE_NORMAL);

  // FlutterThreadPriority::kNormal does not change thread priority, reset to 0
  // here.
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_NORMAL);

  WindowsPlatformThreadPrioritySetter(FlutterThreadPriority::kNormal);
  EXPECT_EQ(GetThreadPriority(GetCurrentThread()), THREAD_PRIORITY_NORMAL);
}

TEST_F(FlutterWindowsEngineTest, AddPluginRegistrarDestructionCallback) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  MockEmbedderApiForKeyboard(modifier,
                             std::make_shared<MockKeyResponseController>());

  engine->Run();

  // Verify that destruction handlers don't overwrite each other.
  int result1 = 0;
  int result2 = 0;
  engine->AddPluginRegistrarDestructionCallback(
      [](FlutterDesktopPluginRegistrarRef ref) {
        auto result = reinterpret_cast<int*>(ref);
        *result = 1;
      },
      reinterpret_cast<FlutterDesktopPluginRegistrarRef>(&result1));
  engine->AddPluginRegistrarDestructionCallback(
      [](FlutterDesktopPluginRegistrarRef ref) {
        auto result = reinterpret_cast<int*>(ref);
        *result = 2;
      },
      reinterpret_cast<FlutterDesktopPluginRegistrarRef>(&result2));

  engine->Stop();
  EXPECT_EQ(result1, 1);
  EXPECT_EQ(result2, 2);
}

TEST_F(FlutterWindowsEngineTest, ScheduleFrame) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  bool called = false;
  modifier.embedder_api().ScheduleFrame =
      MOCK_ENGINE_PROC(ScheduleFrame, ([&called](auto engine) {
                         called = true;
                         return kSuccess;
                       }));

  engine->ScheduleFrame();
  EXPECT_TRUE(called);
}

TEST_F(FlutterWindowsEngineTest, SetNextFrameCallback) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  bool called = false;
  modifier.embedder_api().SetNextFrameCallback = MOCK_ENGINE_PROC(
      SetNextFrameCallback, ([&called](auto engine, auto callback, auto data) {
        called = true;
        return kSuccess;
      }));

  engine->SetNextFrameCallback([]() {});
  EXPECT_TRUE(called);
}

TEST_F(FlutterWindowsEngineTest, GetExecutableName) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EXPECT_EQ(engine->GetExecutableName(), "flutter_windows_unittests.exe");
}

// Ensure that after setting or resetting the high contrast feature,
// the corresponding status flag can be retrieved from the engine.
TEST_F(FlutterWindowsEngineTest, UpdateHighContrastFeature) {
  auto windows_proc_table = std::make_shared<MockWindowsProcTable>();
  EXPECT_CALL(*windows_proc_table, GetHighContrastEnabled)
      .WillOnce(Return(true))
      .WillOnce(Return(false));

  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetWindowsProcTable(windows_proc_table);
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  std::optional<FlutterAccessibilityFeature> engine_flags;
  modifier.embedder_api().UpdateAccessibilityFeatures = MOCK_ENGINE_PROC(
      UpdateAccessibilityFeatures, ([&engine_flags](auto engine, auto flags) {
        engine_flags = flags;
        return kSuccess;
      }));
  modifier.embedder_api().SendPlatformMessage = MOCK_ENGINE_PROC(
      SendPlatformMessage,
      [](auto engine, const auto message) { return kSuccess; });

  // 1: High contrast is enabled.
  engine->UpdateHighContrastMode();

  EXPECT_TRUE(engine->high_contrast_enabled());
  EXPECT_TRUE(engine_flags.has_value());
  EXPECT_TRUE(
      engine_flags.value() &
      FlutterAccessibilityFeature::kFlutterAccessibilityFeatureHighContrast);

  // 2: High contrast is disabled.
  engine_flags.reset();
  engine->UpdateHighContrastMode();

  EXPECT_FALSE(engine->high_contrast_enabled());
  EXPECT_TRUE(engine_flags.has_value());
  EXPECT_FALSE(
      engine_flags.value() &
      FlutterAccessibilityFeature::kFlutterAccessibilityFeatureHighContrast);
}

TEST_F(FlutterWindowsEngineTest, PostRasterThreadTask) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  std::unique_ptr<FlutterWindowsEngine> engine = builder.Build();
  EngineModifier modifier(engine.get());

  modifier.embedder_api().PostRenderThreadTask = MOCK_ENGINE_PROC(
      PostRenderThreadTask, ([](auto engine, auto callback, auto context) {
        callback(context);
        return kSuccess;
      }));

  bool called = false;
  engine->PostRasterThreadTask([&called]() { called = true; });

  EXPECT_TRUE(called);
}

class MockFlutterWindowsView : public FlutterWindowsView {
 public:
  MockFlutterWindowsView(FlutterWindowsEngine* engine,
                         std::unique_ptr<WindowBindingHandler> wbh)
      : FlutterWindowsView(kImplicitViewId, engine, std::move(wbh)) {}
  ~MockFlutterWindowsView() {}

  MOCK_METHOD(void,
              NotifyWinEventWrapper,
              (ui::AXPlatformNodeWin*, ax::mojom::Event),
              (override));
  MOCK_METHOD(HWND, GetWindowHandle, (), (const, override));

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MockFlutterWindowsView);
};

// Verify the view is notified of accessibility announcements.
TEST_F(FlutterWindowsEngineTest, AccessibilityAnnouncement) {
  auto& context = GetContext();
  WindowsConfigBuilder builder{context};
  builder.SetDartEntrypoint("sendAccessibilityAnnouncement");

  bool done = false;
  auto native_entry =
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { done = true; });
  context.AddNativeFunction("Signal", native_entry);

  EnginePtr engine{builder.RunHeadless()};
  ASSERT_NE(engine, nullptr);

  ui::AXPlatformNodeDelegateBase parent_delegate;
  AlertPlatformNodeDelegate delegate{parent_delegate};

  auto window_binding_handler =
      std::make_unique<NiceMock<MockWindowBindingHandler>>();
  EXPECT_CALL(*window_binding_handler, GetAlertDelegate)
      .WillOnce(Return(&delegate));

  auto windows_engine = reinterpret_cast<FlutterWindowsEngine*>(engine.get());
  MockFlutterWindowsView view{windows_engine,
                              std::move(window_binding_handler)};
  EngineModifier modifier{windows_engine};
  modifier.SetImplicitView(&view);

  windows_engine->UpdateSemanticsEnabled(true);

  EXPECT_CALL(view, NotifyWinEventWrapper).Times(1);

  // Rely on timeout mechanism in CI.
  while (!done) {
    windows_engine->task_runner()->ProcessTasks();
  }
}

// Verify the app can send accessibility announcements while in headless mode.
TEST_F(FlutterWindowsEngineTest, AccessibilityAnnouncementHeadless) {
  auto& context = GetContext();
  WindowsConfigBuilder builder{context};
  builder.SetDartEntrypoint("sendAccessibilityAnnouncement");

  bool done = false;
  auto native_entry =
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { done = true; });
  context.AddNativeFunction("Signal", native_entry);

  EnginePtr engine{builder.RunHeadless()};
  ASSERT_NE(engine, nullptr);

  auto windows_engine = reinterpret_cast<FlutterWindowsEngine*>(engine.get());
  windows_engine->UpdateSemanticsEnabled(true);

  // Rely on timeout mechanism in CI.
  while (!done) {
    windows_engine->task_runner()->ProcessTasks();
  }
}

// Verify the engine does not crash if it receives an accessibility event
// it does not support yet.
TEST_F(FlutterWindowsEngineTest, AccessibilityTooltip) {
  fml::testing::LogCapture log_capture;

  auto& context = GetContext();
  WindowsConfigBuilder builder{context};
  builder.SetDartEntrypoint("sendAccessibilityTooltipEvent");

  bool done = false;
  auto native_entry =
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { done = true; });
  context.AddNativeFunction("Signal", native_entry);

  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);

  auto engine = FlutterDesktopViewControllerGetEngine(controller.get());
  auto windows_engine = reinterpret_cast<FlutterWindowsEngine*>(engine);
  windows_engine->UpdateSemanticsEnabled(true);

  // Rely on timeout mechanism in CI.
  while (!done) {
    windows_engine->task_runner()->ProcessTasks();
  }

  // Verify no error was logged.
  // Regression test for:
  // https://github.com/flutter/flutter/issues/144274
  EXPECT_EQ(log_capture.str().find("tooltip"), std::string::npos);
}

class MockWindowsLifecycleManager : public WindowsLifecycleManager {
 public:
  MockWindowsLifecycleManager(FlutterWindowsEngine* engine)
      : WindowsLifecycleManager(engine) {}
  virtual ~MockWindowsLifecycleManager() {}

  MOCK_METHOD(
      void,
      Quit,
      (std::optional<HWND>, std::optional<WPARAM>, std::optional<LPARAM>, UINT),
      (override));
  MOCK_METHOD(void, DispatchMessage, (HWND, UINT, WPARAM, LPARAM), (override));
  MOCK_METHOD(bool, IsLastWindowOfProcess, (), (override));
  MOCK_METHOD(void, SetLifecycleState, (AppLifecycleState), (override));

  void BeginProcessingLifecycle() override {
    WindowsLifecycleManager::BeginProcessingLifecycle();
    if (begin_processing_callback) {
      begin_processing_callback();
    }
  }

  std::function<void()> begin_processing_callback = nullptr;
};

TEST_F(FlutterWindowsEngineTest, TestExit) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("exitTestExit");
  bool finished = false;

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, SetLifecycleState(AppLifecycleState::kResumed));
  EXPECT_CALL(*handler, Quit)
      .WillOnce([&finished](std::optional<HWND> hwnd,
                            std::optional<WPARAM> wparam,
                            std::optional<LPARAM> lparam,
                            UINT exit_code) { finished = exit_code == 0; });
  EXPECT_CALL(*handler, IsLastWindowOfProcess).WillRepeatedly(Return(true));
  modifier.SetLifecycleManager(std::move(handler));

  engine->lifecycle_manager()->BeginProcessingExit();

  engine->Run();

  engine->window_proc_delegate_manager()->OnTopLevelWindowProc(0, WM_CLOSE, 0,
                                                               0);

  // The test will only succeed when this while loop exits. Otherwise it will
  // timeout.
  while (!finished) {
    engine->task_runner()->ProcessTasks();
  }
}

TEST_F(FlutterWindowsEngineTest, TestExitCancel) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("exitTestCancel");
  bool did_call = false;

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, SetLifecycleState(AppLifecycleState::kResumed));
  EXPECT_CALL(*handler, IsLastWindowOfProcess).WillRepeatedly(Return(true));
  EXPECT_CALL(*handler, Quit).Times(0);
  modifier.SetLifecycleManager(std::move(handler));
  engine->lifecycle_manager()->BeginProcessingExit();

  auto binary_messenger =
      std::make_unique<BinaryMessengerImpl>(engine->messenger());
  binary_messenger->SetMessageHandler(
      "flutter/platform", [&did_call](const uint8_t* message,
                                      size_t message_size, BinaryReply reply) {
        std::string contents(message, message + message_size);
        EXPECT_NE(contents.find("\"method\":\"System.exitApplication\""),
                  std::string::npos);
        EXPECT_NE(contents.find("\"type\":\"required\""), std::string::npos);
        EXPECT_NE(contents.find("\"exitCode\":0"), std::string::npos);
        did_call = true;
        char response[] = "";
        reply(reinterpret_cast<uint8_t*>(response), 0);
      });

  engine->Run();

  engine->window_proc_delegate_manager()->OnTopLevelWindowProc(0, WM_CLOSE, 0,
                                                               0);

  while (!did_call) {
    engine->task_runner()->ProcessTasks();
  }
}

// Flutter consumes the first WM_CLOSE message to allow the app to cancel the
// exit. If the app does not cancel the exit, Flutter synthesizes a second
// WM_CLOSE message.
TEST_F(FlutterWindowsEngineTest, TestExitSecondCloseMessage) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("exitTestExit");
  bool second_close = false;

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, SetLifecycleState(AppLifecycleState::kResumed));
  EXPECT_CALL(*handler, IsLastWindowOfProcess).WillOnce(Return(true));
  EXPECT_CALL(*handler, Quit)
      .WillOnce([handler_ptr = handler.get()](
                    std::optional<HWND> hwnd, std::optional<WPARAM> wparam,
                    std::optional<LPARAM> lparam, UINT exit_code) {
        handler_ptr->WindowsLifecycleManager::Quit(hwnd, wparam, lparam,
                                                   exit_code);
      });
  EXPECT_CALL(*handler, DispatchMessage)
      .WillRepeatedly(
          [&engine](HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam) {
            engine->window_proc_delegate_manager()->OnTopLevelWindowProc(
                hwnd, msg, wparam, lparam);
          });
  modifier.SetLifecycleManager(std::move(handler));
  engine->lifecycle_manager()->BeginProcessingExit();

  engine->Run();

  // This delegate will be registered after the lifecycle manager, so it will be
  // called only when a message is not consumed by the lifecycle manager. This
  // should be called on the second, synthesized WM_CLOSE message that the
  // lifecycle manager posts.
  engine->window_proc_delegate_manager()->RegisterTopLevelWindowProcDelegate(
      [](HWND hwnd, UINT message, WPARAM wpar, LPARAM lpar, void* user_data,
         LRESULT* result) {
        switch (message) {
          case WM_CLOSE: {
            bool* called = reinterpret_cast<bool*>(user_data);
            *called = true;
            return true;
          }
        }
        return false;
      },
      reinterpret_cast<void*>(&second_close));

  engine->window_proc_delegate_manager()->OnTopLevelWindowProc(0, WM_CLOSE, 0,
                                                               0);

  while (!second_close) {
    engine->task_runner()->ProcessTasks();
  }
}

TEST_F(FlutterWindowsEngineTest, TestExitCloseMultiWindow) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("exitTestExit");
  bool finished = false;

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, SetLifecycleState(AppLifecycleState::kResumed));
  EXPECT_CALL(*handler, IsLastWindowOfProcess).WillOnce([&finished]() {
    finished = true;
    return false;
  });
  // Quit should not be called when there is more than one window.
  EXPECT_CALL(*handler, Quit).Times(0);
  modifier.SetLifecycleManager(std::move(handler));
  engine->lifecycle_manager()->BeginProcessingExit();

  engine->Run();

  engine->window_proc_delegate_manager()->OnTopLevelWindowProc(0, WM_CLOSE, 0,
                                                               0);

  while (!finished) {
    engine->task_runner()->ProcessTasks();
  }
}

TEST_F(FlutterWindowsEngineTest, LifecycleManagerDisabledByDefault) {
  FlutterWindowsEngineBuilder builder{GetContext()};

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, IsLastWindowOfProcess).Times(0);
  modifier.SetLifecycleManager(std::move(handler));

  engine->window_proc_delegate_manager()->OnTopLevelWindowProc(0, WM_CLOSE, 0,
                                                               0);
}

TEST_F(FlutterWindowsEngineTest, EnableApplicationLifecycle) {
  FlutterWindowsEngineBuilder builder{GetContext()};

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, IsLastWindowOfProcess).WillOnce(Return(false));
  modifier.SetLifecycleManager(std::move(handler));
  engine->lifecycle_manager()->BeginProcessingExit();

  engine->window_proc_delegate_manager()->OnTopLevelWindowProc(0, WM_CLOSE, 0,
                                                               0);
}

TEST_F(FlutterWindowsEngineTest, ApplicationLifecycleExternalWindow) {
  FlutterWindowsEngineBuilder builder{GetContext()};

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, IsLastWindowOfProcess).WillOnce(Return(false));
  modifier.SetLifecycleManager(std::move(handler));
  engine->lifecycle_manager()->BeginProcessingExit();

  engine->lifecycle_manager()->ExternalWindowMessage(0, WM_CLOSE, 0, 0);
}

TEST_F(FlutterWindowsEngineTest, AppStartsInResumedState) {
  FlutterWindowsEngineBuilder builder{GetContext()};

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, SetLifecycleState(AppLifecycleState::kResumed))
      .Times(1);
  modifier.SetLifecycleManager(std::move(handler));
  engine->Run();
}

TEST_F(FlutterWindowsEngineTest, LifecycleStateTransition) {
  FlutterWindowsEngineBuilder builder{GetContext()};

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  engine->Run();

  engine->window_proc_delegate_manager()->OnTopLevelWindowProc(
      (HWND)1, WM_SIZE, SIZE_RESTORED, 0);
  EXPECT_EQ(engine->lifecycle_manager()->GetLifecycleState(),
            AppLifecycleState::kResumed);

  engine->window_proc_delegate_manager()->OnTopLevelWindowProc(
      (HWND)1, WM_SIZE, SIZE_MINIMIZED, 0);
  EXPECT_EQ(engine->lifecycle_manager()->GetLifecycleState(),
            AppLifecycleState::kHidden);

  engine->window_proc_delegate_manager()->OnTopLevelWindowProc(
      (HWND)1, WM_SIZE, SIZE_RESTORED, 0);
  EXPECT_EQ(engine->lifecycle_manager()->GetLifecycleState(),
            AppLifecycleState::kInactive);
}

TEST_F(FlutterWindowsEngineTest, ExternalWindowMessage) {
  FlutterWindowsEngineBuilder builder{GetContext()};

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  // Sets lifecycle state to resumed.
  engine->Run();

  // Ensure HWND(1) is in the set of visible windows before hiding it.
  engine->ProcessExternalWindowMessage(reinterpret_cast<HWND>(1), WM_SHOWWINDOW,
                                       TRUE, NULL);
  engine->ProcessExternalWindowMessage(reinterpret_cast<HWND>(1), WM_SHOWWINDOW,
                                       FALSE, NULL);

  EXPECT_EQ(engine->lifecycle_manager()->GetLifecycleState(),
            AppLifecycleState::kHidden);
}

TEST_F(FlutterWindowsEngineTest, InnerWindowHidden) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  HWND outer = reinterpret_cast<HWND>(1);
  HWND inner = reinterpret_cast<HWND>(2);

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));
  ON_CALL(view, GetWindowHandle).WillByDefault([=]() { return inner; });

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  // Sets lifecycle state to resumed.
  engine->Run();

  // Show both top-level and Flutter window.
  engine->window_proc_delegate_manager()->OnTopLevelWindowProc(
      outer, WM_SHOWWINDOW, TRUE, NULL);
  view.OnWindowStateEvent(inner, WindowStateEvent::kShow);
  view.OnWindowStateEvent(inner, WindowStateEvent::kFocus);

  EXPECT_EQ(engine->lifecycle_manager()->GetLifecycleState(),
            AppLifecycleState::kResumed);

  // Hide Flutter window, but not top level window.
  view.OnWindowStateEvent(inner, WindowStateEvent::kHide);

  // The top-level window is still visible, so we ought not enter hidden state.
  EXPECT_EQ(engine->lifecycle_manager()->GetLifecycleState(),
            AppLifecycleState::kInactive);
}

TEST_F(FlutterWindowsEngineTest, EnableLifecycleState) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("enableLifecycleTest");
  bool finished = false;

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, SetLifecycleState)
      .WillRepeatedly([handler_ptr = handler.get()](AppLifecycleState state) {
        handler_ptr->WindowsLifecycleManager::SetLifecycleState(state);
      });
  modifier.SetLifecycleManager(std::move(handler));

  auto binary_messenger =
      std::make_unique<BinaryMessengerImpl>(engine->messenger());
  // Mark the test only as completed on receiving an inactive state message.
  binary_messenger->SetMessageHandler(
      "flutter/unittest", [&finished](const uint8_t* message,
                                      size_t message_size, BinaryReply reply) {
        std::string contents(message, message + message_size);
        EXPECT_NE(contents.find("AppLifecycleState.inactive"),
                  std::string::npos);
        finished = true;
        char response[] = "";
        reply(reinterpret_cast<uint8_t*>(response), 0);
      });

  engine->Run();

  // Test that setting the state before enabling lifecycle does nothing.
  HWND hwnd = reinterpret_cast<HWND>(1);
  view.OnWindowStateEvent(hwnd, WindowStateEvent::kShow);
  view.OnWindowStateEvent(hwnd, WindowStateEvent::kHide);
  EXPECT_FALSE(finished);

  // Test that we can set the state afterwards.

  engine->lifecycle_manager()->BeginProcessingLifecycle();
  view.OnWindowStateEvent(hwnd, WindowStateEvent::kShow);

  while (!finished) {
    engine->task_runner()->ProcessTasks();
  }
}

TEST_F(FlutterWindowsEngineTest, LifecycleStateToFrom) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("enableLifecycleToFrom");
  bool enabled_lifecycle = false;
  bool dart_responded = false;

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, SetLifecycleState)
      .WillRepeatedly([handler_ptr = handler.get()](AppLifecycleState state) {
        handler_ptr->WindowsLifecycleManager::SetLifecycleState(state);
      });
  handler->begin_processing_callback = [&]() { enabled_lifecycle = true; };
  modifier.SetLifecycleManager(std::move(handler));

  auto binary_messenger =
      std::make_unique<BinaryMessengerImpl>(engine->messenger());
  binary_messenger->SetMessageHandler(
      "flutter/unittest",
      [&](const uint8_t* message, size_t message_size, BinaryReply reply) {
        std::string contents(message, message + message_size);
        EXPECT_NE(contents.find("AppLifecycleState."), std::string::npos);
        dart_responded = true;
        char response[] = "";
        reply(reinterpret_cast<uint8_t*>(response), 0);
      });

  engine->Run();

  while (!enabled_lifecycle) {
    engine->task_runner()->ProcessTasks();
  }

  HWND hwnd = reinterpret_cast<HWND>(1);
  view.OnWindowStateEvent(hwnd, WindowStateEvent::kShow);
  view.OnWindowStateEvent(hwnd, WindowStateEvent::kHide);

  while (!dart_responded) {
    engine->task_runner()->ProcessTasks();
  }
}

TEST_F(FlutterWindowsEngineTest, ChannelListenedTo) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("enableLifecycleToFrom");

  auto engine = builder.Build();
  auto window_binding_handler =
      std::make_unique<::testing::NiceMock<MockWindowBindingHandler>>();
  MockFlutterWindowsView view(engine.get(), std::move(window_binding_handler));

  EngineModifier modifier(engine.get());
  modifier.SetImplicitView(&view);
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };

  bool lifecycle_began = false;
  auto handler = std::make_unique<MockWindowsLifecycleManager>(engine.get());
  EXPECT_CALL(*handler, SetLifecycleState).Times(1);
  handler->begin_processing_callback = [&]() { lifecycle_began = true; };
  modifier.SetLifecycleManager(std::move(handler));

  engine->Run();

  while (!lifecycle_began) {
    engine->task_runner()->ProcessTasks();
  }
}

TEST_F(FlutterWindowsEngineTest, ReceivePlatformViewMessage) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("sendCreatePlatformViewMethod");
  auto engine = builder.Build();

  EngineModifier modifier{engine.get()};
  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };

  bool received_call = false;

  auto manager = std::make_unique<MockPlatformViewManager>(engine.get());
  EXPECT_CALL(*manager, AddPlatformView)
      .WillOnce([&](PlatformViewId id, std::string_view type_name) {
        received_call = true;
        return true;
      });
  modifier.SetPlatformViewPlugin(std::move(manager));

  engine->Run();

  while (!received_call) {
    engine->task_runner()->ProcessTasks();
  }
}

TEST_F(FlutterWindowsEngineTest, AddViewFailureDoesNotHang) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  auto engine = builder.Build();

  EngineModifier modifier{engine.get()};

  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  modifier.embedder_api().AddView = MOCK_ENGINE_PROC(
      AddView,
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine,
         const FlutterAddViewInfo* info) { return kInternalInconsistency; });

  ASSERT_TRUE(engine->Run());

  // Create the first view. This is the implicit view and isn't added to the
  // engine.
  auto implicit_window = std::make_unique<NiceMock<MockWindowBindingHandler>>();

  std::unique_ptr<FlutterWindowsView> implicit_view =
      engine->CreateView(std::move(implicit_window));

  EXPECT_TRUE(implicit_view);

  // Create a second view. The embedder attempts to add it to the engine.
  auto second_window = std::make_unique<NiceMock<MockWindowBindingHandler>>();

  EXPECT_DEBUG_DEATH(engine->CreateView(std::move(second_window)),
                     "FlutterEngineAddView returned an unexpected result");
}

TEST_F(FlutterWindowsEngineTest, RemoveViewFailureDoesNotHang) {
  FlutterWindowsEngineBuilder builder{GetContext()};
  builder.SetDartEntrypoint("sendCreatePlatformViewMethod");
  auto engine = builder.Build();

  EngineModifier modifier{engine.get()};

  modifier.embedder_api().RunsAOTCompiledDartCode = []() { return false; };
  modifier.embedder_api().RemoveView = MOCK_ENGINE_PROC(
      RemoveView,
      [](FLUTTER_API_SYMBOL(FlutterEngine) engine,
         const FlutterRemoveViewInfo* info) { return kInternalInconsistency; });

  ASSERT_TRUE(engine->Run());
  EXPECT_DEBUG_DEATH(engine->RemoveView(123),
                     "FlutterEngineRemoveView returned an unexpected result");
}

}  // namespace testing
}  // namespace flutter

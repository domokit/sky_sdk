// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/public/flutter_windows.h"

#include <dxgi.h>
#include <wrl/client.h>
#include <thread>

#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/shell/platform/common/app_lifecycle_state.h"
#include "flutter/shell/platform/embedder/test_utils/proc_table_replacement.h"
#include "flutter/shell/platform/windows/egl/manager.h"
#include "flutter/shell/platform/windows/testing/engine_modifier.h"
#include "flutter/shell/platform/windows/testing/windows_test.h"
#include "flutter/shell/platform/windows/testing/windows_test_config_builder.h"
#include "flutter/shell/platform/windows/testing/windows_test_context.h"
#include "flutter/shell/platform/windows/windows_lifecycle_manager.h"
#include "flutter/testing/stream_capture.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "third_party/tonic/converter/dart_converter.h"

namespace flutter {
namespace testing {

namespace {

// An EGL manager that initializes EGL but fails to create surfaces.
class HalfBrokenEGLManager : public egl::Manager {
 public:
  HalfBrokenEGLManager() : egl::Manager() {}

  std::unique_ptr<egl::WindowSurface>
  CreateWindowSurface(HWND hwnd, size_t width, size_t height) override {
    return nullptr;
  }
};

class MockWindowsLifecycleManager : public WindowsLifecycleManager {
 public:
  MockWindowsLifecycleManager(FlutterWindowsEngine* engine)
      : WindowsLifecycleManager(engine) {}

  MOCK_METHOD(void, SetLifecycleState, (AppLifecycleState), (override));
};

// Process the next win32 message if there is one. This can be used to
// pump the Windows platform thread task runner.
void PumpMessage() {
  ::MSG msg;
  if (::GetMessage(&msg, nullptr, 0, 0)) {
    ::TranslateMessage(&msg);
    ::DispatchMessage(&msg);
  }
}

}  // namespace

// Verify that we can fetch a texture registrar.
// Prevent regression: https://github.com/flutter/flutter/issues/86617
TEST(WindowsNoFixtureTest, GetTextureRegistrar) {
  FlutterDesktopEngineProperties properties = {};
  properties.assets_path = L"";
  properties.icu_data_path = L"icudtl.dat";
  auto engine = FlutterDesktopEngineCreate(&properties);
  ASSERT_NE(engine, nullptr);
  auto texture_registrar = FlutterDesktopEngineGetTextureRegistrar(engine);
  EXPECT_NE(texture_registrar, nullptr);
  FlutterDesktopEngineDestroy(engine);
}

// Verify we can successfully launch main().
TEST_F(WindowsTest, LaunchMain) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);
}

// Verify there is no unexpected output from launching main.
TEST_F(WindowsTest, LaunchMainHasNoOutput) {
  // Replace stdout & stderr stream buffers with our own.
  StreamCapture stdout_capture(&std::cout);
  StreamCapture stderr_capture(&std::cerr);

  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);

  stdout_capture.Stop();
  stderr_capture.Stop();

  // Verify stdout & stderr have no output.
  EXPECT_TRUE(stdout_capture.GetOutput().empty());
  EXPECT_TRUE(stderr_capture.GetOutput().empty());
}

// Verify we can successfully launch a custom entry point.
TEST_F(WindowsTest, LaunchCustomEntrypoint) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  builder.SetDartEntrypoint("customEntrypoint");
  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);
}

// Verify that engine launches with the custom entrypoint specified in the
// FlutterDesktopEngineRun parameter when no entrypoint is specified in
// FlutterDesktopEngineProperties.dart_entrypoint.
//
// TODO(cbracken): https://github.com/flutter/flutter/issues/109285
TEST_F(WindowsTest, LaunchCustomEntrypointInEngineRunInvocation) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  EnginePtr engine{builder.InitializeEngine()};
  ASSERT_NE(engine, nullptr);

  ASSERT_TRUE(FlutterDesktopEngineRun(engine.get(), "customEntrypoint"));
}

// Verify that the engine can launch in headless mode.
TEST_F(WindowsTest, LaunchHeadlessEngine) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  builder.SetDartEntrypoint("signalViewIds");
  EnginePtr engine{builder.RunHeadless()};
  ASSERT_NE(engine, nullptr);

  std::string view_ids;
  fml::AutoResetWaitableEvent latch;
  context.AddNativeFunction(
      "SignalStringValue", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto handle = Dart_GetNativeArgument(args, 0);
        ASSERT_FALSE(Dart_IsError(handle));
        view_ids = tonic::DartConverter<std::string>::FromDart(handle);
        latch.Signal();
      }));

  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);

  // Verify a headless app has the implicit view.
  latch.Wait();
  EXPECT_EQ(view_ids, "View IDs: [0]");
}

// Verify that the engine can return to headless mode.
TEST_F(WindowsTest, EngineCanTransitionToHeadless) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  EnginePtr engine{builder.RunHeadless()};
  ASSERT_NE(engine, nullptr);

  // Create and then destroy a view controller that does not own its engine.
  // This causes the engine to transition back to headless mode.
  {
    FlutterDesktopViewControllerProperties properties = {};
    ViewControllerPtr controller{
        FlutterDesktopEngineCreateViewController(engine.get(), &properties)};

    ASSERT_NE(controller, nullptr);
  }

  // The engine is back in headless mode now.
  ASSERT_NE(engine, nullptr);

  auto engine_ptr = reinterpret_cast<FlutterWindowsEngine*>(engine.get());
  ASSERT_TRUE(engine_ptr->running());
}

// Verify that accessibility features are initialized when a view is created.
TEST_F(WindowsTest, LaunchRefreshesAccessibility) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  EnginePtr engine{builder.InitializeEngine()};
  EngineModifier modifier{
      reinterpret_cast<FlutterWindowsEngine*>(engine.get())};

  auto called = false;
  modifier.embedder_api().UpdateAccessibilityFeatures = MOCK_ENGINE_PROC(
      UpdateAccessibilityFeatures, ([&called](auto engine, auto flags) {
        called = true;
        return kSuccess;
      }));

  ViewControllerPtr controller{
      FlutterDesktopViewControllerCreate(0, 0, engine.release())};

  ASSERT_TRUE(called);
}

// Verify that engine fails to launch when a conflicting entrypoint in
// FlutterDesktopEngineProperties.dart_entrypoint and the
// FlutterDesktopEngineRun parameter.
//
// TODO(cbracken): https://github.com/flutter/flutter/issues/109285
TEST_F(WindowsTest, LaunchConflictingCustomEntrypoints) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  builder.SetDartEntrypoint("customEntrypoint");
  EnginePtr engine{builder.InitializeEngine()};
  ASSERT_NE(engine, nullptr);

  ASSERT_FALSE(FlutterDesktopEngineRun(engine.get(), "conflictingEntrypoint"));
}

// Verify that native functions can be registered and resolved.
TEST_F(WindowsTest, VerifyNativeFunction) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  builder.SetDartEntrypoint("verifyNativeFunction");

  fml::AutoResetWaitableEvent latch;
  auto native_entry =
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); });
  context.AddNativeFunction("Signal", native_entry);

  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);

  // Wait until signal has been called.
  latch.Wait();
}

// Verify that native functions that pass parameters can be registered and
// resolved.
TEST_F(WindowsTest, VerifyNativeFunctionWithParameters) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  builder.SetDartEntrypoint("verifyNativeFunctionWithParameters");

  bool bool_value = false;
  fml::AutoResetWaitableEvent latch;
  auto native_entry = CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeBooleanArgument(args, 0, &bool_value);
    ASSERT_FALSE(Dart_IsError(handle));
    latch.Signal();
  });
  context.AddNativeFunction("SignalBoolValue", native_entry);

  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);

  // Wait until signalBoolValue has been called.
  latch.Wait();
  EXPECT_TRUE(bool_value);
}

// Verify that Platform.executable returns the executable name.
TEST_F(WindowsTest, PlatformExecutable) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  builder.SetDartEntrypoint("readPlatformExecutable");

  std::string executable_name;
  fml::AutoResetWaitableEvent latch;
  auto native_entry = CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    ASSERT_FALSE(Dart_IsError(handle));
    executable_name = tonic::DartConverter<std::string>::FromDart(handle);
    latch.Signal();
  });
  context.AddNativeFunction("SignalStringValue", native_entry);

  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);

  // Wait until signalStringValue has been called.
  latch.Wait();
  EXPECT_EQ(executable_name, "flutter_windows_unittests.exe");
}

// Verify that native functions that return values can be registered and
// resolved.
TEST_F(WindowsTest, VerifyNativeFunctionWithReturn) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  builder.SetDartEntrypoint("verifyNativeFunctionWithReturn");

  bool bool_value_to_return = true;
  fml::CountDownLatch latch(2);
  auto bool_return_entry = CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
    Dart_SetBooleanReturnValue(args, bool_value_to_return);
    latch.CountDown();
  });
  context.AddNativeFunction("SignalBoolReturn", bool_return_entry);

  bool bool_value_passed = false;
  auto bool_pass_entry = CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeBooleanArgument(args, 0, &bool_value_passed);
    ASSERT_FALSE(Dart_IsError(handle));
    latch.CountDown();
  });
  context.AddNativeFunction("SignalBoolValue", bool_pass_entry);

  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);

  // Wait until signalBoolReturn and signalBoolValue have been called.
  latch.Wait();
  EXPECT_TRUE(bool_value_passed);
}

// Verify the next frame callback is executed.
TEST_F(WindowsTest, NextFrameCallback) {
  struct Captures {
    fml::AutoResetWaitableEvent frame_scheduled_latch;
    fml::AutoResetWaitableEvent frame_drawn_latch;
    std::thread::id thread_id;
    bool done = false;
  };
  Captures captures;

  CreateNewThread("test_platform_thread")->PostTask([&]() {
    captures.thread_id = std::this_thread::get_id();

    auto& context = GetContext();
    WindowsConfigBuilder builder(context);
    builder.SetDartEntrypoint("drawHelloWorld");

    auto native_entry = CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
      ASSERT_FALSE(captures.frame_drawn_latch.IsSignaledForTest());
      captures.frame_scheduled_latch.Signal();
    });
    context.AddNativeFunction("NotifyFirstFrameScheduled", native_entry);

    ViewControllerPtr controller{builder.Run()};
    ASSERT_NE(controller, nullptr);

    auto engine = FlutterDesktopViewControllerGetEngine(controller.get());

    FlutterDesktopEngineSetNextFrameCallback(
        engine,
        [](void* user_data) {
          auto captures = static_cast<Captures*>(user_data);

          ASSERT_TRUE(captures->frame_scheduled_latch.IsSignaledForTest());

          // Callback should execute on platform thread.
          ASSERT_EQ(std::this_thread::get_id(), captures->thread_id);

          // Signal the test passed and end the Windows message loop.
          captures->done = true;
          captures->frame_drawn_latch.Signal();
        },
        &captures);

    // Pump messages for the Windows platform task runner.
    while (!captures.done) {
      PumpMessage();
    }
  });

  captures.frame_drawn_latch.Wait();
}

// Verify the embedder ignores presents to the implicit view when there is no
// implicit view.
TEST_F(WindowsTest, PresentHeadless) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  builder.SetDartEntrypoint("renderImplicitView");

  EnginePtr engine{builder.RunHeadless()};
  ASSERT_NE(engine, nullptr);

  bool done = false;
  FlutterDesktopEngineSetNextFrameCallback(
      engine.get(),
      [](void* user_data) {
        // This executes on the platform thread.
        auto done = reinterpret_cast<std::atomic<bool>*>(user_data);
        *done = true;
      },
      &done);

  // This app is in headless mode, however, the engine assumes the implicit
  // view always exists. Send window metrics for the implicit view, causing
  // the engine to present to the implicit view. The embedder must not crash.
  auto engine_ptr = reinterpret_cast<FlutterWindowsEngine*>(engine.get());
  FlutterWindowMetricsEvent metrics = {};
  metrics.struct_size = sizeof(FlutterWindowMetricsEvent);
  metrics.width = 100;
  metrics.height = 100;
  metrics.pixel_ratio = 1.0;
  metrics.view_id = kImplicitViewId;
  engine_ptr->SendWindowMetricsEvent(metrics);

  // Pump messages for the Windows platform task runner.
  while (!done) {
    PumpMessage();
  }
}

// Implicit view has the implicit view ID.
TEST_F(WindowsTest, GetViewId) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);
  FlutterDesktopViewId view_id =
      FlutterDesktopViewControllerGetViewId(controller.get());

  ASSERT_EQ(view_id, static_cast<FlutterDesktopViewId>(kImplicitViewId));
}

TEST_F(WindowsTest, GetGraphicsAdapter) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);
  auto view = FlutterDesktopViewControllerGetView(controller.get());

  Microsoft::WRL::ComPtr<IDXGIAdapter> dxgi_adapter;
  dxgi_adapter = FlutterDesktopViewGetGraphicsAdapter(view);
  ASSERT_NE(dxgi_adapter, nullptr);
  DXGI_ADAPTER_DESC desc{};
  ASSERT_TRUE(SUCCEEDED(dxgi_adapter->GetDesc(&desc)));
}

// Implicit view has the implicit view ID.
TEST_F(WindowsTest, PluginRegistrarGetImplicitView) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);

  FlutterDesktopEngineRef engine =
      FlutterDesktopViewControllerGetEngine(controller.get());
  FlutterDesktopPluginRegistrarRef registrar =
      FlutterDesktopEngineGetPluginRegistrar(engine, "foo_bar");
  FlutterDesktopViewRef implicit_view =
      FlutterDesktopPluginRegistrarGetView(registrar);

  ASSERT_NE(implicit_view, nullptr);
}

TEST_F(WindowsTest, PluginRegistrarGetView) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);

  FlutterDesktopEngineRef engine =
      FlutterDesktopViewControllerGetEngine(controller.get());
  FlutterDesktopPluginRegistrarRef registrar =
      FlutterDesktopEngineGetPluginRegistrar(engine, "foo_bar");

  FlutterDesktopViewId view_id =
      FlutterDesktopViewControllerGetViewId(controller.get());
  FlutterDesktopViewRef view =
      FlutterDesktopPluginRegistrarGetViewById(registrar, view_id);

  FlutterDesktopViewRef view_123 = FlutterDesktopPluginRegistrarGetViewById(
      registrar, static_cast<FlutterDesktopViewId>(123));

  ASSERT_NE(view, nullptr);
  ASSERT_EQ(view_123, nullptr);
}

TEST_F(WindowsTest, PluginRegistrarGetViewHeadless) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  EnginePtr engine{builder.RunHeadless()};
  ASSERT_NE(engine, nullptr);

  FlutterDesktopPluginRegistrarRef registrar =
      FlutterDesktopEngineGetPluginRegistrar(engine.get(), "foo_bar");

  FlutterDesktopViewRef implicit_view =
      FlutterDesktopPluginRegistrarGetView(registrar);
  FlutterDesktopViewRef view_123 = FlutterDesktopPluginRegistrarGetViewById(
      registrar, static_cast<FlutterDesktopViewId>(123));

  ASSERT_EQ(implicit_view, nullptr);
  ASSERT_EQ(view_123, nullptr);
}

// Verify the app does not crash if EGL initializes successfully but
// the rendering surface cannot be created.
TEST_F(WindowsTest, SurfaceOptional) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  EnginePtr engine{builder.InitializeEngine()};
  EngineModifier modifier{
      reinterpret_cast<FlutterWindowsEngine*>(engine.get())};

  auto egl_manager = std::make_unique<HalfBrokenEGLManager>();
  ASSERT_TRUE(egl_manager->IsValid());
  modifier.SetEGLManager(std::move(egl_manager));

  ViewControllerPtr controller{
      FlutterDesktopViewControllerCreate(0, 0, engine.release())};

  ASSERT_NE(controller, nullptr);
}

// Verify the app produces the expected lifecycle events.
TEST_F(WindowsTest, Lifecycle) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  EnginePtr engine{builder.InitializeEngine()};
  auto windows_engine = reinterpret_cast<FlutterWindowsEngine*>(engine.get());
  EngineModifier modifier{windows_engine};

  auto lifecycle_manager =
      std::make_unique<MockWindowsLifecycleManager>(windows_engine);
  auto lifecycle_manager_ptr = lifecycle_manager.get();
  modifier.SetLifecycleManager(std::move(lifecycle_manager));

  EXPECT_CALL(*lifecycle_manager_ptr,
              SetLifecycleState(AppLifecycleState::kResumed))
      .WillOnce([lifecycle_manager_ptr](AppLifecycleState state) {
        lifecycle_manager_ptr->WindowsLifecycleManager::SetLifecycleState(
            state);
      });

  EXPECT_CALL(*lifecycle_manager_ptr,
              SetLifecycleState(AppLifecycleState::kHidden))
      .WillOnce([lifecycle_manager_ptr](AppLifecycleState state) {
        lifecycle_manager_ptr->WindowsLifecycleManager::SetLifecycleState(
            state);
      });

  // Create a controller. This launches the engine and sets the app lifecycle
  // to the "resumed" state.
  ViewControllerPtr controller{
      FlutterDesktopViewControllerCreate(0, 0, engine.release())};

  FlutterDesktopViewRef view =
      FlutterDesktopViewControllerGetView(controller.get());
  ASSERT_NE(view, nullptr);

  HWND hwnd = FlutterDesktopViewGetHWND(view);
  ASSERT_NE(hwnd, nullptr);

  // Give the window a non-zero size to show it. This does not change the app
  // lifecycle directly. However, destroying the view will now result in a
  // "hidden" app lifecycle event.
  ::MoveWindow(hwnd, /* X */ 0, /* Y */ 0, /* nWidth*/ 100, /* nHeight*/ 100,
               /* bRepaint*/ false);
}

TEST_F(WindowsTest, GetKeyboardStateHeadless) {
  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  builder.SetDartEntrypoint("sendGetKeyboardState");

  std::atomic<bool> done = false;
  context.AddNativeFunction(
      "SignalStringValue", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto handle = Dart_GetNativeArgument(args, 0);
        ASSERT_FALSE(Dart_IsError(handle));
        auto value = tonic::DartConverter<std::string>::FromDart(handle);
        EXPECT_EQ(value, "Success");
        done = true;
      }));

  ViewControllerPtr controller{builder.Run()};
  ASSERT_NE(controller, nullptr);

  // Pump messages for the Windows platform task runner.
  ::MSG msg;
  while (!done) {
    PumpMessage();
  }
}

// Verify the embedder can add and remove views.
TEST_F(WindowsTest, AddRemoveView) {
  std::mutex mutex;
  std::string view_ids;

  auto& context = GetContext();
  WindowsConfigBuilder builder(context);
  builder.SetDartEntrypoint("onMetricsChangedSignalViewIds");

  fml::AutoResetWaitableEvent ready_latch;
  context.AddNativeFunction(
      "Signal", CREATE_NATIVE_ENTRY(
                    [&](Dart_NativeArguments args) { ready_latch.Signal(); }));

  context.AddNativeFunction(
      "SignalStringValue", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto handle = Dart_GetNativeArgument(args, 0);
        ASSERT_FALSE(Dart_IsError(handle));

        std::scoped_lock lock{mutex};
        view_ids = tonic::DartConverter<std::string>::FromDart(handle);
      }));

  // Create the implicit view.
  ViewControllerPtr first_controller{builder.Run()};
  ASSERT_NE(first_controller, nullptr);

  ready_latch.Wait();

  // Create a second view.
  FlutterDesktopEngineRef engine =
      FlutterDesktopViewControllerGetEngine(first_controller.get());
  FlutterDesktopViewControllerProperties properties = {};
  properties.width = 100;
  properties.height = 100;
  ViewControllerPtr second_controller{
      FlutterDesktopEngineCreateViewController(engine, &properties)};
  ASSERT_NE(second_controller, nullptr);

  // Pump messages for the Windows platform task runner until the view is added.
  while (true) {
    PumpMessage();
    std::scoped_lock lock{mutex};
    if (view_ids == "View IDs: [0, 1]") {
      break;
    }
  }

  // Delete the second view and pump messages for the Windows platform task
  // runner until the view is removed.
  second_controller.reset();
  while (true) {
    PumpMessage();
    std::scoped_lock lock{mutex};
    if (view_ids == "View IDs: [0]") {
      break;
    }
  }
}

}  // namespace testing
}  // namespace flutter

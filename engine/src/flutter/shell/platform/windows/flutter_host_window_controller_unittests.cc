// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.
#include "flutter/shell/platform/windows/windowing_handler.h"

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/standard_method_codec.h"
#include "flutter/shell/platform/windows/flutter_host_window_controller.h"
#include "flutter/shell/platform/windows/testing/flutter_windows_engine_builder.h"
#include "flutter/shell/platform/windows/testing/test_binary_messenger.h"
#include "flutter/shell/platform/windows/testing/windows_test.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

namespace {

constexpr char kChannelName[] = "flutter/windowing";
constexpr char kOnWindowChangedMethod[] = "onWindowChanged";
constexpr char kOnWindowDestroyedMethod[] = "onWindowDestroyed";
constexpr char kSizeKey[] = "size";
constexpr char kStateKey[] = "state";
constexpr char kTitleKey[] = "title";
constexpr char kViewIdKey[] = "viewId";

// Process the next Win32 message if there is one. This can be used to
// pump the Windows platform thread task runner.
void PumpMessage() {
  ::MSG msg;
  if (::GetMessage(&msg, nullptr, 0, 0)) {
    ::TranslateMessage(&msg);
    ::DispatchMessage(&msg);
  }
}

Size GetLogicalClientSize(HWND hwnd) {
  RECT rect;
  GetClientRect(hwnd, &rect);
  double const dpr = FlutterDesktopGetDpiForHWND(hwnd) /
                     static_cast<double>(USER_DEFAULT_SCREEN_DPI);
  double const width = rect.right / dpr;
  double const height = rect.bottom / dpr;
  return {width, height};
}

std::wstring GetWindowTitle(HWND hwnd) {
  int length = GetWindowTextLengthW(hwnd);
  if (length == 0)
    return L"";

  std::vector<wchar_t> buffer(length + 1);
  GetWindowTextW(hwnd, buffer.data(), length + 1);
  return std::wstring(buffer.data());
}

class FlutterHostWindowControllerTest : public WindowsTest {
 public:
  FlutterHostWindowControllerTest() = default;
  virtual ~FlutterHostWindowControllerTest() = default;

 protected:
  void SetUp() override {
    InitializeCOM();
    SetDpiAwareness();
    FlutterWindowsEngineBuilder builder(GetContext());
    engine_ = builder.Build();
    engine_->Run();
    controller_ = std::make_unique<FlutterHostWindowController>(engine_.get());
  }

  FlutterWindowsEngine* engine() { return engine_.get(); }
  FlutterHostWindowController* host_window_controller() {
    return controller_.get();
  }

 private:
  void InitializeCOM() const {
    FML_CHECK(SUCCEEDED(::CoInitializeEx(nullptr, COINIT_MULTITHREADED)));
  }

  void SetDpiAwareness() const {
    HMODULE user32_module = LoadLibraryA("user32.dll");
    if (!user32_module) {
      return;
    }
    using SetProcessDpiAwarenessContext = BOOL WINAPI(DPI_AWARENESS_CONTEXT);
    auto set_process_dpi_awareness_context =
        reinterpret_cast<SetProcessDpiAwarenessContext*>(
            GetProcAddress(user32_module, "SetProcessDpiAwarenessContext"));
    if (set_process_dpi_awareness_context != nullptr) {
      set_process_dpi_awareness_context(
          DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2);
    }
    FreeLibrary(user32_module);
  }

  std::unique_ptr<FlutterWindowsEngine> engine_;
  std::unique_ptr<FlutterHostWindowController> controller_;

  FML_DISALLOW_COPY_AND_ASSIGN(FlutterHostWindowControllerTest);
};

}  // namespace

TEST_F(FlutterHostWindowControllerTest, CreateRegularWindow) {
  // Define parameters for the window to be created.
  WindowCreationSettings const settings = {
      .archetype = WindowArchetype::kRegular,
      .size = {800.0, 600.0},
      .title = "window",
  };

  // Create the window.
  std::optional<WindowMetadata> const result =
      host_window_controller()->CreateHostWindow(settings);

  // Validate the returned metadata.
  ASSERT_TRUE(result.has_value());
  EXPECT_NE(engine()->view(result->view_id), nullptr);
  EXPECT_EQ(result->archetype, settings.archetype);
  EXPECT_EQ(result->size.width(), settings.size.width());
  EXPECT_EQ(result->size.height(), settings.size.height());
  EXPECT_FALSE(result->parent_id.has_value());

  // Ensure the window was successfully retrieved.
  FlutterHostWindow* const window =
      host_window_controller()->GetHostWindow(result->view_id);
  ASSERT_NE(window, nullptr);
}

TEST_F(FlutterHostWindowControllerTest, ModifyRegularWindowSize) {
  // Define settings for the window to be created.
  WindowCreationSettings const creation_settings = {
      .archetype = WindowArchetype::kRegular,
      .size = {800.0, 600.0},
  };

  // Create the window.
  std::optional<WindowMetadata> const metadata =
      host_window_controller()->CreateHostWindow(creation_settings);
  ASSERT_TRUE(metadata.has_value());
  // Retrieve the created window and verify it exists.
  FlutterHostWindow* const window =
      host_window_controller()->GetHostWindow(metadata->view_id);
  ASSERT_NE(window, nullptr);

  // Define the modifications to be applied to the window.
  WindowModificationSettings const modification_settings = {
      .size = Size{200.0, 200.0},
  };

  // Test messenger with a handler for onWindowChanged.
  bool done = false;
  TestBinaryMessenger messenger([&](const std::string& channel,
                                    const uint8_t* message, size_t size,
                                    BinaryReply reply) {
    // Ensure the message is sent on the windowing channel.
    ASSERT_EQ(channel, kChannelName);

    // Ensure the decoded method call is valid.
    auto const method = StandardMethodCodec::GetInstance().DecodeMethodCall(
        std::vector<uint8_t>(message, message + size));
    ASSERT_NE(method, nullptr);

    // Handle the onWindowChanged method.
    if (method->method_name() == kOnWindowChangedMethod) {
      // Validate the method arguments.
      auto const& args = *method->arguments();
      ASSERT_TRUE(std::holds_alternative<EncodableMap>(args));
      auto const& args_map = std::get<EncodableMap>(args);

      // Ensure 'viewId' is present and valid.
      auto const& it_viewId = args_map.find(EncodableValue(kViewIdKey));
      ASSERT_NE(it_viewId, args_map.end());
      auto const* value_viewId = std::get_if<FlutterViewId>(&it_viewId->second);
      ASSERT_NE(value_viewId, nullptr);
      EXPECT_GE(*value_viewId, metadata->view_id);
      EXPECT_NE(engine()->view(*value_viewId), nullptr);

      // Ensure 'size' is present and valid.
      auto const& it_size = args_map.find(EncodableValue(kSizeKey));
      ASSERT_NE(it_size, args_map.end());
      auto const* value_size =
          std::get_if<std::vector<EncodableValue>>(&it_size->second);
      ASSERT_NE(value_size, nullptr);
      ASSERT_EQ(value_size->size(), 2);
      auto const* value_width = std::get_if<double>(&value_size->at(0));
      ASSERT_NE(value_width, nullptr);
      auto const* value_height = std::get_if<double>(&value_size->at(1));
      ASSERT_NE(value_height, nullptr);
      EXPECT_EQ(*value_width, modification_settings.size->width());
      EXPECT_EQ(*value_height, modification_settings.size->height());

      done = true;
    }
  });

  // Create the windowing handler with the test messenger.
  WindowingHandler windowing_handler(&messenger, host_window_controller());

  // Apply the modifications.
  EXPECT_TRUE(host_window_controller()->ModifyHostWindow(
      metadata->view_id, modification_settings));

  // Validate the modified settings.
  HWND const window_handle = host_window_controller()
                                 ->GetHostWindow(metadata->view_id)
                                 ->GetWindowHandle();
  Size const new_size = GetLogicalClientSize(window_handle);
  EXPECT_EQ(new_size.width(), modification_settings.size->width());
  EXPECT_EQ(new_size.height(), modification_settings.size->height());

  // Pump messages for the Windows platform task runner.
  while (!done) {
    PumpMessage();
  }
}

TEST_F(FlutterHostWindowControllerTest, ModifyRegularWindowTitle) {
  // Define settings for the window to be created.
  WindowCreationSettings const creation_settings = {
      .archetype = WindowArchetype::kRegular,
      .title = "window",
  };

  // Create the window.
  std::optional<WindowMetadata> const metadata =
      host_window_controller()->CreateHostWindow(creation_settings);
  ASSERT_TRUE(metadata.has_value());
  // Retrieve the created window and verify it exists.
  FlutterHostWindow* const window =
      host_window_controller()->GetHostWindow(metadata->view_id);
  ASSERT_NE(window, nullptr);

  // Define the modifications to be applied to the window.
  WindowModificationSettings const modification_settings = {
      .title = "new title 😉",
  };

  // Apply the modifications.
  EXPECT_TRUE(host_window_controller()->ModifyHostWindow(
      metadata->view_id, modification_settings));

  // Validate the modified settings.
  HWND const window_handle = host_window_controller()
                                 ->GetHostWindow(metadata->view_id)
                                 ->GetWindowHandle();
  std::wstring const new_title = GetWindowTitle(window_handle);
  EXPECT_STREQ(new_title.c_str(), L"new title 😉");
}

TEST_F(FlutterHostWindowControllerTest, ModifyRegularWindowState) {
  // Define settings for the window to be created.
  WindowCreationSettings const creation_settings = {
      .archetype = WindowArchetype::kRegular,
      .state = WindowState::kRestored,
  };

  // Create the window.
  std::optional<WindowMetadata> const metadata =
      host_window_controller()->CreateHostWindow(creation_settings);
  ASSERT_TRUE(metadata.has_value());
  // Retrieve the created window and verify it exists.
  FlutterHostWindow* const window =
      host_window_controller()->GetHostWindow(metadata->view_id);
  ASSERT_NE(window, nullptr);
  EXPECT_EQ(window->GetState(), creation_settings.state);

  // Define the modifications to be applied to the window.
  WindowModificationSettings const modification_settings = {
      .state = WindowState::kMinimized,
  };

  // Apply the modifications.
  EXPECT_TRUE(host_window_controller()->ModifyHostWindow(
      metadata->view_id, modification_settings));

  // Validate the modified settings.
  EXPECT_EQ(window->GetState(), modification_settings.state);
}

TEST_F(FlutterHostWindowControllerTest, DestroyWindow) {
  bool done = false;

  // Test messenger with a handler for onWindowDestroyed.
  TestBinaryMessenger messenger([&](const std::string& channel,
                                    const uint8_t* message, size_t size,
                                    BinaryReply reply) {
    // Ensure the message is sent on the windowing channel.
    ASSERT_EQ(channel, kChannelName);

    // Ensure the decoded method call is valid.
    auto const method = StandardMethodCodec::GetInstance().DecodeMethodCall(
        std::vector<uint8_t>(message, message + size));
    ASSERT_NE(method, nullptr);

    // Handle the onWindowDestroyed method.
    if (method->method_name() == kOnWindowDestroyedMethod) {
      // Validate the method arguments.
      auto const& args = *method->arguments();
      ASSERT_TRUE(std::holds_alternative<EncodableMap>(args));
      auto const& args_map = std::get<EncodableMap>(args);

      // Ensure the viewId is present but not valid anymore.
      auto const& it_viewId = args_map.find(EncodableValue(kViewIdKey));
      ASSERT_NE(it_viewId, args_map.end());
      auto const* value_viewId = std::get_if<FlutterViewId>(&it_viewId->second);
      ASSERT_NE(value_viewId, nullptr);
      EXPECT_GE(*value_viewId, 0);
      EXPECT_EQ(engine()->view(*value_viewId), nullptr);

      done = true;
    }
  });

  // Create the windowing handler with the test messenger.
  WindowingHandler windowing_handler(&messenger, host_window_controller());

  // Define parameters for the window to be created.
  WindowCreationSettings const settings = {
      .archetype = WindowArchetype::kRegular,
      .size = {800.0, 600.0},
      .title = "window",
  };

  // Create the window.
  std::optional<WindowMetadata> const result =
      host_window_controller()->CreateHostWindow(settings);
  ASSERT_TRUE(result.has_value());

  // Destroy the window.
  EXPECT_TRUE(host_window_controller()->DestroyHostWindow(result->view_id));

  // Pump messages for the Windows platform task runner.
  while (!done) {
    PumpMessage();
  }
}

}  // namespace testing
}  // namespace flutter

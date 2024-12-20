// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fuchsia/feedback/cpp/fidl.h>
#include <fuchsia/logger/cpp/fidl.h>
#include <fuchsia/sysmem/cpp/fidl.h>
#include <fuchsia/tracing/provider/cpp/fidl.h>
#include <fuchsia/ui/app/cpp/fidl.h>
#include <fuchsia/ui/display/singleton/cpp/fidl.h>
#include <fuchsia/ui/input/cpp/fidl.h>
#include <fuchsia/ui/test/input/cpp/fidl.h>
#include <fuchsia/ui/test/scene/cpp/fidl.h>
#include <lib/async-loop/testing/cpp/real_loop.h>
#include <lib/async/cpp/task.h>
#include <lib/fidl/cpp/binding_set.h>
#include <lib/sys/component/cpp/testing/realm_builder.h>
#include <lib/sys/component/cpp/testing/realm_builder_types.h>
#include <lib/zx/clock.h>
#include <lib/zx/time.h>
#include <zircon/status.h>
#include <zircon/types.h>
#include <zircon/utc.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/fuchsia/flutter/tests/integration/utils/check_view.h"
#include "flutter/shell/platform/fuchsia/flutter/tests/integration/utils/portable_ui_test.h"

namespace {

// Types imported for the realm_builder library.
using component_testing::ChildRef;
using component_testing::LocalComponentImpl;
using component_testing::ParentRef;
using component_testing::Protocol;
using component_testing::RealmBuilder;
using component_testing::RealmRoot;
using component_testing::Route;

using fuchsia_test_utils::CheckViewExistsInUpdates;
using fuchsia_test_utils::PortableUITest;

/// Max timeout in failure cases.
/// Set this as low as you can that still works across all test platforms.
constexpr zx::duration kTimeout = zx::min(5);

constexpr auto kKeyboardInputListener = "keyboard_input_listener";
constexpr auto kKeyboardInputListenerRef = ChildRef{kKeyboardInputListener};

constexpr auto kTextInputView = "text-input-view";
constexpr auto kTextInputViewRef = ChildRef{kTextInputView};
static constexpr auto kTextInputViewUrl =
    "fuchsia-pkg://fuchsia.com/text-input-view#meta/text-input-view.cm";

constexpr auto kTestUIStackUrl =
    "fuchsia-pkg://fuchsia.com/flatland-scene-manager-test-ui-stack#meta/"
    "test-ui-stack.cm";

/// |KeyboardInputListener| is a local test protocol that our test Flutter app
/// uses to let us know what text is being entered into its only text field.
///
/// The text field contents are reported on almost every change, so if you are
/// entering a long text, you will see calls corresponding to successive
/// additions of characters, not just the end result.
class KeyboardInputListenerServer
    : public fuchsia::ui::test::input::KeyboardInputListener,
      public LocalComponentImpl {
 public:
  explicit KeyboardInputListenerServer(async_dispatcher_t* dispatcher)
      : dispatcher_(dispatcher) {}

  // |fuchsia::ui::test::input::KeyboardInputListener|
  void ReportTextInput(
      fuchsia::ui::test::input::KeyboardInputListenerReportTextInputRequest
          request) override {
    FML_LOG(INFO) << "Flutter app sent: '" << request.text() << "'";
    response_list_.push_back(request.text());
  }

  /// Starts this server.
  void OnStart() override {
    FML_LOG(INFO) << "Starting KeyboardInputListenerServer";
    ASSERT_EQ(ZX_OK, outgoing()->AddPublicService(
                         bindings_.GetHandler(this, dispatcher_)));
  }

  /// Returns true if the response vector values matches `expected`
  bool HasResponse(const std::vector<std::string>& expected) {
    if (response_list_.size() != expected.size()) {
      return false;
    }

    // Iterate through the expected vector
    // Corresponding indices for response_list and expected should contain the
    // same values
    for (size_t i = 0; i < expected.size(); ++i) {
      if (response_list_[i] != expected[i]) {
        return false;
      }
    }

    return true;
  }

  // KeyboardInputListener override
  void ReportReady(ReportReadyCallback callback) override {
    FML_LOG(INFO) << "ReportReady callback ready";
    ready_ = true;
    callback();
  }

 private:
  async_dispatcher_t* dispatcher_ = nullptr;
  fidl::BindingSet<fuchsia::ui::test::input::KeyboardInputListener> bindings_;
  std::vector<std::string> response_list_;
  bool ready_ = false;
};

class TextInputTest : public PortableUITest,
                      public ::testing::Test,
                      public ::testing::WithParamInterface<std::string> {
 protected:
  void SetUp() override {
    PortableUITest::SetUp();

    // Post a "just in case" quit task, if the test hangs.
    async::PostDelayedTask(
        dispatcher(),
        [] {
          FML_LOG(FATAL)
              << "\n\n>> Test did not complete in time, terminating.  <<\n\n";
        },
        kTimeout);

    // Get the display dimensions.
    FML_LOG(INFO)
        << "Waiting for display info from fuchsia.ui.display.singleton.Info";
    fuchsia::ui::display::singleton::InfoPtr display_info =
        realm_root()
            ->component()
            .Connect<fuchsia::ui::display::singleton::Info>();
    display_info->GetMetrics(
        [this](fuchsia::ui::display::singleton::Metrics metrics) {
          display_width_ = metrics.extent_in_px().width;
          display_height_ = metrics.extent_in_px().height;
          FML_LOG(INFO) << "Got display_width = " << display_width_
                        << " and display_height = " << display_height_;
        });
    RunLoopUntil(
        [this] { return display_width_ != 0 && display_height_ != 0; });

    // Register input injection device.
    FML_LOG(INFO) << "Registering input injection device";
    RegisterKeyboard();
  }

  // Guaranteed to be initialized after SetUp().
  uint32_t display_width() const { return display_width_; }
  uint32_t display_height() const { return display_height_; }

  std::string GetTestUIStackUrl() override { return GetParam(); };

  KeyboardInputListenerServer* keyboard_input_listener_server_;

 private:
  void ExtendRealm() override {
    FML_LOG(INFO) << "Extending realm";
    // Key part of service setup: have this test component vend the
    // |KeyboardInputListener| service in the constructed realm.
    auto keyboard_input_listener_server =
        std::make_unique<KeyboardInputListenerServer>(dispatcher());
    keyboard_input_listener_server_ = keyboard_input_listener_server.get();
    realm_builder()->AddLocalChild(
        kKeyboardInputListener,
        [keyboard_input_listener_server =
             std::move(keyboard_input_listener_server)]() mutable {
          return std::move(keyboard_input_listener_server);
        });

    // Add text-input-view to the Realm
    realm_builder()->AddChild(kTextInputView, kTextInputViewUrl,
                              component_testing::ChildOptions{
                                  .environment = kFlutterRunnerEnvironment,
                              });

    // Route KeyboardInputListener to the runner and Flutter app
    realm_builder()->AddRoute(
        Route{.capabilities = {Protocol{
                  fuchsia::ui::test::input::KeyboardInputListener::Name_}},
              .source = kKeyboardInputListenerRef,
              .targets = {kFlutterJitRunnerRef, kTextInputViewRef}});

    // Expose fuchsia.ui.app.ViewProvider from the flutter app.
    realm_builder()->AddRoute(
        Route{.capabilities = {Protocol{fuchsia::ui::app::ViewProvider::Name_}},
              .source = kTextInputViewRef,
              .targets = {ParentRef()}});

    realm_builder()->AddRoute(Route{
        .capabilities =
            {Protocol{fuchsia::ui::input3::Keyboard::Name_},
             Protocol{"fuchsia.accessibility.semantics.SemanticsManager"}},
        .source = kTestUIStackRef,
        .targets = {ParentRef(), kFlutterJitRunnerRef}});
  }
};

INSTANTIATE_TEST_SUITE_P(TextInputTestParameterized,
                         TextInputTest,
                         ::testing::Values(kTestUIStackUrl));

TEST_P(TextInputTest, TextInput) {
  // Launch view
  FML_LOG(INFO) << "Initializing scene";
  LaunchClient();
  FML_LOG(INFO) << "Client launched";

  SimulateTextEntry("Hello\nworld!");
  std::vector<std::string> expected = {"LEFT_SHIFT", "H",    "E", "L", "L", "O",
                                       "ENTER",      "W",    "O", "R", "L", "D",
                                       "LEFT_SHIFT", "KEY_1"};

  RunLoopUntil(
      [&] { return keyboard_input_listener_server_->HasResponse(expected); });
}

}  // namespace

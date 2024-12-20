// Copyright 2022 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fuchsia/accessibility/semantics/cpp/fidl.h>
#include <fuchsia/buildinfo/cpp/fidl.h>
#include <fuchsia/component/cpp/fidl.h>
#include <fuchsia/fonts/cpp/fidl.h>
#include <fuchsia/input/report/cpp/fidl.h>
#include <fuchsia/kernel/cpp/fidl.h>
#include <fuchsia/logger/cpp/fidl.h>
#include <fuchsia/memorypressure/cpp/fidl.h>
#include <fuchsia/metrics/cpp/fidl.h>
#include <fuchsia/net/interfaces/cpp/fidl.h>
#include <fuchsia/tracing/provider/cpp/fidl.h>
#include <fuchsia/ui/app/cpp/fidl.h>
#include <fuchsia/ui/display/singleton/cpp/fidl.h>
#include <fuchsia/ui/input/cpp/fidl.h>
#include <fuchsia/ui/test/input/cpp/fidl.h>
#include <fuchsia/web/cpp/fidl.h>
#include <lib/async/cpp/task.h>
#include <lib/fidl/cpp/binding_set.h>
#include <lib/sys/component/cpp/testing/realm_builder.h>
#include <lib/sys/component/cpp/testing/realm_builder_types.h>
#include <zircon/status.h>
#include <zircon/types.h>
#include <zircon/utc.h>

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/fuchsia/flutter/tests/integration/utils/portable_ui_test.h"
#include "lib/fidl/cpp/interface_ptr.h"

namespace mouse_input_test::testing {
namespace {
// Types imported for the realm_builder library.
using component_testing::ChildRef;
using component_testing::ConfigValue;
using component_testing::LocalComponentImpl;
using component_testing::ParentRef;
using component_testing::Protocol;
using component_testing::Realm;
using component_testing::Route;

using fuchsia_test_utils::PortableUITest;
using RealmBuilder = component_testing::RealmBuilder;

// Alias for Component child name as provided to Realm Builder.
using ChildName = std::string;

// Alias for Component Legacy URL as provided to Realm Builder.
using LegacyUrl = std::string;

// Maximum pointer movement during a clickpad press for the gesture to
// be guaranteed to be interpreted as a click. For movement greater than
// this value, upper layers may, e.g., interpret the gesture as a drag.
//
// This value corresponds to the one used to instantiate the ClickDragHandler
// registered by Input Pipeline in Scene Manager.
constexpr int64_t kClickToDragThreshold = 16.0;

constexpr auto kMouseInputListener = "mouse_input_listener";
constexpr auto kMouseInputListenerRef = ChildRef{kMouseInputListener};
constexpr auto kMouseInputView = "mouse-input-view";
constexpr auto kMouseInputViewRef = ChildRef{kMouseInputView};
constexpr auto kMouseInputViewUrl =
    "fuchsia-pkg://fuchsia.com/mouse-input-view#meta/mouse-input-view.cm";

struct Position {
  double x = 0.0;
  double y = 0.0;
};

// Combines all vectors in `vecs` into one.
template <typename T>
std::vector<T> merge(std::initializer_list<std::vector<T>> vecs) {
  std::vector<T> result;
  for (auto v : vecs) {
    result.insert(result.end(), v.begin(), v.end());
  }
  return result;
}

int ButtonsToInt(
    const std::vector<fuchsia::ui::test::input::MouseButton>& buttons) {
  int result = 0;
  for (const auto& button : buttons) {
    result |= (0x1 >> button);
  }

  return result;
}

// `MouseInputListener` is a local test protocol that our test apps use to let
// us know what position and button press state the mouse cursor has.
class MouseInputListenerServer
    : public fuchsia::ui::test::input::MouseInputListener,
      public LocalComponentImpl {
 public:
  explicit MouseInputListenerServer(async_dispatcher_t* dispatcher)
      : dispatcher_(dispatcher) {}

  void ReportMouseInput(
      fuchsia::ui::test::input::MouseInputListenerReportMouseInputRequest
          request) override {
    FML_LOG(INFO) << "Received MouseInput event";
    events_.push(std::move(request));
  }

  // |MockComponent::OnStart|
  // When the component framework requests for this component to start, this
  // method will be invoked by the realm_builder library.
  void OnStart() override {
    FML_LOG(INFO) << "Starting MouseInputServer";
    ASSERT_EQ(ZX_OK, outgoing()->AddPublicService(
                         fidl::InterfaceRequestHandler<
                             fuchsia::ui::test::input::MouseInputListener>(
                             [this](auto request) {
                               bindings_.AddBinding(this, std::move(request),
                                                    dispatcher_);
                             })));
  }

  size_t SizeOfEvents() const { return events_.size(); }

  fuchsia::ui::test::input::MouseInputListenerReportMouseInputRequest
  PopEvent() {
    auto e = std::move(events_.front());
    events_.pop();
    return e;
  }

  const fuchsia::ui::test::input::MouseInputListenerReportMouseInputRequest&
  LastEvent() const {
    return events_.back();
  }

  void ClearEvents() { events_ = {}; }

 private:
  // Not owned.
  async_dispatcher_t* dispatcher_ = nullptr;
  fidl::BindingSet<fuchsia::ui::test::input::MouseInputListener> bindings_;
  std::queue<
      fuchsia::ui::test::input::MouseInputListenerReportMouseInputRequest>
      events_;
};

class MouseInputTest : public PortableUITest,
                       public ::testing::Test,
                       public ::testing::WithParamInterface<std::string> {
 protected:
  void SetUp() override {
    PortableUITest::SetUp();

    // Register fake mouse device.
    RegisterMouse();

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
  }

  void TearDown() override {
    // at the end of test, ensure event queue is empty.
    ASSERT_EQ(mouse_input_listener_->SizeOfEvents(), 0u);
  }

  MouseInputListenerServer* mouse_input_listener() {
    return mouse_input_listener_;
  }

  // Helper method for checking the test.mouse.MouseInputListener response from
  // the client app.
  void VerifyEvent(
      fuchsia::ui::test::input::MouseInputListenerReportMouseInputRequest&
          pointer_data,
      double expected_x,
      double expected_y,
      std::vector<fuchsia::ui::test::input::MouseButton> expected_buttons,
      const fuchsia::ui::test::input::MouseEventPhase expected_phase,
      const std::string& component_name) {
    FML_LOG(INFO) << "Client received mouse change at ("
                  << pointer_data.local_x() << ", " << pointer_data.local_y()
                  << ") with buttons " << ButtonsToInt(pointer_data.buttons())
                  << ".";
    FML_LOG(INFO) << "Expected mouse change is at approximately (" << expected_x
                  << ", " << expected_y << ") with buttons "
                  << ButtonsToInt(expected_buttons) << ".";

    // Allow for minor rounding differences in coordinates.
    // Note: These approximations don't account for
    // `PointerMotionDisplayScaleHandler` or `PointerMotionSensorScaleHandler`.
    // We will need to do so in order to validate larger motion or different
    // sized displays.
    EXPECT_NEAR(pointer_data.local_x(), expected_x, 1);
    EXPECT_NEAR(pointer_data.local_y(), expected_y, 1);
    EXPECT_EQ(pointer_data.buttons(), expected_buttons);
    EXPECT_EQ(pointer_data.phase(), expected_phase);
    EXPECT_EQ(pointer_data.component_name(), component_name);
  }

  void VerifyEventLocationOnTheRightOfExpectation(
      fuchsia::ui::test::input::MouseInputListenerReportMouseInputRequest&
          pointer_data,
      double expected_x_min,
      double expected_y,
      std::vector<fuchsia::ui::test::input::MouseButton> expected_buttons,
      const fuchsia::ui::test::input::MouseEventPhase expected_phase,
      const std::string& component_name) {
    FML_LOG(INFO) << "Client received mouse change at ("
                  << pointer_data.local_x() << ", " << pointer_data.local_y()
                  << ") with buttons " << ButtonsToInt(pointer_data.buttons())
                  << ".";
    FML_LOG(INFO) << "Expected mouse change is at approximately (>"
                  << expected_x_min << ", " << expected_y << ") with buttons "
                  << ButtonsToInt(expected_buttons) << ".";

    EXPECT_GT(pointer_data.local_x(), expected_x_min);
    EXPECT_NEAR(pointer_data.local_y(), expected_y, 1);
    EXPECT_EQ(pointer_data.buttons(), expected_buttons);
    EXPECT_EQ(pointer_data.phase(), expected_phase);
    EXPECT_EQ(pointer_data.component_name(), component_name);
  }

  // Guaranteed to be initialized after SetUp().
  uint32_t display_width() const { return display_width_; }
  uint32_t display_height() const { return display_height_; }

 private:
  void ExtendRealm() override {
    FML_LOG(INFO) << "Extending realm";

    // Key part of service setup: have this test component vend the
    // |MouseInputListener| service in the constructed realm.
    auto mouse_input_listener =
        std::make_unique<MouseInputListenerServer>(dispatcher());
    mouse_input_listener_ = mouse_input_listener.get();
    realm_builder()->AddLocalChild(
        kMouseInputListener,
        [mouse_input_listener = std::move(mouse_input_listener)]() mutable {
          return std::move(mouse_input_listener);
        });

    realm_builder()->AddChild(kMouseInputView, kMouseInputViewUrl,
                              component_testing::ChildOptions{
                                  .environment = kFlutterRunnerEnvironment,
                              });

    realm_builder()->AddRoute(
        Route{.capabilities = {Protocol{
                  fuchsia::ui::test::input::MouseInputListener::Name_}},
              .source = kMouseInputListenerRef,
              .targets = {kFlutterJitRunnerRef, kMouseInputViewRef}});

    realm_builder()->AddRoute(
        Route{.capabilities = {Protocol{fuchsia::ui::app::ViewProvider::Name_}},
              .source = kMouseInputViewRef,
              .targets = {ParentRef()}});
  }

  ParamType GetTestUIStackUrl() override { return GetParam(); };

  MouseInputListenerServer* mouse_input_listener_;

  uint32_t display_width_ = 0;
  uint32_t display_height_ = 0;
};

// Makes use of gtest's parameterized testing, allowing us
// to test different combinations of test-ui-stack + runners. Currently, there
// is just one combination. Documentation:
// http://go/gunitadvanced#value-parameterized-tests
INSTANTIATE_TEST_SUITE_P(
    MouseInputTestParameterized,
    MouseInputTest,
    ::testing::Values(
        "fuchsia-pkg://fuchsia.com/flatland-scene-manager-test-ui-stack#meta/"
        "test-ui-stack.cm"));

TEST_P(MouseInputTest, DISABLED_FlutterMouseMove) {
  LaunchClient();

  SimulateMouseEvent(/* pressed_buttons = */ {}, /* movement_x = */ 1,
                     /* movement_y = */ 2);
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 1; });

  ASSERT_EQ(mouse_input_listener()->SizeOfEvents(), 1u);

  auto e = mouse_input_listener()->PopEvent();

  // If the first mouse event is cursor movement, Flutter first sends an ADD
  // event with updated location.
  VerifyEvent(e,
              /*expected_x=*/static_cast<double>(display_width()) / 2.f + 1,
              /*expected_y=*/static_cast<double>(display_height()) / 2.f + 2,
              /*expected_buttons=*/{},
              /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::ADD,
              /*component_name=*/"mouse-input-view");
}

TEST_P(MouseInputTest, DISABLED_FlutterMouseDown) {
  LaunchClient();

  SimulateMouseEvent(
      /* pressed_buttons = */ {fuchsia::ui::test::input::MouseButton::FIRST},
      /* movement_x = */ 0, /* movement_y = */ 0);
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 3; });

  ASSERT_EQ(mouse_input_listener()->SizeOfEvents(), 3u);

  auto event_add = mouse_input_listener()->PopEvent();
  auto event_down = mouse_input_listener()->PopEvent();
  auto event_noop_move = mouse_input_listener()->PopEvent();

  // If the first mouse event is a button press, Flutter first sends an ADD
  // event with no buttons.
  VerifyEvent(event_add,
              /*expected_x=*/static_cast<double>(display_width()) / 2.f,
              /*expected_y=*/static_cast<double>(display_height()) / 2.f,
              /*expected_buttons=*/{},
              /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::ADD,
              /*component_name=*/"mouse-input-view");

  // Then Flutter sends a DOWN pointer event with the buttons we care about.
  VerifyEvent(
      event_down,
      /*expected_x=*/static_cast<double>(display_width()) / 2.f,
      /*expected_y=*/static_cast<double>(display_height()) / 2.f,
      /*expected_buttons=*/{fuchsia::ui::test::input::MouseButton::FIRST},
      /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::DOWN,
      /*component_name=*/"mouse-input-view");

  // Then Flutter sends a MOVE pointer event with no new information.
  VerifyEvent(
      event_noop_move,
      /*expected_x=*/static_cast<double>(display_width()) / 2.f,
      /*expected_y=*/static_cast<double>(display_height()) / 2.f,
      /*expected_buttons=*/{fuchsia::ui::test::input::MouseButton::FIRST},
      /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::MOVE,
      /*component_name=*/"mouse-input-view");
}

TEST_P(MouseInputTest, DISABLED_FlutterMouseDownUp) {
  LaunchClient();

  SimulateMouseEvent(
      /* pressed_buttons = */ {fuchsia::ui::test::input::MouseButton::FIRST},
      /* movement_x = */ 0, /* movement_y = */ 0);
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 3; });

  ASSERT_EQ(mouse_input_listener()->SizeOfEvents(), 3u);

  auto event_add = mouse_input_listener()->PopEvent();
  auto event_down = mouse_input_listener()->PopEvent();
  auto event_noop_move = mouse_input_listener()->PopEvent();

  // If the first mouse event is a button press, Flutter first sends an ADD
  // event with no buttons.
  VerifyEvent(event_add,
              /*expected_x=*/static_cast<double>(display_width()) / 2.f,
              /*expected_y=*/static_cast<double>(display_height()) / 2.f,
              /*expected_buttons=*/{},
              /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::ADD,
              /*component_name=*/"mouse-input-view");

  // Then Flutter sends a DOWN pointer event with the buttons we care about.
  VerifyEvent(
      event_down,
      /*expected_x=*/static_cast<double>(display_width()) / 2.f,
      /*expected_y=*/static_cast<double>(display_height()) / 2.f,
      /*expected_buttons=*/{fuchsia::ui::test::input::MouseButton::FIRST},
      /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::DOWN,
      /*component_name=*/"mouse-input-view");

  // Then Flutter sends a MOVE pointer event with no new information.
  VerifyEvent(
      event_noop_move,
      /*expected_x=*/static_cast<double>(display_width()) / 2.f,
      /*expected_y=*/static_cast<double>(display_height()) / 2.f,
      /*expected_buttons=*/{fuchsia::ui::test::input::MouseButton::FIRST},
      /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::MOVE,
      /*component_name=*/"mouse-input-view");

  SimulateMouseEvent(/* pressed_buttons = */ {}, /* movement_x = */ 0,
                     /* movement_y = */ 0);
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 1; });

  ASSERT_EQ(mouse_input_listener()->SizeOfEvents(), 1u);

  auto event_up = mouse_input_listener()->PopEvent();
  VerifyEvent(event_up,
              /*expected_x=*/static_cast<double>(display_width()) / 2.f,
              /*expected_y=*/static_cast<double>(display_height()) / 2.f,
              /*expected_buttons=*/{},
              /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::UP,
              /*component_name=*/"mouse-input-view");
}

TEST_P(MouseInputTest, DISABLED_FlutterMouseDownMoveUp) {
  LaunchClient();

  SimulateMouseEvent(
      /* pressed_buttons = */ {fuchsia::ui::test::input::MouseButton::FIRST},
      /* movement_x = */ 0, /* movement_y = */ 0);
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 3; });

  ASSERT_EQ(mouse_input_listener()->SizeOfEvents(), 3u);

  auto event_add = mouse_input_listener()->PopEvent();
  auto event_down = mouse_input_listener()->PopEvent();
  auto event_noop_move = mouse_input_listener()->PopEvent();

  // If the first mouse event is a button press, Flutter first sends an ADD
  // event with no buttons.
  VerifyEvent(event_add,
              /*expected_x=*/static_cast<double>(display_width()) / 2.f,
              /*expected_y=*/static_cast<double>(display_height()) / 2.f,
              /*expected_buttons=*/{},
              /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::ADD,
              /*component_name=*/"mouse-input-view");

  // Then Flutter sends a DOWN pointer event with the buttons we care about.
  VerifyEvent(
      event_down,
      /*expected_x=*/static_cast<double>(display_width()) / 2.f,
      /*expected_y=*/static_cast<double>(display_height()) / 2.f,
      /*expected_buttons=*/{fuchsia::ui::test::input::MouseButton::FIRST},
      /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::DOWN,
      /*component_name=*/"mouse-input-view");

  // Then Flutter sends a MOVE pointer event with no new information.
  VerifyEvent(
      event_noop_move,
      /*expected_x=*/static_cast<double>(display_width()) / 2.f,
      /*expected_y=*/static_cast<double>(display_height()) / 2.f,
      /*expected_buttons=*/{fuchsia::ui::test::input::MouseButton::FIRST},
      /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::MOVE,
      /*component_name=*/"mouse-input-view");

  SimulateMouseEvent(
      /* pressed_buttons = */ {fuchsia::ui::test::input::MouseButton::FIRST},
      /* movement_x = */ kClickToDragThreshold, /* movement_y = */ 0);
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 1; });

  ASSERT_EQ(mouse_input_listener()->SizeOfEvents(), 1u);

  auto event_move = mouse_input_listener()->PopEvent();

  VerifyEventLocationOnTheRightOfExpectation(
      event_move,
      /*expected_x_min=*/static_cast<double>(display_width()) / 2.f + 1,
      /*expected_y=*/static_cast<double>(display_height()) / 2.f,
      /*expected_buttons=*/{fuchsia::ui::test::input::MouseButton::FIRST},
      /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::MOVE,
      /*component_name=*/"mouse-input-view");

  SimulateMouseEvent(/* pressed_buttons = */ {}, /* movement_x = */ 0,
                     /* movement_y = */ 0);
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 1; });

  ASSERT_EQ(mouse_input_listener()->SizeOfEvents(), 1u);

  auto event_up = mouse_input_listener()->PopEvent();

  VerifyEventLocationOnTheRightOfExpectation(
      event_up,
      /*expected_x_min=*/static_cast<double>(display_width()) / 2.f + 1,
      /*expected_y=*/static_cast<double>(display_height()) / 2.f,
      /*expected_buttons=*/{},
      /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::UP,
      /*component_name=*/"mouse-input-view");
}

// TODO(fxbug.dev/103098): This test shows the issue when sending mouse wheel as
// the first event to Flutter.
// 1. expect Flutter app receive 2 events: ADD - Scroll, but got 3 events: Move
// - Scroll - Scroll.
// 2. the first event flutter app received has random value in buttons field
// Disabled until flutter rolls, since it changes the behavior of this issue.
TEST_P(MouseInputTest, DISABLED_FlutterMouseWheelIssue103098) {
  LaunchClient();

  SimulateMouseScroll(/* pressed_buttons = */ {}, /* scroll_x = */ 1,
                      /* scroll_y = */ 0);
  // Here we expected 2 events, ADD - Scroll, but got 3, Move - Scroll - Scroll.
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 3; });

  double initial_x = static_cast<double>(display_width()) / 2.f;
  double initial_y = static_cast<double>(display_height()) / 2.f;

  auto event_1 = mouse_input_listener()->PopEvent();
  EXPECT_NEAR(event_1.local_x(), initial_x, 1);
  EXPECT_NEAR(event_1.local_y(), initial_y, 1);
  // Flutter will scale the count of ticks to pixel.
  EXPECT_GT(event_1.wheel_x_physical_pixel(), 0);
  EXPECT_EQ(event_1.wheel_y_physical_pixel(), 0);
  EXPECT_EQ(event_1.phase(), fuchsia::ui::test::input::MouseEventPhase::MOVE);

  auto event_2 = mouse_input_listener()->PopEvent();
  VerifyEvent(
      event_2,
      /*expected_x=*/initial_x,
      /*expected_y=*/initial_y,
      /*expected_buttons=*/{},
      /*expected_phase=*/fuchsia::ui::test::input::MouseEventPhase::HOVER,
      /*component_name=*/"mouse-input-view");
  // Flutter will scale the count of ticks to pixel.
  EXPECT_GT(event_2.wheel_x_physical_pixel(), 0);
  EXPECT_EQ(event_2.wheel_y_physical_pixel(), 0);

  auto event_3 = mouse_input_listener()->PopEvent();
  VerifyEvent(
      event_3,
      /*expected_x=*/initial_x,
      /*expected_y=*/initial_y,
      /*expected_buttons=*/{},
      /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::HOVER,
      /*component_name=*/"mouse-input-view");
  // Flutter will scale the count of ticks to pixel.
  EXPECT_GT(event_3.wheel_x_physical_pixel(), 0);
  EXPECT_EQ(event_3.wheel_y_physical_pixel(), 0);
}

TEST_P(MouseInputTest, DISABLED_FlutterMouseWheel) {
  LaunchClient();

  double initial_x = static_cast<double>(display_width()) / 2.f + 1;
  double initial_y = static_cast<double>(display_height()) / 2.f + 2;

  // TODO(fxbug.dev/103098): Send a mouse move as the first event to workaround.
  SimulateMouseEvent(/* pressed_buttons = */ {},
                     /* movement_x = */ 1, /* movement_y = */ 2);
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 1; });

  auto event_add = mouse_input_listener()->PopEvent();
  VerifyEvent(event_add,
              /*expected_x=*/initial_x,
              /*expected_y=*/initial_y,
              /*expected_buttons=*/{},
              /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::ADD,
              /*component_name=*/"mouse-input-view");

  SimulateMouseScroll(/* pressed_buttons = */ {}, /* scroll_x = */ 1,
                      /* scroll_y = */ 0);
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 1; });

  auto event_wheel_h = mouse_input_listener()->PopEvent();

  VerifyEvent(
      event_wheel_h,
      /*expected_x=*/initial_x,
      /*expected_y=*/initial_y,
      /*expected_buttons=*/{},
      /*expected_phase=*/fuchsia::ui::test::input::MouseEventPhase::HOVER,
      /*component_name=*/"mouse-input-view");
  // Flutter will scale the count of ticks to pixel.
  EXPECT_GT(event_wheel_h.wheel_x_physical_pixel(), 0);
  EXPECT_EQ(event_wheel_h.wheel_y_physical_pixel(), 0);

  SimulateMouseScroll(/* pressed_buttons = */ {}, /* scroll_x = */ 0,
                      /* scroll_y = */ 1);
  RunLoopUntil(
      [this] { return this->mouse_input_listener()->SizeOfEvents() == 1; });

  auto event_wheel_v = mouse_input_listener()->PopEvent();

  VerifyEvent(
      event_wheel_v,
      /*expected_x=*/initial_x,
      /*expected_y=*/initial_y,
      /*expected_buttons=*/{},
      /*expected_type=*/fuchsia::ui::test::input::MouseEventPhase::HOVER,
      /*component_name=*/"mouse-input-view");
  // Flutter will scale the count of ticks to pixel.
  EXPECT_LT(event_wheel_v.wheel_y_physical_pixel(), 0);
  EXPECT_EQ(event_wheel_v.wheel_x_physical_pixel(), 0);
}

}  // namespace
}  // namespace mouse_input_test::testing

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fuchsia/accessibility/semantics/cpp/fidl.h>
#include <fuchsia/buildinfo/cpp/fidl.h>
#include <fuchsia/component/cpp/fidl.h>
#include <fuchsia/fonts/cpp/fidl.h>
#include <fuchsia/intl/cpp/fidl.h>
#include <fuchsia/kernel/cpp/fidl.h>
#include <fuchsia/memorypressure/cpp/fidl.h>
#include <fuchsia/metrics/cpp/fidl.h>
#include <fuchsia/net/interfaces/cpp/fidl.h>
#include <fuchsia/sysmem/cpp/fidl.h>
#include <fuchsia/tracing/provider/cpp/fidl.h>
#include <fuchsia/ui/app/cpp/fidl.h>
#include <fuchsia/ui/display/singleton/cpp/fidl.h>
#include <fuchsia/ui/input/cpp/fidl.h>
#include <fuchsia/ui/test/input/cpp/fidl.h>
#include <fuchsia/ui/test/scene/cpp/fidl.h>
#include <fuchsia/web/cpp/fidl.h>
#include <lib/async-loop/testing/cpp/real_loop.h>
#include <lib/async/cpp/task.h>
#include <lib/fidl/cpp/binding_set.h>
#include <lib/sys/component/cpp/testing/realm_builder.h>
#include <lib/sys/component/cpp/testing/realm_builder_types.h>
#include <lib/sys/cpp/component_context.h>
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
#include "flutter/shell/platform/fuchsia/flutter/tests/integration/utils/portable_ui_test.h"

// This test exercises the touch input dispatch path from Input Pipeline to a
// Scenic client. It is a multi-component test, and carefully avoids sleeping or
// polling for component coordination.
// - It runs real Scene Manager and Scenic components.
// - It uses a fake display controller; the physical device is unused.
//
// Components involved
// - This test program
// - Scene Manager
// - Scenic
// - Child view, a Scenic client
//
// Touch dispatch path
// - Test program's injection -> Input Pipeline -> Scenic -> Child view
//
// Setup sequence
// - The test sets up this view hierarchy:
//   - Top level scene, owned by Scene Manager.
//   - Child view, owned by the ui client.
// - The test waits for a Scenic event that verifies the child has UI content in
// the scene graph.
// - The test injects input into Input Pipeline, emulating a display's touch
// report.
// - Input Pipeline dispatches the touch event to Scenic, which in turn
// dispatches it to the child.
// - The child receives the touch event and reports back to the test over a
// custom test-only FIDL.
// - Test waits for the child to report a touch; when the test receives the
// report, the test quits
//   successfully.
//
// This test uses the realm_builder library to construct the topology of
// components and routes services between them. For v2 components, every test
// driver component sits as a child of test_manager in the topology. Thus, the
// topology of a test driver component such as this one looks like this:
//
//     test_manager
//         |
//   touch-input-test.cml (this component)
//
// With the usage of the realm_builder library, we construct a realm during
// runtime and then extend the topology to look like:
//
//    test_manager
//         |
//   touch-input-test.cml (this component)
//         |
//   <created realm root>
//      /      \
//   scenic  input-pipeline
//
// For more information about testing v2 components and realm_builder,
// visit the following links:
//
// Testing: https://fuchsia.dev/fuchsia-src/concepts/testing/v2
// Realm Builder:
// https://fuchsia.dev/fuchsia-src/development/components/v2/realm_builder

namespace touch_input_test::testing {
namespace {
// Types imported for the realm_builder library.
using component_testing::ChildRef;
using component_testing::ConfigValue;
using component_testing::DirectoryContents;
using component_testing::LocalComponentImpl;
using component_testing::ParentRef;
using component_testing::Protocol;
using component_testing::Realm;
using component_testing::RealmRoot;
using component_testing::Route;

using fuchsia_test_utils::PortableUITest;

using RealmBuilder = component_testing::RealmBuilder;

// Max timeout in failure cases.
// Set this as low as you can that still works across all test platforms.
constexpr zx::duration kTimeout = zx::min(1);

constexpr auto kTestUIStackUrl =
    "fuchsia-pkg://fuchsia.com/flatland-scene-manager-test-ui-stack#meta/"
    "test-ui-stack.cm";

constexpr auto kMockTouchInputListener = "touch_input_listener";
constexpr auto kMockTouchInputListenerRef = ChildRef{kMockTouchInputListener};

constexpr auto kTouchInputView = "touch-input-view";
constexpr auto kTouchInputViewRef = ChildRef{kTouchInputView};
constexpr auto kTouchInputViewUrl =
    "fuchsia-pkg://fuchsia.com/touch-input-view#meta/touch-input-view.cm";
constexpr auto kEmbeddingFlutterView = "embedding-flutter-view";
constexpr auto kEmbeddingFlutterViewRef = ChildRef{kEmbeddingFlutterView};
constexpr auto kEmbeddingFlutterViewUrl =
    "fuchsia-pkg://fuchsia.com/embedding-flutter-view#meta/"
    "embedding-flutter-view.cm";

bool CompareDouble(double f0, double f1, double epsilon) {
  return std::abs(f0 - f1) <= epsilon;
}

// This component implements the TouchInput protocol
// and the interface for a RealmBuilder LocalComponentImpl. A LocalComponentImpl
// is a component that is implemented here in the test, as opposed to
// elsewhere in the system. When it's inserted to the realm, it will act
// like a proper component. This is accomplished, in part, because the
// realm_builder library creates the necessary plumbing. It creates a manifest
// for the component and routes all capabilities to and from it.
// LocalComponentImpl:
// https://fuchsia.dev/fuchsia-src/development/testing/components/realm_builder#mock-components
class TouchInputListenerServer
    : public fuchsia::ui::test::input::TouchInputListener,
      public LocalComponentImpl {
 public:
  explicit TouchInputListenerServer(async_dispatcher_t* dispatcher)
      : dispatcher_(dispatcher) {}

  // |fuchsia::ui::test::input::TouchInputListener|
  void ReportTouchInput(
      fuchsia::ui::test::input::TouchInputListenerReportTouchInputRequest
          request) override {
    FML_LOG(INFO) << "Received ReportTouchInput event";
    events_received_.push_back(std::move(request));
  }

  // |LocalComponentImpl::OnStart|
  // When the component framework requests for this component to start, this
  // method will be invoked by the realm_builder library.
  void OnStart() override {
    FML_LOG(INFO) << "Starting TouchInputListenerServer";
    // When this component starts, add a binding to the
    // protocol to this component's outgoing directory.
    ASSERT_EQ(ZX_OK, outgoing()->AddPublicService(
                         fidl::InterfaceRequestHandler<
                             fuchsia::ui::test::input::TouchInputListener>(
                             [this](auto request) {
                               bindings_.AddBinding(this, std::move(request),
                                                    dispatcher_);
                             })));
  }

  const std::vector<
      fuchsia::ui::test::input::TouchInputListenerReportTouchInputRequest>&
  events_received() {
    return events_received_;
  }

 private:
  async_dispatcher_t* dispatcher_ = nullptr;
  fidl::BindingSet<fuchsia::ui::test::input::TouchInputListener> bindings_;
  std::vector<
      fuchsia::ui::test::input::TouchInputListenerReportTouchInputRequest>
      events_received_;
};

class FlutterTapTestBase : public PortableUITest, public ::testing::Test {
 protected:
  ~FlutterTapTestBase() override {
    FML_CHECK(touch_injection_request_count() > 0)
        << "Injection expected but didn't happen.";
  }

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

    // Get the display information using the
    // |fuchsia.ui.display.singleton.Info|.
    FML_LOG(INFO)
        << "Waiting for display info from fuchsia.ui.display.singleton.Info";
    std::optional<bool> display_metrics_obtained;
    fuchsia::ui::display::singleton::InfoPtr display_info =
        realm_root()
            ->component()
            .Connect<fuchsia::ui::display::singleton::Info>();
    display_info->GetMetrics([this, &display_metrics_obtained](auto info) {
      display_width_ = info.extent_in_px().width;
      display_height_ = info.extent_in_px().height;
      display_metrics_obtained = true;
    });
    RunLoopUntil([&display_metrics_obtained] {
      return display_metrics_obtained.has_value();
    });

    // Register input injection device.
    FML_LOG(INFO) << "Registering input injection device";
    RegisterTouchScreen();
  }

  bool LastEventReceivedMatches(float expected_x,
                                float expected_y,
                                std::string component_name) {
    const auto& events_received =
        touch_input_listener_server_->events_received();

    if (events_received.empty()) {
      return false;
    }

    const auto& last_event = events_received.back();

    auto pixel_scale = last_event.has_device_pixel_ratio()
                           ? last_event.device_pixel_ratio()
                           : 1;

    auto actual_x = pixel_scale * last_event.local_x();
    auto actual_y = pixel_scale * last_event.local_y();
    auto actual_component = last_event.component_name();

    bool last_event_matches =
        CompareDouble(actual_x, expected_x, pixel_scale) &&
        CompareDouble(actual_y, expected_y, pixel_scale) &&
        last_event.component_name() == component_name;

    if (last_event_matches) {
      FML_LOG(INFO) << "Received event for component " << component_name
                    << " at (" << expected_x << ", " << expected_y << ")";
    } else {
      FML_LOG(WARNING) << "Expecting event for component " << component_name
                       << " at (" << expected_x << ", " << expected_y << "). "
                       << "Instead received event for component "
                       << actual_component << " at (" << actual_x << ", "
                       << actual_y << "), accounting for pixel scale of "
                       << pixel_scale;
    }

    return last_event_matches;
  }

  // Guaranteed to be initialized after SetUp().
  uint32_t display_width() const { return display_width_; }
  uint32_t display_height() const { return display_height_; }

  std::string GetTestUIStackUrl() override { return kTestUIStackUrl; };

  TouchInputListenerServer* touch_input_listener_server_;
};

class FlutterTapTest : public FlutterTapTestBase {
 private:
  void ExtendRealm() override {
    FML_LOG(INFO) << "Extending realm";
    // Key part of service setup: have this test component vend the
    // |TouchInputListener| service in the constructed realm.
    auto touch_input_listener_server =
        std::make_unique<TouchInputListenerServer>(dispatcher());
    touch_input_listener_server_ = touch_input_listener_server.get();
    realm_builder()->AddLocalChild(
        kMockTouchInputListener, [touch_input_listener_server = std::move(
                                      touch_input_listener_server)]() mutable {
          return std::move(touch_input_listener_server);
        });

    // Add touch-input-view to the Realm
    realm_builder()->AddChild(kTouchInputView, kTouchInputViewUrl,
                              component_testing::ChildOptions{
                                  .environment = kFlutterRunnerEnvironment,
                              });

    // Route the TouchInput protocol capability to the Dart component
    realm_builder()->AddRoute(
        Route{.capabilities = {Protocol{
                  fuchsia::ui::test::input::TouchInputListener::Name_}},
              .source = kMockTouchInputListenerRef,
              .targets = {kFlutterJitRunnerRef, kTouchInputViewRef}});

    realm_builder()->AddRoute(
        Route{.capabilities = {Protocol{fuchsia::ui::app::ViewProvider::Name_}},
              .source = kTouchInputViewRef,
              .targets = {ParentRef()}});
  }
};

class FlutterEmbedTapTest : public FlutterTapTestBase {
 protected:
  void SetUp() override {
    PortableUITest::SetUp(false);

    // Post a "just in case" quit task, if the test hangs.
    async::PostDelayedTask(
        dispatcher(),
        [] {
          FML_LOG(FATAL)
              << "\n\n>> Test did not complete in time, terminating.  <<\n\n";
        },
        kTimeout);
  }

  void LaunchClientWithEmbeddedView() {
    BuildRealm();

    // Get the display information using the
    // |fuchsia.ui.display.singleton.Info|.
    FML_LOG(INFO)
        << "Waiting for display info from fuchsia.ui.display.singleton.Info";
    std::optional<bool> display_metrics_obtained;
    fuchsia::ui::display::singleton::InfoPtr display_info =
        realm_root()
            ->component()
            .Connect<fuchsia::ui::display::singleton::Info>();
    display_info->GetMetrics([this, &display_metrics_obtained](auto info) {
      display_width_ = info.extent_in_px().width;
      display_height_ = info.extent_in_px().height;
      display_metrics_obtained = true;
    });
    RunLoopUntil([&display_metrics_obtained] {
      return display_metrics_obtained.has_value();
    });

    // Register input injection device.
    FML_LOG(INFO) << "Registering input injection device";
    RegisterTouchScreen();

    PortableUITest::LaunchClientWithEmbeddedView();
  }

  // Helper method to add a component argument
  // This will be written into an args.csv file that can be parsed and read
  // by embedding-flutter-view.dart
  //
  // Note: You must call this method before LaunchClientWithEmbeddedView()
  // Realm Builder will not allow you to create a new directory / file in a
  // realm that's already been built
  void AddComponentArgument(std::string component_arg) {
    auto config_directory_contents = DirectoryContents();
    config_directory_contents.AddFile("args.csv", component_arg);
    realm_builder()->RouteReadOnlyDirectory(
        "config-data", {kEmbeddingFlutterViewRef},
        std::move(config_directory_contents));
  }

 private:
  void ExtendRealm() override {
    FML_LOG(INFO) << "Extending realm";
    // Key part of service setup: have this test component vend the
    // |TouchInputListener| service in the constructed realm.
    auto touch_input_listener_server =
        std::make_unique<TouchInputListenerServer>(dispatcher());
    touch_input_listener_server_ = touch_input_listener_server.get();
    realm_builder()->AddLocalChild(
        kMockTouchInputListener, [touch_input_listener_server = std::move(
                                      touch_input_listener_server)]() mutable {
          return std::move(touch_input_listener_server);
        });

    // Add touch-input-view to the Realm
    realm_builder()->AddChild(kTouchInputView, kTouchInputViewUrl,
                              component_testing::ChildOptions{
                                  .environment = kFlutterRunnerEnvironment,
                              });
    // Add embedding-flutter-view to the Realm
    // This component will embed touch-input-view as a child view
    realm_builder()->AddChild(kEmbeddingFlutterView, kEmbeddingFlutterViewUrl,
                              component_testing::ChildOptions{
                                  .environment = kFlutterRunnerEnvironment,
                              });

    // Route the TouchInput protocol capability to the Dart component
    realm_builder()->AddRoute(
        Route{.capabilities = {Protocol{
                  fuchsia::ui::test::input::TouchInputListener::Name_}},
              .source = kMockTouchInputListenerRef,
              .targets = {kFlutterJitRunnerRef, kTouchInputViewRef,
                          kEmbeddingFlutterViewRef}});

    realm_builder()->AddRoute(
        Route{.capabilities = {Protocol{fuchsia::ui::app::ViewProvider::Name_}},
              .source = kEmbeddingFlutterViewRef,
              .targets = {ParentRef()}});
    realm_builder()->AddRoute(
        Route{.capabilities = {Protocol{fuchsia::ui::app::ViewProvider::Name_}},
              .source = kTouchInputViewRef,
              .targets = {kEmbeddingFlutterViewRef}});
  }
};

TEST_F(FlutterTapTest, FlutterTap) {
  // Launch client view, and wait until it's rendering to proceed with the test.
  FML_LOG(INFO) << "Initializing scene";
  LaunchClient();
  FML_LOG(INFO) << "Client launched";

  // touch-input-view logical coordinate space doesn't match the fake touch
  // screen injector's coordinate space, which spans [-1000, 1000] on both axes.
  // Scenic handles figuring out where in the coordinate space
  //  to inject a touch event (this is fixed to a display's bounds).
  InjectTap(-500, -500);
  // For a (-500 [x], -500 [y]) tap, we expect a touch event in the middle of
  // the upper-left quadrant of the screen.
  RunLoopUntil([this] {
    return LastEventReceivedMatches(
        /*expected_x=*/static_cast<float>(display_width() / 4.0f),
        /*expected_y=*/static_cast<float>(display_height() / 4.0f),
        /*component_name=*/"touch-input-view");
  });

  // There should be 1 injected tap
  ASSERT_EQ(touch_injection_request_count(), 1);
}

TEST_F(FlutterEmbedTapTest, FlutterEmbedTap) {
  // Launch view
  FML_LOG(INFO) << "Initializing scene";
  LaunchClientWithEmbeddedView();
  FML_LOG(INFO) << "Client launched";

  {
    // Embedded child view takes up the center of the screen
    // Expect a response from the child view if we inject a tap there
    InjectTap(0, 0);
    RunLoopUntil([this] {
      return LastEventReceivedMatches(
          /*expected_x=*/static_cast<float>(display_width() / 8.0f),
          /*expected_y=*/static_cast<float>(display_height() / 8.0f),
          /*component_name=*/"touch-input-view");
    });
  }

  {
    // Parent view takes up the rest of the screen
    // Validate that parent can still receive taps
    InjectTap(500, 500);
    RunLoopUntil([this] {
      return LastEventReceivedMatches(
          /*expected_x=*/static_cast<float>(display_width() / (4.0f / 3.0f)),
          /*expected_y=*/static_cast<float>(display_height() / (4.0f / 3.0f)),
          /*component_name=*/"embedding-flutter-view");
    });
  }

  // There should be 2 injected taps
  ASSERT_EQ(touch_injection_request_count(), 2);
}

TEST_F(FlutterEmbedTapTest, FlutterEmbedOverlayEnabled) {
  FML_LOG(INFO) << "Initializing scene";
  AddComponentArgument("--showOverlay");
  LaunchClientWithEmbeddedView();
  FML_LOG(INFO) << "Client launched";

  {
    // The bottom-left corner of the overlay is at the center of the screen
    // Expect the overlay / parent view to respond if we inject a tap there
    // and not the embedded child view
    InjectTap(0, 0);
    RunLoopUntil([this] {
      return LastEventReceivedMatches(
          /*expected_x=*/static_cast<float>(display_width() / 2.0f),
          /*expected_y=*/static_cast<float>(display_height() / 2.0f),
          /*component_name=*/"embedding-flutter-view");
    });
  }

  {
    // The embedded child view is just outside of the bottom-left corner of the
    // overlay
    // Expect the embedded child view to still receive taps
    InjectTap(-1, -1);
    RunLoopUntil([this] {
      return LastEventReceivedMatches(
          /*expected_x=*/static_cast<float>(display_width() / 8.0f),
          /*expected_y=*/static_cast<float>(display_height() / 8.0f),
          /*component_name=*/"touch-input-view");
    });
  }

  // There should be 2 injected taps
  ASSERT_EQ(touch_injection_request_count(), 2);
}

}  // namespace
}  // namespace touch_input_test::testing

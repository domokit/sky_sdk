// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "portable_ui_test.h"

#include <fuchsia/inspect/cpp/fidl.h>
#include <fuchsia/logger/cpp/fidl.h>
#include <fuchsia/sysmem/cpp/fidl.h>
#include <fuchsia/sysmem2/cpp/fidl.h>
#include <fuchsia/tracing/provider/cpp/fidl.h>
#include <fuchsia/ui/app/cpp/fidl.h>
#include <lib/async/cpp/task.h>
#include <lib/sys/component/cpp/testing/realm_builder.h>
#include <lib/sys/component/cpp/testing/realm_builder_types.h>

#include "check_view.h"
#include "flutter/fml/logging.h"

namespace fuchsia_test_utils {
namespace {

// Types imported for the realm_builder library.
using component_testing::ChildOptions;
using component_testing::ChildRef;
using component_testing::ParentRef;
using component_testing::Protocol;
using component_testing::RealmRoot;
using component_testing::Route;

using fuchsia_test_utils::CheckViewExistsInSnapshot;

}  // namespace

void PortableUITest::SetUp(bool build_realm) {
  SetUpRealmBase();
  ExtendRealm();

  if (build_realm) {
    BuildRealm();
  }
}

void PortableUITest::BuildRealm() {
  realm_ = std::make_unique<RealmRoot>(realm_builder_.Build());
}

void PortableUITest::SetUpRealmBase() {
  FML_LOG(INFO) << "Setting up realm base";

  // Add Flutter JIT runner as a child of the RealmBuilder
  realm_builder_.AddChild(kFlutterJitRunner, kFlutterJitRunnerUrl);

  // Add environment providing the Flutter JIT runner
  fuchsia::component::decl::Environment flutter_runner_environment;
  flutter_runner_environment.set_name(kFlutterRunnerEnvironment);
  flutter_runner_environment.set_extends(
      fuchsia::component::decl::EnvironmentExtends::REALM);
  flutter_runner_environment.set_runners({});
  auto environment_runners = flutter_runner_environment.mutable_runners();

  // Add Flutter JIT runner to the environment
  fuchsia::component::decl::RunnerRegistration flutter_jit_runner_reg;
  flutter_jit_runner_reg.set_source(fuchsia::component::decl::Ref::WithChild(
      fuchsia::component::decl::ChildRef{.name = kFlutterJitRunner}));
  flutter_jit_runner_reg.set_source_name(kFlutterJitRunner);
  flutter_jit_runner_reg.set_target_name(kFlutterJitRunner);
  environment_runners->push_back(std::move(flutter_jit_runner_reg));
  auto realm_decl = realm_builder_.GetRealmDecl();
  if (!realm_decl.has_environments()) {
    realm_decl.set_environments({});
  }
  auto realm_environments = realm_decl.mutable_environments();
  realm_environments->push_back(std::move(flutter_runner_environment));
  realm_builder_.ReplaceRealmDecl(std::move(realm_decl));

  // Add test UI stack component.
  realm_builder_.AddChild(kTestUIStack, GetTestUIStackUrl());

  // // Route base system services to flutter and the test UI stack.
  realm_builder_.AddRoute(Route{
      .capabilities = {Protocol{fuchsia::logger::LogSink::Name_},
                       Protocol{fuchsia::inspect::InspectSink::Name_},
                       Protocol{fuchsia::sysmem::Allocator::Name_},
                       Protocol{fuchsia::sysmem2::Allocator::Name_},
                       Protocol{fuchsia::tracing::provider::Registry::Name_},
                       Protocol{fuchsia::ui::input::ImeService::Name_},
                       Protocol{kPosixSocketProviderName},
                       Protocol{kVulkanLoaderServiceName},
                       component_testing::Directory{"config-data"}},
      .source = ParentRef(),
      .targets = {kFlutterJitRunnerRef, kTestUIStackRef}});

  // Route UI capabilities to test driver and Flutter runner
  realm_builder_.AddRoute(Route{
      .capabilities = {Protocol{fuchsia::ui::composition::Allocator::Name_},
                       Protocol{fuchsia::ui::composition::Flatland::Name_},
                       Protocol{fuchsia::ui::test::input::Registry::Name_},
                       Protocol{fuchsia::ui::test::scene::Controller::Name_},
                       Protocol{fuchsia::ui::display::singleton::Info::Name_},
                       Protocol{kPointerInjectorRegistryName}},
      .source = kTestUIStackRef,
      .targets = {ParentRef(), kFlutterJitRunnerRef}});
}

void PortableUITest::ProcessViewGeometryResponse(
    fuchsia::ui::observation::geometry::WatchResponse response) {
  // Process update if no error
  if (!response.has_error()) {
    std::vector<fuchsia::ui::observation::geometry::ViewTreeSnapshot>* updates =
        response.mutable_updates();
    if (updates && !updates->empty()) {
      last_view_tree_snapshot_ = std::move(updates->back());
    }
  } else {
    // Otherwise process error
    const auto& error = response.error();
    if (error | fuchsia::ui::observation::geometry::Error::CHANNEL_OVERFLOW) {
      FML_LOG(INFO) << "View Tree watcher channel overflowed";
    } else if (error |
               fuchsia::ui::observation::geometry::Error::BUFFER_OVERFLOW) {
      FML_LOG(INFO) << "View Tree watcher buffer overflowed";
    } else if (error |
               fuchsia::ui::observation::geometry::Error::VIEWS_OVERFLOW) {
      // This one indicates some possible data loss, so we log with a high
      // severity
      FML_LOG(WARNING)
          << "View Tree watcher attempted to report too many views";
    }
  }
}

void PortableUITest::WatchViewGeometry() {
  FML_CHECK(view_tree_watcher_)
      << "View Tree watcher must be registered before calling Watch()";

  view_tree_watcher_->Watch([this](auto response) {
    ProcessViewGeometryResponse(std::move(response));
    WatchViewGeometry();
  });
}

bool PortableUITest::HasViewConnected(zx_koid_t view_ref_koid) {
  return last_view_tree_snapshot_.has_value() &&
         CheckViewExistsInSnapshot(*last_view_tree_snapshot_, view_ref_koid);
}

void PortableUITest::LaunchClient() {
  scene_provider_ =
      realm_->component().Connect<fuchsia::ui::test::scene::Controller>();
  scene_provider_.set_error_handler([](auto) {
    FML_LOG(ERROR) << "Error from test scene provider: "
                   << &zx_status_get_string;
  });

  fuchsia::ui::test::scene::ControllerAttachClientViewRequest request;
  request.set_view_provider(
      realm_->component().Connect<fuchsia::ui::app::ViewProvider>());
  scene_provider_->RegisterViewTreeWatcher(view_tree_watcher_.NewRequest(),
                                           []() {});
  scene_provider_->AttachClientView(
      std::move(request), [this](auto client_view_ref_koid) {
        client_root_view_ref_koid_ = client_view_ref_koid;
      });

  FML_LOG(INFO) << "Waiting for client view ref koid";
  RunLoopUntil([this] { return client_root_view_ref_koid_.has_value(); });

  WatchViewGeometry();

  FML_LOG(INFO) << "Waiting for client view to connect";
  // Wait for the client view to get attached to the view tree.
  RunLoopUntil(
      [this] { return HasViewConnected(*client_root_view_ref_koid_); });
  FML_LOG(INFO) << "Client view has rendered";
}

void PortableUITest::LaunchClientWithEmbeddedView() {
  LaunchClient();
  // At this point, the parent view must have rendered, so we just need to wait
  // for the embedded view.
  RunLoopUntil([this] {
    if (!last_view_tree_snapshot_.has_value() ||
        !last_view_tree_snapshot_->has_views()) {
      return false;
    }

    if (!client_root_view_ref_koid_.has_value()) {
      return false;
    }

    for (const auto& view : last_view_tree_snapshot_->views()) {
      if (!view.has_view_ref_koid() ||
          view.view_ref_koid() != *client_root_view_ref_koid_) {
        continue;
      }

      if (view.children().empty()) {
        return false;
      }

      // NOTE: We can't rely on the presence of the child view in
      // `view.children()` to guarantee that it has rendered. The child view
      // also needs to be present in `last_view_tree_snapshot_->views`.
      return std::count_if(
                 last_view_tree_snapshot_->views().begin(),
                 last_view_tree_snapshot_->views().end(),
                 [view_to_find =
                      view.children().back()](const auto& view_to_check) {
                   return view_to_check.has_view_ref_koid() &&
                          view_to_check.view_ref_koid() == view_to_find;
                 }) > 0;
    }

    return false;
  });

  FML_LOG(INFO) << "Embedded view has rendered";
}

void PortableUITest::RegisterTouchScreen() {
  FML_LOG(INFO) << "Registering fake touch screen";
  input_registry_ =
      realm_->component().Connect<fuchsia::ui::test::input::Registry>();
  input_registry_.set_error_handler([](auto) {
    FML_LOG(ERROR) << "Error from input helper: " << &zx_status_get_string;
  });

  bool touchscreen_registered = false;
  fuchsia::ui::test::input::RegistryRegisterTouchScreenRequest request;
  request.set_device(fake_touchscreen_.NewRequest());
  input_registry_->RegisterTouchScreen(
      std::move(request),
      [&touchscreen_registered]() { touchscreen_registered = true; });

  RunLoopUntil([&touchscreen_registered] { return touchscreen_registered; });
  FML_LOG(INFO) << "Touchscreen registered";
}

void PortableUITest::RegisterMouse() {
  FML_LOG(INFO) << "Registering fake mouse";
  input_registry_ =
      realm_->component().Connect<fuchsia::ui::test::input::Registry>();
  input_registry_.set_error_handler([](auto) {
    FML_LOG(ERROR) << "Error from input helper: " << &zx_status_get_string;
  });

  bool mouse_registered = false;
  fuchsia::ui::test::input::RegistryRegisterMouseRequest request;
  request.set_device(fake_mouse_.NewRequest());
  input_registry_->RegisterMouse(
      std::move(request), [&mouse_registered]() { mouse_registered = true; });

  RunLoopUntil([&mouse_registered] { return mouse_registered; });
  FML_LOG(INFO) << "Mouse registered";
}

void PortableUITest::RegisterKeyboard() {
  FML_LOG(INFO) << "Registering fake keyboard";
  input_registry_ =
      realm_->component().Connect<fuchsia::ui::test::input::Registry>();
  input_registry_.set_error_handler([](auto) {
    FML_LOG(ERROR) << "Error from input helper: " << &zx_status_get_string;
  });

  bool keyboard_registered = false;
  fuchsia::ui::test::input::RegistryRegisterKeyboardRequest request;
  request.set_device(fake_keyboard_.NewRequest());
  input_registry_->RegisterKeyboard(
      std::move(request),
      [&keyboard_registered]() { keyboard_registered = true; });

  RunLoopUntil([&keyboard_registered] { return keyboard_registered; });
  FML_LOG(INFO) << "Keyboard registered";
}

void PortableUITest::InjectTap(int32_t x, int32_t y) {
  fuchsia::ui::test::input::TouchScreenSimulateTapRequest tap_request;
  tap_request.mutable_tap_location()->x = x;
  tap_request.mutable_tap_location()->y = y;

  FML_LOG(INFO) << "Injecting tap at (" << tap_request.tap_location().x << ", "
                << tap_request.tap_location().y << ")";
  fake_touchscreen_->SimulateTap(std::move(tap_request), [this]() {
    ++touch_injection_request_count_;
    FML_LOG(INFO) << "*** Tap injected, count: "
                  << touch_injection_request_count_;
  });
}

void PortableUITest::SimulateMouseEvent(
    std::vector<fuchsia::ui::test::input::MouseButton> pressed_buttons,
    int movement_x,
    int movement_y) {
  fuchsia::ui::test::input::MouseSimulateMouseEventRequest request;
  request.set_pressed_buttons(std::move(pressed_buttons));
  request.set_movement_x(movement_x);
  request.set_movement_y(movement_y);

  FML_LOG(INFO) << "Injecting mouse input";

  fake_mouse_->SimulateMouseEvent(
      std::move(request), [] { FML_LOG(INFO) << "Mouse event injected"; });
}

void PortableUITest::SimulateMouseScroll(
    std::vector<fuchsia::ui::test::input::MouseButton> pressed_buttons,
    int scroll_x,
    int scroll_y,
    bool use_physical_units) {
  FML_LOG(INFO) << "Requesting mouse scroll";
  fuchsia::ui::test::input::MouseSimulateMouseEventRequest request;
  request.set_pressed_buttons(std::move(pressed_buttons));
  if (use_physical_units) {
    request.set_scroll_h_physical_pixel(scroll_x);
    request.set_scroll_v_physical_pixel(scroll_y);
  } else {
    request.set_scroll_h_detent(scroll_x);
    request.set_scroll_v_detent(scroll_y);
  }

  fake_mouse_->SimulateMouseEvent(std::move(request), [] {
    FML_LOG(INFO) << "Mouse scroll event injected";
  });
}

void PortableUITest::SimulateTextEntry(std::string text) {
  FML_LOG(INFO) << "Sending text request";
  bool done = false;

  fuchsia::ui::test::input::KeyboardSimulateUsAsciiTextEntryRequest request;
  request.set_text(text);
  fake_keyboard_->SimulateUsAsciiTextEntry(std::move(request),
                                           [&done]() { done = true; });

  RunLoopUntil([&] { return done; });
  FML_LOG(INFO) << "Text request sent";
}

}  // namespace fuchsia_test_utils

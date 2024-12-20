// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_INTEGRATION_UTILS_PORTABLE_UI_TEST_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_INTEGRATION_UTILS_PORTABLE_UI_TEST_H_

#include <fuchsia/sysmem2/cpp/fidl.h>
#include <fuchsia/ui/app/cpp/fidl.h>
#include <fuchsia/ui/composition/cpp/fidl.h>
#include <fuchsia/ui/display/singleton/cpp/fidl.h>
#include <fuchsia/ui/input/cpp/fidl.h>
#include <fuchsia/ui/test/input/cpp/fidl.h>
#include <fuchsia/ui/test/scene/cpp/fidl.h>
#include <lib/async-loop/testing/cpp/real_loop.h>
#include <lib/sys/component/cpp/testing/realm_builder.h>
#include <lib/sys/component/cpp/testing/realm_builder_types.h>
#include <zircon/status.h>

#include <optional>
#include <vector>

namespace fuchsia_test_utils {
class PortableUITest : public ::loop_fixture::RealLoop {
 public:
  // The FIDL bindings for these services are not exposed in the Fuchsia SDK so
  // we must encode the names manually here.
  static constexpr auto kVulkanLoaderServiceName =
      "fuchsia.vulkan.loader.Loader";
  static constexpr auto kPosixSocketProviderName =
      "fuchsia.posix.socket.Provider";
  static constexpr auto kPointerInjectorRegistryName =
      "fuchsia.ui.pointerinjector.Registry";

  // The naming and references used by Realm Builder
  static constexpr auto kTestUIStack = "ui";
  static constexpr auto kTestUIStackRef =
      component_testing::ChildRef{kTestUIStack};
  static constexpr auto kFlutterJitRunner = "flutter_jit_runner";
  static constexpr auto kFlutterJitRunnerRef =
      component_testing::ChildRef{kFlutterJitRunner};
  static constexpr auto kFlutterJitRunnerUrl =
      "fuchsia-pkg://fuchsia.com/oot_flutter_jit_runner#meta/"
      "flutter_jit_runner.cm";
  static constexpr auto kFlutterRunnerEnvironment = "flutter_runner_env";

  void SetUp(bool build_realm = true);

  // Calls the Build method for Realm Builder to build the realm
  // Can only be called once, panics otherwise
  void BuildRealm();

  // Attaches a client view to the scene, and waits for it to render.
  void LaunchClient();
  // Attaches a view with an embedded child view to the scene, and waits for it
  // to render.
  void LaunchClientWithEmbeddedView();

  // Returns true when the specified view is fully connected to the scene AND
  // has presented at least one frame of content.
  bool HasViewConnected(zx_koid_t view_ref_koid);

  // Registers a fake touch screen device with an injection coordinate space
  // spanning [-1000, 1000] on both axes.
  void RegisterTouchScreen();

  // Registers a fake mouse device, for which mouse movement is measured on a
  // scale of [-1000, 1000] on both axes and scroll is measured from [-100, 100]
  // on both axes.
  void RegisterMouse();

  // Register a fake keyboard
  void RegisterKeyboard();

  // Simulates a tap at location (x, y).
  void InjectTap(int32_t x, int32_t y);

  // Helper method to simulate combinations of button presses/releases and/or
  // mouse movements.
  void SimulateMouseEvent(
      std::vector<fuchsia::ui::test::input::MouseButton> pressed_buttons,
      int movement_x,
      int movement_y);

  // Helper method to simulate a mouse scroll event.
  //
  // Set `use_physical_units` to true to specify scroll in physical pixels and
  // false to specify scroll in detents.
  void SimulateMouseScroll(
      std::vector<fuchsia::ui::test::input::MouseButton> pressed_buttons,
      int scroll_x,
      int scroll_y,
      bool use_physical_units = false);

  // Helper method to simluate text input
  void SimulateTextEntry(std::string text);

 protected:
  component_testing::RealmBuilder* realm_builder() { return &realm_builder_; }
  component_testing::RealmRoot* realm_root() { return realm_.get(); }

  uint32_t display_width_ = 0;
  uint32_t display_height_ = 0;

  int touch_injection_request_count() const {
    return touch_injection_request_count_;
  }

 private:
  void SetUpRealmBase();

  // Configures the test-specific component topology.
  virtual void ExtendRealm() = 0;

  // Returns the test-specific test-ui-stack component url to use.
  // Usually overridden to return a value from gtest GetParam()
  virtual std::string GetTestUIStackUrl() = 0;

  // Helper method to watch for view geometry updates.
  void WatchViewGeometry();

  // Helper method to process a view geometry update.
  void ProcessViewGeometryResponse(
      fuchsia::ui::observation::geometry::WatchResponse response);

  fuchsia::ui::test::input::RegistryPtr input_registry_;
  fuchsia::ui::test::input::TouchScreenPtr fake_touchscreen_;
  fuchsia::ui::test::input::MousePtr fake_mouse_;
  fuchsia::ui::test::input::KeyboardPtr fake_keyboard_;
  fuchsia::ui::test::scene::ControllerPtr scene_provider_;
  fuchsia::ui::observation::geometry::ViewTreeWatcherPtr view_tree_watcher_;

  component_testing::RealmBuilder realm_builder_ =
      component_testing::RealmBuilder::Create();
  std::unique_ptr<component_testing::RealmRoot> realm_;

  // Counts the number of completed requests to inject touch reports into input
  // pipeline.
  int touch_injection_request_count_ = 0;

  // The KOID of the client root view's `ViewRef`.
  std::optional<zx_koid_t> client_root_view_ref_koid_;

  // Holds the most recent view tree snapshot received from the view tree
  // watcher.
  //
  // From this snapshot, we can retrieve relevant view tree state on demand,
  // e.g. if the client view is rendering content.
  std::optional<fuchsia::ui::observation::geometry::ViewTreeSnapshot>
      last_view_tree_snapshot_;
};

}  // namespace fuchsia_test_utils

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_INTEGRATION_UTILS_PORTABLE_UI_TEST_H_

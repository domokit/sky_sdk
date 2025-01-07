// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/runtime_controller.h"
#include "flutter/runtime/runtime_delegate.h"

#include "flutter/lib/ui/semantics/semantics_update.h"
#include "flutter/shell/common/shell_test.h"
#include "flutter/testing/testing.h"

namespace flutter::testing {

fml::AutoResetWaitableEvent message_latch;
using RuntimeControllerTest = ShellTest;

class MockRuntimeDelegate : public RuntimeDelegate {
 public:
  FontCollection font;
  std::vector<SemanticsNodeUpdates> updates;
  std::vector<CustomAccessibilityActionUpdates> actions;
  std::string DefaultRouteName() override { return ""; }

  void ScheduleFrame(bool regenerate_layer_trees = true) override {}

  void OnAllViewsRendered() override {}

  void Render(int64_t view_id,
              std::unique_ptr<flutter::LayerTree> layer_tree,
              float device_pixel_ratio) override {}

  void UpdateSemantics(SemanticsNodeUpdates update,
                       CustomAccessibilityActionUpdates actions) override {
    this->updates.push_back(update);
    this->actions.push_back(actions);
  }

  void SetSemanticsTreeEnabled(bool enabled) override {}

  void HandlePlatformMessage(
      std::unique_ptr<PlatformMessage> message) override {}

  FontCollection& GetFontCollection() override { return font; }

  std::shared_ptr<AssetManager> GetAssetManager() override { return nullptr; }

  void OnRootIsolateCreated() override {};

  void UpdateIsolateDescription(const std::string isolate_name,
                                int64_t isolate_port) override {};

  void SetNeedsReportTimings(bool value) override {};

  std::unique_ptr<std::vector<std::string>> ComputePlatformResolvedLocale(
      const std::vector<std::string>& supported_locale_data) override {
    return nullptr;
  }

  void RequestDartDeferredLibrary(intptr_t loading_unit_id) override {}

  std::weak_ptr<PlatformMessageHandler> GetPlatformMessageHandler()
      const override {
    return {};
  }

  void SendChannelUpdate(std::string name, bool listening) override {}

  double GetScaledFontSize(double unscaled_font_size,
                           int configuration_id) const override {
    return 0.0;
  }
};

class RuntimeControllerTester {
 public:
  explicit RuntimeControllerTester(UIDartState::Context& context)
      : context_(context),
        runtime_controller_(delegate_,
                            nullptr,
                            {},
                            {},
                            {},
                            {},
                            {},
                            nullptr,
                            context_) {}

  void CanUpdateSemanticsWhenSetSemanticsTreeEnabled(SemanticsUpdate* update) {
    ASSERT_TRUE(delegate_.updates.empty());
    ASSERT_TRUE(delegate_.actions.empty());
    runtime_controller_.UpdateSemantics(update);
    // Semantics tree is not yet enabled.
    ASSERT_TRUE(delegate_.updates.empty());
    ASSERT_TRUE(delegate_.actions.empty());

    runtime_controller_.SetSemanticsTreeEnabled(true);
    runtime_controller_.UpdateSemantics(update);
    ASSERT_FALSE(delegate_.updates.empty());
    ASSERT_FALSE(delegate_.actions.empty());
  }

 private:
  MockRuntimeDelegate delegate_;
  UIDartState::Context& context_;
  RuntimeController runtime_controller_;
};

TEST_F(RuntimeControllerTest, CanUpdateSemanticsWhenSetSemanticsTreeEnabled) {
  // The code in this test is mostly setup code to get a SemanticsUpdate object.
  // The real test is in RuntimeControllerTester::CanUpdateSemantics.
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );
  UIDartState::Context context(task_runners);
  std::shared_ptr<RuntimeControllerTester> tester =
      std::make_shared<RuntimeControllerTester>(context);

  auto native_semantics_update = [tester](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    intptr_t peer = 0;
    Dart_Handle result = Dart_GetNativeInstanceField(
        handle, tonic::DartWrappable::kPeerIndex, &peer);
    ASSERT_FALSE(Dart_IsError(result));
    SemanticsUpdate* update = reinterpret_cast<SemanticsUpdate*>(peer);

    tester->CanUpdateSemanticsWhenSetSemanticsTreeEnabled(update);
    message_latch.Signal();
  };

  Settings settings = CreateSettingsForFixture();
  AddNativeCallback("SemanticsUpdate",
                    CREATE_NATIVE_ENTRY(native_semantics_update));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("sendSemanticsUpdate");

  shell->RunEngine(std::move(configuration), [](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch.Wait();
  DestroyShell(std::move(shell), task_runners);
}

}  // namespace flutter::testing

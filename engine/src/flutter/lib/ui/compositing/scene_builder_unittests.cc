// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/compositing/scene_builder.h"

#include <memory>

#include "flutter/common/task_runners.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/common/shell_test.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/testing/testing.h"
#include "lib/ui/compositing/scene.h"

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace flutter {
namespace testing {

TEST_F(ShellTest, SceneBuilderBuildAndSceneDisposeReleasesLayerStack) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();

  // prevent ClearDartWrapper() from deleting the scene builder.
  SceneBuilder* retained_scene_builder;
  Scene* retained_scene;

  auto validate_builder_has_layers =
      [&retained_scene_builder](Dart_NativeArguments args) {
        auto handle = Dart_GetNativeArgument(args, 0);
        intptr_t peer = 0;
        Dart_Handle result = Dart_GetNativeInstanceField(
            handle, tonic::DartWrappable::kPeerIndex, &peer);
        ASSERT_FALSE(Dart_IsError(result));
        retained_scene_builder = reinterpret_cast<SceneBuilder*>(peer);
        retained_scene_builder->AddRef();
        ASSERT_TRUE(retained_scene_builder);
        ASSERT_EQ(retained_scene_builder->layer_stack().size(), 2ul);
      };

  auto validate_builder_has_no_layers =
      [&retained_scene_builder](Dart_NativeArguments args) {
        ASSERT_EQ(retained_scene_builder->layer_stack().size(), 0ul);
        retained_scene_builder->Release();
        retained_scene_builder = nullptr;
      };

  auto capture_scene = [&retained_scene](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    intptr_t peer = 0;
    Dart_Handle result = Dart_GetNativeInstanceField(
        handle, tonic::DartWrappable::kPeerIndex, &peer);
    ASSERT_FALSE(Dart_IsError(result));
    retained_scene = reinterpret_cast<Scene*>(peer);
    retained_scene->AddRef();
    ASSERT_TRUE(retained_scene);
  };

  auto validate_scene_has_no_layers =
      [message_latch, &retained_scene](Dart_NativeArguments args) {
        EXPECT_FALSE(retained_scene->takeLayerTree(100, 100));
        retained_scene->Release();
        retained_scene = nullptr;
        message_latch->Signal();
      };

  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  AddNativeCallback("ValidateBuilderHasLayers",
                    CREATE_NATIVE_ENTRY(validate_builder_has_layers));
  AddNativeCallback("ValidateBuilderHasNoLayers",
                    CREATE_NATIVE_ENTRY(validate_builder_has_no_layers));

  AddNativeCallback("CaptureScene", CREATE_NATIVE_ENTRY(capture_scene));
  AddNativeCallback("ValidateSceneHasNoLayers",
                    CREATE_NATIVE_ENTRY(validate_scene_has_no_layers));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("validateSceneBuilderAndScene");

  shell->RunEngine(std::move(configuration), [](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(ShellTest, EngineLayerDisposeReleasesReference) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();

  std::shared_ptr<ContainerLayer> root_layer;

  auto capture_root_layer = [&root_layer](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    intptr_t peer = 0;
    Dart_Handle result = Dart_GetNativeInstanceField(
        handle, tonic::DartWrappable::kPeerIndex, &peer);
    ASSERT_FALSE(Dart_IsError(result));
    SceneBuilder* scene_builder = reinterpret_cast<SceneBuilder*>(peer);
    ASSERT_TRUE(scene_builder);
    root_layer = scene_builder->layer_stack()[0];
    ASSERT_TRUE(root_layer);
  };

  auto validate_layer_tree_counts = [&root_layer](Dart_NativeArguments args) {
    // One for the EngineLayer, one for our pointer in the test.
    EXPECT_EQ(root_layer->layers().front().use_count(), 2);
  };

  auto validate_engine_layer_dispose =
      [message_latch, &root_layer](Dart_NativeArguments args) {
        // Just one for our pointer now.
        EXPECT_EQ(root_layer->layers().front().use_count(), 1);
        root_layer.reset();
        message_latch->Signal();
      };

  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  AddNativeCallback("CaptureRootLayer",
                    CREATE_NATIVE_ENTRY(capture_root_layer));
  AddNativeCallback("ValidateLayerTreeCounts",
                    CREATE_NATIVE_ENTRY(validate_layer_tree_counts));
  AddNativeCallback("ValidateEngineLayerDispose",
                    CREATE_NATIVE_ENTRY(validate_engine_layer_dispose));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("validateEngineLayerDispose");

  shell->RunEngine(std::move(configuration), [](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();
  DestroyShell(std::move(shell), task_runners);
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(clang-analyzer-core.StackAddressEscape)

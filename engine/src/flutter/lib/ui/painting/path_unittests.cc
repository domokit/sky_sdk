// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/path.h"

#include <memory>

#include "flutter/common/task_runners.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/common/shell_test.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/testing/testing.h"

namespace flutter {
namespace testing {

TEST_F(ShellTest, PathVolatilityOldPathsBecomeNonVolatile) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();

  auto native_validate_path = [message_latch](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    intptr_t peer = 0;
    Dart_Handle result = Dart_GetNativeInstanceField(
        handle, tonic::DartWrappable::kPeerIndex, &peer);
    EXPECT_FALSE(Dart_IsError(result));
    CanvasPath* path = reinterpret_cast<CanvasPath*>(peer);
    EXPECT_TRUE(path);

    for (uint32_t i = 0; i <= DlPath::kMaxVolatileUses * 2; i++) {
      EXPECT_TRUE(path->path().IsVolatile());
      EXPECT_TRUE(path->path().GetSkPath().isVolatile());
      // Getting the SkPath without expressing intent for rendering will not
      // progress towards non-volatility
    }
    EXPECT_TRUE(path->path().IsVolatile());
    EXPECT_TRUE(path->path().GetSkPath().isVolatile());

    for (uint32_t i = 0; i < DlPath::kMaxVolatileUses; i++) {
      path->path().WillRenderSkPath();
      EXPECT_TRUE(path->path().IsVolatile());
      EXPECT_TRUE(path->path().GetSkPath().isVolatile());
    }
    // One last intent to render will make it non-volatile
    path->path().WillRenderSkPath();
    EXPECT_FALSE(path->path().IsVolatile());
    EXPECT_FALSE(path->path().GetSkPath().isVolatile());

    DlPath saved_path = path->path();
    path->addOval(10, 10, 20, 20);

    // Meanwhile if the path being constructed by the CanvasPath object
    // is changed further, new paths extracted via path() are again volatile.
    EXPECT_TRUE(path->path().IsVolatile());
    EXPECT_TRUE(path->path().GetSkPath().isVolatile());

    // But the saved versions copied before the changes are still non-volatile
    EXPECT_FALSE(saved_path.IsVolatile());
    EXPECT_FALSE(saved_path.GetSkPath().isVolatile());

    message_latch->Signal();
  };

  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  AddNativeCallback("ValidatePath", CREATE_NATIVE_ENTRY(native_validate_path));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("createPath");

  shell->RunEngine(std::move(configuration), [](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();

  DestroyShell(std::move(shell), task_runners);
}

// Screen diffing tests use deterministic rendering. Allowing a path to be
// volatile or not for an individual frame can result in minor pixel differences
// that cause the test to fail.
// If deterministic rendering is enabled, the tracker should be disabled and
// paths should always be non-volatile.
TEST_F(ShellTest, DeterministicRenderingDisablesPathVolatility) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();

  auto native_validate_path = [message_latch](Dart_NativeArguments args) {
    auto handle = Dart_GetNativeArgument(args, 0);
    intptr_t peer = 0;
    Dart_Handle result = Dart_GetNativeInstanceField(
        handle, tonic::DartWrappable::kPeerIndex, &peer);
    EXPECT_FALSE(Dart_IsError(result));
    CanvasPath* path = reinterpret_cast<CanvasPath*>(peer);
    EXPECT_TRUE(path);

    for (uint32_t i = 0; i <= DlPath::kMaxVolatileUses * 2; i++) {
      EXPECT_FALSE(path->path().IsVolatile());
      EXPECT_FALSE(path->path().GetSkPath().isVolatile());
      path->path().WillRenderSkPath();
    }
    EXPECT_FALSE(path->path().IsVolatile());
    EXPECT_FALSE(path->path().GetSkPath().isVolatile());

    message_latch->Signal();
  };

  Settings settings = CreateSettingsForFixture();
  settings.skia_deterministic_rendering_on_cpu = true;
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  AddNativeCallback("ValidatePath", CREATE_NATIVE_ENTRY(native_validate_path));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("createPath");

  shell->RunEngine(std::move(configuration), [](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();

  DestroyShell(std::move(shell), task_runners);
}

}  // namespace testing
}  // namespace flutter

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>

#include "flutter/common/task_runners.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/common/shell_test.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/testing/testing.h"
#include "third_party/dart/runtime/include/dart_api.h"

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace flutter {
namespace testing {

using HooksTest = ShellTest;

#define CHECK_DART_ERROR(name) \
  EXPECT_FALSE(Dart_IsError(name)) << Dart_GetError(name)

TEST_F(HooksTest, HooksUnitTests) {
  auto settings = CreateSettingsForFixture();

  TaskRunners task_runners(GetCurrentTestName(),       // label
                           GetCurrentTaskRunner(),     // platform
                           CreateNewThread("raster"),  // raster
                           CreateNewThread("ui"),      // ui
                           CreateNewThread("io")       // io
  );

  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell->IsSetup());

  auto call_hook = [](Dart_NativeArguments args) {
    Dart_Handle hook_name = Dart_GetNativeArgument(args, 0);
    CHECK_DART_ERROR(hook_name);

    Dart_Handle ui_library = Dart_LookupLibrary(tonic::ToDart("dart:ui"));
    CHECK_DART_ERROR(ui_library);

    Dart_Handle hook = Dart_GetField(ui_library, hook_name);
    CHECK_DART_ERROR(hook);

    Dart_Handle arg_count_handle = Dart_GetNativeArgument(args, 1);
    CHECK_DART_ERROR(arg_count_handle);

    int64_t arg_count;
    Dart_IntegerToInt64(arg_count_handle, &arg_count);

    std::vector<Dart_Handle> hook_args;
    for (int i = 0; i < static_cast<int>(arg_count); i++) {
      hook_args.push_back(Dart_GetNativeArgument(args, 2 + i));
      CHECK_DART_ERROR(hook_args.back());
    }

    Dart_Handle hook_result =
        Dart_InvokeClosure(hook, hook_args.size(), hook_args.data());
    CHECK_DART_ERROR(hook_result);
  };

  auto finished = [&message_latch](Dart_NativeArguments args) {
    message_latch->Signal();
  };
  AddNativeCallback("CallHook", CREATE_NATIVE_ENTRY(call_hook));
  AddNativeCallback("Finish", CREATE_NATIVE_ENTRY(finished));

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("hooksTests");

  shell->RunEngine(std::move(configuration), [](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();
  DestroyShell(std::move(shell), task_runners);
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(clang-analyzer-core.StackAddressEscape)

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/dart_isolate.h"

#include "flutter/fml/paths.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/runtime/dart_vm_lifecycle.h"
#include "flutter/testing/dart_isolate_runner.h"
#include "flutter/testing/fixture_test.h"
#include "flutter/testing/testing.h"

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace flutter {
namespace testing {

const std::string kKernelFileName = "no_plugin_registrant_kernel_blob.bin";
const std::string kElfFileName = "no_plugin_registrant_app_elf_snapshot.so";

class DartIsolateTest : public FixtureTest {
 public:
  DartIsolateTest() : FixtureTest(kKernelFileName, kElfFileName, "") {}
};

TEST_F(DartIsolateTest, DartPluginRegistrantIsNotPresent) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());

  std::vector<std::string> messages;
  fml::AutoResetWaitableEvent latch;

  AddNativeCallback(
      "PassMessage",
      CREATE_NATIVE_ENTRY(([&latch, &messages](Dart_NativeArguments args) {
        auto message = tonic::DartConverter<std::string>::FromDart(
            Dart_GetNativeArgument(args, 0));
        messages.push_back(message);
        latch.Signal();
      })));

  auto settings = CreateSettingsForFixture();
  auto did_throw_exception = false;
  settings.unhandled_exception_callback = [&](const std::string& error,
                                              const std::string& stack_trace) {
    did_throw_exception = true;
    return true;
  };

  auto vm_ref = DartVMRef::Create(settings);
  auto thread = CreateNewThread();
  TaskRunners task_runners(GetCurrentTestName(),  //
                           thread,                //
                           thread,                //
                           thread,                //
                           thread                 //
  );

  auto kernel_path =
      fml::paths::JoinPaths({GetFixturesPath(), kKernelFileName});
  auto isolate = RunDartCodeInIsolate(vm_ref, settings, task_runners, "main",
                                      {}, kernel_path);

  ASSERT_TRUE(isolate);
  ASSERT_EQ(isolate->get()->GetPhase(), DartIsolate::Phase::Running);

  latch.Wait();

  ASSERT_EQ(messages.size(), 1u);
  ASSERT_EQ(messages[0], "main() was called");
  ASSERT_FALSE(did_throw_exception);
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(clang-analyzer-core.StackAddressEscape)

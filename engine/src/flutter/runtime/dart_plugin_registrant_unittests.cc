// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/dart_isolate.h"

#include <cstdlib>
#include "flutter/fml/paths.h"
#include "flutter/runtime/dart_plugin_registrant.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/runtime/dart_vm_lifecycle.h"
#include "flutter/testing/dart_isolate_runner.h"
#include "flutter/testing/fixture_test.h"
#include "flutter/testing/testing.h"

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace flutter {
namespace testing {

const std::string kKernelFileName = "plugin_registrant_kernel_blob.bin";
const std::string kElfFileName = "plugin_registrant_app_elf_snapshot.so";

class DartIsolateTest : public FixtureTest {
 public:
  DartIsolateTest() : FixtureTest(kKernelFileName, kElfFileName, "") {}

  void OverrideDartPluginRegistrant(const std::string& override_value) {
    dart_plugin_registrant_library_ = override_value;
    dart_plugin_registrant_library_override =
        dart_plugin_registrant_library_.c_str();
  }

  void SetUp() override {
    std::string source_path = GetSourcePath();
    if (source_path[0] != '/') {
      // On windows we need an extra '/' prefix.
      source_path = "/" + source_path;
    }
    std::string registrant_uri = std::string("file://") + source_path +
                                 "flutter/runtime/fixtures/dart_tool/"
                                 "flutter_build/dart_plugin_registrant.dart";
    OverrideDartPluginRegistrant(registrant_uri);
  }

  void TearDown() override {
    dart_plugin_registrant_library_override = nullptr;
  }

  std::string dart_plugin_registrant_library_;
};

TEST_F(DartIsolateTest, DartPluginRegistrantIsPresent) {
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
  auto isolate =
      RunDartCodeInIsolate(vm_ref, settings, task_runners,
                           "mainForPluginRegistrantTest", {}, kernel_path);

  ASSERT_TRUE(isolate);
  ASSERT_EQ(isolate->get()->GetPhase(), DartIsolate::Phase::Running);

  latch.Wait();

  ASSERT_EQ(messages.size(), 1u);
  ASSERT_EQ(messages[0], "_PluginRegistrant.register() was called");
}

TEST_F(DartIsolateTest, DartPluginRegistrantFromBackgroundIsolate) {
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
  auto isolate = RunDartCodeInIsolate(
      vm_ref, settings, task_runners,
      "callDartPluginRegistrantFromBackgroundIsolate", {}, kernel_path);

  ASSERT_TRUE(isolate);
  ASSERT_EQ(isolate->get()->GetPhase(), DartIsolate::Phase::Running);

  latch.Wait();

  ASSERT_EQ(messages.size(), 1u);
  ASSERT_EQ(messages[0],
            "_PluginRegistrant.register() was called on background isolate");
}

TEST_F(DartIsolateTest, DartPluginRegistrantNotFromBackgroundIsolate) {
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
  auto isolate = RunDartCodeInIsolate(
      vm_ref, settings, task_runners,
      "dontCallDartPluginRegistrantFromBackgroundIsolate", {}, kernel_path);

  ASSERT_TRUE(isolate);
  ASSERT_EQ(isolate->get()->GetPhase(), DartIsolate::Phase::Running);

  latch.Wait();

  ASSERT_EQ(messages.size(), 1u);
  ASSERT_EQ(
      messages[0],
      "_PluginRegistrant.register() was not called on background isolate");
}

TEST_F(DartIsolateTest, DartPluginRegistrantWhenRegisteringBackgroundIsolate) {
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
  auto isolate = RunDartCodeInIsolate(
      vm_ref, settings, task_runners,
      "registerBackgroundIsolateCallsDartPluginRegistrant", {}, kernel_path);

  ASSERT_TRUE(isolate);
  ASSERT_EQ(isolate->get()->GetPhase(), DartIsolate::Phase::Running);

  latch.Wait();

  ASSERT_EQ(messages.size(), 1u);
  ASSERT_EQ(messages[0],
            "_PluginRegistrant.register() was called on background isolate");
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(clang-analyzer-core.StackAddressEscape)

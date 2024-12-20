// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/dart_vm_lifecycle.h"
#include "flutter/testing/dart_isolate_runner.h"
#include "flutter/testing/fixture_test.h"
#include "flutter/testing/testing.h"
#include "flutter/third_party/tonic/converter/dart_converter.h"

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace flutter {
namespace testing {

class TypeConversionsTest : public FixtureTest {
 public:
  TypeConversionsTest()
      : settings_(CreateSettingsForFixture()),
        vm_(DartVMRef::Create(settings_)) {}

  ~TypeConversionsTest() = default;

  [[nodiscard]] bool RunWithEntrypoint(const std::string& entrypoint) {
    if (running_isolate_) {
      return false;
    }
    auto thread = CreateNewThread();
    TaskRunners single_threaded_task_runner(GetCurrentTestName(), thread,
                                            thread, thread, thread);
    auto isolate =
        RunDartCodeInIsolate(vm_, settings_, single_threaded_task_runner,
                             entrypoint, {}, GetDefaultKernelFilePath());
    if (!isolate || isolate->get()->GetPhase() != DartIsolate::Phase::Running) {
      return false;
    }

    running_isolate_ = std::move(isolate);
    return true;
  }

 private:
  Settings settings_;
  DartVMRef vm_;
  std::unique_ptr<AutoIsolateShutdown> running_isolate_;
  FML_DISALLOW_COPY_AND_ASSIGN(TypeConversionsTest);
};

TEST_F(TypeConversionsTest, TestFixture) {
  ASSERT_TRUE(RunWithEntrypoint("main"));
}

TEST_F(TypeConversionsTest, CanConvertEmptyList) {
  fml::AutoResetWaitableEvent event;
  AddNativeCallback(
      "NotifySuccess", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto bool_handle = Dart_GetNativeArgument(args, 0);
        ASSERT_FALSE(tonic::CheckAndHandleError(bool_handle));
        ASSERT_TRUE(tonic::DartConverter<bool>::FromDart(bool_handle));
        event.Signal();
      }));
  AddNativeCallback(
      "NotifyNative", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments) {
        std::vector<int64_t> items;
        auto items_handle = tonic::ToDart(items);
        ASSERT_FALSE(tonic::CheckAndHandleError(items_handle));
        tonic::DartInvokeField(::Dart_RootLibrary(), "testCanConvertEmptyList",
                               {items_handle});
      }));
  ASSERT_TRUE(RunWithEntrypoint("trampoline"));
  event.Wait();
}

TEST_F(TypeConversionsTest, CanConvertListOfStrings) {
  fml::AutoResetWaitableEvent event;
  AddNativeCallback(
      "NotifySuccess", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto bool_handle = Dart_GetNativeArgument(args, 0);
        ASSERT_FALSE(tonic::CheckAndHandleError(bool_handle));
        ASSERT_TRUE(tonic::DartConverter<bool>::FromDart(bool_handle));
        event.Signal();
      }));
  AddNativeCallback(
      "NotifyNative", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments) {
        std::vector<std::string> items;
        items.push_back("tinker");
        items.push_back("tailor");
        items.push_back("soldier");
        items.push_back("sailor");
        auto items_handle = tonic::ToDart(items);
        ASSERT_FALSE(tonic::CheckAndHandleError(items_handle));
        tonic::DartInvokeField(::Dart_RootLibrary(),
                               "testCanConvertListOfStrings", {items_handle});
      }));
  ASSERT_TRUE(RunWithEntrypoint("trampoline"));
  event.Wait();
}

TEST_F(TypeConversionsTest, CanConvertListOfDoubles) {
  fml::AutoResetWaitableEvent event;
  AddNativeCallback(
      "NotifySuccess", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto bool_handle = Dart_GetNativeArgument(args, 0);
        ASSERT_FALSE(tonic::CheckAndHandleError(bool_handle));
        ASSERT_TRUE(tonic::DartConverter<bool>::FromDart(bool_handle));
        event.Signal();
      }));
  AddNativeCallback(
      "NotifyNative", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments) {
        std::vector<double> items;
        items.push_back(1.0);
        items.push_back(2.0);
        items.push_back(3.0);
        items.push_back(4.0);
        auto items_handle = tonic::ToDart(items);
        ASSERT_FALSE(tonic::CheckAndHandleError(items_handle));
        tonic::DartInvokeField(::Dart_RootLibrary(),
                               "testCanConvertListOfDoubles", {items_handle});
      }));
  ASSERT_TRUE(RunWithEntrypoint("trampoline"));
  event.Wait();
}

TEST_F(TypeConversionsTest, CanConvertListOfInts) {
  fml::AutoResetWaitableEvent event;
  AddNativeCallback(
      "NotifySuccess", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto bool_handle = Dart_GetNativeArgument(args, 0);
        ASSERT_FALSE(tonic::CheckAndHandleError(bool_handle));
        ASSERT_TRUE(tonic::DartConverter<bool>::FromDart(bool_handle));
        event.Signal();
      }));
  AddNativeCallback(
      "NotifyNative", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments) {
        std::vector<int32_t> items;
        items.push_back(1);
        items.push_back(2);
        items.push_back(3);
        items.push_back(4);
        auto items_handle = tonic::ToDart(items);
        ASSERT_FALSE(tonic::CheckAndHandleError(items_handle));
        tonic::DartInvokeField(::Dart_RootLibrary(), "testCanConvertListOfInts",
                               {items_handle});
      }));
  ASSERT_TRUE(RunWithEntrypoint("trampoline"));
  event.Wait();
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(clang-analyzer-core.StackAddressEscape)

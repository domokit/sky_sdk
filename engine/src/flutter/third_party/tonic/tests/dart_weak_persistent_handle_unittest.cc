// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/dart_isolate_runner.h"
#include "flutter/testing/fixture_test.h"

namespace flutter {
namespace testing {

namespace {
void NopFinalizer(void* isolate_callback_data, void* peer) {}
}  // namespace

class DartWeakPersistentHandle : public FixtureTest {
 public:
  DartWeakPersistentHandle()
      : settings_(CreateSettingsForFixture()),
        vm_(DartVMRef::Create(settings_)) {}

  ~DartWeakPersistentHandle() = default;

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

  [[nodiscard]] bool RunInIsolateScope(std::function<bool(void)> closure) {
    return running_isolate_->RunInIsolateScope(closure);
  }

 private:
  Settings settings_;
  DartVMRef vm_;
  std::unique_ptr<AutoIsolateShutdown> running_isolate_;
  FML_DISALLOW_COPY_AND_ASSIGN(DartWeakPersistentHandle);
};

TEST_F(DartWeakPersistentHandle, ClearImmediately) {
  auto weak_persistent_value = tonic::DartWeakPersistentValue();

  fml::AutoResetWaitableEvent event;

  AddNativeCallback(
      "GiveObjectToNative", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto handle = Dart_GetNativeArgument(args, 0);

        auto dart_state = tonic::DartState::Current();
        ASSERT_TRUE(dart_state);
        ASSERT_TRUE(tonic::DartState::Current());
        weak_persistent_value.Set(dart_state, handle, nullptr, 0, NopFinalizer);

        weak_persistent_value.Clear();

        event.Signal();
      }));

  ASSERT_TRUE(RunWithEntrypoint("callGiveObjectToNative"));

  event.Wait();
}

TEST_F(DartWeakPersistentHandle, ClearLaterCc) {
  auto weak_persistent_value = tonic::DartWeakPersistentValue();

  fml::AutoResetWaitableEvent event;

  AddNativeCallback(
      "GiveObjectToNative", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto handle = Dart_GetNativeArgument(args, 0);

        auto dart_state = tonic::DartState::Current();
        ASSERT_TRUE(dart_state);
        ASSERT_TRUE(tonic::DartState::Current());
        weak_persistent_value.Set(dart_state, handle, nullptr, 0, NopFinalizer);

        // Do not clear handle immediately.

        event.Signal();
      }));

  ASSERT_TRUE(RunWithEntrypoint("callGiveObjectToNative"));

  event.Wait();

  ASSERT_TRUE(RunInIsolateScope([&weak_persistent_value]() -> bool {
    // Clear on initiative of native.
    weak_persistent_value.Clear();
    return true;
  }));
}

TEST_F(DartWeakPersistentHandle, ClearLaterDart) {
  auto weak_persistent_value = tonic::DartWeakPersistentValue();

  fml::AutoResetWaitableEvent event;

  AddNativeCallback(
      "GiveObjectToNative", CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
        auto handle = Dart_GetNativeArgument(args, 0);

        auto dart_state = tonic::DartState::Current();
        ASSERT_TRUE(dart_state);
        ASSERT_TRUE(tonic::DartState::Current());
        weak_persistent_value.Set(dart_state, handle, nullptr, 0, NopFinalizer);

        // Do not clear handle immediately.
      }));

  AddNativeCallback("SignalDone",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      // Clear on initiative of Dart.
                      weak_persistent_value.Clear();

                      event.Signal();
                    }));

  ASSERT_TRUE(RunWithEntrypoint("testClearLater"));

  event.Wait();
}

// Handle outside the test body scope so it survives until isolate shutdown.
tonic::DartWeakPersistentValue global_weak_persistent_value =
    tonic::DartWeakPersistentValue();

TEST_F(DartWeakPersistentHandle, ClearOnShutdown) {
  fml::AutoResetWaitableEvent event;

  AddNativeCallback("GiveObjectToNative",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      auto handle = Dart_GetNativeArgument(args, 0);

                      auto dart_state = tonic::DartState::Current();
                      ASSERT_TRUE(dart_state);
                      ASSERT_TRUE(tonic::DartState::Current());

                      // The test is repeated, ensure the global var is
                      // cleared before use.
                      global_weak_persistent_value.Clear();

                      global_weak_persistent_value.Set(
                          dart_state, handle, nullptr, 0, NopFinalizer);

                      // Do not clear handle, so it is cleared on shutdown.

                      event.Signal();
                    }));

  ASSERT_TRUE(RunWithEntrypoint("callGiveObjectToNative"));

  event.Wait();
}

}  // namespace testing
}  // namespace flutter

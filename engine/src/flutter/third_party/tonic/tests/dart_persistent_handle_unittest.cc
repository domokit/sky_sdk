// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/dart_isolate_runner.h"
#include "flutter/testing/fixture_test.h"

namespace flutter {
namespace testing {

class DartPersistentHandleTest : public FixtureTest {
 public:
  DartPersistentHandleTest()
      : settings_(CreateSettingsForFixture()),
        vm_(DartVMRef::Create(settings_)),
        thread_(CreateNewThread()),
        task_runners_(GetCurrentTestName(),
                      thread_,
                      thread_,
                      thread_,
                      thread_) {}

  ~DartPersistentHandleTest() = default;

  [[nodiscard]] bool RunWithEntrypoint(const std::string& entrypoint) {
    if (running_isolate_) {
      return false;
    }
    auto isolate =
        RunDartCodeInIsolate(vm_, settings_, task_runners_, entrypoint, {},
                             GetDefaultKernelFilePath());
    if (!isolate || isolate->get()->GetPhase() != DartIsolate::Phase::Running) {
      return false;
    }

    running_isolate_ = std::move(isolate);
    return true;
  }

 protected:
  Settings settings_;
  DartVMRef vm_;
  std::unique_ptr<AutoIsolateShutdown> running_isolate_;
  fml::RefPtr<fml::TaskRunner> thread_;
  TaskRunners task_runners_;
  FML_DISALLOW_COPY_AND_ASSIGN(DartPersistentHandleTest);
};

TEST_F(DartPersistentHandleTest, ClearAfterShutdown) {
  auto persistent_value = tonic::DartPersistentValue();

  fml::AutoResetWaitableEvent event;
  AddNativeCallback("GiveObjectToNative",
                    CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                      auto handle = Dart_GetNativeArgument(args, 0);

                      auto dart_state = tonic::DartState::Current();
                      ASSERT_TRUE(dart_state);
                      ASSERT_TRUE(tonic::DartState::Current());
                      persistent_value.Set(dart_state, handle);

                      event.Signal();
                    }));

  ASSERT_TRUE(RunWithEntrypoint("callGiveObjectToNative"));
  event.Wait();

  running_isolate_->Shutdown();

  fml::AutoResetWaitableEvent clear;
  task_runners_.GetUITaskRunner()->PostTask([&] {
    persistent_value.Clear();
    clear.Signal();
  });
  clear.Wait();
}
}  // namespace testing
}  // namespace flutter

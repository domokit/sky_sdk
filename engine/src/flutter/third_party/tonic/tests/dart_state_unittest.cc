// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/third_party/tonic/dart_state.h"
#include "flutter/common/task_runners.h"
#include "flutter/runtime/dart_vm_lifecycle.h"
#include "flutter/runtime/isolate_configuration.h"
#include "flutter/testing/fixture_test.h"

namespace flutter {
namespace testing {

using DartState = FixtureTest;

TEST_F(DartState, CurrentWithNullDataDoesNotSegfault) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto settings = CreateSettingsForFixture();
  auto vm_snapshot = DartSnapshot::VMSnapshotFromSettings(settings);
  auto isolate_snapshot = DartSnapshot::IsolateSnapshotFromSettings(settings);
  auto vm_ref = DartVMRef::Create(settings, vm_snapshot, isolate_snapshot);
  ASSERT_TRUE(vm_ref);
  auto vm_data = vm_ref.GetVMData();
  ASSERT_TRUE(vm_data);
  TaskRunners task_runners(GetCurrentTestName(),    //
                           GetCurrentTaskRunner(),  //
                           GetCurrentTaskRunner(),  //
                           GetCurrentTaskRunner(),  //
                           GetCurrentTaskRunner()   //
  );
  auto isolate_configuration =
      IsolateConfiguration::InferFromSettings(settings);
  Dart_IsolateFlags isolate_flags;
  Dart_IsolateFlagsInitialize(&isolate_flags);
  isolate_flags.null_safety =
      isolate_configuration->IsNullSafetyEnabled(*isolate_snapshot);
  isolate_flags.snapshot_is_dontneed_safe = isolate_snapshot->IsDontNeedSafe();
  char* error;
  Dart_CreateIsolateGroup(
      "main.dart", "main", vm_data->GetIsolateSnapshot()->GetDataMapping(),
      vm_data->GetIsolateSnapshot()->GetInstructionsMapping(), &isolate_flags,
      nullptr, nullptr, &error);
  ASSERT_FALSE(error) << error;
  ::free(error);

  ASSERT_FALSE(tonic::DartState::Current());

  Dart_ShutdownIsolate();
  ASSERT_TRUE(Dart_CurrentIsolate() == nullptr);
}

TEST_F(DartState, IsShuttingDown) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto settings = CreateSettingsForFixture();
  auto vm_ref = DartVMRef::Create(settings);
  ASSERT_TRUE(vm_ref);
  auto vm_data = vm_ref.GetVMData();
  ASSERT_TRUE(vm_data);
  TaskRunners task_runners(GetCurrentTestName(),    //
                           GetCurrentTaskRunner(),  //
                           GetCurrentTaskRunner(),  //
                           GetCurrentTaskRunner(),  //
                           GetCurrentTaskRunner()   //
  );
  auto isolate_configuration =
      IsolateConfiguration::InferFromSettings(settings);

  UIDartState::Context context(std::move(task_runners));
  context.advisory_script_uri = "main.dart";
  context.advisory_script_entrypoint = "main";
  auto weak_isolate = DartIsolate::CreateRunningRootIsolate(
      vm_data->GetSettings(),              // settings
      vm_data->GetIsolateSnapshot(),       // isolate snapshot
      nullptr,                             // platform configuration
      DartIsolate::Flags{},                // flags
      nullptr,                             // root_isolate_create_callback
      settings.isolate_create_callback,    // isolate create callback
      settings.isolate_shutdown_callback,  // isolate shutdown callback
      "main",                              // dart entrypoint
      std::nullopt,                        // dart entrypoint library
      {},                                  // dart entrypoint arguments
      std::move(isolate_configuration),    // isolate configuration
      std::move(context)                   // engine context
  );
  auto root_isolate = weak_isolate.lock();
  ASSERT_TRUE(root_isolate);

  tonic::DartState* dart_state = root_isolate.get();
  ASSERT_FALSE(dart_state->IsShuttingDown());

  ASSERT_TRUE(root_isolate->Shutdown());

  ASSERT_TRUE(dart_state->IsShuttingDown());
}

}  // namespace testing
}  // namespace flutter

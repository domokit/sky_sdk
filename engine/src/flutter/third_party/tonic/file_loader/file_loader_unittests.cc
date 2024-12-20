// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/dart_isolate_runner.h"
#include "flutter/testing/fixture_test.h"

#include "tonic/converter/dart_converter.h"
#include "tonic/file_loader/file_loader.h"

namespace flutter {
namespace testing {

using FileLoaderTest = FixtureTest;

TEST_F(FileLoaderTest, CanonicalizesFileUrlCorrectly) {
  ASSERT_FALSE(DartVMRef::IsInstanceRunning());
  auto settings = CreateSettingsForFixture();
  auto vm_snapshot = DartSnapshot::VMSnapshotFromSettings(settings);
  auto isolate_snapshot = DartSnapshot::IsolateSnapshotFromSettings(settings);
  auto vm_ref = DartVMRef::Create(settings, vm_snapshot, isolate_snapshot);
  ASSERT_TRUE(vm_ref);

  TaskRunners task_runners(GetCurrentTestName(),    //
                           GetCurrentTaskRunner(),  //
                           GetCurrentTaskRunner(),  //
                           GetCurrentTaskRunner(),  //
                           GetCurrentTaskRunner()   //
  );
  auto isolate = RunDartCodeInIsolate(vm_ref, settings, task_runners, "main",
                                      {}, GetDefaultKernelFilePath());
  ASSERT_TRUE(isolate);

  ASSERT_TRUE(isolate->RunInIsolateScope([]() {
    tonic::FileLoader file_loader;
    std::string original_url = "file:///Users/test/foo";
    Dart_Handle dart_url = tonic::StdStringToDart(original_url);
    auto canonicalized_url = file_loader.CanonicalizeURL(Dart_Null(), dart_url);
    EXPECT_TRUE(canonicalized_url);
    EXPECT_EQ(tonic::StdStringFromDart(canonicalized_url), original_url);
    return true;
  }));
}

}  // namespace testing
}  // namespace flutter

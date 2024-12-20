// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/shell.h"

#include "flutter/benchmarking/benchmarking.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/testing/dart_fixture.h"
#include "flutter/testing/dart_isolate_runner.h"
#include "flutter/testing/testing.h"
#include "fml/synchronization/count_down_latch.h"
#include "runtime/dart_vm_lifecycle.h"

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace flutter::testing {

class DartNativeBenchmarks : public DartFixture, public benchmark::Fixture {
 public:
  DartNativeBenchmarks() : DartFixture() {}

  void SetUp(const ::benchmark::State& state) {}

  void TearDown(const ::benchmark::State& state) {}

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(DartNativeBenchmarks);
};

BENCHMARK_F(DartNativeBenchmarks, TimeToFirstNativeMessageFromIsolateInNewVM)
(benchmark::State& st) {
  while (st.KeepRunning()) {
    fml::AutoResetWaitableEvent latch;
    st.PauseTiming();
    ASSERT_FALSE(DartVMRef::IsInstanceRunning());
    AddNativeCallback("NotifyNative",
                      CREATE_NATIVE_ENTRY(([&latch](Dart_NativeArguments args) {
                        latch.Signal();
                      })));

    const auto settings = CreateSettingsForFixture();
    DartVMRef vm_ref = DartVMRef::Create(settings);

    ThreadHost thread_host("io.flutter.test.DartNativeBenchmarks.",
                           ThreadHost::Type::kPlatform | ThreadHost::Type::kIo |
                               ThreadHost::Type::kUi);
    TaskRunners task_runners(
        "test",
        thread_host.platform_thread->GetTaskRunner(),  // platform
        thread_host.platform_thread->GetTaskRunner(),  // raster
        thread_host.ui_thread->GetTaskRunner(),        // ui
        thread_host.io_thread->GetTaskRunner()         // io
    );

    {
      st.ResumeTiming();
      auto isolate =
          RunDartCodeInIsolate(vm_ref, settings, task_runners, "notifyNative",
                               {}, GetDefaultKernelFilePath());
      ASSERT_TRUE(isolate);
      ASSERT_EQ(isolate->get()->GetPhase(), DartIsolate::Phase::Running);
      latch.Wait();
    }
  }
}

BENCHMARK_F(DartNativeBenchmarks, MultipleDartToNativeMessages)
(benchmark::State& st) {
  while (st.KeepRunning()) {
    fml::CountDownLatch latch(1000);
    st.PauseTiming();
    ASSERT_FALSE(DartVMRef::IsInstanceRunning());
    AddNativeCallback("NotifyNative",
                      CREATE_NATIVE_ENTRY(([&latch](Dart_NativeArguments args) {
                        latch.CountDown();
                      })));

    const auto settings = CreateSettingsForFixture();
    DartVMRef vm_ref = DartVMRef::Create(settings);

    ThreadHost thread_host("io.flutter.test.DartNativeBenchmarks.",
                           ThreadHost::Type::kPlatform | ThreadHost::Type::kIo |
                               ThreadHost::Type::kUi);
    TaskRunners task_runners(
        "test",
        thread_host.platform_thread->GetTaskRunner(),  // platform
        thread_host.platform_thread->GetTaskRunner(),  // raster
        thread_host.ui_thread->GetTaskRunner(),        // ui
        thread_host.io_thread->GetTaskRunner()         // io
    );

    {
      st.ResumeTiming();
      auto isolate = RunDartCodeInIsolate(vm_ref, settings, task_runners,
                                          "thousandCallsToNative", {},
                                          GetDefaultKernelFilePath());
      ASSERT_TRUE(isolate);
      ASSERT_EQ(isolate->get()->GetPhase(), DartIsolate::Phase::Running);
      latch.Wait();
    }
  }
}

}  // namespace flutter::testing

// NOLINTEND(clang-analyzer-core.StackAddressEscape)

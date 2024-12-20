// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include <cstddef>
#include <memory>

#include "flutter/common/task_runners.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/lib/ui/painting/vertices.h"
#include "flutter/lib/ui/window/platform_configuration.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/common/shell_test.h"
#include "flutter/shell/common/thread_host.h"
#include "flutter/testing/testing.h"
#include "gmock/gmock.h"
#include "googletest/googletest/include/gtest/gtest.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/converter/dart_converter.h"

namespace flutter {
namespace testing {

class PlatformConfigurationTest : public ShellTest {};

TEST_F(PlatformConfigurationTest, Initialization) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();

  auto nativeValidateConfiguration =
      [message_latch](Dart_NativeArguments args) {
        PlatformConfiguration* configuration =
            UIDartState::Current()->platform_configuration();
        ASSERT_NE(configuration->GetMetrics(0), nullptr);
        ASSERT_EQ(configuration->GetMetrics(0)->device_pixel_ratio, 1.0);
        ASSERT_EQ(configuration->GetMetrics(0)->physical_width, 0.0);
        ASSERT_EQ(configuration->GetMetrics(0)->physical_height, 0.0);

        message_latch->Signal();
      };

  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  AddNativeCallback("ValidateConfiguration",
                    CREATE_NATIVE_ENTRY(nativeValidateConfiguration));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto run_configuration = RunConfiguration::InferFromSettings(settings);
  run_configuration.SetEntrypoint("validateConfiguration");

  shell->RunEngine(std::move(run_configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(PlatformConfigurationTest, WindowMetricsUpdate) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();

  auto nativeValidateConfiguration =
      [message_latch](Dart_NativeArguments args) {
        PlatformConfiguration* configuration =
            UIDartState::Current()->platform_configuration();

        ASSERT_NE(configuration->GetMetrics(0), nullptr);
        bool has_view = configuration->UpdateViewMetrics(
            0, ViewportMetrics{2.0, 10.0, 20.0, 22, 0});
        ASSERT_TRUE(has_view);
        ASSERT_EQ(configuration->GetMetrics(0)->device_pixel_ratio, 2.0);
        ASSERT_EQ(configuration->GetMetrics(0)->physical_width, 10.0);
        ASSERT_EQ(configuration->GetMetrics(0)->physical_height, 20.0);
        ASSERT_EQ(configuration->GetMetrics(0)->physical_touch_slop, 22);

        message_latch->Signal();
      };

  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  AddNativeCallback("ValidateConfiguration",
                    CREATE_NATIVE_ENTRY(nativeValidateConfiguration));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto run_configuration = RunConfiguration::InferFromSettings(settings);
  run_configuration.SetEntrypoint("validateConfiguration");

  shell->RunEngine(std::move(run_configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(PlatformConfigurationTest, GetWindowReturnsNullForNonexistentId) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();

  auto nativeValidateConfiguration =
      [message_latch](Dart_NativeArguments args) {
        PlatformConfiguration* configuration =
            UIDartState::Current()->platform_configuration();

        ASSERT_EQ(configuration->GetMetrics(1), nullptr);
        ASSERT_EQ(configuration->GetMetrics(2), nullptr);

        message_latch->Signal();
      };

  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  AddNativeCallback("ValidateConfiguration",
                    CREATE_NATIVE_ENTRY(nativeValidateConfiguration));

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto run_configuration = RunConfiguration::InferFromSettings(settings);
  run_configuration.SetEntrypoint("validateConfiguration");

  shell->RunEngine(std::move(run_configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(PlatformConfigurationTest, OnErrorHandlesError) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();
  bool did_throw = false;

  auto finish = [message_latch](Dart_NativeArguments args) {
    message_latch->Signal();
  };
  AddNativeCallback("Finish", CREATE_NATIVE_ENTRY(finish));

  Settings settings = CreateSettingsForFixture();
  settings.unhandled_exception_callback =
      [&did_throw](const std::string& exception,
                   const std::string& stack_trace) -> bool {
    did_throw = true;
    return false;
  };

  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto run_configuration = RunConfiguration::InferFromSettings(settings);
  run_configuration.SetEntrypoint("customOnErrorTrue");

  shell->RunEngine(std::move(run_configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();

  // Flush the UI task runner to make sure errors that were triggered had a turn
  // to propagate.
  task_runners.GetUITaskRunner()->PostTask(
      [&message_latch]() { message_latch->Signal(); });
  message_latch->Wait();

  ASSERT_FALSE(did_throw);
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(PlatformConfigurationTest, OnErrorDoesNotHandleError) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();
  std::string ex;
  std::string st;
  size_t throw_count = 0;

  auto finish = [message_latch](Dart_NativeArguments args) {
    message_latch->Signal();
  };
  AddNativeCallback("Finish", CREATE_NATIVE_ENTRY(finish));

  Settings settings = CreateSettingsForFixture();
  settings.unhandled_exception_callback =
      [&ex, &st, &throw_count](const std::string& exception,
                               const std::string& stack_trace) -> bool {
    throw_count += 1;
    ex = exception;
    st = stack_trace;
    return true;
  };

  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto run_configuration = RunConfiguration::InferFromSettings(settings);
  run_configuration.SetEntrypoint("customOnErrorFalse");

  shell->RunEngine(std::move(run_configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();

  // Flush the UI task runner to make sure errors that were triggered had a turn
  // to propagate.
  task_runners.GetUITaskRunner()->PostTask(
      [&message_latch]() { message_latch->Signal(); });
  message_latch->Wait();

  ASSERT_EQ(throw_count, 1ul);
  ASSERT_EQ(ex, "Exception: false") << ex;
  ASSERT_EQ(st.rfind("#0      customOnErrorFalse", 0), 0ul) << st;
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(PlatformConfigurationTest, OnErrorThrows) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();
  std::vector<std::string> errors;
  size_t throw_count = 0;

  auto finish = [message_latch](Dart_NativeArguments args) {
    message_latch->Signal();
  };
  AddNativeCallback("Finish", CREATE_NATIVE_ENTRY(finish));

  Settings settings = CreateSettingsForFixture();
  settings.unhandled_exception_callback =
      [&errors, &throw_count](const std::string& exception,
                              const std::string& stack_trace) -> bool {
    throw_count += 1;
    errors.push_back(exception);
    errors.push_back(stack_trace);
    return true;
  };

  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto run_configuration = RunConfiguration::InferFromSettings(settings);
  run_configuration.SetEntrypoint("customOnErrorThrow");

  shell->RunEngine(std::move(run_configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();

  // Flush the UI task runner to make sure errors that were triggered had a turn
  // to propagate.
  task_runners.GetUITaskRunner()->PostTask(
      [&message_latch]() { message_latch->Signal(); });
  message_latch->Wait();

  ASSERT_EQ(throw_count, 2ul);
  ASSERT_EQ(errors.size(), 4ul);
  ASSERT_EQ(errors[0], "Exception: throw2") << errors[0];
  ASSERT_EQ(errors[1].rfind("#0      customOnErrorThrow"), 0ul) << errors[1];
  ASSERT_EQ(errors[2], "Exception: throw1") << errors[2];
  ASSERT_EQ(errors[3].rfind("#0      customOnErrorThrow"), 0ul) << errors[3];

  DestroyShell(std::move(shell), task_runners);
}

TEST_F(PlatformConfigurationTest, SetDartPerformanceMode) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();
  auto finish = [message_latch](Dart_NativeArguments args) {
    // call needs to happen on the UI thread.
    Dart_PerformanceMode prev =
        Dart_SetPerformanceMode(Dart_PerformanceMode_Default);
    ASSERT_EQ(Dart_PerformanceMode_Latency, prev);
    message_latch->Signal();
  };
  AddNativeCallback("Finish", CREATE_NATIVE_ENTRY(finish));

  Settings settings = CreateSettingsForFixture();

  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);

  ASSERT_TRUE(shell->IsSetup());
  auto run_configuration = RunConfiguration::InferFromSettings(settings);
  run_configuration.SetEntrypoint("setLatencyPerformanceMode");

  shell->RunEngine(std::move(run_configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  message_latch->Wait();
  DestroyShell(std::move(shell), task_runners);
}

TEST_F(PlatformConfigurationTest, BeginFrameMonotonic) {
  auto message_latch = std::make_shared<fml::AutoResetWaitableEvent>();

  PlatformConfiguration* platform;

  // Called at the load time and will be in an Dart isolate.
  auto nativeValidateConfiguration = [message_latch,
                                      &platform](Dart_NativeArguments args) {
    platform = UIDartState::Current()->platform_configuration();

    // Hijacks the `_begin_frame` in hooks.dart so we can get a callback for
    // validation.
    auto field =
        Dart_GetField(Dart_RootLibrary(), tonic::ToDart("_beginFrameHijack"));
    platform->begin_frame_.Clear();
    platform->begin_frame_.Set(UIDartState::Current(), field);

    message_latch->Signal();
  };
  AddNativeCallback("ValidateConfiguration",
                    CREATE_NATIVE_ENTRY(nativeValidateConfiguration));

  std::vector<int64_t> frame_times;
  std::vector<uint64_t> frame_numbers;

  auto frame_latch = std::make_shared<fml::AutoResetWaitableEvent>();

  // Called for each `_begin_frame` that is hijacked.
  auto nativeBeginFrame = [frame_latch, &frame_times,
                           &frame_numbers](Dart_NativeArguments args) {
    int64_t microseconds;
    uint64_t frame_number;
    Dart_IntegerToInt64(Dart_GetNativeArgument(args, 0), &microseconds);
    Dart_IntegerToUint64(Dart_GetNativeArgument(args, 1), &frame_number);

    frame_times.push_back(microseconds);
    frame_numbers.push_back(frame_number);

    if (frame_times.size() == 3) {
      frame_latch->Signal();
    }
  };
  AddNativeCallback("BeginFrame", CREATE_NATIVE_ENTRY(nativeBeginFrame));

  Settings settings = CreateSettingsForFixture();
  TaskRunners task_runners("test",                  // label
                           GetCurrentTaskRunner(),  // platform
                           CreateNewThread(),       // raster
                           CreateNewThread(),       // ui
                           CreateNewThread()        // io
  );

  std::unique_ptr<Shell> shell = CreateShell(settings, task_runners);
  ASSERT_TRUE(shell->IsSetup());

  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("validateConfiguration");

  shell->RunEngine(std::move(configuration), [&](auto result) {
    ASSERT_EQ(result, Engine::RunStatus::Success);
  });

  // Wait for `nativeValidateConfiguration` to get called.
  message_latch->Wait();

  fml::TaskRunner::RunNowOrPostTask(
      shell->GetTaskRunners().GetUITaskRunner(), [platform]() {
        auto offset = fml::TimeDelta::FromMilliseconds(10);
        auto zero = fml::TimePoint();
        auto one = zero + offset;
        auto two = one + offset;

        platform->BeginFrame(zero, 1);
        platform->BeginFrame(two, 2);
        platform->BeginFrame(one, 3);
      });

  frame_latch->Wait();

  ASSERT_THAT(frame_times, ::testing::ElementsAre(0, 20000, 20000));
  ASSERT_THAT(frame_numbers, ::testing::ElementsAre(1, 2, 3));
  DestroyShell(std::move(shell), task_runners);
}

}  // namespace testing
}  // namespace flutter

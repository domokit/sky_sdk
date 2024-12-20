// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <thread>

#include "flutter/fml/message_loop_impl.h"
#include "flutter/fml/thread.h"
#include "flutter/shell/profiling/sampling_profiler.h"
#include "flutter/testing/testing.h"
#include "gmock/gmock.h"

using testing::_;
using testing::Invoke;

namespace fml {
namespace {
class MockTaskRunner : public fml::TaskRunner {
 public:
  inline static RefPtr<MockTaskRunner> Create() {
    return AdoptRef(new MockTaskRunner());
  }
  MOCK_METHOD(void, PostTask, (const fml::closure& task), (override));
  MOCK_METHOD(void,
              PostTaskForTime,
              (const fml::closure& task, fml::TimePoint target_time),
              (override));
  MOCK_METHOD(void,
              PostDelayedTask,
              (const fml::closure& task, fml::TimeDelta delay),
              (override));
  MOCK_METHOD(bool, RunsTasksOnCurrentThread, (), (override));
  MOCK_METHOD(TaskQueueId, GetTaskQueueId, (), (override));

 private:
  MockTaskRunner() : TaskRunner(fml::RefPtr<MessageLoopImpl>()) {}
};
}  // namespace
}  // namespace fml

namespace flutter {

TEST(SamplingProfilerTest, DeleteAfterStart) {
  auto thread =
      std::make_unique<fml::Thread>(flutter::testing::GetCurrentTestName());
  auto task_runner = fml::MockTaskRunner::Create();
  std::atomic<int> invoke_count = 0;

  // Ignore calls to PostTask since that would require mocking out calls to
  // Dart.
  EXPECT_CALL(*task_runner, PostDelayedTask(_, _))
      .WillRepeatedly(
          Invoke([&](const fml::closure& task, fml::TimeDelta delay) {
            invoke_count.fetch_add(1);
            thread->GetTaskRunner()->PostTask(task);
          }));

  {
    auto profiler = SamplingProfiler(
        "profiler",
        /*profiler_task_runner=*/task_runner, [] { return ProfileSample(); },
        /*num_samples_per_sec=*/1000);
    profiler.Start();
  }
  int invoke_count_at_delete = invoke_count.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(2));  // nyquist
  ASSERT_EQ(invoke_count_at_delete, invoke_count.load());
}

}  // namespace flutter

#define FML_USED_ON_EMBEDDER

#include <initializer_list>

#include "flutter/common/settings.h"
#include "flutter/common/task_runners.h"
#include "flutter/shell/common/switches.h"

#include "gtest/gtest.h"
#include "thread_host.h"
#include "vsync_waiter.h"

namespace flutter {
namespace testing {

class TestVsyncWaiter : public VsyncWaiter {
 public:
  explicit TestVsyncWaiter(const TaskRunners& task_runners)
      : VsyncWaiter(task_runners) {}

  int await_vsync_call_count_ = 0;

 protected:
  void AwaitVSync() override { await_vsync_call_count_++; }
};

TEST(VsyncWaiterTest, NoUnneededAwaitVsync) {
  using flutter::ThreadHost;
  std::string prefix = "vsync_waiter_test";

  fml::MessageLoop::EnsureInitializedForCurrentThread();
  auto task_runner = fml::MessageLoop::GetCurrent().GetTaskRunner();

  const flutter::TaskRunners task_runners(prefix, task_runner, task_runner,
                                          task_runner, task_runner);

  TestVsyncWaiter vsync_waiter(task_runners);

  vsync_waiter.ScheduleSecondaryCallback(1, [] {});
  EXPECT_EQ(vsync_waiter.await_vsync_call_count_, 1);

  vsync_waiter.ScheduleSecondaryCallback(2, [] {});
  EXPECT_EQ(vsync_waiter.await_vsync_call_count_, 1);
}

}  // namespace testing
}  // namespace flutter

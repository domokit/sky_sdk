// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/skia_gpu_object.h"

#include <future>
#include <utility>

#include "flutter/fml/message_loop.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/task_runner.h"
#include "flutter/testing/thread_test.h"
#include "gtest/gtest.h"
#include "third_party/skia/include/core/SkRefCnt.h"
#include "third_party/skia/include/gpu/ganesh/GrTypes.h"

namespace flutter {
namespace testing {

class TestSkObject : public SkRefCnt {
 public:
  TestSkObject(std::shared_ptr<fml::AutoResetWaitableEvent> latch,
               fml::TaskQueueId* dtor_task_queue_id)
      : latch_(std::move(latch)), dtor_task_queue_id_(dtor_task_queue_id) {}

  virtual ~TestSkObject() {
    if (dtor_task_queue_id_) {
      *dtor_task_queue_id_ = fml::MessageLoop::GetCurrentTaskQueueId();
    }
    latch_->Signal();
  }

 private:
  std::shared_ptr<fml::AutoResetWaitableEvent> latch_;
  fml::TaskQueueId* dtor_task_queue_id_;
};

class TestResourceContext : public TestSkObject {
 public:
  TestResourceContext(std::shared_ptr<fml::AutoResetWaitableEvent> latch,
                      fml::TaskQueueId* dtor_task_queue_id)
      : TestSkObject(std::move(latch), dtor_task_queue_id) {}
  ~TestResourceContext() = default;
  void performDeferredCleanup(std::chrono::milliseconds msNotUsed) {}
  void deleteBackendTexture(const GrBackendTexture& texture) {}
  void flushAndSubmit(GrSyncCpu sync) {}
};

class SkiaGpuObjectTest : public ThreadTest {
 public:
  SkiaGpuObjectTest()
      : unref_task_runner_(CreateNewThread()),
        unref_queue_(fml::MakeRefCounted<SkiaUnrefQueue>(
            unref_task_runner(),
            fml::TimeDelta::FromSeconds(0))),
        delayed_unref_queue_(fml::MakeRefCounted<SkiaUnrefQueue>(
            unref_task_runner(),
            fml::TimeDelta::FromSeconds(3))) {
    // The unref queues must be created in the same thread of the
    // unref_task_runner so the queue can access the same-thread-only WeakPtr of
    // the GrContext constructed during the creation.
    std::promise<bool> queues_created;
    unref_task_runner_->PostTask([this, &queues_created]() {
      unref_queue_ = fml::MakeRefCounted<SkiaUnrefQueue>(
          unref_task_runner(), fml::TimeDelta::FromSeconds(0));
      delayed_unref_queue_ = fml::MakeRefCounted<SkiaUnrefQueue>(
          unref_task_runner(), fml::TimeDelta::FromSeconds(3));
      queues_created.set_value(true);
    });
    queues_created.get_future().wait();
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  }

  fml::RefPtr<fml::TaskRunner> unref_task_runner() {
    return unref_task_runner_;
  }
  fml::RefPtr<SkiaUnrefQueue> unref_queue() { return unref_queue_; }
  fml::RefPtr<SkiaUnrefQueue> delayed_unref_queue() {
    return delayed_unref_queue_;
  }

 private:
  fml::RefPtr<fml::TaskRunner> unref_task_runner_;
  fml::RefPtr<SkiaUnrefQueue> unref_queue_;
  fml::RefPtr<SkiaUnrefQueue> delayed_unref_queue_;
};

TEST_F(SkiaGpuObjectTest, QueueSimple) {
  std::shared_ptr<fml::AutoResetWaitableEvent> latch =
      std::make_shared<fml::AutoResetWaitableEvent>();
  fml::TaskQueueId dtor_task_queue_id(0);
  SkRefCnt* ref_object = new TestSkObject(latch, &dtor_task_queue_id);

  unref_queue()->Unref(ref_object);
  latch->Wait();
  ASSERT_EQ(dtor_task_queue_id, unref_task_runner()->GetTaskQueueId());
}

TEST_F(SkiaGpuObjectTest, ObjectDestructor) {
  std::shared_ptr<fml::AutoResetWaitableEvent> latch =
      std::make_shared<fml::AutoResetWaitableEvent>();
  fml::TaskQueueId dtor_task_queue_id(0);
  auto object = sk_make_sp<TestSkObject>(latch, &dtor_task_queue_id);
  {
    SkiaGPUObject<TestSkObject> sk_object(std::move(object), unref_queue());
    // Verify that the default SkiaGPUObject dtor queues and unref.
  }

  latch->Wait();
  ASSERT_EQ(dtor_task_queue_id, unref_task_runner()->GetTaskQueueId());
}

TEST_F(SkiaGpuObjectTest, ObjectReset) {
  std::shared_ptr<fml::AutoResetWaitableEvent> latch =
      std::make_shared<fml::AutoResetWaitableEvent>();
  fml::TaskQueueId dtor_task_queue_id(0);
  SkiaGPUObject<TestSkObject> sk_object(
      sk_make_sp<TestSkObject>(latch, &dtor_task_queue_id), unref_queue());
  // Verify that explicitly resetting the GPU object queues and unref.
  sk_object.reset();
  ASSERT_EQ(sk_object.skia_object(), nullptr);
  latch->Wait();
  ASSERT_EQ(dtor_task_queue_id, unref_task_runner()->GetTaskQueueId());
}

TEST_F(SkiaGpuObjectTest, ObjectResetTwice) {
  std::shared_ptr<fml::AutoResetWaitableEvent> latch =
      std::make_shared<fml::AutoResetWaitableEvent>();
  fml::TaskQueueId dtor_task_queue_id(0);
  SkiaGPUObject<TestSkObject> sk_object(
      sk_make_sp<TestSkObject>(latch, &dtor_task_queue_id), unref_queue());

  sk_object.reset();
  ASSERT_EQ(sk_object.skia_object(), nullptr);
  sk_object.reset();
  ASSERT_EQ(sk_object.skia_object(), nullptr);

  latch->Wait();
  ASSERT_EQ(dtor_task_queue_id, unref_task_runner()->GetTaskQueueId());
}

TEST_F(SkiaGpuObjectTest, UnrefResourceContextInTaskRunnerThread) {
  std::shared_ptr<fml::AutoResetWaitableEvent> latch =
      std::make_shared<fml::AutoResetWaitableEvent>();
  fml::RefPtr<UnrefQueue<TestResourceContext>> unref_queue;
  fml::TaskQueueId dtor_task_queue_id(0);
  unref_task_runner()->PostTask([&]() {
    auto resource_context =
        sk_make_sp<TestResourceContext>(latch, &dtor_task_queue_id);
    unref_queue = fml::MakeRefCounted<UnrefQueue<TestResourceContext>>(
        unref_task_runner(), fml::TimeDelta::FromSeconds(0),
        std::move(resource_context));
    latch->Signal();
  });
  latch->Wait();

  // Delete the unref queue, it will schedule a task to unref the resource
  // context in the task runner's thread.
  unref_queue = nullptr;
  latch->Wait();
  // Verify that the resource context was destroyed in the task runner's thread.
  ASSERT_EQ(dtor_task_queue_id, unref_task_runner()->GetTaskQueueId());
}

}  // namespace testing
}  // namespace flutter

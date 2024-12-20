// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <algorithm>
#include <memory>
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/testing/testing.h"  // IWYU pragma: keep
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "impeller/renderer/backend/gles/handle_gles.h"
#include "impeller/renderer/backend/gles/proc_table_gles.h"
#include "impeller/renderer/backend/gles/reactor_gles.h"
#include "impeller/renderer/backend/gles/test/mock_gles.h"

namespace impeller {
namespace testing {

using ::testing::_;

class TestWorker : public ReactorGLES::Worker {
 public:
  bool CanReactorReactOnCurrentThreadNow(
      const ReactorGLES& reactor) const override {
    return true;
  }
};

TEST(ReactorGLES, CanAttachCleanupCallbacksToHandles) {
  auto mock_gles = MockGLES::Init();
  ProcTableGLES::Resolver resolver = kMockResolverGLES;
  auto proc_table = std::make_unique<ProcTableGLES>(resolver);
  auto worker = std::make_shared<TestWorker>();
  auto reactor = std::make_shared<ReactorGLES>(std::move(proc_table));
  reactor->AddWorker(worker);

  int value = 0;
  auto handle = reactor->CreateHandle(HandleType::kTexture, 1123);
  auto added =
      reactor->RegisterCleanupCallback(handle, [&value]() { value++; });

  EXPECT_TRUE(added);
  EXPECT_TRUE(reactor->React());

  reactor->CollectHandle(handle);
  EXPECT_TRUE(reactor->AddOperation([](const ReactorGLES& reactor) {}));
  EXPECT_TRUE(reactor->React());
  EXPECT_EQ(value, 1);
}

TEST(ReactorGLES, DeletesHandlesDuringShutdown) {
  auto mock_gles_impl = std::make_unique<MockGLESImpl>();

  EXPECT_CALL(*mock_gles_impl, GenTextures(1, _))
      .WillOnce([](GLsizei size, GLuint* queries) { queries[0] = 1234; });
  EXPECT_CALL(*mock_gles_impl, DeleteTextures(1, ::testing::Pointee(1234)))
      .Times(1);

  std::shared_ptr<MockGLES> mock_gles =
      MockGLES::Init(std::move(mock_gles_impl));
  ProcTableGLES::Resolver resolver = kMockResolverGLES;
  auto proc_table = std::make_unique<ProcTableGLES>(resolver);
  auto worker = std::make_shared<TestWorker>();
  auto reactor = std::make_shared<ReactorGLES>(std::move(proc_table));
  reactor->AddWorker(worker);
  reactor->CreateHandle(HandleType::kTexture);
  reactor.reset();
}

TEST(ReactorGLES, UntrackedHandle) {
  auto mock_gles_impl = std::make_unique<MockGLESImpl>();

  EXPECT_CALL(*mock_gles_impl, GenTextures(1, _))
      .WillOnce([](GLsizei size, GLuint* queries) { queries[0] = 1234; });
  EXPECT_CALL(*mock_gles_impl, DeleteTextures(1, ::testing::Pointee(1234)))
      .Times(1);

  std::shared_ptr<MockGLES> mock_gles =
      MockGLES::Init(std::move(mock_gles_impl));
  ProcTableGLES::Resolver resolver = kMockResolverGLES;
  auto proc_table = std::make_unique<ProcTableGLES>(resolver);
  auto worker = std::make_shared<TestWorker>();
  auto reactor = std::make_shared<ReactorGLES>(std::move(proc_table));
  reactor->AddWorker(worker);

  HandleGLES handle = reactor->CreateUntrackedHandle(HandleType::kTexture);
  EXPECT_FALSE(handle.IsDead());
  std::optional<GLuint> glint = reactor->GetGLHandle(handle);
  EXPECT_TRUE(glint.has_value());
  if (glint.has_value()) {
    EXPECT_EQ(1234u, *glint);
  }
  reactor->CollectHandle(handle);
  EXPECT_TRUE(reactor->AddOperation([&](const ReactorGLES&) {}));
  EXPECT_TRUE(reactor->React());
}

TEST(ReactorGLES, NameUntrackedHandle) {
  auto mock_gles_impl = std::make_unique<MockGLESImpl>();

  EXPECT_CALL(*mock_gles_impl, GenTextures(1, _))
      .WillOnce([](GLsizei size, GLuint* queries) { queries[0] = 1234; });
  EXPECT_CALL(*mock_gles_impl,
              ObjectLabelKHR(_, 1234, _, ::testing::StrEq("hello, joe!")))
      .Times(1);

  std::shared_ptr<MockGLES> mock_gles =
      MockGLES::Init(std::move(mock_gles_impl));
  ProcTableGLES::Resolver resolver = kMockResolverGLES;
  auto proc_table = std::make_unique<ProcTableGLES>(resolver);

  if (!proc_table->SupportsDebugLabels()) {
    GTEST_SKIP() << "This device doesn't support labelling.";
  }

  auto worker = std::make_shared<TestWorker>();
  auto reactor = std::make_shared<ReactorGLES>(std::move(proc_table));
  reactor->AddWorker(worker);

  HandleGLES handle = reactor->CreateUntrackedHandle(HandleType::kTexture);
  reactor->SetDebugLabel(handle, "hello, joe!");
}

TEST(ReactorGLES, PerThreadOperationQueues) {
  auto mock_gles = MockGLES::Init();
  ProcTableGLES::Resolver resolver = kMockResolverGLES;
  auto proc_table = std::make_unique<ProcTableGLES>(resolver);
  auto worker = std::make_shared<TestWorker>();
  auto reactor = std::make_shared<ReactorGLES>(std::move(proc_table));
  reactor->AddWorker(worker);

  bool op1_called = false;
  EXPECT_TRUE(
      reactor->AddOperation([&](const ReactorGLES&) { op1_called = true; }));

  fml::AutoResetWaitableEvent event;
  bool op2_called = false;
  std::thread thread([&] {
    EXPECT_TRUE(reactor->AddOperation(
        [&](const ReactorGLES&) { op2_called = true; }, true));
    event.Wait();
    EXPECT_TRUE(reactor->React());
  });

  // Reacting on the main thread should only run the main thread's operation.
  EXPECT_TRUE(reactor->React());
  EXPECT_TRUE(op1_called);
  EXPECT_FALSE(op2_called);

  // Reacting on the second thread will run the second thread's operation.
  event.Signal();
  thread.join();
  EXPECT_TRUE(op2_called);
}

TEST(ReactorGLES, CanDeferOperations) {
  auto mock_gles = MockGLES::Init();
  ProcTableGLES::Resolver resolver = kMockResolverGLES;
  auto proc_table = std::make_unique<ProcTableGLES>(resolver);
  auto worker = std::make_shared<TestWorker>();
  auto reactor = std::make_shared<ReactorGLES>(std::move(proc_table));
  reactor->AddWorker(worker);

  // Add operation executes tasks as long as the reactor can run tasks on
  // the current thread.
  bool did_run = false;
  EXPECT_TRUE(
      reactor->AddOperation([&](const ReactorGLES&) { did_run = true; }));
  EXPECT_TRUE(did_run);

  //...unless defer=true is specified, which only enqueues in the reactor.
  did_run = false;
  EXPECT_TRUE(reactor->AddOperation([&](const ReactorGLES&) { did_run = true; },
                                    /*defer=*/true));
  EXPECT_FALSE(did_run);
  EXPECT_TRUE(reactor->React());
  EXPECT_TRUE(did_run);
}

}  // namespace testing
}  // namespace impeller

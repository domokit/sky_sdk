// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"  // IWYU pragma: keep.
#include "fml/synchronization/waitable_event.h"
#include "impeller/renderer/backend/vulkan/command_pool_vk.h"
#include "impeller/renderer/backend/vulkan/resource_manager_vk.h"
#include "impeller/renderer/backend/vulkan/test/mock_vulkan.h"

namespace impeller {
namespace testing {

TEST(CommandPoolRecyclerVKTest, GetsACommandPoolPerThread) {
  auto const context = MockVulkanContextBuilder().Build();

  {
    // Record the memory location of each pointer to a command pool.
    //
    // These pools have to be held at this context, otherwise they will be
    // dropped and recycled and potentially reused by another thread, causing
    // flaky tests.
    std::shared_ptr<CommandPoolVK> pool1;
    std::shared_ptr<CommandPoolVK> pool2;

    // Create a command pool in two threads and record the memory location.
    std::thread thread1(
        [&]() { pool1 = context->GetCommandPoolRecycler()->Get(); });

    std::thread thread2(
        [&]() { pool2 = context->GetCommandPoolRecycler()->Get(); });

    thread1.join();
    thread2.join();

    // The two command pools should be different.
    EXPECT_NE(pool1, pool2);
  }

  context->Shutdown();
}

TEST(CommandPoolRecyclerVKTest, GetsTheSameCommandPoolOnSameThread) {
  auto const context = MockVulkanContextBuilder().Build();

  auto const pool1 = context->GetCommandPoolRecycler()->Get();
  auto const pool2 = context->GetCommandPoolRecycler()->Get();

  // The two command pools should be the same.
  EXPECT_EQ(pool1.get(), pool2.get());

  context->Shutdown();
}

namespace {

// Invokes the provided callback when the destructor is called.
//
// Can be moved, but not copied.
class DeathRattle final {
 public:
  explicit DeathRattle(std::function<void()> callback)
      : callback_(std::move(callback)) {}

  DeathRattle(DeathRattle&&) = default;
  DeathRattle& operator=(DeathRattle&&) = default;

  ~DeathRattle() { callback_(); }

 private:
  std::function<void()> callback_;
};

}  // namespace

TEST(CommandPoolRecyclerVKTest, ReclaimMakesCommandPoolAvailable) {
  auto const context = MockVulkanContextBuilder().Build();

  {
    // Fetch a pool (which will be created).
    auto const recycler = context->GetCommandPoolRecycler();
    auto const pool = recycler->Get();

    // This normally is called at the end of a frame.
    recycler->Dispose();
  }

  // Add something to the resource manager and have it notify us when it's
  // destroyed. That should give us a non-flaky signal that the pool has been
  // reclaimed as well.
  auto waiter = fml::AutoResetWaitableEvent();
  auto rattle = DeathRattle([&waiter]() { waiter.Signal(); });
  {
    UniqueResourceVKT<DeathRattle> resource(context->GetResourceManager(),
                                            std::move(rattle));
  }
  waiter.Wait();

  // On another thread explicitly, request a new pool.
  std::thread thread([&]() {
    auto const pool = context->GetCommandPoolRecycler()->Get();
    EXPECT_NE(pool.get(), nullptr);
  });

  thread.join();

  // Now check that we only ever created one pool.
  auto const called = GetMockVulkanFunctions(context->GetDevice());
  EXPECT_EQ(std::count(called->begin(), called->end(), "vkCreateCommandPool"),
            1u);

  context->Shutdown();
}

TEST(CommandPoolRecyclerVKTest, CommandBuffersAreRecycled) {
  auto const context = MockVulkanContextBuilder().Build();

  {
    // Fetch a pool (which will be created).
    auto const recycler = context->GetCommandPoolRecycler();
    auto pool = recycler->Get();

    auto buffer = pool->CreateCommandBuffer();
    pool->CollectCommandBuffer(std::move(buffer));

    // This normally is called at the end of a frame.
    recycler->Dispose();
  }

  // Wait for the pool to be reclaimed.
  for (auto i = 0u; i < 2u; i++) {
    auto waiter = fml::AutoResetWaitableEvent();
    auto rattle = DeathRattle([&waiter]() { waiter.Signal(); });
    {
      UniqueResourceVKT<DeathRattle> resource(context->GetResourceManager(),
                                              std::move(rattle));
    }
    waiter.Wait();
  }

  {
    // Create a second pool and command buffer, which should reused the existing
    // pool and cmd buffer.
    auto const recycler = context->GetCommandPoolRecycler();
    auto pool = recycler->Get();

    auto buffer = pool->CreateCommandBuffer();
    pool->CollectCommandBuffer(std::move(buffer));

    // This normally is called at the end of a frame.
    recycler->Dispose();
  }

  // Now check that we only ever created one pool and one command buffer.
  auto const called = GetMockVulkanFunctions(context->GetDevice());
  EXPECT_EQ(std::count(called->begin(), called->end(), "vkCreateCommandPool"),
            1u);
  EXPECT_EQ(
      std::count(called->begin(), called->end(), "vkAllocateCommandBuffers"),
      1u);

  context->Shutdown();
}

TEST(CommandPoolRecyclerVKTest, ExtraCommandBufferAllocationsTriggerTrim) {
  auto const context = MockVulkanContextBuilder().Build();

  {
    // Fetch a pool (which will be created).
    auto const recycler = context->GetCommandPoolRecycler();
    auto pool = recycler->Get();

    // Allocate a large number of command buffers
    for (auto i = 0; i < 64; i++) {
      auto buffer = pool->CreateCommandBuffer();
      pool->CollectCommandBuffer(std::move(buffer));
    }

    // This normally is called at the end of a frame.
    recycler->Dispose();
  }

  // Wait for the pool to be reclaimed.
  for (auto i = 0u; i < 2u; i++) {
    auto waiter = fml::AutoResetWaitableEvent();
    auto rattle = DeathRattle([&waiter]() { waiter.Signal(); });
    {
      UniqueResourceVKT<DeathRattle> resource(context->GetResourceManager(),
                                              std::move(rattle));
    }
    waiter.Wait();
  }

  // Command pool is reset but does not release resources.
  auto called = GetMockVulkanFunctions(context->GetDevice());
  EXPECT_EQ(std::count(called->begin(), called->end(), "vkResetCommandPool"),
            1u);

  // Create the pool a second time, but dont use any command buffers.
  {
    // Fetch a pool (which will be created).
    auto const recycler = context->GetCommandPoolRecycler();
    auto pool = recycler->Get();

    // This normally is called at the end of a frame.
    recycler->Dispose();
  }

  // Wait for the pool to be reclaimed.
  for (auto i = 0u; i < 2u; i++) {
    auto waiter = fml::AutoResetWaitableEvent();
    auto rattle = DeathRattle([&waiter]() { waiter.Signal(); });
    {
      UniqueResourceVKT<DeathRattle> resource(context->GetResourceManager(),
                                              std::move(rattle));
    }
    waiter.Wait();
  }

  // Verify that the cmd pool was trimmed.

  // Now check that we only ever created one pool and one command buffer.
  called = GetMockVulkanFunctions(context->GetDevice());
  EXPECT_EQ(std::count(called->begin(), called->end(),
                       "vkResetCommandPoolReleaseResources"),
            1u);

  context->Shutdown();
}

}  // namespace testing
}  // namespace impeller

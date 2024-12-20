// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/testing/testing.h"
#include "impeller/toolkit/android/choreographer.h"
#include "impeller/toolkit/android/hardware_buffer.h"
#include "impeller/toolkit/android/proc_table.h"
#include "impeller/toolkit/android/shadow_realm.h"
#include "impeller/toolkit/android/surface_control.h"
#include "impeller/toolkit/android/surface_transaction.h"

namespace impeller::android::testing {

#define DISABLE_ANDROID_PROC(name)                 \
  struct Disable##name {                           \
    Disable##name() {                              \
      real_proc = GetMutableProcTable().name.proc; \
      GetMutableProcTable().name.proc = nullptr;   \
    }                                              \
    ~Disable##name() {                             \
      GetMutableProcTable().name.proc = real_proc; \
    }                                              \
    decltype(name)* real_proc;                     \
  } disable##name;

TEST(ToolkitAndroidTest, CanCreateProcTable) {
  ProcTable proc_table;
  ASSERT_TRUE(proc_table.IsValid());
}

TEST(ToolkitAndroidTest, GuardsAgainstZeroSizedDescriptors) {
  auto desc = HardwareBufferDescriptor::MakeForSwapchainImage({0, 0});
  ASSERT_GT(desc.size.width, 0u);
  ASSERT_GT(desc.size.height, 0u);
}

TEST(ToolkitAndroidTest, CanCreateHardwareBuffer) {
  if (!HardwareBuffer::IsAvailableOnPlatform()) {
    GTEST_SKIP() << "Hardware buffers are not supported on this platform.";
  }
  ASSERT_TRUE(HardwareBuffer::IsAvailableOnPlatform());
  auto desc = HardwareBufferDescriptor::MakeForSwapchainImage({100, 100});
  ASSERT_TRUE(desc.IsAllocatable());
  HardwareBuffer buffer(desc);
  ASSERT_TRUE(buffer.IsValid());
}

TEST(ToolkitAndroidTest, CanGetHardwareBufferIDs) {
  if (!HardwareBuffer::IsAvailableOnPlatform()) {
    GTEST_SKIP() << "Hardware buffers are not supported on this platform.";
  }
  ASSERT_TRUE(HardwareBuffer::IsAvailableOnPlatform());
  if (!GetProcTable().AHardwareBuffer_getId.IsAvailable()) {
    GTEST_SKIP() << "Hardware buffer IDs are not available on this platform.";
  }
  auto desc = HardwareBufferDescriptor::MakeForSwapchainImage({100, 100});
  ASSERT_TRUE(desc.IsAllocatable());
  HardwareBuffer buffer(desc);
  ASSERT_TRUE(buffer.IsValid());
  ASSERT_TRUE(buffer.GetSystemUniqueID().has_value());
}

TEST(ToolkitAndroidTest, HardwareBufferNullIDIfAPIUnavailable) {
  DISABLE_ANDROID_PROC(AHardwareBuffer_getId);
  ASSERT_FALSE(HardwareBuffer::GetSystemUniqueID(nullptr).has_value());
}

TEST(ToolkitAndroidTest, CanDescribeHardwareBufferHandles) {
  if (!HardwareBuffer::IsAvailableOnPlatform()) {
    GTEST_SKIP() << "Hardware buffers are not supported on this platform.";
  }
  ASSERT_TRUE(HardwareBuffer::IsAvailableOnPlatform());
  auto desc = HardwareBufferDescriptor::MakeForSwapchainImage({100, 100});
  ASSERT_TRUE(desc.IsAllocatable());
  HardwareBuffer buffer(desc);
  ASSERT_TRUE(buffer.IsValid());
  auto a_desc = HardwareBuffer::Describe(buffer.GetHandle());
  ASSERT_TRUE(a_desc.has_value());
  ASSERT_EQ(a_desc->width, 100u);   // NOLINT
  ASSERT_EQ(a_desc->height, 100u);  // NOLINT
}

TEST(ToolkitAndroidTest, CanApplySurfaceTransaction) {
  if (!SurfaceTransaction::IsAvailableOnPlatform()) {
    GTEST_SKIP() << "Surface controls are not supported on this platform.";
  }
  ASSERT_TRUE(SurfaceTransaction::IsAvailableOnPlatform());
  SurfaceTransaction transaction;
  ASSERT_TRUE(transaction.IsValid());
  fml::AutoResetWaitableEvent event;
  ASSERT_TRUE(transaction.Apply([&event](auto) { event.Signal(); }));
  event.Wait();
}

TEST(ToolkitAndroidTest, SurfacControlsAreAvailable) {
  if (!SurfaceControl::IsAvailableOnPlatform()) {
    GTEST_SKIP() << "Surface controls are not supported on this platform.";
  }
  ASSERT_TRUE(SurfaceControl::IsAvailableOnPlatform());
}

TEST(ToolkitAndroidTest, ChoreographerIsAvailable) {
  if (!Choreographer::IsAvailableOnPlatform()) {
    GTEST_SKIP() << "Choreographer is not supported on this platform.";
  }
  ASSERT_TRUE(Choreographer::IsAvailableOnPlatform());
}

TEST(ToolkitAndroidTest, CanPostAndNotWaitForFrameCallbacks) {
  if (!Choreographer::IsAvailableOnPlatform()) {
    GTEST_SKIP() << "Choreographer is not supported on this platform.";
  }
  const auto& choreographer = Choreographer::GetInstance();
  ASSERT_TRUE(choreographer.IsValid());
  ASSERT_TRUE(choreographer.PostFrameCallback([](auto) {}));
}

TEST(ToolkitAndroidTest, CanPostAndWaitForFrameCallbacks) {
  if (!Choreographer::IsAvailableOnPlatform()) {
    GTEST_SKIP() << "Choreographer is not supported on this platform.";
  }
  if ((true)) {
    GTEST_SKIP()
        << "Disabled till the test harness is in an Android activity. "
           "Running it without one will hang because the choreographer "
           "frame callback will never execute.";
  }
  const auto& choreographer = Choreographer::GetInstance();
  ASSERT_TRUE(choreographer.IsValid());
  fml::AutoResetWaitableEvent event;
  ASSERT_TRUE(choreographer.PostFrameCallback(
      [&event](auto point) { event.Signal(); }));
  event.Wait();
}

TEST(ToolkitAndroidTest, ShouldDisableAHB) {
  EXPECT_FALSE(
      ShadowRealm::ShouldDisableAHBInternal("android-huawei", "30", 30));
  EXPECT_FALSE(
      ShadowRealm::ShouldDisableAHBInternal("something made up", "29", 29));

  EXPECT_TRUE(
      ShadowRealm::ShouldDisableAHBInternal("android-huawei", "29", 29));
  EXPECT_TRUE(
      ShadowRealm::ShouldDisableAHBInternal("something made up", "27", 29));
  EXPECT_TRUE(
      ShadowRealm::ShouldDisableAHBInternal("android-huawei", "garbage", 29));
}

}  // namespace impeller::android::testing

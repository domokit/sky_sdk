// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"  // IWYU pragma: keep
#include "gtest/gtest.h"
#include "impeller/renderer/backend/gles/device_buffer_gles.h"
#include "impeller/renderer/backend/gles/test/mock_gles.h"

namespace impeller {
namespace testing {

using ::testing::_;

namespace {
class TestWorker : public ReactorGLES::Worker {
 public:
  bool CanReactorReactOnCurrentThreadNow(
      const ReactorGLES& reactor) const override {
    return true;
  }
};
}  // namespace

TEST(DeviceBufferGLESTest, BindUniformData) {
  auto mock_gles_impl = std::make_unique<MockGLESImpl>();

  EXPECT_CALL(*mock_gles_impl, GenBuffers(1, _)).Times(1);

  std::shared_ptr<MockGLES> mock_gled =
      MockGLES::Init(std::move(mock_gles_impl));
  ProcTableGLES::Resolver resolver = kMockResolverGLES;
  auto proc_table = std::make_unique<ProcTableGLES>(resolver);
  auto worker = std::make_shared<TestWorker>();
  auto reactor = std::make_shared<ReactorGLES>(std::move(proc_table));
  reactor->AddWorker(worker);

  std::shared_ptr<Allocation> backing_store = std::make_shared<Allocation>();
  ASSERT_TRUE(backing_store->Truncate(Bytes{sizeof(float)}));
  DeviceBufferGLES device_buffer(DeviceBufferDescriptor{.size = sizeof(float)},
                                 reactor, backing_store);
  EXPECT_FALSE(device_buffer.GetHandle().has_value());
  EXPECT_TRUE(device_buffer.BindAndUploadDataIfNecessary(
      DeviceBufferGLES::BindingType::kUniformBuffer));
  EXPECT_TRUE(device_buffer.GetHandle().has_value());
}

}  // namespace testing
}  // namespace impeller

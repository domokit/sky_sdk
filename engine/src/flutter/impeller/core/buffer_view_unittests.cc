// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"
#include "impeller/core/buffer_view.h"

namespace impeller {
namespace testing {

TEST(BufferViewTest, Empty) {
  BufferView buffer_view;
  EXPECT_FALSE(buffer_view);
}

TEST(BufferViewTest, TakeRaw) {
  DeviceBuffer* buffer = reinterpret_cast<DeviceBuffer*>(0xcafebabe);
  BufferView buffer_view(buffer, {0, 123});
  EXPECT_TRUE(buffer_view);
  std::shared_ptr<const DeviceBuffer> taken = buffer_view.TakeBuffer();
  EXPECT_FALSE(taken);
  EXPECT_EQ(buffer_view.GetBuffer(), buffer);
}

}  // namespace testing
}  // namespace impeller

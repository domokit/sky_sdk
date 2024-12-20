// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/dart_service_isolate.h"

#include "flutter/testing/testing.h"

namespace flutter {

TEST(DartServiceIsolateTest, CanAddAndRemoveHandles) {
  ASSERT_EQ(DartServiceIsolate::AddServerStatusCallback(nullptr), 0);
  auto handle = DartServiceIsolate::AddServerStatusCallback([](const auto&) {});
  ASSERT_NE(handle, 0);
  ASSERT_TRUE(DartServiceIsolate::RemoveServerStatusCallback(handle));
}

}  // namespace flutter

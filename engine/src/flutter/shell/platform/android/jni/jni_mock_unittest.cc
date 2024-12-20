// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/jni/jni_mock.h"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(JNIMock, FlutterViewHandlePlatformMessage) {
  JNIMock mock;

  auto message = std::make_unique<PlatformMessage>("<channel-name>", nullptr);
  auto response_id = 1;

  EXPECT_CALL(mock,
              FlutterViewHandlePlatformMessage(
                  ::testing::Property(&std::unique_ptr<PlatformMessage>::get,
                                      message.get()),
                  response_id));

  mock.FlutterViewHandlePlatformMessage(std::move(message), response_id);
}

}  // namespace testing
}  // namespace flutter

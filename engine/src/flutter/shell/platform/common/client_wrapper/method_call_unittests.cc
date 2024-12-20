// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_call.h"

#include <memory>
#include <string>

#include "gtest/gtest.h"

namespace flutter {

TEST(MethodCallTest, Basic) {
  const std::string method_name("method_name");
  const int argument = 42;
  MethodCall<int> method_call(method_name, std::make_unique<int>(argument));
  EXPECT_EQ(method_call.method_name(), method_name);
  ASSERT_NE(method_call.arguments(), nullptr);
  EXPECT_EQ(*method_call.arguments(), 42);
}

}  // namespace flutter

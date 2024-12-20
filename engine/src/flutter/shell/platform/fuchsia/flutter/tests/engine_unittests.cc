// Copyright 2020 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/fuchsia/flutter/engine.h"

#include "flutter/shell/common/thread_host.h"

#include "gtest/gtest.h"

using namespace flutter;

namespace flutter_runner {
namespace testing {
namespace {

std::string GetCurrentTestName() {
  return ::testing::UnitTest::GetInstance()->current_test_info()->name();
}

}  // namespace

TEST(EngineTest, ThreadNames) {
  std::string prefix = GetCurrentTestName();
  flutter::ThreadHost engine_thread_host = Engine::CreateThreadHost(prefix);

  char thread_name[ZX_MAX_NAME_LEN];
  zx::thread::self()->get_property(ZX_PROP_NAME, thread_name,
                                   sizeof(thread_name));
  EXPECT_EQ(std::string(thread_name), prefix + std::string(".platform"));
  EXPECT_EQ(engine_thread_host.platform_thread, nullptr);

  engine_thread_host.raster_thread->GetTaskRunner()->PostTask([&prefix]() {
    char thread_name[ZX_MAX_NAME_LEN];
    zx::thread::self()->get_property(ZX_PROP_NAME, thread_name,
                                     sizeof(thread_name));
    EXPECT_EQ(std::string(thread_name), prefix + std::string(".raster"));
  });
  engine_thread_host.raster_thread->Join();

  engine_thread_host.ui_thread->GetTaskRunner()->PostTask([&prefix]() {
    char thread_name[ZX_MAX_NAME_LEN];
    zx::thread::self()->get_property(ZX_PROP_NAME, thread_name,
                                     sizeof(thread_name));
    EXPECT_EQ(std::string(thread_name), prefix + std::string(".ui"));
  });
  engine_thread_host.ui_thread->Join();

  engine_thread_host.io_thread->GetTaskRunner()->PostTask([&prefix]() {
    char thread_name[ZX_MAX_NAME_LEN];
    zx::thread::self()->get_property(ZX_PROP_NAME, thread_name,
                                     sizeof(thread_name));
    EXPECT_EQ(std::string(thread_name), prefix + std::string(".io"));
  });
  engine_thread_host.io_thread->Join();
}

}  // namespace testing
}  // namespace flutter_runner

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include "flutter/testing/thread_test.h"

namespace flutter::testing {

namespace {

fml::RefPtr<fml::TaskRunner> GetDefaultTaskRunner() {
  fml::MessageLoop::EnsureInitializedForCurrentThread();
  return fml::MessageLoop::GetCurrent().GetTaskRunner();
}

}  // namespace

ThreadTest::ThreadTest() : current_task_runner_(GetDefaultTaskRunner()) {}

fml::RefPtr<fml::TaskRunner> ThreadTest::GetCurrentTaskRunner() {
  return current_task_runner_;
}

fml::RefPtr<fml::TaskRunner> ThreadTest::CreateNewThread(
    const std::string& name) {
  auto thread = std::make_unique<fml::Thread>(name);
  auto runner = thread->GetTaskRunner();
  extra_threads_.emplace_back(std::move(thread));
  return runner;
}

}  // namespace flutter::testing

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_TEST_TIMEOUT_LISTENER_H_
#define FLUTTER_TESTING_TEST_TIMEOUT_LISTENER_H_

#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/fml/task_runner.h"
#include "flutter/fml/thread.h"
#include "flutter/testing/testing.h"

namespace flutter::testing {

class PendingTests;

class TestTimeoutListener : public ::testing::EmptyTestEventListener {
 public:
  explicit TestTimeoutListener(fml::TimeDelta timeout);

  ~TestTimeoutListener();

 private:
  const fml::TimeDelta timeout_;
  fml::Thread listener_thread_;
  fml::RefPtr<fml::TaskRunner> listener_thread_runner_;
  std::shared_ptr<PendingTests> pending_tests_;

  // |testing::EmptyTestEventListener|
  void OnTestStart(const ::testing::TestInfo& test_info) override;

  // |testing::EmptyTestEventListener|
  void OnTestEnd(const ::testing::TestInfo& test_info) override;

  FML_DISALLOW_COPY_AND_ASSIGN(TestTimeoutListener);
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_TEST_TIMEOUT_LISTENER_H_

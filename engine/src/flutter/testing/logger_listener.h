// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_LOGGER_LISTENER_H_
#define FLUTTER_TESTING_LOGGER_LISTENER_H_

#include "flutter/fml/logging.h"
#include "flutter/testing/testing.h"

namespace flutter::testing {

class LoggerListener : public ::testing::EmptyTestEventListener {
 public:
  LoggerListener();

  ~LoggerListener();

  LoggerListener(const LoggerListener&) = delete;

  LoggerListener& operator=(const LoggerListener&) = delete;

  // |testing::EmptyTestEventListener|
  void OnTestStart(const ::testing::TestInfo& test_info) override;

  // |testing::EmptyTestEventListener|
  void OnTestEnd(const ::testing::TestInfo& test_info) override;

  // |testing::EmptyTestEventListener|
  void OnTestDisabled(const ::testing::TestInfo& test_info) override;
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_LOGGER_LISTENER_H_

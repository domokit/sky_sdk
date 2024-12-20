// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/logger_listener.h"

namespace flutter::testing {

LoggerListener::LoggerListener() = default;

LoggerListener::~LoggerListener() = default;

void testing::LoggerListener::OnTestStart(
    const ::testing::TestInfo& test_info) {
  FML_LOG(IMPORTANT) << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>";
  FML_LOG(IMPORTANT) << "Starting Test: " << test_info.test_suite_name() << ":"
                     << test_info.name();
}

std::string TestStatusAsString(const ::testing::TestResult* result) {
  if (result == nullptr) {
    return "UNKNOWN";
  }
  if (result->Passed()) {
    return "PASSED";
  }
  if (result->Skipped()) {
    return "SKIPPED";
  }
  if (result->Failed()) {
    return "FAILED";
  }
  return "UNKNOWN";
}

std::string TestLabel(const ::testing::TestInfo& info) {
  return std::string{info.test_suite_name()} + "." + info.name();
}

std::string TestTimeAsString(const ::testing::TestResult* result) {
  if (result == nullptr) {
    return "UNKNOWN";
  }
  return std::to_string(result->elapsed_time()) + " ms";
}

void testing::LoggerListener::OnTestEnd(const ::testing::TestInfo& info) {
  FML_LOG(IMPORTANT) << "Test " << TestStatusAsString(info.result()) << " ("
                     << TestTimeAsString(info.result())
                     << "): " << TestLabel(info);
  FML_LOG(IMPORTANT) << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<";
}

void testing::LoggerListener::OnTestDisabled(const ::testing::TestInfo& info) {
  FML_LOG(IMPORTANT) << "Test Disabled: " << TestLabel(info);
}

}  // namespace flutter::testing

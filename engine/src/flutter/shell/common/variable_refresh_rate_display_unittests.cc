// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "variable_refresh_rate_display.h"
#include "vsync_waiters_test.h"

#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(VariableRefreshRateDisplayTest, ReportCorrectInitialRefreshRate) {
  auto refresh_rate_reporter = std::make_shared<TestRefreshRateReporter>(60);
  auto display = flutter::VariableRefreshRateDisplay(
      0, std::weak_ptr<TestRefreshRateReporter>(refresh_rate_reporter), 600,
      800, 60);
  ASSERT_EQ(display.GetRefreshRate(), 60);
}

TEST(VariableRefreshRateDisplayTest, ReportCorrectRefreshRateWhenUpdated) {
  auto refresh_rate_reporter = std::make_shared<TestRefreshRateReporter>(60);
  auto display = flutter::VariableRefreshRateDisplay(
      0, std::weak_ptr<TestRefreshRateReporter>(refresh_rate_reporter), 600,
      800, 60);
  refresh_rate_reporter->UpdateRefreshRate(30);
  ASSERT_EQ(display.GetRefreshRate(), 30);
}

TEST(VariableRefreshRateDisplayTest,
     Report0IfReporterSharedPointerIsDestroyedAfterDisplayCreation) {
  auto refresh_rate_reporter = std::make_shared<TestRefreshRateReporter>(60);
  auto display = flutter::VariableRefreshRateDisplay(
      0, std::weak_ptr<TestRefreshRateReporter>(refresh_rate_reporter), 600,
      800, 60);
  refresh_rate_reporter.reset();
  ASSERT_EQ(display.GetRefreshRate(), 0);
}

TEST(VariableRefreshRateDisplayTest,
     Report0IfReporterSharedPointerIsDestroyedBeforeDisplayCreation) {
  auto refresh_rate_reporter = std::make_shared<TestRefreshRateReporter>(60);
  refresh_rate_reporter.reset();
  auto display = flutter::VariableRefreshRateDisplay(
      0, std::weak_ptr<TestRefreshRateReporter>(refresh_rate_reporter), 600,
      800, 60);
  ASSERT_EQ(display.GetRefreshRate(), 0);
}

}  // namespace testing
}  // namespace flutter

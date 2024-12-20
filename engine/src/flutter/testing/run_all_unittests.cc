// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <iostream>
#include <optional>
#include <string>

#include "flutter/fml/backtrace.h"
#include "flutter/fml/build_config.h"
#include "flutter/fml/command_line.h"
#include "flutter/testing/debugger_detection.h"
#include "flutter/testing/test_args.h"
#include "flutter/testing/test_timeout_listener.h"
#include "gtest/gtest.h"

#ifdef FML_OS_IOS
#include <asl.h>
#endif  // FML_OS_IOS

std::optional<fml::TimeDelta> GetTestTimeout() {
  const auto& command_line = flutter::testing::GetArgsForProcess();

  std::string timeout_seconds;
  if (!command_line.GetOptionValue("timeout", &timeout_seconds)) {
    // No timeout specified. Default to 300s.
    return fml::TimeDelta::FromSeconds(300u);
  }

  const auto seconds = std::stoi(timeout_seconds);

  if (seconds < 1) {
    return std::nullopt;
  }

  return fml::TimeDelta::FromSeconds(seconds);
}

int main(int argc, char** argv) {
  fml::InstallCrashHandler();

  flutter::testing::SetArgsForProcess(argc, argv);

#ifdef FML_OS_IOS
  asl_log_descriptor(NULL, NULL, ASL_LEVEL_NOTICE, STDOUT_FILENO,
                     ASL_LOG_DESCRIPTOR_WRITE);
  asl_log_descriptor(NULL, NULL, ASL_LEVEL_ERR, STDERR_FILENO,
                     ASL_LOG_DESCRIPTOR_WRITE);
#endif  // FML_OS_IOS

  ::testing::InitGoogleTest(&argc, argv);
  GTEST_FLAG_SET(death_test_style, "threadsafe");

  // Check if the user has specified a timeout.
  const auto timeout = GetTestTimeout();
  if (!timeout.has_value()) {
    FML_LOG(INFO) << "Timeouts disabled via a command line flag.";
    return RUN_ALL_TESTS();
  }

  // Check if the user is debugging the process.
  if (flutter::testing::GetDebuggerStatus() ==
      flutter::testing::DebuggerStatus::kAttached) {
    FML_LOG(INFO) << "Debugger is attached. Suspending test timeouts.";
    return RUN_ALL_TESTS();
  }

  auto timeout_listener =
      new flutter::testing::TestTimeoutListener(timeout.value());
  auto& listeners = ::testing::UnitTest::GetInstance()->listeners();
  listeners.Append(timeout_listener);
  auto result = RUN_ALL_TESTS();
  delete listeners.Release(timeout_listener);
  return result;
}

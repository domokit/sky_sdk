// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <gtest/gtest.h>

#include "flutter/shell/platform/fuchsia/flutter/flutter_runner_product_configuration.h"

using namespace flutter_runner;

namespace flutter_runner_test {

class FlutterRunnerProductConfigurationTest : public testing::Test {};

TEST_F(FlutterRunnerProductConfigurationTest, InvalidJsonString) {
  const std::string json_string = "{ \"invalid json string\" }}} ";
  const uint64_t expected_intercept_all_input = false;

  FlutterRunnerProductConfiguration product_config =
      FlutterRunnerProductConfiguration(json_string);
  EXPECT_EQ(expected_intercept_all_input,
            product_config.get_intercept_all_input());
}

TEST_F(FlutterRunnerProductConfigurationTest, EmptyJsonString) {
  const std::string json_string = "";
  const uint64_t expected_intercept_all_input = false;

  FlutterRunnerProductConfiguration product_config =
      FlutterRunnerProductConfiguration(json_string);
  EXPECT_EQ(expected_intercept_all_input,
            product_config.get_intercept_all_input());
}

TEST_F(FlutterRunnerProductConfigurationTest, ValidInterceptAllInput) {
  const std::string json_string = "{ \"intercept_all_input\" : true } ";
  const uint64_t expected_intercept_all_input = true;

  FlutterRunnerProductConfiguration product_config =
      FlutterRunnerProductConfiguration(json_string);

  EXPECT_EQ(expected_intercept_all_input,
            product_config.get_intercept_all_input());
}

TEST_F(FlutterRunnerProductConfigurationTest, MissingInterceptAllInput) {
  const std::string json_string = "{ \"intercept_all_input\" : } ";
  const uint64_t expected_intercept_all_input = false;

  FlutterRunnerProductConfiguration product_config =
      FlutterRunnerProductConfiguration(json_string);

  EXPECT_EQ(expected_intercept_all_input,
            product_config.get_intercept_all_input());
}

}  // namespace flutter_runner_test

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FLUTTER_RUNNER_PRODUCT_CONFIGURATION_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FLUTTER_RUNNER_PRODUCT_CONFIGURATION_H_

#include <string>

namespace flutter_runner {

class FlutterRunnerProductConfiguration {
 public:
  FlutterRunnerProductConfiguration() {}
  explicit FlutterRunnerProductConfiguration(std::string json_string);

  bool get_intercept_all_input() { return intercept_all_input_; }
  bool software_rendering() { return software_rendering_; }
  bool enable_shader_warmup() { return enable_shader_warmup_; }
  bool enable_shader_warmup_dart_hooks() {
    return enable_shader_warmup_dart_hooks_;
  }

 private:
  bool intercept_all_input_ = false;
  bool software_rendering_ = false;
  bool enable_shader_warmup_ = false;
  bool enable_shader_warmup_dart_hooks_ = true;
};

}  // namespace flutter_runner
#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FLUTTER_RUNNER_PRODUCT_CONFIGURATION_H_

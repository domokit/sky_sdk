// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter_runner_product_configuration.h"

#include <zircon/assert.h>

#include "flutter/fml/logging.h"
#include "rapidjson/document.h"

namespace flutter_runner {

FlutterRunnerProductConfiguration::FlutterRunnerProductConfiguration(
    std::string json_string) {
  rapidjson::Document document;
  document.Parse(json_string);

  if (!document.IsObject()) {
    FML_LOG(ERROR) << "Failed to parse configuration; using defaults: "
                   << json_string;
    return;
  }

  // Parse out all values we're expecting.
  if (document.HasMember("intercept_all_input")) {
    auto& val = document["intercept_all_input"];
    if (val.IsBool()) {
      intercept_all_input_ = val.GetBool();
    }
  }
  if (document.HasMember("software_rendering")) {
    auto& val = document["software_rendering"];
    if (val.IsBool()) {
      software_rendering_ = val.GetBool();
    }
  }
  if (document.HasMember("enable_shader_warmup")) {
    auto& val = document["enable_shader_warmup"];
    if (val.IsBool()) {
      enable_shader_warmup_ = val.GetBool();
    }
  }
  if (document.HasMember("enable_shader_warmup_dart_hooks")) {
    auto& val = document["enable_shader_warmup_dart_hooks"];
    if (val.IsBool()) {
      enable_shader_warmup_dart_hooks_ = val.GetBool();
    }
  }
}

}  // namespace flutter_runner

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_PLATFORM_DATA_H_
#define FLUTTER_RUNTIME_PLATFORM_DATA_H_

#include <memory>
#include <string>
#include <vector>

#include "flutter/lib/ui/window/viewport_metrics.h"
#include "flutter/shell/common/display.h"

namespace flutter {

//------------------------------------------------------------------------------
/// The struct of platform-specific data used for initializing
/// ui.PlatformDispatcher.
///
/// The framework may request data from ui.PlatformDispatcher before the
/// platform is properly configured. When creating the Shell, the engine sets
/// this struct to default values until the platform is ready to send the real
/// data.
///
/// See also:
///
///  * flutter::Shell::Create, which takes a platform_data to initialize the
///    ui.PlatformDispatcher attached to it.
struct PlatformData {
  PlatformData();

  ~PlatformData();

  // A map from view IDs of existing views to their viewport metrics.
  std::unordered_map<int64_t, ViewportMetrics> viewport_metrics_for_views;

  std::string language_code;
  std::string country_code;
  std::string script_code;
  std::string variant_code;
  std::vector<std::string> locale_data;
  std::string user_settings_data = "{}";
  std::string lifecycle_state;
  bool semantics_enabled = false;
  bool assistive_technology_enabled = false;
  int32_t accessibility_feature_flags_ = 0;
  std::vector<DisplayData> displays;
};

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_PLATFORM_DATA_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/android/shadow_realm.h"

#include <sys/system_properties.h>

namespace impeller::android {

constexpr std::string_view kAndroidHuawei = "android-huawei";

bool ShadowRealm::ShouldDisableAHB() {
  char clientidbase[PROP_VALUE_MAX];
  __system_property_get("ro.com.google.clientidbase", clientidbase);

  auto api_level = android_get_device_api_level();
  char first_api_level[PROP_VALUE_MAX];
  __system_property_get("ro.product.first_api_level", first_api_level);

  return ShouldDisableAHBInternal(clientidbase, first_api_level, api_level);
}

// static
bool ShadowRealm::ShouldDisableAHBInternal(std::string_view clientidbase,
                                           std::string_view first_api_level,
                                           uint32_t api_level) {
  // Most devices that have updated to API 29 don't seem to correctly
  // support AHBs: https://github.com/flutter/flutter/issues/157113
  if (first_api_level.compare("28") == 0 ||
      first_api_level.compare("27") == 0 ||
      first_api_level.compare("26") == 0 ||
      first_api_level.compare("25") == 0 ||
      first_api_level.compare("24") == 0) {
    return true;
  }
  // From local testing, neither the swapchain nor AHB import works, see also:
  // https://github.com/flutter/flutter/issues/154068
  if (clientidbase == kAndroidHuawei && api_level <= 29) {
    return true;
  }
  return false;
}

}  // namespace impeller::android

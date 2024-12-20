// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_ANDROID_SHADOW_REALM_H_
#define FLUTTER_IMPELLER_TOOLKIT_ANDROID_SHADOW_REALM_H_

#include <string_view>

namespace impeller::android {

// Looks like you're going to the Shadow Realm, Jimbo.
class ShadowRealm {
 public:
  /// @brief Whether the device should disable any usage of Android Hardware
  ///        Buffers regardless of stated support.
  static bool ShouldDisableAHB();

  // For testing.
  static bool ShouldDisableAHBInternal(std::string_view clientidbase,
                                       std::string_view first_api_level,
                                       uint32_t api_level);
};

}  // namespace impeller::android

#endif  // FLUTTER_IMPELLER_TOOLKIT_ANDROID_SHADOW_REALM_H_

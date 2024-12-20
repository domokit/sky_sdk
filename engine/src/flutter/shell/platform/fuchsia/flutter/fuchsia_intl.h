// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FUCHSIA_INTL_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FUCHSIA_INTL_H_

#include <fuchsia/intl/cpp/fidl.h>
#include "flutter/fml/mapping.h"

namespace flutter_runner {

// Make a byte vector containing the JSON string used for a localization
// PlatformMessage, using the locale list in the given Profile.
//
// This method does not return a `std::unique_ptr<flutter::PlatformMessage>` for
// testing convenience; that would require an unreasonably large set of
// dependencies for the unit tests.
fml::MallocMapping MakeLocalizationPlatformMessageData(
    const fuchsia::intl::Profile& intl_profile);

}  // namespace flutter_runner

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_FUCHSIA_INTL_H_

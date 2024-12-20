// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_RENDERING_API_SELECTION_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_RENDERING_API_SELECTION_H_

#include <objc/runtime.h>

#include "flutter/fml/macros.h"

namespace flutter {

enum class IOSRenderingAPI {
  kSoftware,
  kMetal,
};

enum class IOSRenderingBackend {
  kSkia,
  kImpeller,
};

// Pass force_software to force software rendering. This is only respected on
// simulators.
IOSRenderingAPI GetRenderingAPIForProcess(bool force_software);

Class GetCoreAnimationLayerClassForRenderingAPI(IOSRenderingAPI rendering_api);

}  // namespace flutter

// Flutter supports Metal on all devices with Apple A7 SoC or above that have
// been updated to or past iOS 10.0. The processor was selected as it is the
// first version at which Metal was supported. The iOS version floor was
// selected due to the availability of features used by Skia.
// Support for Metal on simulators was added by Apple in the SDK for iOS 13.
#if TARGET_OS_SIMULATOR
#define METAL_IOS_VERSION_BASELINE 13.0
#else
#define METAL_IOS_VERSION_BASELINE 10.0
#endif  // TARGET_OS_SIMULATOR

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_RENDERING_API_SELECTION_H_

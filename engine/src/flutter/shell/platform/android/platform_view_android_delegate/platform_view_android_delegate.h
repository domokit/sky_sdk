// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_VIEW_ANDROID_DELEGATE_PLATFORM_VIEW_ANDROID_DELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_VIEW_ANDROID_DELEGATE_PLATFORM_VIEW_ANDROID_DELEGATE_H_

#include <memory>
#include <string>
#include <vector>

#include "flutter/shell/common/platform_view.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"

namespace flutter {

class PlatformViewAndroidDelegate {
 public:
  explicit PlatformViewAndroidDelegate(
      std::shared_ptr<PlatformViewAndroidJNI> jni_facade);
  void UpdateSemantics(
      const flutter::SemanticsNodeUpdates& update,
      const flutter::CustomAccessibilityActionUpdates& actions);

 private:
  const std::shared_ptr<PlatformViewAndroidJNI> jni_facade_;
};
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_PLATFORM_VIEW_ANDROID_DELEGATE_PLATFORM_VIEW_ANDROID_DELEGATE_H_

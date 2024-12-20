// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_DISPLAY_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_DISPLAY_H_

#include <cstdint>

#include "flutter/fml/macros.h"
#include "flutter/shell/common/display.h"
#include "flutter/shell/platform/android/jni/platform_view_android_jni.h"

namespace flutter {

/// A |Display| that listens to refresh rate changes.
class AndroidDisplay : public Display {
 public:
  explicit AndroidDisplay(std::shared_ptr<PlatformViewAndroidJNI> jni_facade);
  ~AndroidDisplay() = default;

  // |Display|
  double GetRefreshRate() const override;

  // |Display|
  virtual double GetWidth() const override;

  // |Display|
  virtual double GetHeight() const override;

  // |Display|
  virtual double GetDevicePixelRatio() const override;

 private:
  std::shared_ptr<PlatformViewAndroidJNI> jni_facade_;

  FML_DISALLOW_COPY_AND_ASSIGN(AndroidDisplay);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_ANDROID_DISPLAY_H_

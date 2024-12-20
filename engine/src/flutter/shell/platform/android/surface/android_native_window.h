// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_ANDROID_NATIVE_WINDOW_H_
#define FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_ANDROID_NATIVE_WINDOW_H_

#include "flutter/fml/build_config.h"

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"
#include "third_party/skia/include/core/SkSize.h"

#if FML_OS_ANDROID
#include <android/native_window.h>
#endif  // FML_OS_ANDROID

namespace flutter {

class AndroidNativeWindow
    : public fml::RefCountedThreadSafe<AndroidNativeWindow> {
 public:
#if FML_OS_ANDROID
  using Handle = ANativeWindow*;
#else   // FML_OS_ANDROID
  using Handle = std::nullptr_t;
#endif  // FML_OS_ANDROID

  bool IsValid() const;

  Handle handle() const;

  SkISize GetSize() const;

  /// Returns true when this AndroidNativeWindow is not backed by a real window
  /// (used for testing).
  bool IsFakeWindow() const { return is_fake_window_; }

 private:
  Handle window_;
  const bool is_fake_window_;

  /// Creates a native window with the given handle. Handle ownership is assumed
  /// by this instance of the native window.
  explicit AndroidNativeWindow(Handle window);

  explicit AndroidNativeWindow(Handle window, bool is_fake_window);

  ~AndroidNativeWindow();

  FML_FRIEND_MAKE_REF_COUNTED(AndroidNativeWindow);
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(AndroidNativeWindow);
  FML_DISALLOW_COPY_AND_ASSIGN(AndroidNativeWindow);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_ANDROID_SURFACE_ANDROID_NATIVE_WINDOW_H_

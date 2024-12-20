// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/surface/android_native_window.h"

namespace flutter {

AndroidNativeWindow::AndroidNativeWindow(Handle window, bool is_fake_window)
    : window_(window), is_fake_window_(is_fake_window) {}

AndroidNativeWindow::AndroidNativeWindow(Handle window)
    : AndroidNativeWindow(window, /*is_fake_window=*/false) {}

AndroidNativeWindow::~AndroidNativeWindow() {
  if (window_ != nullptr) {
#if FML_OS_ANDROID
    ANativeWindow_release(window_);
    window_ = nullptr;
#endif  // FML_OS_ANDROID
  }
}

bool AndroidNativeWindow::IsValid() const {
  return window_ != nullptr;
}

AndroidNativeWindow::Handle AndroidNativeWindow::handle() const {
  return window_;
}

SkISize AndroidNativeWindow::GetSize() const {
#if FML_OS_ANDROID
  return window_ == nullptr ? SkISize::Make(0, 0)
                            : SkISize::Make(ANativeWindow_getWidth(window_),
                                            ANativeWindow_getHeight(window_));
#else   // FML_OS_ANDROID
  return SkISize::Make(0, 0);
#endif  // FML_OS_ANDROID
}

}  // namespace flutter

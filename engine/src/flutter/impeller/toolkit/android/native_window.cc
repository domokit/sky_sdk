// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/android/native_window.h"

namespace impeller::android {

NativeWindow::NativeWindow(ANativeWindow* window) : window_(window) {
  if (window_.get()) {
    GetProcTable().ANativeWindow_acquire(window_.get());
  }
}

NativeWindow::~NativeWindow() = default;

bool NativeWindow::IsValid() const {
  return window_.is_valid();
}

ISize NativeWindow::GetSize() const {
  if (!IsValid()) {
    return {};
  }
  const int32_t width = ANativeWindow_getWidth(window_.get());
  const int32_t height = ANativeWindow_getHeight(window_.get());
  return ISize::MakeWH(std::max(width, 0), std::max(height, 0));
}

ANativeWindow* NativeWindow::GetHandle() const {
  return window_.get();
}

}  // namespace impeller::android

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_VIEW_MODIFIER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_VIEW_MODIFIER_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/egl/window_surface.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"

namespace flutter {

// A test utility class providing the ability to access and alter various
// private fields in a |FlutterWindowsView| instance.
class ViewModifier {
 public:
  explicit ViewModifier(FlutterWindowsView* view) : view_(view) {}

  // Override the EGL surface used by the view.
  //
  // Modifications are to the view, and will last for the lifetime of the
  // view unless overwritten again.
  void SetSurface(std::unique_ptr<egl::WindowSurface> surface) {
    view_->surface_ = std::move(surface);
  }

 private:
  FlutterWindowsView* view_;

  FML_DISALLOW_COPY_AND_ASSIGN(ViewModifier);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_TESTING_VIEW_MODIFIER_H_

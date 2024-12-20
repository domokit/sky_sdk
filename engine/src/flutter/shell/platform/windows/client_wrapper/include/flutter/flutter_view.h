// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_CLIENT_WRAPPER_INCLUDE_FLUTTER_FLUTTER_VIEW_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_CLIENT_WRAPPER_INCLUDE_FLUTTER_FLUTTER_VIEW_H_

#include <flutter_windows.h>

namespace flutter {

// The unique identifier for a view.
typedef int64_t FlutterViewId;

// A view displaying Flutter content.
class FlutterView {
 public:
  explicit FlutterView(FlutterDesktopViewRef view) : view_(view) {}

  // Destroys this reference to the view. The underlying view is not destroyed.
  virtual ~FlutterView() = default;

  // Prevent copying.
  FlutterView(FlutterView const&) = delete;
  FlutterView& operator=(FlutterView const&) = delete;

  // Returns the backing HWND for the view.
  HWND GetNativeWindow() { return FlutterDesktopViewGetHWND(view_); }

  // Returns the DXGI adapter used for rendering or nullptr in case of error.
  IDXGIAdapter* GetGraphicsAdapter() {
    return FlutterDesktopViewGetGraphicsAdapter(view_);
  }

 private:
  // Handle for interacting with the C API's view.
  FlutterDesktopViewRef view_ = nullptr;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_CLIENT_WRAPPER_INCLUDE_FLUTTER_FLUTTER_VIEW_H_

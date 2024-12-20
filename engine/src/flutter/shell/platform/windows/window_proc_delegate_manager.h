// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOW_PROC_DELEGATE_MANAGER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOW_PROC_DELEGATE_MANAGER_H_

#include <Windows.h>

#include <optional>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/public/flutter_windows.h"

namespace flutter {

// Handles registration, unregistration, and dispatching for WindowProc
// delegation.
class WindowProcDelegateManager {
 public:
  explicit WindowProcDelegateManager();
  ~WindowProcDelegateManager();

  // Adds |callback| as a delegate to be called for |OnTopLevelWindowProc|.
  //
  // Multiple calls with the same |callback| will replace the previous
  // registration, even if |user_data| is different.
  void RegisterTopLevelWindowProcDelegate(
      FlutterDesktopWindowProcCallback callback,
      void* user_data);

  // Unregisters |callback| as a delegate for |OnTopLevelWindowProc|.
  void UnregisterTopLevelWindowProcDelegate(
      FlutterDesktopWindowProcCallback callback);

  // Calls any registered WindowProc delegates in the order they were
  // registered.
  //
  // If a result is returned, then the message was handled in such a way that
  // further handling should not be done.
  std::optional<LRESULT> OnTopLevelWindowProc(HWND hwnd,
                                              UINT message,
                                              WPARAM wparam,
                                              LPARAM lparam) const;

 private:
  struct WindowProcDelegate {
    FlutterDesktopWindowProcCallback callback = nullptr;
    void* user_data = nullptr;
  };

  std::vector<WindowProcDelegate> delegates_;

  FML_DISALLOW_COPY_AND_ASSIGN(WindowProcDelegateManager);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOW_PROC_DELEGATE_MANAGER_H_

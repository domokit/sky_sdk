// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/window_proc_delegate_manager.h"

#include <algorithm>

#include "flutter/shell/platform/embedder/embedder.h"

namespace flutter {

WindowProcDelegateManager::WindowProcDelegateManager() = default;
WindowProcDelegateManager::~WindowProcDelegateManager() = default;

void WindowProcDelegateManager::RegisterTopLevelWindowProcDelegate(
    FlutterDesktopWindowProcCallback callback,
    void* user_data) {
  UnregisterTopLevelWindowProcDelegate(callback);

  delegates_.push_back(WindowProcDelegate{
      .callback = callback,
      .user_data = user_data,
  });
}

void WindowProcDelegateManager::UnregisterTopLevelWindowProcDelegate(
    FlutterDesktopWindowProcCallback callback) {
  delegates_.erase(
      std::remove_if(delegates_.begin(), delegates_.end(),
                     [&callback](const WindowProcDelegate& delegate) {
                       return delegate.callback == callback;
                     }),
      delegates_.end());
}

std::optional<LRESULT> WindowProcDelegateManager::OnTopLevelWindowProc(
    HWND hwnd,
    UINT message,
    WPARAM wparam,
    LPARAM lparam) const {
  std::optional<LRESULT> result;
  for (const auto& delegate : delegates_) {
    LPARAM handler_result;
    // Stop as soon as any delegate indicates that it has handled the message.
    if (delegate.callback(hwnd, message, wparam, lparam, delegate.user_data,
                          &handler_result)) {
      result = handler_result;
      break;
    }
  }
  return result;
}

}  // namespace flutter

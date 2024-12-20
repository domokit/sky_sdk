// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_EVENT_WATCHER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_EVENT_WATCHER_H_

#include <Windows.h>

#include <functional>

#include "flutter/fml/macros.h"

namespace flutter {

// A win32 `HANDLE` wrapper for use as a one-time callback.
class EventWatcher {
 public:
  explicit EventWatcher(std::function<void()> callback);
  ~EventWatcher();

  // Returns `HANDLE`, which can be used to register an event listener.
  HANDLE GetHandle();

 private:
  static VOID CALLBACK CallbackForWait(PVOID context, BOOLEAN);

  std::function<void()> callback_;

  HANDLE handle_;
  HANDLE handle_for_wait_;

  FML_DISALLOW_COPY_AND_ASSIGN(EventWatcher);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_EVENT_WATCHER_H_

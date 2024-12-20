// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/event_watcher.h"

namespace flutter {

EventWatcher::EventWatcher(std::function<void()> callback)
    : callback_(callback) {
  handle_ = CreateEvent(NULL, TRUE, FALSE, NULL);

  RegisterWaitForSingleObject(&handle_for_wait_, handle_, &CallbackForWait,
                              reinterpret_cast<void*>(this), INFINITE,
                              WT_EXECUTEONLYONCE | WT_EXECUTEINWAITTHREAD);
}

EventWatcher::~EventWatcher() {
  UnregisterWait(handle_for_wait_);
  CloseHandle(handle_);
}

HANDLE EventWatcher::GetHandle() {
  return handle_;
}

// static
VOID CALLBACK EventWatcher::CallbackForWait(PVOID context, BOOLEAN) {
  EventWatcher* self = reinterpret_cast<EventWatcher*>(context);
  ResetEvent(self->handle_);
  self->callback_();
}

}  // namespace flutter

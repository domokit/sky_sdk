// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/task_runner_window.h"

#include <algorithm>

#include "flutter/fml/logging.h"

namespace flutter {

TaskRunnerWindow::TaskRunnerWindow() {
  WNDCLASS window_class = RegisterWindowClass();
  window_handle_ =
      CreateWindowEx(0, window_class.lpszClassName, L"", 0, 0, 0, 0, 0,
                     HWND_MESSAGE, nullptr, window_class.hInstance, nullptr);

  if (window_handle_) {
    SetWindowLongPtr(window_handle_, GWLP_USERDATA,
                     reinterpret_cast<LONG_PTR>(this));
  } else {
    auto error = GetLastError();
    LPWSTR message = nullptr;
    size_t size = FormatMessageW(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
            FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL, error, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        reinterpret_cast<LPWSTR>(&message), 0, NULL);
    OutputDebugString(message);
    LocalFree(message);
  }
}

TaskRunnerWindow::~TaskRunnerWindow() {
  if (window_handle_) {
    DestroyWindow(window_handle_);
    window_handle_ = nullptr;
  }
  UnregisterClass(window_class_name_.c_str(), nullptr);
}

std::shared_ptr<TaskRunnerWindow> TaskRunnerWindow::GetSharedInstance() {
  static std::weak_ptr<TaskRunnerWindow> instance;
  auto res = instance.lock();
  if (!res) {
    // can't use make_shared with private contructor
    res.reset(new TaskRunnerWindow());
    instance = res;
  }
  return res;
}

void TaskRunnerWindow::WakeUp() {
  if (!PostMessage(window_handle_, WM_NULL, 0, 0)) {
    FML_LOG(ERROR) << "Failed to post message to main thread.";
  }
}

void TaskRunnerWindow::AddDelegate(Delegate* delegate) {
  delegates_.push_back(delegate);
  SetTimer(std::chrono::nanoseconds::zero());
}

void TaskRunnerWindow::RemoveDelegate(Delegate* delegate) {
  auto i = std::find(delegates_.begin(), delegates_.end(), delegate);
  if (i != delegates_.end()) {
    delegates_.erase(i);
  }
}

void TaskRunnerWindow::ProcessTasks() {
  auto next = std::chrono::nanoseconds::max();
  auto delegates_copy(delegates_);
  for (auto delegate : delegates_copy) {
    // if not removed in the meanwhile
    if (std::find(delegates_.begin(), delegates_.end(), delegate) !=
        delegates_.end()) {
      next = std::min(next, delegate->ProcessTasks());
    }
  }
  SetTimer(next);
}

void TaskRunnerWindow::SetTimer(std::chrono::nanoseconds when) {
  if (when == std::chrono::nanoseconds::max()) {
    KillTimer(window_handle_, 0);
  } else {
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(when);
    ::SetTimer(window_handle_, 0, millis.count() + 1, nullptr);
  }
}

WNDCLASS TaskRunnerWindow::RegisterWindowClass() {
  window_class_name_ = L"FlutterTaskRunnerWindow";

  WNDCLASS window_class{};
  window_class.hCursor = nullptr;
  window_class.lpszClassName = window_class_name_.c_str();
  window_class.style = 0;
  window_class.cbClsExtra = 0;
  window_class.cbWndExtra = 0;
  window_class.hInstance = GetModuleHandle(nullptr);
  window_class.hIcon = nullptr;
  window_class.hbrBackground = 0;
  window_class.lpszMenuName = nullptr;
  window_class.lpfnWndProc = WndProc;
  RegisterClass(&window_class);
  return window_class;
}

LRESULT
TaskRunnerWindow::HandleMessage(UINT const message,
                                WPARAM const wparam,
                                LPARAM const lparam) noexcept {
  switch (message) {
    case WM_TIMER:
    case WM_NULL:
      ProcessTasks();
      return 0;
  }
  return DefWindowProcW(window_handle_, message, wparam, lparam);
}

LRESULT TaskRunnerWindow::WndProc(HWND const window,
                                  UINT const message,
                                  WPARAM const wparam,
                                  LPARAM const lparam) noexcept {
  if (auto* that = reinterpret_cast<TaskRunnerWindow*>(
          GetWindowLongPtr(window, GWLP_USERDATA))) {
    return that->HandleMessage(message, wparam, lparam);
  } else {
    return DefWindowProc(window, message, wparam, lparam);
  }
}

}  // namespace flutter

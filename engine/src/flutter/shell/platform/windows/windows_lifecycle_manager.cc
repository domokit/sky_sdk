// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "windows_lifecycle_manager.h"

#include <TlHelp32.h>
#include <WinUser.h>
#include <Windows.h>
#include <tchar.h>

#include "flutter/shell/platform/windows/flutter_windows_engine.h"

namespace flutter {

WindowsLifecycleManager::WindowsLifecycleManager(FlutterWindowsEngine* engine)
    : engine_(engine) {}

WindowsLifecycleManager::~WindowsLifecycleManager() {}

void WindowsLifecycleManager::Quit(std::optional<HWND> hwnd,
                                   std::optional<WPARAM> wparam,
                                   std::optional<LPARAM> lparam,
                                   UINT exit_code) {
  if (!hwnd.has_value()) {
    ::PostQuitMessage(exit_code);
  } else {
    BASE_CHECK(wparam.has_value() && lparam.has_value());
    sent_close_messages_[std::make_tuple(*hwnd, *wparam, *lparam)]++;
    DispatchMessage(*hwnd, WM_CLOSE, *wparam, *lparam);
  }
}

void WindowsLifecycleManager::DispatchMessage(HWND hwnd,
                                              UINT message,
                                              WPARAM wparam,
                                              LPARAM lparam) {
  PostMessage(hwnd, message, wparam, lparam);
}

bool WindowsLifecycleManager::HandleCloseMessage(HWND hwnd,
                                                 WPARAM wparam,
                                                 LPARAM lparam) {
  if (!process_exit_) {
    return false;
  }
  auto key = std::make_tuple(hwnd, wparam, lparam);
  auto itr = sent_close_messages_.find(key);
  if (itr != sent_close_messages_.end()) {
    if (itr->second == 1) {
      sent_close_messages_.erase(itr);
    } else {
      sent_close_messages_[key]--;
    }
    return false;
  }
  if (IsLastWindowOfProcess()) {
    engine_->RequestApplicationQuit(hwnd, wparam, lparam,
                                    AppExitType::cancelable);
    return true;
  }
  return false;
}

bool WindowsLifecycleManager::WindowProc(HWND hwnd,
                                         UINT msg,
                                         WPARAM wpar,
                                         LPARAM lpar,
                                         LRESULT* result) {
  switch (msg) {
    // When WM_CLOSE is received from the final window of an application, we
    // send a request to the framework to see if the app should exit. If it
    // is, we re-dispatch a new WM_CLOSE message. In order to allow the new
    // message to reach other delegates, we ignore it here.
    case WM_CLOSE:
      return HandleCloseMessage(hwnd, wpar, lpar);

    // DWM composition can be disabled on Windows 7.
    // Notify the engine as this can result in screen tearing.
    case WM_DWMCOMPOSITIONCHANGED:
      engine_->OnDwmCompositionChanged();
      break;

    case WM_SIZE:
      if (wpar == SIZE_MAXIMIZED || wpar == SIZE_RESTORED) {
        OnWindowStateEvent(hwnd, WindowStateEvent::kShow);
      } else if (wpar == SIZE_MINIMIZED) {
        OnWindowStateEvent(hwnd, WindowStateEvent::kHide);
      }
      break;

    case WM_SHOWWINDOW:
      if (!wpar) {
        OnWindowStateEvent(hwnd, WindowStateEvent::kHide);
      } else {
        OnWindowStateEvent(hwnd, WindowStateEvent::kShow);
      }
      break;

    case WM_DESTROY:
      OnWindowStateEvent(hwnd, WindowStateEvent::kHide);
      break;
  }
  return false;
}

class ThreadSnapshot {
 public:
  ThreadSnapshot() {
    thread_snapshot_ = ::CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
  }
  ~ThreadSnapshot() {
    if (thread_snapshot_ != INVALID_HANDLE_VALUE) {
      ::CloseHandle(thread_snapshot_);
    }
  }

  std::optional<THREADENTRY32> GetFirstThread() {
    if (thread_snapshot_ == INVALID_HANDLE_VALUE) {
      FML_LOG(ERROR) << "Failed to get thread snapshot";
      return std::nullopt;
    }
    THREADENTRY32 thread;
    thread.dwSize = sizeof(thread);
    if (!::Thread32First(thread_snapshot_, &thread)) {
      DWORD error_num = ::GetLastError();
      char msg[256];
      ::FormatMessageA(
          FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL,
          error_num, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), msg, 256,
          nullptr);
      FML_LOG(ERROR) << "Failed to get thread(" << error_num << "): " << msg;
      return std::nullopt;
    }
    return thread;
  }

  bool GetNextThread(THREADENTRY32& thread) {
    if (thread_snapshot_ == INVALID_HANDLE_VALUE) {
      return false;
    }
    return ::Thread32Next(thread_snapshot_, &thread);
  }

 private:
  HANDLE thread_snapshot_;
};

static int64_t NumWindowsForThread(const THREADENTRY32& thread) {
  int64_t num_windows = 0;
  ::EnumThreadWindows(
      thread.th32ThreadID,
      [](HWND hwnd, LPARAM lparam) {
        int64_t* windows_ptr = reinterpret_cast<int64_t*>(lparam);
        if (::GetParent(hwnd) == nullptr) {
          (*windows_ptr)++;
        }
        return *windows_ptr <= 1 ? TRUE : FALSE;
      },
      reinterpret_cast<LPARAM>(&num_windows));
  return num_windows;
}

bool WindowsLifecycleManager::IsLastWindowOfProcess() {
  DWORD pid = ::GetCurrentProcessId();
  ThreadSnapshot thread_snapshot;
  std::optional<THREADENTRY32> first_thread = thread_snapshot.GetFirstThread();
  if (!first_thread.has_value()) {
    FML_LOG(ERROR) << "No first thread found";
    return true;
  }

  int num_windows = 0;
  THREADENTRY32 thread = *first_thread;
  do {
    if (thread.th32OwnerProcessID == pid) {
      num_windows += NumWindowsForThread(thread);
      if (num_windows > 1) {
        return false;
      }
    }
  } while (thread_snapshot.GetNextThread(thread));

  return num_windows <= 1;
}

void WindowsLifecycleManager::BeginProcessingLifecycle() {
  process_lifecycle_ = true;
}

void WindowsLifecycleManager::BeginProcessingExit() {
  process_exit_ = true;
}

void WindowsLifecycleManager::SetLifecycleState(AppLifecycleState state) {
  if (state_ == state) {
    return;
  }
  state_ = state;
  if (engine_ && process_lifecycle_) {
    const char* state_name = AppLifecycleStateToString(state);
    engine_->SendPlatformMessage("flutter/lifecycle",
                                 reinterpret_cast<const uint8_t*>(state_name),
                                 strlen(state_name), nullptr, nullptr);
  }
}

void WindowsLifecycleManager::OnWindowStateEvent(HWND hwnd,
                                                 WindowStateEvent event) {
  // Synthesize an unfocus event when a focused window is hidden.
  if (event == WindowStateEvent::kHide &&
      focused_windows_.find(hwnd) != focused_windows_.end()) {
    OnWindowStateEvent(hwnd, WindowStateEvent::kUnfocus);
  }

  std::lock_guard guard(state_update_lock_);
  switch (event) {
    case WindowStateEvent::kShow: {
      bool first_shown_window = visible_windows_.empty();
      auto pair = visible_windows_.insert(hwnd);
      if (first_shown_window && pair.second &&
          state_ == AppLifecycleState::kHidden) {
        SetLifecycleState(AppLifecycleState::kInactive);
      }
      break;
    }
    case WindowStateEvent::kHide: {
      bool present = visible_windows_.erase(hwnd);
      bool empty = visible_windows_.empty();
      if (present && empty &&
          (state_ == AppLifecycleState::kResumed ||
           state_ == AppLifecycleState::kInactive)) {
        SetLifecycleState(AppLifecycleState::kHidden);
      }
      break;
    }
    case WindowStateEvent::kFocus: {
      bool first_focused_window = focused_windows_.empty();
      auto pair = focused_windows_.insert(hwnd);
      if (first_focused_window && pair.second &&
          state_ == AppLifecycleState::kInactive) {
        SetLifecycleState(AppLifecycleState::kResumed);
      }
      break;
    }
    case WindowStateEvent::kUnfocus: {
      if (focused_windows_.erase(hwnd) && focused_windows_.empty() &&
          state_ == AppLifecycleState::kResumed) {
        SetLifecycleState(AppLifecycleState::kInactive);
      }
      break;
    }
  }
}

std::optional<LRESULT> WindowsLifecycleManager::ExternalWindowMessage(
    HWND hwnd,
    UINT message,
    WPARAM wparam,
    LPARAM lparam) {
  std::optional<flutter::WindowStateEvent> event = std::nullopt;

  // TODO (schectman): Handle WM_CLOSE messages.
  // https://github.com/flutter/flutter/issues/131497
  switch (message) {
    case WM_SHOWWINDOW:
      event = wparam ? flutter::WindowStateEvent::kShow
                     : flutter::WindowStateEvent::kHide;
      break;
    case WM_SIZE:
      switch (wparam) {
        case SIZE_MINIMIZED:
          event = flutter::WindowStateEvent::kHide;
          break;
        case SIZE_RESTORED:
        case SIZE_MAXIMIZED:
          event = flutter::WindowStateEvent::kShow;
          break;
      }
      break;
    case WM_SETFOCUS:
      event = flutter::WindowStateEvent::kFocus;
      break;
    case WM_KILLFOCUS:
      event = flutter::WindowStateEvent::kUnfocus;
      break;
    case WM_DESTROY:
      event = flutter::WindowStateEvent::kHide;
      break;
    case WM_CLOSE:
      if (HandleCloseMessage(hwnd, wparam, lparam)) {
        return NULL;
      }
      break;
  }

  if (event.has_value()) {
    OnWindowStateEvent(hwnd, *event);
  }

  return std::nullopt;
}

}  // namespace flutter

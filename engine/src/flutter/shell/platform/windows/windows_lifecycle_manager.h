// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOWS_LIFECYCLE_MANAGER_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOWS_LIFECYCLE_MANAGER_H_

#include <Windows.h>

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <set>

#include "flutter/shell/platform/common/app_lifecycle_state.h"

namespace flutter {

class FlutterWindowsEngine;

/// An event representing a change in window state that may update the
// application lifecycle state.
enum class WindowStateEvent {
  kShow,
  kHide,
  kFocus,
  kUnfocus,
};

/// A manager for lifecycle events of the top-level windows.
///
/// WndProc is called for window messages of the top-level Flutter window.
/// ExternalWindowMessage is called for non-flutter top-level window messages.
/// OnWindowStateEvent is called when the visibility or focus state of a window
///   is changed, including the FlutterView window.
class WindowsLifecycleManager {
 public:
  WindowsLifecycleManager(FlutterWindowsEngine* engine);
  virtual ~WindowsLifecycleManager();

  // Called when the engine is notified it should quit, e.g. by an application
  // call to `exitApplication`. When window is std::nullopt, this quits the
  // application. Otherwise, it holds the HWND of the window that initiated the
  // request, and exit_code is unused.
  virtual void Quit(std::optional<HWND> window,
                    std::optional<WPARAM> wparam,
                    std::optional<LPARAM> lparam,
                    UINT exit_code);

  // Intercept top level window WM_CLOSE message and listen to events that may
  // update the application lifecycle.
  bool WindowProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l, LRESULT* result);

  // Signal to start  sending lifecycle state update messages.
  virtual void BeginProcessingLifecycle();

  // Signal to start consuming WM_CLOSE messages.
  virtual void BeginProcessingExit();

  // Update the app lifecycle state in response to a change in window state.
  // When the app lifecycle state actually changes, this sends a platform
  // message to the framework notifying it of the state change.
  virtual void SetLifecycleState(AppLifecycleState state);

  // Respond to a change in window state. Transitions as follows:
  // When the only visible window is hidden, transition from resumed or
  // inactive to hidden.
  // When the only focused window is unfocused, transition from resumed to
  // inactive.
  // When a window is focused, transition from inactive to resumed.
  // When a window is shown, transition from hidden to inactive.
  virtual void OnWindowStateEvent(HWND hwnd, WindowStateEvent event);

  AppLifecycleState GetLifecycleState() { return state_; }

  // Called by the engine when a non-Flutter window receives an event that may
  // alter the lifecycle state. The logic for external windows must differ from
  // that used for FlutterWindow instances, because:
  // - FlutterWindow does not receive WM_SHOW messages,
  // - When FlutterWindow receives WM_SIZE messages, wparam stores no meaningful
  //   information, whereas it usually indicates the action which changed the
  //   window size.
  // When this returns a result, the message has been consumed and should not be
  // processed further. Currently, it will always return nullopt.
  std::optional<LRESULT> ExternalWindowMessage(HWND hwnd,
                                               UINT message,
                                               WPARAM wparam,
                                               LPARAM lparam);

 protected:
  // Check the number of top-level windows associated with this process, and
  // return true only if there are 1 or fewer.
  virtual bool IsLastWindowOfProcess();

  virtual void DispatchMessage(HWND window,
                               UINT msg,
                               WPARAM wparam,
                               LPARAM lparam);

 private:
  // Pass top-level window close notifications to the application lifecycle
  // logic. If the last window of the process receives WM_CLOSE and a listener
  // is registered for WidgetsBindingObserver.didRequestAppExit, the message is
  // sent to the framework to query whether the application should be allowed
  // to quit.
  bool HandleCloseMessage(HWND hwnd, WPARAM wparam, LPARAM lparam);

  FlutterWindowsEngine* engine_;

  std::map<std::tuple<HWND, WPARAM, LPARAM>, int> sent_close_messages_;

  bool process_lifecycle_ = false;
  bool process_exit_ = false;

  std::set<HWND> visible_windows_;

  std::set<HWND> focused_windows_;

  std::mutex state_update_lock_;

  flutter::AppLifecycleState state_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOWS_LIFECYCLE_MANAGER_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/flutter_host_window_controller.h"

#include <dwmapi.h>

#include "flutter/shell/platform/windows/flutter_windows_engine.h"

namespace flutter {

namespace {

// Names of the messages sent by the controller in response to window events.
constexpr char kOnWindowChangedMethod[] = "onWindowChanged";
constexpr char kOnWindowCreatedMethod[] = "onWindowCreated";
constexpr char kOnWindowDestroyedMethod[] = "onWindowDestroyed";

// Keys used in the onWindow* messages sent through the channel.
constexpr char kIsMovingKey[] = "isMoving";
constexpr char kParentViewIdKey[] = "parentViewId";
constexpr char kRelativePositionKey[] = "relativePosition";
constexpr char kSizeKey[] = "size";
constexpr char kViewIdKey[] = "viewId";

}  // namespace

FlutterHostWindowController::FlutterHostWindowController(
    FlutterWindowsEngine* engine)
    : engine_(engine) {}

FlutterHostWindowController::~FlutterHostWindowController() {
  DestroyAllWindows();
}

std::optional<WindowMetadata> FlutterHostWindowController::CreateHostWindow(
    std::wstring const& title,
    WindowSize const& preferred_size,
    WindowArchetype archetype,
    std::optional<WindowPositioner> positioner,
    std::optional<FlutterViewId> parent_view_id) {
  std::optional<HWND> const owner_hwnd =
      parent_view_id.has_value() &&
              windows_.find(parent_view_id.value()) != windows_.end()
          ? std::optional<HWND>{windows_[parent_view_id.value()]
                                    ->GetWindowHandle()}
          : std::nullopt;

  auto window = std::make_unique<FlutterHostWindow>(
      this, title, preferred_size, archetype, owner_hwnd, positioner);
  if (!window->GetWindowHandle()) {
    return std::nullopt;
  }

  // Assume first window is the main window.
  if (windows_.empty()) {
    window->SetQuitOnClose(true);
  }

  FlutterViewId const view_id = window->GetFlutterViewId().value();
  windows_[view_id] = std::move(window);

  SendOnWindowCreated(view_id, parent_view_id);

  WindowMetadata result = {.view_id = view_id,
                           .archetype = archetype,
                           .size = GetWindowSize(view_id),
                           .parent_id = parent_view_id};

  return result;
}

bool FlutterHostWindowController::DestroyHostWindow(FlutterViewId view_id) {
  if (auto it = windows_.find(view_id); it != windows_.end()) {
    FlutterHostWindow* const window = it->second.get();
    HWND const window_handle = window->GetWindowHandle();

    // |window| will be removed from |windows_| when WM_NCDESTROY is handled.
    PostMessage(window->GetWindowHandle(), WM_CLOSE, 0, 0);

    return true;
  }
  return false;
}

FlutterHostWindow* FlutterHostWindowController::GetHostWindow(
    FlutterViewId view_id) const {
  if (auto it = windows_.find(view_id); it != windows_.end()) {
    return it->second.get();
  }
  return nullptr;
}

LRESULT FlutterHostWindowController::HandleMessage(HWND hwnd,
                                                   UINT message,
                                                   WPARAM wparam,
                                                   LPARAM lparam) {
  switch (message) {
    case WM_NCDESTROY: {
      auto const it = std::find_if(
          windows_.begin(), windows_.end(), [hwnd](auto const& window) {
            return window.second->GetWindowHandle() == hwnd;
          });
      if (it != windows_.end()) {
        FlutterViewId const view_id = it->first;
        bool const quit_on_close = it->second->GetQuitOnClose();

        windows_.erase(it);

        SendOnWindowDestroyed(view_id);

        if (quit_on_close) {
          DestroyAllWindows();
        }
      }
    }
      return 0;
    case WM_ACTIVATE:
      if (wparam != WA_INACTIVE) {
        if (FlutterHostWindow* const window =
                FlutterHostWindow::GetThisFromHandle(hwnd)) {
          if (window->GetArchetype() != WindowArchetype::popup) {
            // If a non-popup window is activated, close popups for all windows.
            auto it = windows_.begin();
            while (it != windows_.end()) {
              std::size_t const num_popups_closed =
                  it->second->CloseOwnedPopups();
              if (num_popups_closed > 0) {
                it = windows_.begin();
              } else {
                ++it;
              }
            }
          } else {
            // If a popup window is activated, close its owned popups.
            window->CloseOwnedPopups();
          }
        }
      }
      break;
    case WM_ACTIVATEAPP:
      if (wparam == FALSE) {
        if (FlutterHostWindow* const window =
                FlutterHostWindow::GetThisFromHandle(hwnd)) {
          // Close owned popups if a window belonging to a different application
          // is being activated.
          window->CloseOwnedPopups();
        }
      }
      break;
    case WM_SIZE: {
      auto const it = std::find_if(
          windows_.begin(), windows_.end(), [hwnd](auto const& window) {
            return window.second->GetWindowHandle() == hwnd;
          });
      if (it != windows_.end()) {
        FlutterViewId const view_id = it->first;
        SendOnWindowChanged(view_id);
      }
    } break;
    default:
      break;
  }

  if (FlutterHostWindow* const window =
          FlutterHostWindow::GetThisFromHandle(hwnd)) {
    return window->HandleMessage(hwnd, message, wparam, lparam);
  }
  return DefWindowProc(hwnd, message, wparam, lparam);
}

void FlutterHostWindowController::SetMethodChannel(
    std::shared_ptr<MethodChannel<EncodableValue>> channel) {
  channel_ = std::move(channel);
}

FlutterWindowsEngine* FlutterHostWindowController::engine() const {
  return engine_;
}

void FlutterHostWindowController::DestroyAllWindows() {
  if (!windows_.empty()) {
    // Destroy windows in reverse order of creation.
    for (auto it = std::prev(windows_.end());
         it != std::prev(windows_.begin());) {
      auto current = it--;
      auto const& [view_id, window] = *current;
      if (window->GetWindowHandle()) {
        DestroyHostWindow(view_id);
      }
    }
  }
}

WindowSize FlutterHostWindowController::GetWindowSize(
    FlutterViewId view_id) const {
  HWND const hwnd = windows_.at(view_id)->GetWindowHandle();
  RECT frame_rect;
  DwmGetWindowAttribute(hwnd, DWMWA_EXTENDED_FRAME_BOUNDS, &frame_rect,
                        sizeof(frame_rect));

  // Convert to logical coordinates.
  auto const dpr = FlutterDesktopGetDpiForHWND(hwnd) /
                   static_cast<double>(USER_DEFAULT_SCREEN_DPI);
  frame_rect.left = static_cast<LONG>(frame_rect.left / dpr);
  frame_rect.top = static_cast<LONG>(frame_rect.top / dpr);
  frame_rect.right = static_cast<LONG>(frame_rect.right / dpr);
  frame_rect.bottom = static_cast<LONG>(frame_rect.bottom / dpr);

  auto const width = frame_rect.right - frame_rect.left;
  auto const height = frame_rect.bottom - frame_rect.top;
  return {static_cast<int>(width), static_cast<int>(height)};
}

void FlutterHostWindowController::SendOnWindowChanged(
    FlutterViewId view_id) const {
  if (channel_) {
    WindowSize const size = GetWindowSize(view_id);
    channel_->InvokeMethod(
        kOnWindowChangedMethod,
        std::make_unique<EncodableValue>(EncodableMap{
            {EncodableValue(kViewIdKey), EncodableValue(view_id)},
            {EncodableValue(kSizeKey),
             EncodableValue(EncodableList{EncodableValue(size.width),
                                          EncodableValue(size.height)})},
            {EncodableValue(kRelativePositionKey), EncodableValue()},
            {EncodableValue(kIsMovingKey), EncodableValue()}}));
  }
}

void FlutterHostWindowController::SendOnWindowCreated(
    FlutterViewId view_id,
    std::optional<FlutterViewId> parent_view_id) const {
  if (channel_) {
    channel_->InvokeMethod(
        kOnWindowCreatedMethod,
        std::make_unique<EncodableValue>(EncodableMap{
            {EncodableValue(kViewIdKey), EncodableValue(view_id)},
            {EncodableValue(kParentViewIdKey),
             parent_view_id ? EncodableValue(parent_view_id.value())
                            : EncodableValue()}}));
  }
}

void FlutterHostWindowController::SendOnWindowDestroyed(
    FlutterViewId view_id) const {
  if (channel_) {
    channel_->InvokeMethod(
        kOnWindowDestroyedMethod,
        std::make_unique<EncodableValue>(EncodableMap{
            {EncodableValue(kViewIdKey), EncodableValue(view_id)},
        }));
  }
}

}  // namespace flutter

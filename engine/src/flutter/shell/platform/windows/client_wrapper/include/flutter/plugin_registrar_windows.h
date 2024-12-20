// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_CLIENT_WRAPPER_INCLUDE_FLUTTER_PLUGIN_REGISTRAR_WINDOWS_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_CLIENT_WRAPPER_INCLUDE_FLUTTER_PLUGIN_REGISTRAR_WINDOWS_H_

#include <flutter_windows.h>
#include <windows.h>

#include <memory>
#include <optional>

#include "flutter_view.h"
#include "plugin_registrar.h"

namespace flutter {

// A delegate callback for WindowProc delegation.
//
// Implementations should return a value only if they have handled the message
// and want to stop all further handling.
using WindowProcDelegate = std::function<std::optional<
    LRESULT>(HWND hwnd, UINT message, WPARAM wparam, LPARAM lparam)>;

// An extension to PluginRegistrar providing access to Windows-specific
// functionality.
class PluginRegistrarWindows : public PluginRegistrar {
 public:
  // Creates a new PluginRegistrar. |core_registrar| and the messenger it
  // provides must remain valid as long as this object exists.
  explicit PluginRegistrarWindows(
      FlutterDesktopPluginRegistrarRef core_registrar)
      : PluginRegistrar(core_registrar) {
    FlutterDesktopViewRef implicit_view =
        FlutterDesktopPluginRegistrarGetView(core_registrar);
    if (implicit_view) {
      implicit_view_ = std::make_unique<FlutterView>(implicit_view);
    }
  }

  virtual ~PluginRegistrarWindows() {
    // Must be the first call.
    ClearPlugins();
    // Explicitly cleared to facilitate destruction order testing.
    implicit_view_.reset();
  }

  // Prevent copying.
  PluginRegistrarWindows(PluginRegistrarWindows const&) = delete;
  PluginRegistrarWindows& operator=(PluginRegistrarWindows const&) = delete;

  // Returns the implicit view, or nullptr if there is no implicit view.
  //
  // See:
  // https://api.flutter.dev/flutter/dart-ui/PlatformDispatcher/implicitView.html
  //
  // DEPRECATED: Use |GetViewById| instead.
  FlutterView* GetView() { return implicit_view_.get(); }

  // Returns the view with the given ID, or nullptr if the view does not exist.
  //
  // Destroying the shared pointer destroys the reference to the view; it does
  // not destroy the underlying view.
  std::shared_ptr<FlutterView> GetViewById(FlutterViewId view_id) const {
    FlutterDesktopViewRef view =
        FlutterDesktopPluginRegistrarGetViewById(registrar(), view_id);
    if (!view) {
      return nullptr;
    }

    return std::make_shared<FlutterView>(view);
  }

  // Registers |delegate| to receive WindowProc callbacks for the top-level
  // window containing this Flutter instance. Returns an ID that can be used to
  // unregister the handler.
  //
  // Delegates are not guaranteed to be called:
  // - The application may choose not to delegate WindowProc calls.
  // - If multiple plugins are registered, the first one that returns a value
  //   from the delegate message will "win", and others will not be called.
  //   The order of delegate calls is not defined.
  //
  // Delegates should be implemented as narrowly as possible, only returning
  // a value in cases where it's important that other delegates not run, to
  // minimize the chances of conflicts between plugins.
  int RegisterTopLevelWindowProcDelegate(WindowProcDelegate delegate) {
    if (window_proc_delegates_.empty()) {
      FlutterDesktopPluginRegistrarRegisterTopLevelWindowProcDelegate(
          registrar(), PluginRegistrarWindows::OnTopLevelWindowProc, this);
    }
    int delegate_id = next_window_proc_delegate_id_++;
    window_proc_delegates_.emplace(delegate_id, std::move(delegate));
    return delegate_id;
  }

  // Unregisters a previously registered delegate.
  void UnregisterTopLevelWindowProcDelegate(int proc_id) {
    window_proc_delegates_.erase(proc_id);
    if (window_proc_delegates_.empty()) {
      FlutterDesktopPluginRegistrarUnregisterTopLevelWindowProcDelegate(
          registrar(), PluginRegistrarWindows::OnTopLevelWindowProc);
    }
  }

 private:
  // A FlutterDesktopWindowProcCallback implementation that forwards back to
  // a PluginRegistarWindows instance provided as |user_data|.
  static bool OnTopLevelWindowProc(HWND hwnd,
                                   UINT message,
                                   WPARAM wparam,
                                   LPARAM lparam,
                                   void* user_data,
                                   LRESULT* result) {
    const auto* registrar = static_cast<PluginRegistrarWindows*>(user_data);
    std::optional optional_result = registrar->CallTopLevelWindowProcDelegates(
        hwnd, message, wparam, lparam);
    if (optional_result) {
      *result = *optional_result;
    }
    return optional_result.has_value();
  }

  std::optional<LRESULT> CallTopLevelWindowProcDelegates(HWND hwnd,
                                                         UINT message,
                                                         WPARAM wparam,
                                                         LPARAM lparam) const {
    std::optional<LRESULT> result;
    for (const auto& pair : window_proc_delegates_) {
      result = pair.second(hwnd, message, wparam, lparam);
      // Stop as soon as any delegate indicates that it has handled the message.
      if (result) {
        break;
      }
    }
    return result;
  }

  // The associated FlutterView, if any.
  std::unique_ptr<FlutterView> implicit_view_;

  // The next ID to return from RegisterWindowProcDelegate.
  int next_window_proc_delegate_id_ = 1;

  std::map<int, WindowProcDelegate> window_proc_delegates_;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_CLIENT_WRAPPER_INCLUDE_FLUTTER_PLUGIN_REGISTRAR_WINDOWS_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOW_BINDING_HANDLER_DELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOW_BINDING_HANDLER_DELEGATE_H_

#include <functional>

#include "flutter/shell/platform/common/geometry.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/windows/windows_lifecycle_manager.h"
#include "flutter/third_party/accessibility/ax/platform/ax_fragment_root_delegate_win.h"
#include "flutter/third_party/accessibility/gfx/native_widget_types.h"

namespace flutter {

class WindowBindingHandlerDelegate {
 public:
  using KeyEventCallback = std::function<void(bool)>;

  // Notifies delegate that backing window size has changed.
  //
  // Called by |FlutterWindow| on the platform thread.
  //
  // Returns true if the delegate completed the window resize synchronously.
  // The return value is exposed for unit testing.
  virtual bool OnWindowSizeChanged(size_t width, size_t height) = 0;

  // Notifies delegate that backing window needs to be repainted.
  // Typically called by currently configured WindowBindingHandler.
  virtual void OnWindowRepaint() = 0;

  // Notifies delegate that backing window mouse has moved.
  // Typically called by currently configured WindowBindingHandler.
  virtual void OnPointerMove(double x,
                             double y,
                             FlutterPointerDeviceKind device_kind,
                             int32_t device_id,
                             int modifiers_state) = 0;

  // Notifies delegate that backing window mouse pointer button has been
  // pressed. Typically called by currently configured WindowBindingHandler.
  virtual void OnPointerDown(double x,
                             double y,
                             FlutterPointerDeviceKind device_kind,
                             int32_t device_id,
                             FlutterPointerMouseButtons button) = 0;

  // Notifies delegate that backing window mouse pointer button has been
  // released. Typically called by currently configured WindowBindingHandler.
  virtual void OnPointerUp(double x,
                           double y,
                           FlutterPointerDeviceKind device_kind,
                           int32_t device_id,
                           FlutterPointerMouseButtons button) = 0;

  // Notifies delegate that backing window mouse pointer has left the window.
  // Typically called by currently configured WindowBindingHandler.
  virtual void OnPointerLeave(double x,
                              double y,
                              FlutterPointerDeviceKind device_kind,
                              int32_t device_id) = 0;

  // Notifies delegate that a pan/zoom gesture has started.
  // Typically called by DirectManipulationEventHandler.
  virtual void OnPointerPanZoomStart(int32_t device_id) = 0;

  // Notifies delegate that a pan/zoom gesture has updated.
  // Typically called by DirectManipulationEventHandler.
  virtual void OnPointerPanZoomUpdate(int32_t device_id,
                                      double pan_x,
                                      double pan_y,
                                      double scale,
                                      double rotation) = 0;

  // Notifies delegate that a pan/zoom gesture has ended.
  // Typically called by DirectManipulationEventHandler.
  virtual void OnPointerPanZoomEnd(int32_t device_id) = 0;

  // Notifies delegate that backing window has received text.
  // Typically called by currently configured WindowBindingHandler.
  virtual void OnText(const std::u16string&) = 0;

  // Notifies delegate that backing window size has received key press. Should
  // return true if the event was handled and should not be propagated.
  // Typically called by currently configured WindowBindingHandler.
  virtual void OnKey(int key,
                     int scancode,
                     int action,
                     char32_t character,
                     bool extended,
                     bool was_down,
                     KeyEventCallback callback) = 0;

  // Notifies the delegate that IME composing mode has begun.
  //
  // Triggered when the user begins editing composing text using a multi-step
  // input method such as in CJK text input.
  virtual void OnComposeBegin() = 0;

  // Notifies the delegate that IME composing region have been committed.
  //
  // Triggered when the user triggers a commit of the current composing text
  // while using a multi-step input method such as in CJK text input. Composing
  // continues with the next keypress.
  virtual void OnComposeCommit() = 0;

  // Notifies the delegate that IME composing mode has ended.
  //
  // Triggered when the composing ends, for example when the user presses
  // ESC or when the user triggers a commit of the composing text while using a
  // multi-step input method such as in CJK text input.
  virtual void OnComposeEnd() = 0;

  // Notifies the delegate that IME composing region contents have changed.
  //
  // Triggered when the user edits the composing text while using a multi-step
  // input method such as in CJK text input.
  virtual void OnComposeChange(const std::u16string& text, int cursor_pos) = 0;

  // Notifies delegate that backing window size has recevied scroll.
  // Typically called by currently configured WindowBindingHandler.
  virtual void OnScroll(double x,
                        double y,
                        double delta_x,
                        double delta_y,
                        int scroll_offset_multiplier,
                        FlutterPointerDeviceKind device_kind,
                        int32_t device_id) = 0;

  // Notifies delegate that scroll inertia should be cancelled.
  // Typically called by DirectManipulationEventHandler
  virtual void OnScrollInertiaCancel(int32_t device_id) = 0;

  // Notifies delegate that the Flutter semantics tree should be enabled or
  // disabled.
  virtual void OnUpdateSemanticsEnabled(bool enabled) = 0;

  // Returns the root view accessibility node, or nullptr if none.
  virtual gfx::NativeViewAccessible GetNativeViewAccessible() = 0;

  // Update the status of the high contrast feature.
  virtual void OnHighContrastChanged() = 0;

  // Obtain a pointer to the fragment root delegate.
  // This is required by UIA in order to obtain the fragment root that
  // contains a fragment obtained by, for example, a hit test. Unlike
  // MSAA, UIA elements do not explicitly store or enumerate their
  // children and parents, so a method such as this is required.
  virtual ui::AXFragmentRootDelegateWin* GetAxFragmentRootDelegate() = 0;

  // Called when a window receives an event that may alter application lifecycle
  // state.
  virtual void OnWindowStateEvent(HWND hwnd, WindowStateEvent event) = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_WINDOW_BINDING_HANDLER_DELEGATE_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_VIEW_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_VIEW_H_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/client_wrapper/include/flutter/plugin_registrar.h"
#include "flutter/shell/platform/common/geometry.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/windows/accessibility_bridge_windows.h"
#include "flutter/shell/platform/windows/flutter_windows_engine.h"
#include "flutter/shell/platform/windows/public/flutter_windows.h"
#include "flutter/shell/platform/windows/window_binding_handler.h"
#include "flutter/shell/platform/windows/window_binding_handler_delegate.h"
#include "flutter/shell/platform/windows/window_state.h"
#include "flutter/shell/platform/windows/windows_proc_table.h"

namespace flutter {

// A unique identifier for a view.
using FlutterViewId = int64_t;

// An OS-windowing neutral abstration for a Flutter view that works
// with win32 HWNDs.
class FlutterWindowsView : public WindowBindingHandlerDelegate {
 public:
  // Creates a FlutterWindowsView with the given implementor of
  // WindowBindingHandler.
  FlutterWindowsView(
      FlutterViewId view_id,
      FlutterWindowsEngine* engine,
      std::unique_ptr<WindowBindingHandler> window_binding,
      std::shared_ptr<WindowsProcTable> windows_proc_table = nullptr);

  virtual ~FlutterWindowsView();

  // Get the view's unique identifier.
  FlutterViewId view_id() const;

  // Whether this view is the implicit view.
  //
  // The implicit view is a special view for backwards compatibility.
  // The engine assumes it can always render to this view, even if the app has
  // destroyed the window for this view.
  //
  // Today, the implicit view is the first view that is created. It is the only
  // view that can be created before the engine is launched.
  //
  // The embedder must ignore presents to this view before it is created and
  // after it is destroyed.
  //
  // See:
  // https://api.flutter.dev/flutter/dart-ui/PlatformDispatcher/implicitView.html
  bool IsImplicitView() const;

  // Create a rendering surface for Flutter engine to draw into.
  //
  // This is a no-op if using software rasterization.
  void CreateRenderSurface();

  // Get the EGL surface that backs the Flutter view.
  //
  // This might be nullptr or an invalid surface.
  egl::WindowSurface* surface() const;

  // Return the currently configured HWND.
  virtual HWND GetWindowHandle() const;

  // Returns the engine backing this view.
  FlutterWindowsEngine* GetEngine() const;

  // Tells the engine to generate a new frame
  void ForceRedraw();

  // Callback to clear a previously presented software bitmap.
  virtual bool ClearSoftwareBitmap();

  // Callback for presenting a software bitmap.
  virtual bool PresentSoftwareBitmap(const void* allocation,
                                     size_t row_bytes,
                                     size_t height);

  // Creates a window metric for this view.
  //
  // Used to notify the engine of a view's current size and device pixel ratio.
  FlutterWindowMetricsEvent CreateWindowMetricsEvent() const;

  // Send initial bounds to embedder. Must occur after engine has initialized.
  //
  // This is a no-op if this is not the implicit view. Non-implicit views'
  // initial window metrics are sent when the view is added to the engine.
  void SendInitialBounds();

  // Set the text of the alert, and create it if it does not yet exist.
  void AnnounceAlert(const std::wstring& text);

  // |WindowBindingHandlerDelegate|
  void OnHighContrastChanged() override;

  // Called on the raster thread when |CompositorOpenGL| receives an empty
  // frame. Returns true if the frame can be presented.
  //
  // This destroys and then re-creates the view's surface if a resize is
  // pending.
  bool OnEmptyFrameGenerated();

  // Called on the raster thread when |CompositorOpenGL| receives a frame.
  // Returns true if the frame can be presented.
  //
  // This destroys and then re-creates the view's surface if a resize is pending
  // and |width| and |height| match the target size.
  bool OnFrameGenerated(size_t width, size_t height);

  // Called on the raster thread after |CompositorOpenGL| presents a frame.
  //
  // This completes a view resize if one is pending.
  virtual void OnFramePresented();

  // Sets the cursor that should be used when the mouse is over the Flutter
  // content. See mouse_cursor.dart for the values and meanings of cursor_name.
  void UpdateFlutterCursor(const std::string& cursor_name);

  // Sets the cursor directly from a cursor handle.
  void SetFlutterCursor(HCURSOR cursor);

  // |WindowBindingHandlerDelegate|
  bool OnWindowSizeChanged(size_t width, size_t height) override;

  // |WindowBindingHandlerDelegate|
  void OnWindowRepaint() override;

  // |WindowBindingHandlerDelegate|
  void OnPointerMove(double x,
                     double y,
                     FlutterPointerDeviceKind device_kind,
                     int32_t device_id,
                     int modifiers_state) override;

  // |WindowBindingHandlerDelegate|
  void OnPointerDown(double x,
                     double y,
                     FlutterPointerDeviceKind device_kind,
                     int32_t device_id,
                     FlutterPointerMouseButtons button) override;

  // |WindowBindingHandlerDelegate|
  void OnPointerUp(double x,
                   double y,
                   FlutterPointerDeviceKind device_kind,
                   int32_t device_id,
                   FlutterPointerMouseButtons button) override;

  // |WindowBindingHandlerDelegate|
  void OnPointerLeave(double x,
                      double y,
                      FlutterPointerDeviceKind device_kind,
                      int32_t device_id = 0) override;

  // |WindowBindingHandlerDelegate|
  virtual void OnPointerPanZoomStart(int32_t device_id) override;

  // |WindowBindingHandlerDelegate|
  virtual void OnPointerPanZoomUpdate(int32_t device_id,
                                      double pan_x,
                                      double pan_y,
                                      double scale,
                                      double rotation) override;

  // |WindowBindingHandlerDelegate|
  virtual void OnPointerPanZoomEnd(int32_t device_id) override;

  // |WindowBindingHandlerDelegate|
  void OnText(const std::u16string&) override;

  // |WindowBindingHandlerDelegate|
  void OnKey(int key,
             int scancode,
             int action,
             char32_t character,
             bool extended,
             bool was_down,
             KeyEventCallback callback) override;

  // |WindowBindingHandlerDelegate|
  void OnComposeBegin() override;

  // |WindowBindingHandlerDelegate|
  void OnComposeCommit() override;

  // |WindowBindingHandlerDelegate|
  void OnComposeEnd() override;

  // |WindowBindingHandlerDelegate|
  void OnComposeChange(const std::u16string& text, int cursor_pos) override;

  // |WindowBindingHandlerDelegate|
  void OnScroll(double x,
                double y,
                double delta_x,
                double delta_y,
                int scroll_offset_multiplier,
                FlutterPointerDeviceKind device_kind,
                int32_t device_id) override;

  // |WindowBindingHandlerDelegate|
  void OnScrollInertiaCancel(int32_t device_id) override;

  // |WindowBindingHandlerDelegate|
  virtual void OnUpdateSemanticsEnabled(bool enabled) override;

  // |WindowBindingHandlerDelegate|
  virtual gfx::NativeViewAccessible GetNativeViewAccessible() override;

  // Notifies the delegate of the updated the cursor rect in Flutter root view
  // coordinates.
  virtual void OnCursorRectUpdated(const Rect& rect);

  // Notifies the delegate that the system IME composing state should be reset.
  virtual void OnResetImeComposing();

  // Called when a WM_ONCOMPOSITIONCHANGED message is received.
  void OnDwmCompositionChanged();

  // Get a pointer to the alert node for this view.
  ui::AXPlatformNodeWin* AlertNode() const;

  // |WindowBindingHandlerDelegate|
  virtual ui::AXFragmentRootDelegateWin* GetAxFragmentRootDelegate() override;

  // Called to re/set the accessibility bridge pointer.
  virtual void UpdateSemanticsEnabled(bool enabled);

  std::weak_ptr<AccessibilityBridgeWindows> accessibility_bridge() {
    return accessibility_bridge_;
  }

  // |WindowBindingHandlerDelegate|
  void OnWindowStateEvent(HWND hwnd, WindowStateEvent event) override;

 protected:
  virtual void NotifyWinEventWrapper(ui::AXPlatformNodeWin* node,
                                     ax::mojom::Event event);

  // Create an AccessibilityBridgeWindows using this view.
  virtual std::shared_ptr<AccessibilityBridgeWindows>
  CreateAccessibilityBridge();

 private:
  // Allows setting the surface in tests.
  friend class ViewModifier;

  // Struct holding the state of an individual pointer. The engine doesn't keep
  // track of which buttons have been pressed, so it's the embedding's
  // responsibility.
  struct PointerState {
    // The device kind.
    FlutterPointerDeviceKind device_kind = kFlutterPointerDeviceKindMouse;

    // A virtual pointer ID that is unique across all device kinds.
    int32_t pointer_id = 0;

    // True if the last event sent to Flutter had at least one button pressed.
    bool flutter_state_is_down = false;

    // True if kAdd has been sent to Flutter. Used to determine whether
    // to send a kAdd event before sending an incoming pointer event, since
    // Flutter expects pointers to be added before events are sent for them.
    bool flutter_state_is_added = false;

    // The currently pressed buttons, as represented in FlutterPointerEvent.
    uint64_t buttons = 0;

    // The x position where the last pan/zoom started.
    double pan_zoom_start_x = 0;

    // The y position where the last pan/zoom started.
    double pan_zoom_start_y = 0;
  };

  // States a resize event can be in.
  enum class ResizeState {
    // When a resize event has started but is in progress.
    kResizeStarted,
    // After a resize event starts and the framework has been notified to
    // generate a frame for the right size.
    kFrameGenerated,
    // Default state for when no resize is in progress. Also used to indicate
    // that during a resize event, a frame with the right size has been rendered
    // and the buffers have been swapped.
    kDone,
  };

  // Resize the surface to the desired size.
  //
  // If the dimensions have changed, this destroys the original surface and
  // creates a new one.
  //
  // This must be run on the raster thread. This binds the surface to the
  // current thread.
  //
  // Width and height are the surface's desired physical pixel dimensions.
  bool ResizeRenderSurface(size_t width, size_t height);

  // Sends a window metrics update to the Flutter engine using current window
  // dimensions in physical pixels.
  void SendWindowMetrics(size_t width, size_t height, double pixel_ratio) const;

  // Reports a mouse movement to Flutter engine.
  void SendPointerMove(double x, double y, PointerState* state);

  // Reports mouse press to Flutter engine.
  void SendPointerDown(double x, double y, PointerState* state);

  // Reports mouse release to Flutter engine.
  void SendPointerUp(double x, double y, PointerState* state);

  // Reports mouse left the window client area.
  //
  // Win32 api doesn't have "mouse enter" event. Therefore, there is no
  // SendPointerEnter method. A mouse enter event is tracked then the "move"
  // event is called.
  void SendPointerLeave(double x, double y, PointerState* state);

  void SendPointerPanZoomStart(int32_t device_id, double x, double y);

  void SendPointerPanZoomUpdate(int32_t device_id,
                                double pan_x,
                                double pan_y,
                                double scale,
                                double rotation);

  void SendPointerPanZoomEnd(int32_t device_id);

  // Reports a keyboard character to Flutter engine.
  void SendText(const std::u16string&);

  // Reports a raw keyboard message to Flutter engine.
  void SendKey(int key,
               int scancode,
               int action,
               char32_t character,
               bool extended,
               bool was_down,
               KeyEventCallback callback);

  // Reports an IME compose begin event.
  //
  // Triggered when the user begins editing composing text using a multi-step
  // input method such as in CJK text input.
  void SendComposeBegin();

  // Reports an IME compose commit event.
  //
  // Triggered when the user commits the current composing text while using a
  // multi-step input method such as in CJK text input. Composing continues with
  // the next keypress.
  void SendComposeCommit();

  // Reports an IME compose end event.
  //
  // Triggered when the user commits the composing text while using a multi-step
  // input method such as in CJK text input.
  void SendComposeEnd();

  // Reports an IME composing region change event.
  //
  // Triggered when the user edits the composing text while using a multi-step
  // input method such as in CJK text input.
  void SendComposeChange(const std::u16string& text, int cursor_pos);

  // Reports scroll wheel events to Flutter engine.
  void SendScroll(double x,
                  double y,
                  double delta_x,
                  double delta_y,
                  int scroll_offset_multiplier,
                  FlutterPointerDeviceKind device_kind,
                  int32_t device_id);

  // Reports scroll inertia cancel events to Flutter engine.
  void SendScrollInertiaCancel(int32_t device_id, double x, double y);

  // Creates a PointerState object unless it already exists.
  PointerState* GetOrCreatePointerState(FlutterPointerDeviceKind device_kind,
                                        int32_t device_id);

  // Sets |event_data|'s phase to either kMove or kHover depending on the
  // current primary mouse button state.
  void SetEventPhaseFromCursorButtonState(FlutterPointerEvent* event_data,
                                          const PointerState* state) const;

  // Sends a pointer event to the Flutter engine based on given data.  Since
  // all input messages are passed in physical pixel values, no translation is
  // needed before passing on to engine.
  void SendPointerEventWithData(const FlutterPointerEvent& event_data,
                                PointerState* state);

  // If true, rendering to the window should synchronize with the vsync
  // to prevent screen tearing.
  bool NeedsVsync() const;

  // The view's unique identifier.
  FlutterViewId view_id_;

  // The engine associated with this view.
  FlutterWindowsEngine* engine_ = nullptr;

  // Mocks win32 APIs.
  std::shared_ptr<WindowsProcTable> windows_proc_table_;

  // The EGL surface backing the view.
  //
  // Null if using software rasterization, the surface hasn't been created yet,
  // or if surface creation failed.
  std::unique_ptr<egl::WindowSurface> surface_ = nullptr;

  // Keeps track of pointer states in relation to the window.
  std::unordered_map<int32_t, std::unique_ptr<PointerState>> pointer_states_;

  // Currently configured WindowBindingHandler for view.
  std::unique_ptr<WindowBindingHandler> binding_handler_;

  // Resize events are synchronized using this mutex and the corresponding
  // condition variable.
  std::mutex resize_mutex_;
  std::condition_variable resize_cv_;

  // Indicates the state of a window resize event. Platform thread will be
  // blocked while this is not done. Guarded by resize_mutex_.
  ResizeState resize_status_ = ResizeState::kDone;

  // Target for the window width. Valid when resize_pending_ is set. Guarded by
  // resize_mutex_.
  size_t resize_target_width_ = 0;

  // Target for the window width. Valid when resize_pending_ is set. Guarded by
  // resize_mutex_.
  size_t resize_target_height_ = 0;

  // True when flutter's semantics tree is enabled.
  bool semantics_enabled_ = false;

  // The accessibility bridge associated with this view.
  std::shared_ptr<AccessibilityBridgeWindows> accessibility_bridge_;

  FML_DISALLOW_COPY_AND_ASSIGN(FlutterWindowsView);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_WINDOWS_VIEW_H_

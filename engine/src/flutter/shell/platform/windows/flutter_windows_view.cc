// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/flutter_windows_view.h"

#include <chrono>

#include "flutter/common/constants.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/platform/win/wstring_conversion.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/shell/platform/common/accessibility_bridge.h"
#include "flutter/shell/platform/windows/keyboard_key_channel_handler.h"
#include "flutter/shell/platform/windows/text_input_plugin.h"
#include "flutter/third_party/accessibility/ax/platform/ax_platform_node_win.h"

namespace flutter {

namespace {
// The maximum duration to block the platform thread for while waiting
// for a window resize operation to complete.
constexpr std::chrono::milliseconds kWindowResizeTimeout{100};

/// Returns true if the surface will be updated as part of the resize process.
///
/// This is called on window resize to determine if the platform thread needs
/// to be blocked until the frame with the right size has been rendered. It
/// should be kept in-sync with how the engine deals with a new surface request
/// as seen in `CreateOrUpdateSurface` in `GPUSurfaceGL`.
bool SurfaceWillUpdate(size_t cur_width,
                       size_t cur_height,
                       size_t target_width,
                       size_t target_height) {
  // TODO (https://github.com/flutter/flutter/issues/65061) : Avoid special
  // handling for zero dimensions.
  bool non_zero_target_dims = target_height > 0 && target_width > 0;
  bool not_same_size =
      (cur_height != target_height) || (cur_width != target_width);
  return non_zero_target_dims && not_same_size;
}

/// Update the surface's swap interval to block until the v-blank iff
/// the system compositor is disabled.
void UpdateVsync(const FlutterWindowsEngine& engine,
                 egl::WindowSurface* surface,
                 bool needs_vsync) {
  egl::Manager* egl_manager = engine.egl_manager();
  if (!egl_manager) {
    return;
  }

  auto update_vsync = [egl_manager, surface, needs_vsync]() {
    if (!surface || !surface->IsValid()) {
      return;
    }

    if (!surface->MakeCurrent()) {
      FML_LOG(ERROR) << "Unable to make the render surface current to update "
                        "the swap interval";
      return;
    }

    if (!surface->SetVSyncEnabled(needs_vsync)) {
      FML_LOG(ERROR) << "Unable to update the render surface's swap interval";
    }

    if (!egl_manager->render_context()->ClearCurrent()) {
      FML_LOG(ERROR) << "Unable to clear current surface after updating "
                        "the swap interval";
    }
  };

  // Updating the vsync makes the EGL context and render surface current.
  // If the engine is running, the render surface should only be made current on
  // the raster thread. If the engine is initializing, the raster thread doesn't
  // exist yet and the render surface can be made current on the platform
  // thread.
  if (engine.running()) {
    engine.PostRasterThreadTask(update_vsync);
  } else {
    update_vsync();
  }
}

/// Destroys a rendering surface that backs a Flutter view.
void DestroyWindowSurface(const FlutterWindowsEngine& engine,
                          std::unique_ptr<egl::WindowSurface> surface) {
  // EGL surfaces are used on the raster thread if the engine is running.
  // There may be pending raster tasks that use this surface. Destroy the
  // surface on the raster thread to avoid concurrent uses.
  if (engine.running()) {
    engine.PostRasterThreadTask(fml::MakeCopyable(
        [surface = std::move(surface)] { surface->Destroy(); }));
  } else {
    // There's no raster thread if engine isn't running. The surface can be
    // destroyed on the platform thread.
    surface->Destroy();
  }
}

}  // namespace

FlutterWindowsView::FlutterWindowsView(
    FlutterViewId view_id,
    FlutterWindowsEngine* engine,
    std::unique_ptr<WindowBindingHandler> window_binding,
    std::shared_ptr<WindowsProcTable> windows_proc_table)
    : view_id_(view_id),
      engine_(engine),
      windows_proc_table_(std::move(windows_proc_table)) {
  if (windows_proc_table_ == nullptr) {
    windows_proc_table_ = std::make_shared<WindowsProcTable>();
  }

  // Take the binding handler, and give it a pointer back to self.
  binding_handler_ = std::move(window_binding);
  binding_handler_->SetView(this);
}

FlutterWindowsView::~FlutterWindowsView() {
  // The view owns the child window.
  // Notify the engine the view's child window will no longer be visible.
  engine_->OnWindowStateEvent(GetWindowHandle(), WindowStateEvent::kHide);

  if (surface_) {
    DestroyWindowSurface(*engine_, std::move(surface_));
  }
}

bool FlutterWindowsView::OnEmptyFrameGenerated() {
  // Called on the raster thread.
  std::unique_lock<std::mutex> lock(resize_mutex_);

  if (surface_ == nullptr || !surface_->IsValid()) {
    return false;
  }

  if (resize_status_ != ResizeState::kResizeStarted) {
    return true;
  }

  if (!ResizeRenderSurface(resize_target_height_, resize_target_width_)) {
    return false;
  }

  // Platform thread is blocked for the entire duration until the
  // resize_status_ is set to kDone by |OnFramePresented|.
  resize_status_ = ResizeState::kFrameGenerated;
  return true;
}

bool FlutterWindowsView::OnFrameGenerated(size_t width, size_t height) {
  // Called on the raster thread.
  std::unique_lock<std::mutex> lock(resize_mutex_);

  if (surface_ == nullptr || !surface_->IsValid()) {
    return false;
  }

  if (resize_status_ != ResizeState::kResizeStarted) {
    return true;
  }

  if (resize_target_width_ != width || resize_target_height_ != height) {
    return false;
  }

  if (!ResizeRenderSurface(resize_target_width_, resize_target_height_)) {
    return false;
  }

  // Platform thread is blocked for the entire duration until the
  // resize_status_ is set to kDone by |OnFramePresented|.
  resize_status_ = ResizeState::kFrameGenerated;
  return true;
}

void FlutterWindowsView::UpdateFlutterCursor(const std::string& cursor_name) {
  binding_handler_->UpdateFlutterCursor(cursor_name);
}

void FlutterWindowsView::SetFlutterCursor(HCURSOR cursor) {
  binding_handler_->SetFlutterCursor(cursor);
}

void FlutterWindowsView::ForceRedraw() {
  if (resize_status_ == ResizeState::kDone) {
    // Request new frame.
    engine_->ScheduleFrame();
  }
}

bool FlutterWindowsView::OnWindowSizeChanged(size_t width, size_t height) {
  // Called on the platform thread.
  std::unique_lock<std::mutex> lock(resize_mutex_);

  if (!engine_->egl_manager()) {
    SendWindowMetrics(width, height, binding_handler_->GetDpiScale());
    return true;
  }

  if (!surface_ || !surface_->IsValid()) {
    SendWindowMetrics(width, height, binding_handler_->GetDpiScale());
    return true;
  }

  // We're using OpenGL rendering. Resizing the surface must happen on the
  // raster thread.
  bool surface_will_update =
      SurfaceWillUpdate(surface_->width(), surface_->height(), width, height);
  if (!surface_will_update) {
    SendWindowMetrics(width, height, binding_handler_->GetDpiScale());
    return true;
  }

  resize_status_ = ResizeState::kResizeStarted;
  resize_target_width_ = width;
  resize_target_height_ = height;

  SendWindowMetrics(width, height, binding_handler_->GetDpiScale());

  // Block the platform thread until a frame is presented with the target
  // size. See |OnFrameGenerated|, |OnEmptyFrameGenerated|, and
  // |OnFramePresented|.
  return resize_cv_.wait_for(lock, kWindowResizeTimeout,
                             [&resize_status = resize_status_] {
                               return resize_status == ResizeState::kDone;
                             });
}

void FlutterWindowsView::OnWindowRepaint() {
  ForceRedraw();
}

void FlutterWindowsView::OnPointerMove(double x,
                                       double y,
                                       FlutterPointerDeviceKind device_kind,
                                       int32_t device_id,
                                       int modifiers_state) {
  engine_->keyboard_key_handler()->SyncModifiersIfNeeded(modifiers_state);
  SendPointerMove(x, y, GetOrCreatePointerState(device_kind, device_id));
}

void FlutterWindowsView::OnPointerDown(
    double x,
    double y,
    FlutterPointerDeviceKind device_kind,
    int32_t device_id,
    FlutterPointerMouseButtons flutter_button) {
  if (flutter_button != 0) {
    auto state = GetOrCreatePointerState(device_kind, device_id);
    state->buttons |= flutter_button;
    SendPointerDown(x, y, state);
  }
}

void FlutterWindowsView::OnPointerUp(
    double x,
    double y,
    FlutterPointerDeviceKind device_kind,
    int32_t device_id,
    FlutterPointerMouseButtons flutter_button) {
  if (flutter_button != 0) {
    auto state = GetOrCreatePointerState(device_kind, device_id);
    state->buttons &= ~flutter_button;
    SendPointerUp(x, y, state);
  }
}

void FlutterWindowsView::OnPointerLeave(double x,
                                        double y,
                                        FlutterPointerDeviceKind device_kind,
                                        int32_t device_id) {
  SendPointerLeave(x, y, GetOrCreatePointerState(device_kind, device_id));
}

void FlutterWindowsView::OnPointerPanZoomStart(int32_t device_id) {
  PointerLocation point = binding_handler_->GetPrimaryPointerLocation();
  SendPointerPanZoomStart(device_id, point.x, point.y);
}

void FlutterWindowsView::OnPointerPanZoomUpdate(int32_t device_id,
                                                double pan_x,
                                                double pan_y,
                                                double scale,
                                                double rotation) {
  SendPointerPanZoomUpdate(device_id, pan_x, pan_y, scale, rotation);
}

void FlutterWindowsView::OnPointerPanZoomEnd(int32_t device_id) {
  SendPointerPanZoomEnd(device_id);
}

void FlutterWindowsView::OnText(const std::u16string& text) {
  SendText(text);
}

void FlutterWindowsView::OnKey(int key,
                               int scancode,
                               int action,
                               char32_t character,
                               bool extended,
                               bool was_down,
                               KeyEventCallback callback) {
  SendKey(key, scancode, action, character, extended, was_down, callback);
}

void FlutterWindowsView::OnComposeBegin() {
  SendComposeBegin();
}

void FlutterWindowsView::OnComposeCommit() {
  SendComposeCommit();
}

void FlutterWindowsView::OnComposeEnd() {
  SendComposeEnd();
}

void FlutterWindowsView::OnComposeChange(const std::u16string& text,
                                         int cursor_pos) {
  SendComposeChange(text, cursor_pos);
}

void FlutterWindowsView::OnScroll(double x,
                                  double y,
                                  double delta_x,
                                  double delta_y,
                                  int scroll_offset_multiplier,
                                  FlutterPointerDeviceKind device_kind,
                                  int32_t device_id) {
  SendScroll(x, y, delta_x, delta_y, scroll_offset_multiplier, device_kind,
             device_id);
}

void FlutterWindowsView::OnScrollInertiaCancel(int32_t device_id) {
  PointerLocation point = binding_handler_->GetPrimaryPointerLocation();
  SendScrollInertiaCancel(device_id, point.x, point.y);
}

void FlutterWindowsView::OnUpdateSemanticsEnabled(bool enabled) {
  engine_->UpdateSemanticsEnabled(enabled);
}

gfx::NativeViewAccessible FlutterWindowsView::GetNativeViewAccessible() {
  if (!accessibility_bridge_) {
    return nullptr;
  }

  return accessibility_bridge_->GetChildOfAXFragmentRoot();
}

void FlutterWindowsView::OnCursorRectUpdated(const Rect& rect) {
  binding_handler_->OnCursorRectUpdated(rect);
}

void FlutterWindowsView::OnResetImeComposing() {
  binding_handler_->OnResetImeComposing();
}

// Sends new size  information to FlutterEngine.
void FlutterWindowsView::SendWindowMetrics(size_t width,
                                           size_t height,
                                           double pixel_ratio) const {
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = width;
  event.height = height;
  event.pixel_ratio = pixel_ratio;
  event.view_id = view_id_;
  engine_->SendWindowMetricsEvent(event);
}

FlutterWindowMetricsEvent FlutterWindowsView::CreateWindowMetricsEvent() const {
  PhysicalWindowBounds bounds = binding_handler_->GetPhysicalWindowBounds();
  double pixel_ratio = binding_handler_->GetDpiScale();

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = bounds.width;
  event.height = bounds.height;
  event.pixel_ratio = pixel_ratio;
  event.view_id = view_id_;

  return event;
}

void FlutterWindowsView::SendInitialBounds() {
  // Non-implicit views' initial window metrics are sent when the view is added
  // to the engine.
  if (!IsImplicitView()) {
    return;
  }

  engine_->SendWindowMetricsEvent(CreateWindowMetricsEvent());
}

FlutterWindowsView::PointerState* FlutterWindowsView::GetOrCreatePointerState(
    FlutterPointerDeviceKind device_kind,
    int32_t device_id) {
  // Create a virtual pointer ID that is unique across all device types
  // to prevent pointers from clashing in the engine's converter
  // (lib/ui/window/pointer_data_packet_converter.cc)
  int32_t pointer_id = (static_cast<int32_t>(device_kind) << 28) | device_id;

  auto [it, added] = pointer_states_.try_emplace(pointer_id, nullptr);
  if (added) {
    auto state = std::make_unique<PointerState>();
    state->device_kind = device_kind;
    state->pointer_id = pointer_id;
    it->second = std::move(state);
  }

  return it->second.get();
}

// Set's |event_data|'s phase to either kMove or kHover depending on the current
// primary mouse button state.
void FlutterWindowsView::SetEventPhaseFromCursorButtonState(
    FlutterPointerEvent* event_data,
    const PointerState* state) const {
  // For details about this logic, see FlutterPointerPhase in the embedder.h
  // file.
  if (state->buttons == 0) {
    event_data->phase = state->flutter_state_is_down
                            ? FlutterPointerPhase::kUp
                            : FlutterPointerPhase::kHover;
  } else {
    event_data->phase = state->flutter_state_is_down
                            ? FlutterPointerPhase::kMove
                            : FlutterPointerPhase::kDown;
  }
}

void FlutterWindowsView::SendPointerMove(double x,
                                         double y,
                                         PointerState* state) {
  FlutterPointerEvent event = {};
  event.x = x;
  event.y = y;

  SetEventPhaseFromCursorButtonState(&event, state);
  SendPointerEventWithData(event, state);
}

void FlutterWindowsView::SendPointerDown(double x,
                                         double y,
                                         PointerState* state) {
  FlutterPointerEvent event = {};
  event.x = x;
  event.y = y;

  SetEventPhaseFromCursorButtonState(&event, state);
  SendPointerEventWithData(event, state);

  state->flutter_state_is_down = true;
}

void FlutterWindowsView::SendPointerUp(double x,
                                       double y,
                                       PointerState* state) {
  FlutterPointerEvent event = {};
  event.x = x;
  event.y = y;

  SetEventPhaseFromCursorButtonState(&event, state);
  SendPointerEventWithData(event, state);
  if (event.phase == FlutterPointerPhase::kUp) {
    state->flutter_state_is_down = false;
  }
}

void FlutterWindowsView::SendPointerLeave(double x,
                                          double y,
                                          PointerState* state) {
  FlutterPointerEvent event = {};
  event.x = x;
  event.y = y;
  event.phase = FlutterPointerPhase::kRemove;
  SendPointerEventWithData(event, state);
}

void FlutterWindowsView::SendPointerPanZoomStart(int32_t device_id,
                                                 double x,
                                                 double y) {
  auto state =
      GetOrCreatePointerState(kFlutterPointerDeviceKindTrackpad, device_id);
  state->pan_zoom_start_x = x;
  state->pan_zoom_start_y = y;
  FlutterPointerEvent event = {};
  event.x = x;
  event.y = y;
  event.phase = FlutterPointerPhase::kPanZoomStart;
  SendPointerEventWithData(event, state);
}

void FlutterWindowsView::SendPointerPanZoomUpdate(int32_t device_id,
                                                  double pan_x,
                                                  double pan_y,
                                                  double scale,
                                                  double rotation) {
  auto state =
      GetOrCreatePointerState(kFlutterPointerDeviceKindTrackpad, device_id);
  FlutterPointerEvent event = {};
  event.x = state->pan_zoom_start_x;
  event.y = state->pan_zoom_start_y;
  event.pan_x = pan_x;
  event.pan_y = pan_y;
  event.scale = scale;
  event.rotation = rotation;
  event.phase = FlutterPointerPhase::kPanZoomUpdate;
  SendPointerEventWithData(event, state);
}

void FlutterWindowsView::SendPointerPanZoomEnd(int32_t device_id) {
  auto state =
      GetOrCreatePointerState(kFlutterPointerDeviceKindTrackpad, device_id);
  FlutterPointerEvent event = {};
  event.x = state->pan_zoom_start_x;
  event.y = state->pan_zoom_start_y;
  event.phase = FlutterPointerPhase::kPanZoomEnd;
  SendPointerEventWithData(event, state);
}

void FlutterWindowsView::SendText(const std::u16string& text) {
  engine_->text_input_plugin()->TextHook(text);
}

void FlutterWindowsView::SendKey(int key,
                                 int scancode,
                                 int action,
                                 char32_t character,
                                 bool extended,
                                 bool was_down,
                                 KeyEventCallback callback) {
  engine_->keyboard_key_handler()->KeyboardHook(
      key, scancode, action, character, extended, was_down,
      [=, callback = std::move(callback)](bool handled) {
        if (!handled) {
          engine_->text_input_plugin()->KeyboardHook(
              key, scancode, action, character, extended, was_down);
        }
        callback(handled);
      });
}

void FlutterWindowsView::SendComposeBegin() {
  engine_->text_input_plugin()->ComposeBeginHook();
}

void FlutterWindowsView::SendComposeCommit() {
  engine_->text_input_plugin()->ComposeCommitHook();
}

void FlutterWindowsView::SendComposeEnd() {
  engine_->text_input_plugin()->ComposeEndHook();
}

void FlutterWindowsView::SendComposeChange(const std::u16string& text,
                                           int cursor_pos) {
  engine_->text_input_plugin()->ComposeChangeHook(text, cursor_pos);
}

void FlutterWindowsView::SendScroll(double x,
                                    double y,
                                    double delta_x,
                                    double delta_y,
                                    int scroll_offset_multiplier,
                                    FlutterPointerDeviceKind device_kind,
                                    int32_t device_id) {
  auto state = GetOrCreatePointerState(device_kind, device_id);

  FlutterPointerEvent event = {};
  event.x = x;
  event.y = y;
  event.signal_kind = FlutterPointerSignalKind::kFlutterPointerSignalKindScroll;
  event.scroll_delta_x = delta_x * scroll_offset_multiplier;
  event.scroll_delta_y = delta_y * scroll_offset_multiplier;
  SetEventPhaseFromCursorButtonState(&event, state);
  SendPointerEventWithData(event, state);
}

void FlutterWindowsView::SendScrollInertiaCancel(int32_t device_id,
                                                 double x,
                                                 double y) {
  auto state =
      GetOrCreatePointerState(kFlutterPointerDeviceKindTrackpad, device_id);

  FlutterPointerEvent event = {};
  event.x = x;
  event.y = y;
  event.signal_kind =
      FlutterPointerSignalKind::kFlutterPointerSignalKindScrollInertiaCancel;
  SetEventPhaseFromCursorButtonState(&event, state);
  SendPointerEventWithData(event, state);
}

void FlutterWindowsView::SendPointerEventWithData(
    const FlutterPointerEvent& event_data,
    PointerState* state) {
  // If sending anything other than an add, and the pointer isn't already added,
  // synthesize an add to satisfy Flutter's expectations about events.
  if (!state->flutter_state_is_added &&
      event_data.phase != FlutterPointerPhase::kAdd) {
    FlutterPointerEvent event = {};
    event.phase = FlutterPointerPhase::kAdd;
    event.x = event_data.x;
    event.y = event_data.y;
    event.buttons = 0;
    SendPointerEventWithData(event, state);
  }

  // Don't double-add (e.g., if events are delivered out of order, so an add has
  // already been synthesized).
  if (state->flutter_state_is_added &&
      event_data.phase == FlutterPointerPhase::kAdd) {
    return;
  }

  FlutterPointerEvent event = event_data;
  event.device_kind = state->device_kind;
  event.device = state->pointer_id;
  event.buttons = state->buttons;
  event.view_id = view_id_;

  // Set metadata that's always the same regardless of the event.
  event.struct_size = sizeof(event);
  event.timestamp =
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now().time_since_epoch())
          .count();

  engine_->SendPointerEvent(event);

  if (event_data.phase == FlutterPointerPhase::kAdd) {
    state->flutter_state_is_added = true;
  } else if (event_data.phase == FlutterPointerPhase::kRemove) {
    auto it = pointer_states_.find(state->pointer_id);
    if (it != pointer_states_.end()) {
      pointer_states_.erase(it);
    }
  }
}

void FlutterWindowsView::OnFramePresented() {
  // Called on the engine's raster thread.
  std::unique_lock<std::mutex> lock(resize_mutex_);

  switch (resize_status_) {
    case ResizeState::kResizeStarted:
      // The caller must first call |OnFrameGenerated| or
      // |OnEmptyFrameGenerated| before calling this method. This
      // indicates one of the following:
      //
      // 1. The caller did not call these methods.
      // 2. The caller ignored these methods' result.
      // 3. The platform thread started a resize after the caller called these
      //    methods. We might have presented a frame of the wrong size to the
      //    view.
      return;
    case ResizeState::kFrameGenerated: {
      // A frame was generated for a pending resize.
      // Unblock the platform thread.
      resize_status_ = ResizeState::kDone;
      lock.unlock();
      resize_cv_.notify_all();

      // Blocking the raster thread until DWM flushes alleviates glitches where
      // previous size surface is stretched over current size view.
      windows_proc_table_->DwmFlush();
    }
    case ResizeState::kDone:
      return;
  }
}

bool FlutterWindowsView::ClearSoftwareBitmap() {
  return binding_handler_->OnBitmapSurfaceCleared();
}

bool FlutterWindowsView::PresentSoftwareBitmap(const void* allocation,
                                               size_t row_bytes,
                                               size_t height) {
  return binding_handler_->OnBitmapSurfaceUpdated(allocation, row_bytes,
                                                  height);
}

FlutterViewId FlutterWindowsView::view_id() const {
  return view_id_;
}

bool FlutterWindowsView::IsImplicitView() const {
  return view_id_ == kImplicitViewId;
}

void FlutterWindowsView::CreateRenderSurface() {
  FML_DCHECK(surface_ == nullptr);

  if (engine_->egl_manager()) {
    PhysicalWindowBounds bounds = binding_handler_->GetPhysicalWindowBounds();
    surface_ = engine_->egl_manager()->CreateWindowSurface(
        GetWindowHandle(), bounds.width, bounds.height);

    UpdateVsync(*engine_, surface_.get(), NeedsVsync());

    resize_target_width_ = bounds.width;
    resize_target_height_ = bounds.height;
  }
}

bool FlutterWindowsView::ResizeRenderSurface(size_t width, size_t height) {
  FML_DCHECK(surface_ != nullptr);

  // No-op if the surface is already the desired size.
  if (width == surface_->width() && height == surface_->height()) {
    return true;
  }

  auto const existing_vsync = surface_->vsync_enabled();

  // TODO: Destroying the surface and re-creating it is expensive.
  // Ideally this would use ANGLE's automatic surface sizing instead.
  // See: https://github.com/flutter/flutter/issues/79427
  if (!surface_->Destroy()) {
    FML_LOG(ERROR) << "View resize failed to destroy surface";
    return false;
  }

  std::unique_ptr<egl::WindowSurface> resized_surface =
      engine_->egl_manager()->CreateWindowSurface(GetWindowHandle(), width,
                                                  height);
  if (!resized_surface) {
    FML_LOG(ERROR) << "View resize failed to create surface";
    return false;
  }

  if (!resized_surface->MakeCurrent() ||
      !resized_surface->SetVSyncEnabled(existing_vsync)) {
    // Surfaces block until the v-blank by default.
    // Failing to update the vsync might result in unnecessary blocking.
    // This regresses performance but not correctness.
    FML_LOG(ERROR) << "View resize failed to set vsync";
  }

  surface_ = std::move(resized_surface);
  return true;
}

egl::WindowSurface* FlutterWindowsView::surface() const {
  return surface_.get();
}

void FlutterWindowsView::OnHighContrastChanged() {
  engine_->UpdateHighContrastMode();
}

HWND FlutterWindowsView::GetWindowHandle() const {
  return binding_handler_->GetWindowHandle();
}

FlutterWindowsEngine* FlutterWindowsView::GetEngine() const {
  return engine_;
}

void FlutterWindowsView::AnnounceAlert(const std::wstring& text) {
  auto alert_delegate = binding_handler_->GetAlertDelegate();
  if (!alert_delegate) {
    return;
  }
  alert_delegate->SetText(fml::WideStringToUtf16(text));
  ui::AXPlatformNodeWin* alert_node = binding_handler_->GetAlert();
  NotifyWinEventWrapper(alert_node, ax::mojom::Event::kAlert);
}

void FlutterWindowsView::NotifyWinEventWrapper(ui::AXPlatformNodeWin* node,
                                               ax::mojom::Event event) {
  if (node) {
    node->NotifyAccessibilityEvent(event);
  }
}

ui::AXFragmentRootDelegateWin* FlutterWindowsView::GetAxFragmentRootDelegate() {
  return accessibility_bridge_.get();
}

ui::AXPlatformNodeWin* FlutterWindowsView::AlertNode() const {
  return binding_handler_->GetAlert();
}

std::shared_ptr<AccessibilityBridgeWindows>
FlutterWindowsView::CreateAccessibilityBridge() {
  return std::make_shared<AccessibilityBridgeWindows>(this);
}

void FlutterWindowsView::UpdateSemanticsEnabled(bool enabled) {
  if (semantics_enabled_ != enabled) {
    semantics_enabled_ = enabled;

    if (!semantics_enabled_ && accessibility_bridge_) {
      accessibility_bridge_.reset();
    } else if (semantics_enabled_ && !accessibility_bridge_) {
      accessibility_bridge_ = CreateAccessibilityBridge();
    }
  }
}

void FlutterWindowsView::OnDwmCompositionChanged() {
  UpdateVsync(*engine_, surface_.get(), NeedsVsync());
}

void FlutterWindowsView::OnWindowStateEvent(HWND hwnd, WindowStateEvent event) {
  engine_->OnWindowStateEvent(hwnd, event);
}

bool FlutterWindowsView::NeedsVsync() const {
  // If the Desktop Window Manager composition is enabled,
  // the system itself synchronizes with vsync.
  // See: https://learn.microsoft.com/windows/win32/dwm/composition-ovw
  return !windows_proc_table_->DwmIsCompositionEnabled();
}

}  // namespace flutter

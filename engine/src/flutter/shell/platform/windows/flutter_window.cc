// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/flutter_window.h"

#include <WinUser.h>
#include <dwmapi.h>

#include <chrono>
#include <map>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/windows/dpi_utils.h"
#include "flutter/shell/platform/windows/flutter_windows_engine.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"
#include "flutter/shell/platform/windows/keyboard_utils.h"

namespace flutter {

namespace {

// The Windows DPI system is based on this
// constant for machines running at 100% scaling.
constexpr int base_dpi = 96;

static const int kMinTouchDeviceId = 0;
static const int kMaxTouchDeviceId = 128;

static const int kLinesPerScrollWindowsDefault = 3;

// Maps a Flutter cursor name to an HCURSOR.
//
// Returns the arrow cursor for unknown constants.
//
// This map must be kept in sync with Flutter framework's
// services/mouse_cursor.dart.
static HCURSOR GetCursorByName(const std::string& cursor_name) {
  static auto* cursors = new std::map<std::string, const wchar_t*>{
      {"allScroll", IDC_SIZEALL},
      {"basic", IDC_ARROW},
      {"click", IDC_HAND},
      {"forbidden", IDC_NO},
      {"help", IDC_HELP},
      {"move", IDC_SIZEALL},
      {"none", nullptr},
      {"noDrop", IDC_NO},
      {"precise", IDC_CROSS},
      {"progress", IDC_APPSTARTING},
      {"text", IDC_IBEAM},
      {"resizeColumn", IDC_SIZEWE},
      {"resizeDown", IDC_SIZENS},
      {"resizeDownLeft", IDC_SIZENESW},
      {"resizeDownRight", IDC_SIZENWSE},
      {"resizeLeft", IDC_SIZEWE},
      {"resizeLeftRight", IDC_SIZEWE},
      {"resizeRight", IDC_SIZEWE},
      {"resizeRow", IDC_SIZENS},
      {"resizeUp", IDC_SIZENS},
      {"resizeUpDown", IDC_SIZENS},
      {"resizeUpLeft", IDC_SIZENWSE},
      {"resizeUpRight", IDC_SIZENESW},
      {"resizeUpLeftDownRight", IDC_SIZENWSE},
      {"resizeUpRightDownLeft", IDC_SIZENESW},
      {"wait", IDC_WAIT},
  };
  const wchar_t* idc_name = IDC_ARROW;
  auto it = cursors->find(cursor_name);
  if (it != cursors->end()) {
    idc_name = it->second;
  }
  return ::LoadCursor(nullptr, idc_name);
}

static constexpr int32_t kDefaultPointerDeviceId = 0;

// This method is only valid during a window message related to mouse/touch
// input.
// See
// https://docs.microsoft.com/en-us/windows/win32/tablet/system-events-and-mouse-messages?redirectedfrom=MSDN#distinguishing-pen-input-from-mouse-and-touch.
static FlutterPointerDeviceKind GetFlutterPointerDeviceKind() {
  constexpr LPARAM kTouchOrPenSignature = 0xFF515700;
  constexpr LPARAM kTouchSignature = kTouchOrPenSignature | 0x80;
  constexpr LPARAM kSignatureMask = 0xFFFFFF00;
  LPARAM info = GetMessageExtraInfo();
  if ((info & kSignatureMask) == kTouchOrPenSignature) {
    if ((info & kTouchSignature) == kTouchSignature) {
      return kFlutterPointerDeviceKindTouch;
    }
    return kFlutterPointerDeviceKindStylus;
  }
  return kFlutterPointerDeviceKindMouse;
}

// Translates button codes from Win32 API to FlutterPointerMouseButtons.
static uint64_t ConvertWinButtonToFlutterButton(UINT button) {
  switch (button) {
    case WM_LBUTTONDOWN:
    case WM_LBUTTONUP:
      return kFlutterPointerButtonMousePrimary;
    case WM_RBUTTONDOWN:
    case WM_RBUTTONUP:
      return kFlutterPointerButtonMouseSecondary;
    case WM_MBUTTONDOWN:
    case WM_MBUTTONUP:
      return kFlutterPointerButtonMouseMiddle;
    case XBUTTON1:
      return kFlutterPointerButtonMouseBack;
    case XBUTTON2:
      return kFlutterPointerButtonMouseForward;
  }
  FML_LOG(WARNING) << "Mouse button not recognized: " << button;
  return 0;
}

}  // namespace

FlutterWindow::FlutterWindow(
    int width,
    int height,
    std::shared_ptr<WindowsProcTable> windows_proc_table,
    std::unique_ptr<TextInputManager> text_input_manager)
    : touch_id_generator_(kMinTouchDeviceId, kMaxTouchDeviceId),
      windows_proc_table_(std::move(windows_proc_table)),
      text_input_manager_(std::move(text_input_manager)),
      ax_fragment_root_(nullptr) {
  // Get the DPI of the primary monitor as the initial DPI. If Per-Monitor V2 is
  // supported, |current_dpi_| should be updated in the
  // kWmDpiChangedBeforeParent message.
  current_dpi_ = GetDpiForHWND(nullptr);

  // Get initial value for wheel scroll lines
  // TODO: Listen to changes for this value
  // https://github.com/flutter/flutter/issues/107248
  UpdateScrollOffsetMultiplier();

  if (windows_proc_table_ == nullptr) {
    windows_proc_table_ = std::make_unique<WindowsProcTable>();
  }
  if (text_input_manager_ == nullptr) {
    text_input_manager_ = std::make_unique<TextInputManager>();
  }
  keyboard_manager_ = std::make_unique<KeyboardManager>(this);

  InitializeChild("FLUTTERVIEW", width, height);
  current_cursor_ = ::LoadCursor(nullptr, IDC_ARROW);
}

// Base constructor for mocks
FlutterWindow::FlutterWindow()
    : touch_id_generator_(kMinTouchDeviceId, kMaxTouchDeviceId) {}

FlutterWindow::~FlutterWindow() {
  Destroy();
}

void FlutterWindow::SetView(WindowBindingHandlerDelegate* window) {
  binding_handler_delegate_ = window;
  if (direct_manipulation_owner_) {
    direct_manipulation_owner_->SetBindingHandlerDelegate(window);
  }
  if (restored_ && window) {
    OnWindowStateEvent(WindowStateEvent::kShow);
  }
  if (focused_ && window) {
    OnWindowStateEvent(WindowStateEvent::kFocus);
  }
}

float FlutterWindow::GetDpiScale() {
  return static_cast<float>(GetCurrentDPI()) / static_cast<float>(base_dpi);
}

PhysicalWindowBounds FlutterWindow::GetPhysicalWindowBounds() {
  return {GetCurrentWidth(), GetCurrentHeight()};
}

void FlutterWindow::UpdateFlutterCursor(const std::string& cursor_name) {
  SetFlutterCursor(GetCursorByName(cursor_name));
}

void FlutterWindow::SetFlutterCursor(HCURSOR cursor) {
  current_cursor_ = cursor;
  ::SetCursor(current_cursor_);
}

void FlutterWindow::OnDpiScale(unsigned int dpi) {};

// When DesktopWindow notifies that a WM_Size message has come in
// lets FlutterEngine know about the new size.
void FlutterWindow::OnResize(unsigned int width, unsigned int height) {
  if (binding_handler_delegate_ != nullptr) {
    binding_handler_delegate_->OnWindowSizeChanged(width, height);
  }
}

void FlutterWindow::OnPaint() {
  if (binding_handler_delegate_ != nullptr) {
    binding_handler_delegate_->OnWindowRepaint();
  }
}

void FlutterWindow::OnPointerMove(double x,
                                  double y,
                                  FlutterPointerDeviceKind device_kind,
                                  int32_t device_id,
                                  int modifiers_state) {
  binding_handler_delegate_->OnPointerMove(x, y, device_kind, device_id,
                                           modifiers_state);
}

void FlutterWindow::OnPointerDown(double x,
                                  double y,
                                  FlutterPointerDeviceKind device_kind,
                                  int32_t device_id,
                                  UINT button) {
  uint64_t flutter_button = ConvertWinButtonToFlutterButton(button);
  if (flutter_button != 0) {
    binding_handler_delegate_->OnPointerDown(
        x, y, device_kind, device_id,
        static_cast<FlutterPointerMouseButtons>(flutter_button));
  }
}

void FlutterWindow::OnPointerUp(double x,
                                double y,
                                FlutterPointerDeviceKind device_kind,
                                int32_t device_id,
                                UINT button) {
  uint64_t flutter_button = ConvertWinButtonToFlutterButton(button);
  if (flutter_button != 0) {
    binding_handler_delegate_->OnPointerUp(
        x, y, device_kind, device_id,
        static_cast<FlutterPointerMouseButtons>(flutter_button));
  }
}

void FlutterWindow::OnPointerLeave(double x,
                                   double y,
                                   FlutterPointerDeviceKind device_kind,
                                   int32_t device_id) {
  binding_handler_delegate_->OnPointerLeave(x, y, device_kind, device_id);
}

void FlutterWindow::OnSetCursor() {
  ::SetCursor(current_cursor_);
}

void FlutterWindow::OnText(const std::u16string& text) {
  binding_handler_delegate_->OnText(text);
}

void FlutterWindow::OnKey(int key,
                          int scancode,
                          int action,
                          char32_t character,
                          bool extended,
                          bool was_down,
                          KeyEventCallback callback) {
  binding_handler_delegate_->OnKey(key, scancode, action, character, extended,
                                   was_down, std::move(callback));
}

void FlutterWindow::OnComposeBegin() {
  binding_handler_delegate_->OnComposeBegin();
}

void FlutterWindow::OnComposeCommit() {
  binding_handler_delegate_->OnComposeCommit();
}

void FlutterWindow::OnComposeEnd() {
  binding_handler_delegate_->OnComposeEnd();
}

void FlutterWindow::OnComposeChange(const std::u16string& text,
                                    int cursor_pos) {
  binding_handler_delegate_->OnComposeChange(text, cursor_pos);
}

void FlutterWindow::OnUpdateSemanticsEnabled(bool enabled) {
  binding_handler_delegate_->OnUpdateSemanticsEnabled(enabled);
}

void FlutterWindow::OnScroll(double delta_x,
                             double delta_y,
                             FlutterPointerDeviceKind device_kind,
                             int32_t device_id) {
  POINT point;
  GetCursorPos(&point);

  ScreenToClient(GetWindowHandle(), &point);
  binding_handler_delegate_->OnScroll(point.x, point.y, delta_x, delta_y,
                                      GetScrollOffsetMultiplier(), device_kind,
                                      device_id);
}

void FlutterWindow::OnCursorRectUpdated(const Rect& rect) {
  // Convert the rect from Flutter logical coordinates to device coordinates.
  auto scale = GetDpiScale();
  Point origin(rect.left() * scale, rect.top() * scale);
  Size size(rect.width() * scale, rect.height() * scale);
  UpdateCursorRect(Rect(origin, size));
}

void FlutterWindow::OnResetImeComposing() {
  AbortImeComposing();
}

bool FlutterWindow::OnBitmapSurfaceCleared() {
  HDC dc = ::GetDC(GetWindowHandle());
  bool result = ::PatBlt(dc, 0, 0, current_width_, current_height_, BLACKNESS);
  ::ReleaseDC(GetWindowHandle(), dc);
  return result;
}

bool FlutterWindow::OnBitmapSurfaceUpdated(const void* allocation,
                                           size_t row_bytes,
                                           size_t height) {
  HDC dc = ::GetDC(GetWindowHandle());
  BITMAPINFO bmi = {};
  bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
  bmi.bmiHeader.biWidth = row_bytes / 4;
  bmi.bmiHeader.biHeight = -height;
  bmi.bmiHeader.biPlanes = 1;
  bmi.bmiHeader.biBitCount = 32;
  bmi.bmiHeader.biCompression = BI_RGB;
  bmi.bmiHeader.biSizeImage = 0;
  int ret = ::SetDIBitsToDevice(dc, 0, 0, row_bytes / 4, height, 0, 0, 0,
                                height, allocation, &bmi, DIB_RGB_COLORS);
  ::ReleaseDC(GetWindowHandle(), dc);
  return ret != 0;
}

gfx::NativeViewAccessible FlutterWindow::GetNativeViewAccessible() {
  if (binding_handler_delegate_ == nullptr) {
    return nullptr;
  }

  return binding_handler_delegate_->GetNativeViewAccessible();
}

PointerLocation FlutterWindow::GetPrimaryPointerLocation() {
  POINT point;
  GetCursorPos(&point);
  ScreenToClient(GetWindowHandle(), &point);
  return {(size_t)point.x, (size_t)point.y};
}

void FlutterWindow::OnThemeChange() {
  binding_handler_delegate_->OnHighContrastChanged();
}

ui::AXFragmentRootDelegateWin* FlutterWindow::GetAxFragmentRootDelegate() {
  return binding_handler_delegate_->GetAxFragmentRootDelegate();
}

AlertPlatformNodeDelegate* FlutterWindow::GetAlertDelegate() {
  CreateAxFragmentRoot();
  return alert_delegate_.get();
}

ui::AXPlatformNodeWin* FlutterWindow::GetAlert() {
  CreateAxFragmentRoot();
  return alert_node_.get();
}

void FlutterWindow::OnWindowStateEvent(WindowStateEvent event) {
  switch (event) {
    case WindowStateEvent::kShow:
      restored_ = true;
      break;
    case WindowStateEvent::kHide:
      restored_ = false;
      focused_ = false;
      break;
    case WindowStateEvent::kFocus:
      focused_ = true;
      break;
    case WindowStateEvent::kUnfocus:
      focused_ = false;
      break;
  }
  HWND hwnd = GetWindowHandle();
  if (hwnd && binding_handler_delegate_) {
    binding_handler_delegate_->OnWindowStateEvent(hwnd, event);
  }
}

void FlutterWindow::TrackMouseLeaveEvent(HWND hwnd) {
  if (!tracking_mouse_leave_) {
    TRACKMOUSEEVENT tme;
    tme.cbSize = sizeof(tme);
    tme.hwndTrack = hwnd;
    tme.dwFlags = TME_LEAVE;
    TrackMouseEvent(&tme);
    tracking_mouse_leave_ = true;
  }
}

void FlutterWindow::HandleResize(UINT width, UINT height) {
  current_width_ = width;
  current_height_ = height;
  if (direct_manipulation_owner_) {
    direct_manipulation_owner_->ResizeViewport(width, height);
  }
  OnResize(width, height);
}

FlutterWindow* FlutterWindow::GetThisFromHandle(HWND const window) noexcept {
  return reinterpret_cast<FlutterWindow*>(
      GetWindowLongPtr(window, GWLP_USERDATA));
}

void FlutterWindow::UpdateScrollOffsetMultiplier() {
  UINT lines_per_scroll = kLinesPerScrollWindowsDefault;

  // Get lines per scroll wheel value from Windows
  SystemParametersInfo(SPI_GETWHEELSCROLLLINES, 0, &lines_per_scroll, 0);

  // This logic is based off Chromium's implementation
  // https://source.chromium.org/chromium/chromium/src/+/main:ui/events/blink/web_input_event_builders_win.cc;l=319-331
  scroll_offset_multiplier_ =
      static_cast<float>(lines_per_scroll) * 100.0 / 3.0;
}

void FlutterWindow::InitializeChild(const char* title,
                                    unsigned int width,
                                    unsigned int height) {
  Destroy();
  std::wstring converted_title = NarrowToWide(title);

  WNDCLASS window_class = RegisterWindowClass(converted_title);

  auto* result = CreateWindowEx(
      0, window_class.lpszClassName, converted_title.c_str(),
      WS_CHILD | WS_VISIBLE, CW_DEFAULT, CW_DEFAULT, width, height,
      HWND_MESSAGE, nullptr, window_class.hInstance, this);

  if (result == nullptr) {
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
  SetUserObjectInformationA(GetCurrentProcess(),
                            UOI_TIMERPROC_EXCEPTION_SUPPRESSION, FALSE, 1);
  // SetTimer is not precise, if a 16 ms interval is requested, it will instead
  // often fire in an interval of 32 ms. Providing a value of 14 will ensure it
  // runs every 16 ms, which will allow for 60 Hz trackpad gesture events, which
  // is the maximal frequency supported by SetTimer.
  SetTimer(result, kDirectManipulationTimer, 14, nullptr);
  direct_manipulation_owner_ = std::make_unique<DirectManipulationOwner>(this);
  direct_manipulation_owner_->Init(width, height);
}

HWND FlutterWindow::GetWindowHandle() {
  return window_handle_;
}

BOOL FlutterWindow::Win32PeekMessage(LPMSG lpMsg,
                                     UINT wMsgFilterMin,
                                     UINT wMsgFilterMax,
                                     UINT wRemoveMsg) {
  return ::PeekMessage(lpMsg, window_handle_, wMsgFilterMin, wMsgFilterMax,
                       wRemoveMsg);
}

uint32_t FlutterWindow::Win32MapVkToChar(uint32_t virtual_key) {
  return ::MapVirtualKey(virtual_key, MAPVK_VK_TO_CHAR);
}

UINT FlutterWindow::Win32DispatchMessage(UINT Msg,
                                         WPARAM wParam,
                                         LPARAM lParam) {
  return ::SendMessage(window_handle_, Msg, wParam, lParam);
}

std::wstring FlutterWindow::NarrowToWide(const char* source) {
  size_t length = strlen(source);
  size_t outlen = 0;
  std::wstring wideTitle(length, L'#');
  mbstowcs_s(&outlen, &wideTitle[0], length + 1, source, length);
  return wideTitle;
}

WNDCLASS FlutterWindow::RegisterWindowClass(std::wstring& title) {
  window_class_name_ = title;

  WNDCLASS window_class{};
  window_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
  window_class.lpszClassName = title.c_str();
  window_class.style = CS_HREDRAW | CS_VREDRAW;
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

LRESULT CALLBACK FlutterWindow::WndProc(HWND const window,
                                        UINT const message,
                                        WPARAM const wparam,
                                        LPARAM const lparam) noexcept {
  if (message == WM_NCCREATE) {
    auto cs = reinterpret_cast<CREATESTRUCT*>(lparam);
    SetWindowLongPtr(window, GWLP_USERDATA,
                     reinterpret_cast<LONG_PTR>(cs->lpCreateParams));

    auto that = static_cast<FlutterWindow*>(cs->lpCreateParams);
    that->window_handle_ = window;
    that->text_input_manager_->SetWindowHandle(window);
    RegisterTouchWindow(window, 0);
  } else if (FlutterWindow* that = GetThisFromHandle(window)) {
    return that->HandleMessage(message, wparam, lparam);
  }

  return DefWindowProc(window, message, wparam, lparam);
}

LRESULT
FlutterWindow::HandleMessage(UINT const message,
                             WPARAM const wparam,
                             LPARAM const lparam) noexcept {
  LPARAM result_lparam = lparam;
  int xPos = 0, yPos = 0;
  UINT width = 0, height = 0;
  UINT button_pressed = 0;
  FlutterPointerDeviceKind device_kind;

  switch (message) {
    case kWmDpiChangedBeforeParent:
      current_dpi_ = GetDpiForHWND(window_handle_);
      OnDpiScale(current_dpi_);
      return 0;
    case WM_SIZE:
      width = LOWORD(lparam);
      height = HIWORD(lparam);

      current_width_ = width;
      current_height_ = height;
      HandleResize(width, height);

      OnWindowStateEvent(width == 0 && height == 0 ? WindowStateEvent::kHide
                                                   : WindowStateEvent::kShow);
      break;
    case WM_PAINT:
      OnPaint();
      break;
    case WM_TOUCH: {
      UINT num_points = LOWORD(wparam);
      touch_points_.resize(num_points);
      auto touch_input_handle = reinterpret_cast<HTOUCHINPUT>(lparam);
      if (GetTouchInputInfo(touch_input_handle, num_points,
                            touch_points_.data(), sizeof(TOUCHINPUT))) {
        for (const auto& touch : touch_points_) {
          // Generate a mapped ID for the Windows-provided touch ID
          auto touch_id = touch_id_generator_.GetGeneratedId(touch.dwID);

          POINT pt = {TOUCH_COORD_TO_PIXEL(touch.x),
                      TOUCH_COORD_TO_PIXEL(touch.y)};
          ScreenToClient(window_handle_, &pt);
          auto x = static_cast<double>(pt.x);
          auto y = static_cast<double>(pt.y);

          if (touch.dwFlags & TOUCHEVENTF_DOWN) {
            OnPointerDown(x, y, kFlutterPointerDeviceKindTouch, touch_id,
                          WM_LBUTTONDOWN);
          } else if (touch.dwFlags & TOUCHEVENTF_MOVE) {
            OnPointerMove(x, y, kFlutterPointerDeviceKindTouch, touch_id, 0);
          } else if (touch.dwFlags & TOUCHEVENTF_UP) {
            OnPointerUp(x, y, kFlutterPointerDeviceKindTouch, touch_id,
                        WM_LBUTTONDOWN);
            OnPointerLeave(x, y, kFlutterPointerDeviceKindTouch, touch_id);
            touch_id_generator_.ReleaseNumber(touch.dwID);
          }
        }
        CloseTouchInputHandle(touch_input_handle);
      }
      return 0;
    }
    case WM_MOUSEMOVE:
      device_kind = GetFlutterPointerDeviceKind();
      if (device_kind == kFlutterPointerDeviceKindMouse) {
        TrackMouseLeaveEvent(window_handle_);

        xPos = GET_X_LPARAM(lparam);
        yPos = GET_Y_LPARAM(lparam);
        mouse_x_ = static_cast<double>(xPos);
        mouse_y_ = static_cast<double>(yPos);

        int mods = 0;
        if (wparam & MK_CONTROL) {
          mods |= kControl;
        }
        if (wparam & MK_SHIFT) {
          mods |= kShift;
        }
        OnPointerMove(mouse_x_, mouse_y_, device_kind, kDefaultPointerDeviceId,
                      mods);
      }
      break;
    case WM_MOUSELEAVE:
      device_kind = GetFlutterPointerDeviceKind();
      if (device_kind == kFlutterPointerDeviceKindMouse) {
        OnPointerLeave(mouse_x_, mouse_y_, device_kind,
                       kDefaultPointerDeviceId);
      }

      // Once the tracked event is received, the TrackMouseEvent function
      // resets. Set to false to make sure it's called once mouse movement is
      // detected again.
      tracking_mouse_leave_ = false;
      break;
    case WM_SETCURSOR: {
      UINT hit_test_result = LOWORD(lparam);
      if (hit_test_result == HTCLIENT) {
        OnSetCursor();
        return TRUE;
      }
      break;
    }
    case WM_SETFOCUS:
      OnWindowStateEvent(WindowStateEvent::kFocus);
      ::CreateCaret(window_handle_, nullptr, 1, 1);
      break;
    case WM_KILLFOCUS:
      OnWindowStateEvent(WindowStateEvent::kUnfocus);
      ::DestroyCaret();
      break;
    case WM_LBUTTONDOWN:
    case WM_RBUTTONDOWN:
    case WM_MBUTTONDOWN:
    case WM_XBUTTONDOWN:
      device_kind = GetFlutterPointerDeviceKind();
      if (device_kind != kFlutterPointerDeviceKindMouse) {
        break;
      }

      if (message == WM_LBUTTONDOWN) {
        // Capture the pointer in case the user drags outside the client area.
        // In this case, the "mouse leave" event is delayed until the user
        // releases the button. It's only activated on left click given that
        // it's more common for apps to handle dragging with only the left
        // button.
        SetCapture(window_handle_);
      }
      button_pressed = message;
      if (message == WM_XBUTTONDOWN) {
        button_pressed = GET_XBUTTON_WPARAM(wparam);
      }
      xPos = GET_X_LPARAM(lparam);
      yPos = GET_Y_LPARAM(lparam);
      OnPointerDown(static_cast<double>(xPos), static_cast<double>(yPos),
                    device_kind, kDefaultPointerDeviceId, button_pressed);
      break;
    case WM_LBUTTONUP:
    case WM_RBUTTONUP:
    case WM_MBUTTONUP:
    case WM_XBUTTONUP:
      device_kind = GetFlutterPointerDeviceKind();
      if (device_kind != kFlutterPointerDeviceKindMouse) {
        break;
      }

      if (message == WM_LBUTTONUP) {
        ReleaseCapture();
      }
      button_pressed = message;
      if (message == WM_XBUTTONUP) {
        button_pressed = GET_XBUTTON_WPARAM(wparam);
      }
      xPos = GET_X_LPARAM(lparam);
      yPos = GET_Y_LPARAM(lparam);
      OnPointerUp(static_cast<double>(xPos), static_cast<double>(yPos),
                  device_kind, kDefaultPointerDeviceId, button_pressed);
      break;
    case WM_MOUSEWHEEL:
      OnScroll(0.0,
               -(static_cast<short>(HIWORD(wparam)) /
                 static_cast<double>(WHEEL_DELTA)),
               kFlutterPointerDeviceKindMouse, kDefaultPointerDeviceId);
      break;
    case WM_MOUSEHWHEEL:
      OnScroll((static_cast<short>(HIWORD(wparam)) /
                static_cast<double>(WHEEL_DELTA)),
               0.0, kFlutterPointerDeviceKindMouse, kDefaultPointerDeviceId);
      break;
    case WM_GETOBJECT: {
      LRESULT lresult = OnGetObject(message, wparam, lparam);
      if (lresult) {
        return lresult;
      }
      break;
    }
    case WM_TIMER:
      if (wparam == kDirectManipulationTimer) {
        direct_manipulation_owner_->Update();
        return 0;
      }
      break;
    case DM_POINTERHITTEST: {
      if (direct_manipulation_owner_) {
        UINT contact_id = GET_POINTERID_WPARAM(wparam);
        POINTER_INPUT_TYPE pointer_type;
        if (windows_proc_table_->GetPointerType(contact_id, &pointer_type) &&
            pointer_type == PT_TOUCHPAD) {
          direct_manipulation_owner_->SetContact(contact_id);
        }
      }
      break;
    }
    case WM_INPUTLANGCHANGE:
      // TODO(cbracken): pass this to TextInputManager to aid with
      // language-specific issues.
      break;
    case WM_IME_SETCONTEXT:
      OnImeSetContext(message, wparam, lparam);
      // Strip the ISC_SHOWUICOMPOSITIONWINDOW bit from lparam before passing it
      // to DefWindowProc() so that the composition window is hidden since
      // Flutter renders the composing string itself.
      result_lparam &= ~ISC_SHOWUICOMPOSITIONWINDOW;
      break;
    case WM_IME_STARTCOMPOSITION:
      OnImeStartComposition(message, wparam, lparam);
      // Suppress further processing by DefWindowProc() so that the default
      // system IME style isn't used, but rather the one set in the
      // WM_IME_SETCONTEXT handler.
      return TRUE;
    case WM_IME_COMPOSITION:
      OnImeComposition(message, wparam, lparam);
      if (lparam & GCS_RESULTSTR || lparam & GCS_COMPSTR) {
        // Suppress further processing by DefWindowProc() since otherwise it
        // will emit the result string as WM_CHAR messages on commit. Instead,
        // committing the composing text to the EditableText string is handled
        // in TextInputModel::CommitComposing, triggered by
        // OnImeEndComposition().
        return TRUE;
      }
      break;
    case WM_IME_ENDCOMPOSITION:
      OnImeEndComposition(message, wparam, lparam);
      return TRUE;
    case WM_IME_REQUEST:
      OnImeRequest(message, wparam, lparam);
      break;
    case WM_UNICHAR: {
      // Tell third-pary app, we can support Unicode.
      if (wparam == UNICODE_NOCHAR)
        return TRUE;
      // DefWindowProc will send WM_CHAR for this WM_UNICHAR.
      break;
    }
    case WM_THEMECHANGED:
      OnThemeChange();
      break;
    case WM_DEADCHAR:
    case WM_SYSDEADCHAR:
    case WM_CHAR:
    case WM_SYSCHAR:
    case WM_KEYDOWN:
    case WM_SYSKEYDOWN:
    case WM_KEYUP:
    case WM_SYSKEYUP:
      if (keyboard_manager_->HandleMessage(message, wparam, lparam)) {
        return 0;
      }
      break;
  }

  return Win32DefWindowProc(window_handle_, message, wparam, result_lparam);
}

LRESULT FlutterWindow::OnGetObject(UINT const message,
                                   WPARAM const wparam,
                                   LPARAM const lparam) {
  LRESULT reference_result = static_cast<LRESULT>(0L);

  // Only the lower 32 bits of lparam are valid when checking the object id
  // because it sometimes gets sign-extended incorrectly (but not always).
  DWORD obj_id = static_cast<DWORD>(static_cast<DWORD_PTR>(lparam));

  bool is_uia_request = static_cast<DWORD>(UiaRootObjectId) == obj_id;
  bool is_msaa_request = static_cast<DWORD>(OBJID_CLIENT) == obj_id;

  if (is_uia_request || is_msaa_request) {
    // On Windows, we don't get a notification that the screen reader has been
    // enabled or disabled. There is an API to query for screen reader state,
    // but that state isn't set by all screen readers, including by Narrator,
    // the screen reader that ships with Windows:
    // https://docs.microsoft.com/en-us/windows/win32/winauto/screen-reader-parameter
    //
    // Instead, we enable semantics in Flutter if Windows issues queries for
    // Microsoft Active Accessibility (MSAA) COM objects.
    OnUpdateSemanticsEnabled(true);
  }

  gfx::NativeViewAccessible root_view = GetNativeViewAccessible();
  // TODO(schectman): UIA is currently disabled by default.
  // https://github.com/flutter/flutter/issues/114547
  if (root_view) {
    CreateAxFragmentRoot();
    if (is_uia_request) {
#ifdef FLUTTER_ENGINE_USE_UIA
      // Retrieve UIA object for the root view.
      Microsoft::WRL::ComPtr<IRawElementProviderSimple> root;
      if (SUCCEEDED(
              ax_fragment_root_->GetNativeViewAccessible()->QueryInterface(
                  IID_PPV_ARGS(&root)))) {
        // Return the UIA object via UiaReturnRawElementProvider(). See:
        // https://docs.microsoft.com/en-us/windows/win32/winauto/wm-getobject
        reference_result = UiaReturnRawElementProvider(window_handle_, wparam,
                                                       lparam, root.Get());
      } else {
        FML_LOG(ERROR) << "Failed to query AX fragment root.";
      }
#endif  // FLUTTER_ENGINE_USE_UIA
    } else if (is_msaa_request) {
      // Create the accessibility root if it does not already exist.
      // Return the IAccessible for the root view.
      Microsoft::WRL::ComPtr<IAccessible> root;
      ax_fragment_root_->GetNativeViewAccessible()->QueryInterface(
          IID_PPV_ARGS(&root));
      reference_result = LresultFromObject(IID_IAccessible, wparam, root.Get());
    }
  }
  return reference_result;
}

void FlutterWindow::OnImeSetContext(UINT const message,
                                    WPARAM const wparam,
                                    LPARAM const lparam) {
  if (wparam != 0) {
    text_input_manager_->CreateImeWindow();
  }
}

void FlutterWindow::OnImeStartComposition(UINT const message,
                                          WPARAM const wparam,
                                          LPARAM const lparam) {
  text_input_manager_->CreateImeWindow();
  OnComposeBegin();
}

void FlutterWindow::OnImeComposition(UINT const message,
                                     WPARAM const wparam,
                                     LPARAM const lparam) {
  // Update the IME window position.
  text_input_manager_->UpdateImeWindow();

  if (lparam == 0) {
    OnComposeChange(u"", 0);
    OnComposeCommit();
  }

  // Process GCS_RESULTSTR at fisrt, because Google Japanese Input and ATOK send
  // both GCS_RESULTSTR and GCS_COMPSTR to commit composed text and send new
  // composing text.
  if (lparam & GCS_RESULTSTR) {
    // Commit but don't end composing.
    // Read the committed composing string.
    long pos = text_input_manager_->GetComposingCursorPosition();
    std::optional<std::u16string> text = text_input_manager_->GetResultString();
    if (text) {
      OnComposeChange(text.value(), pos);
      OnComposeCommit();
    }
  }
  if (lparam & GCS_COMPSTR) {
    // Read the in-progress composing string.
    long pos = text_input_manager_->GetComposingCursorPosition();
    std::optional<std::u16string> text =
        text_input_manager_->GetComposingString();
    if (text) {
      OnComposeChange(text.value(), pos);
    }
  }
}

void FlutterWindow::OnImeEndComposition(UINT const message,
                                        WPARAM const wparam,
                                        LPARAM const lparam) {
  text_input_manager_->DestroyImeWindow();
  OnComposeEnd();
}

void FlutterWindow::OnImeRequest(UINT const message,
                                 WPARAM const wparam,
                                 LPARAM const lparam) {
  // TODO(cbracken): Handle IMR_RECONVERTSTRING, IMR_DOCUMENTFEED,
  // and IMR_QUERYCHARPOSITION messages.
  // https://github.com/flutter/flutter/issues/74547
}

void FlutterWindow::AbortImeComposing() {
  text_input_manager_->AbortComposing();
}

void FlutterWindow::UpdateCursorRect(const Rect& rect) {
  text_input_manager_->UpdateCaretRect(rect);
}

UINT FlutterWindow::GetCurrentDPI() {
  return current_dpi_;
}

UINT FlutterWindow::GetCurrentWidth() {
  return current_width_;
}

UINT FlutterWindow::GetCurrentHeight() {
  return current_height_;
}

float FlutterWindow::GetScrollOffsetMultiplier() {
  return scroll_offset_multiplier_;
}

LRESULT FlutterWindow::Win32DefWindowProc(HWND hWnd,
                                          UINT Msg,
                                          WPARAM wParam,
                                          LPARAM lParam) {
  return ::DefWindowProc(hWnd, Msg, wParam, lParam);
}

void FlutterWindow::Destroy() {
  if (window_handle_) {
    text_input_manager_->SetWindowHandle(nullptr);
    DestroyWindow(window_handle_);
    window_handle_ = nullptr;
  }

  UnregisterClass(window_class_name_.c_str(), nullptr);
}

void FlutterWindow::CreateAxFragmentRoot() {
  if (ax_fragment_root_) {
    return;
  }
  ax_fragment_root_ = std::make_unique<ui::AXFragmentRootWin>(
      window_handle_, GetAxFragmentRootDelegate());
  alert_delegate_ =
      std::make_unique<AlertPlatformNodeDelegate>(*ax_fragment_root_);
  ui::AXPlatformNode* alert_node =
      ui::AXPlatformNodeWin::Create(alert_delegate_.get());
  alert_node_.reset(static_cast<ui::AXPlatformNodeWin*>(alert_node));
  ax_fragment_root_->SetAlertNode(alert_node_.get());
}

}  // namespace flutter

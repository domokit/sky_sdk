// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/flutter_host_window.h"

#include <dwmapi.h>

#include "flutter/shell/platform/windows/dpi_utils.h"
#include "flutter/shell/platform/windows/flutter_host_window_controller.h"
#include "flutter/shell/platform/windows/flutter_window.h"
#include "flutter/shell/platform/windows/flutter_windows_view_controller.h"

namespace {

constexpr wchar_t kWindowClassName[] = L"FLUTTER_HOST_WINDOW";

// Dynamically loads the |EnableNonClientDpiScaling| from the User32 module
// so that the non-client area automatically responds to changes in DPI.
// This API is only needed for PerMonitor V1 awareness mode.
void EnableFullDpiSupportIfAvailable(HWND hwnd) {
  HMODULE user32_module = LoadLibraryA("User32.dll");
  if (!user32_module) {
    return;
  }

  using EnableNonClientDpiScaling = BOOL __stdcall(HWND hwnd);

  auto enable_non_client_dpi_scaling =
      reinterpret_cast<EnableNonClientDpiScaling*>(
          GetProcAddress(user32_module, "EnableNonClientDpiScaling"));
  if (enable_non_client_dpi_scaling != nullptr) {
    enable_non_client_dpi_scaling(hwnd);
  }

  FreeLibrary(user32_module);
}

// Dynamically loads |SetWindowCompositionAttribute| from the User32 module to
// make the window's background transparent.
void EnableTransparentWindowBackground(HWND hwnd) {
  HMODULE const user32_module = LoadLibraryA("User32.dll");
  if (!user32_module) {
    return;
  }

  enum WINDOWCOMPOSITIONATTRIB { WCA_ACCENT_POLICY = 19 };

  struct WINDOWCOMPOSITIONATTRIBDATA {
    WINDOWCOMPOSITIONATTRIB Attrib;
    PVOID pvData;
    SIZE_T cbData;
  };

  using SetWindowCompositionAttribute =
      BOOL(__stdcall*)(HWND, WINDOWCOMPOSITIONATTRIBDATA*);

  auto set_window_composition_attribute =
      reinterpret_cast<SetWindowCompositionAttribute>(
          GetProcAddress(user32_module, "SetWindowCompositionAttribute"));
  if (set_window_composition_attribute != nullptr) {
    enum ACCENT_STATE { ACCENT_DISABLED = 0 };

    struct ACCENT_POLICY {
      ACCENT_STATE AccentState;
      DWORD AccentFlags;
      DWORD GradientColor;
      DWORD AnimationId;
    };

    // Set the accent policy to disable window composition.
    ACCENT_POLICY accent = {ACCENT_DISABLED, 2, static_cast<DWORD>(0), 0};
    WINDOWCOMPOSITIONATTRIBDATA data = {.Attrib = WCA_ACCENT_POLICY,
                                        .pvData = &accent,
                                        .cbData = sizeof(accent)};
    set_window_composition_attribute(hwnd, &data);

    // Extend the frame into the client area and set the window's system
    // backdrop type for visual effects.
    MARGINS const margins = {-1};
    ::DwmExtendFrameIntoClientArea(hwnd, &margins);
    INT effect_value = 1;
    ::DwmSetWindowAttribute(hwnd, DWMWA_SYSTEMBACKDROP_TYPE, &effect_value,
                            sizeof(BOOL));
  }

  FreeLibrary(user32_module);
}

// Computes the screen-space anchor rectangle for a window being positioned
// with |positioner|, having |owner_hwnd| as owner, and |owner_rect|
// as the owner's client rectangle, also in screen space. If the positioner
// specifies an anchor rectangle (in logical coordinates), its coordinates are
// scaled using the owner's DPI and offset relative to |owner_rect|.
// Otherwise, the function defaults to using the window frame of |owner_hwnd|
// as the anchor rectangle.
flutter::WindowRectangle GetAnchorRectInScreenSpace(
    flutter::WindowPositioner const& positioner,
    HWND owner_hwnd,
    flutter::WindowRectangle const& owner_rect) {
  if (positioner.anchor_rect) {
    double const dpr = flutter::GetDpiForHWND(owner_hwnd) /
                       static_cast<double>(USER_DEFAULT_SCREEN_DPI);
    return {{owner_rect.top_left.x +
                 static_cast<int>(positioner.anchor_rect->top_left.x * dpr),
             owner_rect.top_left.y +
                 static_cast<int>(positioner.anchor_rect->top_left.y * dpr)},
            {static_cast<int>(positioner.anchor_rect->size.width * dpr),
             static_cast<int>(positioner.anchor_rect->size.height * dpr)}};
  } else {
    // If the anchor rectangle specified in the positioner is std::nullopt,
    // return an anchor rectangle that is equal to the owner's frame.
    RECT frame_rect;
    DwmGetWindowAttribute(owner_hwnd, DWMWA_EXTENDED_FRAME_BOUNDS, &frame_rect,
                          sizeof(frame_rect));
    return {{frame_rect.left, frame_rect.top},
            {frame_rect.right - frame_rect.left,
             frame_rect.bottom - frame_rect.top}};
  }
}

// Calculates the client area of |hwnd| in screen space.
flutter::WindowRectangle GetClientRectInScreenSpace(HWND hwnd) {
  RECT client_rect;
  GetClientRect(hwnd, &client_rect);
  POINT top_left = {0, 0};
  ClientToScreen(hwnd, &top_left);
  POINT bottom_right = {client_rect.right, client_rect.bottom};
  ClientToScreen(hwnd, &bottom_right);
  return {{top_left.x, top_left.y},
          {bottom_right.x - top_left.x, bottom_right.y - top_left.y}};
}

// Calculates the size of the window frame in physical coordinates, based on
// the given |window_size| (also in physical coordinates) and the specified
// |window_style|, |extended_window_style|, and owner window |owner_hwnd|.
flutter::WindowSize GetFrameSizeForWindowSize(
    flutter::WindowSize const& window_size,
    DWORD window_style,
    DWORD extended_window_style,
    HWND owner_hwnd) {
  RECT frame_rect = {0, 0, static_cast<LONG>(window_size.width),
                     static_cast<LONG>(window_size.height)};

  HINSTANCE hInstance = GetModuleHandle(nullptr);
  WNDCLASS window_class = {};
  window_class.lpfnWndProc = DefWindowProc;
  window_class.hInstance = hInstance;
  window_class.lpszClassName = L"FLUTTER_HOST_WINDOW_TEMPORARY";
  RegisterClass(&window_class);

  window_style &= ~WS_VISIBLE;
  if (HWND const window = CreateWindowEx(
          extended_window_style, window_class.lpszClassName, L"", window_style,
          CW_USEDEFAULT, CW_USEDEFAULT, window_size.width, window_size.height,
          owner_hwnd, nullptr, hInstance, nullptr)) {
    DwmGetWindowAttribute(window, DWMWA_EXTENDED_FRAME_BOUNDS, &frame_rect,
                          sizeof(frame_rect));
    DestroyWindow(window);
  }

  UnregisterClass(window_class.lpszClassName, hInstance);

  return {static_cast<int>(frame_rect.right - frame_rect.left),
          static_cast<int>(frame_rect.bottom - frame_rect.top)};
}

// Retrieves the calling thread's last-error code message as a string,
// or a fallback message if the error message cannot be formatted.
std::string GetLastErrorAsString() {
  LPWSTR message_buffer = nullptr;

  if (DWORD const size = FormatMessage(
          FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
              FORMAT_MESSAGE_IGNORE_INSERTS,
          nullptr, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
          reinterpret_cast<LPTSTR>(&message_buffer), 0, nullptr)) {
    std::wstring const wide_message(message_buffer, size);
    LocalFree(message_buffer);
    message_buffer = nullptr;

    if (int const buffer_size =
            WideCharToMultiByte(CP_UTF8, 0, wide_message.c_str(), -1, nullptr,
                                0, nullptr, nullptr)) {
      std::string message(buffer_size, 0);
      WideCharToMultiByte(CP_UTF8, 0, wide_message.c_str(), -1, &message[0],
                          buffer_size, nullptr, nullptr);
      return message;
    }
  }

  if (message_buffer) {
    LocalFree(message_buffer);
  }
  std::ostringstream oss;
  oss << "Format message failed with 0x" << std::hex << std::setfill('0')
      << std::setw(8) << GetLastError();
  return oss.str();
}

// Calculates the rectangle of the monitor that has the largest area of
// intersection with |rect|, in physical coordinates.
flutter::WindowRectangle GetOutputRect(RECT rect) {
  HMONITOR monitor = MonitorFromRect(&rect, MONITOR_DEFAULTTONEAREST);
  MONITORINFO mi;
  mi.cbSize = sizeof(MONITORINFO);
  RECT const bounds =
      GetMonitorInfo(monitor, &mi) ? mi.rcWork : RECT{0, 0, 0, 0};
  return {{bounds.left, bounds.top},
          {bounds.right - bounds.left, bounds.bottom - bounds.top}};
}

// Calculates the required window size, in physical coordinates, to
// accommodate the given |client_size|, in logical coordinates, for a window
// with the specified |window_style| and |extended_window_style|. The result
// accounts for window borders, non-client areas, and the drop-shadow area.
flutter::WindowSize GetWindowSizeForClientSize(
    flutter::WindowSize const& client_size,
    DWORD window_style,
    DWORD extended_window_style,
    HWND owner_hwnd) {
  UINT const dpi = flutter::GetDpiForHWND(owner_hwnd);
  double const scale_factor =
      static_cast<double>(dpi) / USER_DEFAULT_SCREEN_DPI;
  RECT rect = {.left = 0,
               .top = 0,
               .right = static_cast<LONG>(client_size.width * scale_factor),
               .bottom = static_cast<LONG>(client_size.height * scale_factor)};

  HMODULE const user32_module = LoadLibraryA("User32.dll");
  if (user32_module) {
    using AdjustWindowRectExForDpi = BOOL __stdcall(
        LPRECT lpRect, DWORD dwStyle, BOOL bMenu, DWORD dwExStyle, UINT dpi);

    auto* const adjust_window_rect_ext_for_dpi =
        reinterpret_cast<AdjustWindowRectExForDpi*>(
            GetProcAddress(user32_module, "AdjustWindowRectExForDpi"));
    if (adjust_window_rect_ext_for_dpi) {
      if (adjust_window_rect_ext_for_dpi(&rect, window_style, FALSE,
                                         extended_window_style, dpi)) {
        FreeLibrary(user32_module);
        return {static_cast<int>(rect.right - rect.left),
                static_cast<int>(rect.bottom - rect.top)};
      } else {
        FML_LOG(WARNING) << "Failed to run AdjustWindowRectExForDpi: "
                         << GetLastErrorAsString();
      }
    } else {
      FML_LOG(WARNING)
          << "Failed to retrieve AdjustWindowRectExForDpi address from "
             "User32.dll.";
    }
    FreeLibrary(user32_module);
  } else {
    FML_LOG(WARNING) << "Failed to load User32.dll.\n";
  }

  if (!AdjustWindowRectEx(&rect, window_style, FALSE, extended_window_style)) {
    FML_LOG(WARNING) << "Failed to run AdjustWindowRectEx: "
                     << GetLastErrorAsString();
  }
  return {static_cast<int>(rect.right - rect.left),
          static_cast<int>(rect.bottom - rect.top)};
}

// Checks whether the window class of name |class_name| is registered for the
// current application.
bool IsClassRegistered(LPCWSTR class_name) {
  WNDCLASSEX window_class = {};
  return GetClassInfoEx(GetModuleHandle(nullptr), class_name, &window_class) !=
         0;
}

// Window attribute that enables dark mode window decorations.
//
// Redefined in case the developer's machine has a Windows SDK older than
// version 10.0.22000.0.
// See:
// https://docs.microsoft.com/windows/win32/api/dwmapi/ne-dwmapi-dwmwindowattribute
#ifndef DWMWA_USE_IMMERSIVE_DARK_MODE
#define DWMWA_USE_IMMERSIVE_DARK_MODE 20
#endif

// Update the window frame's theme to match the system theme.
void UpdateTheme(HWND window) {
  // Registry key for app theme preference.
  const wchar_t kGetPreferredBrightnessRegKey[] =
      L"Software\\Microsoft\\Windows\\CurrentVersion\\Themes\\Personalize";
  const wchar_t kGetPreferredBrightnessRegValue[] = L"AppsUseLightTheme";

  // A value of 0 indicates apps should use dark mode. A non-zero or missing
  // value indicates apps should use light mode.
  DWORD light_mode;
  DWORD light_mode_size = sizeof(light_mode);
  LSTATUS const result =
      RegGetValue(HKEY_CURRENT_USER, kGetPreferredBrightnessRegKey,
                  kGetPreferredBrightnessRegValue, RRF_RT_REG_DWORD, nullptr,
                  &light_mode, &light_mode_size);

  if (result == ERROR_SUCCESS) {
    BOOL enable_dark_mode = light_mode == 0;
    DwmSetWindowAttribute(window, DWMWA_USE_IMMERSIVE_DARK_MODE,
                          &enable_dark_mode, sizeof(enable_dark_mode));
  }
}

}  // namespace

namespace flutter {

FlutterHostWindow::FlutterHostWindow(FlutterHostWindowController* controller,
                                     std::wstring const& title,
                                     WindowSize const& preferred_client_size,
                                     WindowArchetype archetype,
                                     std::optional<HWND> owner,
                                     std::optional<WindowPositioner> positioner)
    : window_controller_(controller) {
  archetype_ = archetype;

  // Check preconditions and set window styles based on window type.
  DWORD window_style = 0;
  DWORD extended_window_style = 0;
  switch (archetype) {
    case WindowArchetype::regular:
      if (owner.has_value()) {
        FML_LOG(ERROR) << "A regular window cannot have an owner.";
        return;
      }
      if (positioner.has_value()) {
        FML_LOG(ERROR) << "A regular window cannot have a positioner.";
        return;
      }
      window_style |= WS_OVERLAPPEDWINDOW;
      break;
    case WindowArchetype::popup:
      if (!positioner.has_value()) {
        FML_LOG(ERROR) << "A popup window requires a positioner.";
        return;
      }
      if (!owner.has_value()) {
        FML_LOG(ERROR) << "A popup window must have an owner.";
        return;
      }
      window_style |= WS_POPUP;
      break;
    default:
      FML_UNREACHABLE();
  }

  // Calculate the screen space window rectangle for the new window.
  // Default positioning values (CW_USEDEFAULT) are used
  // if the window has no owner or positioner.
  WindowRectangle const window_rect = [&]() -> WindowRectangle {
    WindowSize const window_size = GetWindowSizeForClientSize(
        preferred_client_size, window_style, extended_window_style,
        owner.value_or(nullptr));
    if (owner) {
      if (positioner) {
        // Calculate the window rectangle according to a positioner and
        // the owner's rectangle.
        WindowSize const frame_size = GetFrameSizeForWindowSize(
            window_size, window_style, extended_window_style, owner.value());

        WindowRectangle const owner_rect =
            GetClientRectInScreenSpace(owner.value());

        WindowRectangle const anchor_rect = GetAnchorRectInScreenSpace(
            positioner.value(), owner.value(), owner_rect);

        WindowRectangle const output_rect = GetOutputRect(
            {.left = static_cast<LONG>(anchor_rect.top_left.x),
             .top = static_cast<LONG>(anchor_rect.top_left.y),
             .right = static_cast<LONG>(anchor_rect.top_left.x +
                                        anchor_rect.size.width),
             .bottom = static_cast<LONG>(anchor_rect.top_left.y +
                                         anchor_rect.size.height)});

        WindowRectangle const rect = PlaceWindow(
            positioner.value(), frame_size, anchor_rect,
            positioner->anchor_rect ? owner_rect : anchor_rect, output_rect);

        return {rect.top_left,
                {rect.size.width + window_size.width - frame_size.width,
                 rect.size.height + window_size.height - frame_size.height}};
      }
    }
    return {{CW_USEDEFAULT, CW_USEDEFAULT}, window_size};
  }();

  // Register the window class.
  if (!IsClassRegistered(kWindowClassName)) {
    auto const idi_app_icon = 101;
    WNDCLASSEX window_class = {};
    window_class.cbSize = sizeof(WNDCLASSEX);
    window_class.style = CS_HREDRAW | CS_VREDRAW;
    window_class.lpfnWndProc = FlutterHostWindow::WndProc;
    window_class.hInstance = GetModuleHandle(nullptr);
    window_class.hIcon =
        LoadIcon(window_class.hInstance, MAKEINTRESOURCE(idi_app_icon));
    if (!window_class.hIcon) {
      window_class.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
    }
    window_class.hCursor = LoadCursor(nullptr, IDC_ARROW);
    window_class.lpszClassName = kWindowClassName;

    if (!RegisterClassEx(&window_class)) {
      FML_LOG(ERROR) << "Cannot register window class " << kWindowClassName
                     << ": " << GetLastErrorAsString();
    }
  }

  // Create the native window.
  HWND hwnd = CreateWindowEx(
      extended_window_style, kWindowClassName, title.c_str(), window_style,
      window_rect.top_left.x, window_rect.top_left.y, window_rect.size.width,
      window_rect.size.height, owner.value_or(nullptr), nullptr,
      GetModuleHandle(nullptr), this);

  if (!hwnd) {
    FML_LOG(ERROR) << "Cannot create window: " << GetLastErrorAsString();
    return;
  }

  // Adjust the window position so its origin aligns with the top-left corner
  // of the window frame, not the window rectangle (which includes the
  // drop-shadow). This adjustment must be done post-creation since the frame
  // rectangle is only available after the window has been created.
  RECT frame_rc;
  DwmGetWindowAttribute(hwnd, DWMWA_EXTENDED_FRAME_BOUNDS, &frame_rc,
                        sizeof(frame_rc));
  RECT window_rc;
  GetWindowRect(hwnd, &window_rc);
  LONG const left_dropshadow_width = frame_rc.left - window_rc.left;
  LONG const top_dropshadow_height = window_rc.top - frame_rc.top;
  SetWindowPos(hwnd, nullptr, window_rc.left - left_dropshadow_width,
               window_rc.top - top_dropshadow_height, 0, 0,
               SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE);

  // Set up the view.
  RECT client_rect;
  GetClientRect(hwnd, &client_rect);
  int const width = client_rect.right - client_rect.left;
  int const height = client_rect.bottom - client_rect.top;

  FlutterWindowsEngine* const engine = window_controller_->engine();
  auto view_window = std::make_unique<FlutterWindow>(
      width, height, engine->windows_proc_table());

  std::unique_ptr<FlutterWindowsView> view =
      engine->CreateView(std::move(view_window));
  if (!view) {
    FML_LOG(ERROR) << "Failed to create view";
    return;
  }

  view_controller_ =
      std::make_unique<FlutterWindowsViewController>(nullptr, std::move(view));

  // Launch the engine if it is not running already.
  if (!engine->running() && !engine->Run()) {
    FML_LOG(ERROR) << "Failed to launch engine";
    return;
  }
  // Must happen after engine is running.
  view_controller_->view()->SendInitialBounds();
  // The Windows embedder listens to accessibility updates using the
  // view's HWND. The embedder's accessibility features may be stale if
  // the app was in headless mode.
  view_controller_->engine()->UpdateAccessibilityFeatures();

  // Ensure that basic setup of the view controller was successful.
  if (!view_controller_->view()) {
    FML_LOG(ERROR) << "Failed to set up the view controller";
    return;
  }

  // Update the properties of the owner window, if it exists.
  if (FlutterHostWindow* const owner_window =
          GetThisFromHandle(owner.value_or(nullptr))) {
    owner_window->owned_windows_.insert(this);

    if (archetype == WindowArchetype::popup) {
      ++owner_window->num_owned_popups_;
    }
  }

  UpdateTheme(hwnd);

  SetChildContent(view_controller_->view()->GetWindowHandle());

  // TODO(loicsharma): Hide the window until the first frame is rendered.
  // Single window apps use the engine's next frame callback to show the window.
  // This doesn't work for multi window apps as the engine cannot have multiple
  // next frame callbacks. If multiple windows are created, only the last one
  // will be shown.
  ShowWindow(hwnd, SW_SHOW);

  window_handle_ = hwnd;
}

FlutterHostWindow::~FlutterHostWindow() {
  if (HWND const hwnd = window_handle_) {
    window_handle_ = nullptr;
    DestroyWindow(hwnd);

    // Unregisters the window class. It will fail silently if there are
    // other windows using the class, as only the last window can
    // successfully unregister the class.
    if (!UnregisterClass(kWindowClassName, GetModuleHandle(nullptr))) {
      // Clears the error information after the failed unregistering.
      SetLastError(ERROR_SUCCESS);
    }
  }
}

FlutterHostWindow* FlutterHostWindow::GetThisFromHandle(HWND hwnd) {
  return reinterpret_cast<FlutterHostWindow*>(
      GetWindowLongPtr(hwnd, GWLP_USERDATA));
}

WindowArchetype FlutterHostWindow::GetArchetype() const {
  return archetype_;
}

std::set<FlutterHostWindow*> const& FlutterHostWindow::GetOwnedWindows() const {
  return owned_windows_;
}

std::optional<FlutterViewId> FlutterHostWindow::GetFlutterViewId() const {
  if (!view_controller_ || !view_controller_->view()) {
    return std::nullopt;
  }
  return view_controller_->view()->view_id();
};

FlutterHostWindow* FlutterHostWindow::GetOwnerWindow() const {
  if (HWND const owner_window_handle = GetWindow(GetWindowHandle(), GW_OWNER)) {
    return GetThisFromHandle(owner_window_handle);
  }
  return nullptr;
};

HWND FlutterHostWindow::GetWindowHandle() const {
  return window_handle_;
}

void FlutterHostWindow::SetQuitOnClose(bool quit_on_close) {
  quit_on_close_ = quit_on_close;
}

bool FlutterHostWindow::GetQuitOnClose() const {
  return quit_on_close_;
}

void FlutterHostWindow::FocusViewOf(FlutterHostWindow* window) {
  if (window != nullptr && window->child_content_ != nullptr) {
    SetFocus(window->child_content_);
  }
};

LRESULT FlutterHostWindow::WndProc(HWND hwnd,
                                   UINT message,
                                   WPARAM wparam,
                                   LPARAM lparam) {
  if (message == WM_NCCREATE) {
    auto* const create_struct = reinterpret_cast<CREATESTRUCT*>(lparam);
    SetWindowLongPtr(hwnd, GWLP_USERDATA,
                     reinterpret_cast<LONG_PTR>(create_struct->lpCreateParams));
    auto* const window =
        static_cast<FlutterHostWindow*>(create_struct->lpCreateParams);
    window->window_handle_ = hwnd;

    EnableFullDpiSupportIfAvailable(hwnd);
    EnableTransparentWindowBackground(hwnd);
  } else if (FlutterHostWindow* const window = GetThisFromHandle(hwnd)) {
    return window->window_controller_->HandleMessage(hwnd, message, wparam,
                                                     lparam);
  }

  return DefWindowProc(hwnd, message, wparam, lparam);
}

std::size_t FlutterHostWindow::CloseOwnedPopups() {
  if (num_owned_popups_ == 0) {
    return 0;
  }

  std::set<FlutterHostWindow*> popups;
  for (FlutterHostWindow* const owned : owned_windows_) {
    if (owned->archetype_ == WindowArchetype::popup) {
      popups.insert(owned);
    }
  }

  for (auto it = owned_windows_.begin(); it != owned_windows_.end();) {
    if ((*it)->archetype_ == WindowArchetype::popup) {
      it = owned_windows_.erase(it);
    } else {
      ++it;
    }
  }

  std::size_t const previous_num_owned_popups = num_owned_popups_;

  for (FlutterHostWindow* popup : popups) {
    HWND const owner_handle = GetWindow(popup->window_handle_, GW_OWNER);
    if (FlutterHostWindow* const owner = GetThisFromHandle(owner_handle)) {
      // Popups' owners are drawn with active colors even though they are
      // actually inactive. When a popup is destroyed, the owner might be
      // redrawn as inactive (reflecting its true state) before being redrawn as
      // active. To prevent flickering during this transition, disable
      // redrawing the non-client area as inactive.
      owner->enable_redraw_non_client_as_inactive_ = false;
      DestroyWindow(popup->GetWindowHandle());
      owner->enable_redraw_non_client_as_inactive_ = true;

      // Repaint owner window to make sure its title bar is painted with the
      // color based on its actual activation state.
      if (owner->num_owned_popups_ == 0) {
        SetWindowPos(owner_handle, nullptr, 0, 0, 0, 0,
                     SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_FRAMECHANGED);
      }
    }
  }

  return previous_num_owned_popups - num_owned_popups_;
}

FlutterHostWindow* FlutterHostWindow::FindFirstEnabledDescendant() const {
  if (IsWindowEnabled(GetWindowHandle())) {
    return const_cast<FlutterHostWindow*>(this);
  }

  for (FlutterHostWindow* const owned : GetOwnedWindows()) {
    if (FlutterHostWindow* const result = owned->FindFirstEnabledDescendant()) {
      return result;
    }
  }

  return nullptr;
}

LRESULT FlutterHostWindow::HandleMessage(HWND hwnd,
                                         UINT message,
                                         WPARAM wparam,
                                         LPARAM lparam) {
  switch (message) {
    case WM_DESTROY:
      if (window_handle_) {
        switch (archetype_) {
          case WindowArchetype::regular:
            break;
          case WindowArchetype::popup:
            if (FlutterHostWindow* const owner_window = GetOwnerWindow()) {
              owner_window->owned_windows_.erase(this);
              assert(owner_window->num_owned_popups_ > 0);
              --owner_window->num_owned_popups_;
              FocusViewOf(owner_window);
            }
            break;
          default:
            FML_UNREACHABLE();
        }
        if (quit_on_close_) {
          PostQuitMessage(0);
        }
      }
      return 0;

    case WM_DPICHANGED: {
      auto* const new_scaled_window_rect = reinterpret_cast<RECT*>(lparam);
      LONG const width =
          new_scaled_window_rect->right - new_scaled_window_rect->left;
      LONG const height =
          new_scaled_window_rect->bottom - new_scaled_window_rect->top;
      SetWindowPos(hwnd, nullptr, new_scaled_window_rect->left,
                   new_scaled_window_rect->top, width, height,
                   SWP_NOZORDER | SWP_NOACTIVATE);
      return 0;
    }

    case WM_SIZE: {
      if (child_content_ != nullptr) {
        // Resize and reposition the child content window
        RECT client_rect;
        GetClientRect(hwnd, &client_rect);
        MoveWindow(child_content_, client_rect.left, client_rect.top,
                   client_rect.right - client_rect.left,
                   client_rect.bottom - client_rect.top, TRUE);
      }
      return 0;
    }

    case WM_ACTIVATE:
      // Prevent disabled window from being activated using the task switcher
      if (!IsWindowEnabled(hwnd) && LOWORD(wparam) != WA_INACTIVE) {
        // Redirect focus and activation to the first enabled descendant
        if (FlutterHostWindow* enabled_descendant =
                FindFirstEnabledDescendant()) {
          SetActiveWindow(enabled_descendant->GetWindowHandle());
        }
        return 0;
      }
      FocusViewOf(this);
      return 0;

    case WM_NCACTIVATE:
      if (wparam == FALSE && archetype_ != WindowArchetype::popup) {
        if (!enable_redraw_non_client_as_inactive_ || num_owned_popups_ > 0) {
          // If an inactive title bar is to be drawn, and this is a top-level
          // window with popups, force the title bar to be drawn in its active
          // colors.
          return TRUE;
        }
      }
      break;

    case WM_MOUSEACTIVATE:
      FocusViewOf(this);
      return MA_ACTIVATE;

    case WM_DWMCOLORIZATIONCOLORCHANGED:
      UpdateTheme(hwnd);
      return 0;

    default:
      break;
  }

  return DefWindowProc(hwnd, message, wparam, lparam);
}

void FlutterHostWindow::SetChildContent(HWND content) {
  child_content_ = content;
  SetParent(content, window_handle_);
  RECT client_rect;
  GetClientRect(window_handle_, &client_rect);
  MoveWindow(content, client_rect.left, client_rect.top,
             client_rect.right - client_rect.left,
             client_rect.bottom - client_rect.top, true);
}

}  // namespace flutter

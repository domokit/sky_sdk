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

// RAII wrapper for global Win32 ATOMs.
struct AtomRAII {
  explicit AtomRAII(wchar_t const* name) : atom(GlobalAddAtom(name)) {}
  ~AtomRAII() { GlobalDeleteAtom(atom); }
  ATOM const atom;
};

// Atom used as the identifier for a window property that stores a pointer to a
// |FlutterHostWindow| instance.
AtomRAII const kWindowPropAtom(kWindowClassName);

// Clamps |size| to the size of the virtual screen. Both the parameter and
// return size are in physical coordinates.
flutter::Size ClampToVirtualScreen(flutter::Size size) {
  double const virtual_screen_width = GetSystemMetrics(SM_CXVIRTUALSCREEN);
  double const virtual_screen_height = GetSystemMetrics(SM_CYVIRTUALSCREEN);

  return flutter::Size(std::clamp(size.width(), 0.0, virtual_screen_width),
                       std::clamp(size.height(), 0.0, virtual_screen_height));
}

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

// Calculates the required window size, in physical coordinates, to
// accommodate the given |client_size|, in logical coordinates, constrained by
// optional |min_size| and |max_size|, for a window with the specified
// |window_style| and |extended_window_style|. If |owner_hwnd| is not null, the
// DPI of the display with the largest area of intersection with |owner_hwnd| is
// used for the calculation; otherwise, the primary display's DPI is used. The
// resulting size includes window borders, non-client areas, and drop shadows.
// On error, returns std::nullopt and logs an error message.
std::optional<flutter::Size> GetWindowSizeForClientSize(
    flutter::Size const& client_size,
    std::optional<flutter::Size> min_size,
    std::optional<flutter::Size> max_size,
    DWORD window_style,
    DWORD extended_window_style,
    HWND owner_hwnd) {
  UINT const dpi = flutter::GetDpiForHWND(owner_hwnd);
  double const scale_factor =
      static_cast<double>(dpi) / USER_DEFAULT_SCREEN_DPI;
  RECT rect = {
      .right = static_cast<LONG>(client_size.width() * scale_factor),
      .bottom = static_cast<LONG>(client_size.height() * scale_factor)};

  HMODULE const user32_raw = LoadLibraryA("User32.dll");
  auto free_user32_module = [](HMODULE module) { FreeLibrary(module); };
  std::unique_ptr<std::remove_pointer_t<HMODULE>, decltype(free_user32_module)>
      user32_module(user32_raw, free_user32_module);
  if (!user32_module) {
    FML_LOG(ERROR) << "Failed to load User32.dll.\n";
    return std::nullopt;
  }

  using AdjustWindowRectExForDpi = BOOL __stdcall(
      LPRECT lpRect, DWORD dwStyle, BOOL bMenu, DWORD dwExStyle, UINT dpi);
  auto* const adjust_window_rect_ext_for_dpi =
      reinterpret_cast<AdjustWindowRectExForDpi*>(
          GetProcAddress(user32_raw, "AdjustWindowRectExForDpi"));
  if (!adjust_window_rect_ext_for_dpi) {
    FML_LOG(ERROR) << "Failed to retrieve AdjustWindowRectExForDpi address "
                      "from User32.dll.";
    return std::nullopt;
  }

  if (!adjust_window_rect_ext_for_dpi(&rect, window_style, FALSE,
                                      extended_window_style, dpi)) {
    FML_LOG(ERROR) << "Failed to run AdjustWindowRectExForDpi: "
                   << GetLastErrorAsString();
    return std::nullopt;
  }

  double width = static_cast<double>(rect.right - rect.left);
  double height = static_cast<double>(rect.bottom - rect.top);

  // Apply size constraints
  double const non_client_width = width - (client_size.width() * scale_factor);
  double const non_client_height =
      height - (client_size.height() * scale_factor);
  if (min_size) {
    flutter::Size min_physical_size = ClampToVirtualScreen(
        flutter::Size(min_size->width() * scale_factor + non_client_width,
                      min_size->height() * scale_factor + non_client_height));
    width = std::max(width, min_physical_size.width());
    height = std::max(height, min_physical_size.height());
  }
  if (max_size) {
    flutter::Size max_physical_size = ClampToVirtualScreen(
        flutter::Size(max_size->width() * scale_factor + non_client_width,
                      max_size->height() * scale_factor + non_client_height));
    width = std::min(width, max_physical_size.width());
    height = std::min(height, max_physical_size.height());
  }

  return flutter::Size{width, height};
}

// Checks whether the window class of name |class_name| is registered for the
// current application.
bool IsClassRegistered(LPCWSTR class_name) {
  WNDCLASSEX window_class = {};
  return GetClassInfoEx(GetModuleHandle(nullptr), class_name, &window_class) !=
         0;
}

// Converts std::string to std::wstring.
std::wstring StringToWstring(std::string_view str) {
  if (str.empty()) {
    return {};
  }
  if (int buffer_size =
          MultiByteToWideChar(CP_UTF8, 0, str.data(), str.size(), nullptr, 0)) {
    std::wstring wide_str(buffer_size, L'\0');
    if (MultiByteToWideChar(CP_UTF8, 0, str.data(), str.size(), &wide_str[0],
                            buffer_size)) {
      return wide_str;
    }
  }
  return {};
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

// Updates the window frame's theme to match the system theme.
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

// Associates |instance| with the window |hwnd| as a window property.
// Can be retrieved later using GetInstanceProperty.
// Logs an error if setting the property fails.
void SetInstanceProperty(HWND hwnd, flutter::FlutterHostWindow* instance) {
  if (!SetProp(hwnd, MAKEINTATOM(kWindowPropAtom.atom), instance)) {
    FML_LOG(ERROR) << "Failed to set up instance entry in the property list: "
                   << GetLastErrorAsString();
  }
}

// Retrieves the instance pointer set with SetInstanceProperty, or returns
// nullptr if the property was not set.
flutter::FlutterHostWindow* GetInstanceProperty(HWND hwnd) {
  return reinterpret_cast<flutter::FlutterHostWindow*>(
      GetProp(hwnd, MAKEINTATOM(kWindowPropAtom.atom)));
}

// Removes the instance property associated with |hwnd| previously set with
// SetInstanceProperty. Logs an error if the property is not found.
void RemoveInstanceProperty(HWND hwnd) {
  if (!RemoveProp(hwnd, MAKEINTATOM(kWindowPropAtom.atom))) {
    FML_LOG(ERROR) << "Failed to locate instance entry in the property list";
  }
}

}  // namespace

namespace flutter {

FlutterHostWindow::FlutterHostWindow(FlutterHostWindowController* controller,
                                     WindowCreationSettings const& settings)
    : window_controller_(controller) {
  archetype_ = settings.archetype;

  // Check preconditions and set window styles based on window type.
  DWORD window_style = 0;
  DWORD extended_window_style = 0;
  switch (archetype_) {
    case WindowArchetype::kRegular:
      window_style |= WS_OVERLAPPEDWINDOW;
      break;
    default:
      FML_UNREACHABLE();
  }

  // Validate size constraints.
  min_size_ = settings.min_size;
  max_size_ = settings.max_size;
  if (min_size_ && max_size_) {
    if (min_size_->width() > max_size_->width() ||
        min_size_->height() > max_size_->height()) {
      FML_LOG(ERROR) << "Invalid size constraints.";
      return;
    }
  }

  // Calculate the screen space window rectangle for the new window.
  // Default positioning values (CW_USEDEFAULT) are used
  // if the window has no owner.
  Rect const initial_window_rect = [&]() -> Rect {
    std::optional<Size> const window_size = GetWindowSizeForClientSize(
        settings.size, min_size_, max_size_, window_style,
        extended_window_style, nullptr);
    return {{CW_USEDEFAULT, CW_USEDEFAULT},
            window_size ? *window_size : Size{CW_USEDEFAULT, CW_USEDEFAULT}};
  }();

  // Set up the view.
  FlutterWindowsEngine* const engine = window_controller_->engine();
  auto view_window = std::make_unique<FlutterWindow>(
      initial_window_rect.width(), initial_window_rect.height(),
      engine->windows_proc_table());

  std::unique_ptr<FlutterWindowsView> view =
      engine->CreateView(std::move(view_window));
  if (!view) {
    FML_LOG(ERROR) << "Failed to create view";
    return;
  }

  view_controller_ =
      std::make_unique<FlutterWindowsViewController>(nullptr, std::move(view));
  FML_CHECK(engine->running());
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
      return;
    }
  }

  // Create the native window.
  HWND hwnd = CreateWindowEx(
      extended_window_style, kWindowClassName,
      StringToWstring(settings.title.value_or("")).c_str(), window_style,
      initial_window_rect.left(), initial_window_rect.top(),
      initial_window_rect.width(), initial_window_rect.height(), nullptr,
      nullptr, GetModuleHandle(nullptr), this);

  if (!hwnd) {
    FML_LOG(ERROR) << "Cannot create window: " << GetLastErrorAsString();
    return;
  }

  // Adjust the window position so its origin aligns with the top-left corner
  // of the window frame, not the window rectangle (which includes the
  // drop-shadow). This adjustment must be done post-creation since the frame
  // rectangle is only available after the window has been created.
  RECT frame_rect;
  DwmGetWindowAttribute(hwnd, DWMWA_EXTENDED_FRAME_BOUNDS, &frame_rect,
                        sizeof(frame_rect));
  RECT window_rect;
  GetWindowRect(hwnd, &window_rect);
  LONG const left_dropshadow_width = frame_rect.left - window_rect.left;
  LONG const top_dropshadow_height = window_rect.top - frame_rect.top;
  SetWindowPos(hwnd, nullptr, window_rect.left - left_dropshadow_width,
               window_rect.top - top_dropshadow_height, 0, 0,
               SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE);

  UpdateTheme(hwnd);

  SetChildContent(view_controller_->view()->GetWindowHandle());

  state_ = settings.state.value_or(WindowState::kRestored);

  // TODO(loicsharma): Hide the window until the first frame is rendered.
  // Single window apps use the engine's next frame callback to show the
  // window. This doesn't work for multi window apps as the engine cannot have
  // multiple next frame callbacks. If multiple windows are created, only the
  // last one will be shown.
  UINT const cmd_show = [&]() {
    if (archetype_ == WindowArchetype::kRegular) {
      switch (state_) {
        case WindowState::kRestored:
          return SW_SHOWNORMAL;
          break;
        case WindowState::kMaximized:
          return SW_SHOWMAXIMIZED;
          break;
        case WindowState::kMinimized:
          return SW_SHOWMINIMIZED;
          break;
        default:
          FML_UNREACHABLE();
      }
    }
    return SW_SHOWNORMAL;
  }();
  ShowWindow(hwnd, cmd_show);

  window_handle_ = hwnd;
}

FlutterHostWindow::FlutterHostWindow(FlutterHostWindowController* controller,
                                     HWND hwnd,
                                     FlutterWindowsView* view)
    : window_controller_(controller), window_handle_(hwnd) {
  SetInstanceProperty(hwnd, this);
  FML_CHECK(view != nullptr);
  child_content_ = view->GetWindowHandle();
}

FlutterHostWindow::~FlutterHostWindow() {
  RemoveInstanceProperty(window_handle_);
  HWND const hwnd = std::exchange(window_handle_, nullptr);

  if (view_controller_) {
    DestroyWindow(hwnd);
    // Unregister the window class. Fail silently if other windows are still
    // using the class, as only the last window can successfully unregister it.
    if (!UnregisterClass(kWindowClassName, GetModuleHandle(nullptr))) {
      // Clear the error state after the failed unregistration.
      SetLastError(ERROR_SUCCESS);
    }
  }
}

FlutterHostWindow* FlutterHostWindow::GetThisFromHandle(HWND hwnd) {
  return GetInstanceProperty(hwnd);
}

HWND FlutterHostWindow::GetWindowHandle() const {
  return window_handle_;
}

WindowState FlutterHostWindow::GetState() const {
  return state_;
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
    auto* const window =
        static_cast<FlutterHostWindow*>(create_struct->lpCreateParams);
    SetInstanceProperty(hwnd, window);
    window->window_handle_ = hwnd;

    EnableFullDpiSupportIfAvailable(hwnd);
    EnableTransparentWindowBackground(hwnd);
  } else if (FlutterHostWindow* const window = GetThisFromHandle(hwnd)) {
    return window->window_controller_->HandleMessage(hwnd, message, wparam,
                                                     lparam);
  }

  return DefWindowProc(hwnd, message, wparam, lparam);
}

LRESULT FlutterHostWindow::HandleMessage(HWND hwnd,
                                         UINT message,
                                         WPARAM wparam,
                                         LPARAM lparam) {
  if (window_handle_ && view_controller_) {
    LRESULT* result;
    if (view_controller_->engine()->lifecycle_manager()->WindowProc(
            hwnd, message, wparam, lparam, result)) {
      return 0;
    }
  }

  switch (message) {
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

    case WM_GETMINMAXINFO: {
      RECT window_rect;
      GetWindowRect(hwnd, &window_rect);
      RECT client_rect;
      GetClientRect(hwnd, &client_rect);
      LONG const non_client_width = (window_rect.right - window_rect.left) -
                                    (client_rect.right - client_rect.left);
      LONG const non_client_height = (window_rect.bottom - window_rect.top) -
                                     (client_rect.bottom - client_rect.top);

      UINT const dpi = flutter::GetDpiForHWND(hwnd);
      double const scale_factor =
          static_cast<double>(dpi) / USER_DEFAULT_SCREEN_DPI;

      MINMAXINFO* info = reinterpret_cast<MINMAXINFO*>(lparam);
      if (min_size_) {
        Size const min_physical_size = ClampToVirtualScreen(
            Size(min_size_->width() * scale_factor + non_client_width,
                 min_size_->height() * scale_factor + non_client_height));

        info->ptMinTrackSize.x = min_physical_size.width();
        info->ptMinTrackSize.y = min_physical_size.height();
      }
      if (max_size_) {
        Size const max_physical_size = ClampToVirtualScreen(
            Size(max_size_->width() * scale_factor + non_client_width,
                 max_size_->height() * scale_factor + non_client_height));

        info->ptMaxTrackSize.x = max_physical_size.width();
        info->ptMaxTrackSize.y = max_physical_size.height();
      }
      return 0;
    }

    case WM_SIZE: {
      if (child_content_ != nullptr) {
        // Resize and reposition the child content window.
        RECT client_rect;
        GetClientRect(hwnd, &client_rect);
        MoveWindow(child_content_, client_rect.left, client_rect.top,
                   client_rect.right - client_rect.left,
                   client_rect.bottom - client_rect.top, TRUE);
      }
      return 0;
    }

    case WM_ACTIVATE:
      FocusViewOf(this);
      return 0;

    case WM_MOUSEACTIVATE:
      FocusViewOf(this);
      return MA_ACTIVATE;

    case WM_DWMCOLORIZATIONCOLORCHANGED:
      UpdateTheme(hwnd);
      return 0;

    default:
      break;
  }

  if (!view_controller_) {
    return 0;
  }

  return DefWindowProc(hwnd, message, wparam, lparam);
}

void FlutterHostWindow::SetClientSize(Size const& client_size) const {
  WINDOWINFO window_info = {.cbSize = sizeof(WINDOWINFO)};
  GetWindowInfo(window_handle_, &window_info);

  std::optional<Size> const window_size = GetWindowSizeForClientSize(
      client_size, min_size_, max_size_, window_info.dwStyle,
      window_info.dwExStyle, nullptr);

  Size const size = window_size.value_or(client_size);
  SetWindowPos(window_handle_, NULL, 0, 0, size.width(), size.height(),
               SWP_NOMOVE | SWP_NOZORDER | SWP_NOACTIVATE);
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

void FlutterHostWindow::SetState(WindowState state) {
  WINDOWPLACEMENT window_placement = {.length = sizeof(WINDOWPLACEMENT)};
  GetWindowPlacement(window_handle_, &window_placement);
  window_placement.showCmd = [&]() {
    switch (state) {
      case WindowState::kRestored:
        return SW_RESTORE;
      case WindowState::kMaximized:
        return SW_MAXIMIZE;
      case WindowState::kMinimized:
        return SW_MINIMIZE;
      default:
        FML_UNREACHABLE();
    };
  }();
  SetWindowPlacement(window_handle_, &window_placement);
  state_ = state;
}

void FlutterHostWindow::SetTitle(std::string_view title) const {
  std::wstring title_wide = StringToWstring(title);
  SetWindowText(window_handle_, title_wide.c_str());
}

}  // namespace flutter

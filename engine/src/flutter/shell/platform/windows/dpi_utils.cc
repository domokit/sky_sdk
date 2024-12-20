// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "dpi_utils.h"

#include "flutter/fml/macros.h"

namespace flutter {

namespace {

constexpr UINT kDefaultDpi = 96;

// This is the MDT_EFFECTIVE_DPI value from MONITOR_DPI_TYPE, an enum declared
// in ShellScalingApi.h. Replicating here to avoid importing the library
// directly.
constexpr UINT kEffectiveDpiMonitorType = 0;

template <typename T>

/// Retrieves a function |name| from a given |comBaseModule| into |outProc|.
/// Returns a bool indicating whether the function was found.
bool AssignProcAddress(HMODULE comBaseModule, const char* name, T*& outProc) {
  outProc = reinterpret_cast<T*>(GetProcAddress(comBaseModule, name));
  return *outProc != nullptr;
}

/// A helper class for abstracting various Windows DPI related functions across
/// Windows OS versions.
class DpiHelper {
 public:
  DpiHelper();

  ~DpiHelper();

  /// Returns the DPI for |hwnd|. Supports all DPI awareness modes, and is
  /// backward compatible down to Windows Vista. If |hwnd| is nullptr, returns
  /// the DPI for the primary monitor. If Per-Monitor DPI awareness is not
  /// available, returns the system's DPI.
  UINT GetDpiForWindow(HWND);

  /// Returns the DPI of a given monitor. Defaults to 96 if the API is not
  /// available.
  UINT GetDpiForMonitor(HMONITOR);

 private:
  using GetDpiForWindow_ = UINT __stdcall(HWND);
  using GetDpiForMonitor_ = HRESULT __stdcall(HMONITOR hmonitor,
                                              UINT dpiType,
                                              UINT* dpiX,
                                              UINT* dpiY);
  using EnableNonClientDpiScaling_ = BOOL __stdcall(HWND hwnd);

  GetDpiForWindow_* get_dpi_for_window_ = nullptr;
  GetDpiForMonitor_* get_dpi_for_monitor_ = nullptr;
  EnableNonClientDpiScaling_* enable_non_client_dpi_scaling_ = nullptr;

  HMODULE user32_module_ = nullptr;
  HMODULE shlib_module_ = nullptr;
  bool dpi_for_window_supported_ = false;
  bool dpi_for_monitor_supported_ = false;

  FML_DISALLOW_COPY_AND_ASSIGN(DpiHelper);
};

DpiHelper::DpiHelper() {
  if ((user32_module_ = LoadLibraryA("User32.dll")) != nullptr) {
    dpi_for_window_supported_ = (AssignProcAddress(
        user32_module_, "GetDpiForWindow", get_dpi_for_window_));
  }
  if ((shlib_module_ = LoadLibraryA("Shcore.dll")) != nullptr) {
    dpi_for_monitor_supported_ = AssignProcAddress(
        shlib_module_, "GetDpiForMonitor", get_dpi_for_monitor_);
  }
}

DpiHelper::~DpiHelper() {
  if (user32_module_ != nullptr) {
    FreeLibrary(user32_module_);
  }
  if (shlib_module_ != nullptr) {
    FreeLibrary(shlib_module_);
  }
}

UINT DpiHelper::GetDpiForWindow(HWND hwnd) {
  // GetDpiForWindow returns the DPI for any awareness mode. If not available,
  // or no |hwnd| is provided, fallback to a per monitor, system, or default
  // DPI.
  if (dpi_for_window_supported_ && hwnd != nullptr) {
    return get_dpi_for_window_(hwnd);
  }

  if (dpi_for_monitor_supported_) {
    HMONITOR monitor = nullptr;
    if (hwnd != nullptr) {
      monitor = MonitorFromWindow(hwnd, MONITOR_DEFAULTTOPRIMARY);
    }
    return GetDpiForMonitor(monitor);
  }
  HDC hdc = GetDC(hwnd);
  UINT dpi = GetDeviceCaps(hdc, LOGPIXELSX);
  ReleaseDC(hwnd, hdc);
  return dpi;
}

UINT DpiHelper::GetDpiForMonitor(HMONITOR monitor) {
  if (dpi_for_monitor_supported_) {
    if (monitor == nullptr) {
      const POINT target_point = {0, 0};
      monitor = MonitorFromPoint(target_point, MONITOR_DEFAULTTOPRIMARY);
    }
    UINT dpi_x = 0, dpi_y = 0;
    HRESULT result =
        get_dpi_for_monitor_(monitor, kEffectiveDpiMonitorType, &dpi_x, &dpi_y);
    if (SUCCEEDED(result)) {
      return dpi_x;
    }
  }
  return kDefaultDpi;
}  // namespace

DpiHelper* GetHelper() {
  static DpiHelper* dpi_helper = new DpiHelper();
  return dpi_helper;
}
}  // namespace

UINT GetDpiForHWND(HWND hwnd) {
  return GetHelper()->GetDpiForWindow(hwnd);
}

UINT GetDpiForMonitor(HMONITOR monitor) {
  return GetHelper()->GetDpiForMonitor(monitor);
}
}  // namespace flutter

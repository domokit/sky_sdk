// Copyright (c) 2011 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/win/display.h"

namespace base {
namespace win {

namespace {

template <typename T>
bool AssignProcAddress(HMODULE comBaseModule, const char* name, T*& outProc) {
  outProc = reinterpret_cast<T*>(GetProcAddress(comBaseModule, name));
  return *outProc != nullptr;
}

// Helper class for supporting display scale factor lookup across Windows
// versions, with fallbacks where these lookups are unavailable.
class ScaleHelperWin32 {
 public:
  ScaleHelperWin32();
  ~ScaleHelperWin32();

  /// Returns the scale factor for the specified monitor. Sets |scale| to
  /// SCALE_100_PERCENT if the API is not available.
  HRESULT GetScaleFactorForMonitor(HMONITOR hmonitor,
                                   DEVICE_SCALE_FACTOR* scale) const;

 private:
  using GetScaleFactorForMonitor_ =
      HRESULT __stdcall(HMONITOR hmonitor, DEVICE_SCALE_FACTOR* scale);

  GetScaleFactorForMonitor_* get_scale_factor_for_monitor_ = nullptr;

  HMODULE shlib_module_ = nullptr;
  bool scale_factor_for_monitor_supported_ = false;
};

ScaleHelperWin32::ScaleHelperWin32() {
  if ((shlib_module_ = LoadLibraryA("Shcore.dll")) != nullptr) {
    scale_factor_for_monitor_supported_ =
        AssignProcAddress(shlib_module_, "GetScaleFactorForMonitor",
                          get_scale_factor_for_monitor_);
  }
}

ScaleHelperWin32::~ScaleHelperWin32() {
  if (shlib_module_ != nullptr) {
    FreeLibrary(shlib_module_);
  }
}

HRESULT ScaleHelperWin32::GetScaleFactorForMonitor(
    HMONITOR hmonitor,
    DEVICE_SCALE_FACTOR* scale) const {
  if (hmonitor == nullptr || scale == nullptr) {
    return E_INVALIDARG;
  }
  if (!scale_factor_for_monitor_supported_) {
    *scale = SCALE_100_PERCENT;
    return S_OK;
  }
  return get_scale_factor_for_monitor_(hmonitor, scale);
}

ScaleHelperWin32* GetHelper() {
  static ScaleHelperWin32* helper = new ScaleHelperWin32();
  return helper;
}

}  // namespace

float GetScaleFactorForHWND(HWND hwnd) {
  HMONITOR monitor = MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST);
  DEVICE_SCALE_FACTOR scale = DEVICE_SCALE_FACTOR_INVALID;
  if (SUCCEEDED(GetHelper()->GetScaleFactorForMonitor(monitor, &scale))) {
    return ScaleFactorToFloat(scale);
  }
  return 1.0f;
}

float ScaleFactorToFloat(DEVICE_SCALE_FACTOR scale_factor) {
  switch (scale_factor) {
    case SCALE_100_PERCENT:
      return 1.0f;
    case SCALE_120_PERCENT:
      return 1.2f;
    case SCALE_125_PERCENT:
      return 1.25f;
    case SCALE_140_PERCENT:
      return 1.4f;
    case SCALE_150_PERCENT:
      return 1.5f;
    case SCALE_160_PERCENT:
      return 1.6f;
    case SCALE_175_PERCENT:
      return 1.75f;
    case SCALE_180_PERCENT:
      return 1.8f;
    case SCALE_200_PERCENT:
      return 2.0f;
    case SCALE_225_PERCENT:
      return 2.25f;
    case SCALE_250_PERCENT:
      return 2.5f;
    case SCALE_300_PERCENT:
      return 3.0f;
    case SCALE_350_PERCENT:
      return 3.5f;
    case SCALE_400_PERCENT:
      return 4.0f;
    case SCALE_450_PERCENT:
      return 4.5f;
    case SCALE_500_PERCENT:
      return 5.0f;
    default:
      return 1.0f;
  }
}

}  // namespace win
}  // namespace base

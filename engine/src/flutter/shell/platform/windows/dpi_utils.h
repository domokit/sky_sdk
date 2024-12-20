// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "Windows.h"

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_DPI_UTILS_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_DPI_UTILS_H_

namespace flutter {

/// Returns the DPI for |hwnd|. Supports all DPI awareness modes, and is
/// backward compatible down to Windows Vista. If |hwnd| is nullptr, returns the
/// DPI for the primary monitor. If Per-Monitor DPI awareness is not available,
/// returns the system's DPI.
UINT GetDpiForHWND(HWND hwnd);

/// Returns the DPI of a given monitor. Defaults to 96 if the API is not
/// available.
UINT GetDpiForMonitor(HMONITOR monitor);

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_DPI_UTILS_H_

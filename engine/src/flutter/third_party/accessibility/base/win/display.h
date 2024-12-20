// Copyright (c) 2011 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_WIN_DISPLAY_H_
#define BASE_WIN_DISPLAY_H_

#include <windows.h>

#include <shellscalingapi.h>

#include "base/base_export.h"

namespace base {
namespace win {

float GetScaleFactorForHWND(HWND hwnd);
float ScaleFactorToFloat(DEVICE_SCALE_FACTOR scale_factor);

}  // namespace win
}  // namespace base

#endif  // BASE_WIN_DISPLAY_H_

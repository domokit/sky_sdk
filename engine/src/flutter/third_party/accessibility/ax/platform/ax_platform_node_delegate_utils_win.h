// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_DELEGATE_UTILS_WIN_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_DELEGATE_UTILS_WIN_H_

#include "ax/ax_export.h"

namespace ui {

class AXPlatformNodeDelegate;

// Returns true if the value pattern is supported
AX_EXPORT bool IsValuePatternSupported(AXPlatformNodeDelegate*);

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_DELEGATE_UTILS_WIN_H_

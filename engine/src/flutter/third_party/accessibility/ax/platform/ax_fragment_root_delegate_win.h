// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_FRAGMENT_ROOT_DELEGATE_WIN_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_FRAGMENT_ROOT_DELEGATE_WIN_H_

#include "flutter/third_party/accessibility/gfx/native_widget_types.h"

namespace ui {

// Delegate interface for clients of AXFragmentRootWin. This allows the client
// to relate the fragment root to its neighbors in a loosely coupled way.
class AXFragmentRootDelegateWin {
 public:
  // In our design, a fragment root can have at most one child.
  // See AXFragmentRootWin for more details.
  virtual gfx::NativeViewAccessible GetChildOfAXFragmentRoot() = 0;

  // Optionally returns a parent node for the fragment root. This is used, for
  // example, to place the web content fragment at the correct spot in the
  // browser UI's accessibility tree.
  // If a fragment root returns no parent, the OS will use HWND parent-child
  // relationships to establish the fragment root's location in the tree.
  virtual gfx::NativeViewAccessible GetParentOfAXFragmentRoot() = 0;

  // Return true if the window should be treated as an accessible control or
  // false if the window should be considered a structural detail that should
  // not be exposed to assistive technology users. See AXFragmentRootWin for
  // more details.
  virtual bool IsAXFragmentRootAControlElement() = 0;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_FRAGMENT_ROOT_DELEGATE_WIN_H_

// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_CLIPPING_BEHAVIOR_H_
#define UI_ACCESSIBILITY_AX_CLIPPING_BEHAVIOR_H_

namespace ui {

// The clipping behavior to perform on bounds. Clipping limits a node's bounding
// box to the visible sizes of it's ancestors - which may be hidden or scrolled
// out of view. For a longer discussion on clipping behavior see the link below.
// https://chromium.googlesource.com/chromium/src/+/lkgr/docs/accessibility/offscreen.md
//   kUnclipped: Do not apply clipping to bound results
//   kClipped:   Apply clipping to bound results
enum class AXClippingBehavior { kUnclipped, kClipped };
}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_CLIPPING_BEHAVIOR_H_

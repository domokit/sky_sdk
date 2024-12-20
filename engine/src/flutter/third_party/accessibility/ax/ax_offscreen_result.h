// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_OFFSCREEN_RESULT_H_
#define UI_ACCESSIBILITY_AX_OFFSCREEN_RESULT_H_

namespace ui {

// The onscreen state of result bounds or points. Any object is offscreen if
// it is fully clipped or scrolled out of view by any of its ancestors so that
// it is not rendered on the screen. For a longer discussion on what offscreen
// means in the context of Chromium see the link below.
// https://chromium.googlesource.com/chromium/src/+/lkgr/docs/accessibility/offscreen.md
//   kOnscreen:  The resulting bound or point is onscreen
//   kOffscreen: The resulting bound or point is offscreen
enum class AXOffscreenResult { kOnscreen, kOffscreen };
}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_OFFSCREEN_RESULT_H_

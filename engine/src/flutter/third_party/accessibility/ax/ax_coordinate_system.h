// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_COORDINATE_SYSTEM_H_
#define UI_ACCESSIBILITY_AX_COORDINATE_SYSTEM_H_

namespace ui {

// The coordinate system of bounds or points. The origin for all coordinate
// systems is the upper left hand corner of the region. Frame coordinates
// correspond to the current frame and root frame coordinates are relative to
// the topmost accessibility tree of the same type. For web content, root frame
// coordinates are relative to the root frame of the web page. From within an
// accessibility tree whose root is an iframe, frame coordinates are relative to
// the region of the iframe. From an iframe leaf accessibility node, frame
// coordinates are relative to the containing accessibility tree. For native UI,
// frame coordinates are relative to the current window whereas root frame
// coordinates are relative to the top-level window. The frame coordinates are
// equivalent to the root frame coordinates when the current accessibility tree
// is the root accessibility tree.
//   kScreenPhysicalPixels: Relative to screen space in hardware pixels
//   kScreenDIPs:     Relative to screen space in device-independent pixels
//                    (i.e. accounting for display DPI)
//   kRootFrame:      Relative to the top-level accessibility tree of
//                    the same type
//   kFrame:          Relative to the current accessibility tree
enum class AXCoordinateSystem {
  kScreenPhysicalPixels,
  kScreenDIPs,
  kRootFrame,
  kFrame
};
}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_COORDINATE_SYSTEM_H_

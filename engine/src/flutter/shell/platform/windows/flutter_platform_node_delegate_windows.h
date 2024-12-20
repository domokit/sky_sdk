// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_PLATFORM_NODE_DELEGATE_WINDOWS_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_PLATFORM_NODE_DELEGATE_WINDOWS_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/flutter_platform_node_delegate.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"
#include "flutter/third_party/accessibility/ax/platform/ax_platform_node.h"
#include "flutter/third_party/accessibility/ax/platform/ax_unique_id.h"

namespace flutter {

// The Windows implementation of FlutterPlatformNodeDelegate.
//
// This class implements a wrapper around the Windows accessibility objects
// that compose the accessibility tree.
class FlutterPlatformNodeDelegateWindows : public FlutterPlatformNodeDelegate {
 public:
  FlutterPlatformNodeDelegateWindows(std::weak_ptr<AccessibilityBridge> bridge,
                                     FlutterWindowsView* view);
  virtual ~FlutterPlatformNodeDelegateWindows();

  // |ui::AXPlatformNodeDelegate|
  void Init(std::weak_ptr<OwnerBridge> bridge, ui::AXNode* node) override;

  // |ui::AXPlatformNodeDelegate|
  gfx::NativeViewAccessible GetNativeViewAccessible() override;

  // |ui::AXPlatformNodeDelegate|
  gfx::NativeViewAccessible HitTestSync(
      int screen_physical_pixel_x,
      int screen_physical_pixel_y) const override;

  // |FlutterPlatformNodeDelegate|
  gfx::Rect GetBoundsRect(
      const ui::AXCoordinateSystem coordinate_system,
      const ui::AXClippingBehavior clipping_behavior,
      ui::AXOffscreenResult* offscreen_result) const override;

  // Dispatches a Windows accessibility event of the specified type, generated
  // by the accessibility node associated with this object. This is a
  // convenience wrapper around |NotifyWinEvent|.
  virtual void DispatchWinAccessibilityEvent(ax::mojom::Event event_type);

  // Sets the accessibility focus to the accessibility node associated with
  // this object.
  void SetFocus();

  // | AXPlatformNodeDelegate |
  gfx::AcceleratedWidget GetTargetForNativeAccessibilityEvent() override;

  // | FlutterPlatformNodeDelegate |
  ui::AXPlatformNode* GetPlatformNode() const override;

 private:
  ui::AXPlatformNode* ax_platform_node_;
  std::weak_ptr<AccessibilityBridge> bridge_;
  FlutterWindowsView* view_;

  FML_DISALLOW_COPY_AND_ASSIGN(FlutterPlatformNodeDelegateWindows);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_FLUTTER_PLATFORM_NODE_DELEGATE_WINDOWS_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMNODEDELEGATEMAC_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMNODEDELEGATEMAC_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/accessibility_bridge.h"
#include "flutter/shell/platform/common/flutter_platform_node_delegate.h"
#include "flutter/shell/platform/embedder/embedder.h"

namespace flutter {

//------------------------------------------------------------------------------
/// The macOS implementation of FlutterPlatformNodeDelegate. This class uses
/// AXPlatformNodeMac to manage the macOS-specific accessibility objects.
class FlutterPlatformNodeDelegateMac : public FlutterPlatformNodeDelegate {
 public:
  FlutterPlatformNodeDelegateMac(std::weak_ptr<AccessibilityBridge> bridge,
                                 __weak FlutterViewController* view_controller);
  virtual ~FlutterPlatformNodeDelegateMac();

  void Init(std::weak_ptr<OwnerBridge> bridge, ui::AXNode* node) override;

  void NodeDataChanged(const ui::AXNodeData& old_node_data,
                       const ui::AXNodeData& new_node_data) override;

  //---------------------------------------------------------------------------
  /// @brief      Gets the live region text of this node in UTF-8 format. This
  ///             is useful to determine the changes in between semantics
  ///             updates when generating accessibility events.
  std::string GetLiveRegionText() const;

  // |ui::AXPlatformNodeDelegate|
  gfx::NativeViewAccessible GetNativeViewAccessible() override;

  // |ui::AXPlatformNodeDelegate|
  gfx::NativeViewAccessible GetNSWindow() override;

  // |FlutterPlatformNodeDelegate|
  gfx::NativeViewAccessible GetParent() override;

  // |FlutterPlatformNodeDelegate|
  gfx::Rect GetBoundsRect(
      const ui::AXCoordinateSystem coordinate_system,
      const ui::AXClippingBehavior clipping_behavior,
      ui::AXOffscreenResult* offscreen_result) const override;

 private:
  ui::AXPlatformNode* ax_platform_node_;
  std::weak_ptr<AccessibilityBridge> bridge_;
  __weak FlutterViewController* view_controller_;

  gfx::RectF ConvertBoundsFromLocalToScreen(
      const gfx::RectF& local_bounds) const;
  gfx::RectF ConvertBoundsFromScreenToGlobal(
      const gfx::RectF& window_bounds) const;

  FML_DISALLOW_COPY_AND_ASSIGN(FlutterPlatformNodeDelegateMac);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMNODEDELEGATEMAC_H_

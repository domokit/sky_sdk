// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_ACCESSIBILITY_BRIDGE_WINDOWS_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_ACCESSIBILITY_BRIDGE_WINDOWS_H_

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/common/accessibility_bridge.h"
#include "flutter/third_party/accessibility/ax/platform/ax_fragment_root_delegate_win.h"

namespace flutter {

class FlutterWindowsEngine;
class FlutterWindowsView;
class FlutterPlatformNodeDelegateWindows;

// The Win32 implementation of AccessibilityBridge.
//
// This interacts with Windows accessibility APIs, which includes routing
// accessibility events fired from the framework to Windows, routing native
// Windows accessibility events to the framework, and creating Windows-specific
// FlutterPlatformNodeDelegate objects for each node in the semantics tree.
///
/// AccessibilityBridgeWindows must be created as a shared_ptr, since some
/// methods acquires its weak_ptr.
class AccessibilityBridgeWindows : public AccessibilityBridge,
                                   public ui::AXFragmentRootDelegateWin {
 public:
  AccessibilityBridgeWindows(FlutterWindowsView* view);
  virtual ~AccessibilityBridgeWindows() = default;

  // |AccessibilityBridge|
  void DispatchAccessibilityAction(AccessibilityNodeId target,
                                   FlutterSemanticsAction action,
                                   fml::MallocMapping data) override;

  // Dispatches a Windows accessibility event of the specified type, generated
  // by the accessibility node associated with the specified semantics node.
  //
  // This is a virtual method for the convenience of unit tests.
  virtual void DispatchWinAccessibilityEvent(
      std::shared_ptr<FlutterPlatformNodeDelegateWindows> node_delegate,
      ax::mojom::Event event_type);

  // Sets the accessibility focus to the accessibility node associated with the
  // specified semantics node.
  //
  // This is a virtual method for the convenience of unit tests.
  virtual void SetFocus(
      std::shared_ptr<FlutterPlatformNodeDelegateWindows> node_delegate);

  // |AXFragmentRootDelegateWin|
  gfx::NativeViewAccessible GetChildOfAXFragmentRoot() override;

  // |AXFragmentRootDelegateWin|
  gfx::NativeViewAccessible GetParentOfAXFragmentRoot() override;

  // |AXFragmentRootDelegateWin|
  bool IsAXFragmentRootAControlElement() override;

 protected:
  // |AccessibilityBridge|
  void OnAccessibilityEvent(
      ui::AXEventGenerator::TargetedEvent targeted_event) override;

  // |AccessibilityBridge|
  std::shared_ptr<FlutterPlatformNodeDelegate>
  CreateFlutterPlatformNodeDelegate() override;

  // Retrieve the focused node for accessibility events.
  virtual std::weak_ptr<FlutterPlatformNodeDelegate> GetFocusedNode();

 private:
  FlutterWindowsView* view_;

  FML_DISALLOW_COPY_AND_ASSIGN(AccessibilityBridgeWindows);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_ACCESSIBILITY_BRIDGE_WINDOWS_H_

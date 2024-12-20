// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_TEST_ACCESSIBILITY_BRIDGE_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_TEST_ACCESSIBILITY_BRIDGE_H_

#include "accessibility_bridge.h"

namespace flutter {

class TestAccessibilityBridge : public AccessibilityBridge {
 public:
  TestAccessibilityBridge() = default;

  void DispatchAccessibilityAction(AccessibilityNodeId target,
                                   FlutterSemanticsAction action,
                                   fml::MallocMapping data) override;

  std::vector<ui::AXEventGenerator::Event> accessibility_events;
  std::vector<FlutterSemanticsAction> performed_actions;

 protected:
  void OnAccessibilityEvent(
      ui::AXEventGenerator::TargetedEvent targeted_event) override;

  std::shared_ptr<FlutterPlatformNodeDelegate>
  CreateFlutterPlatformNodeDelegate() override;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_TEST_ACCESSIBILITY_BRIDGE_H_

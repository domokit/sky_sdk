// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_MAC_UNITTEST_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_MAC_UNITTEST_H_

#include "ax_platform_node_mac.h"
#include "ax_platform_node_unittest.h"

namespace ui {

// A test fixture that supports accessing the macOS-specific node
// implementations for the AXTree under test.
class AXPlatformNodeMacTest : public AXPlatformNodeTest {
 public:
  AXPlatformNodeMacTest();
  ~AXPlatformNodeMacTest() override;

  void SetUp() override;
  void TearDown() override;

 protected:
  AXPlatformNode* AXPlatformNodeFromNode(AXNode* node);
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_MAC_UNITTEST_H_

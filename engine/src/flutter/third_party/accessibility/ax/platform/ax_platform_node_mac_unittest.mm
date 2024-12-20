// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_platform_node_mac_unittest.h"

#include "ax_platform_node_mac.h"
#include "gtest/gtest.h"
#include "test_ax_node_wrapper.h"
#include "third_party/accessibility/ax/ax_node_data.h"

namespace ui {

AXPlatformNodeMacTest::AXPlatformNodeMacTest() = default;

AXPlatformNodeMacTest::~AXPlatformNodeMacTest() = default;

void AXPlatformNodeMacTest::SetUp() {}

void AXPlatformNodeMacTest::TearDown() {
  // Destroy the tree and make sure we're not leaking any objects.
  DestroyTree();
  TestAXNodeWrapper::SetGlobalIsWebContent(false);
  ASSERT_EQ(0U, AXPlatformNodeBase::GetInstanceCountForTesting());
}

AXPlatformNode* AXPlatformNodeMacTest::AXPlatformNodeFromNode(AXNode* node) {
  const TestAXNodeWrapper* wrapper = TestAXNodeWrapper::GetOrCreate(GetTree(), node);
  return wrapper ? wrapper->ax_platform_node() : nullptr;
}

// Verify that we can get an AXPlatformNodeMac and AXPlatformNodeCocoa from the tree.
TEST_F(AXPlatformNodeMacTest, CanGetCocoaPlatformNodeFromTree) {
  AXNodeData root;
  root.id = 1;
  root.relative_bounds.bounds = gfx::RectF(0, 0, 40, 40);

  Init(root);
  AXNode* root_node = GetRootAsAXNode();
  ASSERT_TRUE(root_node != nullptr);

  AXPlatformNode* platform_node = AXPlatformNodeFromNode(root_node);
  ASSERT_TRUE(platform_node != nullptr);

  AXPlatformNodeCocoa* native_root = platform_node->GetNativeViewAccessible();
  EXPECT_TRUE(native_root != nullptr);
}

// Test that [AXPlatformNodeCocoa accessbilityRangeForPosition:] doesn't crash.
// https://github.com/flutter/flutter/issues/102416
TEST_F(AXPlatformNodeMacTest, AccessibilityRangeForPositionDoesntCrash) {
  AXNodeData root;
  root.id = 1;
  root.relative_bounds.bounds = gfx::RectF(0, 0, 40, 40);

  Init(root);
  AXNode* root_node = GetRootAsAXNode();
  ASSERT_TRUE(root_node != nullptr);

  AXPlatformNode* platform_node = AXPlatformNodeFromNode(root_node);
  ASSERT_TRUE(platform_node != nullptr);

  NSPoint point = NSMakePoint(0, 0);
  AXPlatformNodeCocoa* native_root = platform_node->GetNativeViewAccessible();
  ASSERT_TRUE(native_root != nullptr);

  [native_root accessibilityRangeForPosition:(NSPoint)point];
}

}  // namespace ui

// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_TEST_AX_NODE_HELPER_H_
#define UI_ACCESSIBILITY_TEST_AX_NODE_HELPER_H_

#include "ax_clipping_behavior.h"
#include "ax_coordinate_system.h"
#include "ax_node.h"
#include "ax_offscreen_result.h"
#include "ax_tree.h"

namespace ui {

// For testing, a TestAXNodeHelper wraps an AXNode. This is a simple
// version of TestAXNodeWrapper.
class TestAXNodeHelper {
 public:
  // Create TestAXNodeHelper instances on-demand from an AXTree and AXNode.
  static TestAXNodeHelper* GetOrCreate(AXTree* tree, AXNode* node);
  ~TestAXNodeHelper();

  gfx::Rect GetBoundsRect(const AXCoordinateSystem coordinate_system,
                          const AXClippingBehavior clipping_behavior,
                          AXOffscreenResult* offscreen_result) const;
  gfx::Rect GetInnerTextRangeBoundsRect(
      const int start_offset,
      const int end_offset,
      const AXCoordinateSystem coordinate_system,
      const AXClippingBehavior clipping_behavior,
      AXOffscreenResult* offscreen_result) const;

 private:
  TestAXNodeHelper(AXTree* tree, AXNode* node);
  int InternalChildCount() const;
  TestAXNodeHelper* InternalGetChild(int index) const;
  const AXNodeData& GetData() const;
  gfx::RectF GetLocation() const;
  gfx::RectF GetInlineTextRect(const int start_offset,
                               const int end_offset) const;
  AXOffscreenResult DetermineOffscreenResult(gfx::RectF bounds) const;

  AXTree* tree_;
  AXNode* node_;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_TEST_AX_NODE_HELPER_H_

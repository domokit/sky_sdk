// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_UNITTEST_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_UNITTEST_H_

#include "gtest/gtest.h"

#include "ax/ax_enums.h"
#include "ax/ax_node.h"
#include "ax/ax_node_data.h"
#include "ax/ax_tree_id.h"
#include "ax/ax_tree_update.h"
#include "ax/test_ax_tree_manager.h"

namespace ui {

class AXPlatformNodeTest : public testing::Test, public TestAXTreeManager {
 public:
  AXPlatformNodeTest();
  ~AXPlatformNodeTest() override;
  AXPlatformNodeTest(const AXPlatformNodeTest&) = delete;
  AXPlatformNodeTest& operator=(const AXPlatformNodeTest&) = delete;

 protected:
  // Initialize given an AXTreeUpdate.
  void Init(const AXTreeUpdate& initial_state);

  // Convenience functions to initialize directly from a few AXNodeData objects.
  void Init(const ui::AXNodeData& node1,
            const ui::AXNodeData& node2 = ui::AXNodeData(),
            const ui::AXNodeData& node3 = ui::AXNodeData(),
            const ui::AXNodeData& node4 = ui::AXNodeData(),
            const ui::AXNodeData& node5 = ui::AXNodeData(),
            const ui::AXNodeData& node6 = ui::AXNodeData(),
            const ui::AXNodeData& node7 = ui::AXNodeData(),
            const ui::AXNodeData& node8 = ui::AXNodeData(),
            const ui::AXNodeData& node9 = ui::AXNodeData(),
            const ui::AXNodeData& node10 = ui::AXNodeData(),
            const ui::AXNodeData& node11 = ui::AXNodeData(),
            const ui::AXNodeData& node12 = ui::AXNodeData());

  AXTreeUpdate BuildTextField();
  AXTreeUpdate BuildTextFieldWithSelectionRange(int32_t start, int32_t stop);
  AXTreeUpdate BuildContentEditable();
  AXTreeUpdate BuildContentEditableWithSelectionRange(int32_t start,
                                                      int32_t end);
  AXTreeUpdate Build3X3Table();
  AXTreeUpdate BuildAriaColumnAndRowCountGrids();

  AXTreeUpdate BuildListBox(
      bool option_1_is_selected,
      bool option_2_is_selected,
      bool option_3_is_selected,
      const std::vector<ax::mojom::State>& additional_state);
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_UNITTEST_H_

// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <unordered_set>

#include "gtest/gtest.h"

#include "ax_enum_util.h"
#include "ax_enums.h"
#include "ax_role_properties.h"

namespace ui {

TEST(AXRolePropertiesTest, TestSupportsHierarchicalLevel) {
  // Test for iterating through all roles and validate if a role supports
  // hierarchical level.
  std::unordered_set<ax::mojom::Role>
      roles_expected_supports_hierarchical_level = {
          ax::mojom::Role::kComment, ax::mojom::Role::kListItem,
          ax::mojom::Role::kRow, ax::mojom::Role::kTabList,
          ax::mojom::Role::kTreeItem};

  for (int role_idx = static_cast<int>(ax::mojom::Role::kMinValue);
       role_idx <= static_cast<int>(ax::mojom::Role::kMaxValue); role_idx++) {
    ax::mojom::Role role = static_cast<ax::mojom::Role>(role_idx);
    bool supports_hierarchical_level = SupportsHierarchicalLevel(role);

    SCOPED_TRACE(testing::Message() << "ax::mojom::Role=" << ToString(role)
                                    << ", Actual: supportsHierarchicalLevel="
                                    << supports_hierarchical_level
                                    << ", Expected: supportsHierarchicalLevel="
                                    << !supports_hierarchical_level);

    if (roles_expected_supports_hierarchical_level.find(role) !=
        roles_expected_supports_hierarchical_level.end())
      EXPECT_TRUE(supports_hierarchical_level);
    else
      EXPECT_FALSE(supports_hierarchical_level);
  }
}

TEST(AXRolePropertiesTest, TestSupportsToggle) {
  // Test for iterating through all roles and validate if a role supports
  // toggle.
  std::unordered_set<ax::mojom::Role> roles_expected_supports_toggle = {
      ax::mojom::Role::kCheckBox, ax::mojom::Role::kMenuItemCheckBox,
      ax::mojom::Role::kSwitch, ax::mojom::Role::kToggleButton};

  for (int role_idx = static_cast<int>(ax::mojom::Role::kMinValue);
       role_idx <= static_cast<int>(ax::mojom::Role::kMaxValue); role_idx++) {
    ax::mojom::Role role = static_cast<ax::mojom::Role>(role_idx);
    bool supports_toggle = SupportsToggle(role);

    SCOPED_TRACE(testing::Message()
                 << "ax::mojom::Role=" << ToString(role)
                 << ", Actual: supportsToggle=" << supports_toggle
                 << ", Expected: supportsToggle=" << !supports_toggle);

    if (roles_expected_supports_toggle.find(role) !=
        roles_expected_supports_toggle.end())
      EXPECT_TRUE(supports_toggle);
    else
      EXPECT_FALSE(supports_toggle);
  }
}
}  // namespace ui

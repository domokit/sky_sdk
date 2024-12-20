// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Included first as it collides with the X11 headers.
#include "gtest/gtest.h"

#include "flutter/shell/platform/linux/fl_accessible_node.h"

// Checks can build a tree of nodes.
TEST(FlAccessibleNodeTest, BuildTree) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(FlAccessibleNode) root = fl_accessible_node_new(engine, 0);
  g_autoptr(FlAccessibleNode) child1 = fl_accessible_node_new(engine, 1);
  fl_accessible_node_set_parent(child1, ATK_OBJECT(root), 0);
  g_autoptr(FlAccessibleNode) child2 = fl_accessible_node_new(engine, 1);
  fl_accessible_node_set_parent(child2, ATK_OBJECT(root), 1);
  g_autoptr(GPtrArray) children =
      g_ptr_array_new_with_free_func(g_object_unref);
  g_ptr_array_add(children, g_object_ref(child1));
  g_ptr_array_add(children, g_object_ref(child2));
  fl_accessible_node_set_children(root, children);

  EXPECT_EQ(atk_object_get_n_accessible_children(ATK_OBJECT(root)), 2);
  EXPECT_EQ(atk_object_get_index_in_parent(ATK_OBJECT(root)), 0);
  g_autoptr(AtkObject) c1 =
      atk_object_ref_accessible_child(ATK_OBJECT(root), 0);
  EXPECT_EQ(ATK_OBJECT(child1), c1);
  g_autoptr(AtkObject) c2 =
      atk_object_ref_accessible_child(ATK_OBJECT(root), 1);
  EXPECT_EQ(ATK_OBJECT(child2), c2);
  EXPECT_EQ(atk_object_get_parent(ATK_OBJECT(root)), nullptr);

  EXPECT_EQ(atk_object_get_parent(ATK_OBJECT(child1)), ATK_OBJECT(root));
  EXPECT_EQ(atk_object_get_index_in_parent(ATK_OBJECT(child1)), 0);
  EXPECT_EQ(atk_object_get_n_accessible_children(ATK_OBJECT(child1)), 0);

  EXPECT_EQ(atk_object_get_parent(ATK_OBJECT(child2)), ATK_OBJECT(root));
  EXPECT_EQ(atk_object_get_index_in_parent(ATK_OBJECT(child2)), 1);
  EXPECT_EQ(atk_object_get_n_accessible_children(ATK_OBJECT(child2)), 0);
}

// Checks node name is exposed to ATK.
TEST(FlAccessibleNodeTest, SetName) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(FlAccessibleNode) node = fl_accessible_node_new(engine, 0);
  fl_accessible_node_set_name(node, "test");
  EXPECT_STREQ(atk_object_get_name(ATK_OBJECT(node)), "test");
}

// Checks node extents are exposed to ATK.
TEST(FlAccessibleNodeTest, SetExtents) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(FlAccessibleNode) node = fl_accessible_node_new(engine, 0);
  fl_accessible_node_set_extents(node, 1, 2, 3, 4);
  gint x, y, width, height;
  atk_component_get_extents(ATK_COMPONENT(node), &x, &y, &width, &height,
                            ATK_XY_PARENT);
  EXPECT_EQ(x, 1);
  EXPECT_EQ(y, 2);
  EXPECT_EQ(width, 3);
  EXPECT_EQ(height, 4);
}

// Checks Flutter flags are mapped to appropriate ATK state.
TEST(FlAccessibleNodeTest, SetFlags) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(FlAccessibleNode) node = fl_accessible_node_new(engine, 0);
  fl_accessible_node_set_flags(
      node, static_cast<FlutterSemanticsFlag>(kFlutterSemanticsFlagIsEnabled |
                                              kFlutterSemanticsFlagIsFocusable |
                                              kFlutterSemanticsFlagIsFocused));

  AtkStateSet* state = atk_object_ref_state_set(ATK_OBJECT(node));
  EXPECT_TRUE(atk_state_set_contains_state(state, ATK_STATE_ENABLED));
  EXPECT_TRUE(atk_state_set_contains_state(state, ATK_STATE_SENSITIVE));
  EXPECT_TRUE(atk_state_set_contains_state(state, ATK_STATE_FOCUSABLE));
  EXPECT_TRUE(atk_state_set_contains_state(state, ATK_STATE_FOCUSED));
  EXPECT_TRUE(!atk_state_set_contains_state(state, ATK_STATE_CHECKED));
  g_object_unref(state);
}

// Checks Flutter flags are mapped to appropriate ATK roles.
TEST(FlAccessibleNodeTest, GetRole) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(FlAccessibleNode) node = fl_accessible_node_new(engine, 0);

  fl_accessible_node_set_flags(
      node, static_cast<FlutterSemanticsFlag>(kFlutterSemanticsFlagIsButton));
  EXPECT_EQ(atk_object_get_role(ATK_OBJECT(node)), ATK_ROLE_PUSH_BUTTON);

  fl_accessible_node_set_flags(node, static_cast<FlutterSemanticsFlag>(
                                         kFlutterSemanticsFlagHasCheckedState));
  EXPECT_EQ(atk_object_get_role(ATK_OBJECT(node)), ATK_ROLE_CHECK_BOX);

  fl_accessible_node_set_flags(
      node, static_cast<FlutterSemanticsFlag>(
                kFlutterSemanticsFlagHasCheckedState |
                kFlutterSemanticsFlagIsInMutuallyExclusiveGroup));
  EXPECT_EQ(atk_object_get_role(ATK_OBJECT(node)), ATK_ROLE_RADIO_BUTTON);

  fl_accessible_node_set_flags(node, static_cast<FlutterSemanticsFlag>(
                                         kFlutterSemanticsFlagHasToggledState));
  EXPECT_EQ(atk_object_get_role(ATK_OBJECT(node)), ATK_ROLE_TOGGLE_BUTTON);

  fl_accessible_node_set_flags(node, kFlutterSemanticsFlagIsTextField);
  EXPECT_EQ(atk_object_get_role(ATK_OBJECT(node)), ATK_ROLE_TEXT);

  fl_accessible_node_set_flags(
      node, static_cast<FlutterSemanticsFlag>(kFlutterSemanticsFlagIsTextField |
                                              kFlutterSemanticsFlagIsObscured));
  EXPECT_EQ(atk_object_get_role(ATK_OBJECT(node)), ATK_ROLE_PASSWORD_TEXT);
}

// Checks Flutter actions are mapped to the appropriate ATK actions.
TEST(FlAccessibleNodeTest, SetActions) {
  g_autoptr(FlDartProject) project = fl_dart_project_new();
  g_autoptr(FlEngine) engine = fl_engine_new(project);

  g_autoptr(FlAccessibleNode) node = fl_accessible_node_new(engine, 0);
  fl_accessible_node_set_actions(
      node, static_cast<FlutterSemanticsAction>(
                kFlutterSemanticsActionTap | kFlutterSemanticsActionLongPress));

  EXPECT_EQ(atk_action_get_n_actions(ATK_ACTION(node)), 2);
  EXPECT_STREQ(atk_action_get_name(ATK_ACTION(node), 0), "Tap");
  EXPECT_STREQ(atk_action_get_name(ATK_ACTION(node), 1), "LongPress");
}

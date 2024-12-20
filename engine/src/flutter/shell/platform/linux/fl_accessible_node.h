// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_ACCESSIBLE_NODE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_ACCESSIBLE_NODE_H_

#include <atk/atk.h>
#include <gio/gio.h>

#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_engine.h"

G_BEGIN_DECLS

// ATK g_autoptr macros weren't added until 2.37. Add them manually.
// https://gitlab.gnome.org/GNOME/atk/-/issues/10
#if !ATK_CHECK_VERSION(2, 37, 0)
G_DEFINE_AUTOPTR_CLEANUP_FUNC(AtkObject, g_object_unref)
#endif

G_DECLARE_DERIVABLE_TYPE(FlAccessibleNode,
                         fl_accessible_node,
                         FL,
                         ACCESSIBLE_NODE,
                         AtkObject);

/**
 * FlAccessibleNode:
 *
 * #FlAccessibleNode is an object that exposes a Flutter accessibility node to
 * ATK.
 */
struct _FlAccessibleNodeClass {
  AtkObjectClass parent_class;

  void (*set_name)(FlAccessibleNode* node, const gchar* name);
  void (*set_extents)(FlAccessibleNode* node,
                      gint x,
                      gint y,
                      gint width,
                      gint height);
  void (*set_flags)(FlAccessibleNode* node, FlutterSemanticsFlag flags);
  void (*set_actions)(FlAccessibleNode* node, FlutterSemanticsAction actions);
  void (*set_value)(FlAccessibleNode* node, const gchar* value);
  void (*set_text_selection)(FlAccessibleNode* node, gint base, gint extent);
  void (*set_text_direction)(FlAccessibleNode* node,
                             FlutterTextDirection direction);

  void (*perform_action)(FlAccessibleNode* node,
                         FlutterSemanticsAction action,
                         GBytes* data);
};

/**
 * fl_accessible_node_new:
 * @engine: the #FlEngine this node came from.
 * @id: the semantics node ID this object represents.
 *
 * Creates a new accessibility object that exposes Flutter accessibility
 * information to ATK.
 *
 * Returns: a new #FlAccessibleNode.
 */
FlAccessibleNode* fl_accessible_node_new(FlEngine* engine, int32_t id);

/**
 * fl_accessible_node_set_parent:
 * @node: an #FlAccessibleNode.
 * @parent: an #AtkObject.
 * @index: the index of this node in the parent.
 *
 * Sets the parent of this node. The parent can be changed at any time.
 */
void fl_accessible_node_set_parent(FlAccessibleNode* node,
                                   AtkObject* parent,
                                   gint index);

/**
 * fl_accessible_node_set_children:
 * @node: an #FlAccessibleNode.
 * @children: (transfer none) (element-type AtkObject): a list of #AtkObject.
 *
 * Sets the children of this node. The children can be changed at any time.
 */
void fl_accessible_node_set_children(FlAccessibleNode* node,
                                     GPtrArray* children);

/**
 * fl_accessible_node_set_name:
 * @node: an #FlAccessibleNode.
 * @name: a node name.
 *
 * Sets the name of this node as reported to the a11y consumer.
 */
void fl_accessible_node_set_name(FlAccessibleNode* node, const gchar* name);

/**
 * fl_accessible_node_set_extents:
 * @node: an #FlAccessibleNode.
 * @x: x co-ordinate of this node relative to its parent.
 * @y: y co-ordinate of this node relative to its parent.
 * @width: width of this node in pixels.
 * @height: height of this node in pixels.
 *
 * Sets the position and size of this node.
 */
void fl_accessible_node_set_extents(FlAccessibleNode* node,
                                    gint x,
                                    gint y,
                                    gint width,
                                    gint height);

/**
 * fl_accessible_node_set_flags:
 * @node: an #FlAccessibleNode.
 * @flags: the flags for this node.
 *
 * Sets the flags for this node.
 */
void fl_accessible_node_set_flags(FlAccessibleNode* node,
                                  FlutterSemanticsFlag flags);

/**
 * fl_accessible_node_set_actions:
 * @node: an #FlAccessibleNode.
 * @actions: the actions this node can perform.
 *
 * Sets the actions that this node can perform.
 */
void fl_accessible_node_set_actions(FlAccessibleNode* node,
                                    FlutterSemanticsAction actions);

/**
 * fl_accessible_node_set_value:
 * @node: an #FlAccessibleNode.
 * @value: a node value.
 *
 * Sets the value of this node.
 */
void fl_accessible_node_set_value(FlAccessibleNode* node, const gchar* value);

/**
 * fl_accessible_node_set_text_selection:
 * @node: an #FlAccessibleNode.
 * @base: the position at which the text selection originates.
 * @extent: the position at which the text selection terminates.
 *
 * Sets the text selection of this node.
 */
void fl_accessible_node_set_text_selection(FlAccessibleNode* node,
                                           gint base,
                                           gint extent);

/**
 * fl_accessible_node_set_text_direction:
 * @node: an #FlAccessibleNode.
 * @direction: the direction of the text.
 *
 * Sets the text direction of this node.
 */
void fl_accessible_node_set_text_direction(FlAccessibleNode* node,
                                           FlutterTextDirection direction);

/**
 * fl_accessible_node_dispatch_action:
 * @node: an #FlAccessibleNode.
 * @action: the action being dispatched.
 * @data: (allow-none): data associated with the action.
 *
 * Performs a semantic action for this node.
 */
void fl_accessible_node_perform_action(FlAccessibleNode* node,
                                       FlutterSemanticsAction action,
                                       GBytes* data);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_ACCESSIBLE_NODE_H_

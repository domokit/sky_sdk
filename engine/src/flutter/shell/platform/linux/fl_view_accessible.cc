// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_view_accessible.h"
#include "flutter/shell/platform/linux/fl_accessible_node.h"
#include "flutter/shell/platform/linux/fl_accessible_text_field.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_value.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_view.h"

static constexpr int32_t kRootSemanticsNodeId = 0;

struct _FlViewAccessible {
  AtkPlug parent_instance;

  GWeakRef engine;

  // Semantics nodes keyed by ID
  GHashTable* semantics_nodes_by_id;

  // Flag to track when root node is created.
  gboolean root_node_created;
};

G_DEFINE_TYPE(FlViewAccessible, fl_view_accessible, ATK_TYPE_PLUG)

static FlAccessibleNode* create_node(FlViewAccessible* self,
                                     FlutterSemanticsNode2* semantics) {
  g_autoptr(FlEngine) engine = FL_ENGINE(g_weak_ref_get(&self->engine));
  if (engine == nullptr) {
    return nullptr;
  }

  if (semantics->flags & kFlutterSemanticsFlagIsTextField) {
    return fl_accessible_text_field_new(engine, semantics->id);
  }

  return fl_accessible_node_new(engine, semantics->id);
}

static FlAccessibleNode* lookup_node(FlViewAccessible* self, int32_t id) {
  return FL_ACCESSIBLE_NODE(
      g_hash_table_lookup(self->semantics_nodes_by_id, GINT_TO_POINTER(id)));
}

// Gets the ATK node for the given id.
// If the node doesn't exist it will be created.
static FlAccessibleNode* get_node(FlViewAccessible* self,
                                  FlutterSemanticsNode2* semantics) {
  FlAccessibleNode* node = lookup_node(self, semantics->id);
  if (node != nullptr) {
    return node;
  }

  node = create_node(self, semantics);
  if (semantics->id == kRootSemanticsNodeId) {
    fl_accessible_node_set_parent(node, ATK_OBJECT(self), 0);
  }
  g_hash_table_insert(self->semantics_nodes_by_id,
                      GINT_TO_POINTER(semantics->id),
                      reinterpret_cast<gpointer>(node));

  // Update when root node is created.
  if (!self->root_node_created && semantics->id == kRootSemanticsNodeId) {
    g_signal_emit_by_name(self, "children-changed::add", 0, node, nullptr);
    self->root_node_created = true;
  }

  return node;
}

// Implements AtkObject::get_n_children
static gint fl_view_accessible_get_n_children(AtkObject* accessible) {
  FlViewAccessible* self = FL_VIEW_ACCESSIBLE(accessible);
  FlAccessibleNode* node = lookup_node(self, 0);

  if (node == nullptr) {
    return 0;
  }

  return 1;
}

// Implements AtkObject::ref_child
static AtkObject* fl_view_accessible_ref_child(AtkObject* accessible, gint i) {
  FlViewAccessible* self = FL_VIEW_ACCESSIBLE(accessible);
  FlAccessibleNode* node = lookup_node(self, 0);

  if (i != 0 || node == nullptr) {
    return nullptr;
  }

  return ATK_OBJECT(g_object_ref(node));
}

// Implements AtkObject::get_role
static AtkRole fl_view_accessible_get_role(AtkObject* accessible) {
  return ATK_ROLE_PANEL;
}

// Implements AtkObject::ref_state_set
static AtkStateSet* fl_view_accessible_ref_state_set(AtkObject* accessible) {
  FlViewAccessible* self = FL_VIEW_ACCESSIBLE(accessible);
  FlAccessibleNode* node = lookup_node(self, 0);
  return node != nullptr ? atk_object_ref_state_set(ATK_OBJECT(node)) : nullptr;
}

static void fl_view_accessible_dispose(GObject* object) {
  FlViewAccessible* self = FL_VIEW_ACCESSIBLE(object);

  g_clear_pointer(&self->semantics_nodes_by_id, g_hash_table_unref);

  g_weak_ref_clear(&self->engine);

  G_OBJECT_CLASS(fl_view_accessible_parent_class)->dispose(object);
}

static void fl_view_accessible_class_init(FlViewAccessibleClass* klass) {
  ATK_OBJECT_CLASS(klass)->get_n_children = fl_view_accessible_get_n_children;
  ATK_OBJECT_CLASS(klass)->ref_child = fl_view_accessible_ref_child;
  ATK_OBJECT_CLASS(klass)->get_role = fl_view_accessible_get_role;
  ATK_OBJECT_CLASS(klass)->ref_state_set = fl_view_accessible_ref_state_set;

  G_OBJECT_CLASS(klass)->dispose = fl_view_accessible_dispose;
}

static void fl_view_accessible_init(FlViewAccessible* self) {
  self->semantics_nodes_by_id = g_hash_table_new_full(
      g_direct_hash, g_direct_equal, nullptr, g_object_unref);
}

FlViewAccessible* fl_view_accessible_new(FlEngine* engine) {
  FlViewAccessible* self =
      FL_VIEW_ACCESSIBLE(g_object_new(fl_view_accessible_get_type(), nullptr));
  g_weak_ref_init(&self->engine, engine);
  return self;
}

void fl_view_accessible_handle_update_semantics(
    FlViewAccessible* self,
    const FlutterSemanticsUpdate2* update) {
  g_autoptr(GHashTable) pending_children =
      g_hash_table_new_full(g_direct_hash, g_direct_equal, nullptr,
                            reinterpret_cast<GDestroyNotify>(fl_value_unref));
  for (size_t i = 0; i < update->node_count; i++) {
    FlutterSemanticsNode2* node = update->nodes[i];
    FlAccessibleNode* atk_node = get_node(self, node);

    fl_accessible_node_set_flags(atk_node, node->flags);
    fl_accessible_node_set_actions(atk_node, node->actions);
    fl_accessible_node_set_name(atk_node, node->label);
    fl_accessible_node_set_extents(
        atk_node, node->rect.left + node->transform.transX,
        node->rect.top + node->transform.transY,
        node->rect.right - node->rect.left, node->rect.bottom - node->rect.top);
    fl_accessible_node_set_value(atk_node, node->value);
    fl_accessible_node_set_text_selection(atk_node, node->text_selection_base,
                                          node->text_selection_extent);
    fl_accessible_node_set_text_direction(atk_node, node->text_direction);

    FlValue* children = fl_value_new_int32_list(
        node->children_in_traversal_order, node->child_count);
    g_hash_table_insert(pending_children, atk_node, children);
  }

  g_hash_table_foreach_remove(
      pending_children,
      [](gpointer key, gpointer value, gpointer user_data) -> gboolean {
        FlViewAccessible* self = FL_VIEW_ACCESSIBLE(user_data);

        FlAccessibleNode* parent = FL_ACCESSIBLE_NODE(key);

        size_t child_count = fl_value_get_length(static_cast<FlValue*>(value));
        const int32_t* children_in_traversal_order =
            fl_value_get_int32_list(static_cast<FlValue*>(value));

        g_autoptr(GPtrArray) children = g_ptr_array_new();
        for (size_t i = 0; i < child_count; i++) {
          FlAccessibleNode* child =
              lookup_node(self, children_in_traversal_order[i]);
          g_assert(child != nullptr);
          fl_accessible_node_set_parent(child, ATK_OBJECT(parent), i);
          g_ptr_array_add(children, child);
        }
        fl_accessible_node_set_children(parent, children);

        return TRUE;
      },
      self);
}

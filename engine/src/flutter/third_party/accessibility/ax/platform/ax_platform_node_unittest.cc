// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_platform_node_unittest.h"

#include "ax/ax_constants.h"
#include "test_ax_node_wrapper.h"

namespace ui {

AXPlatformNodeTest::AXPlatformNodeTest() = default;

AXPlatformNodeTest::~AXPlatformNodeTest() = default;

void AXPlatformNodeTest::Init(const AXTreeUpdate& initial_state) {
  SetTree(std::make_unique<AXTree>(initial_state));
}

void AXPlatformNodeTest::Init(
    const ui::AXNodeData& node1,
    const ui::AXNodeData& node2 /* = ui::AXNodeData() */,
    const ui::AXNodeData& node3 /* = ui::AXNodeData() */,
    const ui::AXNodeData& node4 /* = ui::AXNodeData() */,
    const ui::AXNodeData& node5 /* = ui::AXNodeData() */,
    const ui::AXNodeData& node6 /* = ui::AXNodeData() */,
    const ui::AXNodeData& node7 /* = ui::AXNodeData() */,
    const ui::AXNodeData& node8 /* = ui::AXNodeData() */,
    const ui::AXNodeData& node9 /* = ui::AXNodeData() */,
    const ui::AXNodeData& node10 /* = ui::AXNodeData() */,
    const ui::AXNodeData& node11 /* = ui::AXNodeData() */,
    const ui::AXNodeData& node12 /* = ui::AXNodeData() */) {
  static ui::AXNodeData empty_data;
  int32_t no_id = empty_data.id;
  AXTreeUpdate update;
  update.root_id = node1.id;
  update.nodes.push_back(node1);
  if (node2.id != no_id)
    update.nodes.push_back(node2);
  if (node3.id != no_id)
    update.nodes.push_back(node3);
  if (node4.id != no_id)
    update.nodes.push_back(node4);
  if (node5.id != no_id)
    update.nodes.push_back(node5);
  if (node6.id != no_id)
    update.nodes.push_back(node6);
  if (node7.id != no_id)
    update.nodes.push_back(node7);
  if (node8.id != no_id)
    update.nodes.push_back(node8);
  if (node9.id != no_id)
    update.nodes.push_back(node9);
  if (node10.id != no_id)
    update.nodes.push_back(node10);
  if (node11.id != no_id)
    update.nodes.push_back(node11);
  if (node12.id != no_id)
    update.nodes.push_back(node12);
  Init(update);
}

AXTreeUpdate AXPlatformNodeTest::BuildTextField() {
  AXNodeData text_field_node;
  text_field_node.id = 1;
  text_field_node.role = ax::mojom::Role::kTextField;
  text_field_node.AddState(ax::mojom::State::kEditable);
  text_field_node.SetValue("How now brown cow.");

  AXTreeUpdate update;
  update.root_id = text_field_node.id;
  update.nodes.push_back(text_field_node);
  return update;
}

AXTreeUpdate AXPlatformNodeTest::BuildTextFieldWithSelectionRange(
    int32_t start,
    int32_t stop) {
  AXNodeData text_field_node;
  text_field_node.id = 1;
  text_field_node.role = ax::mojom::Role::kTextField;
  text_field_node.AddState(ax::mojom::State::kEditable);
  text_field_node.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
  text_field_node.AddIntAttribute(ax::mojom::IntAttribute::kTextSelStart,
                                  start);
  text_field_node.AddIntAttribute(ax::mojom::IntAttribute::kTextSelEnd, stop);
  text_field_node.SetValue("How now brown cow.");

  AXTreeUpdate update;
  update.root_id = text_field_node.id;
  update.nodes.push_back(text_field_node);
  return update;
}

AXTreeUpdate AXPlatformNodeTest::BuildContentEditable() {
  AXNodeData content_editable_node;
  content_editable_node.id = 1;
  content_editable_node.role = ax::mojom::Role::kGroup;
  content_editable_node.AddState(ax::mojom::State::kRichlyEditable);
  content_editable_node.AddBoolAttribute(
      ax::mojom::BoolAttribute::kEditableRoot, true);
  content_editable_node.SetValue("How now brown cow.");

  AXTreeUpdate update;
  update.root_id = content_editable_node.id;
  update.nodes.push_back(content_editable_node);
  return update;
}

AXTreeUpdate AXPlatformNodeTest::BuildContentEditableWithSelectionRange(
    int32_t start,
    int32_t end) {
  AXNodeData content_editable_node;
  content_editable_node.id = 1;
  content_editable_node.role = ax::mojom::Role::kGroup;
  content_editable_node.AddState(ax::mojom::State::kRichlyEditable);
  content_editable_node.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected,
                                         true);
  content_editable_node.AddBoolAttribute(
      ax::mojom::BoolAttribute::kEditableRoot, true);
  content_editable_node.SetValue("How now brown cow.");

  AXTreeUpdate update;
  update.root_id = content_editable_node.id;
  update.nodes.push_back(content_editable_node);

  update.has_tree_data = true;
  update.tree_data.sel_anchor_object_id = content_editable_node.id;
  update.tree_data.sel_focus_object_id = content_editable_node.id;
  update.tree_data.sel_anchor_offset = start;
  update.tree_data.sel_focus_offset = end;

  return update;
}

AXTreeUpdate AXPlatformNodeTest::AXPlatformNodeTest::Build3X3Table() {
  /*
    Build a table that looks like:

    ----------------------        (A) Column Header
    |        | (A) | (B) |        (B) Column Header
    ----------------------        (C) Row Header
    |  (C)  |  1  |  2   |        (D) Row Header
    ----------------------
    |  (D)  |  3  |  4   |
    ----------------------
  */

  AXNodeData table;
  table.id = 1;
  table.role = ax::mojom::Role::kTable;

  table.AddIntAttribute(ax::mojom::IntAttribute::kTableRowCount, 3);
  table.AddIntAttribute(ax::mojom::IntAttribute::kTableColumnCount, 3);

  table.child_ids.push_back(50);  // Header
  table.child_ids.push_back(2);   // Row 1
  table.child_ids.push_back(10);  // Row 2

  // Table column header
  AXNodeData table_row_header;
  table_row_header.id = 50;
  table_row_header.role = ax::mojom::Role::kRow;
  table_row_header.child_ids.push_back(51);
  table_row_header.child_ids.push_back(52);
  table_row_header.child_ids.push_back(53);

  AXNodeData table_column_header_1;
  table_column_header_1.id = 51;
  table_column_header_1.role = ax::mojom::Role::kColumnHeader;
  table_column_header_1.AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellRowIndex, 0);
  table_column_header_1.AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellColumnIndex, 0);

  AXNodeData table_column_header_2;
  table_column_header_2.id = 52;
  table_column_header_2.role = ax::mojom::Role::kColumnHeader;
  table_column_header_2.SetName("column header 1");
  table_column_header_2.AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellRowIndex, 0);
  table_column_header_2.AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellColumnIndex, 1);

  AXNodeData table_column_header_3;
  table_column_header_3.id = 53;
  table_column_header_3.role = ax::mojom::Role::kColumnHeader;
  // Either ax::mojom::StringAttribute::kName -or-
  // ax::mojom::StringAttribute::kDescription is acceptable for a description
  table_column_header_3.AddStringAttribute(
      ax::mojom::StringAttribute::kDescription, "column header 2");
  table_column_header_3.AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellRowIndex, 0);
  table_column_header_3.AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellColumnIndex, 2);

  // Row 1
  AXNodeData table_row_1;
  table_row_1.id = 2;
  table_row_1.role = ax::mojom::Role::kRow;

  AXNodeData table_row_header_1;
  table_row_header_1.id = 3;
  table_row_header_1.role = ax::mojom::Role::kRowHeader;
  table_row_header_1.SetName("row header 1");
  table_row_header_1.AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellRowIndex, 1);
  table_row_header_1.AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellColumnIndex, 0);
  table_row_1.child_ids.push_back(table_row_header_1.id);

  AXNodeData table_cell_1;
  table_cell_1.id = 4;
  table_cell_1.role = ax::mojom::Role::kCell;
  table_cell_1.SetName("1");
  table_cell_1.AddIntAttribute(ax::mojom::IntAttribute::kTableCellRowIndex, 1);
  table_cell_1.AddIntAttribute(ax::mojom::IntAttribute::kTableCellColumnIndex,
                               1);
  table_row_1.child_ids.push_back(table_cell_1.id);

  AXNodeData table_cell_2;
  table_cell_2.id = 5;
  table_cell_2.role = ax::mojom::Role::kCell;
  table_cell_2.SetName("2");
  table_cell_2.AddIntAttribute(ax::mojom::IntAttribute::kTableCellRowIndex, 1);
  table_cell_2.AddIntAttribute(ax::mojom::IntAttribute::kTableCellColumnIndex,
                               2);
  table_row_1.child_ids.push_back(table_cell_2.id);

  // Row 2
  AXNodeData table_row_2;
  table_row_2.id = 10;
  table_row_2.role = ax::mojom::Role::kRow;

  AXNodeData table_row_header_2;
  table_row_header_2.id = 11;
  table_row_header_2.role = ax::mojom::Role::kRowHeader;
  // Either ax::mojom::StringAttribute::kName -or-
  // ax::mojom::StringAttribute::kDescription is acceptable for a description
  table_row_header_2.AddStringAttribute(
      ax::mojom::StringAttribute::kDescription, "row header 2");
  table_row_header_2.AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellRowIndex, 2);
  table_row_header_2.AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellColumnIndex, 0);
  table_row_2.child_ids.push_back(table_row_header_2.id);

  AXNodeData table_cell_3;
  table_cell_3.id = 12;
  table_cell_3.role = ax::mojom::Role::kCell;
  table_cell_3.SetName("3");
  table_cell_3.AddIntAttribute(ax::mojom::IntAttribute::kTableCellRowIndex, 2);
  table_cell_3.AddIntAttribute(ax::mojom::IntAttribute::kTableCellColumnIndex,
                               1);
  table_row_2.child_ids.push_back(table_cell_3.id);

  AXNodeData table_cell_4;
  table_cell_4.id = 13;
  table_cell_4.role = ax::mojom::Role::kCell;
  table_cell_4.SetName("4");
  table_cell_4.AddIntAttribute(ax::mojom::IntAttribute::kTableCellRowIndex, 2);
  table_cell_4.AddIntAttribute(ax::mojom::IntAttribute::kTableCellColumnIndex,
                               2);
  table_row_2.child_ids.push_back(table_cell_4.id);

  AXTreeUpdate update;
  update.root_id = table.id;

  // Some of the table testing code will index into |nodes|
  // and change the state of the given node.  If you reorder
  // these, you're going to need to update the tests.
  update.nodes.push_back(table);  // 0

  update.nodes.push_back(table_row_header);       // 1
  update.nodes.push_back(table_column_header_1);  // 2
  update.nodes.push_back(table_column_header_2);  // 3
  update.nodes.push_back(table_column_header_3);  // 4

  update.nodes.push_back(table_row_1);         // 5
  update.nodes.push_back(table_row_header_1);  // 6
  update.nodes.push_back(table_cell_1);        // 7
  update.nodes.push_back(table_cell_2);        // 8

  update.nodes.push_back(table_row_2);         // 9
  update.nodes.push_back(table_row_header_2);  // 10
  update.nodes.push_back(table_cell_3);        // 11
  update.nodes.push_back(table_cell_4);        // 12

  return update;
}

AXTreeUpdate AXPlatformNodeTest::BuildAriaColumnAndRowCountGrids() {
  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kNone;

  // Empty Grid
  AXNodeData empty_grid;
  empty_grid.id = 2;
  empty_grid.role = ax::mojom::Role::kGrid;
  root.child_ids.push_back(empty_grid.id);

  // Grid with a cell that defines aria-rowindex (4) and aria-colindex (5)
  AXNodeData rowcolindex_grid;
  rowcolindex_grid.id = 3;
  rowcolindex_grid.role = ax::mojom::Role::kGrid;
  root.child_ids.push_back(rowcolindex_grid.id);

  AXNodeData rowcolindex_row;
  rowcolindex_row.id = 4;
  rowcolindex_row.role = ax::mojom::Role::kRow;
  rowcolindex_grid.child_ids.push_back(rowcolindex_row.id);

  AXNodeData rowcolindex_cell;
  rowcolindex_cell.id = 5;
  rowcolindex_cell.role = ax::mojom::Role::kCell;
  rowcolindex_cell.AddIntAttribute(
      ax::mojom::IntAttribute::kAriaCellColumnIndex, 5);
  rowcolindex_cell.AddIntAttribute(ax::mojom::IntAttribute::kAriaCellRowIndex,
                                   4);
  rowcolindex_row.child_ids.push_back(rowcolindex_cell.id);

  // Grid that specifies aria-rowcount (2) and aria-colcount (3)
  AXNodeData rowcolcount_grid;
  rowcolcount_grid.id = 6;
  rowcolcount_grid.role = ax::mojom::Role::kGrid;
  rowcolcount_grid.AddIntAttribute(ax::mojom::IntAttribute::kAriaRowCount, 2);
  rowcolcount_grid.AddIntAttribute(ax::mojom::IntAttribute::kAriaColumnCount,
                                   3);
  root.child_ids.push_back(rowcolcount_grid.id);

  // Grid that specifies aria-rowcount and aria-colcount are (-1)
  // ax::mojom::kUnknownAriaColumnOrRowCount
  AXNodeData unknown_grid;
  unknown_grid.id = 7;
  unknown_grid.role = ax::mojom::Role::kGrid;
  unknown_grid.AddIntAttribute(ax::mojom::IntAttribute::kAriaRowCount,
                               ax::mojom::kUnknownAriaColumnOrRowCount);
  unknown_grid.AddIntAttribute(ax::mojom::IntAttribute::kAriaColumnCount,
                               ax::mojom::kUnknownAriaColumnOrRowCount);
  root.child_ids.push_back(unknown_grid.id);

  AXTreeUpdate update;
  update.root_id = root.id;
  update.nodes.push_back(root);
  update.nodes.push_back(empty_grid);
  update.nodes.push_back(rowcolindex_grid);
  update.nodes.push_back(rowcolindex_row);
  update.nodes.push_back(rowcolindex_cell);
  update.nodes.push_back(rowcolcount_grid);
  update.nodes.push_back(unknown_grid);
  return update;
}

AXTreeUpdate AXPlatformNodeTest::BuildListBox(
    bool option_1_is_selected,
    bool option_2_is_selected,
    bool option_3_is_selected,
    const std::vector<ax::mojom::State>& additional_state) {
  AXNodeData listbox;
  listbox.id = 1;
  listbox.role = ax::mojom::Role::kListBox;
  listbox.SetName("ListBox");
  for (auto state : additional_state)
    listbox.AddState(state);

  AXNodeData option_1;
  option_1.id = 2;
  option_1.role = ax::mojom::Role::kListBoxOption;
  option_1.SetName("Option1");
  if (option_1_is_selected)
    option_1.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
  listbox.child_ids.push_back(option_1.id);

  AXNodeData option_2;
  option_2.id = 3;
  option_2.role = ax::mojom::Role::kListBoxOption;
  option_2.SetName("Option2");
  if (option_2_is_selected)
    option_2.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
  listbox.child_ids.push_back(option_2.id);

  AXNodeData option_3;
  option_3.id = 4;
  option_3.role = ax::mojom::Role::kListBoxOption;
  option_3.SetName("Option3");
  if (option_3_is_selected)
    option_3.AddBoolAttribute(ax::mojom::BoolAttribute::kSelected, true);
  listbox.child_ids.push_back(option_3.id);

  AXTreeUpdate update;
  update.root_id = listbox.id;
  update.nodes.push_back(listbox);
  update.nodes.push_back(option_1);
  update.nodes.push_back(option_2);
  update.nodes.push_back(option_3);
  return update;
}

}  // namespace ui

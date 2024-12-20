// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_table_info.h"

#include "gtest/gtest.h"

#include "ax_enums.h"
#include "ax_node.h"
#include "ax_tree.h"

namespace ui {

namespace {

void MakeTable(AXNodeData* table, int id, int row_count, int col_count) {
  table->id = id;
  table->role = ax::mojom::Role::kTable;
  table->AddIntAttribute(ax::mojom::IntAttribute::kTableRowCount, row_count);
  table->AddIntAttribute(ax::mojom::IntAttribute::kTableColumnCount, col_count);
}

void MakeRowGroup(AXNodeData* row_group, int id) {
  row_group->id = id;
  row_group->role = ax::mojom::Role::kRowGroup;
}

void MakeRow(AXNodeData* row, int id, int row_index) {
  row->id = id;
  row->role = ax::mojom::Role::kRow;
  row->AddIntAttribute(ax::mojom::IntAttribute::kTableRowIndex, row_index);
}

void MakeCell(AXNodeData* cell,
              int id,
              int row_index,
              int col_index,
              int row_span = 1,
              int col_span = 1) {
  cell->id = id;
  cell->role = ax::mojom::Role::kCell;
  cell->AddIntAttribute(ax::mojom::IntAttribute::kTableCellRowIndex, row_index);
  cell->AddIntAttribute(ax::mojom::IntAttribute::kTableCellColumnIndex,
                        col_index);
  if (row_span > 1)
    cell->AddIntAttribute(ax::mojom::IntAttribute::kTableCellRowSpan, row_span);
  if (col_span > 1)
    cell->AddIntAttribute(ax::mojom::IntAttribute::kTableCellColumnSpan,
                          col_span);
}

void MakeColumnHeader(AXNodeData* cell,
                      int id,
                      int row_index,
                      int col_index,
                      int row_span = 1,
                      int col_span = 1) {
  MakeCell(cell, id, row_index, col_index, row_span, col_span);
  cell->role = ax::mojom::Role::kColumnHeader;
}

void MakeRowHeader(AXNodeData* cell,
                   int id,
                   int row_index,
                   int col_index,
                   int row_span = 1,
                   int col_span = 1) {
  MakeCell(cell, id, row_index, col_index, row_span, col_span);
  cell->role = ax::mojom::Role::kRowHeader;
}

}  // namespace

// A macro for testing that a std::optional has both a value and that its value
// is set to a particular expectation.
#define EXPECT_OPTIONAL_EQ(expected, actual) \
  EXPECT_TRUE(actual.has_value());           \
  if (actual) {                              \
    EXPECT_EQ(expected, actual.value());     \
  }

class AXTableInfoTest : public testing::Test {
 public:
  AXTableInfoTest() {}
  ~AXTableInfoTest() override {}

 protected:
  AXTableInfo* GetTableInfo(AXTree* tree, AXNode* node) {
    return tree->GetTableInfo(node);
  }

 private:
  BASE_DISALLOW_COPY_AND_ASSIGN(AXTableInfoTest);
};

TEST_F(AXTableInfoTest, SimpleTable) {
  // Simple 2 x 2 table with 2 column headers in first row, 2 cells in second
  // row. The first row is parented by a rowgroup.
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(8);
  MakeTable(&initial_state.nodes[0], 1, 0, 0);
  initial_state.nodes[0].child_ids = {888, 3};

  MakeRowGroup(&initial_state.nodes[1], 888);
  initial_state.nodes[1].child_ids = {2};

  MakeRow(&initial_state.nodes[2], 2, 0);
  initial_state.nodes[2].child_ids = {4, 5};

  MakeRow(&initial_state.nodes[3], 3, 1);
  initial_state.nodes[3].child_ids = {6, 7};
  MakeColumnHeader(&initial_state.nodes[4], 4, 0, 0);
  MakeColumnHeader(&initial_state.nodes[5], 5, 0, 1);
  MakeCell(&initial_state.nodes[6], 6, 1, 0);
  MakeCell(&initial_state.nodes[7], 7, 1, 1);
  AXTree tree(initial_state);

  //
  // Low-level: test the AXTableInfo directly.
  //

  AXTableInfo* table_info = GetTableInfo(&tree, tree.root()->children()[0]);
  EXPECT_FALSE(table_info);

  table_info = GetTableInfo(&tree, tree.root());
  EXPECT_TRUE(table_info);

  EXPECT_EQ(2u, table_info->row_count);
  EXPECT_EQ(2u, table_info->col_count);

  EXPECT_EQ(2U, table_info->row_headers.size());
  EXPECT_EQ(0U, table_info->row_headers[0].size());
  EXPECT_EQ(0U, table_info->row_headers[1].size());

  EXPECT_EQ(2U, table_info->col_headers.size());
  EXPECT_EQ(1U, table_info->col_headers[0].size());
  EXPECT_EQ(4, table_info->col_headers[0][0]);
  EXPECT_EQ(1U, table_info->col_headers[1].size());
  EXPECT_EQ(5, table_info->col_headers[1][0]);

  EXPECT_EQ(4, table_info->cell_ids[0][0]);
  EXPECT_EQ(5, table_info->cell_ids[0][1]);
  EXPECT_EQ(6, table_info->cell_ids[1][0]);
  EXPECT_EQ(7, table_info->cell_ids[1][1]);

  EXPECT_EQ(4U, table_info->unique_cell_ids.size());
  EXPECT_EQ(4, table_info->unique_cell_ids[0]);
  EXPECT_EQ(5, table_info->unique_cell_ids[1]);
  EXPECT_EQ(6, table_info->unique_cell_ids[2]);
  EXPECT_EQ(7, table_info->unique_cell_ids[3]);

  EXPECT_EQ(0u, table_info->cell_id_to_index[4]);
  EXPECT_EQ(1u, table_info->cell_id_to_index[5]);
  EXPECT_EQ(2u, table_info->cell_id_to_index[6]);
  EXPECT_EQ(3u, table_info->cell_id_to_index[7]);

  EXPECT_EQ(2u, table_info->row_nodes.size());
  EXPECT_EQ(2, table_info->row_nodes[0]->data().id);
  EXPECT_EQ(3, table_info->row_nodes[1]->data().id);

  EXPECT_EQ(0U, table_info->extra_mac_nodes.size());

  //
  // High-level: Test the helper functions on AXNode.
  //

  AXNode* table = tree.root();
  EXPECT_TRUE(table->IsTable());
  EXPECT_FALSE(table->IsTableRow());
  EXPECT_FALSE(table->IsTableCellOrHeader());
  EXPECT_OPTIONAL_EQ(2, table->GetTableColCount());
  EXPECT_OPTIONAL_EQ(2, table->GetTableRowCount());

  ASSERT_TRUE(table->GetTableCellFromCoords(0, 0));
  EXPECT_EQ(4, table->GetTableCellFromCoords(0, 0)->id());
  EXPECT_EQ(5, table->GetTableCellFromCoords(0, 1)->id());
  EXPECT_EQ(6, table->GetTableCellFromCoords(1, 0)->id());
  EXPECT_EQ(7, table->GetTableCellFromCoords(1, 1)->id());
  EXPECT_EQ(nullptr, table->GetTableCellFromCoords(2, 1));
  EXPECT_EQ(nullptr, table->GetTableCellFromCoords(1, -1));

  EXPECT_EQ(4, table->GetTableCellFromIndex(0)->id());
  EXPECT_EQ(5, table->GetTableCellFromIndex(1)->id());
  EXPECT_EQ(6, table->GetTableCellFromIndex(2)->id());
  EXPECT_EQ(7, table->GetTableCellFromIndex(3)->id());
  EXPECT_EQ(nullptr, table->GetTableCellFromIndex(-1));
  EXPECT_EQ(nullptr, table->GetTableCellFromIndex(4));

  AXNode* row_0 = tree.GetFromId(2);
  EXPECT_FALSE(row_0->IsTable());
  EXPECT_TRUE(row_0->IsTableRow());
  EXPECT_FALSE(row_0->IsTableCellOrHeader());
  EXPECT_OPTIONAL_EQ(0, row_0->GetTableRowRowIndex());

  AXNode* row_1 = tree.GetFromId(3);
  EXPECT_FALSE(row_1->IsTable());
  EXPECT_TRUE(row_1->IsTableRow());
  EXPECT_FALSE(row_1->IsTableCellOrHeader());
  EXPECT_OPTIONAL_EQ(1, row_1->GetTableRowRowIndex());

  AXNode* cell_0_0 = tree.GetFromId(4);
  EXPECT_FALSE(cell_0_0->IsTable());
  EXPECT_FALSE(cell_0_0->IsTableRow());
  EXPECT_TRUE(cell_0_0->IsTableCellOrHeader());
  EXPECT_OPTIONAL_EQ(0, cell_0_0->GetTableCellIndex());
  EXPECT_OPTIONAL_EQ(0, cell_0_0->GetTableCellColIndex());
  EXPECT_OPTIONAL_EQ(0, cell_0_0->GetTableCellRowIndex());
  EXPECT_OPTIONAL_EQ(1, cell_0_0->GetTableCellColSpan());
  EXPECT_OPTIONAL_EQ(1, cell_0_0->GetTableCellRowSpan());

  AXNode* cell_1_1 = tree.GetFromId(7);
  EXPECT_FALSE(cell_1_1->IsTable());
  EXPECT_FALSE(cell_1_1->IsTableRow());
  EXPECT_TRUE(cell_1_1->IsTableCellOrHeader());
  EXPECT_OPTIONAL_EQ(3, cell_1_1->GetTableCellIndex());
  EXPECT_OPTIONAL_EQ(1, cell_1_1->GetTableCellRowIndex());
  EXPECT_OPTIONAL_EQ(1, cell_1_1->GetTableCellColSpan());
  EXPECT_OPTIONAL_EQ(1, cell_1_1->GetTableCellRowSpan());

  std::vector<AXNode*> col_headers;
  cell_1_1->GetTableCellColHeaders(&col_headers);
  EXPECT_EQ(1U, col_headers.size());
  EXPECT_EQ(5, col_headers[0]->id());

  std::vector<AXNode*> row_headers;
  cell_1_1->GetTableCellRowHeaders(&row_headers);
  EXPECT_EQ(0U, row_headers.size());

  EXPECT_EQ(2u, table->GetTableRowNodeIds().size());
  EXPECT_EQ(2, table->GetTableRowNodeIds()[0]);
  EXPECT_EQ(3, table->GetTableRowNodeIds()[1]);
  EXPECT_EQ(2u, row_0->GetTableRowNodeIds().size());
  EXPECT_EQ(2, row_0->GetTableRowNodeIds()[0]);
  EXPECT_EQ(3, row_0->GetTableRowNodeIds()[1]);
  EXPECT_EQ(2u, row_1->GetTableRowNodeIds().size());
  EXPECT_EQ(2, row_1->GetTableRowNodeIds()[0]);
  EXPECT_EQ(3, row_1->GetTableRowNodeIds()[1]);
  EXPECT_EQ(2u, cell_0_0->GetTableRowNodeIds().size());
  EXPECT_EQ(2, cell_0_0->GetTableRowNodeIds()[0]);
  EXPECT_EQ(3, cell_0_0->GetTableRowNodeIds()[1]);
  EXPECT_EQ(2u, cell_1_1->GetTableRowNodeIds().size());
  EXPECT_EQ(2, cell_1_1->GetTableRowNodeIds()[0]);
  EXPECT_EQ(3, cell_1_1->GetTableRowNodeIds()[1]);
}

TEST_F(AXTableInfoTest, ComputedTableSizeIncludesSpans) {
  // Simple 2 x 2 table with 2 column headers in first row, 2 cells in second
  // row, but two cells have spans, affecting the computed row and column count.
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(7);
  MakeTable(&initial_state.nodes[0], 1, 0, 0);
  initial_state.nodes[0].child_ids = {2, 3};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {4, 5};
  MakeRow(&initial_state.nodes[2], 3, 1);
  initial_state.nodes[2].child_ids = {6, 7};
  MakeCell(&initial_state.nodes[3], 4, 0, 0);
  MakeCell(&initial_state.nodes[4], 5, 0, 1, 1, 5);  // Column span of 5
  MakeCell(&initial_state.nodes[5], 6, 1, 0);
  MakeCell(&initial_state.nodes[6], 7, 1, 1, 3, 1);  // Row span of 3
  AXTree tree(initial_state);

  AXTableInfo* table_info = GetTableInfo(&tree, tree.root());
  EXPECT_EQ(4u, table_info->row_count);
  EXPECT_EQ(6u, table_info->col_count);

  EXPECT_EQ(2u, table_info->row_nodes.size());
  EXPECT_EQ(2, table_info->row_nodes[0]->data().id);
  EXPECT_EQ(3, table_info->row_nodes[1]->data().id);
}

TEST_F(AXTableInfoTest, AuthorRowAndColumnCountsAreRespected) {
  // Simple 1 x 1 table, but the table's authored row and column
  // counts imply a larger table (with missing cells).
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  MakeTable(&initial_state.nodes[0], 1, 8, 9);
  initial_state.nodes[0].child_ids = {2};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {3};
  MakeCell(&initial_state.nodes[2], 3, 0, 1);
  AXTree tree(initial_state);

  AXTableInfo* table_info = GetTableInfo(&tree, tree.root());
  EXPECT_EQ(8u, table_info->row_count);
  EXPECT_EQ(9u, table_info->col_count);

  EXPECT_EQ(1u, table_info->row_nodes.size());
  EXPECT_EQ(2, table_info->row_nodes[0]->data().id);
}

TEST_F(AXTableInfoTest, TableInfoRecomputedOnlyWhenTableChanges) {
  // Simple 1 x 1 table.
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(3);
  MakeTable(&initial_state.nodes[0], 1, 0, 0);
  initial_state.nodes[0].child_ids = {2};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {3};
  MakeCell(&initial_state.nodes[2], 3, 0, 0);
  AXTree tree(initial_state);

  AXTableInfo* table_info = GetTableInfo(&tree, tree.root());
  EXPECT_EQ(1u, table_info->row_count);
  EXPECT_EQ(1u, table_info->col_count);

  // Table info is cached.
  AXTableInfo* table_info_2 = GetTableInfo(&tree, tree.root());
  EXPECT_EQ(table_info, table_info_2);

  // Update the table so that the cell has a span.
  AXTreeUpdate update = initial_state;
  MakeCell(&update.nodes[2], 3, 0, 0, 1, 2);
  EXPECT_TRUE(tree.Unserialize(update));

  AXTableInfo* table_info_3 = GetTableInfo(&tree, tree.root());
  EXPECT_EQ(1u, table_info_3->row_count);
  EXPECT_EQ(2u, table_info_3->col_count);

  EXPECT_EQ(1u, table_info->row_nodes.size());
  EXPECT_EQ(2, table_info->row_nodes[0]->data().id);
}

TEST_F(AXTableInfoTest, CellIdsHandlesSpansAndMissingCells) {
  // 3 column x 2 row table with spans and missing cells:
  //
  // +---+---+---+
  // |   |   5   |
  // + 4 +---+---+
  // |   | 6 |
  // +---+---+
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(6);
  MakeTable(&initial_state.nodes[0], 1, 0, 0);
  initial_state.nodes[0].child_ids = {2, 3};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {4, 5};
  MakeRow(&initial_state.nodes[2], 3, 1);
  initial_state.nodes[2].child_ids = {6};
  MakeCell(&initial_state.nodes[3], 4, 0, 0, 2, 1);  // Row span of 2
  MakeCell(&initial_state.nodes[4], 5, 0, 1, 1, 5);  // Column span of 2
  MakeCell(&initial_state.nodes[5], 6, 1, 1);
  AXTree tree(initial_state);

  AXTableInfo* table_info = GetTableInfo(&tree, tree.root());
  EXPECT_EQ(4, table_info->cell_ids[0][0]);
  EXPECT_EQ(5, table_info->cell_ids[0][1]);
  EXPECT_EQ(5, table_info->cell_ids[0][1]);
  EXPECT_EQ(4, table_info->cell_ids[1][0]);
  EXPECT_EQ(6, table_info->cell_ids[1][1]);
  EXPECT_EQ(0, table_info->cell_ids[1][2]);

  EXPECT_EQ(3U, table_info->unique_cell_ids.size());
  EXPECT_EQ(4, table_info->unique_cell_ids[0]);
  EXPECT_EQ(5, table_info->unique_cell_ids[1]);
  EXPECT_EQ(6, table_info->unique_cell_ids[2]);

  EXPECT_EQ(0u, table_info->cell_id_to_index[4]);
  EXPECT_EQ(1u, table_info->cell_id_to_index[5]);
  EXPECT_EQ(2u, table_info->cell_id_to_index[6]);

  EXPECT_EQ(2u, table_info->row_nodes.size());
  EXPECT_EQ(2, table_info->row_nodes[0]->data().id);
  EXPECT_EQ(3, table_info->row_nodes[1]->data().id);
}

TEST_F(AXTableInfoTest, SkipsGenericAndIgnoredNodes) {
  // Simple 2 x 2 table with 2 cells in the first row, 2 cells in the second
  // row, but with extra divs and ignored nodes in the tree.
  //
  // 1 Table
  //   2 Row
  //     3 Ignored
  //       4 Generic
  //         5 Cell
  //       6 Cell
  //   7 Ignored
  //     8 Row
  //       9 Cell
  //       10 Cell

  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(10);
  MakeTable(&initial_state.nodes[0], 1, 0, 0);
  initial_state.nodes[0].child_ids = {2, 7};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {3};
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].AddState(ax::mojom::State::kIgnored);
  initial_state.nodes[2].child_ids = {4, 6};
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kGenericContainer;
  initial_state.nodes[3].child_ids = {5};
  MakeCell(&initial_state.nodes[4], 5, 0, 0);
  MakeCell(&initial_state.nodes[5], 6, 0, 1);
  initial_state.nodes[6].id = 7;
  initial_state.nodes[6].AddState(ax::mojom::State::kIgnored);
  initial_state.nodes[6].child_ids = {8};
  MakeRow(&initial_state.nodes[7], 8, 1);
  initial_state.nodes[7].child_ids = {9, 10};
  MakeCell(&initial_state.nodes[8], 9, 1, 0);
  MakeCell(&initial_state.nodes[9], 10, 1, 1);
  AXTree tree(initial_state);

  AXTableInfo* table_info = GetTableInfo(&tree, tree.root()->children()[0]);
  EXPECT_FALSE(table_info);

  table_info = GetTableInfo(&tree, tree.root());
  EXPECT_TRUE(table_info);

  EXPECT_EQ(2u, table_info->row_count);
  EXPECT_EQ(2u, table_info->col_count);

  EXPECT_EQ(5, table_info->cell_ids[0][0]);
  EXPECT_EQ(6, table_info->cell_ids[0][1]);
  EXPECT_EQ(9, table_info->cell_ids[1][0]);
  EXPECT_EQ(10, table_info->cell_ids[1][1]);

  EXPECT_EQ(2u, table_info->row_nodes.size());
  EXPECT_EQ(2, table_info->row_nodes[0]->data().id);
  EXPECT_EQ(8, table_info->row_nodes[1]->data().id);
}

TEST_F(AXTableInfoTest, HeadersWithSpans) {
  // Row and column headers spanning multiple cells.
  // In the figure below, 5 and 6 are headers.
  //
  //     +---+---+
  //     |   5   |
  // +---+---+---+
  // |   | 7 |
  // + 6 +---+---+
  // |   |   | 8 |
  // +---+   +---+
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(8);
  MakeTable(&initial_state.nodes[0], 1, 0, 0);
  initial_state.nodes[0].child_ids = {2, 3, 4};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {5};
  MakeRow(&initial_state.nodes[2], 3, 1);
  initial_state.nodes[2].child_ids = {6, 7};
  MakeRow(&initial_state.nodes[3], 4, 2);
  initial_state.nodes[3].child_ids = {8};
  MakeColumnHeader(&initial_state.nodes[4], 5, 0, 1, 1, 2);
  MakeRowHeader(&initial_state.nodes[5], 6, 1, 0, 2, 1);
  MakeCell(&initial_state.nodes[6], 7, 1, 1);
  MakeCell(&initial_state.nodes[7], 8, 2, 2);
  AXTree tree(initial_state);

  AXTableInfo* table_info = GetTableInfo(&tree, tree.root()->children()[0]);
  EXPECT_FALSE(table_info);

  table_info = GetTableInfo(&tree, tree.root());
  EXPECT_TRUE(table_info);

  EXPECT_EQ(3U, table_info->row_headers.size());
  EXPECT_EQ(0U, table_info->row_headers[0].size());
  EXPECT_EQ(1U, table_info->row_headers[1].size());
  EXPECT_EQ(6, table_info->row_headers[1][0]);
  EXPECT_EQ(1U, table_info->row_headers[1].size());
  EXPECT_EQ(6, table_info->row_headers[2][0]);

  EXPECT_EQ(3U, table_info->col_headers.size());
  EXPECT_EQ(0U, table_info->col_headers[0].size());
  EXPECT_EQ(1U, table_info->col_headers[1].size());
  EXPECT_EQ(5, table_info->col_headers[1][0]);
  EXPECT_EQ(1U, table_info->col_headers[2].size());
  EXPECT_EQ(5, table_info->col_headers[2][0]);

  EXPECT_EQ(0, table_info->cell_ids[0][0]);
  EXPECT_EQ(5, table_info->cell_ids[0][1]);
  EXPECT_EQ(5, table_info->cell_ids[0][2]);
  EXPECT_EQ(6, table_info->cell_ids[1][0]);
  EXPECT_EQ(7, table_info->cell_ids[1][1]);
  EXPECT_EQ(0, table_info->cell_ids[1][2]);
  EXPECT_EQ(6, table_info->cell_ids[2][0]);
  EXPECT_EQ(0, table_info->cell_ids[2][1]);
  EXPECT_EQ(8, table_info->cell_ids[2][2]);

  EXPECT_EQ(3u, table_info->row_nodes.size());
  EXPECT_EQ(2, table_info->row_nodes[0]->data().id);
  EXPECT_EQ(3, table_info->row_nodes[1]->data().id);
  EXPECT_EQ(4, table_info->row_nodes[2]->data().id);
}

TEST_F(AXTableInfoTest, ExtraMacNodes) {
  // Simple 2 x 2 table with 2 column headers in first row, 2 cells in second
  // row.
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(7);
  MakeTable(&initial_state.nodes[0], 1, 0, 0);
  initial_state.nodes[0].child_ids = {2, 3};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {4, 5};
  MakeRow(&initial_state.nodes[2], 3, 1);
  initial_state.nodes[2].child_ids = {6, 7};
  MakeColumnHeader(&initial_state.nodes[3], 4, 0, 0);
  MakeColumnHeader(&initial_state.nodes[4], 5, 0, 1);
  MakeCell(&initial_state.nodes[5], 6, 1, 0);
  MakeCell(&initial_state.nodes[6], 7, 1, 1);
  AXTree tree(initial_state);

  tree.SetEnableExtraMacNodes(true);
  AXTableInfo* table_info = GetTableInfo(&tree, tree.root()->children()[0]);
  EXPECT_FALSE(table_info);

  table_info = GetTableInfo(&tree, tree.root());
  EXPECT_TRUE(table_info);

  // We expect 3 extra Mac nodes: two column nodes, and one header node.
  EXPECT_EQ(3U, table_info->extra_mac_nodes.size());

  // The first column.
  AXNodeData extra_node_0 = table_info->extra_mac_nodes[0]->data();
  EXPECT_EQ(-1, table_info->extra_mac_nodes[0]->id());
  EXPECT_EQ(1, table_info->extra_mac_nodes[0]->parent()->id());
  EXPECT_EQ(ax::mojom::Role::kColumn, extra_node_0.role);
  EXPECT_EQ(2U, table_info->extra_mac_nodes[0]->GetIndexInParent());
  EXPECT_EQ(2U, table_info->extra_mac_nodes[0]->GetUnignoredIndexInParent());
  EXPECT_EQ(0, extra_node_0.GetIntAttribute(
                   ax::mojom::IntAttribute::kTableColumnIndex));
  std::vector<int32_t> indirect_child_ids;
  EXPECT_EQ(true, extra_node_0.GetIntListAttribute(
                      ax::mojom::IntListAttribute::kIndirectChildIds,
                      &indirect_child_ids));
  EXPECT_EQ(2U, indirect_child_ids.size());
  EXPECT_EQ(4, indirect_child_ids[0]);
  EXPECT_EQ(6, indirect_child_ids[1]);

  // The second column.
  AXNodeData extra_node_1 = table_info->extra_mac_nodes[1]->data();
  EXPECT_EQ(-2, table_info->extra_mac_nodes[1]->id());
  EXPECT_EQ(1, table_info->extra_mac_nodes[1]->parent()->id());
  EXPECT_EQ(ax::mojom::Role::kColumn, extra_node_1.role);
  EXPECT_EQ(3U, table_info->extra_mac_nodes[1]->GetIndexInParent());
  EXPECT_EQ(3U, table_info->extra_mac_nodes[1]->GetUnignoredIndexInParent());
  EXPECT_EQ(1, extra_node_1.GetIntAttribute(
                   ax::mojom::IntAttribute::kTableColumnIndex));
  indirect_child_ids.clear();
  EXPECT_EQ(true, extra_node_1.GetIntListAttribute(
                      ax::mojom::IntListAttribute::kIndirectChildIds,
                      &indirect_child_ids));
  EXPECT_EQ(2U, indirect_child_ids.size());
  EXPECT_EQ(5, indirect_child_ids[0]);
  EXPECT_EQ(7, indirect_child_ids[1]);

  // The table header container.
  AXNodeData extra_node_2 = table_info->extra_mac_nodes[2]->data();
  EXPECT_EQ(-3, table_info->extra_mac_nodes[2]->id());
  EXPECT_EQ(1, table_info->extra_mac_nodes[2]->parent()->id());
  EXPECT_EQ(ax::mojom::Role::kTableHeaderContainer, extra_node_2.role);
  EXPECT_EQ(4U, table_info->extra_mac_nodes[2]->GetIndexInParent());
  EXPECT_EQ(4U, table_info->extra_mac_nodes[2]->GetUnignoredIndexInParent());
  indirect_child_ids.clear();
  EXPECT_EQ(true, extra_node_2.GetIntListAttribute(
                      ax::mojom::IntListAttribute::kIndirectChildIds,
                      &indirect_child_ids));
  EXPECT_EQ(2U, indirect_child_ids.size());
  EXPECT_EQ(4, indirect_child_ids[0]);
  EXPECT_EQ(5, indirect_child_ids[1]);
}

TEST_F(AXTableInfoTest, TableWithNoIndices) {
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(7);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kTable;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kRow;
  initial_state.nodes[1].child_ids = {4, 5};
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kRow;
  initial_state.nodes[2].child_ids = {6, 7};
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kColumnHeader;
  initial_state.nodes[4].id = 5;
  initial_state.nodes[4].role = ax::mojom::Role::kColumnHeader;
  initial_state.nodes[5].id = 6;
  initial_state.nodes[5].role = ax::mojom::Role::kCell;
  initial_state.nodes[6].id = 7;
  initial_state.nodes[6].role = ax::mojom::Role::kCell;

  AXTree tree(initial_state);
  AXNode* table = tree.root();

  EXPECT_TRUE(table->IsTable());
  EXPECT_FALSE(table->IsTableRow());
  EXPECT_FALSE(table->IsTableCellOrHeader());
  EXPECT_EQ(2, table->GetTableColCount());
  EXPECT_EQ(2, table->GetTableRowCount());

  EXPECT_EQ(2u, table->GetTableRowNodeIds().size());
  EXPECT_EQ(2, table->GetTableRowNodeIds()[0]);
  EXPECT_EQ(3, table->GetTableRowNodeIds()[1]);

  EXPECT_EQ(4, table->GetTableCellFromCoords(0, 0)->id());
  EXPECT_EQ(5, table->GetTableCellFromCoords(0, 1)->id());
  EXPECT_EQ(6, table->GetTableCellFromCoords(1, 0)->id());
  EXPECT_EQ(7, table->GetTableCellFromCoords(1, 1)->id());
  EXPECT_EQ(nullptr, table->GetTableCellFromCoords(2, 1));
  EXPECT_EQ(nullptr, table->GetTableCellFromCoords(1, -1));

  EXPECT_EQ(4, table->GetTableCellFromIndex(0)->id());
  EXPECT_EQ(5, table->GetTableCellFromIndex(1)->id());
  EXPECT_EQ(6, table->GetTableCellFromIndex(2)->id());
  EXPECT_EQ(7, table->GetTableCellFromIndex(3)->id());
  EXPECT_EQ(nullptr, table->GetTableCellFromIndex(-1));
  EXPECT_EQ(nullptr, table->GetTableCellFromIndex(4));

  AXNode* cell_0_0 = tree.GetFromId(4);
  EXPECT_EQ(0, cell_0_0->GetTableCellRowIndex());
  EXPECT_EQ(0, cell_0_0->GetTableCellColIndex());
  AXNode* cell_0_1 = tree.GetFromId(5);
  EXPECT_EQ(0, cell_0_1->GetTableCellRowIndex());
  EXPECT_EQ(1, cell_0_1->GetTableCellColIndex());
  AXNode* cell_1_0 = tree.GetFromId(6);
  EXPECT_EQ(1, cell_1_0->GetTableCellRowIndex());
  EXPECT_EQ(0, cell_1_0->GetTableCellColIndex());
  AXNode* cell_1_1 = tree.GetFromId(7);
  EXPECT_EQ(1, cell_1_1->GetTableCellRowIndex());
  EXPECT_EQ(1, cell_1_1->GetTableCellColIndex());
}

TEST_F(AXTableInfoTest, TableWithPartialIndices) {
  // Start with a table with no indices.
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(7);
  initial_state.nodes[0].id = 1;
  initial_state.nodes[0].role = ax::mojom::Role::kTable;
  initial_state.nodes[0].child_ids = {2, 3};
  initial_state.nodes[1].id = 2;
  initial_state.nodes[1].role = ax::mojom::Role::kRow;
  initial_state.nodes[1].child_ids = {4, 5};
  initial_state.nodes[2].id = 3;
  initial_state.nodes[2].role = ax::mojom::Role::kRow;
  initial_state.nodes[2].child_ids = {6, 7};
  initial_state.nodes[3].id = 4;
  initial_state.nodes[3].role = ax::mojom::Role::kColumnHeader;
  initial_state.nodes[4].id = 5;
  initial_state.nodes[4].role = ax::mojom::Role::kColumnHeader;
  initial_state.nodes[5].id = 6;
  initial_state.nodes[5].role = ax::mojom::Role::kCell;
  initial_state.nodes[6].id = 7;
  initial_state.nodes[6].role = ax::mojom::Role::kCell;

  AXTree tree(initial_state);
  AXNode* table = tree.root();

  EXPECT_EQ(2, table->GetTableColCount());
  EXPECT_EQ(2, table->GetTableRowCount());

  AXNode* cell_0_0 = tree.GetFromId(4);
  EXPECT_EQ(0, cell_0_0->GetTableCellRowIndex());
  EXPECT_EQ(0, cell_0_0->GetTableCellColIndex());
  AXNode* cell_0_1 = tree.GetFromId(5);
  EXPECT_EQ(0, cell_0_1->GetTableCellRowIndex());
  EXPECT_EQ(1, cell_0_1->GetTableCellColIndex());
  AXNode* cell_1_0 = tree.GetFromId(6);
  EXPECT_EQ(1, cell_1_0->GetTableCellRowIndex());
  EXPECT_EQ(0, cell_1_0->GetTableCellColIndex());
  AXNode* cell_1_1 = tree.GetFromId(7);
  EXPECT_EQ(1, cell_1_1->GetTableCellRowIndex());
  EXPECT_EQ(1, cell_1_1->GetTableCellColIndex());

  AXTreeUpdate update = initial_state;
  update.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kTableColumnCount,
                                  5);
  update.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kTableRowCount, 2);
  update.nodes[5].AddIntAttribute(ax::mojom::IntAttribute::kTableCellRowIndex,
                                  2);
  update.nodes[5].AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellColumnIndex, 0);
  update.nodes[6].AddIntAttribute(ax::mojom::IntAttribute::kTableCellRowIndex,
                                  2);
  update.nodes[6].AddIntAttribute(
      ax::mojom::IntAttribute::kTableCellColumnIndex, 2);
  EXPECT_TRUE(tree.Unserialize(update));

  // The largest column index in the table is 2, but the
  // table claims it has a column count of 5. That's allowed.
  EXPECT_EQ(5, table->GetTableColCount());

  // While the table claims it has a row count of 2, the
  // last row has an index of 2, so the correct row count is 3.
  EXPECT_EQ(3, table->GetTableRowCount());

  EXPECT_EQ(2u, table->GetTableRowNodeIds().size());
  EXPECT_EQ(2, table->GetTableRowNodeIds()[0]);
  EXPECT_EQ(3, table->GetTableRowNodeIds()[1]);

  // All of the specified row and cell indices are legal
  // so they're respected.
  EXPECT_EQ(0, cell_0_0->GetTableCellRowIndex());
  EXPECT_EQ(0, cell_0_0->GetTableCellColIndex());
  EXPECT_EQ(0, cell_0_1->GetTableCellRowIndex());
  EXPECT_EQ(1, cell_0_1->GetTableCellColIndex());
  EXPECT_EQ(2, cell_1_0->GetTableCellRowIndex());
  EXPECT_EQ(0, cell_1_0->GetTableCellColIndex());
  EXPECT_EQ(2, cell_1_1->GetTableCellRowIndex());
  EXPECT_EQ(2, cell_1_1->GetTableCellColIndex());

  // Fetching cells by coordinates works.
  EXPECT_EQ(4, table->GetTableCellFromCoords(0, 0)->id());
  EXPECT_EQ(5, table->GetTableCellFromCoords(0, 1)->id());
  EXPECT_EQ(6, table->GetTableCellFromCoords(2, 0)->id());
  EXPECT_EQ(7, table->GetTableCellFromCoords(2, 2)->id());
  EXPECT_EQ(nullptr, table->GetTableCellFromCoords(0, 2));
  EXPECT_EQ(nullptr, table->GetTableCellFromCoords(1, 0));
  EXPECT_EQ(nullptr, table->GetTableCellFromCoords(1, 1));
  EXPECT_EQ(nullptr, table->GetTableCellFromCoords(2, 1));
}

TEST_F(AXTableInfoTest, BadRowIndicesIgnored) {
  // The table claims it has two rows and two columns, but
  // the cell indices for both the first and second rows
  // are for row 2 (zero-based).
  //
  // The cell indexes for the first row should be
  // respected, and for the second row it will get the
  // next row index.
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(7);
  MakeTable(&initial_state.nodes[0], 1, 2, 2);
  initial_state.nodes[0].child_ids = {2, 3};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {4, 5};
  MakeRow(&initial_state.nodes[2], 3, 0);
  initial_state.nodes[2].child_ids = {6, 7};
  MakeCell(&initial_state.nodes[3], 4, 2, 0);
  MakeCell(&initial_state.nodes[4], 5, 2, 1);
  MakeCell(&initial_state.nodes[5], 6, 2, 0);
  MakeCell(&initial_state.nodes[6], 7, 2, 1);
  AXTree tree(initial_state);
  AXNode* table = tree.root();

  EXPECT_EQ(2, table->GetTableColCount());
  EXPECT_EQ(4, table->GetTableRowCount());

  EXPECT_EQ(2u, table->GetTableRowNodeIds().size());
  EXPECT_EQ(2, table->GetTableRowNodeIds()[0]);
  EXPECT_EQ(3, table->GetTableRowNodeIds()[1]);

  AXNode* cell_id_4 = tree.GetFromId(4);
  EXPECT_EQ(2, cell_id_4->GetTableCellRowIndex());
  EXPECT_EQ(0, cell_id_4->GetTableCellColIndex());
  AXNode* cell_id_5 = tree.GetFromId(5);
  EXPECT_EQ(2, cell_id_5->GetTableCellRowIndex());
  EXPECT_EQ(1, cell_id_5->GetTableCellColIndex());
  AXNode* cell_id_6 = tree.GetFromId(6);
  EXPECT_EQ(3, cell_id_6->GetTableCellRowIndex());
  EXPECT_EQ(0, cell_id_6->GetTableCellColIndex());
  AXNode* cell_id_7 = tree.GetFromId(7);
  EXPECT_EQ(3, cell_id_7->GetTableCellRowIndex());
  EXPECT_EQ(1, cell_id_7->GetTableCellColIndex());
}

TEST_F(AXTableInfoTest, BadColIndicesIgnored) {
  // The table claims it has two rows and two columns, but
  // the cell indices for the columns either repeat or
  // go backwards.
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(7);
  MakeTable(&initial_state.nodes[0], 1, 2, 2);
  initial_state.nodes[0].child_ids = {2, 3};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {4, 5};
  MakeRow(&initial_state.nodes[2], 3, 0);
  initial_state.nodes[2].child_ids = {6, 7};
  MakeCell(&initial_state.nodes[3], 4, 0, 1);
  MakeCell(&initial_state.nodes[4], 5, 0, 1);
  MakeCell(&initial_state.nodes[5], 6, 1, 2);
  MakeCell(&initial_state.nodes[6], 7, 1, 1);
  AXTree tree(initial_state);
  AXNode* table = tree.root();

  EXPECT_EQ(4, table->GetTableColCount());
  EXPECT_EQ(2, table->GetTableRowCount());

  EXPECT_EQ(2u, table->GetTableRowNodeIds().size());
  EXPECT_EQ(2, table->GetTableRowNodeIds()[0]);
  EXPECT_EQ(3, table->GetTableRowNodeIds()[1]);

  AXNode* cell_id_4 = tree.GetFromId(4);
  EXPECT_EQ(0, cell_id_4->GetTableCellRowIndex());
  EXPECT_EQ(1, cell_id_4->GetTableCellColIndex());
  AXNode* cell_id_5 = tree.GetFromId(5);
  EXPECT_EQ(0, cell_id_5->GetTableCellRowIndex());
  EXPECT_EQ(2, cell_id_5->GetTableCellColIndex());
  AXNode* cell_id_6 = tree.GetFromId(6);
  EXPECT_EQ(1, cell_id_6->GetTableCellRowIndex());
  EXPECT_EQ(2, cell_id_6->GetTableCellColIndex());
  AXNode* cell_id_7 = tree.GetFromId(7);
  EXPECT_EQ(1, cell_id_7->GetTableCellRowIndex());
  EXPECT_EQ(3, cell_id_7->GetTableCellColIndex());
}

TEST_F(AXTableInfoTest, AriaIndicesInferred) {
  // Note that ARIA indices are 1-based, whereas the rest of
  // the table indices are zero-based.
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(13);
  MakeTable(&initial_state.nodes[0], 1, 3, 3);
  initial_state.nodes[0].AddIntAttribute(ax::mojom::IntAttribute::kAriaRowCount,
                                         5);
  initial_state.nodes[0].AddIntAttribute(
      ax::mojom::IntAttribute::kAriaColumnCount, 5);
  initial_state.nodes[0].child_ids = {2, 3, 4};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {5, 6, 7};
  MakeRow(&initial_state.nodes[2], 3, 1);
  initial_state.nodes[2].AddIntAttribute(
      ax::mojom::IntAttribute::kAriaCellRowIndex, 4);
  initial_state.nodes[2].child_ids = {8, 9, 10};
  MakeRow(&initial_state.nodes[3], 4, 2);
  initial_state.nodes[3].AddIntAttribute(
      ax::mojom::IntAttribute::kAriaCellRowIndex, 4);
  initial_state.nodes[3].child_ids = {11, 12, 13};
  MakeCell(&initial_state.nodes[4], 5, 0, 0);
  initial_state.nodes[4].AddIntAttribute(
      ax::mojom::IntAttribute::kAriaCellRowIndex, 2);
  initial_state.nodes[4].AddIntAttribute(
      ax::mojom::IntAttribute::kAriaCellColumnIndex, 2);
  MakeCell(&initial_state.nodes[5], 6, 0, 1);
  MakeCell(&initial_state.nodes[6], 7, 0, 2);
  MakeCell(&initial_state.nodes[7], 8, 1, 0);
  MakeCell(&initial_state.nodes[8], 9, 1, 1);
  MakeCell(&initial_state.nodes[9], 10, 1, 2);
  MakeCell(&initial_state.nodes[10], 11, 2, 0);
  initial_state.nodes[10].AddIntAttribute(
      ax::mojom::IntAttribute::kAriaCellColumnIndex, 3);
  MakeCell(&initial_state.nodes[11], 12, 2, 1);
  initial_state.nodes[11].AddIntAttribute(
      ax::mojom::IntAttribute::kAriaCellColumnIndex, 2);
  MakeCell(&initial_state.nodes[12], 13, 2, 2);
  initial_state.nodes[12].AddIntAttribute(
      ax::mojom::IntAttribute::kAriaCellColumnIndex, 1);
  AXTree tree(initial_state);
  AXNode* table = tree.root();

  EXPECT_EQ(5, table->GetTableAriaColCount());
  EXPECT_EQ(5, table->GetTableAriaRowCount());

  EXPECT_EQ(3u, table->GetTableRowNodeIds().size());
  EXPECT_EQ(2, table->GetTableRowNodeIds()[0]);
  EXPECT_EQ(3, table->GetTableRowNodeIds()[1]);
  EXPECT_EQ(4, table->GetTableRowNodeIds()[2]);

  // The first row has the first cell ARIA row and column index
  // specified as (2, 2). The rest of the row is inferred.

  AXNode* cell_0_0 = tree.GetFromId(5);
  EXPECT_EQ(2, cell_0_0->GetTableCellAriaRowIndex());
  EXPECT_EQ(2, cell_0_0->GetTableCellAriaColIndex());

  AXNode* cell_0_1 = tree.GetFromId(6);
  EXPECT_EQ(2, cell_0_1->GetTableCellAriaRowIndex());
  EXPECT_EQ(3, cell_0_1->GetTableCellAriaColIndex());

  AXNode* cell_0_2 = tree.GetFromId(7);
  EXPECT_EQ(2, cell_0_2->GetTableCellAriaRowIndex());
  EXPECT_EQ(4, cell_0_2->GetTableCellAriaColIndex());

  // The next row has the ARIA row index set to 4 on the row
  // element. The rest is inferred.

  AXNode* cell_1_0 = tree.GetFromId(8);
  EXPECT_EQ(4, cell_1_0->GetTableCellAriaRowIndex());
  EXPECT_EQ(1, cell_1_0->GetTableCellAriaColIndex());

  AXNode* cell_1_1 = tree.GetFromId(9);
  EXPECT_EQ(4, cell_1_1->GetTableCellAriaRowIndex());
  EXPECT_EQ(2, cell_1_1->GetTableCellAriaColIndex());

  AXNode* cell_1_2 = tree.GetFromId(10);
  EXPECT_EQ(4, cell_1_2->GetTableCellAriaRowIndex());
  EXPECT_EQ(3, cell_1_2->GetTableCellAriaColIndex());

  // The last row has the ARIA row index set to 4 again, which is
  // illegal so we should get 5. The cells have column indices of
  // 3, 2, 1 which is illegal so we ignore the latter two and should
  // end up with column indices of 3, 4, 5.

  AXNode* cell_2_0 = tree.GetFromId(11);
  EXPECT_EQ(5, cell_2_0->GetTableCellAriaRowIndex());
  EXPECT_EQ(3, cell_2_0->GetTableCellAriaColIndex());

  AXNode* cell_2_1 = tree.GetFromId(12);
  EXPECT_EQ(5, cell_2_1->GetTableCellAriaRowIndex());
  EXPECT_EQ(4, cell_2_1->GetTableCellAriaColIndex());

  AXNode* cell_2_2 = tree.GetFromId(13);
  EXPECT_EQ(5, cell_2_2->GetTableCellAriaRowIndex());
  EXPECT_EQ(5, cell_2_2->GetTableCellAriaColIndex());
}

TEST_F(AXTableInfoTest, TableChanges) {
  // Simple 2 col x 1 row table
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(4);
  MakeTable(&initial_state.nodes[0], 1, 0, 0);
  initial_state.nodes[0].child_ids = {2};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {3, 4};
  MakeCell(&initial_state.nodes[2], 3, 0, 0);
  MakeCell(&initial_state.nodes[3], 4, 0, 1);
  AXTree tree(initial_state);

  AXTableInfo* table_info = GetTableInfo(&tree, tree.root());
  EXPECT_TRUE(table_info);

  EXPECT_EQ(1u, table_info->row_count);
  EXPECT_EQ(2u, table_info->col_count);

  // Update the tree to remove the table role.
  AXTreeUpdate update = initial_state;
  update.nodes[0].role = ax::mojom::Role::kGroup;
  ASSERT_TRUE(tree.Unserialize(update));

  table_info = GetTableInfo(&tree, tree.root());
  EXPECT_FALSE(table_info);
}

TEST_F(AXTableInfoTest, ExtraMacNodesChanges) {
  // Simple 2 x 2 table with 2 column headers in first row, 2 cells in second
  // row.
  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes.resize(7);
  MakeTable(&initial_state.nodes[0], 1, 0, 0);
  initial_state.nodes[0].child_ids = {2, 3};
  MakeRow(&initial_state.nodes[1], 2, 0);
  initial_state.nodes[1].child_ids = {4, 5};
  MakeRow(&initial_state.nodes[2], 3, 1);
  initial_state.nodes[2].child_ids = {6, 7};
  MakeColumnHeader(&initial_state.nodes[3], 4, 0, 0);
  MakeColumnHeader(&initial_state.nodes[4], 5, 0, 1);
  MakeCell(&initial_state.nodes[5], 6, 1, 0);
  MakeCell(&initial_state.nodes[6], 7, 1, 1);
  AXTree tree(initial_state);

  tree.SetEnableExtraMacNodes(true);
  AXTableInfo* table_info = GetTableInfo(&tree, tree.root());
  ASSERT_NE(nullptr, table_info);
  // We expect 3 extra Mac nodes: two column nodes, and one header node.
  ASSERT_EQ(3U, table_info->extra_mac_nodes.size());

  // Hide the first row. The number of extra Mac nodes should remain the same,
  // but their data should change.
  AXTreeUpdate update1;
  update1.nodes.resize(1);
  MakeRow(&update1.nodes[0], 2, 0);
  update1.nodes[0].AddState(ax::mojom::State::kIgnored);
  update1.nodes[0].child_ids = {4, 5};
  ASSERT_TRUE(tree.Unserialize(update1));
  table_info = GetTableInfo(&tree, tree.root());
  ASSERT_EQ(3U, table_info->extra_mac_nodes.size());

  {
    // The first column.
    AXNodeData extra_node_0 = table_info->extra_mac_nodes[0]->data();
    EXPECT_EQ(-4, table_info->extra_mac_nodes[0]->id());
    EXPECT_EQ(1, table_info->extra_mac_nodes[0]->parent()->id());
    EXPECT_EQ(ax::mojom::Role::kColumn, extra_node_0.role);
    EXPECT_EQ(2U, table_info->extra_mac_nodes[0]->GetIndexInParent());
    EXPECT_EQ(3U, table_info->extra_mac_nodes[0]->GetUnignoredIndexInParent());
    EXPECT_EQ(0, extra_node_0.GetIntAttribute(
                     ax::mojom::IntAttribute::kTableColumnIndex));
    std::vector<int32_t> indirect_child_ids;
    EXPECT_EQ(true, extra_node_0.GetIntListAttribute(
                        ax::mojom::IntListAttribute::kIndirectChildIds,
                        &indirect_child_ids));
    EXPECT_EQ(1U, indirect_child_ids.size());
    EXPECT_EQ(6, indirect_child_ids[0]);

    // The second column.
    AXNodeData extra_node_1 = table_info->extra_mac_nodes[1]->data();
    EXPECT_EQ(-5, table_info->extra_mac_nodes[1]->id());
    EXPECT_EQ(1, table_info->extra_mac_nodes[1]->parent()->id());
    EXPECT_EQ(ax::mojom::Role::kColumn, extra_node_1.role);
    EXPECT_EQ(3U, table_info->extra_mac_nodes[1]->GetIndexInParent());
    EXPECT_EQ(4U, table_info->extra_mac_nodes[1]->GetUnignoredIndexInParent());
    EXPECT_EQ(1, extra_node_1.GetIntAttribute(
                     ax::mojom::IntAttribute::kTableColumnIndex));
    indirect_child_ids.clear();
    EXPECT_EQ(true, extra_node_1.GetIntListAttribute(
                        ax::mojom::IntListAttribute::kIndirectChildIds,
                        &indirect_child_ids));
    EXPECT_EQ(1U, indirect_child_ids.size());
    EXPECT_EQ(7, indirect_child_ids[0]);

    // The table header container.
    AXNodeData extra_node_2 = table_info->extra_mac_nodes[2]->data();
    EXPECT_EQ(-6, table_info->extra_mac_nodes[2]->id());
    EXPECT_EQ(1, table_info->extra_mac_nodes[2]->parent()->id());
    EXPECT_EQ(ax::mojom::Role::kTableHeaderContainer, extra_node_2.role);
    EXPECT_EQ(4U, table_info->extra_mac_nodes[2]->GetIndexInParent());
    EXPECT_EQ(5U, table_info->extra_mac_nodes[2]->GetUnignoredIndexInParent());
    indirect_child_ids.clear();
    EXPECT_EQ(true, extra_node_2.GetIntListAttribute(
                        ax::mojom::IntListAttribute::kIndirectChildIds,
                        &indirect_child_ids));
    EXPECT_EQ(0U, indirect_child_ids.size());
  }

  // Delete the first row. Again, the number of extra Mac nodes should remain
  // the same, but their data should change.
  AXTreeUpdate update2;
  update2.node_id_to_clear = 2;
  update2.nodes.resize(1);
  MakeTable(&update2.nodes[0], 1, 0, 0);
  update2.nodes[0].child_ids = {3};
  ASSERT_TRUE(tree.Unserialize(update2));
  table_info = GetTableInfo(&tree, tree.root());
  ASSERT_EQ(3U, table_info->extra_mac_nodes.size());

  {
    // The first column.
    AXNodeData extra_node_0 = table_info->extra_mac_nodes[0]->data();
    EXPECT_EQ(-7, table_info->extra_mac_nodes[0]->id());
    EXPECT_EQ(1, table_info->extra_mac_nodes[0]->parent()->id());
    EXPECT_EQ(ax::mojom::Role::kColumn, extra_node_0.role);
    EXPECT_EQ(1U, table_info->extra_mac_nodes[0]->GetIndexInParent());
    EXPECT_EQ(1U, table_info->extra_mac_nodes[0]->GetUnignoredIndexInParent());
    EXPECT_EQ(0, extra_node_0.GetIntAttribute(
                     ax::mojom::IntAttribute::kTableColumnIndex));
    std::vector<int32_t> indirect_child_ids;
    EXPECT_EQ(true, extra_node_0.GetIntListAttribute(
                        ax::mojom::IntListAttribute::kIndirectChildIds,
                        &indirect_child_ids));
    EXPECT_EQ(1U, indirect_child_ids.size());
    EXPECT_EQ(6, indirect_child_ids[0]);

    // The second column.
    AXNodeData extra_node_1 = table_info->extra_mac_nodes[1]->data();
    EXPECT_EQ(-8, table_info->extra_mac_nodes[1]->id());
    EXPECT_EQ(1, table_info->extra_mac_nodes[1]->parent()->id());
    EXPECT_EQ(ax::mojom::Role::kColumn, extra_node_1.role);
    EXPECT_EQ(2U, table_info->extra_mac_nodes[1]->GetIndexInParent());
    EXPECT_EQ(2U, table_info->extra_mac_nodes[1]->GetUnignoredIndexInParent());
    EXPECT_EQ(1, extra_node_1.GetIntAttribute(
                     ax::mojom::IntAttribute::kTableColumnIndex));
    indirect_child_ids.clear();
    EXPECT_EQ(true, extra_node_1.GetIntListAttribute(
                        ax::mojom::IntListAttribute::kIndirectChildIds,
                        &indirect_child_ids));
    EXPECT_EQ(1U, indirect_child_ids.size());
    EXPECT_EQ(7, indirect_child_ids[0]);

    // The table header container.
    AXNodeData extra_node_2 = table_info->extra_mac_nodes[2]->data();
    EXPECT_EQ(-9, table_info->extra_mac_nodes[2]->id());
    EXPECT_EQ(1, table_info->extra_mac_nodes[2]->parent()->id());
    EXPECT_EQ(ax::mojom::Role::kTableHeaderContainer, extra_node_2.role);
    EXPECT_EQ(3U, table_info->extra_mac_nodes[2]->GetIndexInParent());
    EXPECT_EQ(3U, table_info->extra_mac_nodes[2]->GetUnignoredIndexInParent());
    indirect_child_ids.clear();
    EXPECT_EQ(true, extra_node_2.GetIntListAttribute(
                        ax::mojom::IntListAttribute::kIndirectChildIds,
                        &indirect_child_ids));
    EXPECT_EQ(0U, indirect_child_ids.size());
  }
}

TEST_F(AXTableInfoTest, RowColumnSpanChanges) {
  // Simple 2 col x 1 row table
  AXTreeUpdate update;
  update.root_id = 1;
  update.nodes.resize(4);
  MakeTable(&update.nodes[0], 1, 0, 0);
  update.nodes[0].child_ids = {2};
  MakeRow(&update.nodes[1], 2, 0);
  update.nodes[1].child_ids = {3, 10};
  MakeCell(&update.nodes[2], 3, 0, 0);
  MakeCell(&update.nodes[3], 10, 0, 1);
  AXTree tree(update);

  AXTableInfo* table_info = GetTableInfo(&tree, tree.root());
  ASSERT_TRUE(table_info);

  EXPECT_EQ(1u, table_info->row_count);
  EXPECT_EQ(2u, table_info->col_count);

  EXPECT_EQ("|3 |10|\n", table_info->ToString());

  // Add a row to the table.
  update.nodes.resize(6);
  update.nodes[0].child_ids = {2, 4};
  MakeRow(&update.nodes[4], 4, 0);
  update.nodes[4].child_ids = {5};
  MakeCell(&update.nodes[5], 5, -1, -1);

  tree.Unserialize(update);

  table_info = GetTableInfo(&tree, tree.root());
  ASSERT_TRUE(table_info);
  EXPECT_EQ(2u, table_info->row_count);
  EXPECT_EQ(2u, table_info->col_count);
  EXPECT_EQ(
      "|3 |10|\n"
      "|5 |0 |\n",
      table_info->ToString());

  // Add a row to the middle of the table, with a span. Intentionally omit other
  // rows from the update.
  update.nodes.resize(3);
  update.nodes[0].child_ids = {2, 6, 4};
  MakeRow(&update.nodes[1], 6, 0);
  update.nodes[1].child_ids = {7};
  MakeCell(&update.nodes[2], 7, -1, -1, 1, 2);

  tree.Unserialize(update);

  table_info = GetTableInfo(&tree, tree.root());
  ASSERT_TRUE(table_info);
  EXPECT_EQ(3u, table_info->row_count);
  EXPECT_EQ(2u, table_info->col_count);
  EXPECT_EQ(
      "|3 |10|\n"
      "|7 |7 |\n"
      "|5 |0 |\n",
      table_info->ToString());

  // Add a row to the end of the table, with a span. Intentionally omit other
  // rows from the update.
  update.nodes.resize(3);
  update.nodes[0].child_ids = {2, 6, 4, 8};
  MakeRow(&update.nodes[1], 8, 0);
  update.nodes[1].child_ids = {9};
  MakeCell(&update.nodes[2], 9, -1, -1, 2, 3);

  tree.Unserialize(update);

  table_info = GetTableInfo(&tree, tree.root());
  ASSERT_TRUE(table_info);
  EXPECT_EQ(5u, table_info->row_count);
  EXPECT_EQ(3u, table_info->col_count);
  EXPECT_EQ(
      "|3 |10|0 |\n"
      "|7 |7 |0 |\n"
      "|5 |0 |0 |\n"
      "|9 |9 |9 |\n"
      "|9 |9 |9 |\n",
      table_info->ToString());

  // Finally, delete a few rows.
  update.nodes.resize(1);
  update.nodes[0].child_ids = {6, 8};

  tree.Unserialize(update);

  table_info = GetTableInfo(&tree, tree.root());
  ASSERT_TRUE(table_info);
  EXPECT_EQ(3u, table_info->row_count);
  EXPECT_EQ(3u, table_info->col_count);
  EXPECT_EQ(
      "|7|7|0|\n"
      "|9|9|9|\n"
      "|9|9|9|\n",
      table_info->ToString());
}

}  // namespace ui

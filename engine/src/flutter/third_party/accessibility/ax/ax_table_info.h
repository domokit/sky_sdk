// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_TABLE_INFO_H_
#define UI_ACCESSIBILITY_AX_TABLE_INFO_H_

#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "ax_export.h"

namespace ui {

class AXTree;
class AXNode;

// This helper class computes info about tables and grids in AXTrees.
class AX_EXPORT AXTableInfo {
 public:
  struct CellData {
    AXNode* cell;
    int32_t cell_id;
    size_t col_index;
    size_t row_index;
    size_t col_span;
    size_t row_span;
    size_t aria_col_index;
    size_t aria_row_index;
  };

  // Returns nullptr if the node is not a valid table or grid node.
  static AXTableInfo* Create(AXTree* tree, AXNode* table_node);

  ~AXTableInfo();

  // Called automatically on Create(), but must be called again any time
  // the table is invalidated. Returns true if this is still a table.
  bool Update();

  // Whether the data is valid. Whenever the tree is updated in any way,
  // every AXTableInfo is invalidated and needs to be recomputed, just
  // to be safe.
  bool valid() const { return valid_; }
  void Invalidate();

  // The real row count, guaranteed to be at least as large as the
  // maximum row index of any cell.
  size_t row_count = 0;

  // The real column count, guaranteed to be at least as large as the
  // maximum column index of any cell.
  size_t col_count = 0;

  // List of column header nodes IDs for each column index.
  std::vector<std::vector<int32_t>> col_headers;

  // List of row header node IDs for each row index.
  std::vector<std::vector<int32_t>> row_headers;

  // All header cells.
  std::vector<int32_t> all_headers;

  // The id of the element with the caption tag or ARIA role.
  int32_t caption_id;

  // 2-D array of [row][column] -> cell node ID.
  // This may contain duplicates if there is a rowspan or
  // colspan. The entry is empty (zero) only if the cell
  // really is missing from the table.
  std::vector<std::vector<int32_t>> cell_ids;

  // Array of cell data for every unique cell in the table.
  std::vector<CellData> cell_data_vector;

  // Set of all unique cell node IDs in the table.
  std::vector<int32_t> unique_cell_ids;

  // Extra computed nodes for the accessibility tree for macOS:
  // one column node for each table column, followed by one
  // table header container node.
  std::vector<AXNode*> extra_mac_nodes;

  // Map from each cell's node ID to its index in unique_cell_ids.
  std::unordered_map<int32_t, size_t> cell_id_to_index;

  // Map from each row's node ID to its row index.
  std::unordered_map<int32_t, size_t> row_id_to_index;

  // List of ax nodes that represent the rows of the table.
  std::vector<AXNode*> row_nodes;

  // The ARIA row count and column count, if any ARIA table or grid
  // attributes are used in the table at all.
  int aria_row_count = 0;
  int aria_col_count = 0;

  std::string ToString() const;

 private:
  AXTableInfo(AXTree* tree, AXNode* table_node);

  void ClearVectors();
  void BuildCellDataVectorFromRowAndCellNodes(
      const std::vector<AXNode*>& row_node_list,
      const std::vector<std::vector<AXNode*>>& cell_nodes_per_row);
  void BuildCellAndHeaderVectorsFromCellData();
  void UpdateExtraMacNodes();
  void ClearExtraMacNodes();
  AXNode* CreateExtraMacColumnNode(size_t col_index);
  AXNode* CreateExtraMacTableHeaderNode();
  void UpdateExtraMacColumnNodeAttributes(size_t col_index);

  AXTree* tree_ = nullptr;
  AXNode* table_node_ = nullptr;
  bool valid_ = false;
  std::map<int, std::map<int, CellData>> incremental_row_col_map_;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_TABLE_INFO

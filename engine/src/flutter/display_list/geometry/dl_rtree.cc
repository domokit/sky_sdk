// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/geometry/dl_rtree.h"
#include "flutter/display_list/geometry/dl_region.h"

#include "flutter/fml/logging.h"

namespace flutter {

DlRTree::DlRTree(const DlRect rects[],
                 int N,
                 const int ids[],
                 bool p(int),
                 int invalid_id)
    : invalid_id_(invalid_id) {
  if (N <= 0) {
    FML_DCHECK(N >= 0);
    return;
  }
  FML_DCHECK(rects != nullptr);

  // Count the number of rectangles we actually want to track,
  // which includes only non-empty rectangles whose optional
  // ID is not filtered by the predicate.
  int leaf_count = 0;
  for (int i = 0; i < N; i++) {
    if (!rects[i].IsEmpty()) {
      if (ids == nullptr || p(ids[i])) {
        leaf_count++;
      }
    }
  }
  leaf_count_ = leaf_count;

  // Count the total number of nodes (leaf and internal) up front
  // so we can resize the vector just once.
  uint32_t total_node_count = leaf_count;
  uint32_t gen_count = leaf_count;
  while (gen_count > 1) {
    uint32_t family_count = (gen_count + kMaxChildren - 1u) / kMaxChildren;
    total_node_count += family_count;
    gen_count = family_count;
  }

  nodes_.resize(total_node_count);

  // Now place only the tracked rectangles into the nodes array
  // in the first leaf_count_ entries.
  int leaf_index = 0;
  int id = invalid_id;
  for (int i = 0; i < N; i++) {
    if (!rects[i].IsEmpty()) {
      if (ids == nullptr || p(id = ids[i])) {
        Node& node = nodes_[leaf_index++];
        node.bounds = rects[i];
        node.id = id;
      }
    }
  }
  FML_DCHECK(leaf_index == leaf_count);

  // --- Implementation note ---
  // Many R-Tree algorithms attempt to consolidate nearby rectangles
  // into branches of the tree in order to maximize the benefit of
  // bounds testing against whole sub-trees. The Skia code from which
  // this was based, though, indicated that empirical tests against a
  // browser client showed little gains in rendering performance while
  // costing 17% performance in bulk loading the rects into the R-Tree:
  // https://github.com/google/skia/blob/12b6bd042f7cdffb9012c90c3b4885601fc7be95/src/core/SkRTree.cpp#L96
  //
  // Given that this class will most often be used to track rendering
  // operations that were drawn in an app that performs a type of
  // "page layout" with rendering proceeding in a linear fashion from
  // top to bottom (and left to right or right to left), the rectangles
  // are likely nearly sorted when they are delivered to this constructor
  // so leaving them in their original order should show similar results
  // to what Skia found in their empirical browser tests.
  // ---

  // Continually process the previous level (generation) of nodes,
  // combining them into a new generation of parent groups each grouping
  // at most |kMaxChildren| children and joining their bounds into its
  // parent bounds.
  // Each generation will end up reduced by a factor of up to kMaxChildren
  // until there is just one node left, which is the root node of
  // the R-Tree.
  uint32_t gen_start = 0;
  gen_count = leaf_count;
  while (gen_count > 1) {
    uint32_t gen_end = gen_start + gen_count;

    uint32_t family_count = (gen_count + kMaxChildren - 1u) / kMaxChildren;
    FML_DCHECK(gen_end + family_count <= total_node_count);

    // D here is similar to the variable in a Bresenham line algorithm where
    // we want to slowly move |family_count| steps along the minor axis as
    // we move |gen_count| steps along the major axis.
    //
    // Each inner loop increments D by family_count.
    // The inner loop executes a total of gen_count times.
    // Every time D exceeds 0 we subtract gen_count and move to a new parent.
    // All told we will increment D by family_count a total of gen_count times.
    // All told we will decrement D by gen_count a total of family_count times.
    // This leaves D back at its starting value.
    //
    // We could bias/balance where the extra children are placed by varying
    // the initial count of D from 0 to (1 - family_count), but we aren't
    // looking at this process aesthetically so we just use 0 as an initial
    // value. Using 0 provides a "greedy" allocation of the extra children.
    // Bresenham also uses double the size of the steps we use here also to
    // have better rounding of when the minor axis steps occur, but again we
    // don't care about the distribution of the extra children.
    int D = 0;

    uint32_t sibling_index = gen_start;
    uint32_t parent_index = gen_end;
    Node* parent = nullptr;
    while (sibling_index < gen_end) {
      if ((D += family_count) > 0) {
        D -= gen_count;
        FML_DCHECK(parent_index < gen_end + family_count);
        parent = &nodes_[parent_index++];
        parent->bounds = kEmpty;
        parent->child.index = sibling_index;
        parent->child.count = 0;
      }
      FML_DCHECK(parent != nullptr);
      parent->bounds = parent->bounds.Union(nodes_[sibling_index++].bounds);
      parent->child.count++;
    }
    FML_DCHECK(D == 0);
    FML_DCHECK(sibling_index == gen_end);
    FML_DCHECK(parent_index == gen_end + family_count);
    gen_start = gen_end;
    gen_count = family_count;
  }
  FML_DCHECK(gen_start + gen_count == total_node_count);
}

void DlRTree::search(const DlRect& query, std::vector<int>* results) const {
  FML_DCHECK(results != nullptr);
  if (query.IsEmpty()) {
    return;
  }
  if (nodes_.size() <= 0) {
    FML_DCHECK(leaf_count_ == 0);
    return;
  }
  const Node& root = nodes_.back();
  if (root.bounds.IntersectsWithRect(query)) {
    if (nodes_.size() == 1) {
      FML_DCHECK(leaf_count_ == 1);
      // The root node is the only node and it is a leaf node
      results->push_back(0);
    } else {
      search(root, query, results);
    }
  }
}

std::list<DlRect> DlRTree::searchAndConsolidateRects(const DlRect& query,
                                                     bool deband) const {
  // Get the indexes for the operations that intersect with the query rect.
  std::vector<int> intermediary_results;
  search(query, &intermediary_results);

  std::vector<DlIRect> rects;
  rects.reserve(intermediary_results.size());
  for (int index : intermediary_results) {
    DlIRect current_record_rect = DlIRect::RoundOut(bounds(index));
    rects.push_back(current_record_rect);
  }
  DlRegion region(rects);

  auto non_overlapping_rects = region.getRects(deband);
  std::list<DlRect> final_results;
  for (const auto& rect : non_overlapping_rects) {
    final_results.push_back(DlRect::Make(rect));
  }
  return final_results;
}

void DlRTree::search(const Node& parent,
                     const DlRect& query,
                     std::vector<int>* results) const {
  // Caller protects against empty query
  int start = parent.child.index;
  int end = start + parent.child.count;
  for (int i = start; i < end; i++) {
    const Node& node = nodes_[i];
    if (node.bounds.IntersectsWithRect(query)) {
      if (i < leaf_count_) {
        results->push_back(i);
      } else {
        search(node, query, results);
      }
    }
  }
}

const DlRegion& DlRTree::region() const {
  if (!region_) {
    std::vector<DlIRect> rects;
    rects.resize(leaf_count_);
    for (int i = 0; i < leaf_count_; i++) {
      rects[i] = DlIRect::RoundOut(nodes_[i].bounds);
    }
    region_.emplace(rects);
  }
  return *region_;
}

const DlRect& DlRTree::bounds() const {
  if (!nodes_.empty()) {
    return nodes_.back().bounds;
  } else {
    return kEmpty;
  }
}

}  // namespace flutter

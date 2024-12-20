// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/fuchsia/flutter/rtree.h"

#include <list>

#include "flutter/display_list/geometry/dl_region.h"
#include "flutter/fml/logging.h"
#include "third_party/skia/include/core/SkBBHFactory.h"
#include "third_party/skia/include/core/SkRect.h"

namespace flutter {

RTree::RTree() : bbh_{SkRTreeFactory{}()}, all_ops_count_(0) {}

void RTree::insert(const SkRect boundsArray[],
                   const SkBBoxHierarchy::Metadata metadata[],
                   int N) {
  FML_DCHECK(0 == all_ops_count_);
  bbh_->insert(boundsArray, metadata, N);
  for (int i = 0; i < N; i++) {
    if (metadata != nullptr && metadata[i].isDraw) {
      draw_op_[i] = boundsArray[i];
    }
  }
  all_ops_count_ = N;
}

void RTree::insert(const SkRect boundsArray[], int N) {
  insert(boundsArray, nullptr, N);
}

void RTree::search(const SkRect& query, std::vector<int>* results) const {
  bbh_->search(query, results);
}

std::list<SkRect> RTree::searchNonOverlappingDrawnRects(
    const SkRect& query) const {
  // Get the indexes for the operations that intersect with the query rect.
  std::vector<int> intermediary_results;
  search(query, &intermediary_results);

  std::vector<DlIRect> rects;
  for (int index : intermediary_results) {
    auto draw_op = draw_op_.find(index);
    // Ignore records that don't draw anything.
    if (draw_op == draw_op_.end()) {
      continue;
    }
    SkIRect current_record_rect;
    draw_op->second.roundOut(&current_record_rect);
    rects.push_back(ToDlIRect(current_record_rect));
  }

  DlRegion region(rects);
  auto non_overlapping_rects = region.getRects(true);
  std::list<SkRect> final_results;
  for (const auto& rect : non_overlapping_rects) {
    final_results.push_back(SkRect::Make(ToSkIRect(rect)));
  }
  return final_results;
}

size_t RTree::bytesUsed() const {
  return bbh_->bytesUsed();
}

RTreeFactory::RTreeFactory() {
  r_tree_ = sk_make_sp<RTree>();
}

sk_sp<RTree> RTreeFactory::getInstance() {
  return r_tree_;
}

sk_sp<SkBBoxHierarchy> RTreeFactory::operator()() const {
  return r_tree_;
}

}  // namespace flutter

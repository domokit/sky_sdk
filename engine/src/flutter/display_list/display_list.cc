// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <type_traits>

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_op_records.h"
#include "flutter/fml/trace_event.h"

namespace flutter {

const SaveLayerOptions SaveLayerOptions::kNoAttributes = SaveLayerOptions();
const SaveLayerOptions SaveLayerOptions::kWithAttributes =
    kNoAttributes.with_renders_with_attributes();

DisplayList::DisplayList()
    : op_count_(0),
      nested_byte_count_(0),
      nested_op_count_(0),
      total_depth_(0),
      unique_id_(0),
      can_apply_group_opacity_(true),
      is_ui_thread_safe_(true),
      modifies_transparent_black_(false),
      root_has_backdrop_filter_(false),
      root_is_unbounded_(false),
      max_root_blend_mode_(DlBlendMode::kClear) {
  FML_DCHECK(offsets_.size() == 0u);
  FML_DCHECK(storage_.size() == 0u);
}

DisplayList::DisplayList(DisplayListStorage&& storage,
                         std::vector<size_t>&& offsets,
                         uint32_t op_count,
                         size_t nested_byte_count,
                         uint32_t nested_op_count,
                         uint32_t total_depth,
                         const DlRect& bounds,
                         bool can_apply_group_opacity,
                         bool is_ui_thread_safe,
                         bool modifies_transparent_black,
                         DlBlendMode max_root_blend_mode,
                         bool root_has_backdrop_filter,
                         bool root_is_unbounded,
                         sk_sp<const DlRTree> rtree)
    : storage_(std::move(storage)),
      offsets_(std::move(offsets)),
      op_count_(op_count),
      nested_byte_count_(nested_byte_count),
      nested_op_count_(nested_op_count),
      total_depth_(total_depth),
      unique_id_(next_unique_id()),
      bounds_(bounds),
      can_apply_group_opacity_(can_apply_group_opacity),
      is_ui_thread_safe_(is_ui_thread_safe),
      modifies_transparent_black_(modifies_transparent_black),
      root_has_backdrop_filter_(root_has_backdrop_filter),
      root_is_unbounded_(root_is_unbounded),
      max_root_blend_mode_(max_root_blend_mode),
      rtree_(std::move(rtree)) {
  FML_DCHECK(storage_.capacity() == storage_.size());
}

DisplayList::~DisplayList() {
  DisposeOps(storage_, offsets_);
}

uint32_t DisplayList::next_unique_id() {
  static std::atomic<uint32_t> next_id{1};
  uint32_t id;
  do {
    id = next_id.fetch_add(+1, std::memory_order_relaxed);
  } while (id == 0);
  return id;
}

struct SaveInfo {
  SaveInfo(DlIndex previous_restore_index, bool save_was_needed)
      : previous_restore_index(previous_restore_index),
        save_was_needed(save_was_needed) {}

  DlIndex previous_restore_index;
  bool save_was_needed;
};

void DisplayList::RTreeResultsToIndexVector(
    std::vector<DlIndex>& indices,
    const std::vector<int>& rtree_results) const {
  FML_DCHECK(rtree_);
  auto cur_rect = rtree_results.begin();
  auto end_rect = rtree_results.end();
  if (cur_rect >= end_rect) {
    return;
  }
  DlIndex next_render_index = rtree_->id(*cur_rect++);
  DlIndex next_restore_index = std::numeric_limits<DlIndex>::max();
  std::vector<SaveInfo> save_infos;
  for (DlIndex index = 0u; index < offsets_.size(); index++) {
    while (index > next_render_index) {
      if (cur_rect < end_rect) {
        next_render_index = rtree_->id(*cur_rect++);
      } else {
        // Nothing left to render.
        // Nothing left to do, but match our restores from the stack.
        while (!save_infos.empty()) {
          SaveInfo& info = save_infos.back();
          // stack top boolean tells us whether the local variable
          // next_restore_index should be executed. The local variable
          // then gets reset to the value stored in the stack top
          if (info.save_was_needed) {
            FML_DCHECK(next_restore_index < offsets_.size());
            indices.push_back(next_restore_index);
          }
          next_restore_index = info.previous_restore_index;
          save_infos.pop_back();
        }
        return;
      }
    }
    const uint8_t* ptr = storage_.base() + offsets_[index];
    const DLOp* op = reinterpret_cast<const DLOp*>(ptr);
    switch (GetOpCategory(op->type)) {
      case DisplayListOpCategory::kAttribute:
        // Attributes are always needed
        indices.push_back(index);
        break;

      case DisplayListOpCategory::kTransform:
      case DisplayListOpCategory::kClip:
        if (next_render_index < next_restore_index) {
          indices.push_back(index);
        }
        break;

      case DisplayListOpCategory::kRendering:
      case DisplayListOpCategory::kSubDisplayList:
        if (index == next_render_index) {
          indices.push_back(index);
        }
        break;

      case DisplayListOpCategory::kSave:
      case DisplayListOpCategory::kSaveLayer: {
        bool needed = (next_render_index < next_restore_index);
        save_infos.emplace_back(next_restore_index, needed);
        switch (op->type) {
          case DisplayListOpType::kSave:
          case DisplayListOpType::kSaveLayer:
          case DisplayListOpType::kSaveLayerBackdrop:
            next_restore_index =
                static_cast<const SaveOpBase*>(op)->restore_index;
            break;
          default:
            FML_UNREACHABLE();
        }
        if (needed) {
          indices.push_back(index);
        }
        break;
      }

      case DisplayListOpCategory::kRestore: {
        FML_DCHECK(!save_infos.empty());
        FML_DCHECK(index == next_restore_index);
        SaveInfo& info = save_infos.back();
        next_restore_index = info.previous_restore_index;
        if (info.save_was_needed) {
          indices.push_back(index);
        }
        save_infos.pop_back();
        break;
      }

      case DisplayListOpCategory::kInvalidCategory:
        FML_UNREACHABLE();
    }
  }
}

void DisplayList::Dispatch(DlOpReceiver& receiver) const {
  const uint8_t* base = storage_.base();
  for (size_t offset : offsets_) {
    DispatchOneOp(receiver, base + offset);
  }
}

void DisplayList::Dispatch(DlOpReceiver& receiver,
                           const DlIRect& cull_rect) const {
  Dispatch(receiver, DlRect::Make(cull_rect));
}

void DisplayList::Dispatch(DlOpReceiver& receiver,
                           const DlRect& cull_rect) const {
  if (cull_rect.IsEmpty()) {
    return;
  }
  if (!has_rtree() || cull_rect.Contains(GetBounds())) {
    Dispatch(receiver);
  } else {
    auto op_indices = GetCulledIndices(cull_rect);
    const uint8_t* base = storage_.base();
    for (DlIndex index : op_indices) {
      DispatchOneOp(receiver, base + offsets_[index]);
    }
  }
}

void DisplayList::DispatchOneOp(DlOpReceiver& receiver,
                                const uint8_t* ptr) const {
  auto op = reinterpret_cast<const DLOp*>(ptr);
  switch (op->type) {
#define DL_OP_DISPATCH(name)                              \
  case DisplayListOpType::k##name:                        \
    static_cast<const name##Op*>(op)->dispatch(receiver); \
    break;

    FOR_EACH_DISPLAY_LIST_OP(DL_OP_DISPATCH)

#undef DL_OP_DISPATCH

    case DisplayListOpType::kInvalidOp:
    default:
      FML_DCHECK(false) << "Unrecognized op type: "
                        << static_cast<int>(op->type);
  }
}

void DisplayList::DisposeOps(const DisplayListStorage& storage,
                             const std::vector<size_t>& offsets) {
  const uint8_t* base = storage.base();
  if (!base) {
    return;
  }
  for (size_t offset : offsets) {
    auto op = reinterpret_cast<const DLOp*>(base + offset);
    switch (op->type) {
#define DL_OP_DISPOSE(name)                            \
  case DisplayListOpType::k##name:                     \
    if (!std::is_trivially_destructible_v<name##Op>) { \
      static_cast<const name##Op*>(op)->~name##Op();   \
    }                                                  \
    break;

      FOR_EACH_DISPLAY_LIST_OP(DL_OP_DISPOSE)

#undef DL_OP_DISPOSE

      default:
        FML_UNREACHABLE();
    }
  }
}

DisplayListOpCategory DisplayList::GetOpCategory(DlIndex index) const {
  return GetOpCategory(GetOpType(index));
}

DisplayListOpCategory DisplayList::GetOpCategory(DisplayListOpType type) {
  switch (type) {
    case DisplayListOpType::kSetAntiAlias:
    case DisplayListOpType::kSetInvertColors:
    case DisplayListOpType::kSetStrokeCap:
    case DisplayListOpType::kSetStrokeJoin:
    case DisplayListOpType::kSetStyle:
    case DisplayListOpType::kSetStrokeWidth:
    case DisplayListOpType::kSetStrokeMiter:
    case DisplayListOpType::kSetColor:
    case DisplayListOpType::kSetBlendMode:
    case DisplayListOpType::kClearColorFilter:
    case DisplayListOpType::kSetPodColorFilter:
    case DisplayListOpType::kClearColorSource:
    case DisplayListOpType::kSetPodColorSource:
    case DisplayListOpType::kSetImageColorSource:
    case DisplayListOpType::kSetRuntimeEffectColorSource:
    case DisplayListOpType::kClearImageFilter:
    case DisplayListOpType::kSetPodImageFilter:
    case DisplayListOpType::kSetSharedImageFilter:
    case DisplayListOpType::kClearMaskFilter:
    case DisplayListOpType::kSetPodMaskFilter:
      return DisplayListOpCategory::kAttribute;

    case DisplayListOpType::kSave:
      return DisplayListOpCategory::kSave;
    case DisplayListOpType::kSaveLayer:
    case DisplayListOpType::kSaveLayerBackdrop:
      return DisplayListOpCategory::kSaveLayer;
    case DisplayListOpType::kRestore:
      return DisplayListOpCategory::kRestore;

    case DisplayListOpType::kTranslate:
    case DisplayListOpType::kScale:
    case DisplayListOpType::kRotate:
    case DisplayListOpType::kSkew:
    case DisplayListOpType::kTransform2DAffine:
    case DisplayListOpType::kTransformFullPerspective:
    case DisplayListOpType::kTransformReset:
      return DisplayListOpCategory::kTransform;

    case DisplayListOpType::kClipIntersectRect:
    case DisplayListOpType::kClipIntersectOval:
    case DisplayListOpType::kClipIntersectRoundRect:
    case DisplayListOpType::kClipIntersectPath:
    case DisplayListOpType::kClipDifferenceRect:
    case DisplayListOpType::kClipDifferenceOval:
    case DisplayListOpType::kClipDifferenceRoundRect:
    case DisplayListOpType::kClipDifferencePath:
      return DisplayListOpCategory::kClip;

    case DisplayListOpType::kDrawPaint:
    case DisplayListOpType::kDrawColor:
    case DisplayListOpType::kDrawLine:
    case DisplayListOpType::kDrawDashedLine:
    case DisplayListOpType::kDrawRect:
    case DisplayListOpType::kDrawOval:
    case DisplayListOpType::kDrawCircle:
    case DisplayListOpType::kDrawRoundRect:
    case DisplayListOpType::kDrawDiffRoundRect:
    case DisplayListOpType::kDrawArc:
    case DisplayListOpType::kDrawPath:
    case DisplayListOpType::kDrawPoints:
    case DisplayListOpType::kDrawLines:
    case DisplayListOpType::kDrawPolygon:
    case DisplayListOpType::kDrawVertices:
    case DisplayListOpType::kDrawImage:
    case DisplayListOpType::kDrawImageWithAttr:
    case DisplayListOpType::kDrawImageRect:
    case DisplayListOpType::kDrawImageNine:
    case DisplayListOpType::kDrawImageNineWithAttr:
    case DisplayListOpType::kDrawAtlas:
    case DisplayListOpType::kDrawAtlasCulled:
    case DisplayListOpType::kDrawTextBlob:
    case DisplayListOpType::kDrawTextFrame:
    case DisplayListOpType::kDrawShadow:
    case DisplayListOpType::kDrawShadowTransparentOccluder:
      return DisplayListOpCategory::kRendering;

    case DisplayListOpType::kDrawDisplayList:
      return DisplayListOpCategory::kSubDisplayList;

    case DisplayListOpType::kInvalidOp:
      return DisplayListOpCategory::kInvalidCategory;
  }
}

DisplayListOpType DisplayList::GetOpType(DlIndex index) const {
  // Assert unsigned type so we can eliminate >= 0 comparison
  static_assert(std::is_unsigned_v<DlIndex>);
  if (index >= offsets_.size()) {
    return DisplayListOpType::kInvalidOp;
  }

  size_t offset = offsets_[index];
  FML_DCHECK(offset < storage_.size());
  auto ptr = storage_.base() + offset;
  auto op = reinterpret_cast<const DLOp*>(ptr);
  return op->type;
}

static void FillAllIndices(std::vector<DlIndex>& indices, DlIndex size) {
  indices.reserve(size);
  for (DlIndex i = 0u; i < size; i++) {
    indices.push_back(i);
  }
}

std::vector<DlIndex> DisplayList::GetCulledIndices(
    const DlRect& cull_rect) const {
  std::vector<DlIndex> indices;
  if (!cull_rect.IsEmpty()) {
    if (rtree_) {
      std::vector<int> rect_indices;
      rtree_->search(cull_rect, &rect_indices);
      RTreeResultsToIndexVector(indices, rect_indices);
    } else {
      FillAllIndices(indices, offsets_.size());
    }
  }
  return indices;
}

bool DisplayList::Dispatch(DlOpReceiver& receiver, DlIndex index) const {
  // Assert unsigned type so we can eliminate >= 0 comparison
  static_assert(std::is_unsigned_v<DlIndex>);
  if (index >= offsets_.size()) {
    return false;
  }

  size_t offset = offsets_[index];
  FML_DCHECK(offset < storage_.size());
  auto ptr = storage_.base() + offset;

  DispatchOneOp(receiver, ptr);

  return true;
}

static bool CompareOps(const DisplayListStorage& storageA,
                       const std::vector<size_t>& offsetsA,
                       const DisplayListStorage& storageB,
                       const std::vector<size_t>& offsetsB) {
  const uint8_t* base_a = storageA.base();
  const uint8_t* base_b = storageB.base();
  // These conditions are checked by the caller...
  FML_DCHECK(offsetsA.size() == offsetsB.size());
  FML_DCHECK(base_a != base_b);
  size_t bulk_start = 0u;
  for (size_t i = 0; i < offsetsA.size(); i++) {
    size_t offset = offsetsA[i];
    FML_DCHECK(offsetsB[i] == offset);
    auto opA = reinterpret_cast<const DLOp*>(base_a + offset);
    auto opB = reinterpret_cast<const DLOp*>(base_b + offset);
    if (opA->type != opB->type) {
      return false;
    }
    DisplayListCompare result;
    switch (opA->type) {
#define DL_OP_EQUALS(name)                              \
  case DisplayListOpType::k##name:                      \
    result = static_cast<const name##Op*>(opA)->equals( \
        static_cast<const name##Op*>(opB));             \
    break;

      FOR_EACH_DISPLAY_LIST_OP(DL_OP_EQUALS)

#undef DL_OP_EQUALS

      default:
        FML_DCHECK(false);
        return false;
    }
    switch (result) {
      case DisplayListCompare::kNotEqual:
        return false;
      case DisplayListCompare::kUseBulkCompare:
        break;
      case DisplayListCompare::kEqual:
        // Check if we have a backlog of bytes to bulk compare and then
        // reset the bulk compare pointers to the address following this op
        if (bulk_start < offset) {
          const uint8_t* bulk_start_a = base_a + bulk_start;
          const uint8_t* bulk_start_b = base_b + bulk_start;
          if (memcmp(bulk_start_a, bulk_start_b, offset - bulk_start) != 0) {
            return false;
          }
        }
        bulk_start =
            i + 1 < offsetsA.size() ? offsetsA[i + 1] : storageA.size();
        break;
    }
  }
  if (bulk_start < storageA.size()) {
    // Perform a final bulk compare if we have remaining bytes waiting
    const uint8_t* bulk_start_a = base_a + bulk_start;
    const uint8_t* bulk_start_b = base_b + bulk_start;
    if (memcmp(bulk_start_a, bulk_start_b, storageA.size() - bulk_start) != 0) {
      return false;
    }
  }
  return true;
}

bool DisplayList::Equals(const DisplayList* other) const {
  if (this == other) {
    return true;
  }
  if (offsets_.size() != other->offsets_.size() ||
      storage_.size() != other->storage_.size() ||
      op_count_ != other->op_count_) {
    return false;
  }
  if (storage_.base() == other->storage_.base()) {
    return true;
  }
  return CompareOps(storage_, offsets_, other->storage_, other->offsets_);
}

}  // namespace flutter

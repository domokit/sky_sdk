// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/backdrop_filter_layer.h"

namespace flutter {

BackdropFilterLayer::BackdropFilterLayer(
    const std::shared_ptr<DlImageFilter>& filter,
    DlBlendMode blend_mode,
    std::optional<int64_t> backdrop_id)
    : filter_(filter), blend_mode_(blend_mode), backdrop_id_(backdrop_id) {}

void BackdropFilterLayer::Diff(DiffContext* context, const Layer* old_layer) {
  DiffContext::AutoSubtreeRestore subtree(context);
  auto* prev = static_cast<const BackdropFilterLayer*>(old_layer);
  if (!context->IsSubtreeDirty()) {
    FML_DCHECK(prev);
    if (NotEquals(filter_, prev->filter_)) {
      context->MarkSubtreeDirty(context->GetOldLayerPaintRegion(old_layer));
    }
  }

  // Backdrop filter paints everywhere in cull rect
  auto paint_bounds = context->GetCullRect();
  context->AddLayerBounds(paint_bounds);

  if (filter_) {
    paint_bounds = context->MapRect(paint_bounds);
    auto filter_target_bounds = DlIRect::RoundOut(paint_bounds);
    DlIRect filter_input_bounds;  // in screen coordinates
    filter_->get_input_device_bounds(filter_target_bounds, context->GetMatrix(),
                                     filter_input_bounds);
    context->AddReadbackRegion(filter_target_bounds, filter_input_bounds);
  }

  DiffChildren(context, prev);

  context->SetLayerPaintRegion(this, context->CurrentSubtreeRegion());
}

void BackdropFilterLayer::Preroll(PrerollContext* context) {
  Layer::AutoPrerollSaveLayerState save =
      Layer::AutoPrerollSaveLayerState::Create(context, true, bool{filter_});
  if (filter_ && context->view_embedder != nullptr) {
    context->view_embedder->PushFilterToVisitedPlatformViews(
        filter_, ToSkRect(context->state_stack.device_cull_rect()));
  }
  DlRect child_paint_bounds;
  PrerollChildren(context, &child_paint_bounds);
  child_paint_bounds =
      child_paint_bounds.Union(context->state_stack.local_cull_rect());
  set_paint_bounds(child_paint_bounds);
  context->renderable_state_flags = kSaveLayerRenderFlags;
}

void BackdropFilterLayer::Paint(PaintContext& context) const {
  FML_DCHECK(needs_painting(context));

  auto mutator = context.state_stack.save();
  mutator.applyBackdropFilter(paint_bounds(), filter_, blend_mode_,
                              backdrop_id_);

  PaintChildren(context);
}

}  // namespace flutter

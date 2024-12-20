// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/opacity_layer.h"

#include "flutter/flow/layers/cacheable_layer.h"
#include "flutter/flow/raster_cache_util.h"

namespace flutter {

// the opacity_layer couldn't cache itself, so the cache_threshold is the
// max_int
OpacityLayer::OpacityLayer(uint8_t alpha, const DlPoint& offset)
    : CacheableContainerLayer(std::numeric_limits<int>::max(), true),
      alpha_(alpha),
      offset_(offset) {}

void OpacityLayer::Diff(DiffContext* context, const Layer* old_layer) {
  DiffContext::AutoSubtreeRestore subtree(context);
  auto* prev = static_cast<const OpacityLayer*>(old_layer);
  if (!context->IsSubtreeDirty()) {
    FML_DCHECK(prev);
    if (alpha_ != prev->alpha_ || offset_ != prev->offset_) {
      context->MarkSubtreeDirty(context->GetOldLayerPaintRegion(old_layer));
    }
  }
  context->PushTransform(DlMatrix::MakeTranslation(offset_));
  if (context->has_raster_cache()) {
    context->WillPaintWithIntegralTransform();
  }
  DiffChildren(context, prev);
  context->SetLayerPaintRegion(this, context->CurrentSubtreeRegion());
}

void OpacityLayer::Preroll(PrerollContext* context) {
  auto mutator = context->state_stack.save();
  mutator.translate(offset_);
  mutator.applyOpacity(DlRect(), opacity());

#if !SLIMPELLER
  AutoCache auto_cache = AutoCache(layer_raster_cache_item_.get(), context,
                                   context->state_stack.matrix());
#endif  //  !SLIMPELLER

  Layer::AutoPrerollSaveLayerState save =
      Layer::AutoPrerollSaveLayerState::Create(context);

  ContainerLayer::Preroll(context);
  // We store the inheritance ability of our children for |Paint|
  set_children_can_accept_opacity((context->renderable_state_flags &
                                   LayerStateStack::kCallerCanApplyOpacity) !=
                                  0);

  // Now we let our parent layers know that we, too, can inherit opacity
  // regardless of what our children are capable of
  context->renderable_state_flags |= LayerStateStack::kCallerCanApplyOpacity;

  set_paint_bounds(paint_bounds().Shift(offset_));

#if !SLIMPELLER
  if (children_can_accept_opacity()) {
    // For opacity layer, we can use raster_cache children only when the
    // children can't accept opacity so if the children_can_accept_opacity we
    // should tell the AutoCache object don't do raster_cache.
    auto_cache.ShouldNotBeCached();
  }
#endif  //  !SLIMPELLER
}

void OpacityLayer::Paint(PaintContext& context) const {
  FML_DCHECK(needs_painting(context));

  auto mutator = context.state_stack.save();
  mutator.translate(offset_.x, offset_.y);

#if !SLIMPELLER
  if (context.raster_cache) {
    mutator.integralTransform();
  }
#endif  //  !SLIMPELLER

  mutator.applyOpacity(child_paint_bounds(), opacity());

#if !SLIMPELLER
  if (!children_can_accept_opacity()) {
    DlPaint paint;
    if (layer_raster_cache_item_->Draw(context,
                                       context.state_stack.fill(paint))) {
      return;
    }
  }
#endif  //  !SLIMPELLER

  PaintChildren(context);
}

}  // namespace flutter

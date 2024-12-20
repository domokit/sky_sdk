// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_CLIP_SHAPE_LAYER_H_
#define FLUTTER_FLOW_LAYERS_CLIP_SHAPE_LAYER_H_

#include "flutter/flow/layers/cacheable_layer.h"
#include "flutter/flow/layers/container_layer.h"
#include "flutter/flow/paint_utils.h"

namespace flutter {

template <class T>
class ClipShapeLayer : public CacheableContainerLayer {
 public:
  using ClipShape = T;
  ClipShapeLayer(const ClipShape& clip_shape, Clip clip_behavior)
      : CacheableContainerLayer(),
        clip_shape_(clip_shape),
        clip_behavior_(clip_behavior) {
    FML_DCHECK(clip_behavior != Clip::kNone);
  }

  void Diff(DiffContext* context, const Layer* old_layer) override {
    DiffContext::AutoSubtreeRestore subtree(context);
    auto* prev = static_cast<const ClipShapeLayer<ClipShape>*>(old_layer);
    if (!context->IsSubtreeDirty()) {
      FML_DCHECK(prev);
      if (clip_behavior_ != prev->clip_behavior_ ||
          clip_shape_ != prev->clip_shape_) {
        context->MarkSubtreeDirty(context->GetOldLayerPaintRegion(old_layer));
      }
    }
    if (UsesSaveLayer() && context->has_raster_cache()) {
      context->WillPaintWithIntegralTransform();
    }
    if (context->PushCullRect(clip_shape_bounds())) {
      DiffChildren(context, prev);
    }
    context->SetLayerPaintRegion(this, context->CurrentSubtreeRegion());
  }

  void Preroll(PrerollContext* context) override {
    bool uses_save_layer = UsesSaveLayer();

#if !SLIMPELLER
    // We can use the raster_cache for children only when the use_save_layer is
    // true so if use_save_layer is false we pass the layer_raster_item is
    // nullptr which mean we don't do raster cache logic.
    AutoCache cache =
        AutoCache(uses_save_layer ? layer_raster_cache_item_.get() : nullptr,
                  context, context->state_stack.matrix());
#endif  //  !SLIMPELLER

    Layer::AutoPrerollSaveLayerState save =
        Layer::AutoPrerollSaveLayerState::Create(context, UsesSaveLayer());

    auto mutator = context->state_stack.save();
    ApplyClip(mutator);

    DlRect child_paint_bounds;
    PrerollChildren(context, &child_paint_bounds);
    set_paint_bounds(
        child_paint_bounds.IntersectionOrEmpty(clip_shape_bounds()));

    // If we use a SaveLayer then we can accept opacity on behalf
    // of our children and apply it in the saveLayer.
    if (uses_save_layer) {
      context->renderable_state_flags = kSaveLayerRenderFlags;
    }
  }

  void Paint(PaintContext& context) const override {
    FML_DCHECK(needs_painting(context));

    auto mutator = context.state_stack.save();
    ApplyClip(mutator);

    if (!UsesSaveLayer()) {
      PaintChildren(context);
      return;
    }

#if !SLIMPELLER
    if (context.raster_cache) {
      mutator.integralTransform();
      auto restore_apply = context.state_stack.applyState(
          paint_bounds(), LayerStateStack::kCallerCanApplyOpacity);

      DlPaint paint;
      if (layer_raster_cache_item_->Draw(context,
                                         context.state_stack.fill(paint))) {
        return;
      }
    }
#endif  //  !SLIMPELLER

    mutator.saveLayer(paint_bounds());
    PaintChildren(context);
  }

  bool UsesSaveLayer() const {
    return clip_behavior_ == Clip::kAntiAliasWithSaveLayer;
  }

 protected:
  virtual const DlRect clip_shape_bounds() const = 0;
  virtual void ApplyClip(LayerStateStack::MutatorContext& mutator) const = 0;
  virtual ~ClipShapeLayer() = default;

  const ClipShape& clip_shape() const { return clip_shape_; }
  Clip clip_behavior() const { return clip_behavior_; }

 private:
  const ClipShape clip_shape_;
  Clip clip_behavior_;

  FML_DISALLOW_COPY_AND_ASSIGN(ClipShapeLayer);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_LAYERS_CLIP_SHAPE_LAYER_H_

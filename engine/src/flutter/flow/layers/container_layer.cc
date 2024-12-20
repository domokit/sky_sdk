// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/container_layer.h"

#include <optional>

namespace flutter {

ContainerLayer::ContainerLayer() {}

void ContainerLayer::Diff(DiffContext* context, const Layer* old_layer) {
  auto old_container = static_cast<const ContainerLayer*>(old_layer);
  DiffContext::AutoSubtreeRestore subtree(context);
  DiffChildren(context, old_container);
  context->SetLayerPaintRegion(this, context->CurrentSubtreeRegion());
}

void ContainerLayer::PreservePaintRegion(DiffContext* context) {
  Layer::PreservePaintRegion(context);
  for (auto& layer : layers_) {
    layer->PreservePaintRegion(context);
  }
}

void ContainerLayer::DiffChildren(DiffContext* context,
                                  const ContainerLayer* old_layer) {
  if (context->IsSubtreeDirty()) {
    for (auto& layer : layers_) {
      layer->Diff(context, nullptr);
    }
    return;
  }
  FML_DCHECK(old_layer);

  const auto& prev_layers = old_layer->layers_;

  // first mismatched element
  int new_children_top = 0;
  int old_children_top = 0;

  // last mismatched element
  int new_children_bottom = layers_.size() - 1;
  int old_children_bottom = prev_layers.size() - 1;

  while ((old_children_top <= old_children_bottom) &&
         (new_children_top <= new_children_bottom)) {
    if (!layers_[new_children_top]->IsReplacing(
            context, prev_layers[old_children_top].get())) {
      break;
    }
    ++new_children_top;
    ++old_children_top;
  }

  while ((old_children_top <= old_children_bottom) &&
         (new_children_top <= new_children_bottom)) {
    if (!layers_[new_children_bottom]->IsReplacing(
            context, prev_layers[old_children_bottom].get())) {
      break;
    }
    --new_children_bottom;
    --old_children_bottom;
  }

  // old layers that don't match
  for (int i = old_children_top; i <= old_children_bottom; ++i) {
    auto layer = prev_layers[i];
    context->AddDamage(context->GetOldLayerPaintRegion(layer.get()));
  }

  for (int i = 0; i < static_cast<int>(layers_.size()); ++i) {
    if (i < new_children_top || i > new_children_bottom) {
      int i_prev =
          i < new_children_top ? i : prev_layers.size() - (layers_.size() - i);
      auto layer = layers_[i];
      auto prev_layer = prev_layers[i_prev];
      auto paint_region = context->GetOldLayerPaintRegion(prev_layer.get());
      if (layer == prev_layer && !paint_region.has_readback() &&
          !paint_region.has_texture()) {
        // for retained layers, stop processing the subtree and add existing
        // region; We know current subtree is not dirty (every ancestor up to
        // here matches) so the retained subtree will render identically to
        // previous frame; We can only do this if there is no readback in the
        // subtree. Layers that do readback must be able to register readback
        // inside Diff
        context->AddExistingPaintRegion(paint_region);

        // While we don't need to diff retained layers, we still need to
        // associate their paint region with current layer tree so that we can
        // retrieve it in next frame diff
        layer->PreservePaintRegion(context);
      } else {
        layer->Diff(context, prev_layer.get());
      }
    } else {
      DiffContext::AutoSubtreeRestore subtree(context);
      context->MarkSubtreeDirty();
      auto layer = layers_[i];
      layer->Diff(context, nullptr);
    }
  }
}

void ContainerLayer::Add(std::shared_ptr<Layer> layer) {
  layers_.emplace_back(std::move(layer));
}

void ContainerLayer::Preroll(PrerollContext* context) {
  DlRect child_paint_bounds;
  PrerollChildren(context, &child_paint_bounds);
  set_paint_bounds(child_paint_bounds);
}

void ContainerLayer::Paint(PaintContext& context) const {
  FML_DCHECK(needs_painting(context));

  PaintChildren(context);
}

void ContainerLayer::PrerollChildren(PrerollContext* context,
                                     DlRect* child_paint_bounds) {
  // Platform views have no children, so context->has_platform_view should
  // always be false.
  FML_DCHECK(!context->has_platform_view);
  FML_DCHECK(!context->has_texture_layer);

  bool child_has_platform_view = false;
  bool child_has_texture_layer = false;
  bool all_renderable_state_flags = LayerStateStack::kCallerCanApplyAnything;

  for (auto& layer : layers_) {
    // Reset context->has_platform_view and context->has_texture_layer to false
    // so that layers aren't treated as if they have a platform view or texture
    // layer based on one being previously found in a sibling tree.
    context->has_platform_view = false;
    context->has_texture_layer = false;

    // Initialize the renderable state flags to false to force the layer to
    // opt-in to applying state attributes during its |Preroll|
    context->renderable_state_flags = 0;

    layer->Preroll(context);

    all_renderable_state_flags &= context->renderable_state_flags;
    if (child_paint_bounds->IntersectsWithRect(layer->paint_bounds())) {
      // This will allow inheritance by a linear sequence of non-overlapping
      // children, but will fail with a grid or other arbitrary 2D layout.
      // See https://github.com/flutter/flutter/issues/93899
      all_renderable_state_flags = 0;
    }
    *child_paint_bounds = child_paint_bounds->Union(layer->paint_bounds());

    child_has_platform_view =
        child_has_platform_view || context->has_platform_view;
    child_has_texture_layer =
        child_has_texture_layer || context->has_texture_layer;
  }

  context->has_platform_view = child_has_platform_view;
  context->has_texture_layer = child_has_texture_layer;
  context->renderable_state_flags = all_renderable_state_flags;
  set_subtree_has_platform_view(child_has_platform_view);
  set_children_renderable_state_flags(all_renderable_state_flags);
  set_child_paint_bounds(*child_paint_bounds);
}

void ContainerLayer::PaintChildren(PaintContext& context) const {
  // We can no longer call FML_DCHECK here on the needs_painting(context)
  // condition as that test is only valid for the PaintContext that
  // is initially handed to a layer's Paint() method. By the time the
  // layer calls PaintChildren(), though, it may have modified the
  // PaintContext so the test doesn't work in this "context".

  // Apply any outstanding state that the children cannot individually
  // and collectively handle.
  auto restore = context.state_stack.applyState(
      child_paint_bounds(), children_renderable_state_flags());

  // Intentionally not tracing here as there should be no self-time
  // and the trace event on this common function has a small overhead.
  for (auto& layer : layers_) {
    if (layer->needs_painting(context)) {
      layer->Paint(context);
    }
  }
}

}  // namespace flutter

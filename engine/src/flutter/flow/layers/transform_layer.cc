// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/transform_layer.h"

#include <optional>

namespace flutter {

TransformLayer::TransformLayer(const DlMatrix& transform)
    : transform_(transform) {
  FML_DCHECK(transform_.IsFinite());
  if (!transform_.IsFinite()) {
    FML_LOG(ERROR) << "TransformLayer is constructed with an invalid matrix.";
    transform_ = {};
  }
}

void TransformLayer::Diff(DiffContext* context, const Layer* old_layer) {
  DiffContext::AutoSubtreeRestore subtree(context);
  auto* prev = static_cast<const TransformLayer*>(old_layer);
  if (!context->IsSubtreeDirty()) {
    FML_DCHECK(prev);
    if (transform_ != prev->transform_) {
      context->MarkSubtreeDirty(context->GetOldLayerPaintRegion(old_layer));
    }
  }
  context->PushTransform(transform_);
  DiffChildren(context, prev);
  context->SetLayerPaintRegion(this, context->CurrentSubtreeRegion());
}

void TransformLayer::Preroll(PrerollContext* context) {
  auto mutator = context->state_stack.save();
  mutator.transform(transform_);

  DlRect child_paint_bounds;
  PrerollChildren(context, &child_paint_bounds);

  child_paint_bounds = child_paint_bounds.TransformAndClipBounds(transform_);
  set_paint_bounds(child_paint_bounds);
}

void TransformLayer::Paint(PaintContext& context) const {
  FML_DCHECK(needs_painting(context));

  auto mutator = context.state_stack.save();
  mutator.transform(transform_);

  PaintChildren(context);
}

}  // namespace flutter

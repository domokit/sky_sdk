// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/clip_rect_layer.h"

namespace flutter {

ClipRectLayer::ClipRectLayer(const DlRect& clip_rect, Clip clip_behavior)
    : ClipShapeLayer(clip_rect, clip_behavior) {}

const DlRect ClipRectLayer::clip_shape_bounds() const {
  return clip_shape();
}

void ClipRectLayer::ApplyClip(LayerStateStack::MutatorContext& mutator) const {
  mutator.clipRect(clip_shape(), clip_behavior() != Clip::kHardEdge);
}

}  // namespace flutter

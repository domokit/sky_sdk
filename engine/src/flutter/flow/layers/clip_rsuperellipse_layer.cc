// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/layers/clip_rsuperellipse_layer.h"

namespace flutter {

ClipRSuperellipseLayer::ClipRSuperellipseLayer(
    const DlRoundSuperellipse& clip_rsuperellipse,
    Clip clip_behavior)
    : ClipShapeLayer(clip_rsuperellipse, clip_behavior) {}

const DlRect ClipRSuperellipseLayer::clip_shape_bounds() const {
  return clip_shape().GetBounds();
}

void ClipRSuperellipseLayer::ApplyClip(
    LayerStateStack::MutatorContext& mutator) const {
  mutator.clipRSuperellipse(clip_shape(), clip_behavior() != Clip::kHardEdge);
}

}  // namespace flutter

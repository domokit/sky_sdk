// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_CLIP_RRECT_LAYER_H_
#define FLUTTER_FLOW_LAYERS_CLIP_RRECT_LAYER_H_

#include "flutter/flow/layers/clip_shape_layer.h"

namespace flutter {

class ClipRRectLayer : public ClipShapeLayer<DlRoundRect> {
 public:
  ClipRRectLayer(const DlRoundRect& clip_rrect, Clip clip_behavior);

 protected:
  const DlRect clip_shape_bounds() const override;

  void ApplyClip(LayerStateStack::MutatorContext& mutator) const override;

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(ClipRRectLayer);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_LAYERS_CLIP_RRECT_LAYER_H_

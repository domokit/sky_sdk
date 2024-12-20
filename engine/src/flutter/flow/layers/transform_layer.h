// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_TRANSFORM_LAYER_H_
#define FLUTTER_FLOW_LAYERS_TRANSFORM_LAYER_H_

#include "flutter/flow/layers/container_layer.h"

namespace flutter {

class TransformLayer : public ContainerLayer {
 public:
  explicit TransformLayer(const DlMatrix& transform);

  void Diff(DiffContext* context, const Layer* old_layer) override;

  void Preroll(PrerollContext* context) override;

  void Paint(PaintContext& context) const override;

 private:
  DlMatrix transform_;

  FML_DISALLOW_COPY_AND_ASSIGN(TransformLayer);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_LAYERS_TRANSFORM_LAYER_H_

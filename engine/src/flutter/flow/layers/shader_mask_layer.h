// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_SHADER_MASK_LAYER_H_
#define FLUTTER_FLOW_LAYERS_SHADER_MASK_LAYER_H_

#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/flow/layers/cacheable_layer.h"

namespace flutter {

class ShaderMaskLayer : public CacheableContainerLayer {
 public:
  ShaderMaskLayer(std::shared_ptr<DlColorSource> color_source,
                  const DlRect& mask_rect,
                  DlBlendMode blend_mode);

  void Diff(DiffContext* context, const Layer* old_layer) override;

  void Preroll(PrerollContext* context) override;

  void Paint(PaintContext& context) const override;

 private:
  std::shared_ptr<DlColorSource> color_source_;
  DlRect mask_rect_;
  DlBlendMode blend_mode_;

  FML_DISALLOW_COPY_AND_ASSIGN(ShaderMaskLayer);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_LAYERS_SHADER_MASK_LAYER_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_TEXTURE_LAYER_H_
#define FLUTTER_FLOW_LAYERS_TEXTURE_LAYER_H_

#include "flutter/flow/layers/layer.h"

namespace flutter {

class TextureLayer : public Layer {
 public:
  TextureLayer(const DlPoint& offset,
               const DlSize& size,
               int64_t texture_id,
               bool freeze,
               DlImageSampling sampling);

  bool IsReplacing(DiffContext* context, const Layer* layer) const override {
    return layer->as_texture_layer() != nullptr;
  }

  void Diff(DiffContext* context, const Layer* old_layer) override;

  const TextureLayer* as_texture_layer() const override { return this; }

  void Preroll(PrerollContext* context) override;
  void Paint(PaintContext& context) const override;

 private:
  DlPoint offset_;
  DlSize size_;
  int64_t texture_id_;
  bool freeze_;
  DlImageSampling sampling_;

  FML_DISALLOW_COPY_AND_ASSIGN(TextureLayer);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_LAYERS_TEXTURE_LAYER_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_COLOR_FILTER_LAYER_H_
#define FLUTTER_FLOW_LAYERS_COLOR_FILTER_LAYER_H_

#include "flutter/display_list/effects/dl_color_filter.h"
#include "flutter/flow/layers/cacheable_layer.h"
#include "flutter/flow/layers/layer.h"

namespace flutter {

class ColorFilterLayer : public CacheableContainerLayer {
 public:
  explicit ColorFilterLayer(std::shared_ptr<const DlColorFilter> filter);

  void Diff(DiffContext* context, const Layer* old_layer) override;

  void Preroll(PrerollContext* context) override;

  void Paint(PaintContext& context) const override;

 private:
  std::shared_ptr<const DlColorFilter> filter_;

  FML_DISALLOW_COPY_AND_ASSIGN(ColorFilterLayer);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_LAYERS_COLOR_FILTER_LAYER_H_

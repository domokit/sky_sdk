// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FLOW_LAYERS_PLATFORM_VIEW_LAYER_H_
#define FLUTTER_FLOW_LAYERS_PLATFORM_VIEW_LAYER_H_

#include "flutter/display_list/skia/dl_sk_canvas.h"
#include "flutter/flow/layers/layer.h"
#include "third_party/skia/include/core/SkPoint.h"
#include "third_party/skia/include/core/SkSize.h"

namespace flutter {

class PlatformViewLayer : public Layer {
 public:
  PlatformViewLayer(const DlPoint& offset, const DlSize& size, int64_t view_id);

  void Preroll(PrerollContext* context) override;
  void Paint(PaintContext& context) const override;

 private:
  DlPoint offset_;
  DlSize size_;
  int64_t view_id_;

  FML_DISALLOW_COPY_AND_ASSIGN(PlatformViewLayer);
};

}  // namespace flutter

#endif  // FLUTTER_FLOW_LAYERS_PLATFORM_VIEW_LAYER_H_

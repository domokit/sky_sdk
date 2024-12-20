// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_LAYERS_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_LAYERS_H_

#include <memory>
#include <vector>

#include "flutter/flow/embedded_views.h"
#include "flutter/fml/macros.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "third_party/skia/include/core/SkMatrix.h"
#include "third_party/skia/include/core/SkSize.h"

namespace flutter {

class EmbedderLayers {
 public:
  EmbedderLayers(SkISize frame_size,
                 double device_pixel_ratio,
                 SkMatrix root_surface_transformation,
                 uint64_t presentation_time);

  ~EmbedderLayers();

  void PushBackingStoreLayer(const FlutterBackingStore* store,
                             const std::vector<DlIRect>& drawn_region);

  void PushPlatformViewLayer(FlutterPlatformViewIdentifier identifier,
                             const EmbeddedViewParams& params);

  using PresentCallback =
      std::function<bool(FlutterViewId view_id,
                         const std::vector<const FlutterLayer*>& layers)>;
  void InvokePresentCallback(FlutterViewId view_id,
                             const PresentCallback& callback) const;

 private:
  const SkISize frame_size_;
  const double device_pixel_ratio_;
  const SkMatrix root_surface_transformation_;
  std::vector<std::unique_ptr<FlutterPlatformView>> platform_views_referenced_;
  std::vector<std::unique_ptr<FlutterPlatformViewMutation>>
      mutations_referenced_;
  std::vector<std::unique_ptr<std::vector<const FlutterPlatformViewMutation*>>>
      mutations_arrays_referenced_;
  std::vector<std::unique_ptr<FlutterBackingStorePresentInfo>>
      present_info_referenced_;
  std::vector<std::unique_ptr<FlutterRegion>> regions_referenced_;
  std::vector<std::unique_ptr<std::vector<FlutterRect>>> rects_referenced_;
  std::vector<FlutterLayer> presented_layers_;
  uint64_t presentation_time_;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderLayers);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_EMBEDDER_LAYERS_H_

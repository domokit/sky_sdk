// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if !SLIMPELLER

#include "flutter/flow/raster_cache_key.h"
#include <optional>
#include "flutter/flow/layers/container_layer.h"
#include "flutter/flow/layers/display_list_layer.h"
#include "flutter/flow/layers/layer.h"

namespace flutter {

std::optional<std::vector<RasterCacheKeyID>> RasterCacheKeyID::LayerChildrenIds(
    const Layer* layer) {
  FML_DCHECK(layer->as_container_layer());
  auto& children_layers = layer->as_container_layer()->layers();
  auto children_count = children_layers.size();
  if (children_count == 0) {
    return std::nullopt;
  }
  std::vector<RasterCacheKeyID> ids;
  std::transform(
      children_layers.begin(), children_layers.end(), std::back_inserter(ids),
      [](auto& layer) -> RasterCacheKeyID { return layer->caching_key_id(); });
  return ids;
}

}  // namespace flutter

#endif  //  !SLIMPELLER

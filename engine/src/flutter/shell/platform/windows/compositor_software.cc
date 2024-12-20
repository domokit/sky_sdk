// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/compositor_software.h"

#include "flutter/shell/platform/windows/flutter_windows_engine.h"
#include "flutter/shell/platform/windows/flutter_windows_view.h"

namespace flutter {

constexpr int kOpaqueBlack = 0xff000000;

namespace {

/// Calculate the minimum and maximum x and y coordinates to enclose all layers.
FlutterRect CalculateBounds(const FlutterLayer** layers, size_t layers_count) {
  double x_min = HUGE_VAL;
  double x_max = -HUGE_VAL;
  double y_min = HUGE_VAL;
  double y_max = -HUGE_VAL;
  for (const FlutterLayer** layer = layers; layer < layers + layers_count;
       layer++) {
    const FlutterPoint& offset = (*layer)->offset;
    const FlutterSize& size = (*layer)->size;
    x_min = std::min(x_min, offset.x);
    y_min = std::min(y_min, offset.y);
    x_max = std::max(x_max, offset.x + size.width);
    y_max = std::max(y_max, offset.y + size.height);
  }
  return FlutterRect{x_min, y_min, x_max, y_max};
}

/// Blend layer in-place onto allocation, which already holds the previous
/// results of composition.
void BlendLayer(std::vector<uint32_t>& allocation,
                const FlutterLayer& layer,
                int x_min,
                int y_min,
                int width,
                int height) {
  FML_DCHECK(layer.type == kFlutterLayerContentTypeBackingStore);
  auto& backing_store = *layer.backing_store;
  FML_DCHECK(backing_store.type == kFlutterBackingStoreTypeSoftware);
  auto src_data =
      static_cast<const uint32_t*>(backing_store.software.allocation);
  const FlutterPoint& offset = layer.offset;
  const FlutterSize& size = layer.size;

  // Bounds for iteration to prevent out-of-bounds destination coordinates.
  int y_src_min = std::max(0., y_min - offset.y);
  int y_src_max = std::min(size.height, height + y_min - offset.y);
  int x_src_min = std::max(0., x_min - offset.x);
  int x_src_max = std::min(size.width, width + x_min - offset.x);
  for (int y_src = y_src_min; y_src < y_src_max; y_src++) {
    int y_dst = y_src + offset.y - y_min;
    for (int x_src = x_src_min; x_src < x_src_max; x_src++) {
      int x_dst = x_src + offset.x + x_min;
      size_t i_src = y_src * size.width + x_src;
      size_t i_dst = y_dst * width + x_dst;
      uint32_t src = src_data[i_src];
      uint32_t dst = allocation[i_dst];

      int r_src = (src >> 0) & 0xff;
      int g_src = (src >> 8) & 0xff;
      int b_src = (src >> 16) & 0xff;
      int a_src = (src >> 24) & 0xff;

      int r_dst = (dst >> 0) & 0xff;
      int g_dst = (dst >> 8) & 0xff;
      int b_dst = (dst >> 16) & 0xff;

      int r = (r_dst * 255 + (r_src - r_dst) * a_src) / 255;
      int g = (g_dst * 255 + (g_src - g_dst) * a_src) / 255;
      int b = (b_dst * 255 + (b_src - b_dst) * a_src) / 255;

      allocation[i_dst] = (r << 0) | (g << 8) | (b << 16) | (0xff << 24);
    }
  }
}

}  // namespace

CompositorSoftware::CompositorSoftware() {}

bool CompositorSoftware::CreateBackingStore(
    const FlutterBackingStoreConfig& config,
    FlutterBackingStore* result) {
  size_t size = config.size.width * config.size.height * 4;
  void* allocation = std::calloc(size, sizeof(uint8_t));
  if (!allocation) {
    return false;
  }

  result->type = kFlutterBackingStoreTypeSoftware;
  result->software.allocation = allocation;
  result->software.height = config.size.height;
  result->software.row_bytes = config.size.width * 4;
  result->software.user_data = nullptr;
  result->software.destruction_callback = [](void* user_data) {
    // Backing store destroyed in `CompositorSoftware::CollectBackingStore`, set
    // on FlutterCompositor.collect_backing_store_callback during engine start.
  };
  return true;
}

bool CompositorSoftware::CollectBackingStore(const FlutterBackingStore* store) {
  std::free(const_cast<void*>(store->software.allocation));
  return true;
}

bool CompositorSoftware::Present(FlutterWindowsView* view,
                                 const FlutterLayer** layers,
                                 size_t layers_count) {
  FML_DCHECK(view != nullptr);

  // Clear the view if there are no layers to present.
  if (layers_count == 0) {
    return view->ClearSoftwareBitmap();
  }

  // Bypass composition logic if there is only one layer.
  if (layers_count == 1) {
    const FlutterLayer* layer = layers[0];
    FML_DCHECK(layer != nullptr);
    if (layer->type == kFlutterLayerContentTypeBackingStore &&
        layer->offset.x == 0 && layer->offset.y == 0) {
      auto& backing_store = *layer->backing_store;
      FML_DCHECK(backing_store.type == kFlutterBackingStoreTypeSoftware);
      auto& software = backing_store.software;
      return view->PresentSoftwareBitmap(software.allocation,
                                         software.row_bytes, software.height);
    }
  }

  // Composite many layers.
  FlutterRect bounds = CalculateBounds(layers, layers_count);
  // Truncate from double to integer to represent whole pixels.
  int x_min = static_cast<int>(bounds.left);
  int x_max = static_cast<int>(bounds.right);
  int y_min = static_cast<int>(bounds.top);
  int y_max = static_cast<int>(bounds.bottom);

  int width = x_max - x_min;
  int height = y_max - y_min;
  std::vector<uint32_t> allocation(width * height, kOpaqueBlack);

  for (const FlutterLayer** layer = layers; layer < layers + layers_count;
       layer++) {
    // TODO(schectman): handle platform view type layers.
    // https://github.com/flutter/flutter/issues/143375
    if ((*layer)->type == kFlutterLayerContentTypeBackingStore) {
      BlendLayer(allocation, **layer, x_min, y_min, width, height);
    } else {
      FML_UNREACHABLE();
      return false;
    }
  }

  return view->PresentSoftwareBitmap(static_cast<void*>(allocation.data()),
                                     width * sizeof(uint32_t), height);
}

}  // namespace flutter

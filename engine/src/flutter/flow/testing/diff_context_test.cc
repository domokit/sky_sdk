// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "diff_context_test.h"

#include <utility>
#include "flutter/display_list/dl_builder.h"

namespace flutter {
namespace testing {

DiffContextTest::DiffContextTest() {}

Damage DiffContextTest::DiffLayerTree(MockLayerTree& layer_tree,
                                      const MockLayerTree& old_layer_tree,
                                      const DlIRect& additional_damage,
                                      int horizontal_clip_alignment,
                                      int vertical_clip_alignment,
                                      bool use_raster_cache,
                                      bool impeller_enabled) {
  FML_CHECK(layer_tree.size() == old_layer_tree.size());

  DiffContext dc(layer_tree.size(), layer_tree.paint_region_map(),
                 old_layer_tree.paint_region_map(), use_raster_cache,
                 impeller_enabled);
  dc.PushCullRect(DlRect::MakeSize(layer_tree.size()));
  layer_tree.root()->Diff(&dc, old_layer_tree.root());
  return dc.ComputeDamage(additional_damage, horizontal_clip_alignment,
                          vertical_clip_alignment);
}

sk_sp<DisplayList> DiffContextTest::CreateDisplayList(const DlRect& bounds,
                                                      DlColor color) {
  DisplayListBuilder builder;
  builder.DrawRect(bounds, DlPaint().setColor(color));
  return builder.Build();
}

std::shared_ptr<DisplayListLayer> DiffContextTest::CreateDisplayListLayer(
    const sk_sp<DisplayList>& display_list,
    const DlPoint& offset) {
  return std::make_shared<DisplayListLayer>(offset, display_list, false, false);
}

std::shared_ptr<ContainerLayer> DiffContextTest::CreateContainerLayer(
    std::initializer_list<std::shared_ptr<Layer>> layers) {
  auto res = std::make_shared<ContainerLayer>();
  for (const auto& l : layers) {
    res->Add(l);
  }
  return res;
}

std::shared_ptr<OpacityLayer> DiffContextTest::CreateOpacityLater(
    std::initializer_list<std::shared_ptr<Layer>> layers,
    uint8_t alpha,
    const DlPoint& offset) {
  auto res = std::make_shared<OpacityLayer>(alpha, offset);
  for (const auto& l : layers) {
    res->Add(l);
  }
  return res;
}

}  // namespace testing
}  // namespace flutter

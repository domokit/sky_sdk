// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/testing/mock_layer.h"

#include <utility>

#include "flutter/flow/layers/container_layer.h"
#include "flutter/flow/layers/layer.h"
#include "flutter/flow/testing/layer_test.h"
#include "flutter/flow/testing/mock_raster_cache.h"

namespace flutter {
namespace testing {

MockLayer::MockLayer(const DlPath& path, DlPaint paint)
    : fake_paint_path_(path), fake_paint_(std::move(paint)) {}

bool MockLayer::IsReplacing(DiffContext* context, const Layer* layer) const {
  // Similar to PictureLayer, only return true for identical mock layers;
  // That way ContainerLayer::DiffChildren can properly detect mock layer
  // insertion
  auto mock_layer = layer->as_mock_layer();
  return mock_layer && mock_layer->fake_paint_ == fake_paint_ &&
         mock_layer->fake_paint_path_ == fake_paint_path_;
}

void MockLayer::Diff(DiffContext* context, const Layer* old_layer) {
  DiffContext::AutoSubtreeRestore subtree(context);
  context->AddLayerBounds(fake_paint_path_.GetBounds());
  context->SetLayerPaintRegion(this, context->CurrentSubtreeRegion());
}

void MockLayer::Preroll(PrerollContext* context) {
  context->state_stack.fill(&parent_mutators_);
  parent_matrix_ = context->state_stack.matrix();
  parent_cull_rect_ = context->state_stack.local_cull_rect();

  set_parent_has_platform_view(context->has_platform_view);
  set_parent_has_texture_layer(context->has_texture_layer);

  context->has_platform_view = fake_has_platform_view();
  context->has_texture_layer = fake_has_texture_layer();
  set_paint_bounds(fake_paint_path_.GetBounds());
  if (fake_reads_surface()) {
    context->surface_needs_readback = true;
  }
  if (fake_opacity_compatible()) {
    context->renderable_state_flags = LayerStateStack::kCallerCanApplyOpacity;
  }
}

void MockLayer::Paint(PaintContext& context) const {
  FML_DCHECK(needs_painting(context));

  if (expected_paint_matrix_.has_value()) {
    EXPECT_EQ(context.canvas->GetMatrix(), expected_paint_matrix_.value());
  }

  DlPaint paint = fake_paint_;
  context.state_stack.fill(paint);
  context.canvas->DrawPath(fake_paint_path_, paint);
}

void MockCacheableContainerLayer::Preroll(PrerollContext* context) {
  Layer::AutoPrerollSaveLayerState save =
      Layer::AutoPrerollSaveLayerState::Create(context);
  auto cache = AutoCache(layer_raster_cache_item_.get(), context,
                         context->state_stack.matrix());

  ContainerLayer::Preroll(context);
}

void MockCacheableLayer::Preroll(PrerollContext* context) {
  Layer::AutoPrerollSaveLayerState save =
      Layer::AutoPrerollSaveLayerState::Create(context);
  auto cache = AutoCache(raster_cache_item_.get(), context,
                         context->state_stack.matrix());

  MockLayer::Preroll(context);
}

}  // namespace testing
}  // namespace flutter

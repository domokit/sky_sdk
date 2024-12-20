// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder_layers.h"

#include <algorithm>

namespace flutter {

EmbedderLayers::EmbedderLayers(SkISize frame_size,
                               double device_pixel_ratio,
                               SkMatrix root_surface_transformation,
                               uint64_t presentation_time)
    : frame_size_(frame_size),
      device_pixel_ratio_(device_pixel_ratio),
      root_surface_transformation_(root_surface_transformation),
      presentation_time_(presentation_time) {}

EmbedderLayers::~EmbedderLayers() = default;

void EmbedderLayers::PushBackingStoreLayer(
    const FlutterBackingStore* store,
    const std::vector<DlIRect>& paint_region_vec) {
  FlutterLayer layer = {};

  layer.struct_size = sizeof(FlutterLayer);
  layer.type = kFlutterLayerContentTypeBackingStore;
  layer.backing_store = store;

  const auto layer_bounds =
      SkRect::MakeWH(frame_size_.width(), frame_size_.height());

  const auto transformed_layer_bounds =
      root_surface_transformation_.mapRect(layer_bounds);

  layer.offset.x = transformed_layer_bounds.x();
  layer.offset.y = transformed_layer_bounds.y();
  layer.size.width = transformed_layer_bounds.width();
  layer.size.height = transformed_layer_bounds.height();

  auto paint_region_rects = std::make_unique<std::vector<FlutterRect>>();
  paint_region_rects->reserve(paint_region_vec.size());

  for (const auto& rect : paint_region_vec) {
    auto transformed_rect =
        root_surface_transformation_.mapRect(SkRect::Make(ToSkIRect(rect)));
    paint_region_rects->push_back(FlutterRect{
        transformed_rect.x(),
        transformed_rect.y(),
        transformed_rect.right(),
        transformed_rect.bottom(),
    });
  }

  auto paint_region = std::make_unique<FlutterRegion>();
  paint_region->struct_size = sizeof(FlutterRegion);
  paint_region->rects = paint_region_rects->data();
  paint_region->rects_count = paint_region_rects->size();
  rects_referenced_.push_back(std::move(paint_region_rects));

  auto present_info = std::make_unique<FlutterBackingStorePresentInfo>();
  present_info->struct_size = sizeof(FlutterBackingStorePresentInfo);
  present_info->paint_region = paint_region.get();
  regions_referenced_.push_back(std::move(paint_region));
  layer.backing_store_present_info = present_info.get();
  layer.presentation_time = presentation_time_;

  present_info_referenced_.push_back(std::move(present_info));
  presented_layers_.push_back(layer);
}

static std::unique_ptr<FlutterPlatformViewMutation> ConvertMutation(
    double opacity) {
  FlutterPlatformViewMutation mutation = {};
  mutation.type = kFlutterPlatformViewMutationTypeOpacity;
  mutation.opacity = opacity;
  return std::make_unique<FlutterPlatformViewMutation>(mutation);
}

static std::unique_ptr<FlutterPlatformViewMutation> ConvertMutation(
    const SkRect& rect) {
  FlutterPlatformViewMutation mutation = {};
  mutation.type = kFlutterPlatformViewMutationTypeClipRect;
  mutation.clip_rect.left = rect.left();
  mutation.clip_rect.top = rect.top();
  mutation.clip_rect.right = rect.right();
  mutation.clip_rect.bottom = rect.bottom();
  return std::make_unique<FlutterPlatformViewMutation>(mutation);
}

static FlutterSize VectorToSize(const SkVector& vector) {
  FlutterSize size = {};
  size.width = vector.x();
  size.height = vector.y();
  return size;
}

static std::unique_ptr<FlutterPlatformViewMutation> ConvertMutation(
    const SkRRect& rrect) {
  FlutterPlatformViewMutation mutation = {};
  mutation.type = kFlutterPlatformViewMutationTypeClipRoundedRect;
  const auto& rect = rrect.rect();
  mutation.clip_rounded_rect.rect.left = rect.left();
  mutation.clip_rounded_rect.rect.top = rect.top();
  mutation.clip_rounded_rect.rect.right = rect.right();
  mutation.clip_rounded_rect.rect.bottom = rect.bottom();
  mutation.clip_rounded_rect.upper_left_corner_radius =
      VectorToSize(rrect.radii(SkRRect::Corner::kUpperLeft_Corner));
  mutation.clip_rounded_rect.upper_right_corner_radius =
      VectorToSize(rrect.radii(SkRRect::Corner::kUpperRight_Corner));
  mutation.clip_rounded_rect.lower_right_corner_radius =
      VectorToSize(rrect.radii(SkRRect::Corner::kLowerRight_Corner));
  mutation.clip_rounded_rect.lower_left_corner_radius =
      VectorToSize(rrect.radii(SkRRect::Corner::kLowerLeft_Corner));
  return std::make_unique<FlutterPlatformViewMutation>(mutation);
}

static std::unique_ptr<FlutterPlatformViewMutation> ConvertMutation(
    const SkMatrix& matrix) {
  FlutterPlatformViewMutation mutation = {};
  mutation.type = kFlutterPlatformViewMutationTypeTransformation;
  mutation.transformation.scaleX = matrix[SkMatrix::kMScaleX];
  mutation.transformation.skewX = matrix[SkMatrix::kMSkewX];
  mutation.transformation.transX = matrix[SkMatrix::kMTransX];
  mutation.transformation.skewY = matrix[SkMatrix::kMSkewY];
  mutation.transformation.scaleY = matrix[SkMatrix::kMScaleY];
  mutation.transformation.transY = matrix[SkMatrix::kMTransY];
  mutation.transformation.pers0 = matrix[SkMatrix::kMPersp0];
  mutation.transformation.pers1 = matrix[SkMatrix::kMPersp1];
  mutation.transformation.pers2 = matrix[SkMatrix::kMPersp2];
  return std::make_unique<FlutterPlatformViewMutation>(mutation);
}

void EmbedderLayers::PushPlatformViewLayer(
    FlutterPlatformViewIdentifier identifier,
    const EmbeddedViewParams& params) {
  {
    FlutterPlatformView view = {};
    view.struct_size = sizeof(FlutterPlatformView);
    view.identifier = identifier;

    const auto& mutators = params.mutatorsStack();

    std::vector<const FlutterPlatformViewMutation*> mutations_array;

    for (auto i = mutators.Bottom(); i != mutators.Top(); ++i) {
      const auto& mutator = *i;
      switch (mutator->GetType()) {
        case MutatorType::kClipRect: {
          mutations_array.push_back(
              mutations_referenced_
                  .emplace_back(ConvertMutation(mutator->GetRect()))
                  .get());
        } break;
        case MutatorType::kClipRRect: {
          mutations_array.push_back(
              mutations_referenced_
                  .emplace_back(ConvertMutation(mutator->GetRRect()))
                  .get());
        } break;
        case MutatorType::kClipPath: {
          // Unsupported mutation.
        } break;
        case MutatorType::kTransform: {
          const auto& matrix = mutator->GetMatrix();
          if (!matrix.isIdentity()) {
            mutations_array.push_back(
                mutations_referenced_.emplace_back(ConvertMutation(matrix))
                    .get());
          }
        } break;
        case MutatorType::kOpacity: {
          const double opacity =
              std::clamp(mutator->GetAlphaFloat(), 0.0f, 1.0f);
          if (opacity < 1.0) {
            mutations_array.push_back(
                mutations_referenced_.emplace_back(ConvertMutation(opacity))
                    .get());
          }
        } break;
        case MutatorType::kBackdropFilter:
          break;
      }
    }

    if (!mutations_array.empty()) {
      // If there are going to be any mutations, they must first take into
      // account the root surface transformation.
      if (!root_surface_transformation_.isIdentity()) {
        mutations_array.push_back(
            mutations_referenced_
                .emplace_back(ConvertMutation(root_surface_transformation_))
                .get());
      }

      auto mutations =
          std::make_unique<std::vector<const FlutterPlatformViewMutation*>>(
              mutations_array.rbegin(), mutations_array.rend());
      mutations_arrays_referenced_.emplace_back(std::move(mutations));

      view.mutations_count = mutations_array.size();
      view.mutations = mutations_arrays_referenced_.back().get()->data();
    }

    platform_views_referenced_.emplace_back(
        std::make_unique<FlutterPlatformView>(view));
  }

  FlutterLayer layer = {};

  layer.struct_size = sizeof(FlutterLayer);
  layer.type = kFlutterLayerContentTypePlatformView;
  layer.platform_view = platform_views_referenced_.back().get();

  const auto layer_bounds =
      SkRect::MakeXYWH(params.finalBoundingRect().x(),                     //
                       params.finalBoundingRect().y(),                     //
                       params.sizePoints().width() * device_pixel_ratio_,  //
                       params.sizePoints().height() * device_pixel_ratio_  //
      );

  const auto transformed_layer_bounds =
      root_surface_transformation_.mapRect(layer_bounds);

  layer.offset.x = transformed_layer_bounds.x();
  layer.offset.y = transformed_layer_bounds.y();
  layer.size.width = transformed_layer_bounds.width();
  layer.size.height = transformed_layer_bounds.height();

  layer.presentation_time = presentation_time_;

  presented_layers_.push_back(layer);
}

void EmbedderLayers::InvokePresentCallback(
    FlutterViewId view_id,
    const PresentCallback& callback) const {
  std::vector<const FlutterLayer*> presented_layers_pointers;
  presented_layers_pointers.reserve(presented_layers_.size());
  for (const auto& layer : presented_layers_) {
    presented_layers_pointers.push_back(&layer);
  }
  callback(view_id, presented_layers_pointers);
}

}  // namespace flutter

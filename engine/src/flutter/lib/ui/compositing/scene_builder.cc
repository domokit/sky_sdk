// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/compositing/scene_builder.h"
#include <cstdint>

#include "dart_api.h"
#include "flutter/flow/layers/backdrop_filter_layer.h"
#include "flutter/flow/layers/clip_path_layer.h"
#include "flutter/flow/layers/clip_rect_layer.h"
#include "flutter/flow/layers/clip_rrect_layer.h"
#include "flutter/flow/layers/color_filter_layer.h"
#include "flutter/flow/layers/container_layer.h"
#include "flutter/flow/layers/display_list_layer.h"
#include "flutter/flow/layers/image_filter_layer.h"
#include "flutter/flow/layers/layer.h"
#include "flutter/flow/layers/opacity_layer.h"
#include "flutter/flow/layers/performance_overlay_layer.h"
#include "flutter/flow/layers/platform_view_layer.h"
#include "flutter/flow/layers/shader_mask_layer.h"
#include "flutter/flow/layers/texture_layer.h"
#include "flutter/flow/layers/transform_layer.h"
#include "flutter/fml/build_config.h"
#include "flutter/lib/ui/compositing/scene.h"
#include "flutter/lib/ui/floating_point.h"
#include "flutter/lib/ui/painting/matrix.h"
#include "flutter/lib/ui/painting/shader.h"

namespace flutter {

IMPLEMENT_WRAPPERTYPEINFO(ui, SceneBuilder);

SceneBuilder::SceneBuilder() {
  // Add a ContainerLayer as the root layer, so that AddLayer operations are
  // always valid.
  PushLayer(std::make_shared<flutter::ContainerLayer>());
}

SceneBuilder::~SceneBuilder() = default;

void SceneBuilder::pushTransform(Dart_Handle layer_handle,
                                 tonic::Float64List& matrix4,
                                 const fml::RefPtr<EngineLayer>& old_layer) {
  DlMatrix matrix = ToDlMatrix(matrix4);
  auto layer = std::make_shared<flutter::TransformLayer>(matrix);
  PushLayer(layer);
  // matrix4 has to be released before we can return another Dart object
  matrix4.Release();
  EngineLayer::MakeRetained(layer_handle, layer);

  if (old_layer && old_layer->Layer()) {
    layer->AssignOldLayer(old_layer->Layer().get());
  }
}

void SceneBuilder::pushOffset(Dart_Handle layer_handle,
                              double dx,
                              double dy,
                              const fml::RefPtr<EngineLayer>& old_layer) {
  DlMatrix matrix = DlMatrix::MakeTranslation({SafeNarrow(dx), SafeNarrow(dy)});
  auto layer = std::make_shared<flutter::TransformLayer>(matrix);
  PushLayer(layer);
  EngineLayer::MakeRetained(layer_handle, layer);

  if (old_layer && old_layer->Layer()) {
    layer->AssignOldLayer(old_layer->Layer().get());
  }
}

void SceneBuilder::pushClipRect(Dart_Handle layer_handle,
                                double left,
                                double right,
                                double top,
                                double bottom,
                                int clip_behavior,
                                const fml::RefPtr<EngineLayer>& old_layer) {
  DlRect clip_rect = DlRect::MakeLTRB(SafeNarrow(left), SafeNarrow(top),
                                      SafeNarrow(right), SafeNarrow(bottom));
  auto layer = std::make_shared<flutter::ClipRectLayer>(
      clip_rect, static_cast<flutter::Clip>(clip_behavior));
  PushLayer(layer);
  EngineLayer::MakeRetained(layer_handle, layer);

  if (old_layer && old_layer->Layer()) {
    layer->AssignOldLayer(old_layer->Layer().get());
  }
}

void SceneBuilder::pushClipRRect(Dart_Handle layer_handle,
                                 const RRect& rrect,
                                 int clip_behavior,
                                 const fml::RefPtr<EngineLayer>& old_layer) {
  auto layer = std::make_shared<flutter::ClipRRectLayer>(
      rrect.rrect, static_cast<flutter::Clip>(clip_behavior));
  PushLayer(layer);
  EngineLayer::MakeRetained(layer_handle, layer);

  if (old_layer && old_layer->Layer()) {
    layer->AssignOldLayer(old_layer->Layer().get());
  }
}

void SceneBuilder::pushClipPath(Dart_Handle layer_handle,
                                const CanvasPath* path,
                                int clip_behavior,
                                const fml::RefPtr<EngineLayer>& old_layer) {
  flutter::Clip flutter_clip_behavior =
      static_cast<flutter::Clip>(clip_behavior);
  FML_DCHECK(flutter_clip_behavior != flutter::Clip::kNone);
  auto layer = std::make_shared<flutter::ClipPathLayer>(
      path->path(), static_cast<flutter::Clip>(flutter_clip_behavior));
  PushLayer(layer);
  EngineLayer::MakeRetained(layer_handle, layer);

  if (old_layer && old_layer->Layer()) {
    layer->AssignOldLayer(old_layer->Layer().get());
  }
}

void SceneBuilder::pushOpacity(Dart_Handle layer_handle,
                               int alpha,
                               double dx,
                               double dy,
                               const fml::RefPtr<EngineLayer>& old_layer) {
  auto layer = std::make_shared<flutter::OpacityLayer>(
      alpha, DlPoint(SafeNarrow(dx), SafeNarrow(dy)));
  PushLayer(layer);
  EngineLayer::MakeRetained(layer_handle, layer);

  if (old_layer && old_layer->Layer()) {
    layer->AssignOldLayer(old_layer->Layer().get());
  }
}

void SceneBuilder::pushColorFilter(Dart_Handle layer_handle,
                                   const ColorFilter* color_filter,
                                   const fml::RefPtr<EngineLayer>& old_layer) {
  auto layer =
      std::make_shared<flutter::ColorFilterLayer>(color_filter->filter());
  PushLayer(layer);
  EngineLayer::MakeRetained(layer_handle, layer);

  if (old_layer && old_layer->Layer()) {
    layer->AssignOldLayer(old_layer->Layer().get());
  }
}

void SceneBuilder::pushImageFilter(Dart_Handle layer_handle,
                                   const ImageFilter* image_filter,
                                   double dx,
                                   double dy,
                                   const fml::RefPtr<EngineLayer>& old_layer) {
  auto layer = std::make_shared<flutter::ImageFilterLayer>(
      image_filter->filter(DlTileMode::kDecal),
      DlPoint(SafeNarrow(dx), SafeNarrow(dy)));
  PushLayer(layer);
  EngineLayer::MakeRetained(layer_handle, layer);

  if (old_layer && old_layer->Layer()) {
    layer->AssignOldLayer(old_layer->Layer().get());
  }
}

void SceneBuilder::pushBackdropFilter(
    Dart_Handle layer_handle,
    ImageFilter* filter,
    int blend_mode,
    Dart_Handle backdrop_id,
    const fml::RefPtr<EngineLayer>& old_layer) {
  std::optional<int64_t> converted_backdrop_id;
  if (Dart_IsInteger(backdrop_id)) {
    int64_t out;
    Dart_IntegerToInt64(backdrop_id, &out);
    converted_backdrop_id = out;
  }

  auto layer = std::make_shared<flutter::BackdropFilterLayer>(
      filter->filter(DlTileMode::kMirror), static_cast<DlBlendMode>(blend_mode),
      converted_backdrop_id);
  PushLayer(layer);
  EngineLayer::MakeRetained(layer_handle, layer);

  if (old_layer && old_layer->Layer()) {
    layer->AssignOldLayer(old_layer->Layer().get());
  }
}

void SceneBuilder::pushShaderMask(Dart_Handle layer_handle,
                                  Shader* shader,
                                  double mask_rect_left,
                                  double mask_rect_right,
                                  double mask_rect_top,
                                  double mask_rect_bottom,
                                  int blend_mode,
                                  int filter_quality_index,
                                  const fml::RefPtr<EngineLayer>& old_layer) {
  DlRect rect = DlRect::MakeLTRB(
      SafeNarrow(mask_rect_left), SafeNarrow(mask_rect_top),
      SafeNarrow(mask_rect_right), SafeNarrow(mask_rect_bottom));
  auto sampling = ImageFilter::SamplingFromIndex(filter_quality_index);
  auto layer = std::make_shared<flutter::ShaderMaskLayer>(
      shader->shader(sampling), rect, static_cast<DlBlendMode>(blend_mode));
  PushLayer(layer);
  EngineLayer::MakeRetained(layer_handle, layer);

  if (old_layer && old_layer->Layer()) {
    layer->AssignOldLayer(old_layer->Layer().get());
  }
}

void SceneBuilder::addRetained(const fml::RefPtr<EngineLayer>& retained_layer) {
  AddLayer(retained_layer->Layer());
}

void SceneBuilder::pop() {
  PopLayer();
}

void SceneBuilder::addPicture(double dx,
                              double dy,
                              Picture* picture,
                              int hints) {
  if (!picture) {
    // Picture::dispose was called and it has been collected.
    return;
  }

  // Explicitly check for display_list, since the picture object might have
  // been disposed but not collected yet, but the display list is null.
  if (picture->display_list()) {
    auto layer = std::make_unique<flutter::DisplayListLayer>(
        DlPoint(SafeNarrow(dx), SafeNarrow(dy)), picture->display_list(),
        !!(hints & 1), !!(hints & 2));
    AddLayer(std::move(layer));
  }
}

void SceneBuilder::addTexture(double dx,
                              double dy,
                              double width,
                              double height,
                              int64_t texture_id,
                              bool freeze,
                              int filter_quality_index) {
  auto sampling = ImageFilter::SamplingFromIndex(filter_quality_index);
  auto layer = std::make_unique<flutter::TextureLayer>(
      DlPoint(SafeNarrow(dx), SafeNarrow(dy)),
      DlSize(SafeNarrow(width), SafeNarrow(height)), texture_id, freeze,
      sampling);
  AddLayer(std::move(layer));
}

void SceneBuilder::addPlatformView(double dx,
                                   double dy,
                                   double width,
                                   double height,
                                   int64_t view_id) {
  auto layer = std::make_unique<flutter::PlatformViewLayer>(
      DlPoint(SafeNarrow(dx), SafeNarrow(dy)),
      DlSize(SafeNarrow(width), SafeNarrow(height)), view_id);
  AddLayer(std::move(layer));
}

void SceneBuilder::addPerformanceOverlay(uint64_t enabled_options,
                                         double left,
                                         double right,
                                         double top,
                                         double bottom) {
  DlRect rect = DlRect::MakeLTRB(SafeNarrow(left), SafeNarrow(top),
                                 SafeNarrow(right), SafeNarrow(bottom));
  auto layer =
      std::make_unique<flutter::PerformanceOverlayLayer>(enabled_options);
  layer->set_paint_bounds(rect);
  AddLayer(std::move(layer));
}

void SceneBuilder::build(Dart_Handle scene_handle) {
  FML_DCHECK(layer_stack_.size() >= 1);

  Scene::create(scene_handle, std::move(layer_stack_[0]));
  layer_stack_.clear();
  ClearDartWrapper();  // may delete this object.
}

void SceneBuilder::AddLayer(std::shared_ptr<Layer> layer) {
  FML_DCHECK(layer);

  if (!layer_stack_.empty()) {
    layer_stack_.back()->Add(std::move(layer));
  }
}

void SceneBuilder::PushLayer(std::shared_ptr<ContainerLayer> layer) {
  AddLayer(layer);
  layer_stack_.push_back(std::move(layer));
}

void SceneBuilder::PopLayer() {
  // We never pop the root layer, so that AddLayer operations are always valid.
  if (layer_stack_.size() > 1) {
    layer_stack_.pop_back();
  }
}

}  // namespace flutter

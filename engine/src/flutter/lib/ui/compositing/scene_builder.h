// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_COMPOSITING_SCENE_BUILDER_H_
#define FLUTTER_LIB_UI_COMPOSITING_SCENE_BUILDER_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "flutter/flow/layers/container_layer.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/color_filter.h"
#include "flutter/lib/ui/painting/engine_layer.h"
#include "flutter/lib/ui/painting/image_filter.h"
#include "flutter/lib/ui/painting/path.h"
#include "flutter/lib/ui/painting/picture.h"
#include "flutter/lib/ui/painting/rrect.h"
#include "flutter/lib/ui/painting/shader.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

class SceneBuilder : public RefCountedDartWrappable<SceneBuilder> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(SceneBuilder);

 public:
  static void Create(Dart_Handle wrapper) {
    UIDartState::ThrowIfUIOperationsProhibited();
    auto res = fml::MakeRefCounted<SceneBuilder>();
    res->AssociateWithDartWrapper(wrapper);
  }

  ~SceneBuilder() override;

  void pushTransformHandle(Dart_Handle layer_handle,
                           Dart_Handle matrix4_handle,
                           const fml::RefPtr<EngineLayer>& old_layer) {
    tonic::Float64List matrix4(matrix4_handle);
    pushTransform(layer_handle, matrix4, old_layer);
  }
  void pushTransform(Dart_Handle layer_handle,
                     tonic::Float64List& matrix4,
                     const fml::RefPtr<EngineLayer>& old_layer);
  void pushOffset(Dart_Handle layer_handle,
                  double dx,
                  double dy,
                  const fml::RefPtr<EngineLayer>& old_layer);
  void pushClipRect(Dart_Handle layer_handle,
                    double left,
                    double right,
                    double top,
                    double bottom,
                    int clip_behavior,
                    const fml::RefPtr<EngineLayer>& old_layer);
  void pushClipRRect(Dart_Handle layer_handle,
                     const RRect& rrect,
                     int clip_behavior,
                     const fml::RefPtr<EngineLayer>& old_layer);
  void pushClipPath(Dart_Handle layer_handle,
                    const CanvasPath* path,
                    int clip_behavior,
                    const fml::RefPtr<EngineLayer>& old_layer);
  void pushOpacity(Dart_Handle layer_handle,
                   int alpha,
                   double dx,
                   double dy,
                   const fml::RefPtr<EngineLayer>& old_layer);
  void pushColorFilter(Dart_Handle layer_handle,
                       const ColorFilter* color_filter,
                       const fml::RefPtr<EngineLayer>& old_layer);
  void pushImageFilter(Dart_Handle layer_handle,
                       const ImageFilter* image_filter,
                       double dx,
                       double dy,
                       const fml::RefPtr<EngineLayer>& old_layer);
  void pushBackdropFilter(Dart_Handle layer_handle,
                          ImageFilter* filter,
                          int blend_mode,
                          Dart_Handle backdrop_id,
                          const fml::RefPtr<EngineLayer>& old_layer);
  void pushShaderMask(Dart_Handle layer_handle,
                      Shader* shader,
                      double mask_rect_left,
                      double mask_rect_right,
                      double mask_rect_top,
                      double mask_rect_bottom,
                      int blend_mode,
                      int filter_quality_index,
                      const fml::RefPtr<EngineLayer>& old_layer);

  void addRetained(const fml::RefPtr<EngineLayer>& retained_layer);

  void pop();

  void addPerformanceOverlay(uint64_t enabled_options,
                             double left,
                             double right,
                             double top,
                             double bottom);

  void addPicture(double dx, double dy, Picture* picture, int hints);

  void addTexture(double dx,
                  double dy,
                  double width,
                  double height,
                  int64_t texture_id,
                  bool freeze,
                  int filter_quality);

  void addPlatformView(double dx,
                       double dy,
                       double width,
                       double height,
                       int64_t view_id);

  void build(Dart_Handle scene_handle);

  const std::vector<std::shared_ptr<ContainerLayer>>& layer_stack() {
    return layer_stack_;
  }

 private:
  SceneBuilder();

  void AddLayer(std::shared_ptr<Layer> layer);
  void PushLayer(std::shared_ptr<ContainerLayer> layer);
  void PopLayer();

  std::vector<std::shared_ptr<ContainerLayer>> layer_stack_;

  FML_DISALLOW_COPY_AND_ASSIGN(SceneBuilder);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_COMPOSITING_SCENE_BUILDER_H_

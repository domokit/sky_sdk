// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_ENGINE_LAYER_H_
#define FLUTTER_LIB_UI_PAINTING_ENGINE_LAYER_H_

#include "flutter/flow/layers/container_layer.h"
#include "flutter/lib/ui/dart_wrapper.h"

namespace flutter {

class EngineLayer;

class EngineLayer : public RefCountedDartWrappable<EngineLayer> {
  DEFINE_WRAPPERTYPEINFO();

 public:
  ~EngineLayer() override;

  static void MakeRetained(
      Dart_Handle dart_handle,
      const std::shared_ptr<flutter::ContainerLayer>& layer) {
    auto engine_layer = fml::MakeRefCounted<EngineLayer>(layer);
    engine_layer->AssociateWithDartWrapper(dart_handle);
  }

  void dispose();

  std::shared_ptr<flutter::ContainerLayer> Layer() const { return layer_; }

 private:
  explicit EngineLayer(std::shared_ptr<flutter::ContainerLayer> layer);
  std::shared_ptr<flutter::ContainerLayer> layer_;

  FML_FRIEND_MAKE_REF_COUNTED(EngineLayer);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_ENGINE_LAYER_H_

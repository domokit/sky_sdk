// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_COMPOSITING_SCENE_H_
#define FLUTTER_LIB_UI_COMPOSITING_SCENE_H_

#include <cstdint>
#include <memory>

#include "flutter/flow/layers/layer_tree.h"
#include "flutter/lib/ui/dart_wrapper.h"

namespace flutter {

class Scene : public RefCountedDartWrappable<Scene> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(Scene);

 public:
  ~Scene() override;
  static void create(Dart_Handle scene_handle,
                     std::shared_ptr<flutter::Layer> rootLayer);

  std::unique_ptr<flutter::LayerTree> takeLayerTree(uint64_t width,
                                                    uint64_t height);

  Dart_Handle toImageSync(uint32_t width,
                          uint32_t height,
                          Dart_Handle raw_image_handle);

  Dart_Handle toImage(uint32_t width,
                      uint32_t height,
                      Dart_Handle raw_image_handle);

  void dispose();

 private:
  explicit Scene(std::shared_ptr<flutter::Layer> rootLayer);

  // Returns true if `dispose()` has not been called.
  bool valid();

  void RasterizeToImage(uint32_t width,
                        uint32_t height,
                        Dart_Handle raw_image_handle);

  std::unique_ptr<LayerTree> BuildLayerTree(uint32_t width, uint32_t height);

  std::shared_ptr<Layer> layer_tree_root_layer_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_COMPOSITING_SCENE_H_

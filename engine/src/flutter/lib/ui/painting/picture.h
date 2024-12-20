// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_PICTURE_H_
#define FLUTTER_LIB_UI_PAINTING_PICTURE_H_

#include "flutter/display_list/display_list.h"
#include "flutter/flow/layers/layer_tree.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/image.h"
#include "flutter/lib/ui/ui_dart_state.h"

namespace flutter {
class Canvas;

class Picture : public RefCountedDartWrappable<Picture> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(Picture);

 public:
  ~Picture() override;
  static void CreateAndAssociateWithDartWrapper(
      Dart_Handle dart_handle,
      sk_sp<DisplayList> display_list);

  sk_sp<DisplayList> display_list() const { return display_list_; }

  Dart_Handle toImage(uint32_t width,
                      uint32_t height,
                      Dart_Handle raw_image_callback);

  void toImageSync(uint32_t width,
                   uint32_t height,
                   Dart_Handle raw_image_handle);

  void dispose();

  size_t GetAllocationSize() const;

  static void RasterizeToImageSync(sk_sp<DisplayList> display_list,
                                   uint32_t width,
                                   uint32_t height,
                                   Dart_Handle raw_image_handle);

  static Dart_Handle RasterizeToImage(const sk_sp<DisplayList>& display_list,
                                      uint32_t width,
                                      uint32_t height,
                                      Dart_Handle raw_image_callback);

  static Dart_Handle RasterizeLayerTreeToImage(
      std::unique_ptr<LayerTree> layer_tree,
      Dart_Handle raw_image_callback);

  // Callers may provide either a display list or a layer tree, but not both.
  //
  // If a layer tree is provided, it will be flattened on the raster thread, and
  // picture_bounds should be the layer tree's frame_size().
  static Dart_Handle DoRasterizeToImage(const sk_sp<DisplayList>& display_list,
                                        std::unique_ptr<LayerTree> layer_tree,
                                        uint32_t width,
                                        uint32_t height,
                                        Dart_Handle raw_image_callback);

 private:
  explicit Picture(sk_sp<DisplayList> display_list);

  sk_sp<DisplayList> display_list_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_PICTURE_H_

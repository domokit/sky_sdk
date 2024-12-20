// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_SHADER_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_SHADER_H_

#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/gradient.h"
#include "flutter/lib/ui/painting/image.h"
#include "flutter/lib/ui/painting/matrix.h"
#include "flutter/lib/ui/painting/shader.h"
#include "third_party/skia/include/core/SkMatrix.h"
#include "third_party/skia/include/core/SkShader.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

class ImageShader : public Shader {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(ImageShader);

 public:
  ~ImageShader() override;
  static void Create(Dart_Handle wrapper);

  Dart_Handle initWithImage(CanvasImage* image,
                            DlTileMode tmx,
                            DlTileMode tmy,
                            int filter_quality_index,
                            Dart_Handle matrix_handle);

  std::shared_ptr<DlColorSource> shader(DlImageSampling) override;

  int width();
  int height();

  void dispose();

 private:
  ImageShader();

  sk_sp<const DlImage> image_;
  bool sampling_is_locked_;

  std::shared_ptr<DlColorSource> cached_shader_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_SHADER_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/image_shader.h"
#include "flutter/lib/ui/painting/image_filter.h"

#include "flutter/display_list/effects/color_sources/dl_image_color_source.h"
#include "flutter/lib/ui/painting/display_list_image_gpu.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_args.h"
#include "third_party/tonic/dart_binding_macros.h"
#include "third_party/tonic/dart_library_natives.h"

using tonic::ToDart;

namespace flutter {

IMPLEMENT_WRAPPERTYPEINFO(ui, ImageShader);

void ImageShader::Create(Dart_Handle wrapper) {
  auto res = fml::MakeRefCounted<ImageShader>();
  res->AssociateWithDartWrapper(wrapper);
}

Dart_Handle ImageShader::initWithImage(CanvasImage* image,
                                       DlTileMode tmx,
                                       DlTileMode tmy,
                                       int filter_quality_index,
                                       Dart_Handle matrix_handle) {
  // CanvasImage should have already checked for a UI thread safe image.
  if (!image || !image->image()->isUIThreadSafe()) {
    return ToDart("ImageShader constructor called with non-genuine Image.");
  }

  image_ = image->image();
  tonic::Float64List matrix4(matrix_handle);
  DlMatrix local_matrix = ToDlMatrix(matrix4);
  matrix4.Release();
  sampling_is_locked_ = filter_quality_index >= 0;
  DlImageSampling sampling =
      sampling_is_locked_ ? ImageFilter::SamplingFromIndex(filter_quality_index)
                          : DlImageSampling::kLinear;
  cached_shader_ =
      DlColorSource::MakeImage(image_, tmx, tmy, sampling, &local_matrix);
  FML_DCHECK(cached_shader_->isUIThreadSafe());
  return Dart_Null();
}

std::shared_ptr<DlColorSource> ImageShader::shader(DlImageSampling sampling) {
  const DlImageColorSource* image_shader = cached_shader_->asImage();
  FML_DCHECK(image_shader);
  if (sampling_is_locked_ || sampling == image_shader->sampling()) {
    return cached_shader_;
  }
  return image_shader->WithSampling(sampling);
}

int ImageShader::width() {
  return image_->width();
}

int ImageShader::height() {
  return image_->height();
}

void ImageShader::dispose() {
  cached_shader_.reset();
  image_.reset();
  ClearDartWrapper();
}

ImageShader::ImageShader() = default;

ImageShader::~ImageShader() = default;

}  // namespace flutter

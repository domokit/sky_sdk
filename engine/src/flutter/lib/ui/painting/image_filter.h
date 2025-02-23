// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_FILTER_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_FILTER_H_

#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/color_filter.h"
#include "lib/ui/painting/fragment_shader.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace tonic {
class DartLibraryNatives;
}  // namespace tonic

namespace flutter {

class ImageFilter : public RefCountedDartWrappable<ImageFilter> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(ImageFilter);

 public:
  ~ImageFilter() override;
  static void Create(Dart_Handle wrapper);

  static DlImageSampling SamplingFromIndex(int filterQualityIndex);
  static DlFilterMode FilterModeFromIndex(int index);

  void initBlur(double sigma_x, double sigma_y, int tile_mode_index);
  void initDilate(double radius_x, double radius_y);
  void initErode(double radius_x, double radius_y);
  void initMatrix(const tonic::Float64List& matrix4, int filter_quality_index);
  void initColorFilter(ColorFilter* colorFilter);
  void initComposeFilter(ImageFilter* outer, ImageFilter* inner);
  void initShader(ReusableFragmentShader* shader);
  bool equals(ImageFilter* a, ImageFilter* b);

  const std::shared_ptr<DlImageFilter> filter(DlTileMode mode) const;

  static void RegisterNatives(tonic::DartLibraryNatives* natives);

 private:
  ImageFilter();

  std::shared_ptr<DlImageFilter> filter_;
  bool is_dynamic_tile_mode_ = false;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_FILTER_H_

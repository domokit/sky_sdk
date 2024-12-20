// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_IMAGE_FILTER_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_IMAGE_FILTER_H_

#include "flutter/display_list/effects/dl_image_filter.h"
#include "impeller/toolkit/interop/formats.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class ImageFilter final
    : public Object<ImageFilter,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerImageFilter)> {
 public:
  static ScopedObject<ImageFilter> MakeBlur(Scalar x_sigma,
                                            Scalar y_sigma,
                                            flutter::DlTileMode tile_mode);

  static ScopedObject<ImageFilter> MakeDilate(Scalar x_radius, Scalar y_radius);

  static ScopedObject<ImageFilter> MakeErode(Scalar x_radius, Scalar y_radius);

  static ScopedObject<ImageFilter> MakeMatrix(
      const Matrix& matrix,
      flutter::DlImageSampling sampling);

  static ScopedObject<ImageFilter> MakeCompose(const ImageFilter& outer,
                                               const ImageFilter& inner);

  explicit ImageFilter(std::shared_ptr<flutter::DlImageFilter> filter);

  ~ImageFilter() override;

  ImageFilter(const ImageFilter&) = delete;

  ImageFilter& operator=(const ImageFilter&) = delete;

  const std::shared_ptr<flutter::DlImageFilter>& GetImageFilter() const;

 private:
  std::shared_ptr<flutter::DlImageFilter> filter_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_IMAGE_FILTER_H_

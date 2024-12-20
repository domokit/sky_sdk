// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_NINE_PATCH_CONVERTER_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_NINE_PATCH_CONVERTER_H_

#include <memory>

#include "impeller/core/sampler_descriptor.h"
#include "impeller/display_list/canvas.h"
#include "impeller/display_list/paint.h"

namespace impeller {

// Converts a call to draw a nine patch image into a draw atlas call.
class NinePatchConverter {
 public:
  NinePatchConverter();

  ~NinePatchConverter();

  void DrawNinePatch(const std::shared_ptr<Texture>& image,
                     Rect center,
                     Rect dst,
                     const SamplerDescriptor& sampler,
                     Canvas* canvas,
                     Paint* paint);

 private:
  // Return a list of slice coordinates based on the size of the nine-slice
  // parameters in one dimension. Each set of slice coordinates contains a
  // begin/end pair for each of the source (image) and dest (screen) in the
  // order (src0, dst0, src1, dst1). The area from src0 => src1 of the image is
  // painted on the screen from dst0 => dst1 The slices for each dimension are
  // generated independently.
  std::vector<double> InitSlices(double img0,
                                 double imgC0,
                                 double imgC1,
                                 double img1,
                                 double dst0,
                                 double dst1);
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_NINE_PATCH_CONVERTER_H_

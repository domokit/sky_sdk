// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <vector>

#include "impeller/display_list/nine_patch_converter.h"

namespace impeller {

NinePatchConverter::NinePatchConverter() = default;

NinePatchConverter::~NinePatchConverter() = default;

std::vector<double> NinePatchConverter::InitSlices(double img0,
                                                   double imgC0,
                                                   double imgC1,
                                                   double img1,
                                                   double dst0,
                                                   double dst1) {
  auto imageDim = img1 - img0;
  auto destDim = dst1 - dst0;

  if (imageDim == destDim) {
    // If the src and dest are the same size then we do not need scaling
    // We return 4 values for a single slice
    return {img0, dst0, img1, dst1};
  }

  auto edge0Dim = imgC0 - img0;
  auto edge1Dim = img1 - imgC1;
  auto edgesDim = edge0Dim + edge1Dim;

  if (edgesDim >= destDim) {
    // the center portion has disappeared, leaving only the edges to scale to a
    // common center position in the destination this produces only 2 slices
    // which is 8 values
    auto dstC = dst0 + destDim * edge0Dim / edgesDim;
    // clang-format off
    return {
      img0,  dst0, imgC0, dstC,
      imgC1, dstC, img1,  dst1,
    };
    // clang-format on
  }

  // center portion is nonEmpty and only that part is scaled
  // we need 3 slices which is 12 values
  auto dstC0 = dst0 + edge0Dim;
  auto dstC1 = dst1 - edge1Dim;
  // clang-format off
  return {
    img0,  dst0,  imgC0, dstC0,
    imgC0, dstC0, imgC1, dstC1,
    imgC1, dstC1, img1,  dst1,
  };
  // clang-format on
}

void NinePatchConverter::DrawNinePatch(const std::shared_ptr<Texture>& image,
                                       Rect center,
                                       Rect dst,
                                       const SamplerDescriptor& sampler,
                                       Canvas* canvas,
                                       Paint* paint) {
  if (dst.IsEmpty()) {
    return;
  }
  auto image_size = image->GetSize();
  auto hSlices = InitSlices(0, center.GetLeft(), center.GetRight(),
                            image_size.width, dst.GetLeft(), dst.GetRight());
  auto vSlices = InitSlices(0, center.GetTop(), center.GetBottom(),
                            image_size.height, dst.GetTop(), dst.GetBottom());

  for (size_t yi = 0; yi < vSlices.size(); yi += 4) {
    auto srcY0 = vSlices[yi];
    auto dstY0 = vSlices[yi + 1];
    auto srcY1 = vSlices[yi + 2];
    auto dstY1 = vSlices[yi + 3];
    for (size_t xi = 0; xi < hSlices.size(); xi += 4) {
      auto srcX0 = hSlices[xi];
      auto dstX0 = hSlices[xi + 1];
      auto srcX1 = hSlices[xi + 2];
      auto dstX1 = hSlices[xi + 3];
      // TODO(jonahwilliams): consider converting this into a single call to
      // DrawImageAtlas.
      canvas->DrawImageRect(image, Rect::MakeLTRB(srcX0, srcY0, srcX1, srcY1),
                            Rect::MakeLTRB(dstX0, dstY0, dstX1, dstY1), *paint,
                            sampler, SourceRectConstraint::kStrict);
    }
  }
}

}  // namespace impeller

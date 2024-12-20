// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/flow/testing/mock_texture.h"
#include "flutter/flow/layers/layer.h"
#include "flutter/testing/display_list_testing.h"

namespace flutter {
namespace testing {

sk_sp<DlImage> MockTexture::MakeTestTexture(int w, int h, int checker_size) {
  sk_sp<SkSurface> surface =
      SkSurfaces::Raster(SkImageInfo::MakeN32Premul(w, h));
  SkCanvas* canvas = surface->getCanvas();
  SkPaint p0, p1;
  p0.setStyle(SkPaint::kFill_Style);
  p0.setColor(SK_ColorGREEN);
  p1.setStyle(SkPaint::kFill_Style);
  p1.setColor(SK_ColorBLUE);
  p1.setAlpha(128);
  for (int y = 0; y < w; y += checker_size) {
    for (int x = 0; x < h; x += checker_size) {
      SkPaint& cellp = ((x + y) & 1) == 0 ? p0 : p1;
      canvas->drawRect(SkRect::MakeXYWH(x, y, checker_size, checker_size),
                       cellp);
    }
  }
  return DlImage::Make(surface->makeImageSnapshot());
}

MockTexture::MockTexture(int64_t textureId, const sk_sp<DlImage>& texture)
    : Texture(textureId), texture_(texture) {}

void MockTexture::Paint(PaintContext& context,
                        const SkRect& bounds,
                        bool freeze,
                        const DlImageSampling sampling) {
  // MockTexture objects that are not painted are allowed to have a null
  // texture, but when we get to this method we must have a non-null texture.
  FML_DCHECK(texture_ != nullptr);
  SkRect src = SkRect::Make(texture_->bounds());
  if (freeze) {
    FML_DCHECK(src.width() > 2.0f && src.height() > 2.0f);
    src = src.makeInset(1.0f, 1.0f);
  }
  context.canvas->DrawImageRect(texture_, src, bounds, sampling, context.paint);
}

}  // namespace testing
}  // namespace flutter

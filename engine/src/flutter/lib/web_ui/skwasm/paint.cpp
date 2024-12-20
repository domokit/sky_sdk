// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "export.h"
#include "helpers.h"
#include "third_party/skia/include/core/SkColorFilter.h"
#include "third_party/skia/include/core/SkImageFilter.h"
#include "third_party/skia/include/core/SkMaskFilter.h"
#include "third_party/skia/include/core/SkPaint.h"
#include "third_party/skia/include/core/SkShader.h"

using namespace Skwasm;

SKWASM_EXPORT SkPaint* paint_create(bool isAntiAlias,
                                    SkBlendMode blendMode,
                                    SkColor color,
                                    SkPaint::Style style,
                                    SkScalar strokeWidth,
                                    SkPaint::Cap strokeCap,
                                    SkPaint::Join strokeJoin,
                                    SkScalar strokeMiterLimit) {
  auto paint = new SkPaint();
  paint->setAntiAlias(isAntiAlias);
  paint->setBlendMode(blendMode);
  paint->setStyle(style);
  paint->setStrokeWidth(strokeWidth);
  paint->setStrokeCap(strokeCap);
  paint->setStrokeJoin(strokeJoin);
  paint->setColor(color);
  paint->setStrokeMiter(strokeMiterLimit);
  return paint;
}

SKWASM_EXPORT void paint_dispose(SkPaint* paint) {
  delete paint;
}

SKWASM_EXPORT void paint_setShader(SkPaint* paint, SkShader* shader) {
  paint->setShader(sk_ref_sp<SkShader>(shader));
}

SKWASM_EXPORT void paint_setImageFilter(SkPaint* paint, SkImageFilter* filter) {
  paint->setImageFilter(sk_ref_sp<SkImageFilter>(filter));
}

SKWASM_EXPORT void paint_setColorFilter(SkPaint* paint, SkColorFilter* filter) {
  paint->setColorFilter(sk_ref_sp<SkColorFilter>(filter));
}

SKWASM_EXPORT void paint_setMaskFilter(SkPaint* paint, SkMaskFilter* filter) {
  paint->setMaskFilter(sk_ref_sp<SkMaskFilter>(filter));
}

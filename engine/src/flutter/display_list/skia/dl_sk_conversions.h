// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_SKIA_DL_SK_CONVERSIONS_H_
#define FLUTTER_DISPLAY_LIST_SKIA_DL_SK_CONVERSIONS_H_

#include "flutter/display_list/dl_op_receiver.h"
#include "flutter/display_list/skia/dl_sk_types.h"

namespace flutter {

SkPaint ToSk(const DlPaint& paint);
SkPaint ToStrokedSk(const DlPaint& paint);
SkPaint ToNonShaderSk(const DlPaint& paint);

inline SkBlendMode ToSk(DlBlendMode mode) {
  return static_cast<SkBlendMode>(mode);
}

inline SkColor ToSk(DlColor color) {
  return color.argb();
}

inline SkPaint::Style ToSk(DlDrawStyle style) {
  return static_cast<SkPaint::Style>(style);
}

inline SkPaint::Cap ToSk(DlStrokeCap cap) {
  return static_cast<SkPaint::Cap>(cap);
}

inline SkPaint::Join ToSk(DlStrokeJoin join) {
  return static_cast<SkPaint::Join>(join);
}

inline SkTileMode ToSk(DlTileMode dl_mode) {
  return static_cast<SkTileMode>(dl_mode);
}

inline SkBlurStyle ToSk(const DlBlurStyle blur_style) {
  return static_cast<SkBlurStyle>(blur_style);
}

inline SkFilterMode ToSk(const DlFilterMode filter_mode) {
  return static_cast<SkFilterMode>(filter_mode);
}

inline SkVertices::VertexMode ToSk(DlVertexMode dl_mode) {
  return static_cast<SkVertices::VertexMode>(dl_mode);
}

inline SkSamplingOptions ToSk(DlImageSampling sampling) {
  switch (sampling) {
    case DlImageSampling::kCubic:
      return SkSamplingOptions(SkCubicResampler{1 / 3.0f, 1 / 3.0f});
    case DlImageSampling::kLinear:
      return SkSamplingOptions(SkFilterMode::kLinear);
    case DlImageSampling::kMipmapLinear:
      return SkSamplingOptions(SkFilterMode::kLinear, SkMipmapMode::kLinear);
    case DlImageSampling::kNearestNeighbor:
      return SkSamplingOptions(SkFilterMode::kNearest);
  }
}

inline SkCanvas::SrcRectConstraint ToSk(
    DlCanvas::SrcRectConstraint constraint) {
  return static_cast<SkCanvas::SrcRectConstraint>(constraint);
}

inline SkClipOp ToSk(DlCanvas::ClipOp op) {
  return static_cast<SkClipOp>(op);
}

inline SkCanvas::PointMode ToSk(DlCanvas::PointMode mode) {
  return static_cast<SkCanvas::PointMode>(mode);
}

extern sk_sp<SkShader> ToSk(const DlColorSource* source);
inline sk_sp<SkShader> ToSk(
    const std::shared_ptr<const DlColorSource>& source) {
  return ToSk(source.get());
}
inline sk_sp<SkShader> ToSk(const DlColorSource& source) {
  return ToSk(&source);
}

extern sk_sp<SkImageFilter> ToSk(const DlImageFilter* filter);
inline sk_sp<SkImageFilter> ToSk(const std::shared_ptr<DlImageFilter>& filter) {
  return ToSk(filter.get());
}
inline sk_sp<SkImageFilter> ToSk(const DlImageFilter& filter) {
  return ToSk(&filter);
}

extern sk_sp<SkColorFilter> ToSk(const DlColorFilter* filter);
inline sk_sp<SkColorFilter> ToSk(
    const std::shared_ptr<const DlColorFilter>& filter) {
  return ToSk(filter.get());
}
inline sk_sp<SkColorFilter> ToSk(const DlColorFilter& filter) {
  return ToSk(&filter);
}

extern sk_sp<SkMaskFilter> ToSk(const DlMaskFilter* filter);
inline sk_sp<SkMaskFilter> ToSk(
    const std::shared_ptr<const DlMaskFilter>& filter) {
  return ToSk(filter.get());
}
inline sk_sp<SkMaskFilter> ToSk(const DlMaskFilter& filter) {
  return ToSk(&filter);
}

inline SkMatrix* ToSk(const DlMatrix* matrix, SkMatrix& scratch) {
  return matrix ? &scratch.setAll(matrix->m[0], matrix->m[4], matrix->m[12],  //
                                  matrix->m[1], matrix->m[5], matrix->m[13],  //
                                  matrix->m[3], matrix->m[7], matrix->m[15])
                : nullptr;
}

extern sk_sp<SkVertices> ToSk(const std::shared_ptr<DlVertices>& vertices);

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_SKIA_DL_SK_CONVERSIONS_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/paint.h"

#include "flutter/display_list/dl_builder.h"
#include "flutter/fml/logging.h"
#include "flutter/lib/ui/floating_point.h"
#include "flutter/lib/ui/painting/color_filter.h"
#include "flutter/lib/ui/painting/image_filter.h"
#include "flutter/lib/ui/painting/shader.h"
#include "third_party/skia/include/core/SkColorFilter.h"
#include "third_party/skia/include/core/SkImageFilter.h"
#include "third_party/skia/include/core/SkMaskFilter.h"
#include "third_party/skia/include/core/SkShader.h"
#include "third_party/skia/include/core/SkString.h"
#include "third_party/tonic/typed_data/dart_byte_data.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

// Indices for 32bit values.
// Must match //lib/ui/painting.dart.
constexpr int kIsAntiAliasIndex = 0;
constexpr int kColorRedIndex = 1;
constexpr int kColorGreenIndex = 2;
constexpr int kColorBlueIndex = 3;
constexpr int kColorAlphaIndex = 4;
constexpr int kColorSpaceIndex = 5;
constexpr int kBlendModeIndex = 6;
constexpr int kStyleIndex = 7;
constexpr int kStrokeWidthIndex = 8;
constexpr int kStrokeCapIndex = 9;
constexpr int kStrokeJoinIndex = 10;
constexpr int kStrokeMiterLimitIndex = 11;
constexpr int kFilterQualityIndex = 12;
constexpr int kMaskFilterIndex = 13;
constexpr int kMaskFilterBlurStyleIndex = 14;
constexpr int kMaskFilterSigmaIndex = 15;
constexpr int kInvertColorIndex = 16;
constexpr size_t kDataByteCount = 68;  // 4 * (last index + 1)
static_assert(kDataByteCount == sizeof(uint32_t) * (kInvertColorIndex + 1),
              "kDataByteCount must match the size of the data array.");

// Indices for objects.
constexpr int kShaderIndex = 0;
constexpr int kColorFilterIndex = 1;
constexpr int kImageFilterIndex = 2;
constexpr int kObjectCount = 3;  // One larger than largest object index.

// Must be kept in sync with the default in painting.dart.
constexpr uint32_t kBlendModeDefault =
    static_cast<uint32_t>(SkBlendMode::kSrcOver);

// Must be kept in sync with the default in painting.dart, and also with the
// default SkPaintDefaults_MiterLimit in Skia (which is not in a public header).
constexpr float kStrokeMiterLimitDefault = 4.0f;

// Must be kept in sync with the MaskFilter private constants in painting.dart.
enum MaskFilterType { kNull, kBlur };

namespace {
DlColor ReadColor(const tonic::DartByteData& byte_data) {
  const uint32_t* uint_data = static_cast<const uint32_t*>(byte_data.data());
  const float* float_data = static_cast<const float*>(byte_data.data());

  float red = float_data[kColorRedIndex];
  float green = float_data[kColorGreenIndex];
  float blue = float_data[kColorBlueIndex];
  // Invert alpha so 0 initialized buffer has default value;
  float alpha = 1.f - float_data[kColorAlphaIndex];
  uint32_t colorspace = uint_data[kColorSpaceIndex];

  DlColor dl_color(alpha, red, green, blue,
                   static_cast<DlColorSpace>(colorspace));

  return dl_color.withColorSpace(DlColorSpace::kExtendedSRGB);
}
}  // namespace

Paint::Paint(Dart_Handle paint_objects, Dart_Handle paint_data)
    : paint_objects_(paint_objects), paint_data_(paint_data) {}

const DlPaint* Paint::paint(DlPaint& paint,
                            const DisplayListAttributeFlags& flags,
                            DlTileMode tile_mode) const {
  if (isNull()) {
    return nullptr;
  }
  tonic::DartByteData byte_data(paint_data_);
  FML_CHECK(byte_data.length_in_bytes() == kDataByteCount);

  const uint32_t* uint_data = static_cast<const uint32_t*>(byte_data.data());
  const float* float_data = static_cast<const float*>(byte_data.data());

  Dart_Handle values[kObjectCount];
  if (Dart_IsNull(paint_objects_)) {
    if (flags.applies_shader()) {
      paint.setColorSource(nullptr);
    }
    if (flags.applies_color_filter()) {
      paint.setColorFilter(nullptr);
    }
    if (flags.applies_image_filter()) {
      paint.setImageFilter(nullptr);
    }
  } else {
    FML_DCHECK(Dart_IsList(paint_objects_));
    intptr_t length = 0;
    Dart_ListLength(paint_objects_, &length);

    FML_CHECK(length == kObjectCount);
    if (Dart_IsError(
            Dart_ListGetRange(paint_objects_, 0, kObjectCount, values))) {
      return nullptr;
    }

    if (flags.applies_shader()) {
      Dart_Handle shader = values[kShaderIndex];
      if (Dart_IsNull(shader)) {
        paint.setColorSource(nullptr);
      } else {
        if (Shader* decoded = tonic::DartConverter<Shader*>::FromDart(shader)) {
          auto sampling =
              ImageFilter::SamplingFromIndex(uint_data[kFilterQualityIndex]);
          paint.setColorSource(decoded->shader(sampling));
        } else {
          paint.setColorSource(nullptr);
        }
      }
    }

    if (flags.applies_color_filter()) {
      Dart_Handle color_filter = values[kColorFilterIndex];
      if (Dart_IsNull(color_filter)) {
        paint.setColorFilter(nullptr);
      } else {
        ColorFilter* decoded =
            tonic::DartConverter<ColorFilter*>::FromDart(color_filter);
        paint.setColorFilter(decoded->filter());
      }
    }

    if (flags.applies_image_filter()) {
      Dart_Handle image_filter = values[kImageFilterIndex];
      if (Dart_IsNull(image_filter)) {
        paint.setImageFilter(nullptr);
      } else {
        ImageFilter* decoded =
            tonic::DartConverter<ImageFilter*>::FromDart(image_filter);
        paint.setImageFilter(decoded->filter(tile_mode));
      }
    }
  }

  if (flags.applies_anti_alias()) {
    paint.setAntiAlias(uint_data[kIsAntiAliasIndex] == 0);
  }

  if (flags.applies_alpha_or_color()) {
    paint.setColor(ReadColor(byte_data));
  }

  if (flags.applies_blend()) {
    uint32_t encoded_blend_mode = uint_data[kBlendModeIndex];
    uint32_t blend_mode = encoded_blend_mode ^ kBlendModeDefault;
    paint.setBlendMode(static_cast<DlBlendMode>(blend_mode));
  }

  if (flags.applies_style()) {
    uint32_t style = uint_data[kStyleIndex];
    paint.setDrawStyle(static_cast<DlDrawStyle>(style));
  }

  if (flags.is_stroked(paint.getDrawStyle())) {
    float stroke_width = float_data[kStrokeWidthIndex];
    paint.setStrokeWidth(stroke_width);

    float stroke_miter_limit = float_data[kStrokeMiterLimitIndex];
    paint.setStrokeMiter(stroke_miter_limit + kStrokeMiterLimitDefault);

    uint32_t stroke_cap = uint_data[kStrokeCapIndex];
    paint.setStrokeCap(static_cast<DlStrokeCap>(stroke_cap));

    uint32_t stroke_join = uint_data[kStrokeJoinIndex];
    paint.setStrokeJoin(static_cast<DlStrokeJoin>(stroke_join));
  }

  if (flags.applies_color_filter()) {
    paint.setInvertColors(uint_data[kInvertColorIndex] != 0);
  }

  if (flags.applies_mask_filter()) {
    switch (uint_data[kMaskFilterIndex]) {
      case kNull:
        paint.setMaskFilter(nullptr);
        break;
      case kBlur:
        DlBlurStyle blur_style =
            static_cast<DlBlurStyle>(uint_data[kMaskFilterBlurStyleIndex]);
        double sigma = float_data[kMaskFilterSigmaIndex];
        paint.setMaskFilter(
            DlBlurMaskFilter::Make(blur_style, SafeNarrow(sigma)));
        break;
    }
  }

  return &paint;
}

void Paint::toDlPaint(DlPaint& paint, DlTileMode tile_mode) const {
  if (isNull()) {
    return;
  }
  FML_DCHECK(paint == DlPaint());

  tonic::DartByteData byte_data(paint_data_);
  FML_CHECK(byte_data.length_in_bytes() == kDataByteCount);

  const uint32_t* uint_data = static_cast<const uint32_t*>(byte_data.data());
  const float* float_data = static_cast<const float*>(byte_data.data());

  Dart_Handle values[kObjectCount];
  if (!Dart_IsNull(paint_objects_)) {
    FML_DCHECK(Dart_IsList(paint_objects_));
    intptr_t length = 0;
    Dart_ListLength(paint_objects_, &length);

    FML_CHECK(length == kObjectCount);
    if (Dart_IsError(
            Dart_ListGetRange(paint_objects_, 0, kObjectCount, values))) {
      return;
    }

    Dart_Handle shader = values[kShaderIndex];
    if (!Dart_IsNull(shader)) {
      if (Shader* decoded = tonic::DartConverter<Shader*>::FromDart(shader)) {
        auto sampling =
            ImageFilter::SamplingFromIndex(uint_data[kFilterQualityIndex]);
        paint.setColorSource(decoded->shader(sampling));
      }
    }

    Dart_Handle color_filter = values[kColorFilterIndex];
    if (!Dart_IsNull(color_filter)) {
      ColorFilter* decoded =
          tonic::DartConverter<ColorFilter*>::FromDart(color_filter);
      paint.setColorFilter(decoded->filter());
    }

    Dart_Handle image_filter = values[kImageFilterIndex];
    if (!Dart_IsNull(image_filter)) {
      ImageFilter* decoded =
          tonic::DartConverter<ImageFilter*>::FromDart(image_filter);
      paint.setImageFilter(decoded->filter(tile_mode));
    }
  }

  paint.setAntiAlias(uint_data[kIsAntiAliasIndex] == 0);

  paint.setColor(ReadColor(byte_data));

  uint32_t encoded_blend_mode = uint_data[kBlendModeIndex];
  uint32_t blend_mode = encoded_blend_mode ^ kBlendModeDefault;
  paint.setBlendMode(static_cast<DlBlendMode>(blend_mode));

  uint32_t style = uint_data[kStyleIndex];
  paint.setDrawStyle(static_cast<DlDrawStyle>(style));

  float stroke_width = float_data[kStrokeWidthIndex];
  paint.setStrokeWidth(stroke_width);

  float stroke_miter_limit = float_data[kStrokeMiterLimitIndex];
  paint.setStrokeMiter(stroke_miter_limit + kStrokeMiterLimitDefault);

  uint32_t stroke_cap = uint_data[kStrokeCapIndex];
  paint.setStrokeCap(static_cast<DlStrokeCap>(stroke_cap));

  uint32_t stroke_join = uint_data[kStrokeJoinIndex];
  paint.setStrokeJoin(static_cast<DlStrokeJoin>(stroke_join));

  paint.setInvertColors(uint_data[kInvertColorIndex] != 0);

  switch (uint_data[kMaskFilterIndex]) {
    case kNull:
      break;
    case kBlur:
      DlBlurStyle blur_style =
          static_cast<DlBlurStyle>(uint_data[kMaskFilterBlurStyleIndex]);
      float sigma = SafeNarrow(float_data[kMaskFilterSigmaIndex]);
      // Make could return a nullptr here if the values are NOP or
      // do not make sense. We could interpret that as if there was
      // no value passed from Dart at all (i.e. don't change the
      // setting in the paint object as in the kNull branch right
      // above here), but the maskfilter flag was actually set
      // indicating that the developer "tried" to set a mask, so we
      // should set the null value rather than do nothing.
      paint.setMaskFilter(DlBlurMaskFilter::Make(blur_style, sigma));
      break;
  }
}

}  // namespace flutter

namespace tonic {

flutter::Paint DartConverter<flutter::Paint>::FromArguments(
    Dart_NativeArguments args,
    int index,
    Dart_Handle& exception) {
  Dart_Handle paint_objects = Dart_GetNativeArgument(args, index);
  FML_DCHECK(!CheckAndHandleError(paint_objects));

  Dart_Handle paint_data = Dart_GetNativeArgument(args, index + 1);
  FML_DCHECK(!CheckAndHandleError(paint_data));

  return flutter::Paint(paint_objects, paint_data);
}

flutter::PaintData DartConverter<flutter::PaintData>::FromArguments(
    Dart_NativeArguments args,
    int index,
    Dart_Handle& exception) {
  return flutter::PaintData();
}

}  // namespace tonic

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/skia_conversions.h"
#include "flutter/display_list/dl_blend_mode.h"
#include "flutter/display_list/dl_color.h"
#include "third_party/skia/modules/skparagraph/include/Paragraph.h"

namespace impeller {
namespace skia_conversions {

static inline bool SkScalarsNearlyEqual(SkScalar a,
                                        SkScalar b,
                                        SkScalar c,
                                        SkScalar d) {
  return SkScalarNearlyEqual(a, b, kEhCloseEnough) &&
         SkScalarNearlyEqual(a, c, kEhCloseEnough) &&
         SkScalarNearlyEqual(a, d, kEhCloseEnough);
}

bool IsNearlySimpleRRect(const SkRRect& rr) {
  auto [xa, ya] = rr.radii(SkRRect::kUpperLeft_Corner);
  auto [xb, yb] = rr.radii(SkRRect::kLowerLeft_Corner);
  auto [xc, yc] = rr.radii(SkRRect::kUpperRight_Corner);
  auto [xd, yd] = rr.radii(SkRRect::kLowerRight_Corner);
  return SkScalarsNearlyEqual(xa, xb, xc, xd) &&
         SkScalarsNearlyEqual(ya, yb, yc, yd);
}

Rect ToRect(const SkRect& rect) {
  return Rect::MakeLTRB(rect.fLeft, rect.fTop, rect.fRight, rect.fBottom);
}

std::optional<Rect> ToRect(const SkRect* rect) {
  if (rect == nullptr) {
    return std::nullopt;
  }
  return Rect::MakeLTRB(rect->fLeft, rect->fTop, rect->fRight, rect->fBottom);
}

std::optional<const Rect> ToRect(const flutter::DlRect* rect) {
  if (rect == nullptr) {
    return std::nullopt;
  }
  return *rect;
}

std::vector<Rect> ToRects(const SkRect tex[], int count) {
  auto result = std::vector<Rect>();
  for (int i = 0; i < count; i++) {
    result.push_back(ToRect(tex[i]));
  }
  return result;
}

std::vector<Rect> ToRects(const flutter::DlRect tex[], int count) {
  auto result = std::vector<Rect>();
  for (int i = 0; i < count; i++) {
    result.push_back(tex[i]);
  }
  return result;
}

std::vector<Point> ToPoints(const SkPoint points[], int count) {
  std::vector<Point> result(count);
  for (auto i = 0; i < count; i++) {
    result[i] = ToPoint(points[i]);
  }
  return result;
}

std::vector<Point> ToPoints(const flutter::DlPoint points[], int count) {
  std::vector<Point> result(count);
  for (auto i = 0; i < count; i++) {
    result[i] = points[i];
  }
  return result;
}

Point ToPoint(const SkPoint& point) {
  return Point::MakeXY(point.fX, point.fY);
}

Size ToSize(const SkPoint& point) {
  return Size(point.fX, point.fY);
}

Color ToColor(const flutter::DlColor& color) {
  FML_DCHECK(color.getColorSpace() == flutter::DlColorSpace::kExtendedSRGB ||
             color.getColorSpace() == flutter::DlColorSpace::kSRGB);
  return {
      static_cast<Scalar>(color.getRedF()),    //
      static_cast<Scalar>(color.getGreenF()),  //
      static_cast<Scalar>(color.getBlueF()),   //
      static_cast<Scalar>(color.getAlphaF())   //
  };
}

Matrix ToRSXForm(const SkRSXform& form) {
  // clang-format off
    return Matrix{
      form.fSCos, form.fSSin, 0, 0,
     -form.fSSin, form.fSCos, 0, 0,
      0,          0,          1, 0,
      form.fTx,   form.fTy,   0, 1
    };
  // clang-format on
}

std::optional<impeller::PixelFormat> ToPixelFormat(SkColorType type) {
  switch (type) {
    case kRGBA_8888_SkColorType:
      return impeller::PixelFormat::kR8G8B8A8UNormInt;
    case kBGRA_8888_SkColorType:
      return impeller::PixelFormat::kB8G8R8A8UNormInt;
    case kRGBA_F16_SkColorType:
      return impeller::PixelFormat::kR16G16B16A16Float;
    case kBGR_101010x_XR_SkColorType:
      return impeller::PixelFormat::kB10G10R10XR;
    default:
      return std::nullopt;
  }
  return std::nullopt;
}

void ConvertStops(const flutter::DlGradientColorSourceBase* gradient,
                  std::vector<Color>& colors,
                  std::vector<float>& stops) {
  FML_DCHECK(gradient->stop_count() >= 2)
      << "stop_count:" << gradient->stop_count();

  auto* dl_colors = gradient->colors();
  auto* dl_stops = gradient->stops();
  if (dl_stops[0] != 0.0) {
    colors.emplace_back(skia_conversions::ToColor(dl_colors[0]));
    stops.emplace_back(0);
  }
  for (auto i = 0; i < gradient->stop_count(); i++) {
    colors.emplace_back(skia_conversions::ToColor(dl_colors[i]));
    stops.emplace_back(std::clamp(dl_stops[i], 0.0f, 1.0f));
  }
  if (dl_stops[gradient->stop_count() - 1] != 1.0) {
    colors.emplace_back(colors.back());
    stops.emplace_back(1.0);
  }
  for (auto i = 1; i < gradient->stop_count(); i++) {
    stops[i] = std::clamp(stops[i], stops[i - 1], stops[i]);
  }
}

impeller::SamplerDescriptor ToSamplerDescriptor(
    const flutter::DlImageSampling options) {
  impeller::SamplerDescriptor desc;
  switch (options) {
    case flutter::DlImageSampling::kNearestNeighbor:
      desc.min_filter = desc.mag_filter = impeller::MinMagFilter::kNearest;
      desc.mip_filter = impeller::MipFilter::kBase;
      desc.label = "Nearest Sampler";
      break;
    case flutter::DlImageSampling::kLinear:
      desc.min_filter = desc.mag_filter = impeller::MinMagFilter::kLinear;
      desc.mip_filter = impeller::MipFilter::kBase;
      desc.label = "Linear Sampler";
      break;
    case flutter::DlImageSampling::kCubic:
    case flutter::DlImageSampling::kMipmapLinear:
      desc.min_filter = desc.mag_filter = impeller::MinMagFilter::kLinear;
      desc.mip_filter = impeller::MipFilter::kLinear;
      desc.label = "Mipmap Linear Sampler";
      break;
  }
  return desc;
}

Matrix ToMatrix(const SkMatrix& m) {
  return Matrix{
      // clang-format off
      m[0], m[3], 0, m[6],
      m[1], m[4], 0, m[7],
      0,    0,    1, 0,
      m[2], m[5], 0, m[8],
      // clang-format on
  };
}

BlendMode ToBlendMode(flutter::DlBlendMode mode) {
  switch (mode) {
    case flutter::DlBlendMode::kClear:
      return BlendMode::kClear;
    case flutter::DlBlendMode::kSrc:
      return BlendMode::kSource;
    case flutter::DlBlendMode::kDst:
      return BlendMode::kDestination;
    case flutter::DlBlendMode::kSrcOver:
      return BlendMode::kSourceOver;
    case flutter::DlBlendMode::kDstOver:
      return BlendMode::kDestinationOver;
    case flutter::DlBlendMode::kSrcIn:
      return BlendMode::kSourceIn;
    case flutter::DlBlendMode::kDstIn:
      return BlendMode::kDestinationIn;
    case flutter::DlBlendMode::kSrcOut:
      return BlendMode::kSourceOut;
    case flutter::DlBlendMode::kDstOut:
      return BlendMode::kDestinationOut;
    case flutter::DlBlendMode::kSrcATop:
      return BlendMode::kSourceATop;
    case flutter::DlBlendMode::kDstATop:
      return BlendMode::kDestinationATop;
    case flutter::DlBlendMode::kXor:
      return BlendMode::kXor;
    case flutter::DlBlendMode::kPlus:
      return BlendMode::kPlus;
    case flutter::DlBlendMode::kModulate:
      return BlendMode::kModulate;
    case flutter::DlBlendMode::kScreen:
      return BlendMode::kScreen;
    case flutter::DlBlendMode::kOverlay:
      return BlendMode::kOverlay;
    case flutter::DlBlendMode::kDarken:
      return BlendMode::kDarken;
    case flutter::DlBlendMode::kLighten:
      return BlendMode::kLighten;
    case flutter::DlBlendMode::kColorDodge:
      return BlendMode::kColorDodge;
    case flutter::DlBlendMode::kColorBurn:
      return BlendMode::kColorBurn;
    case flutter::DlBlendMode::kHardLight:
      return BlendMode::kHardLight;
    case flutter::DlBlendMode::kSoftLight:
      return BlendMode::kSoftLight;
    case flutter::DlBlendMode::kDifference:
      return BlendMode::kDifference;
    case flutter::DlBlendMode::kExclusion:
      return BlendMode::kExclusion;
    case flutter::DlBlendMode::kMultiply:
      return BlendMode::kMultiply;
    case flutter::DlBlendMode::kHue:
      return BlendMode::kHue;
    case flutter::DlBlendMode::kSaturation:
      return BlendMode::kSaturation;
    case flutter::DlBlendMode::kColor:
      return BlendMode::kColor;
    case flutter::DlBlendMode::kLuminosity:
      return BlendMode::kLuminosity;
  }
  FML_UNREACHABLE();
}

}  // namespace skia_conversions
}  // namespace impeller

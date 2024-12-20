// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_FORMATS_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_FORMATS_H_

#include <vector>

#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/third_party/txt/src/txt/font_style.h"
#include "flutter/third_party/txt/src/txt/font_weight.h"
#include "flutter/third_party/txt/src/txt/paragraph_style.h"
#include "impeller/entity/entity.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/matrix.h"
#include "impeller/geometry/path_builder.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/rect.h"
#include "impeller/geometry/size.h"
#include "impeller/toolkit/interop/impeller.h"

namespace impeller::interop {

constexpr std::optional<SkRect> ToSkiaType(const ImpellerRect* rect) {
  if (!rect) {
    return std::nullopt;
  }
  return SkRect::MakeXYWH(rect->x, rect->y, rect->width, rect->height);
}

constexpr SkPoint ToSkiaType(const Point& point) {
  return SkPoint::Make(point.x, point.y);
}

constexpr SkVector ToSkiaVector(const Size& point) {
  return SkVector::Make(point.width, point.height);
}

constexpr SkRect ToSkiaType(const Rect& rect) {
  return SkRect::MakeXYWH(rect.GetX(),      //
                          rect.GetY(),      //
                          rect.GetWidth(),  //
                          rect.GetHeight()  //
  );
}

constexpr SkPathFillType ToSkiaType(FillType type) {
  switch (type) {
    case FillType::kNonZero:
      return SkPathFillType::kWinding;
    case FillType::kOdd:
      return SkPathFillType::kEvenOdd;
  }
  return SkPathFillType::kWinding;
}

constexpr SkIRect ToSkiaType(IRect rect) {
  return SkIRect::MakeXYWH(rect.GetX(),      //
                           rect.GetY(),      //
                           rect.GetWidth(),  //
                           rect.GetHeight()  //
  );
}

template <class SkiaType, class OtherType>
std::vector<SkiaType> ToSkiaType(const std::vector<OtherType>& other_vec) {
  std::vector<SkiaType> skia_vec;
  skia_vec.reserve(other_vec.size());
  for (const auto& other : other_vec) {
    skia_vec.emplace_back(ToSkiaType(other));
  }
  return skia_vec;
}

constexpr flutter::DlColor ToDisplayListType(Color color) {
  return flutter::DlColor::RGBA(color.red,    //
                                color.green,  //
                                color.blue,   //
                                color.alpha   //
  );
}

constexpr SkMatrix ToSkMatrix(const Matrix& matrix) {
  return SkM44::ColMajor(matrix.m).asM33();
}

template <class DlType, class OtherType>
std::vector<DlType> ToDisplayListType(const std::vector<OtherType>& other_vec) {
  std::vector<DlType> dl_vec;
  dl_vec.reserve(other_vec.size());
  for (const auto& other : other_vec) {
    dl_vec.emplace_back(ToDisplayListType(other));
  }
  return dl_vec;
}

constexpr flutter::DlImageSampling ToDisplayListType(
    ImpellerTextureSampling sampling) {
  switch (sampling) {
    case kImpellerTextureSamplingNearestNeighbor:
      return flutter::DlImageSampling::kNearestNeighbor;
    case kImpellerTextureSamplingLinear:
      return flutter::DlImageSampling::kLinear;
  }
  return flutter::DlImageSampling::kLinear;
}

constexpr flutter::DlBlurStyle ToDisplayListType(ImpellerBlurStyle style) {
  switch (style) {
    case kImpellerBlurStyleNormal:
      return flutter::DlBlurStyle::kNormal;
    case kImpellerBlurStyleSolid:
      return flutter::DlBlurStyle::kSolid;
    case kImpellerBlurStyleOuter:
      return flutter::DlBlurStyle::kOuter;
    case kImpellerBlurStyleInner:
      return flutter::DlBlurStyle::kInner;
  }
  return flutter::DlBlurStyle::kNormal;
}

constexpr flutter::DlBlendMode ToDisplayListType(BlendMode mode) {
  using Mode = flutter::DlBlendMode;
  switch (mode) {
    case BlendMode::kClear:
      return Mode::kClear;
    case BlendMode::kSource:
      return Mode::kSrc;
    case BlendMode::kDestination:
      return Mode::kDst;
    case BlendMode::kSourceOver:
      return Mode::kSrcOver;
    case BlendMode::kDestinationOver:
      return Mode::kDstOver;
    case BlendMode::kSourceIn:
      return Mode::kSrcIn;
    case BlendMode::kDestinationIn:
      return Mode::kDstIn;
    case BlendMode::kSourceOut:
      return Mode::kSrcOut;
    case BlendMode::kDestinationOut:
      return Mode::kDstOut;
    case BlendMode::kSourceATop:
      return Mode::kSrcATop;
    case BlendMode::kDestinationATop:
      return Mode::kDstATop;
    case BlendMode::kXor:
      return Mode::kXor;
    case BlendMode::kPlus:
      return Mode::kPlus;
    case BlendMode::kModulate:
      return Mode::kModulate;
    case BlendMode::kScreen:
      return Mode::kScreen;
    case BlendMode::kOverlay:
      return Mode::kOverlay;
    case BlendMode::kDarken:
      return Mode::kDarken;
    case BlendMode::kLighten:
      return Mode::kLighten;
    case BlendMode::kColorDodge:
      return Mode::kColorDodge;
    case BlendMode::kColorBurn:
      return Mode::kColorBurn;
    case BlendMode::kHardLight:
      return Mode::kHardLight;
    case BlendMode::kSoftLight:
      return Mode::kSoftLight;
    case BlendMode::kDifference:
      return Mode::kDifference;
    case BlendMode::kExclusion:
      return Mode::kExclusion;
    case BlendMode::kMultiply:
      return Mode::kMultiply;
    case BlendMode::kHue:
      return Mode::kHue;
    case BlendMode::kSaturation:
      return Mode::kSaturation;
    case BlendMode::kColor:
      return Mode::kColor;
    case BlendMode::kLuminosity:
      return Mode::kLuminosity;
  }
  return Mode::kSrcOver;
}

inline SkRRect ToSkiaType(const Rect& rect, const RoundingRadii& radii) {
  using Corner = SkRRect::Corner;
  SkVector sk_radii[4];
  sk_radii[Corner::kUpperLeft_Corner] = ToSkiaVector(radii.top_left);
  sk_radii[Corner::kUpperRight_Corner] = ToSkiaVector(radii.top_right);
  sk_radii[Corner::kLowerRight_Corner] = ToSkiaVector(radii.bottom_right);
  sk_radii[Corner::kLowerLeft_Corner] = ToSkiaVector(radii.bottom_left);
  SkRRect result;
  result.setRectRadii(ToSkiaType(rect), sk_radii);
  return result;
}

constexpr Matrix ToImpellerType(const ImpellerMatrix& m) {
  return Matrix(m.m[0], m.m[1], m.m[2], m.m[3],     //
                m.m[4], m.m[5], m.m[6], m.m[7],     //
                m.m[8], m.m[9], m.m[10], m.m[11],   //
                m.m[12], m.m[13], m.m[14], m.m[15]  //
  );
}

constexpr void FromImpellerType(const Matrix& from, ImpellerMatrix& to) {
  to.m[0] = from.m[0];
  to.m[1] = from.m[1];
  to.m[2] = from.m[2];
  to.m[3] = from.m[3];
  to.m[4] = from.m[4];
  to.m[5] = from.m[5];
  to.m[6] = from.m[6];
  to.m[7] = from.m[7];
  to.m[8] = from.m[8];
  to.m[9] = from.m[9];
  to.m[10] = from.m[10];
  to.m[11] = from.m[11];
  to.m[12] = from.m[12];
  to.m[13] = from.m[13];
  to.m[14] = from.m[14];
  to.m[15] = from.m[15];
}

constexpr Size ToImpellerType(const ImpellerSize& size) {
  return Size{size.width, size.height};
}

constexpr Point ToImpellerType(const ImpellerPoint& point) {
  return Point{point.x, point.y};
}

constexpr Size ToImpellerSize(const ImpellerPoint& point) {
  return Size{point.x, point.y};
}

constexpr Rect ToImpellerType(const ImpellerRect& rect) {
  return Rect::MakeXYWH(rect.x, rect.y, rect.width, rect.height);
}

constexpr flutter::DlTileMode ToDisplayListType(ImpellerTileMode mode) {
  switch (mode) {
    case kImpellerTileModeClamp:
      return flutter::DlTileMode::kClamp;
    case kImpellerTileModeRepeat:
      return flutter::DlTileMode::kRepeat;
    case kImpellerTileModeMirror:
      return flutter::DlTileMode::kMirror;
    case kImpellerTileModeDecal:
      return flutter::DlTileMode::kDecal;
  }
  return flutter::DlTileMode::kClamp;
}

constexpr RoundingRadii ToImpellerType(const ImpellerRoundingRadii& radii) {
  auto result = RoundingRadii{};
  result.top_left = ToImpellerSize(radii.top_left);
  result.bottom_left = ToImpellerSize(radii.bottom_left);
  result.top_right = ToImpellerSize(radii.top_right);
  result.bottom_right = ToImpellerSize(radii.bottom_right);
  return result;
}

constexpr FillType ToImpellerType(ImpellerFillType type) {
  switch (type) {
    case kImpellerFillTypeNonZero:
      return FillType::kNonZero;
    case kImpellerFillTypeOdd:
      return FillType::kOdd;
  }
  return FillType::kNonZero;
}

constexpr flutter::DlCanvas::ClipOp ToImpellerType(ImpellerClipOperation op) {
  switch (op) {
    case kImpellerClipOperationDifference:
      return flutter::DlCanvas::ClipOp::kDifference;
    case kImpellerClipOperationIntersect:
      return flutter::DlCanvas::ClipOp::kIntersect;
  }
  return flutter::DlCanvas::ClipOp::kDifference;
}

constexpr Color ToImpellerType(const ImpellerColor& color) {
  Color result;
  result.red = color.red;
  result.green = color.green;
  result.blue = color.blue;
  result.alpha = color.alpha;
  return result;
}

constexpr BlendMode ToImpellerType(ImpellerBlendMode mode) {
  switch (mode) {
    case kImpellerBlendModeClear:
      return BlendMode::kClear;
    case kImpellerBlendModeSource:
      return BlendMode::kSource;
    case kImpellerBlendModeDestination:
      return BlendMode::kDestination;
    case kImpellerBlendModeSourceOver:
      return BlendMode::kSourceOver;
    case kImpellerBlendModeDestinationOver:
      return BlendMode::kDestinationOver;
    case kImpellerBlendModeSourceIn:
      return BlendMode::kSourceIn;
    case kImpellerBlendModeDestinationIn:
      return BlendMode::kDestinationIn;
    case kImpellerBlendModeSourceOut:
      return BlendMode::kSourceOut;
    case kImpellerBlendModeDestinationOut:
      return BlendMode::kDestinationOut;
    case kImpellerBlendModeSourceATop:
      return BlendMode::kSourceATop;
    case kImpellerBlendModeDestinationATop:
      return BlendMode::kDestinationATop;
    case kImpellerBlendModeXor:
      return BlendMode::kXor;
    case kImpellerBlendModePlus:
      return BlendMode::kPlus;
    case kImpellerBlendModeModulate:
      return BlendMode::kModulate;
    case kImpellerBlendModeScreen:
      return BlendMode::kScreen;
    case kImpellerBlendModeOverlay:
      return BlendMode::kOverlay;
    case kImpellerBlendModeDarken:
      return BlendMode::kDarken;
    case kImpellerBlendModeLighten:
      return BlendMode::kLighten;
    case kImpellerBlendModeColorDodge:
      return BlendMode::kColorDodge;
    case kImpellerBlendModeColorBurn:
      return BlendMode::kColorBurn;
    case kImpellerBlendModeHardLight:
      return BlendMode::kHardLight;
    case kImpellerBlendModeSoftLight:
      return BlendMode::kSoftLight;
    case kImpellerBlendModeDifference:
      return BlendMode::kDifference;
    case kImpellerBlendModeExclusion:
      return BlendMode::kExclusion;
    case kImpellerBlendModeMultiply:
      return BlendMode::kMultiply;
    case kImpellerBlendModeHue:
      return BlendMode::kHue;
    case kImpellerBlendModeSaturation:
      return BlendMode::kSaturation;
    case kImpellerBlendModeColor:
      return BlendMode::kColor;
    case kImpellerBlendModeLuminosity:
      return BlendMode::kLuminosity;
  }
  return BlendMode::kSourceOver;
}

constexpr flutter::DlDrawStyle ToDisplayListType(ImpellerDrawStyle style) {
  switch (style) {
    case kImpellerDrawStyleFill:
      return flutter::DlDrawStyle::kFill;
    case kImpellerDrawStyleStroke:
      return flutter::DlDrawStyle::kStroke;
    case kImpellerDrawStyleStrokeAndFill:
      return flutter::DlDrawStyle::kStrokeAndFill;
  }
  return flutter::DlDrawStyle::kFill;
}

constexpr flutter::DlStrokeCap ToDisplayListType(ImpellerStrokeCap cap) {
  switch (cap) {
    case kImpellerStrokeCapButt:
      return flutter::DlStrokeCap::kButt;
    case kImpellerStrokeCapRound:
      return flutter::DlStrokeCap::kRound;
    case kImpellerStrokeCapSquare:
      return flutter::DlStrokeCap::kSquare;
  }
  return flutter::DlStrokeCap::kButt;
}

constexpr flutter::DlStrokeJoin ToDisplayListType(ImpellerStrokeJoin join) {
  switch (join) {
    case kImpellerStrokeJoinMiter:
      return flutter::DlStrokeJoin::kMiter;
    case kImpellerStrokeJoinRound:
      return flutter::DlStrokeJoin::kRound;
    case kImpellerStrokeJoinBevel:
      return flutter::DlStrokeJoin::kBevel;
  }
  return flutter::DlStrokeJoin::kMiter;
}

constexpr PixelFormat ToImpellerType(ImpellerPixelFormat format) {
  switch (format) {
    case kImpellerPixelFormatRGBA8888:
      return PixelFormat::kR8G8B8A8UNormInt;
  }
  return PixelFormat::kR8G8B8A8UNormInt;
}

constexpr ISize ToImpellerType(const ImpellerISize& size) {
  return ISize::MakeWH(size.width, size.height);
}

constexpr flutter::DlColorSpace ToDisplayListType(
    ImpellerColorSpace color_space) {
  switch (color_space) {
    case kImpellerColorSpaceSRGB:
      return flutter::DlColorSpace::kSRGB;
    case kImpellerColorSpaceExtendedSRGB:
      return flutter::DlColorSpace::kExtendedSRGB;
    case kImpellerColorSpaceDisplayP3:
      return flutter::DlColorSpace::kDisplayP3;
  }
  return flutter::DlColorSpace::kSRGB;
}

constexpr flutter::DlColor ToDisplayListType(ImpellerColor color) {
  return flutter::DlColor(color.alpha,                          //
                          color.red,                            //
                          color.green,                          //
                          color.blue,                           //
                          ToDisplayListType(color.color_space)  //
  );
}

constexpr txt::FontWeight ToTxtType(ImpellerFontWeight weight) {
  switch (weight) {
    case kImpellerFontWeight100:
      return txt::FontWeight::w100;
    case kImpellerFontWeight200:
      return txt::FontWeight::w200;
    case kImpellerFontWeight300:
      return txt::FontWeight::w300;
    case kImpellerFontWeight400:
      return txt::FontWeight::w400;
    case kImpellerFontWeight500:
      return txt::FontWeight::w500;
    case kImpellerFontWeight600:
      return txt::FontWeight::w600;
    case kImpellerFontWeight700:
      return txt::FontWeight::w700;
    case kImpellerFontWeight800:
      return txt::FontWeight::w800;
    case kImpellerFontWeight900:
      return txt::FontWeight::w900;
  }
  return txt::FontWeight::w400;
}

constexpr txt::FontStyle ToTxtType(ImpellerFontStyle style) {
  switch (style) {
    case kImpellerFontStyleNormal:
      return txt::FontStyle::normal;
    case kImpellerFontStyleItalic:
      return txt::FontStyle::italic;
  }
  return txt::FontStyle::normal;
}

constexpr txt::TextAlign ToTxtType(ImpellerTextAlignment align) {
  switch (align) {
    case kImpellerTextAlignmentLeft:
      return txt::TextAlign::left;
    case kImpellerTextAlignmentRight:
      return txt::TextAlign::right;
    case kImpellerTextAlignmentCenter:
      return txt::TextAlign::center;
    case kImpellerTextAlignmentJustify:
      return txt::TextAlign::justify;
    case kImpellerTextAlignmentStart:
      return txt::TextAlign::start;
    case kImpellerTextAlignmentEnd:
      return txt::TextAlign::end;
  }
  return txt::TextAlign::left;
}

constexpr txt::TextDirection ToTxtType(ImpellerTextDirection direction) {
  switch (direction) {
    case kImpellerTextDirectionRTL:
      return txt::TextDirection::rtl;
    case kImpellerTextDirectionLTR:
      return txt::TextDirection::ltr;
  }
  return txt::TextDirection::ltr;
}

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_FORMATS_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/display_list_testing.h"

#include <cstdint>
#include <iomanip>

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/effects/dl_color_filters.h"
#include "flutter/display_list/effects/dl_color_sources.h"
#include "flutter/display_list/effects/dl_image_filters.h"

namespace flutter::testing {

// clang-format off
bool DisplayListsEQ_Verbose(const DisplayList* a, const DisplayList* b) {
  if (a->Equals(b)) {
    return true;
  }
  FML_LOG(ERROR) << std::endl
                 << std::endl
                 << *a << std::endl
                 << "not identical to ..." << std::endl
                 << std::endl
                 << *b;
  return false;
}

bool DisplayListsNE_Verbose(const DisplayList* a, const DisplayList* b) {
  if (a->Equals(b)) {
    FML_LOG(ERROR) << std::endl
                   << "DisplayLists are both the same:" << std::endl
                   << *a;
    return false;
  }
  return true;
}

}  // namespace flutter::testing

namespace std {

using DisplayList = flutter::DisplayList;
using DlColor = flutter::DlColor;
using DlPaint = flutter::DlPaint;
using DlCanvas = flutter::DlCanvas;
using DlImage = flutter::DlImage;
using DlDrawStyle = flutter::DlDrawStyle;
using DlBlendMode = flutter::DlBlendMode;
using DlStrokeCap = flutter::DlStrokeCap;
using DlStrokeJoin = flutter::DlStrokeJoin;
using DlBlurStyle = flutter::DlBlurStyle;
using DlFilterMode = flutter::DlFilterMode;
using DlVertexMode = flutter::DlVertexMode;
using DlTileMode = flutter::DlTileMode;
using DlImageSampling = flutter::DlImageSampling;
using SaveLayerOptions = flutter::SaveLayerOptions;
using DisplayListOpType = flutter::DisplayListOpType;
using DisplayListOpCategory = flutter::DisplayListOpCategory;
using DlPath = flutter::DlPath;

using DisplayListStreamDispatcher = flutter::testing::DisplayListStreamDispatcher;

std::ostream& operator<<(std::ostream& os,
                         const DisplayList& display_list) {
  DisplayListStreamDispatcher dispatcher(os);
  os << std::boolalpha;
  os << std::setprecision(std::numeric_limits<long double>::digits10 + 1);
  os << "DisplayList {" << std::endl;
  display_list.Dispatch(dispatcher);
  os << "}" << std::endl;
  return os;
}

std::ostream& operator<<(std::ostream& os, const DlPaint& paint) {
  os << "DlPaint("
     << "isaa: " << paint.isAntiAlias() << ", "
     << paint.getColor() << ", "
     << paint.getBlendMode() << ", "
     << paint.getDrawStyle();
  if (paint.getDrawStyle() != DlDrawStyle::kFill) {
    os << ", width: " << paint.getStrokeWidth()
       << ", miter: " << paint.getStrokeMiter()
       << ", " << paint.getStrokeCap()
       << ", " << paint.getStrokeJoin();
  }
  if (paint.getColorSource()) {
    os << ", " << paint.getColorSource();
  }
  if (paint.getColorFilter()) {
    os << ", " << paint.getColorFilter();
  }
  if (paint.getImageFilter()) {
    os << ", " << paint.getImageFilter();
  }
  if (paint.getMaskFilter()) {
    os << ", " << paint.getMaskFilter();
  }
  if (paint.isInvertColors()) {
    os << ", invertColors: " << paint.isInvertColors();
  }
  return os << ")";
}

#define DLT_OSTREAM_CASE(enum_name, value_name) \
  case enum_name::k##value_name: return os << #enum_name "::k" #value_name

std::ostream& operator<<(std::ostream& os, const DlBlendMode& mode) {
  switch (mode) {
    DLT_OSTREAM_CASE(DlBlendMode, Clear);
    DLT_OSTREAM_CASE(DlBlendMode, Src);
    DLT_OSTREAM_CASE(DlBlendMode, Dst);
    DLT_OSTREAM_CASE(DlBlendMode, SrcOver);
    DLT_OSTREAM_CASE(DlBlendMode, DstOver);
    DLT_OSTREAM_CASE(DlBlendMode, SrcIn);
    DLT_OSTREAM_CASE(DlBlendMode, DstIn);
    DLT_OSTREAM_CASE(DlBlendMode, SrcOut);
    DLT_OSTREAM_CASE(DlBlendMode, DstOut);
    DLT_OSTREAM_CASE(DlBlendMode, SrcATop);
    DLT_OSTREAM_CASE(DlBlendMode, DstATop);
    DLT_OSTREAM_CASE(DlBlendMode, Xor);
    DLT_OSTREAM_CASE(DlBlendMode, Plus);
    DLT_OSTREAM_CASE(DlBlendMode, Modulate);
    DLT_OSTREAM_CASE(DlBlendMode, Screen);

    DLT_OSTREAM_CASE(DlBlendMode, Overlay);
    DLT_OSTREAM_CASE(DlBlendMode, Darken);
    DLT_OSTREAM_CASE(DlBlendMode, Lighten);
    DLT_OSTREAM_CASE(DlBlendMode, ColorDodge);
    DLT_OSTREAM_CASE(DlBlendMode, ColorBurn);
    DLT_OSTREAM_CASE(DlBlendMode, HardLight);
    DLT_OSTREAM_CASE(DlBlendMode, SoftLight);
    DLT_OSTREAM_CASE(DlBlendMode, Difference);
    DLT_OSTREAM_CASE(DlBlendMode, Exclusion);
    DLT_OSTREAM_CASE(DlBlendMode, Multiply);

    DLT_OSTREAM_CASE(DlBlendMode, Hue);
    DLT_OSTREAM_CASE(DlBlendMode, Saturation);
    DLT_OSTREAM_CASE(DlBlendMode, Color);
    DLT_OSTREAM_CASE(DlBlendMode, Luminosity);
  }
  // Not a valid enum, should never happen, but in case we encounter bad data.
  return os << "DlBlendMode::????";
}

extern std::ostream& operator<<(std::ostream& os,
                                const flutter::DisplayListOpType& type) {
  switch (type) {
#define DLT_OP_TYPE_CASE(V) DLT_OSTREAM_CASE(DisplayListOpType, V);
    FOR_EACH_DISPLAY_LIST_OP(DLT_OP_TYPE_CASE)
    DLT_OP_TYPE_CASE(InvalidOp)

#undef DLT_OP_TYPE_CASE
  }
  // Not a valid enum, should never happen, but in case we encounter bad data.
  return os << "DisplayListOpType::???";
}

extern std::ostream& operator<<(
    std::ostream& os, const flutter::DisplayListOpCategory& category) {
  switch (category) {
    DLT_OSTREAM_CASE(DisplayListOpCategory, Attribute);
    DLT_OSTREAM_CASE(DisplayListOpCategory, Transform);
    DLT_OSTREAM_CASE(DisplayListOpCategory, Clip);
    DLT_OSTREAM_CASE(DisplayListOpCategory, Save);
    DLT_OSTREAM_CASE(DisplayListOpCategory, SaveLayer);
    DLT_OSTREAM_CASE(DisplayListOpCategory, Restore);
    DLT_OSTREAM_CASE(DisplayListOpCategory, Rendering);
    DLT_OSTREAM_CASE(DisplayListOpCategory, SubDisplayList);
    DLT_OSTREAM_CASE(DisplayListOpCategory, InvalidCategory);
  }
  // Not a valid enum, should never happen, but in case we encounter bad data.
  return os << "DisplayListOpCategory::???";
}

#undef DLT_OSTREAM_CASE

std::ostream& operator<<(std::ostream& os, const SaveLayerOptions& options) {
  return os << "SaveLayerOptions("
            << "renders_with_attributes: " << options.renders_with_attributes()
            << ", "
            << "can_distribute_opacity: " << options.can_distribute_opacity()
            << ", "
            << "contains_backdrop: " << options.contains_backdrop_filter()
            << ", "
            << "is_unbounded: " << options.content_is_unbounded()
            << ")";
}

static std::ostream& operator<<(std::ostream& os, const SkRect& rect) {
  return os << "SkRect("
            << "left: " << rect.fLeft << ", "
            << "top: " << rect.fTop << ", "
            << "right: " << rect.fRight << ", "
            << "bottom: " << rect.fBottom
            << ")";
}

extern std::ostream& operator<<(std::ostream& os, const DlPath& path) {
  return os << "DlPath("
            << "bounds: " << path.GetSkBounds()
            // should iterate over verbs and coordinates...
            << ")";
}

static std::ostream& operator<<(std::ostream& os, const SkRSXform& xform) {
  return os << "SkRSXform("
            << "scos: " << xform.fSCos << ", "
            << "ssin: " << xform.fSSin << ", "
            << "tx: " << xform.fTx << ", "
            << "ty: " << xform.fTy << ")";
}

std::ostream& operator<<(std::ostream& os, const DlCanvas::ClipOp& op) {
  switch (op) {
    case DlCanvas::ClipOp::kDifference: return os << "ClipOp::kDifference";
    case DlCanvas::ClipOp::kIntersect:  return os << "ClipOp::kIntersect";
  }
}

std::ostream& operator<<(std::ostream& os, const DlCanvas::SrcRectConstraint& constraint) {
  switch (constraint) {
    case DlCanvas::SrcRectConstraint::kFast:
      return os << "SrcRectConstraint::kFast";
    case DlCanvas::SrcRectConstraint::kStrict:
      return os << "SrcRectConstraint::kStrict";
  }
}

std::ostream& operator<<(std::ostream& os, const DlStrokeCap& cap) {
  switch (cap) {
    case DlStrokeCap::kButt:   return os << "Cap::kButt";
    case DlStrokeCap::kRound:  return os << "Cap::kRound";
    case DlStrokeCap::kSquare: return os << "Cap::kSquare";
  }
}

std::ostream& operator<<(std::ostream& os, const DlStrokeJoin& join) {
  switch (join) {
    case DlStrokeJoin::kMiter: return os << "Join::kMiter";
    case DlStrokeJoin::kRound: return os << "Join::kRound";
    case DlStrokeJoin::kBevel: return os << "Join::kBevel";
  }
}

std::ostream& operator<<(std::ostream& os, const DlDrawStyle& style) {
  switch (style) {
    case DlDrawStyle::kFill:          return os << "Style::kFill";
    case DlDrawStyle::kStroke:        return os << "Style::kStroke";
    case DlDrawStyle::kStrokeAndFill: return os << "Style::kStrokeAnFill";
  }
}

std::ostream& operator<<(std::ostream& os, const DlBlurStyle& style) {
  switch (style) {
    case DlBlurStyle::kNormal: return os << "BlurStyle::kNormal";
    case DlBlurStyle::kSolid:  return os << "BlurStyle::kSolid";
    case DlBlurStyle::kOuter:  return os << "BlurStyle::kOuter";
    case DlBlurStyle::kInner:  return os << "BlurStyle::kInner";
  }
}

std::ostream& operator<<(std::ostream& os, const DlCanvas::PointMode& mode) {
  switch (mode) {
    case DlCanvas::PointMode::kPoints:  return os << "PointMode::kPoints";
    case DlCanvas::PointMode::kLines:   return os << "PointMode::kLines";
    case DlCanvas::PointMode::kPolygon: return os << "PointMode::kPolygon";
  }
}

std::ostream& operator<<(std::ostream& os, const DlFilterMode& mode) {
  switch (mode) {
    case DlFilterMode::kNearest: return os << "FilterMode::kNearest";
    case DlFilterMode::kLinear:  return os << "FilterMode::kLinear";

    default: return os << "FilterMode::????";
  }
}

std::ostream& operator<<(std::ostream& os, const DlColor& color) {
  const char* color_space;
  switch(color.getColorSpace()) {
    case flutter::DlColorSpace::kSRGB:
      color_space = "srgb";
      break;
    case flutter::DlColorSpace::kExtendedSRGB:
      color_space = "srgb_xr";
      break;
    case flutter::DlColorSpace::kDisplayP3:
      color_space = "p3";
      break;
  }
  return os << "DlColor(" << //
    color.getAlphaF() << ", " << //
    color.getRedF() << ", " << //
    color.getGreenF() << ", " << //
    color.getBlueF() << ", " << //
    color_space << ")";
}

std::ostream& operator<<(std::ostream& os, DlImageSampling sampling) {
  switch (sampling) {
    case DlImageSampling::kNearestNeighbor: {
      return os << "NearestSampling";
    }
    case DlImageSampling::kLinear: {
      return os << "LinearSampling";
    }
    case DlImageSampling::kMipmapLinear: {
      return os << "MipmapSampling";
    }
    case DlImageSampling::kCubic: {
      return os << "CubicSampling";
    }
  }
}

static std::ostream& operator<<(std::ostream& os, const SkTextBlob* blob) {
  if (blob == nullptr) {
    return os << "no text";
  }
  return os << "&SkTextBlob(ID: " << blob->uniqueID() << ", " << blob->bounds() << ")";
}

static std::ostream& operator<<(std::ostream& os,
                                const impeller::TextFrame* frame) {
  if (frame == nullptr) {
    return os << "no text";
  }
  auto bounds = frame->GetBounds();
  return os << "&TextFrame("
            << bounds.GetLeft() << ", " << bounds.GetTop() << " => "
            << bounds.GetRight() << ", " << bounds.GetBottom() << ")";
}

std::ostream& operator<<(std::ostream& os, const DlVertexMode& mode) {
  switch (mode) {
    case DlVertexMode::kTriangles:     return os << "VertexMode::kTriangles";
    case DlVertexMode::kTriangleStrip: return os << "VertexMode::kTriangleStrip";
    case DlVertexMode::kTriangleFan:   return os << "VertexMode::kTriangleFan";

    default: return os << "VertexMode::????";
  }
}

std::ostream& operator<<(std::ostream& os, const DlTileMode& mode) {
  switch (mode) {
    case DlTileMode::kClamp: return os << "TileMode::kClamp";
    case DlTileMode::kRepeat: return os << "TileMode::kRepeat";
    case DlTileMode::kMirror: return os << "TileMode::kMirror";
    case DlTileMode::kDecal: return os << "TileMode::kDecal";

    default: return os << "TileMode::????";
  }
}

std::ostream& operator<<(std::ostream& os, const DlImage* image) {
  if (image == nullptr) {
    return os << "null image";
  }
  os << "&DlImage(" << image->width() << " x " << image->height() << ", ";
  if (image->skia_image()) {
    os << "skia(" << image->skia_image().get() << "), ";
  }
  if (image->impeller_texture()) {
    os << "impeller(" << image->impeller_texture().get() << "), ";
  }
  return os << "isTextureBacked: " << image->isTextureBacked() << ")";
}

std::ostream& operator<<(std::ostream& os,
                         const flutter::DlImageFilter& filter) {
  DisplayListStreamDispatcher(os, 0).out(filter);
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const flutter::DlColorFilter& filter) {
  DisplayListStreamDispatcher(os, 0).out(filter);
  return os;
}

}  // namespace std

namespace flutter::testing {

std::ostream& DisplayListStreamDispatcher::startl() {
  for (int i = 0; i < cur_indent_; i++) {
    os_ << " ";
  }
  return os_;
}

template <class T>
std::ostream& DisplayListStreamDispatcher::out_array(std::string name,  // NOLINT(performance-unnecessary-value-param)
                                                     int count,
                                                     const T array[]) {
  if (array == nullptr || count < 0) {
    return os_ << "no " << name;
  }
  os_ << name << "[" << count << "] = [" << std::endl;
  indent();
  indent();
  for (int i = 0; i < count; i++) {
    startl() << array[i] << "," << std::endl;
  }
  outdent();
  startl() << "]";
  outdent();
  return os_;
}

void DisplayListStreamDispatcher::setAntiAlias(bool aa) {
  startl() << "setAntiAlias(" << aa << ");" << std::endl;
}
void DisplayListStreamDispatcher::setDrawStyle(DlDrawStyle style) {
  startl() << "setStyle(" << style << ");" << std::endl;
}
void DisplayListStreamDispatcher::setColor(DlColor color) {
  startl() << "setColor(" << color << ");" << std::endl;
}
void DisplayListStreamDispatcher::setStrokeWidth(DlScalar width) {
  startl() << "setStrokeWidth(" << width << ");" << std::endl;
}
void DisplayListStreamDispatcher::setStrokeMiter(DlScalar limit) {
  startl() << "setStrokeMiter(" << limit << ");" << std::endl;
}
void DisplayListStreamDispatcher::setStrokeCap(DlStrokeCap cap) {
  startl() << "setStrokeCap(" << cap << ");" << std::endl;
}
void DisplayListStreamDispatcher::setStrokeJoin(DlStrokeJoin join) {
  startl() << "setStrokeJoin(" << join << ");" << std::endl;
}
void DisplayListStreamDispatcher::setColorSource(const DlColorSource* source) {
  if (source == nullptr) {
    startl() << "setColorSource(no ColorSource);" << std::endl;
    return;
  }
  startl() << "setColorSource(";
  switch (source->type()) {
    case DlColorSourceType::kImage: {
      const DlImageColorSource* image_src = source->asImage();
      FML_DCHECK(image_src);
      os_ << "DlImageColorSource(image: " << image_src->image()
                           << ", hMode: " << image_src->horizontal_tile_mode()
                           << ", vMode: " << image_src->vertical_tile_mode()
                           << ", " << image_src->sampling()
                           << ", " << image_src->matrix_ptr()
                           << ")";
      break;
    }
    case DlColorSourceType::kLinearGradient: {
      const DlLinearGradientColorSource* linear_src = source->asLinearGradient();
      FML_DCHECK(linear_src);
      os_ << "DlLinearGradientSource("
                                 << "start: " << linear_src->start_point()
                                 << ", end: " << linear_src->end_point() << ", ";
                                 out_array("colors", linear_src->stop_count(), linear_src->colors()) << ", ";
                                 out_array("stops", linear_src->stop_count(), linear_src->stops()) << ", "
                                 << linear_src->tile_mode() << ", " << linear_src->matrix_ptr() << ")";
      break;
    }
    case DlColorSourceType::kRadialGradient: {
      const DlRadialGradientColorSource* radial_src = source->asRadialGradient();
      FML_DCHECK(radial_src);
      os_ << "DlRadialGradientSource("
                                 << "center: " << radial_src->center()
                                 << ", radius: " << radial_src->radius() << ", ";
                                 out_array("colors", radial_src->stop_count(), radial_src->colors()) << ", ";
                                 out_array("stops", radial_src->stop_count(), radial_src->stops()) << ", "
                                 << radial_src->tile_mode() << ", " << radial_src->matrix_ptr() << ")";
      break;
    }
    case DlColorSourceType::kConicalGradient: {
      const DlConicalGradientColorSource* conical_src = source->asConicalGradient();
      FML_DCHECK(conical_src);
      os_ << "DlConicalGradientColorSource("
                                 << "start center: " << conical_src->start_center()
                                 << ", start radius: " << conical_src->start_radius()
                                 << ", end center: " << conical_src->end_center()
                                 << ", end radius: " << conical_src->end_radius() << ", ";
                                 out_array("colors", conical_src->stop_count(), conical_src->colors()) << ", ";
                                 out_array("stops", conical_src->stop_count(), conical_src->stops()) << ", "
                                 << conical_src->tile_mode() << ", " << conical_src->matrix_ptr() << ")";
      break;
    }
    case DlColorSourceType::kSweepGradient: {
      const DlSweepGradientColorSource* sweep_src = source->asSweepGradient();
      FML_DCHECK(sweep_src);
      os_ << "DlSweepGradientColorSource("
                                 << "center: " << sweep_src->center()
                                 << ", start: " << sweep_src->start() << ", "
                                 << ", end: " << sweep_src->end() << ", ";
                                 out_array("colors", sweep_src->stop_count(), sweep_src->colors()) << ", ";
                                 out_array("stops", sweep_src->stop_count(), sweep_src->stops()) << ", "
                                 << sweep_src->tile_mode() << ", " << sweep_src->matrix_ptr() << ")";
      break;
    }
    default:
      os_ << "?DlUnknownColorSource?()";
      break;
  }
  os_ << ");" << std::endl;
}
void DisplayListStreamDispatcher::out(const DlColorFilter& filter) {
  switch (filter.type()) {
    case DlColorFilterType::kBlend: {
      const DlBlendColorFilter* blend = filter.asBlend();
      FML_DCHECK(blend);
      os_ << "DlBlendColorFilter(" << blend->color() << ", "
                                   << static_cast<int>(blend->mode()) << ")";
      break;
    }
    case DlColorFilterType::kMatrix: {
      const DlMatrixColorFilter* matrix = filter.asMatrix();
      FML_DCHECK(matrix);
      float values[20];
      matrix->get_matrix(values);
      os_ << "DlMatrixColorFilter(matrix[20] = [" << std::endl;
      indent();
      for (int i = 0; i < 20; i += 5) {
        startl() << values[i] << ", "
                 << values[i+1] << ", "
                 << values[i+2] << ", "
                 << values[i+3] << ", "
                 << values[i+4] << ","
                 << std::endl;
      }
      outdent();
      startl() << "]";
      break;
    }
    case DlColorFilterType::kSrgbToLinearGamma: {
      os_ << "DlSrgbToLinearGammaColorFilter()";
      break;
    }
    case DlColorFilterType::kLinearToSrgbGamma: {
      os_ << "DlLinearToSrgbGammaColorFilter()";
      break;
    }
    default:
      os_ << "?DlUnknownColorFilter?()";
      break;
  }
}
void DisplayListStreamDispatcher::out(const DlColorFilter* filter) {
  if (filter == nullptr) {
    os_ << "no ColorFilter";
  } else {
    os_ << "&";
    out(*filter);
  }
}
void DisplayListStreamDispatcher::setColorFilter(const DlColorFilter* filter) {
  startl() << "setColorFilter(";
  out(filter);
  os_ << ");" << std::endl;
}
void DisplayListStreamDispatcher::setInvertColors(bool invert) {
  startl() << "setInvertColors(" << invert << ");" << std::endl;
}
void DisplayListStreamDispatcher::setBlendMode(DlBlendMode mode) {
  startl() << "setBlendMode(" << mode << ");" << std::endl;
}
void DisplayListStreamDispatcher::setMaskFilter(const DlMaskFilter* filter) {
  if (filter == nullptr) {
    startl() << "setMaskFilter(no MaskFilter);" << std::endl;
    return;
  }
  startl() << "setMaskFilter(";
  switch (filter->type()) {
    case DlMaskFilterType::kBlur: {
      const DlBlurMaskFilter* blur = filter->asBlur();
      FML_DCHECK(blur);
      os_ << "DlMaskFilter(" << blur->style() << ", " << blur->sigma() << ")";
      break;
    }
    default:
      os_ << "?DlUnknownMaskFilter?()";
      break;
  }
  os_ << ");" << std::endl;
}
void DisplayListStreamDispatcher::out(const DlImageFilter& filter) {
  switch (filter.type()) {
    case DlImageFilterType::kBlur: {
      const DlBlurImageFilter* blur = filter.asBlur();
      FML_DCHECK(blur);
      os_ << "DlBlurImageFilter(" << blur->sigma_x() << ", "
                                  << blur->sigma_y() << ", "
                                  << blur->tile_mode() << ")";
      break;
    }
    case DlImageFilterType::kDilate: {
      const DlDilateImageFilter* dilate = filter.asDilate();
      FML_DCHECK(dilate);
      os_ << "DlDilateImageFilter(" << dilate->radius_x() << ", " << dilate->radius_y() << ")";
      break;
    }
    case DlImageFilterType::kErode: {
      const DlErodeImageFilter* erode = filter.asErode();
      FML_DCHECK(erode);
      os_ << "DlErodeImageFilter(" << erode->radius_x() << ", " << erode->radius_y() << ")";
      break;
    }
    case DlImageFilterType::kMatrix: {
      const DlMatrixImageFilter* matrix = filter.asMatrix();
      FML_DCHECK(matrix);
      os_ << "DlMatrixImageFilter(" << matrix->matrix() << ", " << matrix->sampling() << ")";
      break;
    }
    case DlImageFilterType::kCompose: {
      const DlComposeImageFilter* compose = filter.asCompose();
      FML_DCHECK(compose);
      os_ << "DlComposeImageFilter(" << std::endl;
      indent();
      startl() << "outer: ";
      indent(7);
      out(compose->outer().get());
      os_ << "," << std::endl;
      outdent(7);
      startl() << "inner: ";
      indent(7);
      out(compose->inner().get());
      os_ << std::endl;
      outdent(7);
      outdent();
      startl() << ")";
      break;
    }
    case DlImageFilterType::kColorFilter: {
      const DlColorFilterImageFilter* color_filter = filter.asColorFilter();
      FML_DCHECK(color_filter);
      os_ << "DlColorFilterImageFilter(";
      out(*color_filter->color_filter());
      os_ << ")";
      break;
    }
    case DlImageFilterType::kLocalMatrix: {
      const DlLocalMatrixImageFilter* local_matrix = filter.asLocalMatrix();
      FML_DCHECK(local_matrix);
      os_ << "DlLocalMatrixImageFilter(" << local_matrix->matrix();
      os_ << "," << std::endl;
      indent(25);
      startl() << "filter: ";
      out(local_matrix->image_filter().get());
      os_ << std::endl;
      outdent(25);
      startl() << ")";
      break;
    }
    case flutter::DlImageFilterType::kRuntimeEffect: {
      [[maybe_unused]] const DlRuntimeEffectImageFilter* runtime_effect = filter.asRuntimeEffectFilter();
      FML_DCHECK(runtime_effect);
      os_ << "DlRuntimeEffectImageFilter(";
      os_ << runtime_effect->samplers().size() << " samplers, ";
      os_ << runtime_effect->uniform_data()->size() << " uniform bytes)";
      break;
    }
  }
}
void DisplayListStreamDispatcher::out(const DlImageFilter* filter) {
  if (filter == nullptr) {
    os_ << "no ImageFilter";
  } else {
    os_ << "&";
    indent(1);
    out(*filter);
    outdent(1);
  }
}
void DisplayListStreamDispatcher::setImageFilter(const DlImageFilter* filter) {
  startl() << "setImageFilter(";
  indent(15);
  out(filter);
  outdent(15);
  os_ << ");" << std::endl;
}
void DisplayListStreamDispatcher::save() {
  startl() << "save();" << std::endl;
  startl() << "{" << std::endl;
  indent();
}
void DisplayListStreamDispatcher::saveLayer(const DlRect& bounds,
                                            const SaveLayerOptions options,
                                            const DlImageFilter* backdrop,
                                            std::optional<int64_t> backdrop_id) {
  startl() << "saveLayer(" << bounds << ", " << options;
  if (backdrop) {
    os_ << "," << std::endl;
    indent(10);
    if (backdrop_id.has_value()) {
      startl() << "backdrop: " << backdrop_id.value() << ", ";
    } else {
      startl() << "backdrop: (no id), ";
    }
    out(backdrop);
    outdent(10);
  } else {
    os_ << ", no backdrop";
  }
  os_ << ");" << std::endl;
  startl() << "{" << std::endl;
  indent();
}
void DisplayListStreamDispatcher::restore() {
  outdent();
  startl() << "}" << std::endl;
  startl() << "restore();" << std::endl;
}

void DisplayListStreamDispatcher::translate(DlScalar tx, DlScalar ty) {
  startl() << "translate(" << tx << ", " << ty << ");" << std::endl;
}
void DisplayListStreamDispatcher::scale(DlScalar sx, DlScalar sy) {
  startl() << "scale(" << sx << ", " << sy << ");" << std::endl;
}
void DisplayListStreamDispatcher::rotate(DlScalar degrees) {
  startl() << "rotate(" << degrees << ");" << std::endl;
}
void DisplayListStreamDispatcher::skew(DlScalar sx, DlScalar sy) {
  startl() << "skew(" << sx << ", " << sy << ");" << std::endl;
}
void DisplayListStreamDispatcher::transform2DAffine(
    DlScalar mxx, DlScalar mxy, DlScalar mxt,
    DlScalar myx, DlScalar myy, DlScalar myt) {
  startl() << "transform2DAffine(" << std::endl;
  indent();
  {
    startl()
        << "[" << mxx << ", " << mxy << ", " << mxt << "], "
        << std::endl;
    startl()
        << "[" << myx << ", " << myy  << ", " << myt << "], "
        << std::endl;
  }
  outdent();
  startl() << ");" << std::endl;
}
void DisplayListStreamDispatcher::transformFullPerspective(
    DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
    DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
    DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
    DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) {
  startl() << "transformFullPerspective(" << std::endl;
  indent();
  {
    startl()
        << "[" << mxx << ", " << mxy << ", " << mxz << ", " << mxt << "], "
        << std::endl;
    startl()
        << "[" << myx << ", " << myy << ", " << myz << ", " << myt << "], "
        << std::endl;
    startl()
        << "[" << mzx << ", " << mzy << ", " << mzz << ", " << mzt << "], "
        << std::endl;
    startl()
        << "[" << mwx << ", " << mwy << ", " << mwz << ", " << mwt << "]"
        << std::endl;
  }
  outdent();
  startl() << ");" << std::endl;
}
void DisplayListStreamDispatcher::transformReset() {
  startl() << "transformReset();" << std::endl;
}

void DisplayListStreamDispatcher::clipRect(const DlRect& rect, ClipOp clip_op,
                                           bool is_aa) {
  startl() << "clipRect("
           << rect << ", "
           << clip_op << ", "
           << "isaa: " << is_aa
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::clipOval(const DlRect& bounds, ClipOp clip_op,
                                           bool is_aa) {
  startl() << "clipOval("
           << bounds << ", "
           << clip_op << ", "
           << "isaa: " << is_aa
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::clipRoundRect(const DlRoundRect& rrect,
                                                ClipOp clip_op,
                                                bool is_aa) {
  startl() << "clipRRect("
           << rrect << ", "
           << clip_op << ", "
           << "isaa: " << is_aa
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::clipPath(const DlPath& path, ClipOp clip_op,
                                           bool is_aa) {
  startl() << "clipPath("
           << path << ", "
           << clip_op << ", "
           << "isaa: " << is_aa
           << ");" << std::endl;
}

void DisplayListStreamDispatcher::drawColor(DlColor color, DlBlendMode mode) {
  startl() << "drawColor("
           << color << ", "
           << mode
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawPaint() {
  startl() << "drawPaint();" << std::endl;
}
void DisplayListStreamDispatcher::drawLine(const DlPoint& p0,
                                           const DlPoint& p1) {
  startl() << "drawLine(" << p0 << ", " << p1 << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawDashedLine(const DlPoint& p0,
                                                 const DlPoint& p1,
                                                 DlScalar on_length,
                                                 DlScalar off_length) {
  startl() << "drawDashedLine("
           << p0 << ", "
           << p1 << ", "
           << on_length << ", "
           << off_length
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawRect(const DlRect& rect) {
  startl() << "drawRect(" << rect << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawOval(const DlRect& bounds) {
  startl() << "drawOval(" << bounds << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawCircle(const DlPoint& center,
                                             DlScalar radius) {
  startl() << "drawCircle(" << center << ", " << radius << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawRoundRect(const DlRoundRect& rrect) {
  startl() << "drawRRect(" << rrect << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawDiffRoundRect(const DlRoundRect& outer,
                                                    const DlRoundRect& inner) {
  startl() << "drawDRRect(outer: " << outer << ", " << std::endl;
  startl() << "           inner: " << inner << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawPath(const DlPath& path) {
  startl() << "drawPath(" << path << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawArc(const DlRect& oval_bounds,
                                          DlScalar start_degrees,
                                          DlScalar sweep_degrees,
                                          bool use_center) {
  startl() << "drawArc("
           << oval_bounds << ", "
           << start_degrees << ", "
           << sweep_degrees << ", "
           << "use_center: " << use_center
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawPoints(PointMode mode,
                                             uint32_t count,
                                             const DlPoint points[]) {
  startl() << "drawPoints(" << mode << ", ";
                          out_array("points", count, points)
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawVertices(const std::shared_ptr<DlVertices>& vertices,
                                               DlBlendMode mode) {
  startl() << "drawVertices("
               << "DlVertices("
                   << vertices->mode() << ", ";
                   out_array("vertices", vertices->vertex_count(), vertices->vertex_data()) << ", ";
                   out_array("texture_coords", vertices->vertex_count(), vertices->texture_coordinate_data()) << ", ";
                   out_array("colors", vertices->vertex_count(), vertices->colors()) << ", ";
                   out_array("indices", vertices->index_count(), vertices->indices())
                   << "), " << mode << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawImage(const sk_sp<DlImage> image,
                                            const DlPoint& point,
                                            DlImageSampling sampling,
                                            bool render_with_attributes) {
  startl() << "drawImage(" << image.get() << "," << std::endl;
  startl() << "          " << point << ", "
                           << sampling << ", "
                           << "with attributes: " << render_with_attributes
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawImageRect(const sk_sp<DlImage> image,
                                                const DlRect& src,
                                                const DlRect& dst,
                                                DlImageSampling sampling,
                                                bool render_with_attributes,
                                                SrcRectConstraint constraint) {
  startl() << "drawImageRect(" << image.get() << "," << std::endl;
  startl() << "              src: " << src << "," << std::endl;
  startl() << "              dst: " << dst << "," << std::endl;
  startl() << "              " << sampling << ", "
                               << "with attributes: " << render_with_attributes << ", "
                               << constraint
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawImageNine(const sk_sp<DlImage> image,
                                                const DlIRect& center,
                                                const DlRect& dst,
                                                DlFilterMode filter,
                                                bool render_with_attributes) {
  startl() << "drawImageNine(" << image.get() << "," << std::endl;
  startl() << "              center: " << center << "," << std::endl;
  startl() << "              dst: " << dst << "," << std::endl;
  startl() << "              " << filter << ", "
                               << "with attributes: " << render_with_attributes
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawAtlas(const sk_sp<DlImage> atlas,
                                            const SkRSXform xform[],
                                            const DlRect tex[],
                                            const DlColor colors[],
                                            int count,
                                            DlBlendMode mode,
                                            DlImageSampling sampling,
                                            const DlRect* cull_rect,
                                            bool render_with_attributes) {
  startl() << "drawAtlas(" << atlas.get() << ", ";
                   out_array("xforms", count, xform) << ", ";
                   out_array("tex_coords", count, tex) << ", ";
                   out_array("colors", count, colors) << ", "
                   << mode << ", " << sampling << ", cull: " << cull_rect << ", "
                   << "with attributes: " << render_with_attributes
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawDisplayList(
    const sk_sp<DisplayList> display_list, DlScalar opacity) {
  startl() << "drawDisplayList("
           << "ID: " << display_list->unique_id() << ", "
           << "bounds: " << display_list->bounds() << ", "
           << "opacity: " << opacity
           << ");" << std::endl;
}
void DisplayListStreamDispatcher::drawTextBlob(const sk_sp<SkTextBlob> blob,
                                               DlScalar x,
                                               DlScalar y) {
  startl() << "drawTextBlob("
           << blob.get() << ", "
           << x << ", " << y << ");" << std::endl;
}

void DisplayListStreamDispatcher::drawTextFrame(
    const std::shared_ptr<impeller::TextFrame>& text_frame,
    DlScalar x,
    DlScalar y) {
  startl() << "drawTextFrame("
    << text_frame.get() << ", "
    << x << ", " << y << ");" << std::endl;
}

void DisplayListStreamDispatcher::drawShadow(const DlPath& path,
                                             const DlColor color,
                                             const DlScalar elevation,
                                             bool transparent_occluder,
                                             DlScalar dpr) {
  startl() << "drawShadow("
           << path << ", "
           << color << ", "
           << elevation << ", "
           << transparent_occluder << ", "
           << dpr
           << ");" << std::endl;
}
// clang-format on

}  // namespace flutter::testing

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/dl_dispatcher.h"

#include <algorithm>
#include <cstring>
#include <memory>
#include <optional>
#include <vector>

#include "display_list/dl_sampling_options.h"
#include "display_list/effects/dl_image_filter.h"
#include "flutter/fml/logging.h"
#include "impeller/core/formats.h"
#include "impeller/display_list/aiks_context.h"
#include "impeller/display_list/canvas.h"
#include "impeller/display_list/dl_atlas_geometry.h"
#include "impeller/display_list/dl_vertices_geometry.h"
#include "impeller/display_list/nine_patch_converter.h"
#include "impeller/display_list/skia_conversions.h"
#include "impeller/entity/contents/atlas_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/filters/filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/geometry/ellipse_geometry.h"
#include "impeller/entity/geometry/fill_path_geometry.h"
#include "impeller/entity/geometry/rect_geometry.h"
#include "impeller/entity/geometry/round_rect_geometry.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/path.h"
#include "impeller/geometry/path_builder.h"
#include "impeller/geometry/scalar.h"
#include "impeller/geometry/sigma.h"
#include "impeller/typographer/font_glyph_pair.h"

namespace impeller {

#if !defined(NDEBUG)
#define USE_DEPTH_WATCHER true
#else
#define USE_DEPTH_WATCHER false
#endif  //  !defined(NDEBUG)

#if USE_DEPTH_WATCHER

// Invoke this macro at the top of any DlOpReceiver dispatch function
// using a number indicating the maximum depth that the operation is
// expected to consume in the Canvas. Most rendering ops consume 1
// except for DrawImageNine that currently consumes 1 per section (i.e. 9).
// Attribute, clip and transform ops do not consume depth but this
// macro can still be used with an argument of 0 to verify that expectation.
//
// The watchdog object allocated here will automatically double-check
// the depth usage at any exit point to the function, or any other
// point at which it falls out of scope.
#define AUTO_DEPTH_WATCHER(d)                            \
  DepthWatcher _watcher(__FILE__, __LINE__, GetCanvas(), \
                        paint_.mask_blur_descriptor.has_value(), d)

// While the AUTO_DEPTH_WATCHER macro will check the depth usage at
// any exit point from the dispatch function, sometimes the dispatch
// functions are somewhat compounded and result in multiple Canvas
// calls.
//
// Invoke this macro at any key points in the middle of a dispatch
// function to verify that you still haven't exceeded the maximum
// allowed depth. This is especially useful if the function does
// an implicit save/restore where the restore call might assert the
// depth constraints in a function in Canvas that can't be as easily
// traced back to a given dispatch function as these macros can.
#define AUTO_DEPTH_CHECK() _watcher.check(__FILE__, __LINE__)

// Helper class, use the AUTO_DEPTH_WATCHER macros to access it
struct DepthWatcher {
  DepthWatcher(const std::string& file,
               int line,
               const impeller::Canvas& canvas,
               bool has_mask_blur,
               int allowed)
      : file_(file),
        line_(line),
        canvas_(canvas),
        allowed_(has_mask_blur ? allowed + 1 : allowed),
        old_depth_(canvas.GetOpDepth()),
        old_max_(canvas.GetMaxOpDepth()) {}

  ~DepthWatcher() { check(file_, line_); }

  void check(const std::string& file, int line) {
    FML_CHECK(canvas_.GetOpDepth() <= (old_depth_ + allowed_) &&
              canvas_.GetOpDepth() <= old_max_)
        << std::endl
        << "from " << file << ":" << line << std::endl
        << "old/allowed/current/max = " << old_depth_ << "/" << allowed_ << "/"
        << canvas_.GetOpDepth() << "/" << old_max_;
  }

 private:
  const std::string file_;
  const int line_;

  const impeller::Canvas& canvas_;
  const uint64_t allowed_;
  const uint64_t old_depth_;
  const uint64_t old_max_;
};

#else  // USE_DEPTH_WATCHER

#define AUTO_DEPTH_WATCHER(d)
#define AUTO_DEPTH_CHECK()

#endif  // USE_DEPTH_WATCHER

#define UNIMPLEMENTED \
  FML_DLOG(ERROR) << "Unimplemented detail in " << __FUNCTION__;

static impeller::SamplerDescriptor ToSamplerDescriptor(
    const flutter::DlFilterMode options) {
  impeller::SamplerDescriptor desc;
  switch (options) {
    case flutter::DlFilterMode::kNearest:
      desc.min_filter = desc.mag_filter = impeller::MinMagFilter::kNearest;
      desc.label = "Nearest Sampler";
      break;
    case flutter::DlFilterMode::kLinear:
      desc.min_filter = desc.mag_filter = impeller::MinMagFilter::kLinear;
      desc.label = "Linear Sampler";
      break;
    default:
      break;
  }
  return desc;
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setAntiAlias(bool aa) {
  AUTO_DEPTH_WATCHER(0u);

  // Nothing to do because AA is implicit.
}

static Paint::Style ToStyle(flutter::DlDrawStyle style) {
  switch (style) {
    case flutter::DlDrawStyle::kFill:
      return Paint::Style::kFill;
    case flutter::DlDrawStyle::kStroke:
      return Paint::Style::kStroke;
    case flutter::DlDrawStyle::kStrokeAndFill:
      UNIMPLEMENTED;
      break;
  }
  return Paint::Style::kFill;
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setDrawStyle(flutter::DlDrawStyle style) {
  AUTO_DEPTH_WATCHER(0u);

  paint_.style = ToStyle(style);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setColor(flutter::DlColor color) {
  AUTO_DEPTH_WATCHER(0u);

  paint_.color = skia_conversions::ToColor(color);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setStrokeWidth(DlScalar width) {
  AUTO_DEPTH_WATCHER(0u);

  paint_.stroke_width = width;
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setStrokeMiter(DlScalar limit) {
  AUTO_DEPTH_WATCHER(0u);

  paint_.stroke_miter = limit;
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setStrokeCap(flutter::DlStrokeCap cap) {
  AUTO_DEPTH_WATCHER(0u);

  switch (cap) {
    case flutter::DlStrokeCap::kButt:
      paint_.stroke_cap = Cap::kButt;
      break;
    case flutter::DlStrokeCap::kRound:
      paint_.stroke_cap = Cap::kRound;
      break;
    case flutter::DlStrokeCap::kSquare:
      paint_.stroke_cap = Cap::kSquare;
      break;
  }
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setStrokeJoin(flutter::DlStrokeJoin join) {
  AUTO_DEPTH_WATCHER(0u);

  switch (join) {
    case flutter::DlStrokeJoin::kMiter:
      paint_.stroke_join = Join::kMiter;
      break;
    case flutter::DlStrokeJoin::kRound:
      paint_.stroke_join = Join::kRound;
      break;
    case flutter::DlStrokeJoin::kBevel:
      paint_.stroke_join = Join::kBevel;
      break;
  }
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setColorSource(const flutter::DlColorSource* source) {
  AUTO_DEPTH_WATCHER(0u);

  paint_.color_source = source;
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setColorFilter(const flutter::DlColorFilter* filter) {
  AUTO_DEPTH_WATCHER(0u);

  paint_.color_filter = filter;
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setInvertColors(bool invert) {
  AUTO_DEPTH_WATCHER(0u);

  paint_.invert_colors = invert;
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setBlendMode(flutter::DlBlendMode dl_mode) {
  AUTO_DEPTH_WATCHER(0u);

  paint_.blend_mode = skia_conversions::ToBlendMode(dl_mode);
}

static FilterContents::BlurStyle ToBlurStyle(flutter::DlBlurStyle blur_style) {
  switch (blur_style) {
    case flutter::DlBlurStyle::kNormal:
      return FilterContents::BlurStyle::kNormal;
    case flutter::DlBlurStyle::kSolid:
      return FilterContents::BlurStyle::kSolid;
    case flutter::DlBlurStyle::kOuter:
      return FilterContents::BlurStyle::kOuter;
    case flutter::DlBlurStyle::kInner:
      return FilterContents::BlurStyle::kInner;
  }
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setMaskFilter(const flutter::DlMaskFilter* filter) {
  AUTO_DEPTH_WATCHER(0u);

  // Needs https://github.com/flutter/flutter/issues/95434
  if (filter == nullptr) {
    paint_.mask_blur_descriptor = std::nullopt;
    return;
  }
  switch (filter->type()) {
    case flutter::DlMaskFilterType::kBlur: {
      auto blur = filter->asBlur();

      paint_.mask_blur_descriptor = {
          .style = ToBlurStyle(blur->style()),
          .sigma = Sigma(blur->sigma()),
          .respect_ctm = blur->respectCTM(),
      };
      break;
    }
  }
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::setImageFilter(const flutter::DlImageFilter* filter) {
  AUTO_DEPTH_WATCHER(0u);

  paint_.image_filter = filter;
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::save(uint32_t total_content_depth) {
  AUTO_DEPTH_WATCHER(1u);

  GetCanvas().Save(total_content_depth);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::saveLayer(const DlRect& bounds,
                                 const flutter::SaveLayerOptions& options,
                                 uint32_t total_content_depth,
                                 flutter::DlBlendMode max_content_mode,
                                 const flutter::DlImageFilter* backdrop,
                                 std::optional<int64_t> backdrop_id) {
  AUTO_DEPTH_WATCHER(1u);

  auto paint = options.renders_with_attributes() ? paint_ : Paint{};
  auto promise = options.content_is_clipped()
                     ? ContentBoundsPromise::kMayClipContents
                     : ContentBoundsPromise::kContainsContents;
  std::optional<Rect> impeller_bounds;
  // If the content is unbounded but has developer specified bounds, we take
  // the original bounds so that we clip the content as expected.
  if (!options.content_is_unbounded() || options.bounds_from_caller()) {
    impeller_bounds = bounds;
  }

  GetCanvas().SaveLayer(
      paint, impeller_bounds, backdrop, promise, total_content_depth,
      // Unbounded content can still have user specified bounds that require a
      // saveLayer to be created to perform the clip.
      options.can_distribute_opacity() && !options.content_is_unbounded(),
      backdrop_id  //
  );
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::restore() {
  GetCanvas().Restore();
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::translate(DlScalar tx, DlScalar ty) {
  AUTO_DEPTH_WATCHER(0u);

  GetCanvas().Translate({tx, ty, 0.0});
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::scale(DlScalar sx, DlScalar sy) {
  AUTO_DEPTH_WATCHER(0u);

  GetCanvas().Scale({sx, sy, 1.0});
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::rotate(DlScalar degrees) {
  AUTO_DEPTH_WATCHER(0u);

  GetCanvas().Rotate(Degrees{degrees});
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::skew(DlScalar sx, DlScalar sy) {
  AUTO_DEPTH_WATCHER(0u);

  GetCanvas().Skew(sx, sy);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::transform2DAffine(DlScalar mxx,
                                         DlScalar mxy,
                                         DlScalar mxt,
                                         DlScalar myx,
                                         DlScalar myy,
                                         DlScalar myt) {
  AUTO_DEPTH_WATCHER(0u);

  // clang-format off
  transformFullPerspective(
    mxx, mxy,  0, mxt,
    myx, myy,  0, myt,
    0  ,   0,  1,   0,
    0  ,   0,  0,   1
  );
  // clang-format on
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::transformFullPerspective(DlScalar mxx,
                                                DlScalar mxy,
                                                DlScalar mxz,
                                                DlScalar mxt,
                                                DlScalar myx,
                                                DlScalar myy,
                                                DlScalar myz,
                                                DlScalar myt,
                                                DlScalar mzx,
                                                DlScalar mzy,
                                                DlScalar mzz,
                                                DlScalar mzt,
                                                DlScalar mwx,
                                                DlScalar mwy,
                                                DlScalar mwz,
                                                DlScalar mwt) {
  AUTO_DEPTH_WATCHER(0u);

  // The order of arguments is row-major but Impeller matrices are
  // column-major.
  // clang-format off
  auto transform = Matrix{
    mxx, myx, mzx, mwx,
    mxy, myy, mzy, mwy,
    mxz, myz, mzz, mwz,
    mxt, myt, mzt, mwt
  };
  // clang-format on
  GetCanvas().Transform(transform);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::transformReset() {
  AUTO_DEPTH_WATCHER(0u);

  GetCanvas().ResetTransform();
  GetCanvas().Transform(initial_matrix_);
}

static Entity::ClipOperation ToClipOperation(
    flutter::DlCanvas::ClipOp clip_op) {
  switch (clip_op) {
    case flutter::DlCanvas::ClipOp::kDifference:
      return Entity::ClipOperation::kDifference;
    case flutter::DlCanvas::ClipOp::kIntersect:
      return Entity::ClipOperation::kIntersect;
  }
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::clipRect(const DlRect& rect,
                                ClipOp clip_op,
                                bool is_aa) {
  AUTO_DEPTH_WATCHER(0u);

  RectGeometry geom(rect);
  GetCanvas().ClipGeometry(geom, ToClipOperation(clip_op), /*is_aa=*/is_aa);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::clipOval(const DlRect& bounds,
                                ClipOp clip_op,
                                bool is_aa) {
  AUTO_DEPTH_WATCHER(0u);

  EllipseGeometry geom(bounds);
  GetCanvas().ClipGeometry(geom, ToClipOperation(clip_op));
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::clipRoundRect(const DlRoundRect& rrect,
                                     ClipOp sk_op,
                                     bool is_aa) {
  AUTO_DEPTH_WATCHER(0u);

  auto clip_op = ToClipOperation(sk_op);
  if (rrect.IsRect()) {
    RectGeometry geom(rrect.GetBounds());
    GetCanvas().ClipGeometry(geom, clip_op, /*is_aa=*/is_aa);
  } else if (rrect.IsOval()) {
    EllipseGeometry geom(rrect.GetBounds());
    GetCanvas().ClipGeometry(geom, clip_op);
  } else if (rrect.GetRadii().AreAllCornersSame()) {
    RoundRectGeometry geom(rrect.GetBounds(), rrect.GetRadii().top_left);
    GetCanvas().ClipGeometry(geom, clip_op);
  } else {
    FillPathGeometry geom(PathBuilder{}.AddRoundRect(rrect).TakePath());
    GetCanvas().ClipGeometry(geom, clip_op);
  }
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::clipPath(const DlPath& path, ClipOp sk_op, bool is_aa) {
  AUTO_DEPTH_WATCHER(0u);

  auto clip_op = ToClipOperation(sk_op);

  DlRect rect;
  if (path.IsRect(&rect)) {
    RectGeometry geom(rect);
    GetCanvas().ClipGeometry(geom, clip_op, /*is_aa=*/is_aa);
  } else if (path.IsOval(&rect)) {
    EllipseGeometry geom(rect);
    GetCanvas().ClipGeometry(geom, clip_op);
  } else {
    SkRRect rrect;
    if (path.IsSkRRect(&rrect) && rrect.isSimple()) {
      RoundRectGeometry geom(skia_conversions::ToRect(rrect.rect()),
                             skia_conversions::ToSize(rrect.getSimpleRadii()));
      GetCanvas().ClipGeometry(geom, clip_op);
    } else {
      FillPathGeometry geom(path.GetPath());
      GetCanvas().ClipGeometry(geom, clip_op);
    }
  }
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawColor(flutter::DlColor color,
                                 flutter::DlBlendMode dl_mode) {
  AUTO_DEPTH_WATCHER(1u);

  Paint paint;
  paint.color = skia_conversions::ToColor(color);
  paint.blend_mode = skia_conversions::ToBlendMode(dl_mode);
  GetCanvas().DrawPaint(paint);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawPaint() {
  AUTO_DEPTH_WATCHER(1u);

  GetCanvas().DrawPaint(paint_);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawLine(const DlPoint& p0, const DlPoint& p1) {
  AUTO_DEPTH_WATCHER(1u);

  GetCanvas().DrawLine(p0, p1, paint_);
}

void DlDispatcherBase::drawDashedLine(const DlPoint& p0,
                                      const DlPoint& p1,
                                      DlScalar on_length,
                                      DlScalar off_length) {
  AUTO_DEPTH_WATCHER(1u);

  Scalar length = p0.GetDistance(p1);
  // Reasons to defer to regular DrawLine:
  //   length is non-positive - drawLine will draw appropriate "dot"
  //   off_length is non-positive - no gaps, drawLine will draw it solid
  //   on_length is negative - invalid dashing
  // Note that a 0 length "on" dash will draw "dot"s every "off" distance
  // apart
  if (length > 0.0f && on_length >= 0.0f && off_length > 0.0f) {
    Point delta = (p1 - p0) / length;  // length > 0 already tested
    PathBuilder builder;

    Scalar consumed = 0.0f;
    while (consumed < length) {
      builder.MoveTo(p0 + delta * consumed);

      Scalar dash_end = consumed + on_length;
      if (dash_end < length) {
        builder.LineTo(p0 + delta * dash_end);
      } else {
        builder.LineTo(p1);
        // Should happen anyway due to the math, but let's make it explicit
        // in case of bit errors. We're done with this line.
        break;
      }

      consumed = dash_end + off_length;
    }

    Paint stroke_paint = paint_;
    stroke_paint.style = Paint::Style::kStroke;
    GetCanvas().DrawPath(builder.TakePath(), stroke_paint);
  } else {
    drawLine(p0, p1);
  }
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawRect(const DlRect& rect) {
  AUTO_DEPTH_WATCHER(1u);

  GetCanvas().DrawRect(rect, paint_);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawOval(const DlRect& bounds) {
  AUTO_DEPTH_WATCHER(1u);

  GetCanvas().DrawOval(bounds, paint_);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawCircle(const DlPoint& center, DlScalar radius) {
  AUTO_DEPTH_WATCHER(1u);

  GetCanvas().DrawCircle(center, radius, paint_);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawRoundRect(const DlRoundRect& rrect) {
  AUTO_DEPTH_WATCHER(1u);

  GetCanvas().DrawRoundRect(rrect, paint_);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawDiffRoundRect(const DlRoundRect& outer,
                                         const DlRoundRect& inner) {
  AUTO_DEPTH_WATCHER(1u);

  PathBuilder builder;
  builder.AddRoundRect(outer);
  builder.AddRoundRect(inner);
  builder.SetBounds(outer.GetBounds().Union(inner.GetBounds()));
  GetCanvas().DrawPath(builder.TakePath(FillType::kOdd), paint_);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawPath(const DlPath& path) {
  AUTO_DEPTH_WATCHER(1u);

  SimplifyOrDrawPath(GetCanvas(), path, paint_);
}

void DlDispatcherBase::SimplifyOrDrawPath(Canvas& canvas,
                                          const DlPath& path,
                                          const Paint& paint) {
  DlRect rect;

  // We can't "optimize" a path into a rectangle if it's open.
  bool closed;
  if (path.IsRect(&rect, &closed) && closed) {
    canvas.DrawRect(rect, paint);
    return;
  }

  SkRRect rrect;
  if (path.IsSkRRect(&rrect) && rrect.isSimple()) {
    canvas.DrawRoundRect(flutter::ToDlRoundRect(rrect), paint);
    return;
  }

  if (path.IsOval(&rect)) {
    canvas.DrawOval(rect, paint);
    return;
  }

  canvas.DrawPath(path.GetPath(), paint);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawArc(const DlRect& oval_bounds,
                               DlScalar start_degrees,
                               DlScalar sweep_degrees,
                               bool use_center) {
  AUTO_DEPTH_WATCHER(1u);

  PathBuilder builder;
  builder.AddArc(oval_bounds, Degrees(start_degrees), Degrees(sweep_degrees),
                 use_center);
  GetCanvas().DrawPath(builder.TakePath(), paint_);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawPoints(PointMode mode,
                                  uint32_t count,
                                  const DlPoint points[]) {
  AUTO_DEPTH_WATCHER(1u);

  Paint paint = paint_;
  paint.style = Paint::Style::kStroke;
  switch (mode) {
    case flutter::DlCanvas::PointMode::kPoints: {
      // Cap::kButt is also treated as a square.
      PointStyle point_style = paint.stroke_cap == Cap::kRound
                                   ? PointStyle::kRound
                                   : PointStyle::kSquare;
      Scalar radius = paint.stroke_width;
      if (radius > 0) {
        radius /= 2.0;
      }
      GetCanvas().DrawPoints(points, count, radius, paint, point_style);
    } break;
    case flutter::DlCanvas::PointMode::kLines:
      for (uint32_t i = 1; i < count; i += 2) {
        Point p0 = points[i - 1];
        Point p1 = points[i];
        GetCanvas().DrawLine(p0, p1, paint, /*reuse_depth=*/i > 1);
      }
      break;
    case flutter::DlCanvas::PointMode::kPolygon:
      if (count > 1) {
        Point p0 = points[0];
        for (uint32_t i = 1; i < count; i++) {
          Point p1 = points[i];
          GetCanvas().DrawLine(p0, p1, paint, /*reuse_depth=*/i > 1);
          p0 = p1;
        }
      }
      break;
  }
}

void DlDispatcherBase::drawVertices(
    const std::shared_ptr<flutter::DlVertices>& vertices,
    flutter::DlBlendMode dl_mode) {}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawImage(const sk_sp<flutter::DlImage> image,
                                 const DlPoint& point,
                                 flutter::DlImageSampling sampling,
                                 bool render_with_attributes) {
  AUTO_DEPTH_WATCHER(1u);

  if (!image) {
    return;
  }

  auto texture = image->impeller_texture();
  if (!texture) {
    return;
  }

  const auto size = texture->GetSize();
  const auto src = DlRect::MakeWH(size.width, size.height);
  const auto dest = DlRect::MakeXYWH(point.x, point.y, size.width, size.height);

  drawImageRect(image,                      // image
                src,                        // source rect
                dest,                       // destination rect
                sampling,                   // sampling options
                render_with_attributes,     // render with attributes
                SrcRectConstraint::kStrict  // constraint
  );
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawImageRect(
    const sk_sp<flutter::DlImage> image,
    const DlRect& src,
    const DlRect& dst,
    flutter::DlImageSampling sampling,
    bool render_with_attributes,
    SrcRectConstraint constraint = SrcRectConstraint::kFast) {
  AUTO_DEPTH_WATCHER(1u);

  GetCanvas().DrawImageRect(
      image->impeller_texture(),                       // image
      src,                                             // source rect
      dst,                                             // destination rect
      render_with_attributes ? paint_ : Paint(),       // paint
      skia_conversions::ToSamplerDescriptor(sampling)  // sampling
  );
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawImageNine(const sk_sp<flutter::DlImage> image,
                                     const DlIRect& center,
                                     const DlRect& dst,
                                     flutter::DlFilterMode filter,
                                     bool render_with_attributes) {
  AUTO_DEPTH_WATCHER(9u);

  NinePatchConverter converter = {};
  converter.DrawNinePatch(image->impeller_texture(),
                          Rect::MakeLTRB(center.GetLeft(), center.GetTop(),
                                         center.GetRight(), center.GetBottom()),
                          dst, ToSamplerDescriptor(filter), &GetCanvas(),
                          &paint_);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawAtlas(const sk_sp<flutter::DlImage> atlas,
                                 const SkRSXform xform[],
                                 const DlRect tex[],
                                 const flutter::DlColor colors[],
                                 int count,
                                 flutter::DlBlendMode mode,
                                 flutter::DlImageSampling sampling,
                                 const DlRect* cull_rect,
                                 bool render_with_attributes) {
  AUTO_DEPTH_WATCHER(1u);

  auto geometry =
      DlAtlasGeometry(atlas->impeller_texture(),                        //
                      xform,                                            //
                      tex,                                              //
                      colors,                                           //
                      static_cast<size_t>(count),                       //
                      skia_conversions::ToBlendMode(mode),              //
                      skia_conversions::ToSamplerDescriptor(sampling),  //
                      skia_conversions::ToRect(cull_rect)               //
      );
  auto atlas_contents = std::make_shared<AtlasContents>();
  atlas_contents->SetGeometry(&geometry);

  GetCanvas().DrawAtlas(atlas_contents, paint_);
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawDisplayList(
    const sk_sp<flutter::DisplayList> display_list,
    DlScalar opacity) {
  AUTO_DEPTH_WATCHER(display_list->total_depth());

  // Save all values that must remain untouched after the operation.
  Paint saved_paint = paint_;
  Matrix saved_initial_matrix = initial_matrix_;

  // Establish a new baseline for interpreting the new DL.
  // Matrix and clip are left untouched, the current
  // transform is saved as the new base matrix, and paint
  // values are reset to defaults.
  initial_matrix_ = GetCanvas().GetCurrentTransform();
  paint_ = Paint();

  // Handle passed opacity in the most brute-force way by using
  // a SaveLayer. If the display_list is able to inherit the
  // opacity, this could also be handled by modulating all of its
  // attribute settings (for example, color), by the indicated
  // opacity.
  int restore_count = GetCanvas().GetSaveCount();
  if (opacity < SK_Scalar1) {
    Paint save_paint;
    save_paint.color = Color(0, 0, 0, opacity);
    GetCanvas().SaveLayer(
        save_paint, skia_conversions::ToRect(display_list->bounds()), nullptr,
        ContentBoundsPromise::kContainsContents, display_list->total_depth(),
        display_list->can_apply_group_opacity());
  } else {
    // The display list may alter the clip, which must be restored to the
    // current clip at the end of playback.
    GetCanvas().Save(display_list->total_depth());
  }

  // TODO(131445): Remove this restriction if we can correctly cull with
  // perspective transforms.
  if (display_list->has_rtree() && !initial_matrix_.HasPerspective()) {
    // The canvas remembers the screen-space culling bounds clipped by
    // the surface and the history of clip calls. DisplayList can cull
    // the ops based on a rectangle expressed in its "destination bounds"
    // so we need the canvas to transform those into the current local
    // coordinate space into which the DisplayList will be rendered.
    auto global_culling_bounds = GetCanvas().GetLocalCoverageLimit();
    if (global_culling_bounds.has_value()) {
      Rect cull_rect = global_culling_bounds->TransformBounds(
          GetCanvas().GetCurrentTransform().Invert());
      display_list->Dispatch(
          *this, SkRect::MakeLTRB(cull_rect.GetLeft(), cull_rect.GetTop(),
                                  cull_rect.GetRight(), cull_rect.GetBottom()));
    } else {
      // If the culling bounds are empty, this display list can be skipped
      // entirely.
    }
  } else {
    display_list->Dispatch(*this);
  }

  // Restore all saved state back to what it was before we interpreted
  // the display_list
  AUTO_DEPTH_CHECK();
  GetCanvas().RestoreToCount(restore_count);
  initial_matrix_ = saved_initial_matrix;
  paint_ = saved_paint;
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawTextBlob(const sk_sp<SkTextBlob> blob,
                                    DlScalar x,
                                    DlScalar y) {
  // When running with Impeller enabled Skia text blobs are converted to
  // Impeller text frames in paragraph_skia.cc
  UNIMPLEMENTED;
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawTextFrame(
    const std::shared_ptr<TextFrame>& text_frame,
    DlScalar x,
    DlScalar y) {
  AUTO_DEPTH_WATCHER(1u);

  GetCanvas().DrawTextFrame(text_frame,             //
                            impeller::Point{x, y},  //
                            paint_                  //
  );
}

// |flutter::DlOpReceiver|
void DlDispatcherBase::drawShadow(const DlPath& path,
                                  const flutter::DlColor color,
                                  const DlScalar elevation,
                                  bool transparent_occluder,
                                  DlScalar dpr) {
  AUTO_DEPTH_WATCHER(1u);

  Color spot_color = skia_conversions::ToColor(color);
  spot_color.alpha *= 0.25;

  // Compute the spot color -- ported from SkShadowUtils::ComputeTonalColors.
  {
    Scalar max =
        std::max(std::max(spot_color.red, spot_color.green), spot_color.blue);
    Scalar min =
        std::min(std::min(spot_color.red, spot_color.green), spot_color.blue);
    Scalar luminance = (min + max) * 0.5;

    Scalar alpha_adjust =
        (2.6f + (-2.66667f + 1.06667f * spot_color.alpha) * spot_color.alpha) *
        spot_color.alpha;
    Scalar color_alpha =
        (3.544762f + (-4.891428f + 2.3466f * luminance) * luminance) *
        luminance;
    color_alpha = std::clamp(alpha_adjust * color_alpha, 0.0f, 1.0f);

    Scalar greyscale_alpha =
        std::clamp(spot_color.alpha * (1 - 0.4f * luminance), 0.0f, 1.0f);

    Scalar color_scale = color_alpha * (1 - greyscale_alpha);
    Scalar tonal_alpha = color_scale + greyscale_alpha;
    Scalar unpremul_scale = tonal_alpha != 0 ? color_scale / tonal_alpha : 0;
    spot_color = Color(unpremul_scale * spot_color.red,
                       unpremul_scale * spot_color.green,
                       unpremul_scale * spot_color.blue, tonal_alpha);
  }

  Vector3 light_position(0, -1, 1);
  Scalar occluder_z = dpr * elevation;

  constexpr Scalar kLightRadius = 800 / 600;  // Light radius / light height

  Paint paint;
  paint.style = Paint::Style::kFill;
  paint.color = spot_color;
  paint.mask_blur_descriptor = Paint::MaskBlurDescriptor{
      .style = FilterContents::BlurStyle::kNormal,
      .sigma = Radius{kLightRadius * occluder_z /
                      GetCanvas().GetCurrentTransform().GetScale().y},
  };

  GetCanvas().Save(1u);
  GetCanvas().PreConcat(
      Matrix::MakeTranslation(Vector2(0, -occluder_z * light_position.y)));

  SimplifyOrDrawPath(GetCanvas(), path, paint);
  AUTO_DEPTH_CHECK();

  GetCanvas().Restore();
}

/// Subclasses

static bool RequiresReadbackForBlends(
    const ContentContext& renderer,
    flutter::DlBlendMode max_root_blend_mode) {
  return !renderer.GetDeviceCapabilities().SupportsFramebufferFetch() &&
         skia_conversions::ToBlendMode(max_root_blend_mode) >
             Entity::kLastPipelineBlendMode;
}

CanvasDlDispatcher::CanvasDlDispatcher(ContentContext& renderer,
                                       RenderTarget& render_target,
                                       bool has_root_backdrop_filter,
                                       flutter::DlBlendMode max_root_blend_mode,
                                       IRect cull_rect)
    : canvas_(renderer,
              render_target,
              has_root_backdrop_filter ||
                  RequiresReadbackForBlends(renderer, max_root_blend_mode),
              cull_rect),
      renderer_(renderer) {}

Canvas& CanvasDlDispatcher::GetCanvas() {
  return canvas_;
}

void CanvasDlDispatcher::drawVertices(
    const std::shared_ptr<flutter::DlVertices>& vertices,
    flutter::DlBlendMode dl_mode) {
  AUTO_DEPTH_WATCHER(1u);

  GetCanvas().DrawVertices(
      std::make_shared<DlVerticesGeometry>(vertices, renderer_),
      skia_conversions::ToBlendMode(dl_mode), paint_);
}

void CanvasDlDispatcher::SetBackdropData(
    std::unordered_map<int64_t, BackdropData> backdrop,
    size_t backdrop_count) {
  GetCanvas().SetBackdropData(std::move(backdrop), backdrop_count);
}

//// Text Frame Dispatcher

FirstPassDispatcher::FirstPassDispatcher(const ContentContext& renderer,
                                         const Matrix& initial_matrix,
                                         const Rect cull_rect)
    : renderer_(renderer), matrix_(initial_matrix) {
  cull_rect_state_.push_back(cull_rect);
}

FirstPassDispatcher::~FirstPassDispatcher() {
  FML_DCHECK(cull_rect_state_.size() == 1);
}

void FirstPassDispatcher::save() {
  stack_.emplace_back(matrix_);
  cull_rect_state_.push_back(cull_rect_state_.back());
}

void FirstPassDispatcher::saveLayer(const DlRect& bounds,
                                    const flutter::SaveLayerOptions options,
                                    const flutter::DlImageFilter* backdrop,
                                    std::optional<int64_t> backdrop_id) {
  save();

  backdrop_count_ += (backdrop == nullptr ? 0 : 1);
  if (backdrop != nullptr && backdrop_id.has_value()) {
    std::shared_ptr<flutter::DlImageFilter> shared_backdrop =
        backdrop->shared();
    std::unordered_map<int64_t, BackdropData>::iterator existing =
        backdrop_data_.find(backdrop_id.value());
    if (existing == backdrop_data_.end()) {
      backdrop_data_[backdrop_id.value()] =
          BackdropData{.backdrop_count = 1, .last_backdrop = shared_backdrop};
    } else {
      BackdropData& data = existing->second;
      data.backdrop_count++;
      if (data.all_filters_equal) {
        data.all_filters_equal = (*data.last_backdrop == *shared_backdrop);
        data.last_backdrop = shared_backdrop;
      }
    }
  }

  // This dispatcher does not track enough state to accurately compute
  // cull rects with image filters.
  auto global_cull_rect = cull_rect_state_.back();
  if (has_image_filter_ || global_cull_rect.IsMaximum()) {
    cull_rect_state_.back() = Rect::MakeMaximum();
  } else {
    auto global_save_bounds = bounds.TransformBounds(matrix_);
    auto new_cull_rect = global_cull_rect.Intersection(global_save_bounds);
    if (new_cull_rect.has_value()) {
      cull_rect_state_.back() = new_cull_rect.value();
    } else {
      cull_rect_state_.back() = Rect::MakeLTRB(0, 0, 0, 0);
    }
  }
}

void FirstPassDispatcher::restore() {
  matrix_ = stack_.back();
  stack_.pop_back();
  cull_rect_state_.pop_back();
}

void FirstPassDispatcher::translate(DlScalar tx, DlScalar ty) {
  matrix_ = matrix_.Translate({tx, ty});
}

void FirstPassDispatcher::scale(DlScalar sx, DlScalar sy) {
  matrix_ = matrix_.Scale({sx, sy, 1.0f});
}

void FirstPassDispatcher::rotate(DlScalar degrees) {
  matrix_ = matrix_ * Matrix::MakeRotationZ(Degrees(degrees));
}

void FirstPassDispatcher::skew(DlScalar sx, DlScalar sy) {
  matrix_ = matrix_ * Matrix::MakeSkew(sx, sy);
}

// clang-format off
  // 2x3 2D affine subset of a 4x4 transform in row major order
  void FirstPassDispatcher::transform2DAffine(DlScalar mxx, DlScalar mxy, DlScalar mxt,
                                              DlScalar myx, DlScalar myy, DlScalar myt) {
    matrix_ = matrix_ * Matrix::MakeColumn(
        mxx,  myx,  0.0f, 0.0f,
        mxy,  myy,  0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        mxt,  myt,  0.0f, 1.0f
    );
  }

  // full 4x4 transform in row major order
  void FirstPassDispatcher::transformFullPerspective(
      DlScalar mxx, DlScalar mxy, DlScalar mxz, DlScalar mxt,
      DlScalar myx, DlScalar myy, DlScalar myz, DlScalar myt,
      DlScalar mzx, DlScalar mzy, DlScalar mzz, DlScalar mzt,
      DlScalar mwx, DlScalar mwy, DlScalar mwz, DlScalar mwt) {
    matrix_ = matrix_ * Matrix::MakeColumn(
        mxx, myx, mzx, mwx,
        mxy, myy, mzy, mwy,
        mxz, myz, mzz, mwz,
        mxt, myt, mzt, mwt
    );
  }
// clang-format on

void FirstPassDispatcher::transformReset() {
  matrix_ = Matrix();
}

void FirstPassDispatcher::drawTextFrame(
    const std::shared_ptr<impeller::TextFrame>& text_frame,
    DlScalar x,
    DlScalar y) {
  GlyphProperties properties;
  if (paint_.style == Paint::Style::kStroke) {
    properties.stroke = true;
    properties.stroke_cap = paint_.stroke_cap;
    properties.stroke_join = paint_.stroke_join;
    properties.stroke_miter = paint_.stroke_miter;
    properties.stroke_width = paint_.stroke_width;
  }
  if (text_frame->HasColor()) {
    // Alpha is always applied when rendering, remove it here so
    // we do not double-apply the alpha.
    properties.color = paint_.color.WithAlpha(1.0);
  }
  auto scale = TextFrame::RoundScaledFontSize(
      (matrix_ * Matrix::MakeTranslation(Point(x, y))).GetMaxBasisLengthXY());

  renderer_.GetLazyGlyphAtlas()->AddTextFrame(
      text_frame,                                       //
      scale,                                            //
      Point(x, y),                                      //
      (properties.stroke || text_frame->HasColor())     //
          ? std::optional<GlyphProperties>(properties)  //
          : std::nullopt                                //
  );
}

const Rect FirstPassDispatcher::GetCurrentLocalCullingBounds() const {
  auto cull_rect = cull_rect_state_.back();
  if (!cull_rect.IsEmpty() && !cull_rect.IsMaximum()) {
    Matrix inverse = matrix_.Invert();
    cull_rect = cull_rect.TransformBounds(inverse);
  }
  return cull_rect;
}

void FirstPassDispatcher::drawDisplayList(
    const sk_sp<flutter::DisplayList> display_list,
    DlScalar opacity) {
  [[maybe_unused]] size_t stack_depth = stack_.size();
  save();
  Paint old_paint = paint_;
  paint_ = Paint{};
  bool old_has_image_filter = has_image_filter_;
  has_image_filter_ = false;

  if (matrix_.HasPerspective()) {
    display_list->Dispatch(*this);
  } else {
    Rect local_cull_bounds = GetCurrentLocalCullingBounds();
    if (local_cull_bounds.IsMaximum()) {
      display_list->Dispatch(*this);
    } else if (!local_cull_bounds.IsEmpty()) {
      IRect cull_rect = IRect::RoundOut(local_cull_bounds);
      display_list->Dispatch(*this,
                             SkIRect::MakeLTRB(cull_rect.GetLeft(),   //
                                               cull_rect.GetTop(),    //
                                               cull_rect.GetRight(),  //
                                               cull_rect.GetBottom()  //
                                               ));
    }
  }

  restore();
  paint_ = old_paint;
  has_image_filter_ = old_has_image_filter;
  FML_DCHECK(stack_depth == stack_.size());
}

// |flutter::DlOpReceiver|
void FirstPassDispatcher::setDrawStyle(flutter::DlDrawStyle style) {
  paint_.style = ToStyle(style);
}

// |flutter::DlOpReceiver|
void FirstPassDispatcher::setColor(flutter::DlColor color) {
  paint_.color = skia_conversions::ToColor(color);
}

// |flutter::DlOpReceiver|
void FirstPassDispatcher::setStrokeWidth(DlScalar width) {
  paint_.stroke_width = width;
}

// |flutter::DlOpReceiver|
void FirstPassDispatcher::setStrokeMiter(DlScalar limit) {
  paint_.stroke_miter = limit;
}

// |flutter::DlOpReceiver|
void FirstPassDispatcher::setStrokeCap(flutter::DlStrokeCap cap) {
  switch (cap) {
    case flutter::DlStrokeCap::kButt:
      paint_.stroke_cap = Cap::kButt;
      break;
    case flutter::DlStrokeCap::kRound:
      paint_.stroke_cap = Cap::kRound;
      break;
    case flutter::DlStrokeCap::kSquare:
      paint_.stroke_cap = Cap::kSquare;
      break;
  }
}

// |flutter::DlOpReceiver|
void FirstPassDispatcher::setStrokeJoin(flutter::DlStrokeJoin join) {
  switch (join) {
    case flutter::DlStrokeJoin::kMiter:
      paint_.stroke_join = Join::kMiter;
      break;
    case flutter::DlStrokeJoin::kRound:
      paint_.stroke_join = Join::kRound;
      break;
    case flutter::DlStrokeJoin::kBevel:
      paint_.stroke_join = Join::kBevel;
      break;
  }
}

// |flutter::DlOpReceiver|
void FirstPassDispatcher::setImageFilter(const flutter::DlImageFilter* filter) {
  if (filter == nullptr) {
    has_image_filter_ = false;
  } else {
    has_image_filter_ = true;
  }
}

std::pair<std::unordered_map<int64_t, BackdropData>, size_t>
FirstPassDispatcher::TakeBackdropData() {
  std::unordered_map<int64_t, BackdropData> temp;
  std::swap(temp, backdrop_data_);
  return std::make_pair(temp, backdrop_count_);
}

std::shared_ptr<Texture> DisplayListToTexture(
    const sk_sp<flutter::DisplayList>& display_list,
    ISize size,
    AiksContext& context,
    bool reset_host_buffer,
    bool generate_mips) {
  int mip_count = 1;
  if (generate_mips) {
    mip_count = size.MipCount();
  }
  // Do not use the render target cache as the lifecycle of this texture
  // will outlive a particular frame.
  impeller::RenderTargetAllocator render_target_allocator =
      impeller::RenderTargetAllocator(
          context.GetContext()->GetResourceAllocator());
  impeller::RenderTarget target;
  if (context.GetContext()->GetCapabilities()->SupportsOffscreenMSAA()) {
    target = render_target_allocator.CreateOffscreenMSAA(
        *context.GetContext(),  // context
        size,                   // size
        /*mip_count=*/mip_count,
        "Picture Snapshot MSAA",  // label
        impeller::RenderTarget::
            kDefaultColorAttachmentConfigMSAA  // color_attachment_config
    );
  } else {
    target = render_target_allocator.CreateOffscreen(
        *context.GetContext(),  // context
        size,                   // size
        /*mip_count=*/mip_count,
        "Picture Snapshot",  // label
        impeller::RenderTarget::
            kDefaultColorAttachmentConfig  // color_attachment_config
    );
  }

  SkIRect sk_cull_rect = SkIRect::MakeWH(size.width, size.height);
  impeller::FirstPassDispatcher collector(
      context.GetContentContext(), impeller::Matrix(), Rect::MakeSize(size));
  display_list->Dispatch(collector, sk_cull_rect);
  impeller::CanvasDlDispatcher impeller_dispatcher(
      context.GetContentContext(),               //
      target,                                    //
      display_list->root_has_backdrop_filter(),  //
      display_list->max_root_blend_mode(),       //
      impeller::IRect::MakeSize(size)            //
  );
  const auto& [data, count] = collector.TakeBackdropData();
  impeller_dispatcher.SetBackdropData(data, count);
  display_list->Dispatch(impeller_dispatcher, sk_cull_rect);
  impeller_dispatcher.FinishRecording();

  if (reset_host_buffer) {
    context.GetContentContext().GetTransientsBuffer().Reset();
  }
  context.GetContentContext().GetLazyGlyphAtlas()->ResetTextFrames();
  context.GetContext()->DisposeThreadLocalCachedResources();

  return target.GetRenderTargetTexture();
}

bool RenderToOnscreen(ContentContext& context,
                      RenderTarget render_target,
                      const sk_sp<flutter::DisplayList>& display_list,
                      SkIRect cull_rect,
                      bool reset_host_buffer) {
  Rect ip_cull_rect = Rect::MakeLTRB(cull_rect.left(), cull_rect.top(),
                                     cull_rect.right(), cull_rect.bottom());
  FirstPassDispatcher collector(context, impeller::Matrix(), ip_cull_rect);
  display_list->Dispatch(collector, cull_rect);

  impeller::CanvasDlDispatcher impeller_dispatcher(
      context,                                   //
      render_target,                             //
      display_list->root_has_backdrop_filter(),  //
      display_list->max_root_blend_mode(),       //
      IRect::RoundOut(ip_cull_rect)              //
  );
  const auto& [data, count] = collector.TakeBackdropData();
  impeller_dispatcher.SetBackdropData(data, count);
  display_list->Dispatch(impeller_dispatcher, cull_rect);
  impeller_dispatcher.FinishRecording();
  if (reset_host_buffer) {
    context.GetTransientsBuffer().Reset();
  }
  context.GetLazyGlyphAtlas()->ResetTextFrames();

  return true;
}

}  // namespace impeller

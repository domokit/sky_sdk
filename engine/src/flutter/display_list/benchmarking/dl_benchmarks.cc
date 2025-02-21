// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/benchmarking/dl_benchmarks.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_op_flags.h"
#include "flutter/display_list/skia/dl_sk_canvas.h"
#include "flutter/display_list/testing/dl_test_snippets.h"

#include "third_party/skia/include/core/SkBitmap.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/core/SkTextBlob.h"

namespace flutter {
namespace testing {

DlPaint GetPaintForRun(unsigned attributes) {
  DlPaint paint;

  if (attributes & kStrokedStyle && attributes & kFilledStyle) {
    // Not currently exposed by Flutter, but we can probably benchmark this in
    // the future
    paint.setDrawStyle(DlDrawStyle::kStrokeAndFill);
  } else if (attributes & kStrokedStyle) {
    paint.setDrawStyle(DlDrawStyle::kStroke);
  } else if (attributes & kFilledStyle) {
    paint.setDrawStyle(DlDrawStyle::kFill);
  }

  if (attributes & kHairlineStroke) {
    paint.setStrokeWidth(0.0f);
  } else {
    paint.setStrokeWidth(1.0f);
  }

  paint.setAntiAlias(attributes & kAntiAliasing);
  return paint;
}

void AnnotateAttributes(unsigned attributes,
                        benchmark::State& state,
                        const DisplayListAttributeFlags flags) {
  if (flags.always_stroked()) {
    state.counters["HairlineStroke"] = attributes & kHairlineStroke ? 1 : 0;
  }
  if (flags.applies_style()) {
    state.counters["HairlineStroke"] = attributes & kHairlineStroke ? 1 : 0;
    state.counters["StrokedStyle"] = attributes & kStrokedStyle ? 1 : 0;
    state.counters["FilledStyle"] = attributes & kFilledStyle ? 1 : 0;
  }
  if (flags.applies_anti_alias()) {
    state.counters["AntiAliasing"] = attributes & kAntiAliasing ? 1 : 0;
  }
}

// Constants chosen to produce benchmark results in the region of 1-50ms
constexpr size_t kLinesToDraw = 10000;
constexpr size_t kRectsToDraw = 5000;
constexpr size_t kOvalsToDraw = 1000;
constexpr size_t kCirclesToDraw = 5000;
constexpr size_t kRRectsToDraw = 5000;
constexpr size_t kDRRectsToDraw = 2000;
constexpr size_t kArcSweepSetsToDraw = 1000;
constexpr size_t kImagesToDraw = 500;
constexpr size_t kFixedCanvasSize = 1024;

// Draw a series of diagonal lines across a square canvas of width/height of
// the length requested. The lines will start from the top left corner to the
// bottom right corner, and move from left to right (at the top) and from right
// to left (at the bottom) until 10,000 lines are drawn.
//
// The resulting image will be an hourglass shape.
void BM_DrawLine(benchmark::State& state,
                 BackendType backend_type,
                 unsigned attributes) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kDrawLineFlags);

  size_t length = state.range(0);

  surface_provider->InitializeSurface(length, length);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  state.counters["DrawCallCount"] = kLinesToDraw;
  for (size_t i = 0; i < kLinesToDraw; i++) {
    builder.DrawLine(DlPoint(i % length, 0),
                     DlPoint(length - i % length, length), paint);
  }

  auto display_list = builder.Build();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawLine-" +
                  std::to_string(state.range(0)) + ".png";
  surface_provider->Snapshot(filename);
}

// Draws a series of square rects of the requested width across
// the canvas and repeats until `kRectsToDraw` rects have been drawn.
//
// Half the drawn rects will not have an integral offset.
void BM_DrawRect(benchmark::State& state,
                 BackendType backend_type,
                 unsigned attributes) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kDrawRectFlags);

  size_t length = state.range(0);
  size_t canvas_size = length * 2;
  surface_provider->InitializeSurface(canvas_size, canvas_size);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  // As rects have DlScalar dimensions, we want to ensure that we also
  // draw rects with non-integer position and size
  const DlPoint offset(0.5f, 0.5f);
  DlPoint origin;
  DlSize size(length, length);

  state.counters["DrawCallCount"] = kRectsToDraw;
  for (size_t i = 0; i < kRectsToDraw; i++) {
    DlRect rect = DlRect::MakeOriginSize(origin, size);
    builder.DrawRect(rect, paint);
    origin += offset;
    if (origin.x + size.width > canvas_size) {
      origin.x -= canvas_size;
    }
    if (rect.GetBottom() > canvas_size) {
      origin.y -= canvas_size;
    }
  }

  auto display_list = builder.Build();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawRect-" +
                  std::to_string(state.range(0)) + ".png";
  surface_provider->Snapshot(filename);
}

// Draws a series of ovals of the requested height with aspect ratio 3:2 across
// the canvas and repeats until `kOvalsToDraw` ovals have been drawn.
//
// Half the drawn ovals will not have an integral offset.
void BM_DrawOval(benchmark::State& state,
                 BackendType backend_type,
                 unsigned attributes) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kDrawOvalFlags);

  size_t length = state.range(0);
  size_t canvas_size = length * 2;
  surface_provider->InitializeSurface(canvas_size, canvas_size);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  const DlPoint offset(0.5f, 0.5f);
  DlPoint origin;
  DlSize size(length * 1.5f, length);

  state.counters["DrawCallCount"] = kOvalsToDraw;
  for (size_t i = 0; i < kOvalsToDraw; i++) {
    DlRect rect = DlRect::MakeOriginSize(origin, size);
    builder.DrawOval(rect, paint);
    origin += offset;
    if (origin.x + size.width > canvas_size) {
      origin.x -= canvas_size;
    }
    if (origin.y + size.height > canvas_size) {
      origin.y -= canvas_size;
    }
  }
  auto display_list = builder.Build();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawOval-" +
                  std::to_string(state.range(0)) + ".png";
  surface_provider->Snapshot(filename);
}

// Draws a series of circles of the requested radius across
// the canvas and repeats until `kCirclesToDraw` circles have been drawn.
//
// Half the drawn circles will not have an integral center point.
void BM_DrawCircle(benchmark::State& state,
                   BackendType backend_type,
                   unsigned attributes) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kDrawCircleFlags);

  size_t length = state.range(0);
  size_t canvas_size = length * 2;
  surface_provider->InitializeSurface(canvas_size, canvas_size);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  DlScalar radius = length / 2.0f;
  const DlPoint offset(0.5f, 0.5f);

  DlPoint center = DlPoint(radius, radius);

  state.counters["DrawCallCount"] = kCirclesToDraw;
  for (size_t i = 0; i < kCirclesToDraw; i++) {
    builder.DrawCircle(center, radius, paint);
    center += offset;
    if (center.x + radius > canvas_size) {
      center.x = radius;
    }
    if (center.y + radius > canvas_size) {
      center.y = radius;
    }
  }
  auto display_list = builder.Build();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawCircle-" +
                  std::to_string(state.range(0)) + ".png";
  surface_provider->Snapshot(filename);
}

// Draws a series of rounded rects of the requested width across
// the canvas and repeats until `kRRectsToDraw` rects have been drawn.
//
// Half the drawn rounded rects will not have an integral offset.
void BM_DrawRRect(benchmark::State& state,
                  BackendType backend_type,
                  unsigned attributes,
                  RRectType type) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kDrawRRectFlags);

  size_t length = state.range(0);
  size_t canvas_size = length * 2;
  surface_provider->InitializeSurface(canvas_size, canvas_size);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  DlRoundingRadii radii;
  switch (type) {
    case RRectType::kSimple:
      radii.top_left = DlSize(5.0f, 5.0f);
      radii.top_right = DlSize(5.0f, 5.0f);
      radii.bottom_right = DlSize(5.0f, 5.0f);
      radii.bottom_left = DlSize(5.0f, 5.0f);
      break;
    case RRectType::kNinePatch:
      radii.top_left = DlSize(5.0f, 2.0f);
      radii.top_right = DlSize(3.0f, 2.0f);
      radii.bottom_right = DlSize(3.0f, 4.0f);
      radii.bottom_left = DlSize(5.0f, 4.0f);
      break;
    case RRectType::kComplex:
      radii.top_left = DlSize(5.0f, 4.0f);
      radii.top_right = DlSize(4.0f, 5.0f);
      radii.bottom_right = DlSize(3.0f, 6.0f);
      radii.bottom_left = DlSize(2.0f, 7.0f);
      break;
    default:
      FML_UNREACHABLE();
  }

  const DlScalar offset = 0.5f;
  const DlScalar multiplier = length / 16.0f;

  DlRoundRect rrect = DlRoundRect::MakeRectRadii(
      DlRect::MakeLTRB(0, 0, length, length), radii * multiplier);

  state.counters["DrawCallCount"] = kRRectsToDraw;
  for (size_t i = 0; i < kRRectsToDraw; i++) {
    builder.DrawRoundRect(rrect, paint);
    rrect = rrect.Shift(offset, offset);
    if (rrect.GetBounds().GetRight() > canvas_size) {
      rrect = rrect.Shift(-canvas_size, 0);
    }
    if (rrect.GetBounds().GetBottom() > canvas_size) {
      rrect = rrect.Shift(0, -canvas_size);
    }
  }
  auto display_list = builder.Build();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawRRect-" +
                  std::to_string(state.range(0)) + ".png";
  surface_provider->Snapshot(filename);
}

// Draws a series of "DR" rects of the requested width across
// the canvas and repeats until `kRRectsToDraw` rects have been drawn.
//
// A "DR" rect is a shape consisting of the difference between two
// rounded rects.
//
// Half the drawn DR rects will not have an integral offset.
void BM_DrawDRRect(benchmark::State& state,
                   BackendType backend_type,
                   unsigned attributes,
                   RRectType type) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kDrawDRRectFlags);

  size_t length = state.range(0);
  size_t canvas_size = length * 2;
  surface_provider->InitializeSurface(canvas_size, canvas_size);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  DlRoundingRadii radii;
  switch (type) {
    case RRectType::kSimple:
      radii.top_left = DlSize(5.0f, 5.0f);
      radii.top_right = DlSize(5.0f, 5.0f);
      radii.bottom_right = DlSize(5.0f, 5.0f);
      radii.bottom_left = DlSize(5.0f, 5.0f);
      break;
    case RRectType::kNinePatch:
      radii.top_left = DlSize(5.0f, 7.0f);
      radii.top_right = DlSize(3.0f, 7.0f);
      radii.bottom_right = DlSize(3.0f, 4.0f);
      radii.bottom_left = DlSize(5.0f, 4.0f);
      break;
    case RRectType::kComplex:
      radii.top_left = DlSize(5.0f, 4.0f);
      radii.top_right = DlSize(4.0f, 5.0f);
      radii.bottom_right = DlSize(3.0f, 6.0f);
      radii.bottom_left = DlSize(8.0f, 7.0f);
      break;
    default:
      FML_UNREACHABLE();
  }

  const DlScalar offset = 0.5f;
  const DlScalar multiplier = length / 16.0f;

  DlRoundRect rrect = DlRoundRect::MakeRectRadii(
      DlRect::MakeLTRB(0, 0, length, length), radii * multiplier);
  DlRoundRect rrect_2 = DlRoundRect::MakeRectRadii(
      DlRect::MakeLTRB(0, 0, length, length).Expand(-0.1f * length),
      radii * multiplier);

  state.counters["DrawCallCount"] = kDRRectsToDraw;
  for (size_t i = 0; i < kDRRectsToDraw; i++) {
    builder.DrawDiffRoundRect(rrect, rrect_2, paint);
    rrect = rrect.Shift(offset, offset);
    rrect_2 = rrect_2.Shift(offset, offset);
    if (rrect.GetBounds().GetRight() > canvas_size) {
      rrect = rrect.Shift(-canvas_size, 0);
      rrect_2 = rrect_2.Shift(-canvas_size, 0);
    }
    if (rrect.GetBounds().GetBottom() > canvas_size) {
      rrect = rrect.Shift(0, -canvas_size);
      rrect_2 = rrect_2.Shift(0, -canvas_size);
    }
  }
  auto display_list = builder.Build();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawDRRect-" +
                  std::to_string(state.range(0)) + ".png";
  surface_provider->Snapshot(filename);
}

void BM_DrawArc(benchmark::State& state,
                BackendType backend_type,
                unsigned attributes) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state,
                     DisplayListOpFlags::kDrawArcNoCenterFlags);

  size_t length = state.range(0);
  size_t canvas_size = length * 2;
  surface_provider->InitializeSurface(canvas_size, canvas_size);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  DlScalar starting_angle = 0.0f;
  DlPoint offset(0.5f, 0.5f);

  // Just some random sweeps that will mostly circumnavigate the circle
  std::vector<DlScalar> segment_sweeps = {5.5f,  -10.0f, 42.0f, 71.7f, 90.0f,
                                          37.5f, 17.9f,  32.0f, 379.4f};

  DlPoint origin;
  DlSize size(length, length);

  state.counters["DrawCallCount"] = kArcSweepSetsToDraw * segment_sweeps.size();
  for (size_t i = 0; i < kArcSweepSetsToDraw; i++) {
    DlRect bounds = DlRect::MakeOriginSize(origin, size);
    for (DlScalar sweep : segment_sweeps) {
      builder.DrawArc(bounds, starting_angle, sweep, false, paint);
      starting_angle += sweep + 5.0f;
    }
    origin += offset;
    if (origin.x + size.width > canvas_size) {
      origin.x -= canvas_size;
    }
    if (origin.y + size.height > canvas_size) {
      origin.y -= canvas_size;
    }
  }

  auto display_list = builder.Build();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawArc-" +
                  std::to_string(state.range(0)) + ".png";
  surface_provider->Snapshot(filename);
}

// Returns a list of DlPoints that represent `n` points equally spaced out
// along the circumference of a circle with radius `r` and centered on `center`.
std::vector<DlPoint> GetPolygonPoints(size_t n, DlPoint center, DlScalar r) {
  std::vector<DlPoint> points;
  DlScalar x, y;
  float angle;
  float full_circle = 2.0f * M_PI;
  for (size_t i = 0; i < n; i++) {
    angle = (full_circle / static_cast<float>(n)) * static_cast<float>(i);
    x = center.x + r * std::cosf(angle);
    y = center.y + r * std::sinf(angle);
    points.emplace_back(x, y);
  }
  return points;
}

// Creates a path that represents a regular polygon with `sides` sides,
// centered on `center` with a radius of `radius`. The control points are
// equally spaced out along the circumference of the circle described by
// `radius` and `center`.
//
// The path segment connecting each control point is a line segment.
void GetLinesPath(DlPathBuilder& path_builder,
                  size_t sides,
                  DlPoint center,
                  float radius) {
  std::vector<DlPoint> points = GetPolygonPoints(sides, center, radius);
  path_builder.MoveTo(points[0]);
  for (size_t i = 1; i < sides; i++) {
    path_builder.LineTo(points[i]);
  }
  path_builder.LineTo(points[0]);
  path_builder.Close();
}

// Creates a path that represents a regular polygon with `sides` sides,
// centered on `center` with a radius of `radius`. The control points are
// equally spaced out along the circumference of the circle described by
// `radius` and `center`.
//
// The path segment connecting each control point is a quad bezier, with the
// bezier control point being on a circle with 80% of `radius` and with the
// control point angle half way between the start and end point angles for the
// polygon segment.
void GetQuadsPath(DlPathBuilder& path_builder,
                  size_t sides,
                  DlPoint center,
                  float radius) {
  std::vector<DlPoint> points = GetPolygonPoints(sides, center, radius);
  std::vector<DlPoint> control_points =
      GetPolygonPoints(sides * 2, center, radius * 0.8f);

  path_builder.MoveTo(points[0]);
  for (size_t i = 1; i < sides; i++) {
    path_builder.QuadraticCurveTo(control_points[2 * i - 1], points[i]);
  }
  path_builder.QuadraticCurveTo(control_points[2 * sides - 1], points[0]);
  path_builder.Close();
}

// Creates a path that represents a regular polygon with `sides` sides,
// centered on `center` with a radius of `radius`. The control points are
// equally spaced out along the circumference of the circle described by
// `radius` and `center`.
//
// The path segment connecting each control point is a conic, with the
// control point being on a circle with 80% of `radius` and with the
// control point angle half way between the start and end point angles for the
// polygon segment, and the conic weight set to 3.7f.
void GetConicsPath(DlPathBuilder& path_builder,
                   size_t sides,
                   DlPoint center,
                   float radius) {
  std::vector<DlPoint> points = GetPolygonPoints(sides, center, radius);
  std::vector<DlPoint> control_points =
      GetPolygonPoints(sides * 2, center, radius * 0.8f);

  path_builder.MoveTo(points[0]);
  for (size_t i = 1; i < sides; i++) {
    path_builder.ConicCurveTo(control_points[2 * i - 1], points[i], 3.7f);
  }
  path_builder.ConicCurveTo(control_points[2 * sides - 1], points[0], 3.7f);
  path_builder.Close();
}

// Creates a path that represents a regular polygon with `sides` sides,
// centered on `center` with a radius of `radius`. The control points are
// equally spaced out along the circumference of the circle described by
// `radius` and `center`.
//
// The path segment connecting each control point is a cubic, with the first
// control point being on a circle with 80% of `radius` and with the second
// control point being on a circle with 120% of `radius`. The first
// control point is 1/3, and the second control point is 2/3, of the angle
// between the start and end point angles for the polygon segment.
void GetCubicsPath(DlPathBuilder& path_builder,
                   size_t sides,
                   DlPoint center,
                   float radius) {
  std::vector<DlPoint> points = GetPolygonPoints(sides, center, radius);
  std::vector<DlPoint> inner_control_points =
      GetPolygonPoints(sides * 3, center, radius * 0.8f);
  std::vector<DlPoint> outer_control_points =
      GetPolygonPoints(sides * 3, center, radius * 1.2f);

  path_builder.MoveTo(points[0]);
  for (size_t i = 1; i < sides; i++) {
    path_builder.CubicCurveTo(inner_control_points[3 * i - 2],
                              outer_control_points[3 * i - 1], points[i]);
  }
  path_builder.CubicCurveTo(inner_control_points[3 * sides - 2],
                            outer_control_points[3 * sides - 1], points[0]);
  path_builder.Close();
}

// Returns a path generated by one of the above path generators
// which is multiplied `number` times centered on each of the `number` control
// points along the circumference of a circle centered on `center` with radius
// `radius`.
//
// Each of the polygons will have `sides` sides, and the resulting path will be
// bounded by a circle with radius of 150% of `radius` (or another 20% on top of
// that for cubics)
void MultiplyPath(DlPathBuilder& path_builder,
                  PathVerb type,
                  DlPoint center,
                  size_t sides,
                  size_t number,
                  float radius) {
  std::vector<DlPoint> center_points =
      GetPolygonPoints(number, center, radius / 2.0f);

  for (DlPoint p : center_points) {
    switch (type) {
      case PathVerb::kLine:
        GetLinesPath(path_builder, sides, p, radius);
        break;
      case PathVerb::kQuad:
        GetQuadsPath(path_builder, sides, p, radius);
        break;
      case PathVerb::kConic:
        GetConicsPath(path_builder, sides, p, radius);
        break;
      case PathVerb::kCubic:
        GetCubicsPath(path_builder, sides, p, radius);
        break;
      default:
        break;
    }
  }
}

std::string VerbToString(PathVerb type) {
  switch (type) {
    case PathVerb::kLine:
      return "Lines";
    case PathVerb::kQuad:
      return "Quads";
    case PathVerb::kConic:
      return "Conics";
    case PathVerb::kCubic:
      return "Cubics";
    default:
      return "Unknown";
  }
}

// Draws a series of overlapping 20-sided polygons where the path segment
// between each point is one of the verb types defined in PathVerb.
//
// The number of polygons drawn will be varied to get an overall path
// with approximately 20*N verbs, so we can get an idea of the fixed
// cost of using drawPath as well as an idea of how the cost varies according
// to the verb count.
void BM_DrawPath(benchmark::State& state,
                 BackendType backend_type,
                 unsigned attributes,
                 PathVerb type) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kDrawPathFlags);

  size_t length = kFixedCanvasSize;
  surface_provider->InitializeSurface(length, length);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  DlPathBuilder path_builder;

  std::string label = VerbToString(type);
  DlPoint center = DlPoint(length / 2.0f, length / 2.0f);
  float radius = length * 0.25f;
  state.SetComplexityN(state.range(0));

  MultiplyPath(path_builder, type, center, 20, state.range(0), radius);
  DlPath path = DlPath(path_builder);

  state.counters["VerbCount"] = path.GetPath().GetComponentCount();
  state.counters["DrawCallCount"] = 1;

  builder.DrawPath(path, paint);
  auto display_list = builder.Build();

  // Prime any path conversions
  canvas->DrawDisplayList(display_list);
  surface->FlushSubmitCpuSync();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawPath-" + label +
                  "-" + std::to_string(state.range(0)) + ".png";
  surface_provider->Snapshot(filename);
}

// Returns a set of vertices that describe a circle that has a
// radius of `radius` and outer vertex count of approximately
// `vertex_count`. The final number of vertices will differ as we
// need to ensure the correct usage of vertices to ensure we do not
// request degenerate triangles be drawn. This final count is output
// through `final_vertex_count`.
//
// The resulting vertices will describe a disc consisting of a series
// of triangles with two vertices on the circumference of the disc,
// and the final vertex being the center point of the disc.
//
// Each vertex colour will alternate through Red, Green, Blue and Cyan.
std::shared_ptr<DlVertices> GetTestVertices(DlPoint center,
                                            float radius,
                                            size_t vertex_count,
                                            DlVertexMode mode,
                                            size_t& final_vertex_count) {
  size_t outer_vertex_count = vertex_count / 2;
  std::vector<DlPoint> outer_points =
      GetPolygonPoints(outer_vertex_count, center, radius);

  std::vector<DlPoint> vertices;
  std::vector<DlColor> colors;

  switch (mode) {
    case DlVertexMode::kTriangleFan:
      // Calling the points on the outer circle O_0, O_1, O_2, ..., and
      // the center point C, this should create a triangle fan with vertices
      // C, O_0, O_1, O_2, O_3, ...
      vertices.push_back(center);
      colors.push_back(DlColor(SK_ColorCYAN));
      for (size_t i = 0; i <= outer_points.size(); i++) {
        vertices.push_back(outer_points[i % outer_points.size()]);
        if (i % 3 == 0) {
          colors.push_back(DlColor(SK_ColorRED));
        } else if (i % 3 == 1) {
          colors.push_back(DlColor(SK_ColorGREEN));
        } else {
          colors.push_back(DlColor(SK_ColorBLUE));
        }
      }
      break;
    case DlVertexMode::kTriangles:
      // Calling the points on the outer circle O_0, O_1, O_2, ..., and
      // the center point C, this should create a series of triangles with
      // vertices O_0, O_1, C, O_1, O_2, C, O_2, O_3, C, ...
      for (size_t i = 0; i < outer_vertex_count; i++) {
        vertices.push_back(outer_points[i % outer_points.size()]);
        colors.push_back(DlColor(SK_ColorRED));
        vertices.push_back(outer_points[(i + 1) % outer_points.size()]);
        colors.push_back(DlColor(SK_ColorGREEN));
        vertices.push_back(center);
        colors.push_back(DlColor(SK_ColorBLUE));
      }
      break;
    case DlVertexMode::kTriangleStrip:
      // Calling the points on the outer circle O_0, O_1, O_2, ..., and
      // the center point C, this should create a strip with vertices
      // O_0, O_1, C, O_2, O_3, C, O_4, O_5, C, ...
      for (size_t i = 0; i <= outer_vertex_count; i++) {
        vertices.push_back(outer_points[i % outer_points.size()]);
        colors.push_back(i % 2 ? DlColor(SK_ColorRED) : DlColor(SK_ColorGREEN));
        if (i % 2 == 1) {
          vertices.push_back(center);
          colors.push_back(DlColor(SK_ColorBLUE));
        }
      }
      break;
    default:
      break;
  }

  final_vertex_count = vertices.size();
  return DlVertices::Make(mode, vertices.size(), vertices.data(), nullptr,
                          colors.data());
}

std::string VertexModeToString(DlVertexMode mode) {
  switch (mode) {
    case DlVertexMode::kTriangleStrip:
      return "TriangleStrip";
    case DlVertexMode::kTriangleFan:
      return "TriangleFan";
    case DlVertexMode::kTriangles:
      return "Triangles";
  }
  return "Unknown";
}

// Draws a series of discs generated by `GetTestVertices()` with
// 50 vertices in each disc. The number of discs drawn will vary according
// to the benchmark input, and the benchmark will automatically calculate
// the Big-O complexity of `DrawVertices` with N being the number of vertices
// being drawn.
//
// The discs drawn will be centered on points along a circle with radius of 25%
// of the canvas width/height, with each point being equally spaced out.
void BM_DrawVertices(benchmark::State& state,
                     BackendType backend_type,
                     unsigned attributes,
                     DlVertexMode mode) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kDrawVerticesFlags);

  size_t length = kFixedCanvasSize;
  surface_provider->InitializeSurface(length, length);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  DlPoint center = DlPoint(length / 2.0f, length / 2.0f);

  float radius = length / 4.0f;

  size_t vertex_count, total_vertex_count = 0;
  size_t disc_count = state.range(0);

  std::vector<DlPoint> center_points =
      GetPolygonPoints(disc_count, center, radius / 4.0f);

  state.counters["DrawCallCount"] = center_points.size();
  for (DlPoint p : center_points) {
    std::shared_ptr<DlVertices> vertices =
        GetTestVertices(p, radius, 50, mode, vertex_count);
    total_vertex_count += vertex_count;
    builder.DrawVertices(vertices, DlBlendMode::kSrc, paint);
  }

  state.counters["VertexCount"] = total_vertex_count;
  state.SetComplexityN(total_vertex_count);

  auto display_list = builder.Build();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawVertices-" +
                  std::to_string(disc_count) + "-" + VertexModeToString(mode) +
                  ".png";
  surface_provider->Snapshot(filename);
}

// Generate `count` test points.
//
// The points are distributed using some fixed constant offsets that were
// chosen to appear somewhat random.
//
// The points generated will wrap in x and y for the bounds of `canvas_size`.
std::vector<DlPoint> GetTestPoints(size_t count, DlISize canvas_size) {
  std::vector<DlPoint> points;

  // Some arbitrary offsets to use when building the list of points
  std::vector<DlScalar> delta_x = {10.0f, 6.3f, 15.0f, 3.5f, 22.6f, 4.7f};
  std::vector<DlScalar> delta_y = {9.3f, -5.4f, 8.5f, -12.0f, 19.2f, -19.6f};

  DlPoint current;
  for (size_t i = 0; i < count; i++) {
    points.push_back(current);
    current +=
        DlPoint(delta_x[i % delta_x.size()], delta_y[i % delta_y.size()]);
    if (current.x > canvas_size.width) {
      current += DlPoint(-canvas_size.width, 25.0f);
    }
    if (current.y > canvas_size.height) {
      current += DlPoint(0.0f, -canvas_size.height);
    }
  }

  return points;
}

std::string PointModeToString(DlPointMode mode) {
  switch (mode) {
    case DlPointMode::kLines:
      return "Lines";
    case DlPointMode::kPolygon:
      return "Polygon";
    case DlPointMode::kPoints:
    default:
      return "Points";
  }
}

// Draws a series of points generated by `GetTestPoints()` above to
// a fixed-size canvas. The benchmark will vary the number of points drawn,
// and they can be drawn in one of three modes - Lines, Polygon or Points mode.
//
// This benchmark will automatically calculate the Big-O complexity of
// `DrawPoints` with N being the number of points being drawn.
void BM_DrawPoints(benchmark::State& state,
                   BackendType backend_type,
                   unsigned attributes,
                   DlPointMode mode) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  switch (mode) {
    case DlPointMode::kPoints:
      AnnotateAttributes(attributes, state,
                         DisplayListOpFlags::kDrawPointsAsPointsFlags);
      break;
    case DlPointMode::kLines:
      AnnotateAttributes(attributes, state,
                         DisplayListOpFlags::kDrawPointsAsLinesFlags);
      break;
    case DlPointMode::kPolygon:
      AnnotateAttributes(attributes, state,
                         DisplayListOpFlags::kDrawPointsAsPolygonFlags);
      break;
  }

  size_t length = kFixedCanvasSize;
  surface_provider->InitializeSurface(length, length);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  size_t point_count = state.range(0);
  state.SetComplexityN(point_count);
  state.counters["PointCount"] = point_count;
  state.counters["DrawCallCount"] = 1;

  std::vector<DlPoint> points =
      GetTestPoints(point_count, DlISize(length, length));
  builder.DrawPoints(mode, points.size(), points.data(), paint);

  auto display_list = builder.Build();

  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawPoints-" +
                  PointModeToString(mode) + "-" + std::to_string(point_count) +
                  ".png";
  surface_provider->Snapshot(filename);
}

sk_sp<SkImage> ImageFromBitmapWithNewID(const SkBitmap& bitmap) {
  // If we create an SkPixmap with a ref to the SkBitmap's pixel data,
  // then create an SkImage from that, we always get a new generation ID,
  // so we will avoid hitting the cache.
  SkPixmap pixmap;
  bitmap.peekPixels(&pixmap);
  return SkImages::RasterFromPixmap(pixmap, nullptr, nullptr);
}

// Draws `kImagesToDraw` bitmaps to a canvas, either with texture-backed
// bitmaps or bitmaps that need to be uploaded to the GPU first.
void BM_DrawImage(benchmark::State& state,
                  BackendType backend_type,
                  unsigned attributes,
                  DlImageSampling options,
                  bool upload_bitmap) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state,
                     DisplayListOpFlags::kDrawImageWithPaintFlags);

  size_t bitmap_size = state.range(0);
  size_t canvas_size = 2 * bitmap_size;
  surface_provider->InitializeSurface(canvas_size, canvas_size);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  sk_sp<SkImage> image;
  std::shared_ptr<DlSurfaceInstance> offscreen_instance;
  sk_sp<SkSurface> offscreen;
  SkBitmap bitmap;

  if (upload_bitmap) {
    SkImageInfo info = SkImageInfo::Make(bitmap_size, bitmap_size,
                                         SkColorType::kRGBA_8888_SkColorType,
                                         SkAlphaType::kPremul_SkAlphaType);
    bitmap.allocPixels(info, 0);
    bitmap.eraseColor(SK_ColorBLUE);
  } else {
    offscreen_instance =
        surface_provider->MakeOffscreenSurface(bitmap_size, bitmap_size);
    offscreen = offscreen_instance->sk_surface();
    offscreen->getCanvas()->clear(SK_ColorRED);
  }

  const DlPoint offset(0.5f, 0.5f);
  DlPoint dst;

  state.counters["DrawCallCount"] = kImagesToDraw;
  for (size_t i = 0; i < kImagesToDraw; i++) {
    image = upload_bitmap ? ImageFromBitmapWithNewID(bitmap)
                          : offscreen->makeImageSnapshot();
    builder.DrawImage(DlImage::Make(image), dst, options, &paint);

    dst += offset;
    if (dst.x + bitmap_size > canvas_size) {
      dst.x = 0;
    }
    if (dst.y + bitmap_size > canvas_size) {
      dst.y = 0;
    }
  }

  auto display_list = builder.Build();

  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawImage-" +
                  (upload_bitmap ? "Upload-" : "Texture-") +
                  std::to_string(bitmap_size) + ".png";
  surface_provider->Snapshot(filename);
}

std::string ConstraintToString(DlSrcRectConstraint constraint) {
  switch (constraint) {
    case DlSrcRectConstraint::kStrict:
      return "Strict";
    case DlSrcRectConstraint::kFast:
      return "Fast";
    default:
      return "Unknown";
  }
}

// Draws `kImagesToDraw` bitmaps to a canvas, either with texture-backed
// bitmaps or bitmaps that need to be uploaded to the GPU first.
//
// The bitmaps are shrunk down to 75% of their size when rendered to the canvas.
void BM_DrawImageRect(benchmark::State& state,
                      BackendType backend_type,
                      unsigned attributes,
                      DlImageSampling options,
                      DlSrcRectConstraint constraint,
                      bool upload_bitmap) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state,
                     DisplayListOpFlags::kDrawImageRectWithPaintFlags);

  size_t bitmap_size = state.range(0);
  size_t canvas_size = 2 * bitmap_size;
  surface_provider->InitializeSurface(canvas_size, canvas_size);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  sk_sp<SkImage> image;
  std::shared_ptr<DlSurfaceInstance> offscreen_instance;
  sk_sp<SkSurface> offscreen;
  SkBitmap bitmap;

  if (upload_bitmap) {
    SkImageInfo info = SkImageInfo::Make(bitmap_size, bitmap_size,
                                         SkColorType::kRGBA_8888_SkColorType,
                                         SkAlphaType::kPremul_SkAlphaType);
    bitmap.allocPixels(info, 0);
    bitmap.eraseColor(SK_ColorBLUE);
  } else {
    offscreen_instance =
        surface_provider->MakeOffscreenSurface(bitmap_size, bitmap_size);
    offscreen = offscreen_instance->sk_surface();
    offscreen->getCanvas()->clear(SK_ColorRED);
  }

  const DlPoint offset(0.5f, 0.5f);
  DlRect src = DlRect::MakeXYWH(bitmap_size / 4.0f, bitmap_size / 4.0f,
                                bitmap_size / 2.0f, bitmap_size / 2.0f);
  DlPoint origin;
  DlSize size(bitmap_size * 0.75f, bitmap_size * 0.75f);

  state.counters["DrawCallCount"] = kImagesToDraw;
  for (size_t i = 0; i < kImagesToDraw; i++) {
    image = upload_bitmap ? ImageFromBitmapWithNewID(bitmap)
                          : offscreen->makeImageSnapshot();
    DlRect dst = DlRect::MakeOriginSize(origin, size);
    builder.DrawImageRect(DlImage::Make(image), src, dst, options, &paint,
                          constraint);
    origin += offset;
    if (origin.x + size.width > canvas_size) {
      origin.x = 0.0f;
    }
    if (origin.y + size.height > canvas_size) {
      origin.y = 0.0f;
    }
  }

  auto display_list = builder.Build();

  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawImageRect-" +
                  (upload_bitmap ? "Upload-" : "Texture-") +
                  ConstraintToString(constraint) + "-" +
                  std::to_string(bitmap_size) + ".png";
  surface_provider->Snapshot(filename);
}

std::string FilterModeToString(const DlFilterMode mode) {
  switch (mode) {
    case DlFilterMode::kNearest:
      return "Nearest";
    case DlFilterMode::kLinear:
      return "Linear";
    default:
      return "Unknown";
  }
}

// Draws `kImagesToDraw` bitmaps to a canvas, either with texture-backed
// bitmaps or bitmaps that need to be uploaded to the GPU first.
//
// The image is split into 9 sub-rects and stretched proportionally for final
// rendering.
void BM_DrawImageNine(benchmark::State& state,
                      BackendType backend_type,
                      unsigned attributes,
                      const DlFilterMode filter,
                      bool upload_bitmap) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state,
                     DisplayListOpFlags::kDrawImageNineWithPaintFlags);

  size_t bitmap_size = state.range(0);
  size_t canvas_size = 2 * bitmap_size;
  surface_provider->InitializeSurface(canvas_size, canvas_size);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  DlIRect center = DlIRect::MakeXYWH(bitmap_size / 4, bitmap_size / 4,
                                     bitmap_size / 2, bitmap_size / 2);

  sk_sp<SkImage> image;
  std::shared_ptr<DlSurfaceInstance> offscreen_instance;
  sk_sp<SkSurface> offscreen;
  SkBitmap bitmap;

  if (upload_bitmap) {
    SkImageInfo info = SkImageInfo::Make(bitmap_size, bitmap_size,
                                         SkColorType::kRGBA_8888_SkColorType,
                                         SkAlphaType::kPremul_SkAlphaType);
    bitmap.allocPixels(info, 0);
    bitmap.eraseColor(SK_ColorBLUE);
  } else {
    offscreen_instance =
        surface_provider->MakeOffscreenSurface(bitmap_size, bitmap_size);
    offscreen = offscreen_instance->sk_surface();
    offscreen->getCanvas()->clear(SK_ColorRED);
  }

  const DlPoint offset(0.5f, 0.5f);
  DlPoint origin;
  DlSize size(bitmap_size * 0.75f, bitmap_size * 0.75f);

  state.counters["DrawCallCount"] = kImagesToDraw;
  for (size_t i = 0; i < kImagesToDraw; i++) {
    image = upload_bitmap ? ImageFromBitmapWithNewID(bitmap)
                          : offscreen->makeImageSnapshot();
    DlRect dst = DlRect::MakeOriginSize(origin, size);
    builder.DrawImageNine(DlImage::Make(image), center, dst, filter, &paint);
    origin += offset;
    if (origin.x + size.width > canvas_size) {
      origin.x = 0.0f;
    }
    if (origin.y + size.height > canvas_size) {
      origin.y = 0.0f;
    }
  }

  auto display_list = builder.Build();

  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawImageNine-" +
                  (upload_bitmap ? "Upload-" : "Texture-") +
                  FilterModeToString(filter) + "-" +
                  std::to_string(bitmap_size) + ".png";
  surface_provider->Snapshot(filename);
}

// Draws a series of glyph runs with 32 glyphs in each run. The number of runs
// may vary according to the benchmark parameters. The text will start in the
// upper left corner of the canvas and advance from left to right and wrap at
// the canvas boundaries in both x and y.
//
// This benchmark will automatically calculate the Big-O complexity of
// `DrawTextBlob` with N being the number of glyphs being drawn.
void BM_DrawTextBlob(benchmark::State& state,
                     BackendType backend_type,
                     unsigned attributes) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kDrawTextBlobFlags);

  size_t draw_calls = state.range(0);
  size_t canvas_size = kFixedCanvasSize;
  surface_provider->InitializeSurface(canvas_size, canvas_size);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  state.SetComplexityN(draw_calls);
  state.counters["DrawCallCount_Varies"] = draw_calls;
  state.counters["GlyphCount"] = draw_calls;
  char character[2] = {'A', '\0'};

  for (size_t i = 0; i < draw_calls; i++) {
    character[0] = 'A' + (i % 26);
    auto blob = SkTextBlob::MakeFromString(character, CreateTestFontOfSize(20));
    builder.DrawTextBlob(blob, 50.0f, 50.0f, paint);
  }

  auto display_list = builder.Build();

  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawTextBlob-" +
                  std::to_string(draw_calls) + ".png";
  surface_provider->Snapshot(filename);
}

// Draw the shadow for a 10-sided regular polygon where the polygon's
// sides are denoted by one of a Line, Quad, Conic or Cubic path segment.
//
// The elevation of the light source will vary according to the benchmark
// paremeters.
//
// The benchmark can be run with either a transparent occluder or an opaque
// occluder.
void BM_DrawShadow(benchmark::State& state,
                   BackendType backend_type,
                   unsigned attributes,
                   bool transparent_occluder,
                   PathVerb type) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kDrawShadowFlags);

  size_t length = kFixedCanvasSize;
  surface_provider->InitializeSurface(length, length);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  DlPathBuilder path_builder;

  DlPoint center = DlPoint(length / 2.0f, length / 2.0f);
  float radius = length * 0.25f;

  switch (type) {
    case PathVerb::kLine:
      GetLinesPath(path_builder, 10, center, radius);
      break;
    case PathVerb::kQuad:
      GetQuadsPath(path_builder, 10, center, radius);
      break;
    case PathVerb::kConic:
      GetConicsPath(path_builder, 10, center, radius);
      break;
    case PathVerb::kCubic:
      GetCubicsPath(path_builder, 10, center, radius);
      break;
    default:
      break;
  }

  float elevation = state.range(0);
  state.counters["DrawCallCount"] = 1;

  DlPath path = DlPath(path_builder);

  // We can hardcode dpr to 1.0f as we're varying elevation, and dpr is only
  // ever used in conjunction with elevation.
  builder.DrawShadow(path, DlColor(SK_ColorBLUE), elevation,
                     transparent_occluder, 1.0f);
  auto display_list = builder.Build();

  // Prime the path conversion.
  canvas->DrawDisplayList(display_list);
  surface->FlushSubmitCpuSync();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-DrawShadow-" +
                  VerbToString(type) + "-" +
                  (transparent_occluder ? "Transparent-" : "Opaque-") +
                  std::to_string(elevation) + "-" + ".png";
  surface_provider->Snapshot(filename);
}

// Calls saveLayer N times from the root canvas layer, and optionally calls
// saveLayer a further M times nested inside that top saveLayer call.
//
// The total number of saveLayer calls will be N * (M+1).
//
// In each saveLayer call, simply draw the colour red with no clip rect.
void BM_SaveLayer(benchmark::State& state,
                  BackendType backend_type,
                  unsigned attributes,
                  size_t save_depth) {
  auto surface_provider = DlSurfaceProvider::Create(backend_type);
  DisplayListBuilder builder;
  DlPaint paint = GetPaintForRun(attributes);

  AnnotateAttributes(attributes, state, DisplayListOpFlags::kSaveLayerFlags);

  size_t length = kFixedCanvasSize;
  surface_provider->InitializeSurface(length, length);
  auto surface = surface_provider->GetPrimarySurface();
  auto canvas = surface->GetCanvas();

  size_t save_layer_calls = state.range(0);

  // Ensure we draw two overlapping rects to avoid any peephole optimisations
  DlRect rect1 = DlRect::MakeLTRB(0, 0, 0.75f * length, 0.75f * length);
  DlRect rect2 =
      DlRect::MakeLTRB(0.25f * length, 0.25f * length, length, length);

  state.counters["DrawCallCount_Varies"] = save_layer_calls * save_depth;
  for (size_t i = 0; i < save_layer_calls; i++) {
    for (size_t j = 0; j < save_depth; j++) {
      builder.SaveLayer(std::nullopt, nullptr);
      builder.DrawRect(rect1, paint);
      builder.DrawRect(rect2, paint);
    }
    for (size_t j = 0; j < save_depth; j++) {
      builder.Restore();
    }
  }
  auto display_list = builder.Build();

  // We only want to time the actual rasterization.
  for ([[maybe_unused]] auto _ : state) {
    canvas->DrawDisplayList(display_list);
    surface->FlushSubmitCpuSync();
  }

  auto filename = surface_provider->backend_name() + "-SaveLayer-" +
                  std::to_string(save_depth) + "-" +
                  std::to_string(save_layer_calls) + ".png";
  surface_provider->Snapshot(filename);
}

#ifdef ENABLE_SOFTWARE_BENCHMARKS
RUN_DISPLAYLIST_BENCHMARKS(Software)
#endif

#ifdef ENABLE_OPENGL_BENCHMARKS
RUN_DISPLAYLIST_BENCHMARKS(OpenGL)
#endif

#ifdef ENABLE_METAL_BENCHMARKS
RUN_DISPLAYLIST_BENCHMARKS(Metal)
#endif

}  // namespace testing
}  // namespace flutter

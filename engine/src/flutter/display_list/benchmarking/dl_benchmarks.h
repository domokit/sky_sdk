// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_BENCHMARKING_DL_BENCHMARKS_H_
#define FLUTTER_DISPLAY_LIST_BENCHMARKING_DL_BENCHMARKS_H_

#include "flutter/display_list/dl_canvas.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/dl_vertices.h"
#include "flutter/display_list/testing/dl_test_surface_provider.h"

#include "third_party/benchmark/include/benchmark/benchmark.h"

namespace flutter {
namespace testing {

enum BenchmarkAttributes {
  kEmpty = 0,
  kStrokedStyle = 1 << 0,
  kFilledStyle = 1 << 1,
  kHairlineStroke = 1 << 2,
  kAntiAliasing = 1 << 3
};

enum class RRectType {
  kSimple,
  kNinePatch,
  kComplex,
};

enum class PathVerb {
  kLine,
  kQuad,
  kConic,
  kCubic,
};

DlPaint GetPaintForRun(unsigned attributes);

using BackendType = DlSurfaceProvider::BackendType;

// Benchmarks

void BM_DrawLine(benchmark::State& state,
                 BackendType backend_type,
                 unsigned attributes);
void BM_DrawRect(benchmark::State& state,
                 BackendType backend_type,
                 unsigned attributes);
void BM_DrawCircle(benchmark::State& state,
                   BackendType backend_type,
                   unsigned attributes);
void BM_DrawOval(benchmark::State& state,
                 BackendType backend_type,
                 unsigned attributes);
void BM_DrawArc(benchmark::State& state,
                BackendType backend_type,
                unsigned attributes);
void BM_DrawRRect(benchmark::State& state,
                  BackendType backend_type,
                  unsigned attributes,
                  RRectType type);
void BM_DrawDRRect(benchmark::State& state,
                   BackendType backend_type,
                   unsigned attributes,
                   RRectType type);
void BM_DrawPath(benchmark::State& state,
                 BackendType backend_type,
                 unsigned attributes,
                 PathVerb type);
void BM_DrawPoints(benchmark::State& state,
                   BackendType backend_type,
                   unsigned attributes,
                   DlPointMode mode);
void BM_DrawVertices(benchmark::State& state,
                     BackendType backend_type,
                     unsigned attributes,
                     DlVertexMode mode);
void BM_DrawImage(benchmark::State& state,
                  BackendType backend_type,
                  unsigned attributes,
                  DlImageSampling options,
                  bool upload_bitmap);
void BM_DrawImageRect(benchmark::State& state,
                      BackendType backend_type,
                      unsigned attributes,
                      DlImageSampling options,
                      DlSrcRectConstraint constraint,
                      bool upload_bitmap);
void BM_DrawImageNine(benchmark::State& state,
                      BackendType backend_type,
                      unsigned attributes,
                      const DlFilterMode filter,
                      bool upload_bitmap);
void BM_DrawTextBlob(benchmark::State& state,
                     BackendType backend_type,
                     unsigned attributes);
void BM_DrawShadow(benchmark::State& state,
                   BackendType backend_type,
                   unsigned attributes,
                   bool transparent_occluder,
                   PathVerb type);
void BM_SaveLayer(benchmark::State& state,
                  BackendType backend_type,
                  unsigned attributes,
                  size_t save_depth);
// clang-format off

// DrawLine
#define DRAW_LINE_BENCHMARKS(BACKEND, ATTRIBUTES)                       \
  BENCHMARK_CAPTURE(BM_DrawLine, BACKEND,                               \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES)                                         \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 2048)                                                 \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawRect
#define DRAW_RECT_BENCHMARKS(BACKEND, ATTRIBUTES)                       \
  BENCHMARK_CAPTURE(BM_DrawRect, BACKEND,                               \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES)                                         \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 2048)                                                 \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawOval
#define DRAW_OVAL_BENCHMARKS(BACKEND, ATTRIBUTES)                       \
  BENCHMARK_CAPTURE(BM_DrawOval, BACKEND,                               \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES)                                         \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 2048)                                                 \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawCircle
#define DRAW_CIRCLE_BENCHMARKS(BACKEND, ATTRIBUTES)                     \
  BENCHMARK_CAPTURE(BM_DrawCircle, BACKEND,                             \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES)                                         \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 2048)                                                 \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawArc
#define DRAW_ARC_BENCHMARKS(BACKEND, ATTRIBUTES)                        \
  BENCHMARK_CAPTURE(BM_DrawArc, BACKEND,                                \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES)                                         \
      ->RangeMultiplier(2)                                              \
      ->Range(128, 2048)                                                \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawPath
#define DRAW_PATH_BENCHMARKS(BACKEND, ATTRIBUTES)                       \
  BENCHMARK_CAPTURE(BM_DrawPath,                                        \
                    Lines/BACKEND,                                      \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    PathVerb::kLine)                                    \
      ->RangeMultiplier(2)                                              \
      ->Range(8, 512)                                                   \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond)                                   \
      ->Complexity();                                                   \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawPath,                                        \
                    Quads/BACKEND,                                      \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    PathVerb::kQuad)                                    \
      ->RangeMultiplier(2)                                              \
      ->Range(8, 512)                                                   \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond)                                   \
      ->Complexity();                                                   \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawPath,                                        \
                    Conics/BACKEND,                                     \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    PathVerb::kConic)                                   \
      ->RangeMultiplier(2)                                              \
      ->Range(8, 512)                                                   \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond)                                   \
      ->Complexity();                                                   \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawPath,                                        \
                    Cubics/BACKEND,                                     \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    PathVerb::kCubic)                                   \
      ->RangeMultiplier(2)                                              \
      ->Range(8, 512)                                                   \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond)                                   \
      ->Complexity();

// DrawPoints
#define DRAW_POINTS_BENCHMARKS(BACKEND, ATTRIBUTES)                     \
  BENCHMARK_CAPTURE(BM_DrawPoints, Points/BACKEND,                      \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlPointMode::kPoints)                               \
      ->RangeMultiplier(2)                                              \
      ->Range(1024, 32768)                                              \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawPoints, Lines/BACKEND,                       \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlPointMode::kLines)                                \
      ->RangeMultiplier(2)                                              \
      ->Range(1024, 32768)                                              \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawPoints, Polygon/BACKEND,                     \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlPointMode::kPolygon)                              \
      ->RangeMultiplier(2)                                              \
      ->Range(1024, 32768)                                              \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawVertices
#define DRAW_VERTICES_BENCHMARKS(BACKEND, ATTRIBUTES)                   \
  BENCHMARK_CAPTURE(BM_DrawVertices,                                    \
                    TriangleStrip/BACKEND,                              \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlVertexMode::kTriangleStrip)                       \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 2048)                                                 \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond)                                   \
      ->Complexity();                                                   \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawVertices,                                    \
                    TriangleFan/BACKEND,                                \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlVertexMode::kTriangleFan)                         \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 2048)                                                 \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond)                                   \
      ->Complexity();                                                   \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawVertices,                                    \
                    Triangles/BACKEND,                                  \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlVertexMode::kTriangles)                           \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 2048)                                                 \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond)                                   \
      ->Complexity();

// DrawRRect
#define DRAW_RRECT_BENCHMARKS(BACKEND, ATTRIBUTES)                      \
  BENCHMARK_CAPTURE(BM_DrawRRect, Symmetric/BACKEND,                    \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    RRectType::kSimple)                                 \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawRRect, NinePatch/BACKEND,                    \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    RRectType::kNinePatch)                              \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawRRect, Complex/BACKEND,                      \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    RRectType::kComplex)                                \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawDRRect
#define DRAW_DRRECT_BENCHMARKS(BACKEND, ATTRIBUTES)                     \
  BENCHMARK_CAPTURE(BM_DrawDRRect, Symmetric/BACKEND,                   \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    RRectType::kSimple)                                 \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawDRRect, NinePatch/BACKEND,                   \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    RRectType::kNinePatch)                              \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawDRRect, Complex/BACKEND,                     \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    RRectType::kComplex)                                \
      ->RangeMultiplier(2)                                              \
      ->Range(16, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawImage
#define DRAW_IMAGE_BENCHMARKS(BACKEND, ATTRIBUTES)                      \
  BENCHMARK_CAPTURE(BM_DrawImage, Texture/BACKEND,                      \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlImageSampling::kNearestNeighbor, false)           \
      ->RangeMultiplier(2)                                              \
      ->Range(128, 512)                                                 \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawImage, Upload/BACKEND,                       \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlImageSampling::kNearestNeighbor, true)            \
      ->RangeMultiplier(2)                                              \
      ->Range(128, 512)                                                 \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawImageRect
#define DRAW_IMAGE_RECT_BENCHMARKS(BACKEND, ATTRIBUTES)                 \
  BENCHMARK_CAPTURE(                                                    \
      BM_DrawImageRect, Texture/Strict/BACKEND,                         \
      BackendType::k##BACKEND##Backend,                                 \
      ATTRIBUTES,                                                       \
      DlImageSampling::kNearestNeighbor,                                \
      DlSrcRectConstraint::kStrict, false)                              \
      ->RangeMultiplier(2)                                              \
      ->Range(32, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(                                                    \
      BM_DrawImageRect, Texture/Fast/BACKEND,                           \
      BackendType::k##BACKEND##Backend,                                 \
      ATTRIBUTES,                                                       \
      DlImageSampling::kNearestNeighbor,                                \
      DlSrcRectConstraint::kFast, false)                                \
      ->RangeMultiplier(2)                                              \
      ->Range(32, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(                                                    \
      BM_DrawImageRect, Upload/Strict/BACKEND,                          \
      BackendType::k##BACKEND##Backend,                                 \
      ATTRIBUTES,                                                       \
      DlImageSampling::kNearestNeighbor,                                \
      DlSrcRectConstraint::kStrict, true)                               \
      ->RangeMultiplier(2)                                              \
      ->Range(32, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(                                                    \
      BM_DrawImageRect, Upload/Fast/BACKEND,                            \
      BackendType::k##BACKEND##Backend,                                 \
      ATTRIBUTES,                                                       \
      DlImageSampling::kNearestNeighbor,                                \
      DlSrcRectConstraint::kFast, true)                                 \
      ->RangeMultiplier(2)                                              \
      ->Range(32, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawImageNine
#define DRAW_IMAGE_NINE_BENCHMARKS(BACKEND, ATTRIBUTES)                 \
  BENCHMARK_CAPTURE(BM_DrawImageNine, Texture/Nearest/BACKEND,          \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlFilterMode::kNearest, false)                      \
      ->RangeMultiplier(2)                                              \
      ->Range(32, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawImageNine, Upload/Nearest/BACKEND,           \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlFilterMode::kNearest, true)                       \
      ->RangeMultiplier(2)                                              \
      ->Range(32, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawImageNine, Texture/Linear/BACKEND,           \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlFilterMode::kLinear, false)                       \
      ->RangeMultiplier(2)                                              \
      ->Range(32, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawImageNine, Upload/Linear/BACKEND,            \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    DlFilterMode::kLinear, true)                        \
      ->RangeMultiplier(2)                                              \
      ->Range(32, 256)                                                  \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// DrawTextBlob
#define DRAW_TEXT_BLOB_BENCHMARKS(BACKEND, ATTRIBUTES)                  \
  BENCHMARK_CAPTURE(BM_DrawTextBlob, BACKEND,                           \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES)                                         \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 256)                                                   \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond)                                   \
      ->Complexity();

// DrawShadow
#define DRAW_SHADOW_BENCHMARKS(BACKEND, ATTRIBUTES)                     \
  BENCHMARK_CAPTURE(BM_DrawShadow, Lines/Transparent/BACKEND,           \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    true,                                               \
                    PathVerb::kLine)                                    \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 32)                                                    \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawShadow, Quads/Transparent/BACKEND,           \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    true,                                               \
                    PathVerb::kQuad)                                    \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 32)                                                    \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawShadow, Conics/Transparent/BACKEND,          \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    true,                                               \
                    PathVerb::kConic)                                   \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 32)                                                    \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawShadow, Cubics/Transparent/BACKEND,          \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    true,                                               \
                    PathVerb::kCubic)                                   \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 32)                                                    \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawShadow, Lines/Opaque/BACKEND,                \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    false,                                              \
                    PathVerb::kLine)                                    \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 32)                                                    \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawShadow, Quads/Opaque/BACKEND,                \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    false,                                              \
                    PathVerb::kQuad)                                    \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 32)                                                    \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawShadow, Conics/Opaque/BACKEND,               \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    false,                                              \
                    PathVerb::kConic)                                   \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 32)                                                    \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_DrawShadow, Cubics/Opaque/BACKEND,               \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    false,                                              \
                    PathVerb::kCubic)                                   \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 32)                                                    \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// SaveLayer
#define SAVE_LAYER_BENCHMARKS(BACKEND, ATTRIBUTES)                      \
  BENCHMARK_CAPTURE(BM_SaveLayer, Depth 1/BACKEND,                      \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    1)                                                  \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 128)                                                   \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);                                  \
                                                                        \
  BENCHMARK_CAPTURE(BM_SaveLayer, Depth 8/BACKEND,                      \
                    BackendType::k##BACKEND##Backend,                   \
                    ATTRIBUTES,                                         \
                    8)                                                  \
      ->RangeMultiplier(2)                                              \
      ->Range(1, 128)                                                   \
      ->UseRealTime()                                                   \
      ->Unit(benchmark::kMillisecond);

// Applies stroke style and antialiasing
#define STROKE_BENCHMARKS(BACKEND, ATTRIBUTES)                           \
  DRAW_LINE_BENCHMARKS(BACKEND, ATTRIBUTES)                              \
  DRAW_POINTS_BENCHMARKS(BACKEND, ATTRIBUTES)                            \
  DRAW_RECT_BENCHMARKS(BACKEND, ATTRIBUTES)                              \
  DRAW_OVAL_BENCHMARKS(BACKEND, ATTRIBUTES)                              \
  DRAW_CIRCLE_BENCHMARKS(BACKEND, ATTRIBUTES)                            \
  DRAW_ARC_BENCHMARKS(BACKEND, ATTRIBUTES)                               \
  DRAW_PATH_BENCHMARKS(BACKEND, ATTRIBUTES)                              \
  DRAW_RRECT_BENCHMARKS(BACKEND, ATTRIBUTES)                             \
  DRAW_DRRECT_BENCHMARKS(BACKEND, ATTRIBUTES)                            \
  DRAW_TEXT_BLOB_BENCHMARKS(BACKEND, ATTRIBUTES)

// Applies fill style and antialiasing
#define FILL_BENCHMARKS(BACKEND, ATTRIBUTES)                             \
  DRAW_RECT_BENCHMARKS(BACKEND, ATTRIBUTES)                              \
  DRAW_OVAL_BENCHMARKS(BACKEND, ATTRIBUTES)                              \
  DRAW_CIRCLE_BENCHMARKS(BACKEND, ATTRIBUTES)                            \
  DRAW_ARC_BENCHMARKS(BACKEND, ATTRIBUTES)                               \
  DRAW_PATH_BENCHMARKS(BACKEND, ATTRIBUTES)                              \
  DRAW_RRECT_BENCHMARKS(BACKEND, ATTRIBUTES)                             \
  DRAW_DRRECT_BENCHMARKS(BACKEND, ATTRIBUTES)                            \
  DRAW_TEXT_BLOB_BENCHMARKS(BACKEND, ATTRIBUTES)

// Applies antialiasing
#define ANTI_ALIASING_BENCHMARKS(BACKEND, ATTRIBUTES)                    \
  DRAW_IMAGE_BENCHMARKS(BACKEND, ATTRIBUTES)                             \
  DRAW_IMAGE_RECT_BENCHMARKS(BACKEND, ATTRIBUTES)

// Does not apply style or antialiasing
#define OTHER_BENCHMARKS(BACKEND, ATTRIBUTES)                            \
  DRAW_IMAGE_NINE_BENCHMARKS(BACKEND, ATTRIBUTES)                        \
  DRAW_VERTICES_BENCHMARKS(BACKEND, ATTRIBUTES)                          \
  DRAW_SHADOW_BENCHMARKS(BACKEND, ATTRIBUTES)                            \
  SAVE_LAYER_BENCHMARKS(BACKEND, ATTRIBUTES)

#define RUN_DISPLAYLIST_BENCHMARKS(BACKEND)                        \
  STROKE_BENCHMARKS(BACKEND, kStrokedStyle)                        \
  STROKE_BENCHMARKS(BACKEND, kStrokedStyle | kAntiAliasing)        \
  STROKE_BENCHMARKS(BACKEND, kStrokedStyle | kHairlineStroke)      \
  STROKE_BENCHMARKS(BACKEND, kStrokedStyle | kHairlineStroke |     \
                             kAntiAliasing)                        \
  FILL_BENCHMARKS(BACKEND, kFilledStyle)                           \
  FILL_BENCHMARKS(BACKEND, kFilledStyle | kAntiAliasing)           \
  ANTI_ALIASING_BENCHMARKS(BACKEND, kEmpty)                        \
  ANTI_ALIASING_BENCHMARKS(BACKEND, kAntiAliasing)                 \
  OTHER_BENCHMARKS(BACKEND, kEmpty)

// clang-format on

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_BENCHMARKING_DL_BENCHMARKS_H_

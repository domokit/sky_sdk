// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/benchmarking/benchmarking.h"

#include "flutter/display_list/geometry/dl_region.h"
#include "flutter/fml/logging.h"
#include "third_party/skia/include/core/SkRegion.h"

#include <random>

namespace {

using DlIRect = flutter::DlIRect;

template <typename RNG>
std::vector<DlIRect> GenerateRects(RNG& rng,
                                   const DlIRect& bounds,
                                   int numRects,
                                   int maxSize) {
  auto max_size_x = std::min(maxSize, bounds.GetWidth());
  auto max_size_y = std::min(maxSize, bounds.GetHeight());

  std::uniform_int_distribution pos_x(bounds.GetLeft(),
                                      bounds.GetRight() - max_size_x);
  std::uniform_int_distribution pos_y(bounds.GetTop(),
                                      bounds.GetBottom() - max_size_y);
  std::uniform_int_distribution size_x(1, max_size_x);
  std::uniform_int_distribution size_y(1, max_size_y);

  std::vector<DlIRect> rects;
  for (int i = 0; i < numRects; ++i) {
    DlIRect rect =
        DlIRect::MakeXYWH(pos_x(rng), pos_y(rng), size_x(rng), size_y(rng));
    rects.push_back(rect);
  }
  return rects;
}

template <typename RNG>
DlIRect RandomSubRect(RNG& rng, const DlIRect& rect, double size_factor) {
  FML_DCHECK(size_factor <= 1);

  int32_t width = rect.GetWidth() * size_factor;
  int32_t height = rect.GetHeight() * size_factor;

  std::uniform_int_distribution pos_x(0, rect.GetWidth() - width);
  std::uniform_int_distribution pos_y(0, rect.GetHeight() - height);

  return DlIRect::MakeXYWH(rect.GetLeft() + pos_x(rng),
                           rect.GetTop() + pos_y(rng),  //
                           width, height);
}

class SkRegionAdapter {
 public:
  explicit SkRegionAdapter(const std::vector<DlIRect>& rects) {
    region_.setRects(flutter::ToSkIRects(rects.data()), rects.size());
  }

  DlIRect getBounds() { return flutter::ToDlIRect(region_.getBounds()); }

  static SkRegionAdapter unionRegions(const SkRegionAdapter& a1,
                                      const SkRegionAdapter& a2) {
    SkRegionAdapter result(a1);
    result.region_.op(a2.region_, SkRegion::kUnion_Op);
    return result;
  }

  static SkRegionAdapter intersectRegions(const SkRegionAdapter& a1,
                                          const SkRegionAdapter& a2) {
    SkRegionAdapter result(a1);
    result.region_.op(a2.region_, SkRegion::kIntersect_Op);
    return result;
  }

  bool intersects(const SkRegionAdapter& region) {
    return region_.intersects(region.region_);
  }

  bool intersects(const DlIRect& rect) {
    return region_.intersects(flutter::ToSkIRect(rect));
  }

  std::vector<DlIRect> getRects() {
    std::vector<DlIRect> rects;
    SkRegion::Iterator it(region_);
    while (!it.done()) {
      rects.push_back(flutter::ToDlIRect(it.rect()));
      it.next();
    }
    return rects;
  }

 private:
  SkRegion region_;
};

class DlRegionAdapter {
 public:
  explicit DlRegionAdapter(const std::vector<DlIRect>& rects)
      : region_(rects) {}

  static DlRegionAdapter unionRegions(const DlRegionAdapter& a1,
                                      const DlRegionAdapter& a2) {
    return DlRegionAdapter(
        flutter::DlRegion::MakeUnion(a1.region_, a2.region_));
  }

  static DlRegionAdapter intersectRegions(const DlRegionAdapter& a1,
                                          const DlRegionAdapter& a2) {
    return DlRegionAdapter(
        flutter::DlRegion::MakeIntersection(a1.region_, a2.region_));
  }

  DlIRect getBounds() { return region_.bounds(); }

  bool intersects(const DlRegionAdapter& region) {
    return region_.intersects(region.region_);
  }

  bool intersects(const DlIRect& rect) { return region_.intersects(rect); }

  std::vector<DlIRect> getRects() { return region_.getRects(false); }

 private:
  explicit DlRegionAdapter(flutter::DlRegion&& region)
      : region_(std::move(region)) {}

  flutter::DlRegion region_;
};

template <typename Region>
void RunFromRectsBenchmark(benchmark::State& state, int maxSize) {
  std::random_device d;
  std::seed_seq seed{2, 1, 3};
  std::mt19937 rng(seed);

  std::uniform_int_distribution pos(0, 4000);
  std::uniform_int_distribution size(1, maxSize);

  std::vector<DlIRect> rects;
  for (int i = 0; i < 2000; ++i) {
    DlIRect rect = DlIRect::MakeXYWH(pos(rng), pos(rng), size(rng), size(rng));
    rects.push_back(rect);
  }

  while (state.KeepRunning()) {
    Region region(rects);
  }
}

template <typename Region>
void RunGetRectsBenchmark(benchmark::State& state, int maxSize) {
  std::random_device d;
  std::seed_seq seed{2, 1, 3};
  std::mt19937 rng(seed);

  std::uniform_int_distribution pos(0, 4000);
  std::uniform_int_distribution size(1, maxSize);

  std::vector<DlIRect> rects;
  for (int i = 0; i < 2000; ++i) {
    DlIRect rect = DlIRect::MakeXYWH(pos(rng), pos(rng), size(rng), size(rng));
    rects.push_back(rect);
  }

  Region region(rects);

  while (state.KeepRunning()) {
    auto vec2 = region.getRects();
  }
}

enum RegionOp { kUnion, kIntersection };

template <typename Region>
void RunRegionOpBenchmark(benchmark::State& state,
                          RegionOp op,
                          bool withSingleRect,
                          int maxSize,
                          double sizeFactor) {
  std::random_device d;
  std::seed_seq seed{2, 1, 3};
  std::mt19937 rng(seed);

  DlIRect bounds1 = DlIRect::MakeWH(4000, 4000);
  DlIRect bounds2 = RandomSubRect(rng, bounds1, sizeFactor);

  auto rects = GenerateRects(rng, bounds1, 500, maxSize);
  Region region1(rects);

  rects = GenerateRects(rng, bounds2, withSingleRect ? 1 : 500 * sizeFactor,
                        maxSize);
  Region region2(rects);

  switch (op) {
    case kUnion:
      while (state.KeepRunning()) {
        Region::unionRegions(region1, region2);
      }
      break;
    case kIntersection:
      while (state.KeepRunning()) {
        Region::intersectRegions(region1, region2);
      }
      break;
  }
}

template <typename Region>
void RunIntersectsRegionBenchmark(benchmark::State& state,
                                  int maxSize,
                                  double sizeFactor) {
  std::random_device d;
  std::seed_seq seed{2, 1, 3};
  std::mt19937 rng(seed);

  DlIRect bounds1 = DlIRect::MakeWH(4000, 4000);
  DlIRect bounds2 = RandomSubRect(rng, bounds1, sizeFactor);

  auto rects = GenerateRects(rng, bounds1, 500, maxSize);
  Region region1(rects);

  rects = GenerateRects(rng, bounds2, 500 * sizeFactor, maxSize);
  Region region2(rects);

  while (state.KeepRunning()) {
    region1.intersects(region2);
  }
}

template <typename Region>
void RunIntersectsSingleRectBenchmark(benchmark::State& state, int maxSize) {
  std::random_device d;
  std::seed_seq seed{2, 1, 3};
  std::mt19937 rng(seed);

  std::uniform_int_distribution pos(0, 4000);
  std::uniform_int_distribution size(1, maxSize);

  std::vector<DlIRect> rects;
  for (int i = 0; i < 500; ++i) {
    DlIRect rect = DlIRect::MakeXYWH(pos(rng), pos(rng), size(rng), size(rng));
    rects.push_back(rect);
  }
  Region region1(rects);

  rects.clear();
  for (int i = 0; i < 100; ++i) {
    DlIRect rect = DlIRect::MakeXYWH(pos(rng), pos(rng), size(rng), size(rng));
    rects.push_back(rect);
  }

  while (state.KeepRunning()) {
    for (auto& rect : rects) {
      region1.intersects(rect);
    }
  }
}

}  // namespace

namespace flutter {

static void BM_DlRegion_FromRects(benchmark::State& state, int maxSize) {
  RunFromRectsBenchmark<DlRegionAdapter>(state, maxSize);
}

static void BM_SkRegion_FromRects(benchmark::State& state, int maxSize) {
  RunFromRectsBenchmark<SkRegionAdapter>(state, maxSize);
}

static void BM_DlRegion_GetRects(benchmark::State& state, int maxSize) {
  RunGetRectsBenchmark<DlRegionAdapter>(state, maxSize);
}

static void BM_SkRegion_GetRects(benchmark::State& state, int maxSize) {
  RunGetRectsBenchmark<SkRegionAdapter>(state, maxSize);
}

static void BM_DlRegion_Operation(benchmark::State& state,
                                  RegionOp op,
                                  bool withSingleRect,
                                  int maxSize,
                                  double sizeFactor) {
  RunRegionOpBenchmark<DlRegionAdapter>(state, op, withSingleRect, maxSize,
                                        sizeFactor);
}

static void BM_SkRegion_Operation(benchmark::State& state,
                                  RegionOp op,
                                  bool withSingleRect,
                                  int maxSize,
                                  double sizeFactor) {
  RunRegionOpBenchmark<SkRegionAdapter>(state, op, withSingleRect, maxSize,
                                        sizeFactor);
}

static void BM_DlRegion_IntersectsRegion(benchmark::State& state,
                                         int maxSize,
                                         double sizeFactor) {
  RunIntersectsRegionBenchmark<DlRegionAdapter>(state, maxSize, sizeFactor);
}

static void BM_SkRegion_IntersectsRegion(benchmark::State& state,
                                         int maxSize,
                                         double sizeFactor) {
  RunIntersectsRegionBenchmark<SkRegionAdapter>(state, maxSize, sizeFactor);
}

static void BM_DlRegion_IntersectsSingleRect(benchmark::State& state,
                                             int maxSize) {
  RunIntersectsSingleRectBenchmark<DlRegionAdapter>(state, maxSize);
}

static void BM_SkRegion_IntersectsSingleRect(benchmark::State& state,
                                             int maxSize) {
  RunIntersectsSingleRectBenchmark<SkRegionAdapter>(state, maxSize);
}

const double kSizeFactorSmall = 0.3;

BENCHMARK_CAPTURE(BM_DlRegion_IntersectsSingleRect, Tiny, 30)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsSingleRect, Tiny, 30)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_DlRegion_IntersectsSingleRect, Small, 100)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsSingleRect, Small, 100)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_DlRegion_IntersectsSingleRect, Medium, 400)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsSingleRect, Medium, 400)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_DlRegion_IntersectsSingleRect, Large, 1500)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsSingleRect, Large, 1500)
    ->Unit(benchmark::kNanosecond);

BENCHMARK_CAPTURE(BM_DlRegion_IntersectsRegion, Tiny, 30, 1.0)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsRegion, Tiny, 30, 1.0)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_DlRegion_IntersectsRegion, Small, 100, 1.0)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsRegion, Small, 100, 1.0)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_DlRegion_IntersectsRegion, Medium, 400, 1.0)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsRegion, Medium, 400, 1.0)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_DlRegion_IntersectsRegion, Large, 1500, 1.0)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsRegion, Large, 1500, 1.0)
    ->Unit(benchmark::kNanosecond);

BENCHMARK_CAPTURE(BM_DlRegion_IntersectsRegion,
                  TinyAsymmetric,
                  30,
                  kSizeFactorSmall)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsRegion,
                  TinyAsymmetric,
                  30,
                  kSizeFactorSmall)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_DlRegion_IntersectsRegion,
                  SmallAsymmetric,
                  100,
                  kSizeFactorSmall)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsRegion,
                  SmallAsymmetric,
                  100,
                  kSizeFactorSmall)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_DlRegion_IntersectsRegion,
                  MediumAsymmetric,
                  400,
                  kSizeFactorSmall)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsRegion,
                  MediumAsymmetric,
                  400,
                  kSizeFactorSmall)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_DlRegion_IntersectsRegion,
                  LargeAsymmetric,
                  1500,
                  kSizeFactorSmall)
    ->Unit(benchmark::kNanosecond);
BENCHMARK_CAPTURE(BM_SkRegion_IntersectsRegion,
                  LargeAsymmetric,
                  1500,
                  kSizeFactorSmall)
    ->Unit(benchmark::kNanosecond);

BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Union_Tiny,
                  RegionOp::kUnion,
                  false,
                  30,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Union_Tiny,
                  RegionOp::kUnion,
                  false,
                  30,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Union_Small,
                  RegionOp::kUnion,
                  false,
                  100,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Union_Small,
                  RegionOp::kUnion,
                  false,
                  100,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Union_Medium,
                  RegionOp::kUnion,
                  false,
                  400,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Union_Medium,
                  RegionOp::kUnion,
                  false,
                  400,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Union_Large,
                  RegionOp::kUnion,
                  false,
                  1500,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Union_Large,
                  RegionOp::kUnion,
                  false,
                  1500,
                  1.0)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Union_TinyAsymmetric,
                  RegionOp::kUnion,
                  false,
                  30,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Union_TinyAsymmetric,
                  RegionOp::kUnion,
                  false,
                  30,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Union_SmallAsymmetric,
                  RegionOp::kUnion,
                  false,
                  100,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Union_SmallAsymmetric,
                  RegionOp::kUnion,
                  false,
                  100,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Union_MediumAsymmetric,
                  RegionOp::kUnion,
                  false,
                  400,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Union_MediumAsymmetric,
                  RegionOp::kUnion,
                  false,
                  400,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Union_LargeAsymmetric,
                  RegionOp::kUnion,
                  false,
                  1500,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Union_LargeAsymmetric,
                  RegionOp::kUnion,
                  false,
                  1500,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_Tiny,
                  RegionOp::kIntersection,
                  false,
                  30,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_Tiny,
                  RegionOp::kIntersection,
                  false,
                  30,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_Small,
                  RegionOp::kIntersection,
                  false,
                  100,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_Small,
                  RegionOp::kIntersection,
                  false,
                  100,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_Medium,
                  RegionOp::kIntersection,
                  false,
                  400,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_Medium,
                  RegionOp::kIntersection,
                  false,
                  400,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_Large,
                  RegionOp::kIntersection,
                  false,
                  1500,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_Large,
                  RegionOp::kIntersection,
                  false,
                  1500,
                  1.0)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_TinyAsymmetric,
                  RegionOp::kIntersection,
                  false,
                  30,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_TinyAsymmetric,
                  RegionOp::kIntersection,
                  false,
                  30,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_SmallAsymmetric,
                  RegionOp::kIntersection,
                  false,
                  100,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_SmallAsymmetric,
                  RegionOp::kIntersection,
                  false,
                  100,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_MediumAsymmetric,
                  RegionOp::kIntersection,
                  false,
                  400,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_MediumAsymmetric,
                  RegionOp::kIntersection,
                  false,
                  400,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_LargeAsymmetric,
                  RegionOp::kIntersection,
                  false,
                  1500,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_LargeAsymmetric,
                  RegionOp::kIntersection,
                  false,
                  1500,
                  kSizeFactorSmall)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_SingleRect_Tiny,
                  RegionOp::kIntersection,
                  true,
                  30,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_SingleRect_Tiny,
                  RegionOp::kIntersection,
                  true,
                  30,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_SingleRect_Small,
                  RegionOp::kIntersection,
                  true,
                  100,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_SingleRect_Small,
                  RegionOp::kIntersection,
                  true,
                  100,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_SingleRect_Medium,
                  RegionOp::kIntersection,
                  true,
                  400,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_SingleRect_Medium,
                  RegionOp::kIntersection,
                  true,
                  400,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_Operation,
                  Intersection_SingleRect_Large,
                  RegionOp::kIntersection,
                  true,
                  1500,
                  1.0)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_Operation,
                  Intersection_SingleRect_Large,
                  RegionOp::kIntersection,
                  true,
                  1500,
                  1.0)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DlRegion_FromRects, Tiny, 30)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_FromRects, Tiny, 30)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_FromRects, Small, 100)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_FromRects, Small, 100)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_FromRects, Medium, 400)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_FromRects, Medium, 400)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_FromRects, Large, 1500)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_FromRects, Large, 1500)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DlRegion_GetRects, Tiny, 30)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_GetRects, Tiny, 30)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_GetRects, Small, 100)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_GetRects, Small, 100)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_GetRects, Medium, 400)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_GetRects, Medium, 400)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DlRegion_GetRects, Large, 1500)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_SkRegion_GetRects, Large, 1500)
    ->Unit(benchmark::kMicrosecond);

}  // namespace flutter
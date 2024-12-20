// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <functional>

#include "flutter/benchmarking/benchmarking.h"

#include "flutter/impeller/geometry/matrix.h"
#include "flutter/impeller/geometry/rect.h"
#include "third_party/skia/include/core/SkM44.h"
#include "third_party/skia/include/core/SkMatrix.h"
#include "third_party/skia/include/core/SkPoint3.h"

namespace flutter {

namespace {

static constexpr float kPiOver4 = impeller::kPiOver4;

static constexpr float kSimplePerspective[16] = {
    // clang-format off
    2.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 2.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 3.0f
    // clang-format on
};

// The following matrices slowly slide a perspective-skewed transform of
// the rect past the near clipping plane. Keeping the Y perspective factor
// at -0.006 allows the X perspective factor to slowly move the transformed
// rectangle into the clipping pane over the range of [-0.01, to -0.025].
static constexpr float kClipOneCornerPerspective[16] = {
    // clang-format off
    2.0f, 0.0f, 0.0f, -0.01f,
    0.0f, 2.0f, 0.0f, -0.006f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 3.0f
    // clang-format on
};

static constexpr float kClipTwoCornersPerspective[16] = {
    // clang-format off
    2.0f, 0.0f, 0.0f, -.015f,
    0.0f, 2.0f, 0.0f, -.006f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 3.0f
    // clang-format on
};

static constexpr float kClipThreeCornersPerspective[16] = {
    // clang-format off
    2.0f, 0.0f, 0.0f, -.02f,
    0.0f, 2.0f, 0.0f, -.006f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 3.0f
    // clang-format on
};

static constexpr float kClipFourCornersPerspective[16] = {
    // clang-format off
    2.0f, 0.0f, 0.0f, -.025f,
    0.0f, 2.0f, 0.0f, -.006f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 3.0f
    // clang-format on
};

enum class AdapterType {
  kSkMatrix,
  kSkM44,
  kImpellerMatrix,
};

union TestPoint {
  TestPoint() {}

  SkPoint sk_point;
  impeller::Point impeller_point;
};

union TestRect {
  TestRect() {}

  SkRect sk_rect;
  impeller::Rect impeller_rect;
};

union TestTransform {
  TestTransform() {}

  SkMatrix sk_matrix;
  SkM44 sk_m44;
  impeller::Matrix impeller_matrix;
};

// We use a virtual adapter class rather than templating the BM_* methods
// to prevent the compiler from optimizing the benchmark bodies into a
// null set of instructions because the calculation can be proven to have
// no side effects and the result is never used.
class TransformAdapter {
 public:
  TransformAdapter() = default;
  virtual ~TransformAdapter() = default;

  // Two methods to test the overhead of just calling a virtual method on
  // the adapter (should be the same for all inheriting subclasses) and
  // for a method that does a conversion to and from the TestRect object
  // (which should be the same as the method call overhead since it does
  // no work).
  virtual void DoNothing(TestTransform& ignored) const = 0;

  virtual void InitRectLTRB(TestRect& rect,
                            float left,
                            float top,
                            float right,
                            float bottom) const = 0;
  virtual void InitPoint(TestPoint& point, float x, float y) const = 0;

  // The actual methods that do work and are the meat of the benchmarks.
  virtual void InitTransformIdentity(TestTransform& result) const = 0;
  virtual void InitTransformColMatrix(TestTransform& result,
                                      const float m[16]) const = 0;

  virtual void Translate(TestTransform& result, float tx, float ty) const = 0;
  virtual void Scale(TestTransform& result, float sx, float sy) const = 0;
  virtual void RotateRadians(TestTransform& result, float radians) const = 0;

  virtual void Concat(const TestTransform& a,
                      const TestTransform& b,
                      TestTransform& result) const = 0;

  virtual void TransformPoint(const TestTransform& transform,
                              const TestPoint& in,
                              TestPoint& out) const = 0;
  virtual void TransformPoints(const TestTransform& transform,
                               const TestPoint in[],
                               TestPoint out[],
                               int n) const = 0;
  virtual void TransformRectFast(const TestTransform& transform,
                                 const TestRect& in,
                                 TestRect& out) const = 0;
  virtual void TransformAndClipRect(const TestTransform& transform,
                                    const TestRect& in,
                                    TestRect& out) const = 0;
  virtual int CountClippedCorners(const TestTransform& transform,
                                  const TestRect& rect) const = 0;
  virtual void InvertUnchecked(const TestTransform& transform,
                               TestTransform& result) const = 0;
  virtual bool InvertAndCheck(const TestTransform& transform,
                              TestTransform& result) const = 0;
};

class SkiaAdapterBase : public TransformAdapter {
 public:
  // DoNothing methods used to measure overhead for various operations
  void DoNothing(TestTransform& ignored) const override {}

  void InitPoint(TestPoint& point, float x, float y) const override {
    point.sk_point.set(x, y);
  }

  void InitRectLTRB(TestRect& rect,
                    float left,
                    float top,
                    float right,
                    float bottom) const override {
    rect.sk_rect.setLTRB(left, top, right, bottom);
  }
};

class SkMatrixAdapter : public SkiaAdapterBase {
 public:
  SkMatrixAdapter() = default;
  ~SkMatrixAdapter() = default;

  void InitTransformIdentity(TestTransform& result) const override {
    result.sk_matrix.setIdentity();
  }

  virtual void InitTransformColMatrix(TestTransform& result,
                                      const float m[16]) const override {
    result.sk_matrix = SkM44::ColMajor(m).asM33();
  }

  void Translate(TestTransform& result, float tx, float ty) const override {
    result.sk_matrix.preTranslate(tx, ty);
  }

  void Scale(TestTransform& result, float sx, float sy) const override {
    result.sk_matrix.preScale(sx, sy);
  }

  void RotateRadians(TestTransform& result, float radians) const override {
    result.sk_matrix.preRotate(SkRadiansToDegrees(radians));
  }

  void Concat(const TestTransform& a,
              const TestTransform& b,
              TestTransform& result) const override {
    result.sk_matrix = SkMatrix::Concat(a.sk_matrix, b.sk_matrix);
  }

  void TransformPoint(const TestTransform& transform,
                      const TestPoint& in,
                      TestPoint& out) const override {
    out.sk_point = transform.sk_matrix.mapPoint(in.sk_point);
  }

  void TransformPoints(const TestTransform& transform,
                       const TestPoint in[],
                       TestPoint out[],
                       int n) const override {
    static_assert(sizeof(TestPoint) == sizeof(SkPoint));
    transform.sk_matrix.mapPoints(reinterpret_cast<SkPoint*>(out),
                                  reinterpret_cast<const SkPoint*>(in), n);
  }

  void TransformRectFast(const TestTransform& transform,
                         const TestRect& in,
                         TestRect& out) const override {
    out.sk_rect =
        transform.sk_matrix.mapRect(in.sk_rect, SkApplyPerspectiveClip::kNo);
  }

  void TransformAndClipRect(const TestTransform& transform,
                            const TestRect& in,
                            TestRect& out) const override {
    out.sk_rect =
        transform.sk_matrix.mapRect(in.sk_rect, SkApplyPerspectiveClip::kYes);
  }

  int CountClippedCorners(const TestTransform& transform,
                          const TestRect& rect) const {
    SkPoint3 homogenous[4];
    SkPoint corners[4];
    rect.sk_rect.toQuad(corners);
    transform.sk_matrix.mapHomogeneousPoints(homogenous, corners, 4);
    int count = 0;
    for (SkPoint3 hpt : homogenous) {
      if (hpt.fZ <= 0) {
        count++;
      }
    }
    return count;
  }

  void InvertUnchecked(const TestTransform& transform,
                       TestTransform& result) const override {
    [[maybe_unused]]
    bool ret = transform.sk_matrix.invert(&result.sk_matrix);
  }

  bool InvertAndCheck(const TestTransform& transform,
                      TestTransform& result) const override {
    return transform.sk_matrix.invert(&result.sk_matrix);
  }
};

class SkM44Adapter : public SkiaAdapterBase {
 public:
  SkM44Adapter() = default;
  ~SkM44Adapter() = default;

  void InitTransformIdentity(TestTransform& storage) const override {
    storage.sk_m44.setIdentity();
  }

  virtual void InitTransformColMatrix(TestTransform& result,
                                      const float m[16]) const override {
    result.sk_m44 = SkM44::ColMajor(m);
  }

  void Translate(TestTransform& storage, float tx, float ty) const override {
    storage.sk_m44.preTranslate(tx, ty);
  }

  void Scale(TestTransform& storage, float sx, float sy) const override {
    storage.sk_m44.preScale(sx, sy);
  }

  void RotateRadians(TestTransform& storage, float radians) const override {
    storage.sk_m44.preConcat(SkM44::Rotate({0, 0, 1}, radians));
  }

  void Concat(const TestTransform& a,
              const TestTransform& b,
              TestTransform& result) const override {
    result.sk_m44.setConcat(a.sk_m44, b.sk_m44);
  }

  void TransformPoint(const TestTransform& transform,
                      const TestPoint& in,
                      TestPoint& out) const override {
    out.sk_point = transform.sk_m44.asM33().mapPoint(in.sk_point);
  }

  void TransformPoints(const TestTransform& transform,
                       const TestPoint in[],
                       TestPoint out[],
                       int n) const override {
    static_assert(sizeof(TestPoint) == sizeof(SkPoint));
    transform.sk_m44.asM33().mapPoints(reinterpret_cast<SkPoint*>(out),
                                       reinterpret_cast<const SkPoint*>(in), n);
  }

  void TransformRectFast(const TestTransform& transform,
                         const TestRect& in,
                         TestRect& out) const override {
    // clang-format off
    out.sk_rect = transform.sk_m44
                      .asM33()
                      .mapRect(in.sk_rect, SkApplyPerspectiveClip::kNo);
    // clang-format on
  }

  void TransformAndClipRect(const TestTransform& transform,
                            const TestRect& in,
                            TestRect& out) const override {
    // clang-format off
    out.sk_rect = transform.sk_m44
                      .asM33()
                      .mapRect(in.sk_rect, SkApplyPerspectiveClip::kYes);
    // clang-format on
  }

  int CountClippedCorners(const TestTransform& transform,
                          const TestRect& rect) const {
    SkPoint3 homogenous[4];
    SkPoint corners[4];
    rect.sk_rect.toQuad(corners);
    transform.sk_m44.asM33().mapHomogeneousPoints(homogenous, corners, 4);
    int count = 0;
    for (SkPoint3 hpt : homogenous) {
      if (hpt.fZ <= 0) {
        count++;
      }
    }
    return count;
  }

  void InvertUnchecked(const TestTransform& transform,
                       TestTransform& result) const override {
    [[maybe_unused]]
    bool ret = transform.sk_m44.invert(&result.sk_m44);
  }

  bool InvertAndCheck(const TestTransform& transform,
                      TestTransform& result) const override {
    return transform.sk_m44.invert(&result.sk_m44);
  }
};

class ImpellerMatrixAdapter : public TransformAdapter {
 public:
  ImpellerMatrixAdapter() = default;
  ~ImpellerMatrixAdapter() = default;

  void DoNothing(TestTransform& ignored) const override {}

  void InitPoint(TestPoint& point, float x, float y) const override {
    point.impeller_point = impeller::Point(x, y);
  }

  void InitRectLTRB(TestRect& rect,
                    float left,
                    float top,
                    float right,
                    float bottom) const override {
    rect.impeller_rect = impeller::Rect::MakeLTRB(left, top, right, bottom);
  }

  void InitTransformIdentity(TestTransform& storage) const override {
    storage.impeller_matrix = impeller::Matrix();
  }

  virtual void InitTransformColMatrix(TestTransform& result,
                                      const float m[16]) const override {
    // clang-format off
    result.impeller_matrix = impeller::Matrix::MakeColumn(
        m[ 0], m[ 1], m[ 2], m[ 3],
        m[ 4], m[ 5], m[ 6], m[ 7],
        m[ 8], m[ 9], m[10], m[11],
        m[12], m[13], m[14], m[15]
    );
    // clang-format on
  }

  void Translate(TestTransform& storage, float tx, float ty) const override {
    storage.impeller_matrix = storage.impeller_matrix.Translate({tx, ty});
  }

  void Scale(TestTransform& storage, float sx, float sy) const override {
    storage.impeller_matrix = storage.impeller_matrix.Scale({sx, sy, 1.0f});
  }

  void RotateRadians(TestTransform& storage, float radians) const override {
    storage.impeller_matrix =
        storage.impeller_matrix *
        impeller::Matrix::MakeRotationZ(impeller::Radians(radians));
  }

  void Concat(const TestTransform& a,
              const TestTransform& b,
              TestTransform& result) const override {
    result.impeller_matrix = a.impeller_matrix * b.impeller_matrix;
  }

  void TransformPoint(const TestTransform& transform,
                      const TestPoint& in,
                      TestPoint& out) const override {
    out.impeller_point = transform.impeller_matrix * in.impeller_point;
  }

  void TransformPoints(const TestTransform& transform,
                       const TestPoint in[],
                       TestPoint out[],
                       int n) const override {
    for (int i = 0; i < n; i++) {
      out[i].impeller_point = transform.impeller_matrix * in[i].impeller_point;
    }
  }

  void TransformRectFast(const TestTransform& transform,
                         const TestRect& in,
                         TestRect& out) const override {
    out.impeller_rect =
        in.impeller_rect.TransformBounds(transform.impeller_matrix);
  }

  void TransformAndClipRect(const TestTransform& transform,
                            const TestRect& in,
                            TestRect& out) const override {
    out.impeller_rect =
        in.impeller_rect.TransformAndClipBounds(transform.impeller_matrix);
  }

  int CountClippedCorners(const TestTransform& transform,
                          const TestRect& rect) const {
    auto corners = rect.impeller_rect.GetPoints();
    int count = 0;
    for (auto pt : corners) {
      auto hpt = transform.impeller_matrix.TransformHomogenous(pt);
      if (hpt.z <= 0) {
        count++;
      }
    }
    return count;
  }

  void InvertUnchecked(const TestTransform& transform,
                       TestTransform& result) const override {
    result.impeller_matrix = transform.impeller_matrix.Invert();
  }

  bool InvertAndCheck(const TestTransform& transform,
                      TestTransform& result) const override {
    result.impeller_matrix = transform.impeller_matrix.Invert();
    return transform.impeller_matrix.IsInvertible();
  }
};

// The rect argument is passed in by the TransformRect benchmarks so that
// the setup function can make sure that it sets up a transform that
// clips the desired number of corners during the bounds transform.
using SetupFunction =
    std::function<void(TransformAdapter*, TestTransform&, const TestRect*)>;

static void SetupIdentity(const TransformAdapter* adapter,
                          TestTransform& transform,
                          const TestRect* rect) {
  adapter->InitTransformIdentity(transform);
}

static void SetupTranslate(const TransformAdapter* adapter,
                           TestTransform& transform,
                           const TestRect* rect) {
  adapter->InitTransformIdentity(transform);
  adapter->Translate(transform, 10.2, 12.3);
}

static void SetupScale(const TransformAdapter* adapter,
                       TestTransform& transform,
                       const TestRect* rect) {
  adapter->InitTransformIdentity(transform);
  adapter->Scale(transform, 2.0, 2.0);
}

static void SetupScaleTranslate(const TransformAdapter* adapter,
                                TestTransform& transform,
                                const TestRect* rect) {
  adapter->InitTransformIdentity(transform);
  adapter->Scale(transform, 2.0, 2.0);
  adapter->Translate(transform, 10.2, 12.3);
}

static void SetupRotate(const TransformAdapter* adapter,
                        TestTransform& transform,
                        const TestRect* rect) {
  adapter->InitTransformIdentity(transform);
  adapter->RotateRadians(transform, kPiOver4);
}

static void SetupPerspective(const TransformAdapter* adapter,
                             TestTransform& transform,
                             const TestRect* rect) {
  adapter->InitTransformColMatrix(transform, kSimplePerspective);
  if (rect) {
    FML_CHECK(adapter->CountClippedCorners(transform, *rect) == 0);
  }
}

static void SetupPerspectiveClipNone(const TransformAdapter* adapter,
                                     TestTransform& transform,
                                     const TestRect* rect) {
  adapter->InitTransformColMatrix(transform, kSimplePerspective);
  if (rect) {
    FML_CHECK(adapter->CountClippedCorners(transform, *rect) == 0);
  }
}

static void SetupPerspectiveClipOne(const TransformAdapter* adapter,
                                    TestTransform& transform,
                                    const TestRect* rect) {
  adapter->InitTransformColMatrix(transform, kClipOneCornerPerspective);
  if (rect) {
    FML_CHECK(adapter->CountClippedCorners(transform, *rect) == 1);
  }
}

static void SetupPerspectiveClipTwo(const TransformAdapter* adapter,
                                    TestTransform& transform,
                                    const TestRect* rect) {
  adapter->InitTransformColMatrix(transform, kClipTwoCornersPerspective);
  if (rect) {
    FML_CHECK(adapter->CountClippedCorners(transform, *rect) == 2);
  }
}

static void SetupPerspectiveClipThree(const TransformAdapter* adapter,
                                      TestTransform& transform,
                                      const TestRect* rect) {
  adapter->InitTransformColMatrix(transform, kClipThreeCornersPerspective);
  if (rect) {
    FML_CHECK(adapter->CountClippedCorners(transform, *rect) == 3);
  }
}

static void SetupPerspectiveClipFour(const TransformAdapter* adapter,
                                     TestTransform& transform,
                                     const TestRect* rect) {
  adapter->InitTransformColMatrix(transform, kClipFourCornersPerspective);
  if (rect) {
    FML_CHECK(adapter->CountClippedCorners(transform, *rect) == 4);
  }
}

// We use a function to return the appropriate adapter so that all methods
// used in benchmarking are "pure virtual" and cannot be optimized out
// due to issues such as the arguments being constexpr and the result
// simplified to a constant value.
static std::unique_ptr<TransformAdapter> GetAdapter(AdapterType type) {
  switch (type) {
    case AdapterType::kSkMatrix:
      return std::make_unique<SkMatrixAdapter>();
    case AdapterType::kSkM44:
      return std::make_unique<SkM44Adapter>();
    case AdapterType::kImpellerMatrix:
      return std::make_unique<ImpellerMatrixAdapter>();
  }
  FML_UNREACHABLE();
}

}  // namespace

static void BM_AdapterDispatchOverhead(benchmark::State& state,
                                       AdapterType type) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  while (state.KeepRunning()) {
    adapter->DoNothing(transform);
  }
}

static void BM_SetIdentity(benchmark::State& state, AdapterType type) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  while (state.KeepRunning()) {
    adapter->InitTransformIdentity(transform);
  }
}

static void BM_SetPerspective(benchmark::State& state, AdapterType type) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  while (state.KeepRunning()) {
    adapter->InitTransformColMatrix(transform, kSimplePerspective);
  }
}

static void BM_Translate(benchmark::State& state,
                         AdapterType type,
                         float tx,
                         float ty) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  adapter->InitTransformIdentity(transform);
  bool flip = true;
  while (state.KeepRunning()) {
    if (flip) {
      adapter->Translate(transform, tx, ty);
    } else {
      adapter->Translate(transform, -tx, -ty);
    }
    flip = !flip;
  }
}

static void BM_Scale(benchmark::State& state, AdapterType type, float scale) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  adapter->InitTransformIdentity(transform);
  float inv_scale = 1.0f / scale;
  bool flip = true;
  while (state.KeepRunning()) {
    if (flip) {
      adapter->Scale(transform, scale, scale);
    } else {
      adapter->Scale(transform, inv_scale, inv_scale);
    }
    flip = !flip;
  }
}

static void BM_Rotate(benchmark::State& state,
                      AdapterType type,
                      float radians) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  adapter->InitTransformIdentity(transform);
  while (state.KeepRunning()) {
    adapter->RotateRadians(transform, radians);
  }
}

static void BM_Concat(benchmark::State& state,
                      AdapterType type,
                      const SetupFunction& a_setup,
                      const SetupFunction& b_setup) {
  auto adapter = GetAdapter(type);
  TestTransform a, b, result;
  a_setup(adapter.get(), a, nullptr);
  b_setup(adapter.get(), b, nullptr);
  while (state.KeepRunning()) {
    adapter->Concat(a, b, result);
  }
}

static void BM_TransformPoint(benchmark::State& state,
                              AdapterType type,
                              const SetupFunction& setup) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  setup(adapter.get(), transform, nullptr);
  TestPoint point, result;
  adapter->InitPoint(point, 25.7, 32.4);
  while (state.KeepRunning()) {
    adapter->TransformPoint(transform, point, result);
  }
}

static void BM_TransformPoints(benchmark::State& state,
                               AdapterType type,
                               const SetupFunction& setup) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  setup(adapter.get(), transform, nullptr);
  const int Xs = 10;
  const int Ys = 10;
  const int N = Xs * Ys;
  TestPoint points[N];
  for (int i = 0; i < Xs; i++) {
    for (int j = 0; j < Ys; j++) {
      int index = i * Xs + j;
      FML_CHECK(index < N);
      adapter->InitPoint(points[index], i * 23.3 + 17, j * 32.7 + 12);
    }
  }
  TestPoint results[N];
  int64_t item_count = 0;
  while (state.KeepRunning()) {
    adapter->TransformPoints(transform, points, results, N);
    item_count += N;
  }
  state.SetItemsProcessed(item_count);
}

static void BM_TransformRectFast(benchmark::State& state,
                                 AdapterType type,
                                 const SetupFunction& setup) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  TestRect rect, result;
  adapter->InitRectLTRB(rect, 100, 100, 200, 200);
  setup(adapter.get(), transform, &rect);
  while (state.KeepRunning()) {
    adapter->TransformRectFast(transform, rect, result);
  }
}

static void BM_TransformAndClipRect(benchmark::State& state,
                                    AdapterType type,
                                    const SetupFunction& setup) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  TestRect rect, result;
  adapter->InitRectLTRB(rect, 100, 100, 200, 200);
  setup(adapter.get(), transform, &rect);
  while (state.KeepRunning()) {
    adapter->TransformAndClipRect(transform, rect, result);
  }
}

static void BM_InvertUnchecked(benchmark::State& state,
                               AdapterType type,
                               const SetupFunction& setup) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  setup(adapter.get(), transform, nullptr);
  TestTransform result;
  while (state.KeepRunning()) {
    adapter->InvertUnchecked(transform, result);
  }
}

static void BM_InvertAndCheck(benchmark::State& state,
                              AdapterType type,
                              const SetupFunction& setup) {
  auto adapter = GetAdapter(type);
  TestTransform transform;
  setup(adapter.get(), transform, nullptr);
  TestTransform result;
  while (state.KeepRunning()) {
    adapter->InvertAndCheck(transform, result);
  }
}

#define BENCHMARK_CAPTURE_TYPE(name, type) \
  BENCHMARK_CAPTURE(name, type, AdapterType::k##type)

#define BENCHMARK_CAPTURE_TYPE_ARGS(name, type, ...) \
  BENCHMARK_CAPTURE(name, type, AdapterType::k##type, __VA_ARGS__)

#define BENCHMARK_CAPTURE_ALL(name)       \
  BENCHMARK_CAPTURE_TYPE(name, SkMatrix); \
  BENCHMARK_CAPTURE_TYPE(name, SkM44);    \
  BENCHMARK_CAPTURE_TYPE(name, ImpellerMatrix)

#define BENCHMARK_CAPTURE_ALL_ARGS(name, ...)               \
  BENCHMARK_CAPTURE_TYPE_ARGS(name, SkMatrix, __VA_ARGS__); \
  BENCHMARK_CAPTURE_TYPE_ARGS(name, SkM44, __VA_ARGS__);    \
  BENCHMARK_CAPTURE_TYPE_ARGS(name, ImpellerMatrix, __VA_ARGS__)

BENCHMARK_CAPTURE_ALL(BM_AdapterDispatchOverhead);

BENCHMARK_CAPTURE_ALL(BM_SetIdentity);
BENCHMARK_CAPTURE_ALL(BM_SetPerspective);
BENCHMARK_CAPTURE_ALL_ARGS(BM_Translate, 10.0f, 15.0f);
BENCHMARK_CAPTURE_ALL_ARGS(BM_Scale, 2.0f);
BENCHMARK_CAPTURE_ALL_ARGS(BM_Rotate, kPiOver4);

// clang-format off
#define BENCHMARK_CAPTURE_TYPE_SETUP(name, type, setup) \
  BENCHMARK_CAPTURE(name, setup/type, AdapterType::k##type, Setup##setup)
// clang-format on

#define BENCHMARK_CAPTURE_ALL_SETUP(name, setup)       \
  BENCHMARK_CAPTURE_TYPE_SETUP(name, SkMatrix, setup); \
  BENCHMARK_CAPTURE_TYPE_SETUP(name, SkM44, setup);    \
  BENCHMARK_CAPTURE_TYPE_SETUP(name, ImpellerMatrix, setup)

// clang-format off
#define BENCHMARK_CAPTURE_TYPE_SETUP2(name, type, setup1, setup2)   \
  BENCHMARK_CAPTURE(name, setup1*setup2/type, AdapterType::k##type, \
                    Setup##setup1, Setup##setup2)
// clang-format on

#define BENCHMARK_CAPTURE_ALL_SETUP2(name, setup1, setup2)       \
  BENCHMARK_CAPTURE_TYPE_SETUP2(name, SkMatrix, setup1, setup2); \
  BENCHMARK_CAPTURE_TYPE_SETUP2(name, SkM44, setup1, setup2);    \
  BENCHMARK_CAPTURE_TYPE_SETUP2(name, ImpellerMatrix, setup1, setup2)

BENCHMARK_CAPTURE_ALL_SETUP2(BM_Concat, Scale, Translate);
BENCHMARK_CAPTURE_ALL_SETUP2(BM_Concat, ScaleTranslate, ScaleTranslate);
BENCHMARK_CAPTURE_ALL_SETUP2(BM_Concat, ScaleTranslate, Rotate);
BENCHMARK_CAPTURE_ALL_SETUP2(BM_Concat, ScaleTranslate, Perspective);
BENCHMARK_CAPTURE_ALL_SETUP2(BM_Concat, Perspective, ScaleTranslate);

BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertUnchecked, Identity);
BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertUnchecked, Translate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertUnchecked, Scale);
BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertUnchecked, ScaleTranslate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertUnchecked, Rotate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertUnchecked, Perspective);

BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertAndCheck, Identity);
BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertAndCheck, Translate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertAndCheck, Scale);
BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertAndCheck, ScaleTranslate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertAndCheck, Rotate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_InvertAndCheck, Perspective);

BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoint, Identity);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoint, Translate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoint, Scale);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoint, ScaleTranslate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoint, Rotate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoint, Perspective);

BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoints, Identity);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoints, Translate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoints, Scale);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoints, ScaleTranslate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoints, Rotate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformPoints, Perspective);

BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformRectFast, Identity);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformRectFast, Translate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformRectFast, Scale);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformRectFast, ScaleTranslate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformRectFast, Rotate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformRectFast, PerspectiveClipNone);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformRectFast, PerspectiveClipOne);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformRectFast, PerspectiveClipTwo);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformRectFast, PerspectiveClipThree);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformRectFast, PerspectiveClipFour);

BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformAndClipRect, Identity);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformAndClipRect, Translate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformAndClipRect, Scale);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformAndClipRect, ScaleTranslate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformAndClipRect, Rotate);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformAndClipRect, PerspectiveClipNone);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformAndClipRect, PerspectiveClipOne);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformAndClipRect, PerspectiveClipTwo);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformAndClipRect, PerspectiveClipThree);
BENCHMARK_CAPTURE_ALL_SETUP(BM_TransformAndClipRect, PerspectiveClipFour);

}  // namespace flutter

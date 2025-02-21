// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/benchmarking/benchmarking.h"
#include "flutter/display_list/testing/dl_test_snippets.h"
#include "flutter/display_list/utils/dl_receiver_utils.h"

namespace flutter {

DlOpReceiver& DisplayListBuilderBenchmarkAccessor(DisplayListBuilder& builder) {
  return builder.asReceiver();
}

namespace {

static std::vector<testing::DisplayListInvocationGroup> allRenderingOps =
    testing::CreateAllRenderingOps();

static std::vector<testing::DisplayListInvocationGroup> allOps =
    testing::CreateAllGroups();

enum class DisplayListBuilderBenchmarkType {
  kDefault,
  kBounds,
  kRtree,
  kBoundsAndRtree,
};

enum class DisplayListDispatchBenchmarkType {
  kDefaultNoRtree,
  kDefaultWithRtree,
  kCulledWithRtree,
};

static void InvokeAllRenderingOps(DisplayListBuilder& builder) {
  DlOpReceiver& receiver = DisplayListBuilderBenchmarkAccessor(builder);
  for (auto& group : allRenderingOps) {
    for (size_t i = 0; i < group.variants.size(); i++) {
      auto& invocation = group.variants[i];
      invocation.Invoke(receiver);
    }
  }
}

static void InvokeAllOps(DisplayListBuilder& builder) {
  DlOpReceiver& receiver = DisplayListBuilderBenchmarkAccessor(builder);
  for (auto& group : allOps) {
    // Save/restore around each group so that the clip and transform
    // test ops do not walk us out to infinity or prevent any future
    // rendering ops.
    receiver.save();
    for (size_t i = 0; i < group.variants.size(); i++) {
      auto& invocation = group.variants[i];
      invocation.Invoke(receiver);
    }
    receiver.restore();
  }
}

static void Complete(DisplayListBuilder& builder,
                     DisplayListBuilderBenchmarkType type) {
  auto display_list = builder.Build();
  switch (type) {
    case DisplayListBuilderBenchmarkType::kBounds:
      display_list->bounds();
      break;
    case DisplayListBuilderBenchmarkType::kRtree:
      display_list->rtree();
      break;
    case DisplayListBuilderBenchmarkType::kBoundsAndRtree:
      display_list->bounds();
      display_list->rtree();
      break;
    case DisplayListBuilderBenchmarkType::kDefault:
      break;
  }
}

bool NeedPrepareRTree(DisplayListBuilderBenchmarkType type) {
  return type == DisplayListBuilderBenchmarkType::kRtree ||
         type == DisplayListBuilderBenchmarkType::kBoundsAndRtree;
}

bool NeedPrepareRTree(DisplayListDispatchBenchmarkType type) {
  return type != DisplayListDispatchBenchmarkType::kDefaultNoRtree;
}

}  // namespace

static void BM_DisplayListBuilderDefault(benchmark::State& state,
                                         DisplayListBuilderBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  while (state.KeepRunning()) {
    DisplayListBuilder builder(prepare_rtree);
    InvokeAllRenderingOps(builder);
    Complete(builder, type);
  }
}

static void BM_DisplayListBuilderWithScaleAndTranslate(
    benchmark::State& state,
    DisplayListBuilderBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  while (state.KeepRunning()) {
    DisplayListBuilder builder(prepare_rtree);
    builder.Scale(3.5, 3.5);
    builder.Translate(10.3, 6.9);
    InvokeAllRenderingOps(builder);
    Complete(builder, type);
  }
}

static void BM_DisplayListBuilderWithPerspective(
    benchmark::State& state,
    DisplayListBuilderBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  while (state.KeepRunning()) {
    DisplayListBuilder builder(prepare_rtree);
    builder.TransformFullPerspective(0, 1, 0, 12, 1, 0, 0, 33, 3, 2, 5, 29, 0,
                                     0, 0, 12);
    InvokeAllRenderingOps(builder);
    Complete(builder, type);
  }
}

static void BM_DisplayListBuilderWithClipRect(
    benchmark::State& state,
    DisplayListBuilderBenchmarkType type) {
  DlRect clip_bounds = DlRect::MakeLTRB(6.5, 7.3, 90.2, 85.7);
  bool prepare_rtree = NeedPrepareRTree(type);
  while (state.KeepRunning()) {
    DisplayListBuilder builder(prepare_rtree);
    builder.ClipRect(clip_bounds, DlClipOp::kIntersect, true);
    InvokeAllRenderingOps(builder);
    Complete(builder, type);
  }
}

static void BM_DisplayListBuilderWithGlobalSaveLayer(
    benchmark::State& state,
    DisplayListBuilderBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  while (state.KeepRunning()) {
    DisplayListBuilder builder(prepare_rtree);
    builder.Scale(3.5, 3.5);
    builder.Translate(10.3, 6.9);
    builder.SaveLayer(std::nullopt, nullptr);
    builder.Translate(45.3, 27.9);
    DlOpReceiver& receiver = DisplayListBuilderBenchmarkAccessor(builder);
    for (auto& group : allRenderingOps) {
      for (size_t i = 0; i < group.variants.size(); i++) {
        auto& invocation = group.variants[i];
        invocation.Invoke(receiver);
      }
    }
    builder.Restore();
    Complete(builder, type);
  }
}

static void BM_DisplayListBuilderWithSaveLayer(
    benchmark::State& state,
    DisplayListBuilderBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  while (state.KeepRunning()) {
    DisplayListBuilder builder(prepare_rtree);
    DlOpReceiver& receiver = DisplayListBuilderBenchmarkAccessor(builder);
    for (auto& group : allRenderingOps) {
      for (size_t i = 0; i < group.variants.size(); i++) {
        auto& invocation = group.variants[i];
        builder.SaveLayer(std::nullopt, nullptr);
        invocation.Invoke(receiver);
        builder.Restore();
      }
    }
    Complete(builder, type);
  }
}

static void BM_DisplayListBuilderWithSaveLayerAndImageFilter(
    benchmark::State& state,
    DisplayListBuilderBenchmarkType type) {
  DlPaint layer_paint;
  layer_paint.setImageFilter(&testing::kTestBlurImageFilter1);
  DlRect layer_bounds = DlRect::MakeLTRB(6.5, 7.3, 35.2, 42.7);
  bool prepare_rtree = NeedPrepareRTree(type);
  while (state.KeepRunning()) {
    DisplayListBuilder builder(prepare_rtree);
    DlOpReceiver& receiver = DisplayListBuilderBenchmarkAccessor(builder);
    for (auto& group : allRenderingOps) {
      for (size_t i = 0; i < group.variants.size(); i++) {
        auto& invocation = group.variants[i];
        builder.SaveLayer(layer_bounds, &layer_paint);
        invocation.Invoke(receiver);
        builder.Restore();
      }
    }
    Complete(builder, type);
  }
}

class DlOpReceiverIgnore : public IgnoreAttributeDispatchHelper,
                           public IgnoreTransformDispatchHelper,
                           public IgnoreClipDispatchHelper,
                           public IgnoreDrawDispatchHelper {};

static void BM_DisplayListDispatchDefault(
    benchmark::State& state,
    DisplayListDispatchBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  DisplayListBuilder builder(prepare_rtree);
  for (int i = 0; i < 5; i++) {
    InvokeAllOps(builder);
  }
  auto display_list = builder.Build();
  DlOpReceiverIgnore receiver;
  while (state.KeepRunning()) {
    display_list->Dispatch(receiver);
  }
}

static void BM_DisplayListDispatchByIndexDefault(
    benchmark::State& state,
    DisplayListDispatchBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  DisplayListBuilder builder(prepare_rtree);
  for (int i = 0; i < 5; i++) {
    InvokeAllOps(builder);
  }
  auto display_list = builder.Build();
  DlOpReceiverIgnore receiver;
  while (state.KeepRunning()) {
    DlIndex end = display_list->GetRecordCount();
    for (DlIndex i = 0u; i < end; i++) {
      display_list->Dispatch(receiver, i);
    }
  }
}

static void BM_DisplayListDispatchByIteratorDefault(
    benchmark::State& state,
    DisplayListDispatchBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  DisplayListBuilder builder(prepare_rtree);
  for (int i = 0; i < 5; i++) {
    InvokeAllOps(builder);
  }
  auto display_list = builder.Build();
  DlOpReceiverIgnore receiver;
  while (state.KeepRunning()) {
    for (DlIndex i : *display_list) {
      display_list->Dispatch(receiver, i);
    }
  }
}

static void BM_DisplayListDispatchByVectorDefault(
    benchmark::State& state,
    DisplayListDispatchBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  DisplayListBuilder builder(prepare_rtree);
  for (int i = 0; i < 5; i++) {
    InvokeAllOps(builder);
  }
  auto display_list = builder.Build();
  DlOpReceiverIgnore receiver;
  while (state.KeepRunning()) {
    std::vector<DlIndex> indices =
        display_list->GetCulledIndices(display_list->GetBounds());
    for (DlIndex index : indices) {
      display_list->Dispatch(receiver, index);
    }
  }
}

static void BM_DisplayListDispatchCull(benchmark::State& state,
                                       DisplayListDispatchBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  DisplayListBuilder builder(prepare_rtree);
  for (int i = 0; i < 5; i++) {
    InvokeAllOps(builder);
  }
  auto display_list = builder.Build();
  DlRect rect = DlRect::MakeLTRB(0, 0, 100, 100);
  EXPECT_FALSE(rect.Contains(display_list->GetBounds()));
  DlOpReceiverIgnore receiver;
  while (state.KeepRunning()) {
    display_list->Dispatch(receiver, rect);
  }
}

static void BM_DisplayListDispatchByVectorCull(
    benchmark::State& state,
    DisplayListDispatchBenchmarkType type) {
  bool prepare_rtree = NeedPrepareRTree(type);
  DisplayListBuilder builder(prepare_rtree);
  for (int i = 0; i < 5; i++) {
    InvokeAllOps(builder);
  }
  auto display_list = builder.Build();
  DlRect rect = DlRect::MakeLTRB(0, 0, 100, 100);
  EXPECT_FALSE(rect.Contains(display_list->GetBounds()));
  DlOpReceiverIgnore receiver;
  while (state.KeepRunning()) {
    std::vector<DlIndex> indices = display_list->GetCulledIndices(rect);
    for (DlIndex index : indices) {
      display_list->Dispatch(receiver, index);
    }
  }
}

BENCHMARK_CAPTURE(BM_DisplayListBuilderDefault,
                  kDefault,
                  DisplayListBuilderBenchmarkType::kDefault)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderDefault,
                  kBounds,
                  DisplayListBuilderBenchmarkType::kBounds)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderDefault,
                  kRtree,
                  DisplayListBuilderBenchmarkType::kRtree)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderDefault,
                  kBoundsAndRtree,
                  DisplayListBuilderBenchmarkType::kBoundsAndRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListBuilderWithScaleAndTranslate,
                  kDefault,
                  DisplayListBuilderBenchmarkType::kDefault)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithScaleAndTranslate,
                  kBounds,
                  DisplayListBuilderBenchmarkType::kBounds)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithScaleAndTranslate,
                  kRtree,
                  DisplayListBuilderBenchmarkType::kRtree)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithScaleAndTranslate,
                  kBoundsAndRtree,
                  DisplayListBuilderBenchmarkType::kBoundsAndRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListBuilderWithPerspective,
                  kDefault,
                  DisplayListBuilderBenchmarkType::kDefault)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithPerspective,
                  kBounds,
                  DisplayListBuilderBenchmarkType::kBounds)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithPerspective,
                  kRtree,
                  DisplayListBuilderBenchmarkType::kRtree)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithPerspective,
                  kBoundsAndRtree,
                  DisplayListBuilderBenchmarkType::kBoundsAndRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListBuilderWithClipRect,
                  kDefault,
                  DisplayListBuilderBenchmarkType::kDefault)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithClipRect,
                  kBounds,
                  DisplayListBuilderBenchmarkType::kBounds)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithClipRect,
                  kRtree,
                  DisplayListBuilderBenchmarkType::kRtree)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithClipRect,
                  kBoundsAndRtree,
                  DisplayListBuilderBenchmarkType::kBoundsAndRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListBuilderWithGlobalSaveLayer,
                  kDefault,
                  DisplayListBuilderBenchmarkType::kDefault)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithGlobalSaveLayer,
                  kBounds,
                  DisplayListBuilderBenchmarkType::kBounds)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithGlobalSaveLayer,
                  kRtree,
                  DisplayListBuilderBenchmarkType::kRtree)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithGlobalSaveLayer,
                  kBoundsAndRtree,
                  DisplayListBuilderBenchmarkType::kBoundsAndRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListBuilderWithSaveLayer,
                  kDefault,
                  DisplayListBuilderBenchmarkType::kDefault)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithSaveLayer,
                  kBounds,
                  DisplayListBuilderBenchmarkType::kBounds)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithSaveLayer,
                  kRtree,
                  DisplayListBuilderBenchmarkType::kRtree)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithSaveLayer,
                  kBoundsAndRtree,
                  DisplayListBuilderBenchmarkType::kBoundsAndRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListBuilderWithSaveLayerAndImageFilter,
                  kDefault,
                  DisplayListBuilderBenchmarkType::kDefault)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithSaveLayerAndImageFilter,
                  kBounds,
                  DisplayListBuilderBenchmarkType::kBounds)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithSaveLayerAndImageFilter,
                  kRtree,
                  DisplayListBuilderBenchmarkType::kRtree)
    ->Unit(benchmark::kMicrosecond);
BENCHMARK_CAPTURE(BM_DisplayListBuilderWithSaveLayerAndImageFilter,
                  kBoundsAndRtree,
                  DisplayListBuilderBenchmarkType::kBoundsAndRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListDispatchDefault,
                  kDefaultNoRtree,
                  DisplayListDispatchBenchmarkType::kDefaultNoRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListDispatchDefault,
                  kDefaultWithRtree,
                  DisplayListDispatchBenchmarkType::kDefaultWithRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListDispatchCull,
                  kCulledWithRtree,
                  DisplayListDispatchBenchmarkType::kCulledWithRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListDispatchByIndexDefault,
                  kDefaultNoRtree,
                  DisplayListDispatchBenchmarkType::kDefaultNoRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListDispatchByIteratorDefault,
                  kDefaultNoRtree,
                  DisplayListDispatchBenchmarkType::kDefaultNoRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListDispatchByVectorDefault,
                  kDefaultNoRtree,
                  DisplayListDispatchBenchmarkType::kDefaultNoRtree)
    ->Unit(benchmark::kMicrosecond);

BENCHMARK_CAPTURE(BM_DisplayListDispatchByVectorCull,
                  kCulledWithRtree,
                  DisplayListDispatchBenchmarkType::kCulledWithRtree)
    ->Unit(benchmark::kMicrosecond);

}  // namespace flutter

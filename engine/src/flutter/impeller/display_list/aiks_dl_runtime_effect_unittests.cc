// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>

#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/display_list/effects/dl_runtime_effect.h"
#include "flutter/impeller/display_list/aiks_unittests.h"

#include "include/core/SkPath.h"
#include "include/core/SkRRect.h"

namespace impeller {
namespace testing {

using namespace flutter;

namespace {
std::shared_ptr<DlColorSource> MakeRuntimeEffect(
    AiksTest* test,
    std::string_view name,
    const std::shared_ptr<std::vector<uint8_t>>& uniform_data = {},
    const std::vector<std::shared_ptr<DlColorSource>>& samplers = {}) {
  auto runtime_stages = test->OpenAssetAsRuntimeStage(name.data());
  auto runtime_stage = runtime_stages[PlaygroundBackendToRuntimeStageBackend(
      test->GetBackend())];
  FML_CHECK(runtime_stage);
  FML_CHECK(runtime_stage->IsDirty());

  auto dl_runtime_effect = DlRuntimeEffect::MakeImpeller(runtime_stage);

  return DlColorSource::MakeRuntimeEffect(dl_runtime_effect, samplers,
                                          uniform_data);
}
}  // namespace

// Regression test for https://github.com/flutter/flutter/issues/126701 .
TEST_P(AiksTest, CanRenderClippedRuntimeEffects) {
  struct FragUniforms {
    Vector2 iResolution;
    Scalar iTime;
  } frag_uniforms = {.iResolution = Vector2(400, 400), .iTime = 100.0};
  auto uniform_data = std::make_shared<std::vector<uint8_t>>();
  uniform_data->resize(sizeof(FragUniforms));
  memcpy(uniform_data->data(), &frag_uniforms, sizeof(FragUniforms));

  DlPaint paint;
  paint.setColorSource(
      MakeRuntimeEffect(this, "runtime_stage_example.frag.iplr", uniform_data));

  DisplayListBuilder builder;
  builder.Save();
  builder.ClipRRect(
      SkRRect::MakeRectXY(SkRect::MakeXYWH(0, 0, 400, 400), 10.0, 10.0),
      DlCanvas::ClipOp::kIntersect);
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 400, 400), paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, DrawPaintTransformsBounds) {
  struct FragUniforms {
    Size size;
  } frag_uniforms = {.size = Size::MakeWH(400, 400)};
  auto uniform_data = std::make_shared<std::vector<uint8_t>>();
  uniform_data->resize(sizeof(FragUniforms));
  memcpy(uniform_data->data(), &frag_uniforms, sizeof(FragUniforms));

  DlPaint paint;
  paint.setColorSource(
      MakeRuntimeEffect(this, "gradient.frag.iplr", uniform_data));

  DisplayListBuilder builder;
  builder.Save();
  builder.Scale(GetContentScale().x, GetContentScale().y);
  builder.DrawPaint(paint);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderRuntimeEffectFilter) {
  auto runtime_stages =
      OpenAssetAsRuntimeStage("runtime_stage_filter_example.frag.iplr");

  std::shared_ptr<RuntimeStage> runtime_stage =
      runtime_stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];
  ASSERT_TRUE(runtime_stage);
  ASSERT_TRUE(runtime_stage->IsDirty());

  std::vector<std::shared_ptr<DlColorSource>> sampler_inputs = {
      nullptr,
  };
  auto uniform_data = std::make_shared<std::vector<uint8_t>>();
  uniform_data->resize(sizeof(Vector2));

  DlPaint paint;
  paint.setColor(DlColor::kAqua());
  paint.setImageFilter(DlImageFilter::MakeRuntimeEffect(
      DlRuntimeEffect::MakeImpeller(runtime_stage), sampler_inputs,
      uniform_data));

  DisplayListBuilder builder;
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 400, 400), paint);

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller

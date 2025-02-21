// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/benchmarking/dl_complexity.h"
#include "flutter/display_list/benchmarking/dl_complexity_gl.h"
#include "flutter/display_list/benchmarking/dl_complexity_metal.h"
#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_sampling_options.h"
#include "flutter/display_list/testing/dl_test_snippets.h"
#include "flutter/testing/testing.h"

namespace flutter {
namespace testing {

namespace {

std::vector<DisplayListComplexityCalculator*> Calculators() {
  return {DisplayListMetalComplexityCalculator::GetInstance(),
          DisplayListGLComplexityCalculator::GetInstance(),
          DisplayListNaiveComplexityCalculator::GetInstance()};
}

std::vector<DisplayListComplexityCalculator*> AccumulatorCalculators() {
  return {DisplayListMetalComplexityCalculator::GetInstance(),
          DisplayListGLComplexityCalculator::GetInstance()};
}

std::vector<DlPoint> GetTestPoints() {
  std::vector<DlPoint> points;
  points.emplace_back(0, 0);
  points.emplace_back(10, 0);
  points.emplace_back(10, 10);
  points.emplace_back(20, 10);
  points.emplace_back(20, 20);

  return points;
}

}  // namespace

TEST(DisplayListComplexity, EmptyDisplayList) {
  auto display_list = GetSampleDisplayList(0);

  auto calculators = Calculators();
  for (auto calculator : calculators) {
    ASSERT_EQ(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DisplayListCeiling) {
  auto display_list = GetSampleDisplayList();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    calculator->SetComplexityCeiling(10u);
    ASSERT_EQ(calculator->Compute(display_list.get()), 10u);
    calculator->SetComplexityCeiling(std::numeric_limits<unsigned int>::max());
  }
}

TEST(DisplayListComplexity, NestedDisplayList) {
  auto display_list = GetSampleNestedDisplayList();

  auto calculators = Calculators();
  for (auto calculator : calculators) {
    // There's only one draw call in the "outer" DisplayList, which calls
    // drawDisplayList with the "inner" DisplayList. To ensure we are
    // recursing correctly into the inner DisplayList, check that we aren't
    // returning 0 (if the function is a no-op) or 1 (as the op_count is 1)
    ASSERT_GT(calculator->Compute(display_list.get()), 1u);
  }
}

TEST(DisplayListComplexity, AntiAliasing) {
  DisplayListBuilder builder_no_aa;
  builder_no_aa.DrawLine(DlPoint(0, 0), DlPoint(100, 100), DlPaint());
  auto display_list_no_aa = builder_no_aa.Build();

  DisplayListBuilder builder_aa;
  builder_aa.DrawLine(DlPoint(0, 0), DlPoint(100, 100),
                      DlPaint().setAntiAlias(true));
  auto display_list_aa = builder_aa.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list_no_aa.get()),
              calculator->Compute(display_list_aa.get()));
  }
}

TEST(DisplayListComplexity, StrokeWidth) {
  DisplayListBuilder builder_stroke_0;
  builder_stroke_0.DrawLine(DlPoint(0, 0), DlPoint(100, 100),
                            DlPaint().setStrokeWidth(0.0f));
  auto display_list_stroke_0 = builder_stroke_0.Build();

  DisplayListBuilder builder_stroke_1;
  builder_stroke_1.DrawLine(DlPoint(0, 0), DlPoint(100, 100),
                            DlPaint().setStrokeWidth(1.0f));
  auto display_list_stroke_1 = builder_stroke_1.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list_stroke_0.get()),
              calculator->Compute(display_list_stroke_1.get()));
  }
}

TEST(DisplayListComplexity, Style) {
  DisplayListBuilder builder_filled;
  builder_filled.DrawRect(DlRect::MakeXYWH(10, 10, 80, 80),
                          DlPaint().setDrawStyle(DlDrawStyle::kFill));
  auto display_list_filled = builder_filled.Build();

  DisplayListBuilder builder_stroked;
  builder_stroked.DrawRect(DlRect::MakeXYWH(10, 10, 80, 80),
                           DlPaint().setDrawStyle(DlDrawStyle::kStroke));
  auto display_list_stroked = builder_stroked.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list_filled.get()),
              calculator->Compute(display_list_stroked.get()));
  }
}

TEST(DisplayListComplexity, SaveLayers) {
  DisplayListBuilder builder;
  builder.SaveLayer(std::nullopt, nullptr);
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawPath) {
  DisplayListBuilder builder_line;
  DlPathBuilder line_path_builder;
  line_path_builder.MoveTo(DlPoint(0, 0));
  line_path_builder.LineTo(DlPoint(10, 10));
  line_path_builder.Close();
  builder_line.DrawPath(DlPath(line_path_builder), DlPaint());
  auto display_list_line = builder_line.Build();

  DisplayListBuilder builder_quad;
  DlPathBuilder quad_path_builder;
  quad_path_builder.MoveTo(DlPoint(0, 0));
  quad_path_builder.QuadraticCurveTo(DlPoint(10, 10), DlPoint(10, 20));
  quad_path_builder.Close();
  builder_quad.DrawPath(DlPath(quad_path_builder), DlPaint());
  auto display_list_quad = builder_quad.Build();

  DisplayListBuilder builder_conic;
  DlPathBuilder conic_path_builder;
  conic_path_builder.MoveTo(DlPoint(0, 0));
  conic_path_builder.ConicCurveTo(DlPoint(10, 10), DlPoint(10, 20), 1.5f);
  conic_path_builder.Close();
  builder_conic.DrawPath(DlPath(conic_path_builder), DlPaint());
  auto display_list_conic = builder_conic.Build();

  DisplayListBuilder builder_cubic;
  DlPathBuilder cubic_path_builder;
  cubic_path_builder.MoveTo(DlPoint(0, 0));
  cubic_path_builder.CubicCurveTo(DlPoint(10, 10), DlPoint(10, 20),
                                  DlPoint(20, 20));
  builder_cubic.DrawPath(DlPath(cubic_path_builder), DlPaint());
  auto display_list_cubic = builder_cubic.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list_line.get()), 0u);
    ASSERT_NE(calculator->Compute(display_list_quad.get()), 0u);
    ASSERT_NE(calculator->Compute(display_list_conic.get()), 0u);
    ASSERT_NE(calculator->Compute(display_list_cubic.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawShadow) {
  DisplayListBuilder builder_line;
  DlPathBuilder line_path_builder;
  line_path_builder.MoveTo(DlPoint(0, 0));
  line_path_builder.LineTo(DlPoint(10, 10));
  line_path_builder.Close();
  builder_line.DrawShadow(DlPath(line_path_builder), DlColor(SK_ColorRED),
                          10.0f, false, 1.0f);
  auto display_list_line = builder_line.Build();

  DisplayListBuilder builder_quad;
  DlPathBuilder quad_path_builder;
  quad_path_builder.MoveTo(DlPoint(0, 0));
  quad_path_builder.QuadraticCurveTo(DlPoint(10, 10), DlPoint(10, 20));
  quad_path_builder.Close();
  builder_quad.DrawShadow(DlPath(quad_path_builder), DlColor(SK_ColorRED),
                          10.0f, false, 1.0f);
  auto display_list_quad = builder_quad.Build();

  DisplayListBuilder builder_conic;
  DlPathBuilder conic_path_builder;
  conic_path_builder.MoveTo(DlPoint(0, 0));
  conic_path_builder.ConicCurveTo(DlPoint(10, 10), DlPoint(10, 20), 1.5f);
  conic_path_builder.Close();
  builder_conic.DrawShadow(DlPath(conic_path_builder), DlColor(SK_ColorRED),
                           10.0f, false, 1.0f);
  auto display_list_conic = builder_conic.Build();

  DisplayListBuilder builder_cubic;
  DlPathBuilder cubic_path_builder;
  cubic_path_builder.MoveTo(DlPoint(0, 0));
  cubic_path_builder.CubicCurveTo(DlPoint(10, 10), DlPoint(10, 20),
                                  DlPoint(20, 20));
  builder_cubic.DrawShadow(DlPath(cubic_path_builder), DlColor(SK_ColorRED),
                           10.0f, false, 1.0f);
  auto display_list_cubic = builder_cubic.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list_line.get()), 0u);
    ASSERT_NE(calculator->Compute(display_list_quad.get()), 0u);
    ASSERT_NE(calculator->Compute(display_list_conic.get()), 0u);
    ASSERT_NE(calculator->Compute(display_list_cubic.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawOval) {
  DisplayListBuilder builder;
  builder.DrawOval(DlRect::MakeXYWH(10, 10, 100, 80), DlPaint());
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawCircle) {
  DisplayListBuilder builder;
  builder.DrawCircle(DlPoint(50, 50), 10.0f, DlPaint());
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawRoundRect) {
  DisplayListBuilder builder;
  builder.DrawRoundRect(
      DlRoundRect::MakeRectXY(DlRect::MakeXYWH(10, 10, 80, 80), 2.0f, 3.0f),
      DlPaint());
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawDiffRoundRect) {
  DisplayListBuilder builder;
  DlRoundRect outer =
      DlRoundRect::MakeRectXY(DlRect::MakeXYWH(10, 10, 80, 80), 2.0f, 3.0f);
  DlRoundRect inner =
      DlRoundRect::MakeRectXY(DlRect::MakeXYWH(15, 15, 70, 70), 1.5f, 1.5f);
  builder.DrawDiffRoundRect(outer, inner, DlPaint());
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawArc) {
  DisplayListBuilder builder;
  builder.DrawArc(DlRect::MakeXYWH(10, 10, 100, 80), 0.0f, 10.0f, true,
                  DlPaint());
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawVertices) {
  auto points = GetTestPoints();
  auto vertices = DlVertices::Make(DlVertexMode::kTriangles, points.size(),
                                   points.data(), nullptr, nullptr);
  DisplayListBuilder builder;
  builder.DrawVertices(vertices, DlBlendMode::kSrc, DlPaint());
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawTextBlob) {
  auto text_blob =
      GetTestTextBlob("The quick brown fox jumps over the lazy dog.", 20.0f);

  DisplayListBuilder builder;
  builder.DrawTextBlob(text_blob, 0.0f, 0.0f, DlPaint());
  auto display_list = builder.Build();

  DisplayListBuilder builder_multiple;
  builder_multiple.DrawTextBlob(text_blob, 0.0f, 0.0f, DlPaint());
  builder_multiple.DrawTextBlob(text_blob, 0.0f, 0.0f, DlPaint());
  auto display_list_multiple = builder_multiple.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
    ASSERT_GT(calculator->Compute(display_list_multiple.get()),
              calculator->Compute(display_list.get()));
  }
}

TEST(DisplayListComplexity, DrawPoints) {
  auto points = GetTestPoints();
  DisplayListBuilder builder_lines;
  builder_lines.DrawPoints(DlPointMode::kLines, points.size(), points.data(),
                           DlPaint());
  auto display_list_lines = builder_lines.Build();

  DisplayListBuilder builder_points;
  builder_points.DrawPoints(DlPointMode::kPoints, points.size(), points.data(),
                            DlPaint());
  auto display_list_points = builder_points.Build();

  DisplayListBuilder builder_polygon;
  builder_polygon.DrawPoints(DlPointMode::kPolygon, points.size(),
                             points.data(), DlPaint());
  auto display_list_polygon = builder_polygon.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list_lines.get()), 0u);
    ASSERT_NE(calculator->Compute(display_list_points.get()), 0u);
    ASSERT_NE(calculator->Compute(display_list_polygon.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawImage) {
  auto image = MakeTestImage(50, 50, DlColor::kBlue().withAlphaF(0.5));

  DisplayListBuilder builder;
  builder.DrawImage(image, DlPoint(0, 0), DlImageSampling::kNearestNeighbor,
                    nullptr);
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawImageNine) {
  auto image = MakeTestImage(50, 50, DlColor::kBlue().withAlphaF(0.5));

  DlIRect center = DlIRect::MakeXYWH(5, 5, 20, 20);
  DlRect dest = DlRect::MakeXYWH(0, 0, 50, 50);

  DisplayListBuilder builder;
  builder.DrawImageNine(image, center, dest, DlFilterMode::kNearest, nullptr);
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawImageRect) {
  auto image = MakeTestImage(50, 50, DlColor::kBlue().withAlphaF(0.5));

  DlRect src = DlRect::MakeXYWH(0, 0, 50, 50);
  DlRect dest = DlRect::MakeXYWH(0, 0, 50, 50);

  DisplayListBuilder builder;
  builder.DrawImageRect(image, src, dest, DlImageSampling::kNearestNeighbor,
                        nullptr);
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

TEST(DisplayListComplexity, DrawAtlas) {
  auto image = MakeTestImage(50, 50, DlColor::kBlue().withAlphaF(0.5));

  std::vector<DlRect> rects;
  std::vector<DlRSTransform> xforms;
  for (int i = 0; i < 10; i++) {
    rects.push_back(DlRect::MakeXYWH(0, 0, 10, 10));
    xforms.push_back(DlRSTransform(1, 0, 0, 0));
  }

  DisplayListBuilder builder;
  builder.DrawAtlas(image, xforms.data(), rects.data(), nullptr, 10,
                    DlBlendMode::kSrc, DlImageSampling::kNearestNeighbor,
                    nullptr, nullptr);
  auto display_list = builder.Build();

  auto calculators = AccumulatorCalculators();
  for (auto calculator : calculators) {
    ASSERT_NE(calculator->Compute(display_list.get()), 0u);
  }
}

}  // namespace testing
}  // namespace flutter

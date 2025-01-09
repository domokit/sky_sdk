// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <algorithm>
#include <cstring>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "flutter/display_list/testing/dl_test_snippets.h"
#include "fml/logging.h"
#include "gtest/gtest.h"
#include "impeller/core/device_buffer.h"
#include "impeller/core/formats.h"
#include "impeller/core/host_buffer.h"
#include "impeller/core/raw_ptr.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/entity/contents/clip_contents.h"
#include "impeller/entity/contents/conical_gradient_contents.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/contents/contents.h"
#include "impeller/entity/contents/filters/color_filter_contents.h"
#include "impeller/entity/contents/filters/filter_contents.h"
#include "impeller/entity/contents/filters/gaussian_blur_filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"
#include "impeller/entity/contents/linear_gradient_contents.h"
#include "impeller/entity/contents/radial_gradient_contents.h"
#include "impeller/entity/contents/runtime_effect_contents.h"
#include "impeller/entity/contents/solid_color_contents.h"
#include "impeller/entity/contents/solid_rrect_blur_contents.h"
#include "impeller/entity/contents/sweep_gradient_contents.h"
#include "impeller/entity/contents/text_contents.h"
#include "impeller/entity/contents/texture_contents.h"
#include "impeller/entity/contents/tiled_texture_contents.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/entity_playground.h"
#include "impeller/entity/geometry/geometry.h"
#include "impeller/entity/geometry/point_field_geometry.h"
#include "impeller/entity/geometry/round_superellipse_geometry.h"
#include "impeller/entity/geometry/stroke_path_geometry.h"
#include "impeller/entity/geometry/superellipse_geometry.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/geometry_asserts.h"
#include "impeller/geometry/path_builder.h"
#include "impeller/geometry/point.h"
#include "impeller/geometry/sigma.h"
#include "impeller/geometry/vector.h"
#include "impeller/playground/playground.h"
#include "impeller/playground/widgets.h"
#include "impeller/renderer/command.h"
#include "impeller/renderer/pipeline_descriptor.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/render_target.h"
#include "impeller/renderer/testing/mocks.h"
#include "impeller/renderer/vertex_buffer_builder.h"
#include "impeller/typographer/backends/skia/text_frame_skia.h"
#include "impeller/typographer/backends/skia/typographer_context_skia.h"
#include "third_party/imgui/imgui.h"
#include "third_party/skia/include/core/SkTextBlob.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace impeller {
namespace testing {

using EntityTest = EntityPlayground;
INSTANTIATE_PLAYGROUND_SUITE(EntityTest);

TEST_P(EntityTest, CanCreateEntity) {
  Entity entity;
  ASSERT_TRUE(entity.GetTransform().IsIdentity());
}

TEST_P(EntityTest, FilterCoverageRespectsCropRect) {
  auto image = CreateTextureForFixture("boston.jpg");
  auto filter = ColorFilterContents::MakeBlend(BlendMode::kSoftLight,
                                               FilterInput::Make({image}));

  // Without the crop rect (default behavior).
  {
    auto actual = filter->GetCoverage({});
    auto expected = Rect::MakeSize(image->GetSize());

    ASSERT_TRUE(actual.has_value());
    ASSERT_RECT_NEAR(actual.value(), expected);
  }

  // With the crop rect.
  {
    auto expected = Rect::MakeLTRB(50, 50, 100, 100);
    filter->SetCoverageHint(expected);
    auto actual = filter->GetCoverage({});

    ASSERT_TRUE(actual.has_value());
    ASSERT_RECT_NEAR(actual.value(), expected);
  }
}

TEST_P(EntityTest, GeometryBoundsAreTransformed) {
  auto geometry = Geometry::MakeRect(Rect::MakeXYWH(100, 100, 100, 100));
  auto transform = Matrix::MakeScale({2.0, 2.0, 2.0});

  ASSERT_RECT_NEAR(geometry->GetCoverage(transform).value(),
                   Rect::MakeXYWH(200, 200, 200, 200));
}

TEST_P(EntityTest, ThreeStrokesInOnePath) {
  Path path = PathBuilder{}
                  .MoveTo({100, 100})
                  .LineTo({100, 200})
                  .MoveTo({100, 300})
                  .LineTo({100, 400})
                  .MoveTo({100, 500})
                  .LineTo({100, 600})
                  .TakePath();

  Entity entity;
  entity.SetTransform(Matrix::MakeScale(GetContentScale()));
  auto contents = std::make_unique<SolidColorContents>();

  std::unique_ptr<Geometry> geom = Geometry::MakeStrokePath(path, 5.0);
  contents->SetGeometry(geom.get());
  contents->SetColor(Color::Red());
  entity.SetContents(std::move(contents));
  ASSERT_TRUE(OpenPlaygroundHere(std::move(entity)));
}

TEST_P(EntityTest, StrokeWithTextureContents) {
  auto bridge = CreateTextureForFixture("bay_bridge.jpg");
  Path path = PathBuilder{}
                  .MoveTo({100, 100})
                  .LineTo({100, 200})
                  .MoveTo({100, 300})
                  .LineTo({100, 400})
                  .MoveTo({100, 500})
                  .LineTo({100, 600})
                  .TakePath();

  Entity entity;
  entity.SetTransform(Matrix::MakeScale(GetContentScale()));
  auto contents = std::make_unique<TiledTextureContents>();
  std::unique_ptr<Geometry> geom = Geometry::MakeStrokePath(path, 100.0);
  contents->SetGeometry(geom.get());
  contents->SetTexture(bridge);
  contents->SetTileModes(Entity::TileMode::kClamp, Entity::TileMode::kClamp);
  entity.SetContents(std::move(contents));
  ASSERT_TRUE(OpenPlaygroundHere(std::move(entity)));
}

TEST_P(EntityTest, TriangleInsideASquare) {
  auto callback = [&](ContentContext& context, RenderPass& pass) {
    Point offset(100, 100);

    PlaygroundPoint point_a(Point(10, 10) + offset, 20, Color::White());
    Point a = DrawPlaygroundPoint(point_a);
    PlaygroundPoint point_b(Point(210, 10) + offset, 20, Color::White());
    Point b = DrawPlaygroundPoint(point_b);
    PlaygroundPoint point_c(Point(210, 210) + offset, 20, Color::White());
    Point c = DrawPlaygroundPoint(point_c);
    PlaygroundPoint point_d(Point(10, 210) + offset, 20, Color::White());
    Point d = DrawPlaygroundPoint(point_d);
    PlaygroundPoint point_e(Point(50, 50) + offset, 20, Color::White());
    Point e = DrawPlaygroundPoint(point_e);
    PlaygroundPoint point_f(Point(100, 50) + offset, 20, Color::White());
    Point f = DrawPlaygroundPoint(point_f);
    PlaygroundPoint point_g(Point(50, 150) + offset, 20, Color::White());
    Point g = DrawPlaygroundPoint(point_g);
    Path path = PathBuilder{}
                    .MoveTo(a)
                    .LineTo(b)
                    .LineTo(c)
                    .LineTo(d)
                    .Close()
                    .MoveTo(e)
                    .LineTo(f)
                    .LineTo(g)
                    .Close()
                    .TakePath();

    Entity entity;
    entity.SetTransform(Matrix::MakeScale(GetContentScale()));
    auto contents = std::make_unique<SolidColorContents>();
    std::unique_ptr<Geometry> geom = Geometry::MakeStrokePath(path, 20.0);
    contents->SetGeometry(geom.get());
    contents->SetColor(Color::Red());
    entity.SetContents(std::move(contents));

    return entity.Render(context, pass);
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, StrokeCapAndJoinTest) {
  const Point padding(300, 250);
  const Point margin(140, 180);

  auto callback = [&](ContentContext& context, RenderPass& pass) {
    // Slightly above sqrt(2) by default, so that right angles are just below
    // the limit and acute angles are over the limit (causing them to get
    // beveled).
    static Scalar miter_limit = 1.41421357;
    static Scalar width = 30;

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
      ImGui::SliderFloat("Miter limit", &miter_limit, 0, 30);
      ImGui::SliderFloat("Stroke width", &width, 0, 100);
      if (ImGui::Button("Reset")) {
        miter_limit = 1.41421357;
        width = 30;
      }
    }
    ImGui::End();

    auto world_matrix = Matrix::MakeScale(GetContentScale());
    auto render_path = [width = width, &context, &pass, &world_matrix](
                           const Path& path, Cap cap, Join join) {
      auto contents = std::make_unique<SolidColorContents>();
      std::unique_ptr<Geometry> geom =
          Geometry::MakeStrokePath(path, width, miter_limit, cap, join);
      contents->SetGeometry(geom.get());
      contents->SetColor(Color::Red());

      Entity entity;
      entity.SetTransform(world_matrix);
      entity.SetContents(std::move(contents));

      auto coverage = entity.GetCoverage();
      if (coverage.has_value()) {
        auto bounds_contents = std::make_unique<SolidColorContents>();

        std::unique_ptr<Geometry> geom = Geometry::MakeFillPath(
            PathBuilder{}.AddRect(entity.GetCoverage().value()).TakePath());

        bounds_contents->SetGeometry(geom.get());
        bounds_contents->SetColor(Color::Green().WithAlpha(0.5));
        Entity bounds_entity;
        bounds_entity.SetContents(std::move(bounds_contents));
        bounds_entity.Render(context, pass);
      }

      entity.Render(context, pass);
    };

    const Point a_def(0, 0), b_def(0, 100), c_def(150, 0), d_def(150, -100),
        e_def(75, 75);
    const Scalar r = 30;
    // Cap::kButt demo.
    {
      Point off = Point(0, 0) * padding + margin;
      PlaygroundPoint point_a(off + a_def, r, Color::Black());
      PlaygroundPoint point_b(off + b_def, r, Color::White());
      auto [a, b] = DrawPlaygroundLine(point_a, point_b);
      PlaygroundPoint point_c(off + c_def, r, Color::Black());
      PlaygroundPoint point_d(off + d_def, r, Color::White());
      auto [c, d] = DrawPlaygroundLine(point_c, point_d);
      render_path(PathBuilder{}.AddCubicCurve(a, b, d, c).TakePath(),
                  Cap::kButt, Join::kBevel);
    }

    // Cap::kSquare demo.
    {
      Point off = Point(1, 0) * padding + margin;
      PlaygroundPoint point_a(off + a_def, r, Color::Black());
      PlaygroundPoint point_b(off + b_def, r, Color::White());
      auto [a, b] = DrawPlaygroundLine(point_a, point_b);
      PlaygroundPoint point_c(off + c_def, r, Color::Black());
      PlaygroundPoint point_d(off + d_def, r, Color::White());
      auto [c, d] = DrawPlaygroundLine(point_c, point_d);
      render_path(PathBuilder{}.AddCubicCurve(a, b, d, c).TakePath(),
                  Cap::kSquare, Join::kBevel);
    }

    // Cap::kRound demo.
    {
      Point off = Point(2, 0) * padding + margin;
      PlaygroundPoint point_a(off + a_def, r, Color::Black());
      PlaygroundPoint point_b(off + b_def, r, Color::White());
      auto [a, b] = DrawPlaygroundLine(point_a, point_b);
      PlaygroundPoint point_c(off + c_def, r, Color::Black());
      PlaygroundPoint point_d(off + d_def, r, Color::White());
      auto [c, d] = DrawPlaygroundLine(point_c, point_d);
      render_path(PathBuilder{}.AddCubicCurve(a, b, d, c).TakePath(),
                  Cap::kRound, Join::kBevel);
    }

    // Join::kBevel demo.
    {
      Point off = Point(0, 1) * padding + margin;
      PlaygroundPoint point_a = PlaygroundPoint(off + a_def, r, Color::White());
      PlaygroundPoint point_b = PlaygroundPoint(off + e_def, r, Color::White());
      PlaygroundPoint point_c = PlaygroundPoint(off + c_def, r, Color::White());
      Point a = DrawPlaygroundPoint(point_a);
      Point b = DrawPlaygroundPoint(point_b);
      Point c = DrawPlaygroundPoint(point_c);
      render_path(
          PathBuilder{}.MoveTo(a).LineTo(b).LineTo(c).Close().TakePath(),
          Cap::kButt, Join::kBevel);
    }

    // Join::kMiter demo.
    {
      Point off = Point(1, 1) * padding + margin;
      PlaygroundPoint point_a(off + a_def, r, Color::White());
      PlaygroundPoint point_b(off + e_def, r, Color::White());
      PlaygroundPoint point_c(off + c_def, r, Color::White());
      Point a = DrawPlaygroundPoint(point_a);
      Point b = DrawPlaygroundPoint(point_b);
      Point c = DrawPlaygroundPoint(point_c);
      render_path(
          PathBuilder{}.MoveTo(a).LineTo(b).LineTo(c).Close().TakePath(),
          Cap::kButt, Join::kMiter);
    }

    // Join::kRound demo.
    {
      Point off = Point(2, 1) * padding + margin;
      PlaygroundPoint point_a(off + a_def, r, Color::White());
      PlaygroundPoint point_b(off + e_def, r, Color::White());
      PlaygroundPoint point_c(off + c_def, r, Color::White());
      Point a = DrawPlaygroundPoint(point_a);
      Point b = DrawPlaygroundPoint(point_b);
      Point c = DrawPlaygroundPoint(point_c);
      render_path(
          PathBuilder{}.MoveTo(a).LineTo(b).LineTo(c).Close().TakePath(),
          Cap::kButt, Join::kRound);
    }

    return true;
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, CubicCurveTest) {
  // Compare with https://fiddle.skia.org/c/b3625f26122c9de7afe7794fcf25ead3
  Path path =
      PathBuilder{}
          .MoveTo({237.164, 125.003})
          .CubicCurveTo({236.709, 125.184}, {236.262, 125.358},
                        {235.81, 125.538})
          .CubicCurveTo({235.413, 125.68}, {234.994, 125.832},
                        {234.592, 125.977})
          .CubicCurveTo({234.592, 125.977}, {234.591, 125.977},
                        {234.59, 125.977})
          .CubicCurveTo({222.206, 130.435}, {207.708, 135.753},
                        {192.381, 141.429})
          .CubicCurveTo({162.77, 151.336}, {122.17, 156.894}, {84.1123, 160})
          .Close()
          .TakePath();
  Entity entity;
  entity.SetTransform(Matrix::MakeScale(GetContentScale()));

  std::unique_ptr<Geometry> geom = Geometry::MakeFillPath(path);

  auto contents = std::make_shared<SolidColorContents>();
  contents->SetColor(Color::Red());
  contents->SetGeometry(geom.get());

  entity.SetContents(contents);
  ASSERT_TRUE(OpenPlaygroundHere(std::move(entity)));
}

TEST_P(EntityTest, CanDrawCorrectlyWithRotatedTransform) {
  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    const char* input_axis[] = {"X", "Y", "Z"};
    static int rotation_axis_index = 0;
    static float rotation = 0;
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat("Rotation", &rotation, -kPi, kPi);
    ImGui::Combo("Rotation Axis", &rotation_axis_index, input_axis,
                 sizeof(input_axis) / sizeof(char*));
    Matrix rotation_matrix;
    switch (rotation_axis_index) {
      case 0:
        rotation_matrix = Matrix::MakeRotationX(Radians(rotation));
        break;
      case 1:
        rotation_matrix = Matrix::MakeRotationY(Radians(rotation));
        break;
      case 2:
        rotation_matrix = Matrix::MakeRotationZ(Radians(rotation));
        break;
      default:
        rotation_matrix = Matrix{};
        break;
    }

    if (ImGui::Button("Reset")) {
      rotation = 0;
    }
    ImGui::End();
    Matrix current_transform =
        Matrix::MakeScale(GetContentScale())
            .MakeTranslation(
                Vector3(Point(pass.GetRenderTargetSize().width / 2.0,
                              pass.GetRenderTargetSize().height / 2.0)));
    Matrix result_transform = current_transform * rotation_matrix;
    Path path =
        PathBuilder{}.AddRect(Rect::MakeXYWH(-300, -400, 600, 800)).TakePath();

    Entity entity;
    entity.SetTransform(result_transform);

    std::unique_ptr<Geometry> geom = Geometry::MakeFillPath(path);

    auto contents = std::make_shared<SolidColorContents>();
    contents->SetColor(Color::Red());
    contents->SetGeometry(geom.get());

    entity.SetContents(contents);
    return entity.Render(context, pass);
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, CubicCurveAndOverlapTest) {
  // Compare with https://fiddle.skia.org/c/7a05a3e186c65a8dfb732f68020aae06
  Path path =
      PathBuilder{}
          .MoveTo({359.934, 96.6335})
          .CubicCurveTo({358.189, 96.7055}, {356.436, 96.7908},
                        {354.673, 96.8895})
          .CubicCurveTo({354.571, 96.8953}, {354.469, 96.9016},
                        {354.367, 96.9075})
          .CubicCurveTo({352.672, 97.0038}, {350.969, 97.113},
                        {349.259, 97.2355})
          .CubicCurveTo({349.048, 97.2506}, {348.836, 97.2678},
                        {348.625, 97.2834})
          .CubicCurveTo({347.019, 97.4014}, {345.407, 97.5299},
                        {343.789, 97.6722})
          .CubicCurveTo({343.428, 97.704}, {343.065, 97.7402},
                        {342.703, 97.7734})
          .CubicCurveTo({341.221, 97.9086}, {339.736, 98.0505},
                        {338.246, 98.207})
          .CubicCurveTo({337.702, 98.2642}, {337.156, 98.3292},
                        {336.612, 98.3894})
          .CubicCurveTo({335.284, 98.5356}, {333.956, 98.6837},
                        {332.623, 98.8476})
          .CubicCurveTo({332.495, 98.8635}, {332.366, 98.8818},
                        {332.237, 98.8982})
          .LineTo({332.237, 102.601})
          .LineTo({321.778, 102.601})
          .LineTo({321.778, 100.382})
          .CubicCurveTo({321.572, 100.413}, {321.367, 100.442},
                        {321.161, 100.476})
          .CubicCurveTo({319.22, 100.79}, {317.277, 101.123},
                        {315.332, 101.479})
          .CubicCurveTo({315.322, 101.481}, {315.311, 101.482},
                        {315.301, 101.484})
          .LineTo({310.017, 105.94})
          .LineTo({309.779, 105.427})
          .LineTo({314.403, 101.651})
          .CubicCurveTo({314.391, 101.653}, {314.379, 101.656},
                        {314.368, 101.658})
          .CubicCurveTo({312.528, 102.001}, {310.687, 102.366},
                        {308.846, 102.748})
          .CubicCurveTo({307.85, 102.955}, {306.855, 103.182}, {305.859, 103.4})
          .CubicCurveTo({305.048, 103.579}, {304.236, 103.75},
                        {303.425, 103.936})
          .LineTo({299.105, 107.578})
          .LineTo({298.867, 107.065})
          .LineTo({302.394, 104.185})
          .LineTo({302.412, 104.171})
          .CubicCurveTo({301.388, 104.409}, {300.366, 104.67},
                        {299.344, 104.921})
          .CubicCurveTo({298.618, 105.1}, {297.89, 105.269}, {297.165, 105.455})
          .CubicCurveTo({295.262, 105.94}, {293.36, 106.445},
                        {291.462, 106.979})
          .CubicCurveTo({291.132, 107.072}, {290.802, 107.163},
                        {290.471, 107.257})
          .CubicCurveTo({289.463, 107.544}, {288.455, 107.839},
                        {287.449, 108.139})
          .CubicCurveTo({286.476, 108.431}, {285.506, 108.73},
                        {284.536, 109.035})
          .CubicCurveTo({283.674, 109.304}, {282.812, 109.579},
                        {281.952, 109.859})
          .CubicCurveTo({281.177, 110.112}, {280.406, 110.377},
                        {279.633, 110.638})
          .CubicCurveTo({278.458, 111.037}, {277.256, 111.449},
                        {276.803, 111.607})
          .CubicCurveTo({276.76, 111.622}, {276.716, 111.637},
                        {276.672, 111.653})
          .CubicCurveTo({275.017, 112.239}, {273.365, 112.836},
                        {271.721, 113.463})
          .LineTo({271.717, 113.449})
          .CubicCurveTo({271.496, 113.496}, {271.238, 113.559},
                        {270.963, 113.628})
          .CubicCurveTo({270.893, 113.645}, {270.822, 113.663},
                        {270.748, 113.682})
          .CubicCurveTo({270.468, 113.755}, {270.169, 113.834},
                        {269.839, 113.926})
          .CubicCurveTo({269.789, 113.94}, {269.732, 113.957},
                        {269.681, 113.972})
          .CubicCurveTo({269.391, 114.053}, {269.081, 114.143},
                        {268.756, 114.239})
          .CubicCurveTo({268.628, 114.276}, {268.5, 114.314},
                        {268.367, 114.354})
          .CubicCurveTo({268.172, 114.412}, {267.959, 114.478},
                        {267.752, 114.54})
          .CubicCurveTo({263.349, 115.964}, {258.058, 117.695},
                        {253.564, 119.252})
          .CubicCurveTo({253.556, 119.255}, {253.547, 119.258},
                        {253.538, 119.261})
          .CubicCurveTo({251.844, 119.849}, {250.056, 120.474},
                        {248.189, 121.131})
          .CubicCurveTo({248, 121.197}, {247.812, 121.264}, {247.621, 121.331})
          .CubicCurveTo({247.079, 121.522}, {246.531, 121.715},
                        {245.975, 121.912})
          .CubicCurveTo({245.554, 122.06}, {245.126, 122.212},
                        {244.698, 122.364})
          .CubicCurveTo({244.071, 122.586}, {243.437, 122.811},
                        {242.794, 123.04})
          .CubicCurveTo({242.189, 123.255}, {241.58, 123.472},
                        {240.961, 123.693})
          .CubicCurveTo({240.659, 123.801}, {240.357, 123.909},
                        {240.052, 124.018})
          .CubicCurveTo({239.12, 124.351}, {238.18, 124.687}, {237.22, 125.032})
          .LineTo({237.164, 125.003})
          .CubicCurveTo({236.709, 125.184}, {236.262, 125.358},
                        {235.81, 125.538})
          .CubicCurveTo({235.413, 125.68}, {234.994, 125.832},
                        {234.592, 125.977})
          .CubicCurveTo({234.592, 125.977}, {234.591, 125.977},
                        {234.59, 125.977})
          .CubicCurveTo({222.206, 130.435}, {207.708, 135.753},
                        {192.381, 141.429})
          .CubicCurveTo({162.77, 151.336}, {122.17, 156.894}, {84.1123, 160})
          .LineTo({360, 160})
          .LineTo({360, 119.256})
          .LineTo({360, 106.332})
          .LineTo({360, 96.6307})
          .CubicCurveTo({359.978, 96.6317}, {359.956, 96.6326},
                        {359.934, 96.6335})
          .Close()
          .MoveTo({337.336, 124.143})
          .CubicCurveTo({337.274, 122.359}, {338.903, 121.511},
                        {338.903, 121.511})
          .CubicCurveTo({338.903, 121.511}, {338.96, 123.303},
                        {337.336, 124.143})
          .Close()
          .MoveTo({340.082, 121.849})
          .CubicCurveTo({340.074, 121.917}, {340.062, 121.992},
                        {340.046, 122.075})
          .CubicCurveTo({340.039, 122.109}, {340.031, 122.142},
                        {340.023, 122.177})
          .CubicCurveTo({340.005, 122.26}, {339.98, 122.346},
                        {339.952, 122.437})
          .CubicCurveTo({339.941, 122.473}, {339.931, 122.507},
                        {339.918, 122.544})
          .CubicCurveTo({339.873, 122.672}, {339.819, 122.804},
                        {339.75, 122.938})
          .CubicCurveTo({339.747, 122.944}, {339.743, 122.949},
                        {339.74, 122.955})
          .CubicCurveTo({339.674, 123.08}, {339.593, 123.205},
                        {339.501, 123.328})
          .CubicCurveTo({339.473, 123.366}, {339.441, 123.401},
                        {339.41, 123.438})
          .CubicCurveTo({339.332, 123.534}, {339.243, 123.625},
                        {339.145, 123.714})
          .CubicCurveTo({339.105, 123.75}, {339.068, 123.786},
                        {339.025, 123.821})
          .CubicCurveTo({338.881, 123.937}, {338.724, 124.048},
                        {338.539, 124.143})
          .CubicCurveTo({338.532, 123.959}, {338.554, 123.79},
                        {338.58, 123.626})
          .CubicCurveTo({338.58, 123.625}, {338.58, 123.625}, {338.58, 123.625})
          .CubicCurveTo({338.607, 123.455}, {338.65, 123.299},
                        {338.704, 123.151})
          .CubicCurveTo({338.708, 123.14}, {338.71, 123.127},
                        {338.714, 123.117})
          .CubicCurveTo({338.769, 122.971}, {338.833, 122.838},
                        {338.905, 122.712})
          .CubicCurveTo({338.911, 122.702}, {338.916, 122.69200000000001},
                        {338.922, 122.682})
          .CubicCurveTo({338.996, 122.557}, {339.072, 122.444},
                        {339.155, 122.34})
          .CubicCurveTo({339.161, 122.333}, {339.166, 122.326},
                        {339.172, 122.319})
          .CubicCurveTo({339.256, 122.215}, {339.339, 122.12},
                        {339.425, 122.037})
          .CubicCurveTo({339.428, 122.033}, {339.431, 122.03},
                        {339.435, 122.027})
          .CubicCurveTo({339.785, 121.687}, {340.106, 121.511},
                        {340.106, 121.511})
          .CubicCurveTo({340.106, 121.511}, {340.107, 121.645},
                        {340.082, 121.849})
          .Close()
          .MoveTo({340.678, 113.245})
          .CubicCurveTo({340.594, 113.488}, {340.356, 113.655},
                        {340.135, 113.775})
          .CubicCurveTo({339.817, 113.948}, {339.465, 114.059},
                        {339.115, 114.151})
          .CubicCurveTo({338.251, 114.379}, {337.34, 114.516},
                        {336.448, 114.516})
          .CubicCurveTo({335.761, 114.516}, {335.072, 114.527},
                        {334.384, 114.513})
          .CubicCurveTo({334.125, 114.508}, {333.862, 114.462},
                        {333.605, 114.424})
          .CubicCurveTo({332.865, 114.318}, {332.096, 114.184},
                        {331.41, 113.883})
          .CubicCurveTo({330.979, 113.695}, {330.442, 113.34},
                        {330.672, 112.813})
          .CubicCurveTo({331.135, 111.755}, {333.219, 112.946},
                        {334.526, 113.833})
          .CubicCurveTo({334.54, 113.816}, {334.554, 113.8}, {334.569, 113.784})
          .CubicCurveTo({333.38, 112.708}, {331.749, 110.985},
                        {332.76, 110.402})
          .CubicCurveTo({333.769, 109.82}, {334.713, 111.93},
                        {335.228, 113.395})
          .CubicCurveTo({334.915, 111.889}, {334.59, 109.636},
                        {335.661, 109.592})
          .CubicCurveTo({336.733, 109.636}, {336.408, 111.889},
                        {336.07, 113.389})
          .CubicCurveTo({336.609, 111.93}, {337.553, 109.82},
                        {338.563, 110.402})
          .CubicCurveTo({339.574, 110.984}, {337.942, 112.708},
                        {336.753, 113.784})
          .CubicCurveTo({336.768, 113.8}, {336.782, 113.816},
                        {336.796, 113.833})
          .CubicCurveTo({338.104, 112.946}, {340.187, 111.755},
                        {340.65, 112.813})
          .CubicCurveTo({340.71, 112.95}, {340.728, 113.102},
                        {340.678, 113.245})
          .Close()
          .MoveTo({346.357, 106.771})
          .CubicCurveTo({346.295, 104.987}, {347.924, 104.139},
                        {347.924, 104.139})
          .CubicCurveTo({347.924, 104.139}, {347.982, 105.931},
                        {346.357, 106.771})
          .Close()
          .MoveTo({347.56, 106.771})
          .CubicCurveTo({347.498, 104.987}, {349.127, 104.139},
                        {349.127, 104.139})
          .CubicCurveTo({349.127, 104.139}, {349.185, 105.931},
                        {347.56, 106.771})
          .Close()
          .TakePath();
  Entity entity;
  entity.SetTransform(Matrix::MakeScale(GetContentScale()));

  std::unique_ptr<Geometry> geom = Geometry::MakeFillPath(path);

  auto contents = std::make_shared<SolidColorContents>();
  contents->SetColor(Color::Red());
  contents->SetGeometry(geom.get());

  entity.SetContents(contents);
  ASSERT_TRUE(OpenPlaygroundHere(std::move(entity)));
}

TEST_P(EntityTest, SolidColorContentsStrokeSetStrokeCapsAndJoins) {
  {
    auto geometry = Geometry::MakeStrokePath(Path{});
    auto path_geometry = static_cast<StrokePathGeometry*>(geometry.get());
    // Defaults.
    ASSERT_EQ(path_geometry->GetStrokeCap(), Cap::kButt);
    ASSERT_EQ(path_geometry->GetStrokeJoin(), Join::kMiter);
  }

  {
    auto geometry = Geometry::MakeStrokePath(Path{}, 1.0, 4.0, Cap::kSquare);
    auto path_geometry = static_cast<StrokePathGeometry*>(geometry.get());
    ASSERT_EQ(path_geometry->GetStrokeCap(), Cap::kSquare);
  }

  {
    auto geometry = Geometry::MakeStrokePath(Path{}, 1.0, 4.0, Cap::kRound);
    auto path_geometry = static_cast<StrokePathGeometry*>(geometry.get());
    ASSERT_EQ(path_geometry->GetStrokeCap(), Cap::kRound);
  }
}

TEST_P(EntityTest, SolidColorContentsStrokeSetMiterLimit) {
  {
    auto geometry = Geometry::MakeStrokePath(Path{});
    auto path_geometry = static_cast<StrokePathGeometry*>(geometry.get());
    ASSERT_FLOAT_EQ(path_geometry->GetMiterLimit(), 4);
  }

  {
    auto geometry = Geometry::MakeStrokePath(Path{}, 1.0,
                                             /*miter_limit=*/8.0);
    auto path_geometry = static_cast<StrokePathGeometry*>(geometry.get());
    ASSERT_FLOAT_EQ(path_geometry->GetMiterLimit(), 8);
  }

  {
    auto geometry = Geometry::MakeStrokePath(Path{}, 1.0,
                                             /*miter_limit=*/-1.0);
    auto path_geometry = static_cast<StrokePathGeometry*>(geometry.get());
    ASSERT_FLOAT_EQ(path_geometry->GetMiterLimit(), 4);
  }
}

TEST_P(EntityTest, BlendingModeOptions) {
  std::vector<const char*> blend_mode_names;
  std::vector<BlendMode> blend_mode_values;
  {
    // Force an exhausiveness check with a switch. When adding blend modes,
    // update this switch with a new name/value to make it selectable in the
    // test GUI.

    const BlendMode b{};
    static_assert(b == BlendMode::kClear);  // Ensure the first item in
                                            // the switch is the first
                                            // item in the enum.
    static_assert(Entity::kLastPipelineBlendMode == BlendMode::kModulate);
    switch (b) {
      case BlendMode::kClear:
        blend_mode_names.push_back("Clear");
        blend_mode_values.push_back(BlendMode::kClear);
      case BlendMode::kSource:
        blend_mode_names.push_back("Source");
        blend_mode_values.push_back(BlendMode::kSource);
      case BlendMode::kDestination:
        blend_mode_names.push_back("Destination");
        blend_mode_values.push_back(BlendMode::kDestination);
      case BlendMode::kSourceOver:
        blend_mode_names.push_back("SourceOver");
        blend_mode_values.push_back(BlendMode::kSourceOver);
      case BlendMode::kDestinationOver:
        blend_mode_names.push_back("DestinationOver");
        blend_mode_values.push_back(BlendMode::kDestinationOver);
      case BlendMode::kSourceIn:
        blend_mode_names.push_back("SourceIn");
        blend_mode_values.push_back(BlendMode::kSourceIn);
      case BlendMode::kDestinationIn:
        blend_mode_names.push_back("DestinationIn");
        blend_mode_values.push_back(BlendMode::kDestinationIn);
      case BlendMode::kSourceOut:
        blend_mode_names.push_back("SourceOut");
        blend_mode_values.push_back(BlendMode::kSourceOut);
      case BlendMode::kDestinationOut:
        blend_mode_names.push_back("DestinationOut");
        blend_mode_values.push_back(BlendMode::kDestinationOut);
      case BlendMode::kSourceATop:
        blend_mode_names.push_back("SourceATop");
        blend_mode_values.push_back(BlendMode::kSourceATop);
      case BlendMode::kDestinationATop:
        blend_mode_names.push_back("DestinationATop");
        blend_mode_values.push_back(BlendMode::kDestinationATop);
      case BlendMode::kXor:
        blend_mode_names.push_back("Xor");
        blend_mode_values.push_back(BlendMode::kXor);
      case BlendMode::kPlus:
        blend_mode_names.push_back("Plus");
        blend_mode_values.push_back(BlendMode::kPlus);
      case BlendMode::kModulate:
        blend_mode_names.push_back("Modulate");
        blend_mode_values.push_back(BlendMode::kModulate);
    };
  }

  auto callback = [&](ContentContext& context, RenderPass& pass) {
    auto world_matrix = Matrix::MakeScale(GetContentScale());
    auto draw_rect = [&context, &pass, &world_matrix](
                         Rect rect, Color color, BlendMode blend_mode) -> bool {
      using VS = SolidFillPipeline::VertexShader;
      using FS = SolidFillPipeline::FragmentShader;

      VertexBufferBuilder<VS::PerVertexData> vtx_builder;
      {
        auto r = rect.GetLTRB();
        vtx_builder.AddVertices({
            {Point(r[0], r[1])},
            {Point(r[2], r[1])},
            {Point(r[2], r[3])},
            {Point(r[0], r[1])},
            {Point(r[2], r[3])},
            {Point(r[0], r[3])},
        });
      }

      pass.SetCommandLabel("Blended Rectangle");
      auto options = OptionsFromPass(pass);
      options.blend_mode = blend_mode;
      options.primitive_type = PrimitiveType::kTriangle;
      pass.SetPipeline(context.GetSolidFillPipeline(options));
      pass.SetVertexBuffer(
          vtx_builder.CreateVertexBuffer(context.GetTransientsBuffer()));

      VS::FrameInfo frame_info;
      frame_info.mvp = pass.GetOrthographicTransform() * world_matrix;
      VS::BindFrameInfo(
          pass, context.GetTransientsBuffer().EmplaceUniform(frame_info));
      FS::FragInfo frag_info;
      frag_info.color = color.Premultiply();
      FS::BindFragInfo(
          pass, context.GetTransientsBuffer().EmplaceUniform(frame_info));
      return pass.Draw().ok();
    };

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    static Color color1(1, 0, 0, 0.5), color2(0, 1, 0, 0.5);
    ImGui::ColorEdit4("Color 1", reinterpret_cast<float*>(&color1));
    ImGui::ColorEdit4("Color 2", reinterpret_cast<float*>(&color2));
    static int current_blend_index = 3;
    ImGui::ListBox("Blending mode", &current_blend_index,
                   blend_mode_names.data(), blend_mode_names.size());
    ImGui::End();

    BlendMode selected_mode = blend_mode_values[current_blend_index];

    Point a, b, c, d;
    PlaygroundPoint point_a(Point(400, 100), 20, Color::White());
    PlaygroundPoint point_b(Point(200, 300), 20, Color::White());
    std::tie(a, b) = DrawPlaygroundLine(point_a, point_b);
    PlaygroundPoint point_c(Point(470, 190), 20, Color::White());
    PlaygroundPoint point_d(Point(270, 390), 20, Color::White());
    std::tie(c, d) = DrawPlaygroundLine(point_c, point_d);

    bool result = true;
    result = result &&
             draw_rect(Rect::MakeXYWH(0, 0, pass.GetRenderTargetSize().width,
                                      pass.GetRenderTargetSize().height),
                       Color(), BlendMode::kClear);
    result = result && draw_rect(Rect::MakeLTRB(a.x, a.y, b.x, b.y), color1,
                                 BlendMode::kSourceOver);
    result = result && draw_rect(Rect::MakeLTRB(c.x, c.y, d.x, d.y), color2,
                                 selected_mode);
    return result;
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, BezierCircleScaled) {
  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    static float scale = 20;

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat("Scale", &scale, 1, 100);
    ImGui::End();

    Entity entity;
    entity.SetTransform(Matrix::MakeScale(GetContentScale()));
    auto path = PathBuilder{}
                    .MoveTo({97.325, 34.818})
                    .CubicCurveTo({98.50862885295136, 34.81812293973836},
                                  {99.46822048142015, 33.85863261475589},
                                  {99.46822048142015, 32.67499810206613})
                    .CubicCurveTo({99.46822048142015, 31.491363589376355},
                                  {98.50862885295136, 30.53187326439389},
                                  {97.32499434685802, 30.531998226542708})
                    .CubicCurveTo({96.14153655073771, 30.532123170035373},
                                  {95.18222070648729, 31.491540299350355},
                                  {95.18222070648729, 32.67499810206613})
                    .CubicCurveTo({95.18222070648729, 33.85845590478189},
                                  {96.14153655073771, 34.81787303409686},
                                  {97.32499434685802, 34.81799797758954})
                    .Close()
                    .TakePath();
    entity.SetTransform(
        Matrix::MakeScale({scale, scale, 1.0}).Translate({-90, -20, 0}));

    std::unique_ptr<Geometry> geom = Geometry::MakeFillPath(path);

    auto contents = std::make_shared<SolidColorContents>();
    contents->SetColor(Color::Red());
    contents->SetGeometry(geom.get());

    entity.SetContents(contents);
    return entity.Render(context, pass);
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, Filters) {
  auto bridge = CreateTextureForFixture("bay_bridge.jpg");
  auto boston = CreateTextureForFixture("boston.jpg");
  auto kalimba = CreateTextureForFixture("kalimba.jpg");
  ASSERT_TRUE(bridge && boston && kalimba);

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    auto fi_bridge = FilterInput::Make(bridge);
    auto fi_boston = FilterInput::Make(boston);
    auto fi_kalimba = FilterInput::Make(kalimba);

    std::shared_ptr<FilterContents> blend0 = ColorFilterContents::MakeBlend(
        BlendMode::kModulate, {fi_kalimba, fi_boston});

    auto blend1 = ColorFilterContents::MakeBlend(
        BlendMode::kScreen,
        {FilterInput::Make(blend0), fi_bridge, fi_bridge, fi_bridge});

    Entity entity;
    entity.SetTransform(Matrix::MakeScale(GetContentScale()) *
                        Matrix::MakeTranslation({500, 300}) *
                        Matrix::MakeScale(Vector2{0.5, 0.5}));
    entity.SetContents(blend1);
    return entity.Render(context, pass);
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, GaussianBlurFilter) {
  auto boston =
      CreateTextureForFixture("boston.jpg", /*enable_mipmapping=*/true);
  ASSERT_TRUE(boston);

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    const char* input_type_names[] = {"Texture", "Solid Color"};
    const char* blur_type_names[] = {"Image blur", "Mask blur"};
    const char* pass_variation_names[] = {"New"};
    const char* blur_style_names[] = {"Normal", "Solid", "Outer", "Inner"};
    const char* tile_mode_names[] = {"Clamp", "Repeat", "Mirror", "Decal"};
    const FilterContents::BlurStyle blur_styles[] = {
        FilterContents::BlurStyle::kNormal, FilterContents::BlurStyle::kSolid,
        FilterContents::BlurStyle::kOuter, FilterContents::BlurStyle::kInner};
    const Entity::TileMode tile_modes[] = {
        Entity::TileMode::kClamp, Entity::TileMode::kRepeat,
        Entity::TileMode::kMirror, Entity::TileMode::kDecal};

    // UI state.
    static int selected_input_type = 0;
    static Color input_color = Color::Black();
    static int selected_blur_type = 0;
    static int selected_pass_variation = 0;
    static bool combined_sigma = false;
    static float blur_amount_coarse[2] = {0, 0};
    static float blur_amount_fine[2] = {10, 10};
    static int selected_blur_style = 0;
    static int selected_tile_mode = 3;
    static Color cover_color(1, 0, 0, 0.2);
    static Color bounds_color(0, 1, 0, 0.1);
    static float offset[2] = {500, 400};
    static float rotation = 0;
    static float scale[2] = {0.65, 0.65};
    static float skew[2] = {0, 0};
    static float path_rect[4] = {0, 0,
                                 static_cast<float>(boston->GetSize().width),
                                 static_cast<float>(boston->GetSize().height)};

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
      ImGui::Combo("Input type", &selected_input_type, input_type_names,
                   sizeof(input_type_names) / sizeof(char*));
      if (selected_input_type == 0) {
        ImGui::SliderFloat("Input opacity", &input_color.alpha, 0, 1);
      } else {
        ImGui::ColorEdit4("Input color",
                          reinterpret_cast<float*>(&input_color));
      }
      ImGui::Combo("Blur type", &selected_blur_type, blur_type_names,
                   sizeof(blur_type_names) / sizeof(char*));
      if (selected_blur_type == 0) {
        ImGui::Combo("Pass variation", &selected_pass_variation,
                     pass_variation_names,
                     sizeof(pass_variation_names) / sizeof(char*));
      }
      ImGui::Checkbox("Combined sigma", &combined_sigma);
      if (combined_sigma) {
        ImGui::SliderFloat("Sigma (coarse)", blur_amount_coarse, 0, 1000);
        ImGui::SliderFloat("Sigma (fine)", blur_amount_fine, 0, 10);
        blur_amount_coarse[1] = blur_amount_coarse[0];
        blur_amount_fine[1] = blur_amount_fine[0];
      } else {
        ImGui::SliderFloat2("Sigma (coarse)", blur_amount_coarse, 0, 1000);
        ImGui::SliderFloat2("Sigma (fine)", blur_amount_fine, 0, 10);
      }
      ImGui::Combo("Blur style", &selected_blur_style, blur_style_names,
                   sizeof(blur_style_names) / sizeof(char*));
      ImGui::Combo("Tile mode", &selected_tile_mode, tile_mode_names,
                   sizeof(tile_mode_names) / sizeof(char*));
      ImGui::ColorEdit4("Cover color", reinterpret_cast<float*>(&cover_color));
      ImGui::ColorEdit4("Bounds color ",
                        reinterpret_cast<float*>(&bounds_color));
      ImGui::SliderFloat2("Translation", offset, 0,
                          pass.GetRenderTargetSize().width);
      ImGui::SliderFloat("Rotation", &rotation, 0, kPi * 2);
      ImGui::SliderFloat2("Scale", scale, 0, 3);
      ImGui::SliderFloat2("Skew", skew, -3, 3);
      ImGui::SliderFloat4("Path XYWH", path_rect, -1000, 1000);
    }
    ImGui::End();

    auto blur_sigma_x = Sigma{blur_amount_coarse[0] + blur_amount_fine[0]};
    auto blur_sigma_y = Sigma{blur_amount_coarse[1] + blur_amount_fine[1]};

    std::shared_ptr<Contents> input;
    Size input_size;

    auto input_rect =
        Rect::MakeXYWH(path_rect[0], path_rect[1], path_rect[2], path_rect[3]);

    std::unique_ptr<Geometry> solid_color_input;
    if (selected_input_type == 0) {
      auto texture = std::make_shared<TextureContents>();
      texture->SetSourceRect(Rect::MakeSize(boston->GetSize()));
      texture->SetDestinationRect(input_rect);
      texture->SetTexture(boston);
      texture->SetOpacity(input_color.alpha);

      input = texture;
      input_size = input_rect.GetSize();
    } else {
      auto fill = std::make_shared<SolidColorContents>();
      fill->SetColor(input_color);
      solid_color_input =
          Geometry::MakeFillPath(PathBuilder{}.AddRect(input_rect).TakePath());

      fill->SetGeometry(solid_color_input.get());

      input = fill;
      input_size = input_rect.GetSize();
    }

    std::shared_ptr<FilterContents> blur;
    switch (selected_pass_variation) {
      case 0:
        blur = std::make_shared<GaussianBlurFilterContents>(
            blur_sigma_x.sigma, blur_sigma_y.sigma,
            tile_modes[selected_tile_mode], blur_styles[selected_blur_style],
            /*geometry=*/nullptr);
        blur->SetInputs({FilterInput::Make(input)});
        break;
      case 1:
        blur = FilterContents::MakeGaussianBlur(
            FilterInput::Make(input), blur_sigma_x, blur_sigma_y,
            tile_modes[selected_tile_mode], blur_styles[selected_blur_style]);
        break;
    };
    FML_CHECK(blur);

    auto mask_blur = FilterContents::MakeBorderMaskBlur(
        FilterInput::Make(input), blur_sigma_x, blur_sigma_y,
        blur_styles[selected_blur_style]);

    auto ctm = Matrix::MakeScale(GetContentScale()) *
               Matrix::MakeTranslation(Vector3(offset[0], offset[1])) *
               Matrix::MakeRotationZ(Radians(rotation)) *
               Matrix::MakeScale(Vector2(scale[0], scale[1])) *
               Matrix::MakeSkew(skew[0], skew[1]) *
               Matrix::MakeTranslation(-Point(input_size) / 2);

    auto target_contents = selected_blur_type == 0 ? blur : mask_blur;

    Entity entity;
    entity.SetContents(target_contents);
    entity.SetTransform(ctm);

    entity.Render(context, pass);

    // Renders a red "cover" rectangle that shows the original position of the
    // unfiltered input.
    Entity cover_entity;
    std::unique_ptr<Geometry> geom =
        Geometry::MakeFillPath(PathBuilder{}.AddRect(input_rect).TakePath());
    auto contents = std::make_shared<SolidColorContents>();
    contents->SetColor(cover_color);
    contents->SetGeometry(geom.get());
    cover_entity.SetContents(std::move(contents));
    cover_entity.SetTransform(ctm);
    cover_entity.Render(context, pass);

    // Renders a green bounding rect of the target filter.
    Entity bounds_entity;
    std::optional<Rect> target_contents_coverage =
        target_contents->GetCoverage(entity);
    if (target_contents_coverage.has_value()) {
      std::unique_ptr<Geometry> geom = Geometry::MakeFillPath(
          PathBuilder{}
              .AddRect(target_contents->GetCoverage(entity).value())
              .TakePath());
      auto contents = std::make_shared<SolidColorContents>();
      contents->SetColor(bounds_color);
      contents->SetGeometry(geom.get());

      bounds_entity.SetContents(contents);
      bounds_entity.SetTransform(Matrix());
      bounds_entity.Render(context, pass);
    }

    return true;
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, MorphologyFilter) {
  auto boston = CreateTextureForFixture("boston.jpg");
  ASSERT_TRUE(boston);

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    const char* morphology_type_names[] = {"Dilate", "Erode"};
    const FilterContents::MorphType morphology_types[] = {
        FilterContents::MorphType::kDilate, FilterContents::MorphType::kErode};
    static Color input_color = Color::Black();
    // UI state.
    static int selected_morphology_type = 0;
    static float radius[2] = {20, 20};
    static Color cover_color(1, 0, 0, 0.2);
    static Color bounds_color(0, 1, 0, 0.1);
    static float offset[2] = {500, 400};
    static float rotation = 0;
    static float scale[2] = {0.65, 0.65};
    static float skew[2] = {0, 0};
    static float path_rect[4] = {0, 0,
                                 static_cast<float>(boston->GetSize().width),
                                 static_cast<float>(boston->GetSize().height)};
    static float effect_transform_scale = 1;

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
      ImGui::Combo("Morphology type", &selected_morphology_type,
                   morphology_type_names,
                   sizeof(morphology_type_names) / sizeof(char*));
      ImGui::SliderFloat2("Radius", radius, 0, 200);
      ImGui::SliderFloat("Input opacity", &input_color.alpha, 0, 1);
      ImGui::ColorEdit4("Cover color", reinterpret_cast<float*>(&cover_color));
      ImGui::ColorEdit4("Bounds color ",
                        reinterpret_cast<float*>(&bounds_color));
      ImGui::SliderFloat2("Translation", offset, 0,
                          pass.GetRenderTargetSize().width);
      ImGui::SliderFloat("Rotation", &rotation, 0, kPi * 2);
      ImGui::SliderFloat2("Scale", scale, 0, 3);
      ImGui::SliderFloat2("Skew", skew, -3, 3);
      ImGui::SliderFloat4("Path XYWH", path_rect, -1000, 1000);
      ImGui::SliderFloat("Effect transform scale", &effect_transform_scale, 0,
                         3);
    }
    ImGui::End();

    std::shared_ptr<Contents> input;
    Size input_size;

    auto input_rect =
        Rect::MakeXYWH(path_rect[0], path_rect[1], path_rect[2], path_rect[3]);
    auto texture = std::make_shared<TextureContents>();
    texture->SetSourceRect(Rect::MakeSize(boston->GetSize()));
    texture->SetDestinationRect(input_rect);
    texture->SetTexture(boston);
    texture->SetOpacity(input_color.alpha);

    input = texture;
    input_size = input_rect.GetSize();

    auto contents = FilterContents::MakeMorphology(
        FilterInput::Make(input), Radius{radius[0]}, Radius{radius[1]},
        morphology_types[selected_morphology_type]);
    contents->SetEffectTransform(Matrix::MakeScale(
        Vector2{effect_transform_scale, effect_transform_scale}));

    auto ctm = Matrix::MakeScale(GetContentScale()) *
               Matrix::MakeTranslation(Vector3(offset[0], offset[1])) *
               Matrix::MakeRotationZ(Radians(rotation)) *
               Matrix::MakeScale(Vector2(scale[0], scale[1])) *
               Matrix::MakeSkew(skew[0], skew[1]) *
               Matrix::MakeTranslation(-Point(input_size) / 2);

    Entity entity;
    entity.SetContents(contents);
    entity.SetTransform(ctm);

    entity.Render(context, pass);

    // Renders a red "cover" rectangle that shows the original position of  the
    // unfiltered input.
    Entity cover_entity;
    std::unique_ptr<Geometry> geom =
        Geometry::MakeFillPath(PathBuilder{}.AddRect(input_rect).TakePath());
    auto cover_contents = std::make_shared<SolidColorContents>();
    cover_contents->SetColor(cover_color);
    cover_contents->SetGeometry(geom.get());
    cover_entity.SetContents(cover_contents);
    cover_entity.SetTransform(ctm);
    cover_entity.Render(context, pass);

    // Renders a green bounding rect of the target filter.
    Entity bounds_entity;
    std::unique_ptr<Geometry> bounds_geom = Geometry::MakeFillPath(
        PathBuilder{}
            .AddRect(contents->GetCoverage(entity).value())
            .TakePath());
    auto bounds_contents = std::make_shared<SolidColorContents>();
    bounds_contents->SetColor(bounds_color);
    bounds_contents->SetGeometry(bounds_geom.get());
    bounds_entity.SetContents(std::move(bounds_contents));
    bounds_entity.SetTransform(Matrix());

    bounds_entity.Render(context, pass);

    return true;
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, SetBlendMode) {
  Entity entity;
  ASSERT_EQ(entity.GetBlendMode(), BlendMode::kSourceOver);
  entity.SetBlendMode(BlendMode::kClear);
  ASSERT_EQ(entity.GetBlendMode(), BlendMode::kClear);
}

TEST_P(EntityTest, ContentsGetBoundsForEmptyPathReturnsNullopt) {
  Entity entity;
  entity.SetContents(std::make_shared<SolidColorContents>());
  ASSERT_FALSE(entity.GetCoverage().has_value());
}

TEST_P(EntityTest, SolidStrokeCoverageIsCorrect) {
  {
    auto geometry = Geometry::MakeStrokePath(
        PathBuilder{}.AddLine({0, 0}, {10, 10}).TakePath(), 4.0, 4.0,
        Cap::kButt, Join::kBevel);

    Entity entity;
    auto contents = std::make_unique<SolidColorContents>();
    contents->SetGeometry(geometry.get());
    contents->SetColor(Color::Black());
    entity.SetContents(std::move(contents));
    auto actual = entity.GetCoverage();
    auto expected = Rect::MakeLTRB(-2, -2, 12, 12);

    ASSERT_TRUE(actual.has_value());
    ASSERT_RECT_NEAR(actual.value(), expected);
  }

  // Cover the Cap::kSquare case.
  {
    auto geometry = Geometry::MakeStrokePath(
        PathBuilder{}.AddLine({0, 0}, {10, 10}).TakePath(), 4.0, 4.0,
        Cap::kSquare, Join::kBevel);

    Entity entity;
    auto contents = std::make_unique<SolidColorContents>();
    contents->SetGeometry(geometry.get());
    contents->SetColor(Color::Black());
    entity.SetContents(std::move(contents));
    auto actual = entity.GetCoverage();
    auto expected =
        Rect::MakeLTRB(-sqrt(8), -sqrt(8), 10 + sqrt(8), 10 + sqrt(8));

    ASSERT_TRUE(actual.has_value());
    ASSERT_RECT_NEAR(actual.value(), expected);
  }

  // Cover the Join::kMiter case.
  {
    auto geometry = Geometry::MakeStrokePath(
        PathBuilder{}.AddLine({0, 0}, {10, 10}).TakePath(), 4.0, 2.0,
        Cap::kSquare, Join::kMiter);

    Entity entity;
    auto contents = std::make_unique<SolidColorContents>();
    contents->SetGeometry(geometry.get());
    contents->SetColor(Color::Black());
    entity.SetContents(std::move(contents));
    auto actual = entity.GetCoverage();
    auto expected = Rect::MakeLTRB(-4, -4, 14, 14);

    ASSERT_TRUE(actual.has_value());
    ASSERT_RECT_NEAR(actual.value(), expected);
  }
}

TEST_P(EntityTest, BorderMaskBlurCoverageIsCorrect) {
  auto fill = std::make_shared<SolidColorContents>();
  auto geom = Geometry::MakeFillPath(
      PathBuilder{}.AddRect(Rect::MakeXYWH(0, 0, 300, 400)).TakePath());
  fill->SetGeometry(geom.get());
  fill->SetColor(Color::CornflowerBlue());
  auto border_mask_blur = FilterContents::MakeBorderMaskBlur(
      FilterInput::Make(fill), Radius{3}, Radius{4});

  {
    Entity e;
    e.SetTransform(Matrix());
    auto actual = border_mask_blur->GetCoverage(e);
    auto expected = Rect::MakeXYWH(-3, -4, 306, 408);
    ASSERT_TRUE(actual.has_value());
    ASSERT_RECT_NEAR(actual.value(), expected);
  }

  {
    Entity e;
    e.SetTransform(Matrix::MakeRotationZ(Radians{kPi / 4}));
    auto actual = border_mask_blur->GetCoverage(e);
    auto expected = Rect::MakeXYWH(-287.792, -4.94975, 504.874, 504.874);
    ASSERT_TRUE(actual.has_value());
    ASSERT_RECT_NEAR(actual.value(), expected);
  }
}

TEST_P(EntityTest, SolidFillCoverageIsCorrect) {
  // No transform
  {
    auto fill = std::make_shared<SolidColorContents>();
    fill->SetColor(Color::CornflowerBlue());
    auto expected = Rect::MakeLTRB(100, 110, 200, 220);
    auto geom =
        Geometry::MakeFillPath(PathBuilder{}.AddRect(expected).TakePath());
    fill->SetGeometry(geom.get());

    auto coverage = fill->GetCoverage({});
    ASSERT_TRUE(coverage.has_value());
    ASSERT_RECT_NEAR(coverage.value(), expected);
  }

  // Entity transform
  {
    auto fill = std::make_shared<SolidColorContents>();
    auto geom = Geometry::MakeFillPath(
        PathBuilder{}.AddRect(Rect::MakeLTRB(100, 110, 200, 220)).TakePath());
    fill->SetColor(Color::CornflowerBlue());
    fill->SetGeometry(geom.get());

    Entity entity;
    entity.SetTransform(Matrix::MakeTranslation(Vector2(4, 5)));
    entity.SetContents(std::move(fill));

    auto coverage = entity.GetCoverage();
    auto expected = Rect::MakeLTRB(104, 115, 204, 225);
    ASSERT_TRUE(coverage.has_value());
    ASSERT_RECT_NEAR(coverage.value(), expected);
  }

  // No coverage for fully transparent colors
  {
    auto fill = std::make_shared<SolidColorContents>();
    auto geom = Geometry::MakeFillPath(
        PathBuilder{}.AddRect(Rect::MakeLTRB(100, 110, 200, 220)).TakePath());
    fill->SetColor(Color::WhiteTransparent());
    fill->SetGeometry(geom.get());

    auto coverage = fill->GetCoverage({});
    ASSERT_FALSE(coverage.has_value());
  }
}

TEST_P(EntityTest, RRectShadowTest) {
  auto callback = [&](ContentContext& context, RenderPass& pass) {
    static Color color = Color::Red();
    static float corner_radius = 100;
    static float blur_radius = 100;
    static bool show_coverage = false;
    static Color coverage_color = Color::Green().WithAlpha(0.2);

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat("Corner radius", &corner_radius, 0, 300);
    ImGui::SliderFloat("Blur radius", &blur_radius, 0, 300);
    ImGui::ColorEdit4("Color", reinterpret_cast<Scalar*>(&color));
    ImGui::Checkbox("Show coverage", &show_coverage);
    if (show_coverage) {
      ImGui::ColorEdit4("Coverage color",
                        reinterpret_cast<Scalar*>(&coverage_color));
    }
    ImGui::End();

    PlaygroundPoint top_left_point(Point(200, 200), 30, Color::White());
    PlaygroundPoint bottom_right_point(Point(600, 400), 30, Color::White());
    auto [top_left, bottom_right] =
        DrawPlaygroundLine(top_left_point, bottom_right_point);
    auto rect =
        Rect::MakeLTRB(top_left.x, top_left.y, bottom_right.x, bottom_right.y);

    auto contents = std::make_unique<SolidRRectBlurContents>();
    contents->SetRRect(rect, {corner_radius, corner_radius});
    contents->SetColor(color);
    contents->SetSigma(Radius(blur_radius));

    Entity entity;
    entity.SetTransform(Matrix::MakeScale(GetContentScale()));
    entity.SetContents(std::move(contents));
    entity.Render(context, pass);

    auto coverage = entity.GetCoverage();
    if (show_coverage && coverage.has_value()) {
      auto bounds_contents = std::make_unique<SolidColorContents>();
      auto geom = Geometry::MakeFillPath(
          PathBuilder{}.AddRect(entity.GetCoverage().value()).TakePath());
      bounds_contents->SetGeometry(geom.get());
      bounds_contents->SetColor(coverage_color.Premultiply());
      Entity bounds_entity;
      bounds_entity.SetContents(std::move(bounds_contents));
      bounds_entity.Render(context, pass);
    }

    return true;
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, ColorMatrixFilterCoverageIsCorrect) {
  // Set up a simple color background.
  auto fill = std::make_shared<SolidColorContents>();
  auto geom = Geometry::MakeFillPath(
      PathBuilder{}.AddRect(Rect::MakeXYWH(0, 0, 300, 400)).TakePath());
  fill->SetGeometry(geom.get());
  fill->SetColor(Color::Coral());

  // Set the color matrix filter.
  ColorMatrix matrix = {
      1, 1, 1, 1, 1,  //
      1, 1, 1, 1, 1,  //
      1, 1, 1, 1, 1,  //
      1, 1, 1, 1, 1,  //
  };

  auto filter =
      ColorFilterContents::MakeColorMatrix(FilterInput::Make(fill), matrix);

  Entity e;
  e.SetTransform(Matrix());

  // Confirm that the actual filter coverage matches the expected coverage.
  auto actual = filter->GetCoverage(e);
  auto expected = Rect::MakeXYWH(0, 0, 300, 400);

  ASSERT_TRUE(actual.has_value());
  ASSERT_RECT_NEAR(actual.value(), expected);
}

TEST_P(EntityTest, ColorMatrixFilterEditable) {
  auto bay_bridge = CreateTextureForFixture("bay_bridge.jpg");
  ASSERT_TRUE(bay_bridge);

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    // UI state.
    static ColorMatrix color_matrix = {
        1, 0, 0, 0, 0,  //
        0, 3, 0, 0, 0,  //
        0, 0, 1, 0, 0,  //
        0, 0, 0, 1, 0,  //
    };
    static float offset[2] = {500, 400};
    static float rotation = 0;
    static float scale[2] = {0.65, 0.65};
    static float skew[2] = {0, 0};

    // Define the ImGui
    ImGui::Begin("Color Matrix", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
      std::string label = "##1";
      for (int i = 0; i < 20; i += 5) {
        ImGui::InputScalarN(label.c_str(), ImGuiDataType_Float,
                            &(color_matrix.array[i]), 5, nullptr, nullptr,
                            "%.2f", 0);
        label[2]++;
      }

      ImGui::SliderFloat2("Translation", &offset[0], 0,
                          pass.GetRenderTargetSize().width);
      ImGui::SliderFloat("Rotation", &rotation, 0, kPi * 2);
      ImGui::SliderFloat2("Scale", &scale[0], 0, 3);
      ImGui::SliderFloat2("Skew", &skew[0], -3, 3);
    }
    ImGui::End();

    // Set the color matrix filter.
    auto filter = ColorFilterContents::MakeColorMatrix(
        FilterInput::Make(bay_bridge), color_matrix);

    // Define the entity with the color matrix filter.
    Entity entity;
    entity.SetTransform(
        Matrix::MakeScale(GetContentScale()) *
        Matrix::MakeTranslation(Vector3(offset[0], offset[1])) *
        Matrix::MakeRotationZ(Radians(rotation)) *
        Matrix::MakeScale(Vector2(scale[0], scale[1])) *
        Matrix::MakeSkew(skew[0], skew[1]) *
        Matrix::MakeTranslation(-Point(bay_bridge->GetSize()) / 2));
    entity.SetContents(filter);
    entity.Render(context, pass);

    return true;
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, LinearToSrgbFilterCoverageIsCorrect) {
  // Set up a simple color background.
  auto geom = Geometry::MakeFillPath(
      PathBuilder{}.AddRect(Rect::MakeXYWH(0, 0, 300, 400)).TakePath());
  auto fill = std::make_shared<SolidColorContents>();
  fill->SetGeometry(geom.get());
  fill->SetColor(Color::MintCream());

  auto filter =
      ColorFilterContents::MakeLinearToSrgbFilter(FilterInput::Make(fill));

  Entity e;
  e.SetTransform(Matrix());

  // Confirm that the actual filter coverage matches the expected coverage.
  auto actual = filter->GetCoverage(e);
  auto expected = Rect::MakeXYWH(0, 0, 300, 400);

  ASSERT_TRUE(actual.has_value());
  ASSERT_RECT_NEAR(actual.value(), expected);
}

TEST_P(EntityTest, LinearToSrgbFilter) {
  auto image = CreateTextureForFixture("kalimba.jpg");
  ASSERT_TRUE(image);

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    auto filtered =
        ColorFilterContents::MakeLinearToSrgbFilter(FilterInput::Make(image));

    // Define the entity that will serve as the control image as a Gaussian blur
    // filter with no filter at all.
    Entity entity_left;
    entity_left.SetTransform(Matrix::MakeScale(GetContentScale()) *
                             Matrix::MakeTranslation({100, 300}) *
                             Matrix::MakeScale(Vector2{0.5, 0.5}));
    auto unfiltered = FilterContents::MakeGaussianBlur(FilterInput::Make(image),
                                                       Sigma{0}, Sigma{0});
    entity_left.SetContents(unfiltered);

    // Define the entity that will be filtered from linear to sRGB.
    Entity entity_right;
    entity_right.SetTransform(Matrix::MakeScale(GetContentScale()) *
                              Matrix::MakeTranslation({500, 300}) *
                              Matrix::MakeScale(Vector2{0.5, 0.5}));
    entity_right.SetContents(filtered);
    return entity_left.Render(context, pass) &&
           entity_right.Render(context, pass);
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, SrgbToLinearFilterCoverageIsCorrect) {
  // Set up a simple color background.
  auto fill = std::make_shared<SolidColorContents>();
  auto geom = Geometry::MakeFillPath(
      PathBuilder{}.AddRect(Rect::MakeXYWH(0, 0, 300, 400)).TakePath());
  fill->SetGeometry(geom.get());
  fill->SetColor(Color::DeepPink());

  auto filter =
      ColorFilterContents::MakeSrgbToLinearFilter(FilterInput::Make(fill));

  Entity e;
  e.SetTransform(Matrix());

  // Confirm that the actual filter coverage matches the expected coverage.
  auto actual = filter->GetCoverage(e);
  auto expected = Rect::MakeXYWH(0, 0, 300, 400);

  ASSERT_TRUE(actual.has_value());
  ASSERT_RECT_NEAR(actual.value(), expected);
}

TEST_P(EntityTest, SrgbToLinearFilter) {
  auto image = CreateTextureForFixture("embarcadero.jpg");
  ASSERT_TRUE(image);

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    auto filtered =
        ColorFilterContents::MakeSrgbToLinearFilter(FilterInput::Make(image));

    // Define the entity that will serve as the control image as a Gaussian blur
    // filter with no filter at all.
    Entity entity_left;
    entity_left.SetTransform(Matrix::MakeScale(GetContentScale()) *
                             Matrix::MakeTranslation({100, 300}) *
                             Matrix::MakeScale(Vector2{0.5, 0.5}));
    auto unfiltered = FilterContents::MakeGaussianBlur(FilterInput::Make(image),
                                                       Sigma{0}, Sigma{0});
    entity_left.SetContents(unfiltered);

    // Define the entity that will be filtered from sRGB to linear.
    Entity entity_right;
    entity_right.SetTransform(Matrix::MakeScale(GetContentScale()) *
                              Matrix::MakeTranslation({500, 300}) *
                              Matrix::MakeScale(Vector2{0.5, 0.5}));
    entity_right.SetContents(filtered);
    return entity_left.Render(context, pass) &&
           entity_right.Render(context, pass);
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

static Vector3 RGBToYUV(Vector3 rgb, YUVColorSpace yuv_color_space) {
  Vector3 yuv;
  switch (yuv_color_space) {
    case YUVColorSpace::kBT601FullRange:
      yuv.x = rgb.x * 0.299 + rgb.y * 0.587 + rgb.z * 0.114;
      yuv.y = rgb.x * -0.169 + rgb.y * -0.331 + rgb.z * 0.5 + 0.5;
      yuv.z = rgb.x * 0.5 + rgb.y * -0.419 + rgb.z * -0.081 + 0.5;
      break;
    case YUVColorSpace::kBT601LimitedRange:
      yuv.x = rgb.x * 0.257 + rgb.y * 0.516 + rgb.z * 0.100 + 0.063;
      yuv.y = rgb.x * -0.145 + rgb.y * -0.291 + rgb.z * 0.439 + 0.5;
      yuv.z = rgb.x * 0.429 + rgb.y * -0.368 + rgb.z * -0.071 + 0.5;
      break;
  }
  return yuv;
}

static std::vector<std::shared_ptr<Texture>> CreateTestYUVTextures(
    Context* context,
    YUVColorSpace yuv_color_space) {
  Vector3 red = {244.0 / 255.0, 67.0 / 255.0, 54.0 / 255.0};
  Vector3 green = {76.0 / 255.0, 175.0 / 255.0, 80.0 / 255.0};
  Vector3 blue = {33.0 / 255.0, 150.0 / 255.0, 243.0 / 255.0};
  Vector3 white = {1.0, 1.0, 1.0};
  Vector3 red_yuv = RGBToYUV(red, yuv_color_space);
  Vector3 green_yuv = RGBToYUV(green, yuv_color_space);
  Vector3 blue_yuv = RGBToYUV(blue, yuv_color_space);
  Vector3 white_yuv = RGBToYUV(white, yuv_color_space);
  std::vector<Vector3> yuvs{red_yuv, green_yuv, blue_yuv, white_yuv};
  std::vector<uint8_t> y_data;
  std::vector<uint8_t> uv_data;
  for (int i = 0; i < 4; i++) {
    auto yuv = yuvs[i];
    uint8_t y = std::round(yuv.x * 255.0);
    uint8_t u = std::round(yuv.y * 255.0);
    uint8_t v = std::round(yuv.z * 255.0);
    for (int j = 0; j < 16; j++) {
      y_data.push_back(y);
    }
    for (int j = 0; j < 8; j++) {
      uv_data.push_back(j % 2 == 0 ? u : v);
    }
  }
  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();

  impeller::TextureDescriptor y_texture_descriptor;
  y_texture_descriptor.storage_mode = impeller::StorageMode::kHostVisible;
  y_texture_descriptor.format = PixelFormat::kR8UNormInt;
  y_texture_descriptor.size = {8, 8};
  auto y_texture =
      context->GetResourceAllocator()->CreateTexture(y_texture_descriptor);
  auto y_mapping = std::make_shared<fml::DataMapping>(y_data);
  auto y_mapping_buffer =
      context->GetResourceAllocator()->CreateBufferWithCopy(*y_mapping);

  blit_pass->AddCopy(DeviceBuffer::AsBufferView(y_mapping_buffer), y_texture);

  impeller::TextureDescriptor uv_texture_descriptor;
  uv_texture_descriptor.storage_mode = impeller::StorageMode::kHostVisible;
  uv_texture_descriptor.format = PixelFormat::kR8G8UNormInt;
  uv_texture_descriptor.size = {4, 4};
  auto uv_texture =
      context->GetResourceAllocator()->CreateTexture(uv_texture_descriptor);
  auto uv_mapping = std::make_shared<fml::DataMapping>(uv_data);
  auto uv_mapping_buffer =
      context->GetResourceAllocator()->CreateBufferWithCopy(*uv_mapping);

  blit_pass->AddCopy(DeviceBuffer::AsBufferView(uv_mapping_buffer), uv_texture);

  if (!blit_pass->EncodeCommands() ||
      !context->GetCommandQueue()->Submit({cmd_buffer}).ok()) {
    FML_DLOG(ERROR) << "Could not copy contents into Y/UV texture.";
  }

  return {y_texture, uv_texture};
}

TEST_P(EntityTest, YUVToRGBFilter) {
  if (GetParam() == PlaygroundBackend::kOpenGLES) {
    // TODO(114588) : Support YUV to RGB filter on OpenGLES backend.
    GTEST_SKIP()
        << "YUV to RGB filter is not supported on OpenGLES backend yet.";
  }

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    YUVColorSpace yuv_color_space_array[2]{YUVColorSpace::kBT601FullRange,
                                           YUVColorSpace::kBT601LimitedRange};
    for (int i = 0; i < 2; i++) {
      auto yuv_color_space = yuv_color_space_array[i];
      auto textures =
          CreateTestYUVTextures(GetContext().get(), yuv_color_space);
      auto filter_contents = FilterContents::MakeYUVToRGBFilter(
          textures[0], textures[1], yuv_color_space);
      Entity filter_entity;
      filter_entity.SetContents(filter_contents);
      auto snapshot = filter_contents->RenderToSnapshot(context, filter_entity);

      Entity entity;
      auto contents = TextureContents::MakeRect(Rect::MakeLTRB(0, 0, 256, 256));
      contents->SetTexture(snapshot->texture);
      contents->SetSourceRect(Rect::MakeSize(snapshot->texture->GetSize()));
      entity.SetContents(contents);
      entity.SetTransform(
          Matrix::MakeTranslation({static_cast<Scalar>(100 + 400 * i), 300}));
      entity.Render(context, pass);
    }
    return true;
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, RuntimeEffect) {
  auto runtime_stages =
      OpenAssetAsRuntimeStage("runtime_stage_example.frag.iplr");
  auto runtime_stage =
      runtime_stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];
  ASSERT_TRUE(runtime_stage);
  ASSERT_TRUE(runtime_stage->IsDirty());

  bool expect_dirty = true;

  PipelineRef first_pipeline;
  std::unique_ptr<Geometry> geom = Geometry::MakeCover();

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    EXPECT_EQ(runtime_stage->IsDirty(), expect_dirty);

    auto contents = std::make_shared<RuntimeEffectContents>();
    contents->SetGeometry(geom.get());
    contents->SetRuntimeStage(runtime_stage);

    struct FragUniforms {
      Vector2 iResolution;
      Scalar iTime;
    } frag_uniforms = {
        .iResolution = Vector2(GetWindowSize().width, GetWindowSize().height),
        .iTime = static_cast<Scalar>(GetSecondsElapsed()),
    };
    auto uniform_data = std::make_shared<std::vector<uint8_t>>();
    uniform_data->resize(sizeof(FragUniforms));
    memcpy(uniform_data->data(), &frag_uniforms, sizeof(FragUniforms));
    contents->SetUniformData(uniform_data);

    Entity entity;
    entity.SetContents(contents);
    bool result = contents->Render(context, entity, pass);

    if (expect_dirty) {
      first_pipeline = pass.GetCommands().back().pipeline;
    } else {
      EXPECT_EQ(pass.GetCommands().back().pipeline, first_pipeline);
    }
    expect_dirty = false;
    return result;
  };

  // Simulate some renders and hot reloading of the shader.
  auto content_context = GetContentContext();
  {
    RenderTarget target =
        content_context->GetRenderTargetCache()->CreateOffscreen(
            *content_context->GetContext(), {1, 1}, 1u);

    testing::MockRenderPass mock_pass(GetContext(), target);
    callback(*content_context, mock_pass);
    callback(*content_context, mock_pass);

    // Dirty the runtime stage.
    runtime_stages = OpenAssetAsRuntimeStage("runtime_stage_example.frag.iplr");
    runtime_stage =
        runtime_stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];

    ASSERT_TRUE(runtime_stage->IsDirty());
    expect_dirty = true;

    callback(*content_context, mock_pass);
  }
}

TEST_P(EntityTest, RuntimeEffectCanSuccessfullyRender) {
  auto runtime_stages =
      OpenAssetAsRuntimeStage("runtime_stage_example.frag.iplr");
  auto runtime_stage =
      runtime_stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];
  ASSERT_TRUE(runtime_stage);
  ASSERT_TRUE(runtime_stage->IsDirty());

  auto contents = std::make_shared<RuntimeEffectContents>();
  auto geom = Geometry::MakeCover();
  contents->SetGeometry(geom.get());
  contents->SetRuntimeStage(runtime_stage);

  struct FragUniforms {
    Vector2 iResolution;
    Scalar iTime;
  } frag_uniforms = {
      .iResolution = Vector2(GetWindowSize().width, GetWindowSize().height),
      .iTime = static_cast<Scalar>(GetSecondsElapsed()),
  };
  auto uniform_data = std::make_shared<std::vector<uint8_t>>();
  uniform_data->resize(sizeof(FragUniforms));
  memcpy(uniform_data->data(), &frag_uniforms, sizeof(FragUniforms));
  contents->SetUniformData(uniform_data);

  Entity entity;
  entity.SetContents(contents);

  // Create a render target with a depth-stencil, similar to how EntityPass
  // does.
  RenderTarget target =
      GetContentContext()->GetRenderTargetCache()->CreateOffscreenMSAA(
          *GetContext(), {GetWindowSize().width, GetWindowSize().height}, 1,
          "RuntimeEffect Texture");
  testing::MockRenderPass pass(GetContext(), target);

  ASSERT_TRUE(contents->Render(*GetContentContext(), entity, pass));
  ASSERT_EQ(pass.GetCommands().size(), 1u);
  const auto& command = pass.GetCommands()[0];
  ASSERT_TRUE(command.pipeline->GetDescriptor()
                  .GetDepthStencilAttachmentDescriptor()
                  .has_value());
  ASSERT_TRUE(command.pipeline->GetDescriptor()
                  .GetFrontStencilAttachmentDescriptor()
                  .has_value());
}

TEST_P(EntityTest, RuntimeEffectCanPrecache) {
  auto runtime_stages =
      OpenAssetAsRuntimeStage("runtime_stage_example.frag.iplr");
  auto runtime_stage =
      runtime_stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];
  ASSERT_TRUE(runtime_stage);
  ASSERT_TRUE(runtime_stage->IsDirty());

  auto contents = std::make_shared<RuntimeEffectContents>();
  contents->SetRuntimeStage(runtime_stage);

  EXPECT_TRUE(contents->BootstrapShader(*GetContentContext()));
}

TEST_P(EntityTest, RuntimeEffectSetsRightSizeWhenUniformIsStruct) {
  if (GetBackend() != PlaygroundBackend::kVulkan) {
    GTEST_SKIP() << "Test only applies to Vulkan";
  }

  auto runtime_stages =
      OpenAssetAsRuntimeStage("runtime_stage_example.frag.iplr");
  auto runtime_stage =
      runtime_stages[PlaygroundBackendToRuntimeStageBackend(GetBackend())];
  ASSERT_TRUE(runtime_stage);
  ASSERT_TRUE(runtime_stage->IsDirty());

  auto contents = std::make_shared<RuntimeEffectContents>();
  auto geom = Geometry::MakeCover();
  contents->SetGeometry(geom.get());
  contents->SetRuntimeStage(runtime_stage);

  struct FragUniforms {
    Vector2 iResolution;
    Scalar iTime;
  } frag_uniforms = {
      .iResolution = Vector2(GetWindowSize().width, GetWindowSize().height),
      .iTime = static_cast<Scalar>(GetSecondsElapsed()),
  };
  auto uniform_data = std::make_shared<std::vector<uint8_t>>();
  uniform_data->resize(sizeof(FragUniforms));
  memcpy(uniform_data->data(), &frag_uniforms, sizeof(FragUniforms));

  auto buffer_view = RuntimeEffectContents::EmplaceVulkanUniform(
      uniform_data, GetContentContext()->GetTransientsBuffer(),
      runtime_stage->GetUniforms()[0]);

  // 16 bytes:
  //   8 bytes for iResolution
  //   4 bytes for iTime
  //   4 bytes padding
  EXPECT_EQ(buffer_view.GetRange().length, 16u);
}

TEST_P(EntityTest, ColorFilterWithForegroundColorAdvancedBlend) {
  auto image = CreateTextureForFixture("boston.jpg");
  auto filter = ColorFilterContents::MakeBlend(
      BlendMode::kColorBurn, FilterInput::Make({image}), Color::Red());

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    Entity entity;
    entity.SetTransform(Matrix::MakeScale(GetContentScale()) *
                        Matrix::MakeTranslation({500, 300}) *
                        Matrix::MakeScale(Vector2{0.5, 0.5}));
    entity.SetContents(filter);
    return entity.Render(context, pass);
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, ColorFilterWithForegroundColorClearBlend) {
  auto image = CreateTextureForFixture("boston.jpg");
  auto filter = ColorFilterContents::MakeBlend(
      BlendMode::kClear, FilterInput::Make({image}), Color::Red());

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    Entity entity;
    entity.SetTransform(Matrix::MakeScale(GetContentScale()) *
                        Matrix::MakeTranslation({500, 300}) *
                        Matrix::MakeScale(Vector2{0.5, 0.5}));
    entity.SetContents(filter);
    return entity.Render(context, pass);
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, ColorFilterWithForegroundColorSrcBlend) {
  auto image = CreateTextureForFixture("boston.jpg");
  auto filter = ColorFilterContents::MakeBlend(
      BlendMode::kSource, FilterInput::Make({image}), Color::Red());

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    Entity entity;
    entity.SetTransform(Matrix::MakeScale(GetContentScale()) *
                        Matrix::MakeTranslation({500, 300}) *
                        Matrix::MakeScale(Vector2{0.5, 0.5}));
    entity.SetContents(filter);
    return entity.Render(context, pass);
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, ColorFilterWithForegroundColorDstBlend) {
  auto image = CreateTextureForFixture("boston.jpg");
  auto filter = ColorFilterContents::MakeBlend(
      BlendMode::kDestination, FilterInput::Make({image}), Color::Red());

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    Entity entity;
    entity.SetTransform(Matrix::MakeScale(GetContentScale()) *
                        Matrix::MakeTranslation({500, 300}) *
                        Matrix::MakeScale(Vector2{0.5, 0.5}));
    entity.SetContents(filter);
    return entity.Render(context, pass);
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, ColorFilterWithForegroundColorSrcInBlend) {
  auto image = CreateTextureForFixture("boston.jpg");
  auto filter = ColorFilterContents::MakeBlend(
      BlendMode::kSourceIn, FilterInput::Make({image}), Color::Red());

  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    Entity entity;
    entity.SetTransform(Matrix::MakeScale(GetContentScale()) *
                        Matrix::MakeTranslation({500, 300}) *
                        Matrix::MakeScale(Vector2{0.5, 0.5}));
    entity.SetContents(filter);
    return entity.Render(context, pass);
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, CoverageForStrokePathWithNegativeValuesInTransform) {
  auto arrow_head = PathBuilder{}
                        .MoveTo({50, 120})
                        .LineTo({120, 190})
                        .LineTo({190, 120})
                        .TakePath();
  auto geometry = Geometry::MakeStrokePath(arrow_head, 15.0, 4.0, Cap::kRound,
                                           Join::kRound);

  auto transform = Matrix::MakeTranslation({300, 300}) *
                   Matrix::MakeRotationZ(Radians(kPiOver2));
  // Note that e[0][0] used to be tested here, but it was -epsilon solely
  // due to floating point inaccuracy in the transcendental trig functions.
  // e[1][0] is the intended negative value that we care about (-1.0) as it
  // comes from the rotation of pi/2.
  EXPECT_LT(transform.e[1][0], 0.0f);
  auto coverage = geometry->GetCoverage(transform);
  ASSERT_RECT_NEAR(coverage.value(), Rect::MakeXYWH(102.5, 342.5, 85, 155));
}

TEST_P(EntityTest, SolidColorContentsIsOpaque) {
  Matrix matrix;
  SolidColorContents contents;
  auto geom = Geometry::MakeRect(Rect::MakeLTRB(0, 0, 10, 10));
  contents.SetGeometry(geom.get());

  contents.SetColor(Color::CornflowerBlue());
  EXPECT_TRUE(contents.IsOpaque(matrix));
  contents.SetColor(Color::CornflowerBlue().WithAlpha(0.5));
  EXPECT_FALSE(contents.IsOpaque(matrix));

  // Create stroked path that required alpha coverage.
  geom = Geometry::MakeStrokePath(
      PathBuilder{}.AddLine({0, 0}, {100, 100}).TakePath(),
      /*stroke_width=*/0.05);
  contents.SetGeometry(geom.get());
  contents.SetColor(Color::CornflowerBlue());

  EXPECT_FALSE(contents.IsOpaque(matrix));
}

TEST_P(EntityTest, ConicalGradientContentsIsOpaque) {
  Matrix matrix;
  ConicalGradientContents contents;
  auto geom = Geometry::MakeRect(Rect::MakeLTRB(0, 0, 10, 10));
  contents.SetGeometry(geom.get());

  contents.SetColors({Color::CornflowerBlue()});
  EXPECT_FALSE(contents.IsOpaque(matrix));
  contents.SetColors({Color::CornflowerBlue().WithAlpha(0.5)});
  EXPECT_FALSE(contents.IsOpaque(matrix));

  // Create stroked path that required alpha coverage.
  geom = Geometry::MakeStrokePath(
      PathBuilder{}.AddLine({0, 0}, {100, 100}).TakePath(),
      /*stroke_width=*/0.05);
  contents.SetGeometry(geom.get());
  contents.SetColors({Color::CornflowerBlue()});

  EXPECT_FALSE(contents.IsOpaque(matrix));
}

TEST_P(EntityTest, LinearGradientContentsIsOpaque) {
  Matrix matrix;
  LinearGradientContents contents;
  auto geom = Geometry::MakeRect(Rect::MakeLTRB(0, 0, 10, 10));
  contents.SetGeometry(geom.get());

  contents.SetColors({Color::CornflowerBlue()});
  EXPECT_TRUE(contents.IsOpaque(matrix));
  contents.SetColors({Color::CornflowerBlue().WithAlpha(0.5)});
  EXPECT_FALSE(contents.IsOpaque(matrix));
  contents.SetColors({Color::CornflowerBlue()});
  contents.SetTileMode(Entity::TileMode::kDecal);
  EXPECT_FALSE(contents.IsOpaque(matrix));

  // Create stroked path that required alpha coverage.
  geom = Geometry::MakeStrokePath(
      PathBuilder{}.AddLine({0, 0}, {100, 100}).TakePath(),
      /*stroke_width=*/0.05);
  contents.SetGeometry(geom.get());
  contents.SetColors({Color::CornflowerBlue()});

  EXPECT_FALSE(contents.IsOpaque(matrix));
}

TEST_P(EntityTest, RadialGradientContentsIsOpaque) {
  Matrix matrix;
  RadialGradientContents contents;
  auto geom = Geometry::MakeRect(Rect::MakeLTRB(0, 0, 10, 10));
  contents.SetGeometry(geom.get());

  contents.SetColors({Color::CornflowerBlue()});
  EXPECT_TRUE(contents.IsOpaque(matrix));
  contents.SetColors({Color::CornflowerBlue().WithAlpha(0.5)});
  EXPECT_FALSE(contents.IsOpaque(matrix));
  contents.SetColors({Color::CornflowerBlue()});
  contents.SetTileMode(Entity::TileMode::kDecal);
  EXPECT_FALSE(contents.IsOpaque(matrix));

  // Create stroked path that required alpha coverage.
  geom = Geometry::MakeStrokePath(
      PathBuilder{}.AddLine({0, 0}, {100, 100}).TakePath(),
      /*stroke_width=*/0.05);
  contents.SetGeometry(geom.get());
  contents.SetColors({Color::CornflowerBlue()});

  EXPECT_FALSE(contents.IsOpaque(matrix));
}

TEST_P(EntityTest, SweepGradientContentsIsOpaque) {
  Matrix matrix;
  RadialGradientContents contents;
  auto geom = Geometry::MakeRect(Rect::MakeLTRB(0, 0, 10, 10));
  contents.SetGeometry(geom.get());

  contents.SetColors({Color::CornflowerBlue()});
  EXPECT_TRUE(contents.IsOpaque(matrix));
  contents.SetColors({Color::CornflowerBlue().WithAlpha(0.5)});
  EXPECT_FALSE(contents.IsOpaque(matrix));
  contents.SetColors({Color::CornflowerBlue()});
  contents.SetTileMode(Entity::TileMode::kDecal);
  EXPECT_FALSE(contents.IsOpaque(matrix));

  // Create stroked path that required alpha coverage.
  geom = Geometry::MakeStrokePath(
      PathBuilder{}.AddLine({0, 0}, {100, 100}).TakePath(),
      /*stroke_width=*/0.05);
  contents.SetGeometry(geom.get());
  contents.SetColors({Color::CornflowerBlue()});

  EXPECT_FALSE(contents.IsOpaque(matrix));
}

TEST_P(EntityTest, TiledTextureContentsIsOpaque) {
  Matrix matrix;
  auto bay_bridge = CreateTextureForFixture("bay_bridge.jpg");
  TiledTextureContents contents;
  contents.SetTexture(bay_bridge);
  // This is a placeholder test. Images currently never decompress as opaque
  // (whether in Flutter or the playground), and so this should currently always
  // return false in practice.
  EXPECT_FALSE(contents.IsOpaque(matrix));
}

TEST_P(EntityTest, PointFieldGeometryCoverage) {
  std::vector<Point> points = {{10, 20}, {100, 200}};
  PointFieldGeometry geometry(points.data(), 2, 5.0, false);
  ASSERT_EQ(geometry.GetCoverage(Matrix()), Rect::MakeLTRB(5, 15, 105, 205));
  ASSERT_EQ(geometry.GetCoverage(Matrix::MakeTranslation({30, 0, 0})),
            Rect::MakeLTRB(35, 15, 135, 205));
}

TEST_P(EntityTest, ColorFilterContentsWithLargeGeometry) {
  Entity entity;
  entity.SetTransform(Matrix::MakeScale(GetContentScale()));
  auto src_contents = std::make_shared<SolidColorContents>();
  auto src_geom = Geometry::MakeRect(Rect::MakeLTRB(-300, -500, 30000, 50000));
  src_contents->SetGeometry(src_geom.get());
  src_contents->SetColor(Color::Red());

  auto dst_contents = std::make_shared<SolidColorContents>();
  auto dst_geom = Geometry::MakeRect(Rect::MakeLTRB(300, 500, 20000, 30000));
  dst_contents->SetGeometry(dst_geom.get());
  dst_contents->SetColor(Color::Blue());

  auto contents = ColorFilterContents::MakeBlend(
      BlendMode::kSourceOver, {FilterInput::Make(dst_contents, false),
                               FilterInput::Make(src_contents, false)});
  entity.SetContents(std::move(contents));
  ASSERT_TRUE(OpenPlaygroundHere(std::move(entity)));
}

TEST_P(EntityTest, TextContentsCeilsGlyphScaleToDecimal) {
  ASSERT_EQ(TextFrame::RoundScaledFontSize(0.4321111f), 0.43f);
  ASSERT_EQ(TextFrame::RoundScaledFontSize(0.5321111f), 0.53f);
  ASSERT_EQ(TextFrame::RoundScaledFontSize(2.1f), 2.1f);
  ASSERT_EQ(TextFrame::RoundScaledFontSize(0.0f), 0.0f);
  ASSERT_EQ(TextFrame::RoundScaledFontSize(100000000.0f), 48.0f);
}

TEST_P(EntityTest, SpecializationConstantsAreAppliedToVariants) {
  auto content_context = GetContentContext();

  auto default_gyph = content_context->GetGlyphAtlasPipeline({
      .color_attachment_pixel_format = PixelFormat::kR8G8B8A8UNormInt,
      .has_depth_stencil_attachments = false,
  });
  auto alt_gyph = content_context->GetGlyphAtlasPipeline(
      {.color_attachment_pixel_format = PixelFormat::kR8G8B8A8UNormInt,
       .has_depth_stencil_attachments = true});

  EXPECT_NE(default_gyph, alt_gyph);
  EXPECT_EQ(default_gyph->GetDescriptor().GetSpecializationConstants(),
            alt_gyph->GetDescriptor().GetSpecializationConstants());

  auto use_a8 = GetContext()->GetCapabilities()->GetDefaultGlyphAtlasFormat() ==
                PixelFormat::kA8UNormInt;

  std::vector<Scalar> expected_constants = {static_cast<Scalar>(use_a8)};
  EXPECT_EQ(default_gyph->GetDescriptor().GetSpecializationConstants(),
            expected_constants);
}

TEST_P(EntityTest, DecalSpecializationAppliedToMorphologyFilter) {
  auto content_context = GetContentContext();
  auto default_color_burn = content_context->GetMorphologyFilterPipeline({
      .color_attachment_pixel_format = PixelFormat::kR8G8B8A8UNormInt,
  });

  auto decal_supported = static_cast<Scalar>(
      GetContext()->GetCapabilities()->SupportsDecalSamplerAddressMode());
  std::vector<Scalar> expected_constants = {decal_supported};
  ASSERT_EQ(default_color_burn->GetDescriptor().GetSpecializationConstants(),
            expected_constants);
}

// This doesn't really tell you if the hashes will have frequent
// collisions, but since this type is only used to hash a bounded
// set of options, we can just compare benchmarks.
TEST_P(EntityTest, ContentContextOptionsHasReasonableHashFunctions) {
  ContentContextOptions opts;
  auto hash_a = opts.ToKey();

  opts.blend_mode = BlendMode::kColorBurn;
  auto hash_b = opts.ToKey();

  opts.has_depth_stencil_attachments = false;
  auto hash_c = opts.ToKey();

  opts.primitive_type = PrimitiveType::kPoint;
  auto hash_d = opts.ToKey();

  EXPECT_NE(hash_a, hash_b);
  EXPECT_NE(hash_b, hash_c);
  EXPECT_NE(hash_c, hash_d);
}

#ifdef FML_OS_LINUX
TEST_P(EntityTest, FramebufferFetchVulkanBindingOffsetIsTheSame) {
  // Using framebuffer fetch on Vulkan requires that we maintain a subpass input
  // binding that we don't have a good route for configuring with the
  // current metadata approach. This test verifies that the binding value
  // doesn't change
  // from the expected constant.
  // See also:
  //   * impeller/renderer/backend/vulkan/binding_helpers_vk.cc
  //   * impeller/entity/shaders/blending/framebuffer_blend.frag
  // This test only works on Linux because macOS hosts incorrectly
  // populate  the
  // Vulkan descriptor sets based on the MSL compiler settings.

  bool expected_layout = false;
  for (const DescriptorSetLayout& layout : FramebufferBlendColorBurnPipeline::
           FragmentShader::kDescriptorSetLayouts) {
    if (layout.binding == 64 &&
        layout.descriptor_type == DescriptorType::kInputAttachment) {
      expected_layout = true;
    }
  }
  EXPECT_TRUE(expected_layout);
}
#endif

TEST_P(EntityTest, FillPathGeometryGetPositionBufferReturnsExpectedMode) {
  RenderTarget target;
  testing::MockRenderPass mock_pass(GetContext(), target);

  auto get_result = [this, &mock_pass](const Path& path) {
    auto geometry = Geometry::MakeFillPath(
        path, /* inner rect */ Rect::MakeLTRB(0, 0, 100, 100));
    return geometry->GetPositionBuffer(*GetContentContext(), {}, mock_pass);
  };

  // Convex path
  {
    GeometryResult result =
        get_result(PathBuilder{}
                       .AddRect(Rect::MakeLTRB(0, 0, 100, 100))
                       .SetConvexity(Convexity::kConvex)
                       .TakePath());
    EXPECT_EQ(result.mode, GeometryResult::Mode::kNormal);
  }

  // Concave path
  {
    Path path = PathBuilder{}
                    .MoveTo({0, 0})
                    .LineTo({100, 0})
                    .LineTo({100, 100})
                    .LineTo({50, 50})
                    .Close()
                    .TakePath();
    GeometryResult result = get_result(path);
    EXPECT_EQ(result.mode, GeometryResult::Mode::kNonZero);
  }
}

TEST_P(EntityTest, FailOnValidationError) {
  if (GetParam() != PlaygroundBackend::kVulkan) {
    GTEST_SKIP() << "Validation is only fatal on Vulkan backend.";
  }
  EXPECT_DEATH(
      // The easiest way to trigger a validation error is to try to compile
      // a shader with an unsupported pixel format.
      GetContentContext()->GetBlendColorBurnPipeline({
          .color_attachment_pixel_format = PixelFormat::kUnknown,
          .has_depth_stencil_attachments = false,
      }),
      "");
}

TEST_P(EntityTest, CanComputeGeometryForEmptyPathsWithoutCrashing) {
  PathBuilder builder = {};
  builder.AddRect(Rect::MakeLTRB(0, 0, 0, 0));
  Path path = builder.TakePath();

  EXPECT_TRUE(path.GetBoundingBox()->IsEmpty());

  auto geom = Geometry::MakeFillPath(path);

  Entity entity;
  RenderTarget target =
      GetContentContext()->GetRenderTargetCache()->CreateOffscreen(
          *GetContext(), {1, 1}, 1u);
  testing::MockRenderPass render_pass(GetContext(), target);
  auto position_result =
      geom->GetPositionBuffer(*GetContentContext(), entity, render_pass);

  EXPECT_EQ(position_result.vertex_buffer.vertex_count, 0u);

  EXPECT_EQ(geom->GetResultMode(), GeometryResult::Mode::kNormal);
}

TEST_P(EntityTest, CanRenderEmptyPathsWithoutCrashing) {
  PathBuilder builder = {};
  builder.AddRect(Rect::MakeLTRB(0, 0, 0, 0));
  Path path = builder.TakePath();

  EXPECT_TRUE(path.GetBoundingBox()->IsEmpty());

  auto contents = std::make_shared<SolidColorContents>();
  std::unique_ptr<Geometry> geom = Geometry::MakeFillPath(path);
  contents->SetGeometry(geom.get());
  contents->SetColor(Color::Red());

  Entity entity;
  entity.SetTransform(Matrix::MakeScale(GetContentScale()));
  entity.SetContents(contents);

  ASSERT_TRUE(OpenPlaygroundHere(std::move(entity)));
}

TEST_P(EntityTest, DrawSuperEllipse) {
  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    // UI state.
    static float alpha = 10;
    static float beta = 10;
    static float radius = 40;
    static int degree = 4;
    static Color color = Color::Red();

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat("Alpha", &alpha, 0, 100);
    ImGui::SliderFloat("Beta", &beta, 0, 100);
    ImGui::SliderInt("Degreee", &degree, 1, 20);
    ImGui::SliderFloat("Radius", &radius, 0, 400);
    ImGui::ColorEdit4("Color", reinterpret_cast<float*>(&color));
    ImGui::End();

    auto contents = std::make_shared<SolidColorContents>();
    std::unique_ptr<SuperellipseGeometry> geom =
        std::make_unique<SuperellipseGeometry>(Point{400, 400}, radius, degree,
                                               alpha, beta);
    contents->SetColor(color);
    contents->SetGeometry(geom.get());

    Entity entity;
    entity.SetContents(contents);

    return entity.Render(context, pass);
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, DrawRoundSuperEllipse) {
  auto callback = [&](ContentContext& context, RenderPass& pass) -> bool {
    // UI state.
    static float center_x = 100;
    static float center_y = 100;
    static float width = 900;
    static float height = 900;
    static float corner_radius = 300;
    static Color color = Color::Red();

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat("Center X", &center_x, 0, 1000);
    ImGui::SliderFloat("Center Y", &center_y, 0, 1000);
    ImGui::SliderFloat("Width", &width, 0, 1000);
    ImGui::SliderFloat("Height", &height, 0, 1000);
    ImGui::SliderFloat("Corner radius", &corner_radius, 0, 500);
    ImGui::End();

    auto contents = std::make_shared<SolidColorContents>();
    std::unique_ptr<RoundSuperellipseGeometry> geom =
        std::make_unique<RoundSuperellipseGeometry>(
            Rect::MakeOriginSize({center_x, center_y}, {width, height}),
            corner_radius);
    contents->SetColor(color);
    contents->SetGeometry(geom.get());

    Entity entity;
    entity.SetContents(contents);

    return entity.Render(context, pass);
  };

  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(EntityTest, SolidColorApplyColorFilter) {
  auto contents = SolidColorContents();
  contents.SetColor(Color::CornflowerBlue().WithAlpha(0.75));
  auto result = contents.ApplyColorFilter([](const Color& color) {
    return color.Blend(Color::LimeGreen().WithAlpha(0.75), BlendMode::kScreen);
  });
  ASSERT_TRUE(result);
  ASSERT_COLOR_NEAR(contents.GetColor(),
                    Color(0.424452, 0.828743, 0.79105, 0.9375));
}

#define APPLY_COLOR_FILTER_GRADIENT_TEST(name)                                 \
  TEST_P(EntityTest, name##GradientApplyColorFilter) {                         \
    auto contents = name##GradientContents();                                  \
    contents.SetColors({Color::CornflowerBlue().WithAlpha(0.75)});             \
    auto result = contents.ApplyColorFilter([](const Color& color) {           \
      return color.Blend(Color::LimeGreen().WithAlpha(0.75),                   \
                         BlendMode::kScreen);                                  \
    });                                                                        \
    ASSERT_TRUE(result);                                                       \
                                                                               \
    std::vector<Color> expected = {Color(0.433247, 0.879523, 0.825324, 0.75)}; \
    ASSERT_COLORS_NEAR(contents.GetColors(), expected);                        \
  }

APPLY_COLOR_FILTER_GRADIENT_TEST(Linear);
APPLY_COLOR_FILTER_GRADIENT_TEST(Radial);
APPLY_COLOR_FILTER_GRADIENT_TEST(Conical);
APPLY_COLOR_FILTER_GRADIENT_TEST(Sweep);

TEST_P(EntityTest, GiantStrokePathAllocation) {
  PathBuilder builder{};
  for (int i = 0; i < 10000; i++) {
    builder.LineTo(Point(i, i));
  }
  Path path = builder.TakePath();
  auto geom = Geometry::MakeStrokePath(path, /*stroke_width=*/10);

  ContentContext content_context(GetContext(), /*typographer_context=*/nullptr);
  Entity entity;

  auto cmd_buffer = content_context.GetContext()->CreateCommandBuffer();

  RenderTargetAllocator allocator(
      content_context.GetContext()->GetResourceAllocator());

  auto render_target = allocator.CreateOffscreen(
      *content_context.GetContext(), /*size=*/{10, 10}, /*mip_count=*/1);
  auto pass = cmd_buffer->CreateRenderPass(render_target);

  GeometryResult result =
      geom->GetPositionBuffer(content_context, entity, *pass);

  // Validate the buffer data overflowed the small buffer
  EXPECT_GT(result.vertex_buffer.vertex_count, kPointArenaSize);

  // Validate that there are no uninitialized points near the gap.
  Point* written_data = reinterpret_cast<Point*>(
      (result.vertex_buffer.vertex_buffer.GetBuffer()->OnGetContents() +
       result.vertex_buffer.vertex_buffer.GetRange().offset));

  std::vector<Point> expected = {
      Point(1019.46, 1026.54),  //
      Point(1026.54, 1019.46),  //
      Point(1020.45, 1027.54),  //
      Point(1027.54, 1020.46),  //
      Point(1020.46, 1027.53)   //
  };

  Point point = written_data[kPointArenaSize - 2];
  EXPECT_NEAR(point.x, expected[0].x, 0.1);
  EXPECT_NEAR(point.y, expected[0].y, 0.1);

  point = written_data[kPointArenaSize - 1];
  EXPECT_NEAR(point.x, expected[1].x, 0.1);
  EXPECT_NEAR(point.y, expected[1].y, 0.1);

  point = written_data[kPointArenaSize];
  EXPECT_NEAR(point.x, expected[2].x, 0.1);
  EXPECT_NEAR(point.y, expected[2].y, 0.1);

  point = written_data[kPointArenaSize + 1];
  EXPECT_NEAR(point.x, expected[3].x, 0.1);
  EXPECT_NEAR(point.y, expected[3].y, 0.1);

  point = written_data[kPointArenaSize + 2];
  EXPECT_NEAR(point.x, expected[4].x, 0.1);
  EXPECT_NEAR(point.y, expected[4].y, 0.1);
}

TEST_P(EntityTest, GiantLineStripPathAllocation) {
  PathBuilder builder{};
  for (int i = 0; i < 10000; i++) {
    builder.LineTo(Point(i, i));
  }
  Path path = builder.TakePath();

  ContentContext content_context(GetContext(), /*typographer_context=*/nullptr);
  Entity entity;

  auto host_buffer = HostBuffer::Create(GetContext()->GetResourceAllocator(),
                                        GetContext()->GetIdleWaiter());
  auto tessellator = Tessellator();

  auto vertex_buffer = tessellator.GenerateLineStrip(path, *host_buffer, 1.0);

  // Validate the buffer data overflowed the small buffer
  EXPECT_GT(vertex_buffer.vertex_count, kPointArenaSize);

  // Validate that there are no uninitialized points near the gap.
  Point* written_data = reinterpret_cast<Point*>(
      (vertex_buffer.vertex_buffer.GetBuffer()->OnGetContents() +
       vertex_buffer.vertex_buffer.GetRange().offset));

  std::vector<Point> expected = {
      Point(4093, 4093),  //
      Point(4094, 4094),  //
      Point(4095, 4095),  //
      Point(4096, 4096),  //
      Point(4097, 4097)   //
  };

  Point point = written_data[kPointArenaSize - 2];
  EXPECT_NEAR(point.x, expected[0].x, 0.1);
  EXPECT_NEAR(point.y, expected[0].y, 0.1);

  point = written_data[kPointArenaSize - 1];
  EXPECT_NEAR(point.x, expected[1].x, 0.1);
  EXPECT_NEAR(point.y, expected[1].y, 0.1);

  point = written_data[kPointArenaSize];
  EXPECT_NEAR(point.x, expected[2].x, 0.1);
  EXPECT_NEAR(point.y, expected[2].y, 0.1);

  point = written_data[kPointArenaSize + 1];
  EXPECT_NEAR(point.x, expected[3].x, 0.1);
  EXPECT_NEAR(point.y, expected[3].y, 0.1);

  point = written_data[kPointArenaSize + 2];
  EXPECT_NEAR(point.x, expected[4].x, 0.1);
  EXPECT_NEAR(point.y, expected[4].y, 0.1);
}

}  // namespace testing
}  // namespace impeller

// NOLINTEND(bugprone-unchecked-optional-access)

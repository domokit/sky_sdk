// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include "flutter/testing/testing.h"
#include "gtest/gtest.h"
#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/geometry/geometry.h"
#include "impeller/entity/geometry/stroke_path_geometry.h"
#include "impeller/geometry/constants.h"
#include "impeller/geometry/geometry_asserts.h"
#include "impeller/geometry/path_builder.h"
#include "impeller/renderer/testing/mocks.h"

inline ::testing::AssertionResult SolidVerticesNear(
    std::vector<impeller::Point> a,
    std::vector<impeller::Point> b) {
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure() << "Colors length does not match";
  }
  for (auto i = 0u; i < b.size(); i++) {
    if (!PointNear(a[i], b[i])) {
      return ::testing::AssertionFailure() << "Positions are not equal.";
    }
  }
  return ::testing::AssertionSuccess();
}

inline ::testing::AssertionResult TextureVerticesNear(
    std::vector<impeller::TextureFillVertexShader::PerVertexData> a,
    std::vector<impeller::TextureFillVertexShader::PerVertexData> b) {
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure() << "Colors length does not match";
  }
  for (auto i = 0u; i < b.size(); i++) {
    if (!PointNear(a[i].position, b[i].position)) {
      return ::testing::AssertionFailure() << "Positions are not equal.";
    }
    if (!PointNear(a[i].texture_coords, b[i].texture_coords)) {
      return ::testing::AssertionFailure() << "Texture coords are not equal.";
    }
  }
  return ::testing::AssertionSuccess();
}

#define EXPECT_SOLID_VERTICES_NEAR(a, b) \
  EXPECT_PRED2(&::SolidVerticesNear, a, b)
#define EXPECT_TEXTURE_VERTICES_NEAR(a, b) \
  EXPECT_PRED2(&::TextureVerticesNear, a, b)

namespace impeller {

class ImpellerEntityUnitTestAccessor {
 public:
  static std::vector<Point> GenerateSolidStrokeVertices(
      const Path::Polyline& polyline,
      Scalar stroke_width,
      Scalar miter_limit,
      Join stroke_join,
      Cap stroke_cap,
      Scalar scale) {
    return StrokePathGeometry::GenerateSolidStrokeVertices(
        polyline, stroke_width, miter_limit, stroke_join, stroke_cap, scale);
  }
};

namespace testing {

TEST(EntityGeometryTest, RectGeometryCoversArea) {
  auto geometry = Geometry::MakeRect(Rect::MakeLTRB(0, 0, 100, 100));
  ASSERT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(0, 0, 100, 100)));
  ASSERT_FALSE(geometry->CoversArea({}, Rect::MakeLTRB(-1, 0, 100, 100)));
  ASSERT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(1, 1, 100, 100)));
  ASSERT_TRUE(geometry->CoversArea({}, Rect()));
}

TEST(EntityGeometryTest, FillPathGeometryCoversArea) {
  auto path = PathBuilder{}.AddRect(Rect::MakeLTRB(0, 0, 100, 100)).TakePath();
  auto geometry = Geometry::MakeFillPath(
      path, /* inner rect */ Rect::MakeLTRB(0, 0, 100, 100));
  ASSERT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(0, 0, 100, 100)));
  ASSERT_FALSE(geometry->CoversArea({}, Rect::MakeLTRB(-1, 0, 100, 100)));
  ASSERT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(1, 1, 100, 100)));
  ASSERT_TRUE(geometry->CoversArea({}, Rect()));
}

TEST(EntityGeometryTest, FillPathGeometryCoversAreaNoInnerRect) {
  auto path = PathBuilder{}.AddRect(Rect::MakeLTRB(0, 0, 100, 100)).TakePath();
  auto geometry = Geometry::MakeFillPath(path);
  ASSERT_FALSE(geometry->CoversArea({}, Rect::MakeLTRB(0, 0, 100, 100)));
  ASSERT_FALSE(geometry->CoversArea({}, Rect::MakeLTRB(-1, 0, 100, 100)));
  ASSERT_FALSE(geometry->CoversArea({}, Rect::MakeLTRB(1, 1, 100, 100)));
  ASSERT_FALSE(geometry->CoversArea({}, Rect()));
}

TEST(EntityGeometryTest, LineGeometryCoverage) {
  {
    auto geometry = Geometry::MakeLine({10, 10}, {20, 10}, 2, Cap::kButt);
    EXPECT_EQ(geometry->GetCoverage({}), Rect::MakeLTRB(10, 9, 20, 11));
    EXPECT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(10, 9, 20, 11)));
  }

  {
    auto geometry = Geometry::MakeLine({10, 10}, {20, 10}, 2, Cap::kSquare);
    EXPECT_EQ(geometry->GetCoverage({}), Rect::MakeLTRB(9, 9, 21, 11));
    EXPECT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(9, 9, 21, 11)));
  }

  {
    auto geometry = Geometry::MakeLine({10, 10}, {10, 20}, 2, Cap::kButt);
    EXPECT_EQ(geometry->GetCoverage({}), Rect::MakeLTRB(9, 10, 11, 20));
    EXPECT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(9, 10, 11, 20)));
  }

  {
    auto geometry = Geometry::MakeLine({10, 10}, {10, 20}, 2, Cap::kSquare);
    EXPECT_EQ(geometry->GetCoverage({}), Rect::MakeLTRB(9, 9, 11, 21));
    EXPECT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(9, 9, 11, 21)));
  }
}

TEST(EntityGeometryTest, RoundRectGeometryCoversArea) {
  auto geometry =
      Geometry::MakeRoundRect(Rect::MakeLTRB(0, 0, 100, 100), Size(20, 20));
  EXPECT_FALSE(geometry->CoversArea({}, Rect::MakeLTRB(15, 15, 85, 85)));
  EXPECT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(20, 20, 80, 80)));
  EXPECT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(30, 1, 70, 99)));
  EXPECT_TRUE(geometry->CoversArea({}, Rect::MakeLTRB(1, 30, 99, 70)));
}

TEST(EntityGeometryTest, GeometryResultHasReasonableDefaults) {
  GeometryResult result;
  EXPECT_EQ(result.type, PrimitiveType::kTriangleStrip);
  EXPECT_EQ(result.transform, Matrix());
  EXPECT_EQ(result.mode, GeometryResult::Mode::kNormal);
}

TEST(EntityGeometryTest, AlphaCoverageStrokePaths) {
  auto matrix = Matrix::MakeScale(Vector2{3.0, 3.0});
  EXPECT_EQ(Geometry::MakeStrokePath({}, 0.5)->ComputeAlphaCoverage(matrix), 1);
  EXPECT_NEAR(Geometry::MakeStrokePath({}, 0.1)->ComputeAlphaCoverage(matrix),
              0.6, 0.05);
  EXPECT_NEAR(Geometry::MakeStrokePath({}, 0.05)->ComputeAlphaCoverage(matrix),
              0.3, 0.05);
  EXPECT_NEAR(Geometry::MakeStrokePath({}, 0.01)->ComputeAlphaCoverage(matrix),
              0.1, 0.1);
  EXPECT_NEAR(
      Geometry::MakeStrokePath({}, 0.0000005)->ComputeAlphaCoverage(matrix),
      1e-05, 0.001);
  EXPECT_EQ(Geometry::MakeStrokePath({}, 0)->ComputeAlphaCoverage(matrix), 1);
  EXPECT_EQ(Geometry::MakeStrokePath({}, 40)->ComputeAlphaCoverage(matrix), 1);
}

}  // namespace testing
}  // namespace impeller

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_vertices.h"
#include "flutter/display_list/testing/dl_test_equality.h"
#include "flutter/display_list/utils/dl_comparable.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(DisplayListVertices, MakeWithZeroAndNegativeVerticesAndIndices) {
  std::shared_ptr<DlVertices> vertices1 = DlVertices::Make(
      DlVertexMode::kTriangles, 0, nullptr, nullptr, nullptr, 0, nullptr);
  EXPECT_NE(vertices1, nullptr);
  EXPECT_EQ(vertices1->vertex_count(), 0);
  EXPECT_EQ(vertices1->vertex_data(), nullptr);
  EXPECT_EQ(vertices1->texture_coordinate_data(), nullptr);
  EXPECT_EQ(vertices1->colors(), nullptr);
  EXPECT_EQ(vertices1->index_count(), 0);
  EXPECT_EQ(vertices1->indices(), nullptr);

  std::shared_ptr<DlVertices> vertices2 = DlVertices::Make(
      DlVertexMode::kTriangles, -1, nullptr, nullptr, nullptr, -1, nullptr);
  EXPECT_NE(vertices2, nullptr);
  EXPECT_EQ(vertices2->vertex_count(), 0);
  EXPECT_EQ(vertices2->vertex_data(), nullptr);
  EXPECT_EQ(vertices2->texture_coordinate_data(), nullptr);
  EXPECT_EQ(vertices2->colors(), nullptr);
  EXPECT_EQ(vertices2->index_count(), 0);
  EXPECT_EQ(vertices2->indices(), nullptr);

  TestEquals(*vertices1, *vertices2);
}

TEST(DisplayListVertices, MakeWithTexAndColorAndIndices) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlPoint texture_coords[3] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
  };
  DlColor colors[3] = {
      DlColor::kRed(),
      DlColor::kCyan(),
      DlColor::kGreen(),
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  std::shared_ptr<DlVertices> vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, texture_coords, colors, 6, indices);

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_NE(vertices->texture_coordinate_data(), nullptr);
  ASSERT_NE(vertices->colors(), nullptr);
  ASSERT_NE(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->texture_coordinate_data()[i], texture_coords[i]);
    ASSERT_EQ(vertices->colors()[i], colors[i]);
  }
  ASSERT_EQ(vertices->index_count(), 6);
  for (int i = 0; i < 6; i++) {
    ASSERT_EQ(vertices->indices()[i], indices[i]);
  }
}

TEST(DisplayListVertices, MakeWithTexAndColor) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlPoint texture_coords[3] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
  };
  DlColor colors[3] = {
      DlColor::kRed(),
      DlColor::kCyan(),
      DlColor::kGreen(),
  };

  std::shared_ptr<DlVertices> vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, texture_coords, colors, 6, nullptr);

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_NE(vertices->texture_coordinate_data(), nullptr);
  ASSERT_NE(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->texture_coordinate_data()[i], texture_coords[i]);
    ASSERT_EQ(vertices->colors()[i], colors[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, MakeWithTexAndIndices) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlPoint texture_coords[3] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  std::shared_ptr<DlVertices> vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, texture_coords, nullptr, 6, indices);

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_NE(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_NE(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->texture_coordinate_data()[i], texture_coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 6);
  for (int i = 0; i < 6; i++) {
    ASSERT_EQ(vertices->indices()[i], indices[i]);
  }
}

TEST(DisplayListVertices, MakeWithColorAndIndices) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlColor colors[3] = {
      DlColor::kRed(),
      DlColor::kCyan(),
      DlColor::kGreen(),
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  std::shared_ptr<DlVertices> vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, nullptr, colors, 6, indices);

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_NE(vertices->colors(), nullptr);
  ASSERT_NE(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->colors()[i], colors[i]);
  }
  ASSERT_EQ(vertices->index_count(), 6);
  for (int i = 0; i < 6; i++) {
    ASSERT_EQ(vertices->indices()[i], indices[i]);
  }
}

TEST(DisplayListVertices, MakeWithTex) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlPoint texture_coords[3] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
  };

  std::shared_ptr<DlVertices> vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, texture_coords, nullptr, 6, nullptr);

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_NE(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->texture_coordinate_data()[i], texture_coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, MakeWithColor) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlColor colors[3] = {
      DlColor::kRed(),
      DlColor::kCyan(),
      DlColor::kGreen(),
  };

  std::shared_ptr<DlVertices> vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, nullptr, colors, 6, nullptr);

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_NE(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->colors()[i], colors[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, MakeWithIndices) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  std::shared_ptr<DlVertices> vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, nullptr, nullptr, 6, indices);

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_NE(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 6);
  for (int i = 0; i < 6; i++) {
    ASSERT_EQ(vertices->indices()[i], indices[i]);
  }
}

TEST(DisplayListVertices, MakeWithNoOptionalData) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };

  std::shared_ptr<DlVertices> vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, nullptr, nullptr, 6, nullptr);

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, MakeWithIndicesButZeroIndexCount) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  std::shared_ptr<DlVertices> vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, nullptr, nullptr, 0, indices);

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, MakeWithIndicesButNegativeIndexCount) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  std::shared_ptr<DlVertices> vertices = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, nullptr, nullptr, -5, indices);

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

using Builder = DlVertices::Builder;

TEST(DisplayListVertices, BuilderFlags) {
  Builder::Flags flags;
  EXPECT_FALSE(flags.has_texture_coordinates);
  EXPECT_FALSE(flags.has_colors);

  flags |= Builder::kHasTextureCoordinates;
  EXPECT_TRUE(flags.has_texture_coordinates);
  EXPECT_FALSE(flags.has_colors);

  flags |= Builder::kHasColors;
  EXPECT_TRUE(flags.has_texture_coordinates);
  EXPECT_TRUE(flags.has_colors);

  flags = Builder::Flags();
  EXPECT_FALSE(flags.has_texture_coordinates);
  EXPECT_FALSE(flags.has_colors);

  flags |= Builder::kHasColors;
  EXPECT_FALSE(flags.has_texture_coordinates);
  EXPECT_TRUE(flags.has_colors);

  flags |= Builder::kHasTextureCoordinates;
  EXPECT_TRUE(flags.has_texture_coordinates);
  EXPECT_TRUE(flags.has_colors);

  EXPECT_FALSE(Builder::kNone.has_texture_coordinates);
  EXPECT_FALSE(Builder::kNone.has_colors);

  EXPECT_TRUE(Builder::kHasTextureCoordinates.has_texture_coordinates);
  EXPECT_FALSE(Builder::kHasTextureCoordinates.has_colors);

  EXPECT_FALSE(Builder::kHasColors.has_texture_coordinates);
  EXPECT_TRUE(Builder::kHasColors.has_colors);

  EXPECT_TRUE((Builder::kHasTextureCoordinates | Builder::kHasColors)  //
                  .has_texture_coordinates);
  EXPECT_TRUE((Builder::kHasTextureCoordinates | Builder::kHasColors)  //
                  .has_colors);
}

TEST(DisplayListVertices, BuildWithZeroAndNegativeVerticesAndIndices) {
  Builder builder1(DlVertexMode::kTriangles, 0, Builder::kNone, 0);
  EXPECT_TRUE(builder1.is_valid());
  std::shared_ptr<DlVertices> vertices1 = builder1.build();
  EXPECT_NE(vertices1, nullptr);
  EXPECT_EQ(vertices1->vertex_count(), 0);
  EXPECT_EQ(vertices1->vertex_data(), nullptr);
  EXPECT_EQ(vertices1->texture_coordinate_data(), nullptr);
  EXPECT_EQ(vertices1->colors(), nullptr);
  EXPECT_EQ(vertices1->index_count(), 0);
  EXPECT_EQ(vertices1->indices(), nullptr);

  Builder builder2(DlVertexMode::kTriangles, -1, Builder::kNone, -1);
  EXPECT_TRUE(builder2.is_valid());
  std::shared_ptr<DlVertices> vertices2 = builder2.build();
  EXPECT_NE(vertices2, nullptr);
  EXPECT_EQ(vertices2->vertex_count(), 0);
  EXPECT_EQ(vertices2->vertex_data(), nullptr);
  EXPECT_EQ(vertices2->texture_coordinate_data(), nullptr);
  EXPECT_EQ(vertices2->colors(), nullptr);
  EXPECT_EQ(vertices2->index_count(), 0);
  EXPECT_EQ(vertices2->indices(), nullptr);

  TestEquals(*vertices1, *vertices2);
}

TEST(DisplayListVertices, BuildWithTexAndColorAndIndices) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlPoint texture_coords[3] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
  };
  DlColor colors[3] = {
      DlColor::kRed(),
      DlColor::kCyan(),
      DlColor::kGreen(),
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  Builder builder(DlVertexMode::kTriangles, 3,  //
                  Builder::kHasTextureCoordinates | Builder::kHasColors, 6);
  builder.store_vertices(coords);
  builder.store_texture_coordinates(texture_coords);
  builder.store_colors(colors);
  builder.store_indices(indices);
  std::shared_ptr<DlVertices> vertices = builder.build();

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_NE(vertices->texture_coordinate_data(), nullptr);
  ASSERT_NE(vertices->colors(), nullptr);
  ASSERT_NE(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->texture_coordinate_data()[i], texture_coords[i]);
    ASSERT_EQ(vertices->colors()[i], colors[i]);
  }
  ASSERT_EQ(vertices->index_count(), 6);
  for (int i = 0; i < 6; i++) {
    ASSERT_EQ(vertices->indices()[i], indices[i]);
  }

  Builder builder2(DlVertexMode::kTriangles, 3,  //
                   Builder::kHasTextureCoordinates | Builder::kHasColors, 6);
  builder2.store_vertices(coords);
  builder2.store_texture_coordinates(texture_coords);
  builder2.store_colors(colors);
  builder2.store_indices(indices);
  std::shared_ptr<DlVertices> vertices2 = builder2.build();

  TestEquals(*vertices, *vertices2);

  std::shared_ptr<DlVertices> vertices3 = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, texture_coords, colors, 6, indices);

  TestEquals(*vertices, *vertices3);
}

TEST(DisplayListVertices, BuildWithTexAndColor) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlPoint texture_coords[3] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
  };
  DlColor colors[3] = {
      DlColor::kRed(),
      DlColor::kCyan(),
      DlColor::kGreen(),
  };

  Builder builder(DlVertexMode::kTriangles, 3,  //
                  Builder::kHasTextureCoordinates | Builder::kHasColors, 0);
  builder.store_vertices(coords);
  builder.store_texture_coordinates(texture_coords);
  builder.store_colors(colors);
  std::shared_ptr<DlVertices> vertices = builder.build();

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_NE(vertices->texture_coordinate_data(), nullptr);
  ASSERT_NE(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->texture_coordinate_data()[i], texture_coords[i]);
    ASSERT_EQ(vertices->colors()[i], colors[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, BuildWithTexAndIndices) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlPoint texture_coords[3] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  Builder builder(DlVertexMode::kTriangles, 3,  //
                  Builder::kHasTextureCoordinates, 6);
  builder.store_vertices(coords);
  builder.store_texture_coordinates(texture_coords);
  builder.store_indices(indices);
  std::shared_ptr<DlVertices> vertices = builder.build();

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_NE(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_NE(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->texture_coordinate_data()[i], texture_coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 6);
  for (int i = 0; i < 6; i++) {
    ASSERT_EQ(vertices->indices()[i], indices[i]);
  }
}

TEST(DisplayListVertices, BuildWithColorAndIndices) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  uint32_t colors[3] = {
      0xffff0000,
      0xff00ffff,
      0xff00ff00,
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  Builder builder(DlVertexMode::kTriangles, 3,  //
                  Builder::kHasColors, 6);
  builder.store_vertices(coords);
  builder.store_colors(colors);
  builder.store_indices(indices);
  std::shared_ptr<DlVertices> vertices = builder.build();

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_NE(vertices->colors(), nullptr);
  ASSERT_NE(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->colors()[i].argb(), colors[i]);
  }
  ASSERT_EQ(vertices->index_count(), 6);
  for (int i = 0; i < 6; i++) {
    ASSERT_EQ(vertices->indices()[i], indices[i]);
  }
}

TEST(DisplayListVertices, BuildWithTexUsingPoints) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlPoint texture_coords[3] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
  };

  Builder builder(DlVertexMode::kTriangles, 3,  //
                  Builder::kHasTextureCoordinates, 0);
  builder.store_vertices(coords);
  builder.store_texture_coordinates(texture_coords);
  std::shared_ptr<DlVertices> vertices = builder.build();

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_NE(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->texture_coordinate_data()[i], texture_coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, BuildWithTexUsingFloats) {
  float coords[6] = {
      2,  3,  //
      5,  6,  //
      15, 20,
  };
  float texture_coords[6] = {
      102, 103,  //
      105, 106,  //
      115, 120,
  };

  Builder builder(DlVertexMode::kTriangles, 3,  //
                  Builder::kHasTextureCoordinates, 0);
  builder.store_vertices(coords);
  builder.store_texture_coordinates(texture_coords);
  std::shared_ptr<DlVertices> vertices = builder.build();

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_NE(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i].x, coords[i * 2 + 0]);
    ASSERT_EQ(vertices->vertex_data()[i].y, coords[i * 2 + 1]);
    ASSERT_EQ(vertices->texture_coordinate_data()[i].x,
              texture_coords[i * 2 + 0]);
    ASSERT_EQ(vertices->texture_coordinate_data()[i].y,
              texture_coords[i * 2 + 1]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, BuildUsingFloatsSameAsPoints) {
  DlPoint coord_points[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlPoint texture_coord_points[3] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
  };

  float coord_floats[6] = {
      2,  3,  //
      5,  6,  //
      15, 20,
  };
  float texture_coord_floats[6] = {
      102, 103,  //
      105, 106,  //
      115, 120,
  };

  Builder builder_points(DlVertexMode::kTriangles, 3,  //
                         Builder::kHasTextureCoordinates, 0);
  builder_points.store_vertices(coord_points);
  builder_points.store_texture_coordinates(texture_coord_points);
  std::shared_ptr<DlVertices> vertices_points = builder_points.build();

  Builder builder_floats(DlVertexMode::kTriangles, 3,  //
                         Builder::kHasTextureCoordinates, 0);
  builder_floats.store_vertices(coord_floats);
  builder_floats.store_texture_coordinates(texture_coord_floats);
  std::shared_ptr<DlVertices> vertices_floats = builder_floats.build();

  TestEquals(*vertices_points, *vertices_floats);
}

TEST(DisplayListVertices, BuildWithColor) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  uint32_t colors[3] = {
      0xffff0000,
      0xff00ffff,
      0xff00ff00,
  };

  Builder builder(DlVertexMode::kTriangles, 3,  //
                  Builder::kHasColors, 0);
  builder.store_vertices(coords);
  builder.store_colors(colors);
  std::shared_ptr<DlVertices> vertices = builder.build();

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_NE(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
    ASSERT_EQ(vertices->colors()[i].argb(), colors[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, BuildWithIndices) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  Builder builder(DlVertexMode::kTriangles, 3, Builder::kNone, 6);
  builder.store_vertices(coords);
  builder.store_indices(indices);
  std::shared_ptr<DlVertices> vertices = builder.build();

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_NE(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 6);
  for (int i = 0; i < 6; i++) {
    ASSERT_EQ(vertices->indices()[i], indices[i]);
  }
}

TEST(DisplayListVertices, BuildWithNoOptionalData) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };

  Builder builder(DlVertexMode::kTriangles, 3, Builder::kNone, 0);
  builder.store_vertices(coords);
  std::shared_ptr<DlVertices> vertices = builder.build();

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, BuildWithNegativeIndexCount) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };

  Builder builder(DlVertexMode::kTriangles, 3, Builder::kNone, -5);
  builder.store_vertices(coords);
  std::shared_ptr<DlVertices> vertices = builder.build();

  ASSERT_NE(vertices, nullptr);
  ASSERT_NE(vertices->vertex_data(), nullptr);
  ASSERT_EQ(vertices->texture_coordinate_data(), nullptr);
  ASSERT_EQ(vertices->colors(), nullptr);
  ASSERT_EQ(vertices->indices(), nullptr);

  ASSERT_EQ(vertices->GetBounds(), DlRect::MakeLTRB(2, 3, 15, 20));
  ASSERT_EQ(vertices->mode(), DlVertexMode::kTriangles);
  ASSERT_EQ(vertices->vertex_count(), 3);
  for (int i = 0; i < 3; i++) {
    ASSERT_EQ(vertices->vertex_data()[i], coords[i]);
  }
  ASSERT_EQ(vertices->index_count(), 0);
}

TEST(DisplayListVertices, TestEquals) {
  DlPoint coords[3] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
  };
  DlPoint texture_coords[3] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
  };
  DlColor colors[3] = {
      DlColor::kRed(),
      DlColor::kCyan(),
      DlColor::kGreen(),
  };
  uint16_t indices[6] = {
      2, 1, 0,  //
      1, 2, 0,
  };

  std::shared_ptr<DlVertices> vertices1 = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, texture_coords, colors, 6, indices);
  std::shared_ptr<DlVertices> vertices2 = DlVertices::Make(
      DlVertexMode::kTriangles, 3, coords, texture_coords, colors, 6, indices);
  TestEquals(*vertices1, *vertices2);
}

TEST(DisplayListVertices, TestNotEquals) {
  DlPoint coords[4] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
      DlPoint(53, 62),
  };
  DlPoint wrong_coords[4] = {
      DlPoint(2, 3),
      DlPoint(5, 6),
      DlPoint(15, 20),
      DlPoint(57, 62),
  };
  DlPoint texture_coords[4] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 120),
      DlPoint(153, 162),
  };
  DlPoint wrong_texture_coords[4] = {
      DlPoint(102, 103),
      DlPoint(105, 106),
      DlPoint(115, 121),
      DlPoint(153, 162),
  };
  DlColor colors[4] = {
      DlColor::kRed(),
      DlColor::kCyan(),
      DlColor::kGreen(),
      DlColor::kMagenta(),
  };
  DlColor wrong_colors[4] = {
      DlColor::kRed(),
      DlColor::kBlue(),
      DlColor::kGreen(),
      DlColor::kMagenta(),
  };
  uint16_t indices[9] = {
      2, 1, 0,  //
      1, 2, 0,  //
      1, 2, 3,
  };
  uint16_t wrong_indices[9] = {
      2, 1, 0,  //
      1, 2, 0,  //
      2, 3, 1,
  };

  std::shared_ptr<DlVertices> vertices1 = DlVertices::Make(
      DlVertexMode::kTriangles, 4, coords, texture_coords, colors, 9, indices);

  {
    std::shared_ptr<DlVertices> vertices2 =
        DlVertices::Make(DlVertexMode::kTriangleFan, 4, coords,  //
                         texture_coords, colors, 9, indices);
    TestNotEquals(*vertices1, *vertices2, "vertex mode differs");
  }
  {
    std::shared_ptr<DlVertices> vertices2 =
        DlVertices::Make(DlVertexMode::kTriangles, 3, coords,  //
                         texture_coords, colors, 9, indices);
    TestNotEquals(*vertices1, *vertices2, "vertex count differs");
  }
  {
    std::shared_ptr<DlVertices> vertices2 =
        DlVertices::Make(DlVertexMode::kTriangles, 4, wrong_coords,  //
                         texture_coords, colors, 9, indices);
    TestNotEquals(*vertices1, *vertices2, "vertex coordinates differ");
  }
  {
    std::shared_ptr<DlVertices> vertices2 =
        DlVertices::Make(DlVertexMode::kTriangles, 4, coords,  //
                         wrong_texture_coords, colors, 9, indices);
    TestNotEquals(*vertices1, *vertices2, "texture coordinates differ");
  }
  {
    std::shared_ptr<DlVertices> vertices2 =
        DlVertices::Make(DlVertexMode::kTriangles, 4, coords,  //
                         texture_coords, wrong_colors, 9, indices);
    TestNotEquals(*vertices1, *vertices2, "colors differ");
  }
  {
    std::shared_ptr<DlVertices> vertices2 =
        DlVertices::Make(DlVertexMode::kTriangles, 4, coords,  //
                         texture_coords, colors, 6, indices);
    TestNotEquals(*vertices1, *vertices2, "index count differs");
  }
  {
    std::shared_ptr<DlVertices> vertices2 =
        DlVertices::Make(DlVertexMode::kTriangles, 4, coords,  //
                         texture_coords, colors, 9, wrong_indices);
    TestNotEquals(*vertices1, *vertices2, "indices differ");
  }
}

}  // namespace testing
}  // namespace flutter

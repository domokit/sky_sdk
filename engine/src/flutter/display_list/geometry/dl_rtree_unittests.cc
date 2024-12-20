// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/geometry/dl_rtree.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

#ifndef NDEBUG
TEST(DisplayListRTree, NullRectListNonZeroCount) {
  EXPECT_DEATH_IF_SUPPORTED(new DlRTree(nullptr, 1), "rects != nullptr");
}

TEST(DisplayListRTree, NegativeCount) {
  EXPECT_DEATH_IF_SUPPORTED(new DlRTree(nullptr, -1), "N >= 0");
}

TEST(DisplayListRTree, NullSearchResultVector) {
  DlRTree tree(nullptr, 0);
  EXPECT_DEATH_IF_SUPPORTED(tree.search(DlRect::MakeLTRB(0, 0, 1, 1), nullptr),
                            "results != nullptr");
}
#endif

TEST(DisplayListRTree, NullRectListZeroCount) {
  DlRTree tree(nullptr, 0);
  EXPECT_EQ(tree.leaf_count(), 0);
  EXPECT_EQ(tree.node_count(), 0);
  std::vector<int> results;
  auto huge = DlRect::MakeLTRB(-1e6, -1e6, 1e6, 1e6);
  tree.search(huge, &results);
  EXPECT_EQ(results.size(), 0u);
  auto list = tree.searchAndConsolidateRects(huge);
  EXPECT_EQ(list.size(), 0u);
}

TEST(DisplayListRTree, ManySizes) {
  // A diagonal of non-overlapping 10x10 rectangles spaced 20
  // pixels apart.
  // Rect 1 goes from  0 to 10
  // Rect 2 goes from 20 to 30
  // etc. in both dimensions
  const int kMaxN = 250;
  DlRect rects[kMaxN + 1];
  int ids[kMaxN + 1];
  for (int i = 0; i <= kMaxN; i++) {
    rects[i] = DlRect::MakeXYWH(i * 20, i * 20, 10, 10);
    ids[i] = i + 42;
  }
  std::vector<int> results;
  for (int N = 0; N <= kMaxN; N++) {
    DlRTree tree(rects, N, ids);
    auto desc = "node count = " + std::to_string(N);
    EXPECT_EQ(tree.leaf_count(), N) << desc;
    EXPECT_GE(tree.node_count(), N) << desc;
    EXPECT_EQ(tree.id(-1), -1) << desc;
    EXPECT_EQ(tree.bounds(-1), DlRect()) << desc;
    EXPECT_EQ(tree.id(N), -1) << desc;
    EXPECT_EQ(tree.bounds(N), DlRect()) << desc;
    results.clear();
    tree.search(DlRect(), &results);
    EXPECT_EQ(results.size(), 0u) << desc;
    results.clear();
    tree.search(DlRect::MakeLTRB(2, 2, 8, 8), &results);
    if (N == 0) {
      EXPECT_EQ(results.size(), 0u) << desc;
    } else {
      EXPECT_EQ(results.size(), 1u) << desc;
      EXPECT_EQ(results[0], 0) << desc;
      EXPECT_EQ(tree.id(results[0]), ids[0]) << desc;
      EXPECT_EQ(tree.bounds(results[0]), rects[0]) << desc;
      for (int i = 1; i < N; i++) {
        results.clear();
        auto query = DlRect::MakeXYWH(i * 20 + 2, i * 20 + 2, 6, 6);
        tree.search(query, &results);
        EXPECT_EQ(results.size(), 1u) << desc;
        EXPECT_EQ(results[0], i) << desc;
        EXPECT_EQ(tree.id(results[0]), ids[i]) << desc;
        EXPECT_EQ(tree.bounds(results[0]), rects[i]) << desc;
        auto list = tree.searchAndConsolidateRects(query);
        EXPECT_EQ(list.size(), 1u);
        EXPECT_EQ(list.front(), rects[i]);
      }
    }
  }
}

TEST(DisplayListRTree, HugeSize) {
  // A diagonal of non-overlapping 10x10 rectangles spaced 20
  // pixels apart.
  // Rect 1 goes from  0 to 10
  // Rect 2 goes from 20 to 30
  // etc. in both dimensions
  const int N = 10000;
  DlRect rects[N];
  int ids[N];
  for (int i = 0; i < N; i++) {
    rects[i] = DlRect::MakeXYWH(i * 20, i * 20, 10, 10);
    ids[i] = i + 42;
  }
  DlRTree tree(rects, N, ids);
  EXPECT_EQ(tree.leaf_count(), N);
  EXPECT_GE(tree.node_count(), N);
  EXPECT_EQ(tree.id(-1), -1);
  EXPECT_EQ(tree.bounds(-1), DlRect());
  EXPECT_EQ(tree.id(N), -1);
  EXPECT_EQ(tree.bounds(N), DlRect());
  std::vector<int> results;
  tree.search(DlRect(), &results);
  EXPECT_EQ(results.size(), 0u);
  for (int i = 0; i < N; i++) {
    results.clear();
    tree.search(DlRect::MakeXYWH(i * 20 + 2, i * 20 + 2, 6, 6), &results);
    EXPECT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0], i);
    EXPECT_EQ(tree.id(results[0]), ids[i]);
    EXPECT_EQ(tree.bounds(results[0]), rects[i]);
  }
}

TEST(DisplayListRTree, Grid) {
  // Non-overlapping 10 x 10 rectangles starting at 5, 5 with
  // 10 pixels between them.
  // Rect 1 goes from  5 to 15
  // Rect 2 goes from 25 to 35
  // etc. in both dimensions
  const int ROWS = 10;
  const int COLS = 10;
  const int N = ROWS * COLS;
  DlRect rects[N];
  int ids[N];
  for (int r = 0; r < ROWS; r++) {
    int y = r * 20 + 5;
    for (int c = 0; c < COLS; c++) {
      int x = c * 20 + 5;
      int i = r * COLS + c;
      rects[i] = DlRect::MakeXYWH(x, y, 10, 10);
      ids[i] = i + 42;
    }
  }
  DlRTree tree(rects, N, ids);
  EXPECT_EQ(tree.leaf_count(), N);
  EXPECT_GE(tree.node_count(), N);
  EXPECT_EQ(tree.id(-1), -1);
  EXPECT_EQ(tree.bounds(-1), DlRect());
  EXPECT_EQ(tree.id(N), -1);
  EXPECT_EQ(tree.bounds(N), DlRect());
  std::vector<int> results;
  tree.search(DlRect(), &results);
  EXPECT_EQ(results.size(), 0u);
  // Testing eqch rect for a single hit
  for (int r = 0; r < ROWS; r++) {
    int y = r * 20 + 5;
    for (int c = 0; c < COLS; c++) {
      int x = c * 20 + 5;
      int i = r * COLS + c;
      auto desc =
          "row " + std::to_string(r + 1) + ", col " + std::to_string(c + 1);
      results.clear();
      auto query = DlRect::MakeXYWH(x + 2, y + 2, 6, 6);
      tree.search(query, &results);
      EXPECT_EQ(results.size(), 1u) << desc;
      EXPECT_EQ(results[0], i) << desc;
      EXPECT_EQ(tree.id(results[0]), ids[i]) << desc;
      EXPECT_EQ(tree.bounds(results[0]), rects[i]) << desc;
      auto list = tree.searchAndConsolidateRects(query);
      EXPECT_EQ(list.size(), 1u);
      EXPECT_EQ(list.front(), rects[i]);
    }
  }
  // Testing inside each gap for no hits
  for (int r = 1; r < ROWS; r++) {
    int y = r * 20 + 5;
    for (int c = 1; c < COLS; c++) {
      int x = c * 20 + 5;
      auto desc =
          "row " + std::to_string(r + 1) + ", col " + std::to_string(c + 1);
      results.clear();
      auto query = DlRect::MakeXYWH(x - 8, y - 8, 6, 6);
      tree.search(query, &results);
      EXPECT_EQ(results.size(), 0u) << desc;
      auto list = tree.searchAndConsolidateRects(query);
      EXPECT_EQ(list.size(), 0u) << desc;
    }
  }
  // Spanning each gap for a quad of hits
  for (int r = 1; r < ROWS; r++) {
    int y = r * 20 + 5;
    for (int c = 1; c < COLS; c++) {
      int x = c * 20 + 5;
      // We will hit this rect and the ones above/left of us
      int i = r * COLS + c;
      auto desc =
          "row " + std::to_string(r + 1) + ", col " + std::to_string(c + 1);
      results.clear();
      auto query = DlRect::MakeXYWH(x - 11, y - 11, 12, 12);
      tree.search(query, &results);
      EXPECT_EQ(results.size(), 4u) << desc;

      // First rect is above and to the left
      EXPECT_EQ(results[0], i - COLS - 1) << desc;
      EXPECT_EQ(tree.id(results[0]), ids[i - COLS - 1]) << desc;
      EXPECT_EQ(tree.bounds(results[0]), rects[i - COLS - 1]) << desc;

      // Second rect is above
      EXPECT_EQ(results[1], i - COLS) << desc;
      EXPECT_EQ(tree.id(results[1]), ids[i - COLS]) << desc;
      EXPECT_EQ(tree.bounds(results[1]), rects[i - COLS]) << desc;

      // Third rect is left
      EXPECT_EQ(results[2], i - 1) << desc;
      EXPECT_EQ(tree.id(results[2]), ids[i - 1]) << desc;
      EXPECT_EQ(tree.bounds(results[2]), rects[i - 1]) << desc;

      // Fourth rect is us
      EXPECT_EQ(results[3], i) << desc;
      EXPECT_EQ(tree.id(results[3]), ids[i]) << desc;
      EXPECT_EQ(tree.bounds(results[3]), rects[i]) << desc;

      auto list = tree.searchAndConsolidateRects(query);
      EXPECT_EQ(list.size(), 4u);
      list.remove(rects[i - COLS - 1]);
      list.remove(rects[i - COLS]);
      list.remove(rects[i - 1]);
      list.remove(rects[i]);
      EXPECT_EQ(list.size(), 0u);
    }
  }
}

TEST(DisplayListRTree, OverlappingRects) {
  // Rectangles are centered at coordinates 15, 35, and 55 and are 15 wide
  // This gives them 10 pixels of overlap with the rectangles on either
  // side of them and the 10 pixels around their center coordinate are
  // exclusive to themselves.
  // So, horizontally and vertically, they cover the following ranges:
  // First  row/col:  0 to 30
  // Second row/col: 20 to 50
  // Third  row/col: 40 to 70
  // Coords  0 to 20 are only the first row/col
  // Coords 20 to 30 are both first and second row/col
  // Coords 30 to 40 are only the second row/col
  // Coords 40 to 50 are both second and third row/col
  // Coords 50 to 70 are only the third row/col
  //
  // In either dimension:
  // 0------------------20--------30--------40--------50------------------70
  // |         rect1               |
  //                     |  1 & 2  |
  //                     |            rect2            |
  //                                         |  2 & 3  |
  //                                         |                rect3        |
  DlRect rects[9];
  for (int r = 0; r < 3; r++) {
    int y = 15 + 20 * r;
    for (int c = 0; c < 3; c++) {
      int x = 15 + 20 * c;
      rects[r * 3 + c] = DlRect::MakeLTRB(x - 15, y - 15, x + 15, y + 15);
    }
  }
  DlRTree tree(rects, 9);
  // Tiny rects only intersecting a single source rect
  for (int r = 0; r < 3; r++) {
    int y = 15 + 20 * r;
    for (int c = 0; c < 3; c++) {
      int x = 15 + 20 * c;
      auto query = DlRect::MakeLTRB(x - 1, y - 1, x + 1, y + 1);
      auto list = tree.searchAndConsolidateRects(query);
      EXPECT_EQ(list.size(), 1u);
      EXPECT_EQ(list.front(), rects[r * 3 + c]);
    }
  }
  // Wide rects intersecting 3 source rects horizontally
  for (int r = 0; r < 3; r++) {
    int c = 1;
    int y = 15 + 20 * r;
    int x = 15 + 20 * c;
    auto query = DlRect::MakeLTRB(x - 6, y - 1, x + 6, y + 1);
    auto list = tree.searchAndConsolidateRects(query);
    EXPECT_EQ(list.size(), 1u);
    EXPECT_EQ(list.front(), DlRect::MakeLTRB(x - 35, y - 15, x + 35, y + 15));
  }
  // Tall rects intersecting 3 source rects vertically
  for (int c = 0; c < 3; c++) {
    int r = 1;
    int x = 15 + 20 * c;
    int y = 15 + 20 * r;
    auto query = DlRect::MakeLTRB(x - 1, y - 6, x + 1, y + 6);
    auto list = tree.searchAndConsolidateRects(query);
    EXPECT_EQ(list.size(), 1u);
    EXPECT_EQ(list.front(), DlRect::MakeLTRB(x - 15, y - 35, x + 15, y + 35));
  }
  // Finally intersecting all 9 rects
  auto query = DlRect::MakeLTRB(35 - 6, 35 - 6, 35 + 6, 35 + 6);
  auto list = tree.searchAndConsolidateRects(query);
  EXPECT_EQ(list.size(), 1u);
  EXPECT_EQ(list.front(), DlRect::MakeLTRB(0, 0, 70, 70));
}

TEST(DisplayListRTree, Region) {
  DlRect rect[9];
  for (int i = 0; i < 9; i++) {
    rect[i] = DlRect::MakeXYWH(i * 10, i * 10, 20, 20);
  }
  DlRTree rtree(rect, 9);
  const auto& region = rtree.region();
  auto rects = region.getRects(true);
  std::vector<DlIRect> expected_rects{
      DlIRect::MakeLTRB(0, 0, 20, 10),    DlIRect::MakeLTRB(0, 10, 30, 20),
      DlIRect::MakeLTRB(10, 20, 40, 30),  DlIRect::MakeLTRB(20, 30, 50, 40),
      DlIRect::MakeLTRB(30, 40, 60, 50),  DlIRect::MakeLTRB(40, 50, 70, 60),
      DlIRect::MakeLTRB(50, 60, 80, 70),  DlIRect::MakeLTRB(60, 70, 90, 80),
      DlIRect::MakeLTRB(70, 80, 100, 90), DlIRect::MakeLTRB(80, 90, 100, 100),
  };
  EXPECT_EQ(rects.size(), expected_rects.size());
}

}  // namespace testing
}  // namespace flutter

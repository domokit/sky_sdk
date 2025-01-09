
// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/windowing.h"

#include <array>

#include "flutter/fml/logging.h"
#include "gtest/gtest.h"

namespace flutter {

namespace {

using Positioner = WindowPositioner;
using Anchor = Positioner::Anchor;
using Constraint = Positioner::ConstraintAdjustment;
using Rectangle = WindowRectangle;
using Point = WindowPoint;
using Size = WindowSize;

struct WindowPlacementTest
    : ::testing::TestWithParam<std::tuple<Anchor, Anchor>> {
  struct ClientAnchorsToParentConfig {
    Rectangle const display_area = {{0, 0}, {800, 600}};
    Size const parent_size = {400, 300};
    Size const child_size = {100, 50};
    Point const parent_position = {
        (display_area.size.width - parent_size.width) / 2,
        (display_area.size.height - parent_size.height) / 2};
  } client_anchors_to_parent_config;

  Rectangle const display_area = {{0, 0}, {640, 480}};
  Size const parent_size = {600, 400};
  Size const child_size = {300, 300};
  Rectangle const rectangle_away_from_rhs = {{20, 20}, {20, 20}};
  Rectangle const rectangle_near_rhs = {{590, 20}, {10, 20}};
  Rectangle const rectangle_away_from_bottom = {{20, 20}, {20, 20}};
  Rectangle const rectangle_near_bottom = {{20, 380}, {20, 20}};
  Rectangle const rectangle_near_both_sides = {{0, 20}, {600, 20}};
  Rectangle const rectangle_near_both_sides_and_bottom = {{0, 380}, {600, 20}};
  Rectangle const rectangle_near_all_sides = {{0, 20}, {600, 380}};
  Rectangle const rectangle_near_both_bottom_right = {{400, 380}, {200, 20}};
  Point const parent_position = {
      (display_area.size.width - parent_size.width) / 2,
      (display_area.size.height - parent_size.height) / 2};

  Positioner positioner;

  Rectangle anchor_rect() {
    Rectangle rectangle{positioner.anchor_rect.value()};
    return {rectangle.top_left + parent_position, rectangle.size};
  }

  Rectangle parent_rect() { return {parent_position, parent_size}; }

  Point on_top_edge() {
    return anchor_rect().top_left - Point{0, child_size.height};
  }

  Point on_left_edge() {
    return anchor_rect().top_left - Point{child_size.width, 0};
  }
};

std::vector<std::tuple<Anchor, Anchor>> all_anchor_combinations() {
  std::array const all_anchors = {
      Anchor::top_left,    Anchor::top,    Anchor::top_right,
      Anchor::left,        Anchor::center, Anchor::right,
      Anchor::bottom_left, Anchor::bottom, Anchor::bottom_right,
  };
  std::vector<std::tuple<Anchor, Anchor>> combinations;
  combinations.reserve(all_anchors.size() * all_anchors.size());

  for (Anchor const parent_anchor : all_anchors) {
    for (Anchor const child_anchor : all_anchors) {
      combinations.push_back(std::make_tuple(parent_anchor, child_anchor));
    }
  }
  return combinations;
}

}  // namespace

TEST_F(WindowPlacementTest, ClientAnchorsToParentGivenRectAnchorRightOfParent) {
  Rectangle const& display_area = client_anchors_to_parent_config.display_area;
  Size const& parent_size = client_anchors_to_parent_config.parent_size;
  Size const& child_size = client_anchors_to_parent_config.child_size;
  Point const& parent_position =
      client_anchors_to_parent_config.parent_position;

  int const rect_size = 10;
  Rectangle const overlapping_right = {
      parent_position +
          Point{parent_size.width - rect_size / 2, parent_size.height / 2},
      {rect_size, rect_size}};

  Positioner const positioner = {
      .anchor_rect = overlapping_right,
      .parent_anchor = Anchor::top_right,
      .child_anchor = Anchor::top_left,
      .constraint_adjustment =
          static_cast<Constraint>(static_cast<int>(Constraint::slide_y) |
                                  static_cast<int>(Constraint::resize_x))};

  WindowRectangle const child_rect =
      PlaceWindow(positioner, child_size, positioner.anchor_rect.value(),
                  {parent_position, parent_size}, display_area);

  Point const expected_position =
      parent_position + Point{parent_size.width, parent_size.height / 2};

  EXPECT_EQ(child_rect.top_left, expected_position);
  EXPECT_EQ(child_rect.size, child_size);
}

TEST_F(WindowPlacementTest, ClientAnchorsToParentGivenRectAnchorAboveParent) {
  Rectangle const& display_area = client_anchors_to_parent_config.display_area;
  Size const& parent_size = client_anchors_to_parent_config.parent_size;
  Size const& child_size = client_anchors_to_parent_config.child_size;
  Point const& parent_position =
      client_anchors_to_parent_config.parent_position;

  int const rect_size = 10;
  Rectangle const overlapping_above = {
      parent_position + Point{parent_size.width / 2, -rect_size / 2},
      {rect_size, rect_size}};

  Positioner const positioner = {.anchor_rect = overlapping_above,
                                 .parent_anchor = Anchor::top_right,
                                 .child_anchor = Anchor::bottom_right,
                                 .constraint_adjustment = Constraint::slide_x};

  WindowRectangle const child_rect =
      PlaceWindow(positioner, child_size, positioner.anchor_rect.value(),
                  {parent_position, parent_size}, display_area);

  Point const expected_position = parent_position +
                                  Point{parent_size.width / 2 + rect_size, 0} -
                                  static_cast<Point>(child_size);

  EXPECT_EQ(child_rect.top_left, expected_position);
  EXPECT_EQ(child_rect.size, child_size);
}

TEST_F(WindowPlacementTest, ClientAnchorsToParentGivenOffsetRightOfParent) {
  Rectangle const& display_area = client_anchors_to_parent_config.display_area;
  Size const& parent_size = client_anchors_to_parent_config.parent_size;
  Size const& child_size = client_anchors_to_parent_config.child_size;
  Point const& parent_position =
      client_anchors_to_parent_config.parent_position;

  int const rect_size = 10;
  Rectangle const mid_right = {
      parent_position +
          Point{parent_size.width - rect_size, parent_size.height / 2},
      {rect_size, rect_size}};

  Positioner const positioner = {
      .anchor_rect = mid_right,
      .parent_anchor = Anchor::top_right,
      .child_anchor = Anchor::top_left,
      .offset = Point{rect_size, 0},
      .constraint_adjustment =
          static_cast<Constraint>(static_cast<int>(Constraint::slide_y) |
                                  static_cast<int>(Constraint::resize_x))};

  WindowRectangle const child_rect =
      PlaceWindow(positioner, child_size, positioner.anchor_rect.value(),
                  {parent_position, parent_size}, display_area);

  Point const expected_position =
      parent_position + Point{parent_size.width, parent_size.height / 2};

  EXPECT_EQ(child_rect.top_left, expected_position);
  EXPECT_EQ(child_rect.size, child_size);
}

TEST_F(WindowPlacementTest, ClientAnchorsToParentGivenOffsetAboveParent) {
  Rectangle const& display_area = client_anchors_to_parent_config.display_area;
  Size const& parent_size = client_anchors_to_parent_config.parent_size;
  Size const& child_size = client_anchors_to_parent_config.child_size;
  Point const& parent_position =
      client_anchors_to_parent_config.parent_position;

  int const rect_size = 10;
  Rectangle const mid_top = {parent_position + Point{parent_size.width / 2, 0},
                             {rect_size, rect_size}};

  Positioner const positioner = {.anchor_rect = mid_top,
                                 .parent_anchor = Anchor::top_right,
                                 .child_anchor = Anchor::bottom_right,
                                 .offset = Point{0, -rect_size},
                                 .constraint_adjustment = Constraint::slide_x};

  WindowRectangle const child_rect =
      PlaceWindow(positioner, child_size, positioner.anchor_rect.value(),
                  {parent_position, parent_size}, display_area);

  Point const expected_position = parent_position +
                                  Point{parent_size.width / 2 + rect_size, 0} -
                                  static_cast<Point>(child_size);

  EXPECT_EQ(child_rect.top_left, expected_position);
  EXPECT_EQ(child_rect.size, child_size);
}

TEST_F(WindowPlacementTest,
       ClientAnchorsToParentGivenRectAndOffsetBelowLeftParent) {
  Rectangle const& display_area = client_anchors_to_parent_config.display_area;
  Size const& parent_size = client_anchors_to_parent_config.parent_size;
  Size const& child_size = client_anchors_to_parent_config.child_size;
  Point const& parent_position =
      client_anchors_to_parent_config.parent_position;

  int const rect_size = 10;
  Rectangle const below_left = {
      parent_position + Point{-rect_size, parent_size.height},
      {rect_size, rect_size}};

  Positioner const positioner = {
      .anchor_rect = below_left,
      .parent_anchor = Anchor::bottom_left,
      .child_anchor = Anchor::top_right,
      .offset = Point{-rect_size, rect_size},
      .constraint_adjustment = Constraint::resize_any};

  WindowRectangle const child_rect =
      PlaceWindow(positioner, child_size, positioner.anchor_rect.value(),
                  {parent_position, parent_size}, display_area);

  Point const expected_position = parent_position +
                                  Point{0, parent_size.height} -
                                  Point{child_size.width, 0};

  EXPECT_EQ(child_rect.top_left, expected_position);
  EXPECT_EQ(child_rect.size, child_size);
}

TEST_P(WindowPlacementTest, CanAttachByEveryAnchorGivenNoConstraintAdjustment) {
  positioner.anchor_rect = Rectangle{{100, 50}, {20, 20}};
  positioner.constraint_adjustment = Constraint{};
  std::tie(positioner.parent_anchor, positioner.child_anchor) = GetParam();

  auto const position_of = [](Anchor anchor, Rectangle rectangle) -> Point {
    switch (anchor) {
      case Anchor::top_left:
        return rectangle.top_left;
      case Anchor::top:
        return rectangle.top_left + Point{rectangle.size.width / 2, 0};
      case Anchor::top_right:
        return rectangle.top_left + Point{rectangle.size.width, 0};
      case Anchor::left:
        return rectangle.top_left + Point{0, rectangle.size.height / 2};
      case Anchor::center:
        return rectangle.top_left +
               Point{rectangle.size.width / 2, rectangle.size.height / 2};
      case Anchor::right:
        return rectangle.top_left +
               Point{rectangle.size.width, rectangle.size.height / 2};
      case Anchor::bottom_left:
        return rectangle.top_left + Point{0, rectangle.size.height};
      case Anchor::bottom:
        return rectangle.top_left +
               Point{rectangle.size.width / 2, rectangle.size.height};
      case Anchor::bottom_right:
        return rectangle.top_left + static_cast<Point>(rectangle.size);
      default:
        FML_UNREACHABLE();
    }
  };

  Point const anchor_position =
      position_of(positioner.parent_anchor, anchor_rect());

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(position_of(positioner.child_anchor, child_rect), anchor_position);
}

INSTANTIATE_TEST_SUITE_P(AnchorCombinations,
                         WindowPlacementTest,
                         ::testing::ValuesIn(all_anchor_combinations()));

TEST_F(WindowPlacementTest,
       PlacementIsFlippedGivenAnchorRectNearRightSideAndOffset) {
  int const x_offset = 42;
  int const y_offset = 13;

  positioner.anchor_rect = rectangle_near_rhs;
  positioner.constraint_adjustment = Constraint::flip_x;
  positioner.offset = Point{x_offset, y_offset};
  positioner.child_anchor = Anchor::top_left;
  positioner.parent_anchor = Anchor::top_right;

  Point const expected_position =
      on_left_edge() + Point{-1 * x_offset, y_offset};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
}

TEST_F(WindowPlacementTest,
       PlacementIsFlippedGivenAnchorRectNearBottomAndOffset) {
  int const x_offset = 42;
  int const y_offset = 13;

  positioner.anchor_rect = rectangle_near_bottom;
  positioner.constraint_adjustment = Constraint::flip_y;
  positioner.offset = Point{x_offset, y_offset};
  positioner.child_anchor = Anchor::top_left;
  positioner.parent_anchor = Anchor::bottom_left;

  Point const expected_position =
      on_top_edge() + Point{x_offset, -1 * y_offset};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
}

TEST_F(WindowPlacementTest,
       PlacementIsFlippedBothWaysGivenAnchorRectNearBottomRightAndOffset) {
  int const x_offset = 42;
  int const y_offset = 13;

  positioner.anchor_rect = rectangle_near_both_bottom_right;
  positioner.constraint_adjustment = Constraint::flip_any;
  positioner.offset = Point{x_offset, y_offset};
  positioner.child_anchor = Anchor::top_left;
  positioner.parent_anchor = Anchor::bottom_right;

  Point const expected_position = anchor_rect().top_left -
                                  static_cast<Point>(child_size) -
                                  Point{x_offset, y_offset};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
}

TEST_F(WindowPlacementTest, PlacementCanSlideInXGivenAnchorRectNearRightSide) {
  positioner.anchor_rect = rectangle_near_rhs;
  positioner.constraint_adjustment = Constraint::slide_x;
  positioner.child_anchor = Anchor::top_left;
  positioner.parent_anchor = Anchor::top_right;

  Point const expected_position = {
      (display_area.top_left.x + display_area.size.width) - child_size.width,
      anchor_rect().top_left.y};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
}

TEST_F(WindowPlacementTest, PlacementCanSlideInXGivenAnchorRectNearLeftSide) {
  Rectangle const rectangle_near_left_side = {{0, 20}, {20, 20}};

  positioner.anchor_rect = rectangle_near_left_side;
  positioner.constraint_adjustment = Constraint::slide_x;
  positioner.child_anchor = Anchor::top_right;
  positioner.parent_anchor = Anchor::top_left;

  Point const expected_position = {display_area.top_left.x,
                                   anchor_rect().top_left.y};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
}

TEST_F(WindowPlacementTest, PlacementCanSlideInYGivenAnchorRectNearBottom) {
  positioner.anchor_rect = rectangle_near_bottom;
  positioner.constraint_adjustment = Constraint::slide_y;
  positioner.child_anchor = Anchor::top_left;
  positioner.parent_anchor = Anchor::bottom_left;

  Point const expected_position = {
      anchor_rect().top_left.x,
      (display_area.top_left.y + display_area.size.height) - child_size.height};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
}

TEST_F(WindowPlacementTest, PlacementCanSlideInYGivenAnchorRectNearTop) {
  positioner.anchor_rect = rectangle_near_all_sides;
  positioner.constraint_adjustment = Constraint::slide_y;
  positioner.child_anchor = Anchor::bottom_left;
  positioner.parent_anchor = Anchor::top_left;

  Point const expected_position = {anchor_rect().top_left.x,
                                   display_area.top_left.y};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
}

TEST_F(WindowPlacementTest,
       PlacementCanSlideInXAndYGivenAnchorRectNearBottomRightAndOffset) {
  positioner.anchor_rect = rectangle_near_both_bottom_right;
  positioner.constraint_adjustment = Constraint::slide_any;
  positioner.child_anchor = Anchor::top_left;
  positioner.parent_anchor = Anchor::bottom_left;

  Point const expected_position = {
      (display_area.top_left + static_cast<Point>(display_area.size)) -
      static_cast<Point>(child_size)};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
}

TEST_F(WindowPlacementTest, PlacementCanResizeInXGivenAnchorRectNearRightSide) {
  positioner.anchor_rect = rectangle_near_rhs;
  positioner.constraint_adjustment = Constraint::resize_x;
  positioner.child_anchor = Anchor::top_left;
  positioner.parent_anchor = Anchor::top_right;

  Point const expected_position =
      anchor_rect().top_left + Point{anchor_rect().size.width, 0};
  Size const expected_size = {
      (display_area.top_left.x + display_area.size.width) -
          (anchor_rect().top_left.x + anchor_rect().size.width),
      child_size.height};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
  EXPECT_EQ(child_rect.size, expected_size);
}

TEST_F(WindowPlacementTest, PlacementCanResizeInXGivenAnchorRectNearLeftSide) {
  Rectangle const rectangle_near_left_side = {{0, 20}, {20, 20}};

  positioner.anchor_rect = rectangle_near_left_side;
  positioner.constraint_adjustment = Constraint::resize_x;
  positioner.child_anchor = Anchor::top_right;
  positioner.parent_anchor = Anchor::top_left;

  Point const expected_position = {display_area.top_left.x,
                                   anchor_rect().top_left.y};
  Size const expected_size = {
      anchor_rect().top_left.x - display_area.top_left.x, child_size.height};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
  EXPECT_EQ(child_rect.size, expected_size);
}

TEST_F(WindowPlacementTest, PlacementCanResizeInYGivenAnchorRectNearBottom) {
  positioner.anchor_rect = rectangle_near_bottom;
  positioner.constraint_adjustment = Constraint::resize_y;
  positioner.child_anchor = Anchor::top_left;
  positioner.parent_anchor = Anchor::bottom_left;

  Point const expected_position =
      anchor_rect().top_left + Point{0, anchor_rect().size.height};
  Size const expected_size = {
      child_size.width,
      (display_area.top_left.y + display_area.size.height) -
          (anchor_rect().top_left.y + anchor_rect().size.height)};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
  EXPECT_EQ(child_rect.size, expected_size);
}

TEST_F(WindowPlacementTest, PlacementCanResizeInYGivenAnchorRectNearTop) {
  positioner.anchor_rect = rectangle_near_all_sides;
  positioner.constraint_adjustment = Constraint::resize_y;
  positioner.child_anchor = Anchor::bottom_left;
  positioner.parent_anchor = Anchor::top_left;

  Point const expected_position = {anchor_rect().top_left.x,
                                   display_area.top_left.y};
  Size const expected_size = {
      child_size.width, anchor_rect().top_left.y - display_area.top_left.y};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
  EXPECT_EQ(child_rect.size, expected_size);
}

TEST_F(WindowPlacementTest,
       PlacementCanResizeInXAndYGivenAnchorRectNearBottomRightAndOffset) {
  positioner.anchor_rect = rectangle_near_both_bottom_right;
  positioner.constraint_adjustment = Constraint::resize_any;
  positioner.child_anchor = Anchor::top_left;
  positioner.parent_anchor = Anchor::bottom_right;

  Point const expected_position =
      anchor_rect().top_left + static_cast<Point>(anchor_rect().size);
  Size const expected_size = {
      (display_area.top_left.x + display_area.size.width) - expected_position.x,
      (display_area.top_left.y + display_area.size.height) -
          expected_position.y};

  WindowRectangle const child_rect = PlaceWindow(
      positioner, child_size, anchor_rect(), parent_rect(), display_area);

  EXPECT_EQ(child_rect.top_left, expected_position);
  EXPECT_EQ(child_rect.size, expected_size);
}

}  // namespace flutter

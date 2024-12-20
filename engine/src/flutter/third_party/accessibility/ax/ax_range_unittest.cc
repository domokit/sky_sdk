// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_range.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"

#include "ax_enums.h"
#include "ax_node.h"
#include "ax_node_data.h"
#include "ax_node_position.h"
#include "ax_tree.h"
#include "ax_tree_id.h"
#include "ax_tree_update.h"
#include "test_ax_node_helper.h"
#include "test_ax_tree_manager.h"

namespace ui {

using TestPositionInstance =
    std::unique_ptr<AXPosition<AXNodePosition, AXNode>>;
using TestPositionRange = AXRange<AXPosition<AXNodePosition, AXNode>>;

namespace {

bool ContainerEQ(std::vector<gfx::Rect> actual,
                 std::vector<gfx::Rect> expected) {
  for (size_t i = 0; i < actual.size(); i++) {
    if (actual[i] != expected[i])
      return false;
  }
  return true;
}

constexpr AXNode::AXID ROOT_ID = 1;
constexpr AXNode::AXID DIV1_ID = 2;
constexpr AXNode::AXID BUTTON_ID = 3;
constexpr AXNode::AXID DIV2_ID = 4;
constexpr AXNode::AXID CHECK_BOX1_ID = 5;
constexpr AXNode::AXID CHECK_BOX2_ID = 6;
constexpr AXNode::AXID TEXT_FIELD_ID = 7;
constexpr AXNode::AXID STATIC_TEXT1_ID = 8;
constexpr AXNode::AXID INLINE_BOX1_ID = 9;
constexpr AXNode::AXID LINE_BREAK1_ID = 10;
constexpr AXNode::AXID STATIC_TEXT2_ID = 11;
constexpr AXNode::AXID INLINE_BOX2_ID = 12;
constexpr AXNode::AXID LINE_BREAK2_ID = 13;
constexpr AXNode::AXID PARAGRAPH_ID = 14;
constexpr AXNode::AXID STATIC_TEXT3_ID = 15;
constexpr AXNode::AXID INLINE_BOX3_ID = 16;

class TestAXRangeScreenRectDelegate : public AXRangeRectDelegate {
 public:
  explicit TestAXRangeScreenRectDelegate(TestAXTreeManager* tree_manager)
      : tree_manager_(tree_manager) {}
  virtual ~TestAXRangeScreenRectDelegate() = default;
  TestAXRangeScreenRectDelegate(const TestAXRangeScreenRectDelegate& delegate) =
      delete;
  TestAXRangeScreenRectDelegate& operator=(
      const TestAXRangeScreenRectDelegate& delegate) = delete;

  gfx::Rect GetInnerTextRangeBoundsRect(
      AXTreeID tree_id,
      AXNode::AXID node_id,
      int start_offset,
      int end_offset,
      ui::AXClippingBehavior clipping_behavior,
      AXOffscreenResult* offscreen_result) override {
    if (tree_manager_->GetTreeID() != tree_id)
      return gfx::Rect();

    AXNode* node = tree_manager_->GetNodeFromTree(node_id);
    if (!node)
      return gfx::Rect();

    TestAXNodeHelper* wrapper =
        TestAXNodeHelper::GetOrCreate(tree_manager_->GetTree(), node);
    return wrapper->GetInnerTextRangeBoundsRect(
        start_offset, end_offset, AXCoordinateSystem::kScreenDIPs,
        clipping_behavior, offscreen_result);
  }

  gfx::Rect GetBoundsRect(AXTreeID tree_id,
                          AXNode::AXID node_id,
                          AXOffscreenResult* offscreen_result) override {
    if (tree_manager_->GetTreeID() != tree_id)
      return gfx::Rect();

    AXNode* node = tree_manager_->GetNodeFromTree(node_id);
    if (!node)
      return gfx::Rect();

    TestAXNodeHelper* wrapper =
        TestAXNodeHelper::GetOrCreate(tree_manager_->GetTree(), node);
    return wrapper->GetBoundsRect(AXCoordinateSystem::kScreenDIPs,
                                  AXClippingBehavior::kClipped,
                                  offscreen_result);
  }

 private:
  TestAXTreeManager* const tree_manager_;
};

class AXRangeTest : public testing::Test, public TestAXTreeManager {
 public:
  const std::u16string EMPTY = base::ASCIIToUTF16("");
  const std::u16string NEWLINE = base::ASCIIToUTF16("\n");
  const std::u16string BUTTON = base::ASCIIToUTF16("Button");
  const std::u16string LINE_1 = base::ASCIIToUTF16("Line 1");
  const std::u16string LINE_2 = base::ASCIIToUTF16("Line 2");
  const std::u16string TEXT_FIELD =
      LINE_1.substr().append(NEWLINE).append(LINE_2).append(NEWLINE);
  const std::u16string AFTER_LINE = base::ASCIIToUTF16("After");
  const std::u16string ALL_TEXT =
      BUTTON.substr().append(TEXT_FIELD).append(AFTER_LINE);

  AXRangeTest() = default;
  ~AXRangeTest() override = default;

 protected:
  void SetUp() override;

  AXNodeData root_;
  AXNodeData div1_;
  AXNodeData div2_;
  AXNodeData button_;
  AXNodeData check_box1_;
  AXNodeData check_box2_;
  AXNodeData text_field_;
  AXNodeData line_break1_;
  AXNodeData line_break2_;
  AXNodeData static_text1_;
  AXNodeData static_text2_;
  AXNodeData static_text3_;
  AXNodeData inline_box1_;
  AXNodeData inline_box2_;
  AXNodeData inline_box3_;
  AXNodeData paragraph_;

 private:
  BASE_DISALLOW_COPY_AND_ASSIGN(AXRangeTest);
};

void AXRangeTest::SetUp() {
  // Most tests use kSuppressCharacter behavior.
  g_ax_embedded_object_behavior = AXEmbeddedObjectBehavior::kSuppressCharacter;

  root_.id = ROOT_ID;
  div1_.id = DIV1_ID;
  div2_.id = DIV2_ID;
  button_.id = BUTTON_ID;
  check_box1_.id = CHECK_BOX1_ID;
  check_box2_.id = CHECK_BOX2_ID;
  text_field_.id = TEXT_FIELD_ID;
  line_break1_.id = LINE_BREAK1_ID;
  line_break2_.id = LINE_BREAK2_ID;
  static_text1_.id = STATIC_TEXT1_ID;
  static_text2_.id = STATIC_TEXT2_ID;
  static_text3_.id = STATIC_TEXT3_ID;
  inline_box1_.id = INLINE_BOX1_ID;
  inline_box2_.id = INLINE_BOX2_ID;
  inline_box3_.id = INLINE_BOX3_ID;
  paragraph_.id = PARAGRAPH_ID;

  root_.role = ax::mojom::Role::kDialog;
  root_.AddState(ax::mojom::State::kFocusable);
  root_.SetName(ALL_TEXT);
  root_.relative_bounds.bounds = gfx::RectF(0, 0, 800, 600);

  div1_.role = ax::mojom::Role::kGenericContainer;
  div1_.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
  div1_.child_ids.push_back(button_.id);
  div1_.child_ids.push_back(div2_.id);
  root_.child_ids.push_back(div1_.id);

  button_.role = ax::mojom::Role::kButton;
  button_.SetHasPopup(ax::mojom::HasPopup::kMenu);
  button_.SetName(BUTTON);
  button_.SetValue(BUTTON);
  button_.relative_bounds.bounds = gfx::RectF(20, 20, 100, 30);
  button_.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                          check_box1_.id);

  div2_.role = ax::mojom::Role::kGenericContainer;
  div2_.child_ids.push_back(check_box1_.id);
  div2_.child_ids.push_back(check_box2_.id);

  check_box1_.role = ax::mojom::Role::kCheckBox;
  check_box1_.SetCheckedState(ax::mojom::CheckedState::kTrue);
  check_box1_.SetName("Checkbox 1");
  check_box1_.relative_bounds.bounds = gfx::RectF(120, 20, 30, 30);
  check_box1_.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                              button_.id);
  check_box1_.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                              check_box2_.id);

  check_box2_.role = ax::mojom::Role::kCheckBox;
  check_box2_.SetCheckedState(ax::mojom::CheckedState::kTrue);
  check_box2_.SetName("Checkbox 2");
  check_box2_.relative_bounds.bounds = gfx::RectF(150, 20, 30, 30);
  check_box2_.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                              check_box1_.id);

  text_field_.role = ax::mojom::Role::kGroup;
  text_field_.AddState(ax::mojom::State::kEditable);
  text_field_.SetValue(TEXT_FIELD);
  text_field_.AddIntListAttribute(
      ax::mojom::IntListAttribute::kCachedLineStarts,
      std::vector<int32_t>{0, 7});
  text_field_.child_ids.push_back(static_text1_.id);
  text_field_.child_ids.push_back(line_break1_.id);
  text_field_.child_ids.push_back(static_text2_.id);
  text_field_.child_ids.push_back(line_break2_.id);
  root_.child_ids.push_back(text_field_.id);

  static_text1_.role = ax::mojom::Role::kStaticText;
  static_text1_.AddState(ax::mojom::State::kEditable);
  static_text1_.SetName(LINE_1);
  static_text1_.child_ids.push_back(inline_box1_.id);

  inline_box1_.role = ax::mojom::Role::kInlineTextBox;
  inline_box1_.AddState(ax::mojom::State::kEditable);
  inline_box1_.SetName(LINE_1);
  inline_box1_.relative_bounds.bounds = gfx::RectF(20, 50, 30, 30);
  std::vector<int32_t> character_offsets1;
  // The width of each character is 5px.
  character_offsets1.push_back(25);  // "L" {20, 50, 5x30}
  character_offsets1.push_back(30);  // "i" {25, 50, 5x30}
  character_offsets1.push_back(35);  // "n" {30, 50, 5x30}
  character_offsets1.push_back(40);  // "e" {35, 50, 5x30}
  character_offsets1.push_back(45);  // " " {40, 50, 5x30}
  character_offsets1.push_back(50);  // "1" {45, 50, 5x30}
  inline_box1_.AddIntListAttribute(
      ax::mojom::IntListAttribute::kCharacterOffsets, character_offsets1);
  inline_box1_.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                   std::vector<int32_t>{0, 5});
  inline_box1_.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                   std::vector<int32_t>{4, 6});
  inline_box1_.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                               line_break1_.id);

  line_break1_.role = ax::mojom::Role::kLineBreak;
  line_break1_.AddState(ax::mojom::State::kEditable);
  line_break1_.SetName(NEWLINE);
  line_break1_.relative_bounds.bounds = gfx::RectF(50, 50, 0, 30);
  line_break1_.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                               inline_box1_.id);

  static_text2_.role = ax::mojom::Role::kStaticText;
  static_text2_.AddState(ax::mojom::State::kEditable);
  static_text2_.SetName(LINE_2);
  static_text2_.child_ids.push_back(inline_box2_.id);

  inline_box2_.role = ax::mojom::Role::kInlineTextBox;
  inline_box2_.AddState(ax::mojom::State::kEditable);
  inline_box2_.SetName(LINE_2);
  inline_box2_.relative_bounds.bounds = gfx::RectF(20, 80, 42, 30);
  std::vector<int32_t> character_offsets2;
  // The width of each character is 7 px.
  character_offsets2.push_back(27);  // "L" {20, 80, 7x30}
  character_offsets2.push_back(34);  // "i" {27, 80, 7x30}
  character_offsets2.push_back(41);  // "n" {34, 80, 7x30}
  character_offsets2.push_back(48);  // "e" {41, 80, 7x30}
  character_offsets2.push_back(55);  // " " {48, 80, 7x30}
  character_offsets2.push_back(62);  // "2" {55, 80, 7x30}
  inline_box2_.AddIntListAttribute(
      ax::mojom::IntListAttribute::kCharacterOffsets, character_offsets2);
  inline_box2_.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                   std::vector<int32_t>{0, 5});
  inline_box2_.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                   std::vector<int32_t>{4, 6});
  inline_box2_.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                               line_break2_.id);

  line_break2_.role = ax::mojom::Role::kLineBreak;
  line_break2_.AddState(ax::mojom::State::kEditable);
  line_break2_.SetName(NEWLINE);
  line_break2_.relative_bounds.bounds = gfx::RectF(62, 80, 0, 30);
  line_break2_.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                               inline_box2_.id);

  paragraph_.role = ax::mojom::Role::kParagraph;
  paragraph_.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);
  paragraph_.child_ids.push_back(static_text3_.id);
  root_.child_ids.push_back(paragraph_.id);

  static_text3_.role = ax::mojom::Role::kStaticText;
  static_text3_.SetName(AFTER_LINE);
  static_text3_.child_ids.push_back(inline_box3_.id);

  inline_box3_.role = ax::mojom::Role::kInlineTextBox;
  inline_box3_.SetName(AFTER_LINE);
  inline_box3_.relative_bounds.bounds = gfx::RectF(20, 110, 50, 30);
  std::vector<int32_t> character_offsets3;
  // The width of each character is 10 px.
  character_offsets3.push_back(30);  // "A" {20, 110, 10x30}
  character_offsets3.push_back(40);  // "f" {30, 110, 10x30}
  character_offsets3.push_back(50);  // "t" {40, 110, 10x30}
  character_offsets3.push_back(60);  // "e" {50, 110, 10x30}
  character_offsets3.push_back(70);  // "r" {60, 110, 10x30}
  inline_box3_.AddIntListAttribute(
      ax::mojom::IntListAttribute::kCharacterOffsets, character_offsets3);
  inline_box3_.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                   std::vector<int32_t>{0});
  inline_box3_.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                   std::vector<int32_t>{5});

  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes = {
      root_,        div1_,        button_,       div2_,
      check_box1_,  check_box2_,  text_field_,   static_text1_,
      inline_box1_, line_break1_, static_text2_, inline_box2_,
      line_break2_, paragraph_,   static_text3_, inline_box3_};
  initial_state.has_tree_data = true;
  initial_state.tree_data.tree_id = AXTreeID::CreateNewAXTreeID();
  initial_state.tree_data.title = "Dialog title";

  SetTree(std::make_unique<AXTree>(initial_state));
}

}  // namespace

TEST_F(AXRangeTest, EqualityOperators) {
  TestPositionInstance null_position = AXNodePosition::CreateNullPosition();
  TestPositionInstance test_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance test_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break1_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance test_position3 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  // Invalid ranges (with at least one null endpoint).
  TestPositionRange null_position_and_nullptr(null_position->Clone(), nullptr);
  TestPositionRange nullptr_and_test_position(nullptr, test_position1->Clone());
  TestPositionRange test_position_and_null_position(test_position2->Clone(),
                                                    null_position->Clone());

  TestPositionRange test_positions_1_and_2(test_position1->Clone(),
                                           test_position2->Clone());
  TestPositionRange test_positions_2_and_1(test_position2->Clone(),
                                           test_position1->Clone());
  TestPositionRange test_positions_1_and_3(test_position1->Clone(),
                                           test_position3->Clone());
  TestPositionRange test_positions_2_and_3(test_position2->Clone(),
                                           test_position3->Clone());
  TestPositionRange test_positions_3_and_2(test_position3->Clone(),
                                           test_position2->Clone());

  EXPECT_EQ(null_position_and_nullptr, nullptr_and_test_position);
  EXPECT_EQ(nullptr_and_test_position, test_position_and_null_position);
  EXPECT_NE(null_position_and_nullptr, test_positions_2_and_1);
  EXPECT_NE(test_positions_2_and_1, test_position_and_null_position);
  EXPECT_EQ(test_positions_1_and_2, test_positions_1_and_2);
  EXPECT_NE(test_positions_2_and_1, test_positions_1_and_2);
  EXPECT_EQ(test_positions_3_and_2, test_positions_2_and_3);
  EXPECT_NE(test_positions_1_and_2, test_positions_2_and_3);
  EXPECT_EQ(test_positions_1_and_2, test_positions_1_and_3);
}

TEST_F(AXRangeTest, AsForwardRange) {
  TestPositionRange null_range(AXNodePosition::CreateNullPosition(),
                               AXNodePosition::CreateNullPosition());
  null_range = null_range.AsForwardRange();
  EXPECT_TRUE(null_range.IsNull());

  TestPositionInstance tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), button_.id, 0 /* child_index */);
  TestPositionInstance text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break1_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionRange tree_to_text_range(text_position1->Clone(),
                                       tree_position->Clone());
  tree_to_text_range = tree_to_text_range.AsForwardRange();
  EXPECT_EQ(*tree_position, *tree_to_text_range.anchor());
  EXPECT_EQ(*text_position1, *tree_to_text_range.focus());

  TestPositionRange text_to_text_range(text_position2->Clone(),
                                       text_position1->Clone());
  text_to_text_range = text_to_text_range.AsForwardRange();
  EXPECT_EQ(*text_position1, *text_to_text_range.anchor());
  EXPECT_EQ(*text_position2, *text_to_text_range.focus());
}

TEST_F(AXRangeTest, IsCollapsed) {
  TestPositionRange null_range(AXNodePosition::CreateNullPosition(),
                               AXNodePosition::CreateNullPosition());
  null_range = null_range.AsForwardRange();
  EXPECT_FALSE(null_range.IsCollapsed());

  TestPositionInstance tree_position1 = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 0 /* child_index */);
  // Since there are no children in inline_box1_, the following is essentially
  // an "after text" position which should not compare as equivalent to the
  // above tree position which is a "before text" position inside the text
  // field.
  TestPositionInstance tree_position2 = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, 0 /* child_index */);

  TestPositionInstance text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance text_position3 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionRange tree_to_null_range(tree_position1->Clone(),
                                       AXNodePosition::CreateNullPosition());
  EXPECT_TRUE(tree_to_null_range.IsNull());
  EXPECT_FALSE(tree_to_null_range.IsCollapsed());

  TestPositionRange null_to_text_range(AXNodePosition::CreateNullPosition(),
                                       text_position1->Clone());
  EXPECT_TRUE(null_to_text_range.IsNull());
  EXPECT_FALSE(null_to_text_range.IsCollapsed());

  TestPositionRange tree_to_tree_range(tree_position2->Clone(),
                                       tree_position1->Clone());
  EXPECT_TRUE(tree_to_tree_range.IsCollapsed());

  // A tree and a text position that essentially point to the same text offset
  // are equivalent, even if they are anchored to a different node.
  TestPositionRange tree_to_text_range(tree_position1->Clone(),
                                       text_position1->Clone());
  EXPECT_TRUE(tree_to_text_range.IsCollapsed());

  // The following positions are not equivalent since tree_position2 is an
  // "after text" position.
  tree_to_text_range =
      TestPositionRange(tree_position2->Clone(), text_position2->Clone());
  EXPECT_FALSE(tree_to_text_range.IsCollapsed());

  TestPositionRange text_to_text_range(text_position1->Clone(),
                                       text_position1->Clone());
  EXPECT_TRUE(text_to_text_range.IsCollapsed());

  // Two text positions that essentially point to the same text offset are
  // equivalent, even if they are anchored to a different node.
  text_to_text_range =
      TestPositionRange(text_position1->Clone(), text_position2->Clone());
  EXPECT_TRUE(text_to_text_range.IsCollapsed());

  text_to_text_range =
      TestPositionRange(text_position1->Clone(), text_position3->Clone());
  EXPECT_FALSE(text_to_text_range.IsCollapsed());
}

TEST_F(AXRangeTest, BeginAndEndIterators) {
  TestPositionInstance null_position = AXNodePosition::CreateNullPosition();
  TestPositionInstance test_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance test_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance test_position3 = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance test_position4 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionRange nullptr_and_null_position(nullptr, null_position->Clone());
  EXPECT_EQ(TestPositionRange::Iterator(), nullptr_and_null_position.begin());
  EXPECT_EQ(TestPositionRange::Iterator(), nullptr_and_null_position.end());

  TestPositionRange test_position1_and_nullptr(test_position1->Clone(),
                                               nullptr);
  EXPECT_EQ(TestPositionRange::Iterator(), test_position1_and_nullptr.begin());
  EXPECT_EQ(TestPositionRange::Iterator(), test_position1_and_nullptr.end());

  TestPositionRange null_position_and_test_position2(null_position->Clone(),
                                                     test_position2->Clone());
  EXPECT_EQ(TestPositionRange::Iterator(),
            null_position_and_test_position2.begin());
  EXPECT_EQ(TestPositionRange::Iterator(),
            null_position_and_test_position2.end());

  TestPositionRange test_position1_and_test_position2(test_position1->Clone(),
                                                      test_position2->Clone());
  EXPECT_NE(TestPositionRange::Iterator(test_position1->Clone(),
                                        test_position4->Clone()),
            test_position1_and_test_position2.begin());
  EXPECT_NE(TestPositionRange::Iterator(test_position1->Clone(),
                                        test_position3->Clone()),
            test_position1_and_test_position2.begin());
  EXPECT_EQ(TestPositionRange::Iterator(test_position1->Clone(),
                                        test_position2->Clone()),
            test_position1_and_test_position2.begin());
  EXPECT_EQ(TestPositionRange::Iterator(nullptr, test_position2->Clone()),
            test_position1_and_test_position2.end());

  TestPositionRange test_position3_and_test_position4(test_position3->Clone(),
                                                      test_position4->Clone());
  EXPECT_NE(TestPositionRange::Iterator(test_position1->Clone(),
                                        test_position4->Clone()),
            test_position3_and_test_position4.begin());
  EXPECT_NE(TestPositionRange::Iterator(test_position2->Clone(),
                                        test_position4->Clone()),
            test_position3_and_test_position4.begin());
  EXPECT_EQ(TestPositionRange::Iterator(test_position3->Clone(),
                                        test_position4->Clone()),
            test_position3_and_test_position4.begin());
  EXPECT_NE(TestPositionRange::Iterator(nullptr, test_position2->Clone()),
            test_position3_and_test_position4.end());
  EXPECT_NE(TestPositionRange::Iterator(nullptr, test_position3->Clone()),
            test_position3_and_test_position4.end());
  EXPECT_EQ(TestPositionRange::Iterator(nullptr, test_position4->Clone()),
            test_position3_and_test_position4.end());
}

TEST_F(AXRangeTest, LeafTextRangeIteration) {
  TestPositionInstance button_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance button_middle = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance button_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  // Since a check box is not visible to the text representation, it spans an
  // empty anchor whose start and end positions are the same.
  TestPositionInstance check_box1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance check_box2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance line1_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line1_middle = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line1_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance line_break1_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line_break1_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break1_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance line2_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line2_middle = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line2_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance line_break2_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line_break2_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break2_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance after_line_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box3_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance after_line_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box3_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  std::vector<TestPositionRange> expected_ranges;
  auto TestRangeIterator =
      [&expected_ranges](const TestPositionRange& test_range) {
        std::vector<TestPositionRange> actual_ranges;
        for (TestPositionRange leaf_text_range : test_range) {
          EXPECT_TRUE(leaf_text_range.IsLeafTextRange());
          actual_ranges.emplace_back(std::move(leaf_text_range));
        }

        EXPECT_EQ(expected_ranges.size(), actual_ranges.size());
        size_t element_count =
            std::min(expected_ranges.size(), actual_ranges.size());
        for (size_t i = 0; i < element_count; ++i) {
          EXPECT_EQ(expected_ranges[i], actual_ranges[i]);
          EXPECT_EQ(expected_ranges[i].anchor()->GetAnchor(),
                    actual_ranges[i].anchor()->GetAnchor());
        }
      };

  // Iterating over a null range; note that expected_ranges is empty.
  TestRangeIterator(TestPositionRange(nullptr, nullptr));

  TestPositionRange non_null_degenerate_range(check_box1->Clone(),
                                              check_box1->Clone());
  expected_ranges.clear();
  expected_ranges.emplace_back(check_box1->Clone(), check_box1->Clone());
  TestRangeIterator(non_null_degenerate_range);

  TestPositionRange empty_text_forward_range(button_end->Clone(),
                                             line1_start->Clone());
  TestPositionRange empty_text_backward_range(line1_start->Clone(),
                                              button_end->Clone());
  expected_ranges.clear();
  expected_ranges.emplace_back(button_end->Clone(), button_end->Clone());
  expected_ranges.emplace_back(check_box1->Clone(), check_box1->Clone());
  expected_ranges.emplace_back(check_box2->Clone(), check_box2->Clone());
  expected_ranges.emplace_back(line1_start->Clone(), line1_start->Clone());
  TestRangeIterator(empty_text_forward_range);
  TestRangeIterator(empty_text_backward_range);

  TestPositionRange entire_anchor_forward_range(button_start->Clone(),
                                                button_end->Clone());
  TestPositionRange entire_anchor_backward_range(button_end->Clone(),
                                                 button_start->Clone());
  expected_ranges.clear();
  expected_ranges.emplace_back(button_start->Clone(), button_end->Clone());
  TestRangeIterator(entire_anchor_forward_range);
  TestRangeIterator(entire_anchor_backward_range);

  TestPositionRange across_anchors_forward_range(button_middle->Clone(),
                                                 line1_middle->Clone());
  TestPositionRange across_anchors_backward_range(line1_middle->Clone(),
                                                  button_middle->Clone());
  expected_ranges.clear();
  expected_ranges.emplace_back(button_middle->Clone(), button_end->Clone());
  expected_ranges.emplace_back(check_box1->Clone(), check_box1->Clone());
  expected_ranges.emplace_back(check_box2->Clone(), check_box2->Clone());
  expected_ranges.emplace_back(line1_start->Clone(), line1_middle->Clone());
  TestRangeIterator(across_anchors_forward_range);
  TestRangeIterator(across_anchors_backward_range);

  TestPositionRange starting_at_end_position_forward_range(
      line1_end->Clone(), line2_middle->Clone());
  TestPositionRange starting_at_end_position_backward_range(
      line2_middle->Clone(), line1_end->Clone());
  expected_ranges.clear();
  expected_ranges.emplace_back(line1_end->Clone(), line1_end->Clone());
  expected_ranges.emplace_back(line_break1_start->Clone(),
                               line_break1_end->Clone());
  expected_ranges.emplace_back(line2_start->Clone(), line2_middle->Clone());
  TestRangeIterator(starting_at_end_position_forward_range);
  TestRangeIterator(starting_at_end_position_backward_range);

  TestPositionRange ending_at_start_position_forward_range(
      line1_middle->Clone(), line2_start->Clone());
  TestPositionRange ending_at_start_position_backward_range(
      line2_start->Clone(), line1_middle->Clone());
  expected_ranges.clear();
  expected_ranges.emplace_back(line1_middle->Clone(), line1_end->Clone());
  expected_ranges.emplace_back(line_break1_start->Clone(),
                               line_break1_end->Clone());
  expected_ranges.emplace_back(line2_start->Clone(), line2_start->Clone());
  TestRangeIterator(ending_at_start_position_forward_range);
  TestRangeIterator(ending_at_start_position_backward_range);

  TestPositionInstance range_start = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  TestPositionInstance range_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, ALL_TEXT.length() /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionRange entire_test_forward_range(range_start->Clone(),
                                              range_end->Clone());
  TestPositionRange entire_test_backward_range(range_end->Clone(),
                                               range_start->Clone());
  expected_ranges.clear();
  expected_ranges.emplace_back(button_start->Clone(), button_end->Clone());
  expected_ranges.emplace_back(check_box1->Clone(), check_box1->Clone());
  expected_ranges.emplace_back(check_box2->Clone(), check_box2->Clone());
  expected_ranges.emplace_back(line1_start->Clone(), line1_end->Clone());
  expected_ranges.emplace_back(line_break1_start->Clone(),
                               line_break1_end->Clone());
  expected_ranges.emplace_back(line2_start->Clone(), line2_end->Clone());
  expected_ranges.emplace_back(line_break2_start->Clone(),
                               line_break2_end->Clone());
  expected_ranges.emplace_back(after_line_start->Clone(),
                               after_line_end->Clone());
  TestRangeIterator(entire_test_forward_range);
  TestRangeIterator(entire_test_backward_range);
}

TEST_F(AXRangeTest, GetTextWithWholeObjects) {
  // Create a range starting from the button object and ending at the last
  // character of the root, i.e. at the last character of the second line in the
  // text field.
  TestPositionInstance start = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  TestPositionInstance end = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, ALL_TEXT.length() /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(end->IsTextPosition());
  TestPositionRange forward_range(start->Clone(), end->Clone());
  EXPECT_EQ(ALL_TEXT, forward_range.GetText());
  TestPositionRange backward_range(std::move(end), std::move(start));
  EXPECT_EQ(ALL_TEXT, backward_range.GetText());

  // Button
  start = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(start->IsTextPosition());
  end = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, BUTTON.length() /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(end->IsTextPosition());
  TestPositionRange button_range(start->Clone(), end->Clone());
  EXPECT_EQ(BUTTON, button_range.GetText());
  TestPositionRange button_range_backward(std::move(end), std::move(start));
  EXPECT_EQ(BUTTON, button_range_backward.GetText());

  // text_field_
  start = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  end = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, TEXT_FIELD.length() /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(start->IsTextPosition());
  ASSERT_TRUE(end->IsTextPosition());
  TestPositionRange text_field_range(start->Clone(), end->Clone());
  EXPECT_EQ(TEXT_FIELD, text_field_range.GetText());
  TestPositionRange text_field_range_backward(std::move(end), std::move(start));
  EXPECT_EQ(TEXT_FIELD, text_field_range_backward.GetText());

  // static_text1_
  start = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(start->IsTextPosition());
  end = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1_.id, LINE_1.length() /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(end->IsTextPosition());
  TestPositionRange static_text1_range(start->Clone(), end->Clone());
  EXPECT_EQ(LINE_1, static_text1_range.GetText());
  TestPositionRange static_text1_range_backward(std::move(end),
                                                std::move(start));
  EXPECT_EQ(LINE_1, static_text1_range_backward.GetText());

  // static_text2_
  start = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(start->IsTextPosition());
  end = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, LINE_2.length() /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(end->IsTextPosition());
  TestPositionRange static_text2_range(start->Clone(), end->Clone());
  EXPECT_EQ(LINE_2, static_text2_range.GetText());
  TestPositionRange static_text2_range_backward(std::move(end),
                                                std::move(start));
  EXPECT_EQ(LINE_2, static_text2_range_backward.GetText());

  // static_text1_ to static_text2_
  std::u16string text_between_text1_start_and_text2_end =
      LINE_1.substr().append(NEWLINE).append(LINE_2);
  start = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(start->IsTextPosition());
  end = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, LINE_2.length() /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(end->IsTextPosition());
  TestPositionRange static_text_range(start->Clone(), end->Clone());
  EXPECT_EQ(text_between_text1_start_and_text2_end,
            static_text_range.GetText());
  TestPositionRange static_text_range_backward(std::move(end),
                                               std::move(start));
  EXPECT_EQ(text_between_text1_start_and_text2_end,
            static_text_range_backward.GetText());

  // root_ to static_text2_'s end
  std::u16string text_up_to_text2_end =
      BUTTON.substr(0).append(LINE_1).append(NEWLINE).append(LINE_2);
  start = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                             0 /* child_index */);
  end = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, LINE_2.length() /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(end->IsTextPosition());
  TestPositionRange root_to_static2_text_range(start->Clone(), end->Clone());
  EXPECT_EQ(text_up_to_text2_end, root_to_static2_text_range.GetText());
  TestPositionRange root_to_static2_text_range_backward(std::move(end),
                                                        std::move(start));
  EXPECT_EQ(text_up_to_text2_end,
            root_to_static2_text_range_backward.GetText());

  // root_ to static_text2_'s start
  std::u16string text_up_to_text2_start =
      BUTTON.substr(0).append(LINE_1).append(NEWLINE);
  start = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                             0 /* child_index */);
  end = AXNodePosition::CreateTreePosition(GetTreeID(), static_text2_.id,
                                           0 /* child_index */);
  TestPositionRange root_to_static2_tree_range(start->Clone(), end->Clone());
  EXPECT_EQ(text_up_to_text2_start, root_to_static2_tree_range.GetText());
  TestPositionRange root_to_static2_tree_range_backward(std::move(end),
                                                        std::move(start));
  EXPECT_EQ(text_up_to_text2_start,
            root_to_static2_tree_range_backward.GetText());
}

TEST_F(AXRangeTest, GetTextWithTextOffsets) {
  std::u16string most_text = BUTTON.substr(2).append(TEXT_FIELD.substr(0, 11));
  // Create a range starting from the button object and ending two characters
  // before the end of the root.
  TestPositionInstance start = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 2 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(start->IsTextPosition());
  TestPositionInstance end = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(end->IsTextPosition());
  TestPositionRange forward_range(start->Clone(), end->Clone());
  EXPECT_EQ(most_text, forward_range.GetText());
  TestPositionRange backward_range(std::move(end), std::move(start));
  EXPECT_EQ(most_text, backward_range.GetText());

  // root_ to static_text2_'s start with offsets
  std::u16string text_up_to_text2_tree_start =
      BUTTON.substr(0).append(TEXT_FIELD.substr(0, 10));
  start = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                             0 /* child_index */);
  end = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(end->IsTextPosition());
  TestPositionRange root_to_static2_tree_range(start->Clone(), end->Clone());
  EXPECT_EQ(text_up_to_text2_tree_start, root_to_static2_tree_range.GetText());
  TestPositionRange root_to_static2_tree_range_backward(std::move(end),
                                                        std::move(start));
  EXPECT_EQ(text_up_to_text2_tree_start,
            root_to_static2_tree_range_backward.GetText());
}

TEST_F(AXRangeTest, GetTextWithEmptyRanges) {
  // empty string with non-leaf tree position
  TestPositionInstance start = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  TestPositionRange non_leaf_tree_range(start->Clone(), start->Clone());
  EXPECT_EQ(EMPTY, non_leaf_tree_range.GetText());

  // empty string with leaf tree position
  start = AXNodePosition::CreateTreePosition(GetTreeID(), inline_box1_.id,
                                             0 /* child_index */);
  TestPositionRange leaf_empty_range(start->Clone(), start->Clone());
  EXPECT_EQ(EMPTY, leaf_empty_range.GetText());

  // empty string with leaf text position and no offset
  start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionRange leaf_text_no_offset(start->Clone(), start->Clone());
  EXPECT_EQ(EMPTY, leaf_text_no_offset.GetText());

  // empty string with leaf text position with offset
  start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionRange leaf_text_offset(start->Clone(), start->Clone());
  EXPECT_EQ(EMPTY, leaf_text_offset.GetText());

  // empty string with non-leaf text with no offset
  start = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionRange non_leaf_text_no_offset(start->Clone(), start->Clone());
  EXPECT_EQ(EMPTY, non_leaf_text_no_offset.GetText());

  // empty string with non-leaf text position with offset
  start = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionRange non_leaf_text_offset(start->Clone(), start->Clone());
  EXPECT_EQ(EMPTY, non_leaf_text_offset.GetText());

  // empty string with same position between two anchors, but different offsets
  TestPositionInstance after_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break1_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance before_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionRange same_position_different_anchors_forward(
      after_end->Clone(), before_start->Clone());
  EXPECT_EQ(EMPTY, same_position_different_anchors_forward.GetText());
  TestPositionRange same_position_different_anchors_backward(
      before_start->Clone(), after_end->Clone());
  EXPECT_EQ(EMPTY, same_position_different_anchors_backward.GetText());
}

TEST_F(AXRangeTest, GetTextAddingNewlineBetweenParagraphs) {
  TestPositionInstance button_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance button_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance line1_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line1_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance line2_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line2_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance after_line_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box3_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance after_line_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box3_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  auto TestGetTextForRange = [](TestPositionInstance range_start,
                                TestPositionInstance range_end,
                                const std::u16string& expected_text,
                                const size_t expected_appended_newlines_count) {
    TestPositionRange forward_test_range(range_start->Clone(),
                                         range_end->Clone());
    TestPositionRange backward_test_range(std::move(range_end),
                                          std::move(range_start));
    size_t appended_newlines_count = 0;
    EXPECT_EQ(expected_text, forward_test_range.GetText(
                                 AXTextConcatenationBehavior::kAsInnerText, -1,
                                 false, &appended_newlines_count));
    EXPECT_EQ(expected_appended_newlines_count, appended_newlines_count);
    EXPECT_EQ(expected_text, backward_test_range.GetText(
                                 AXTextConcatenationBehavior::kAsInnerText, -1,
                                 false, &appended_newlines_count));
    EXPECT_EQ(expected_appended_newlines_count, appended_newlines_count);
  };

  std::u16string button_start_to_line1_end =
      BUTTON.substr().append(NEWLINE).append(LINE_1);
  TestGetTextForRange(button_start->Clone(), line1_end->Clone(),
                      button_start_to_line1_end, 1);
  std::u16string button_start_to_line1_start = BUTTON.substr().append(NEWLINE);
  TestGetTextForRange(button_start->Clone(), line1_start->Clone(),
                      button_start_to_line1_start, 1);
  std::u16string button_end_to_line1_end = NEWLINE.substr().append(LINE_1);
  TestGetTextForRange(button_end->Clone(), line1_end->Clone(),
                      button_end_to_line1_end, 1);
  std::u16string button_end_to_line1_start = NEWLINE;
  TestGetTextForRange(button_end->Clone(), line1_start->Clone(),
                      button_end_to_line1_start, 1);

  std::u16string line2_start_to_after_line_end =
      LINE_2.substr().append(NEWLINE).append(AFTER_LINE);
  TestGetTextForRange(line2_start->Clone(), after_line_end->Clone(),
                      line2_start_to_after_line_end, 0);
  std::u16string line2_start_to_after_line_start =
      LINE_2.substr().append(NEWLINE);
  TestGetTextForRange(line2_start->Clone(), after_line_start->Clone(),
                      line2_start_to_after_line_start, 0);
  std::u16string line2_end_to_after_line_end =
      NEWLINE.substr().append(AFTER_LINE);
  TestGetTextForRange(line2_end->Clone(), after_line_end->Clone(),
                      line2_end_to_after_line_end, 0);
  std::u16string line2_end_to_after_line_start = NEWLINE;
  TestGetTextForRange(line2_end->Clone(), after_line_start->Clone(),
                      line2_end_to_after_line_start, 0);

  std::u16string all_text =
      BUTTON.substr().append(NEWLINE).append(TEXT_FIELD).append(AFTER_LINE);
  TestPositionInstance start = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance end = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, ALL_TEXT.length() /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestGetTextForRange(std::move(start), std::move(end), all_text, 1);
}

TEST_F(AXRangeTest, GetTextWithMaxCount) {
  TestPositionInstance line1_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line2_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionRange test_range(line1_start->Clone(), line2_end->Clone());
  EXPECT_EQ(LINE_1.substr(0, 2),
            test_range.GetText(AXTextConcatenationBehavior::kAsInnerText, 2));

  // Test the case where an appended newline falls right at max_count.
  EXPECT_EQ(LINE_1.substr().append(NEWLINE),
            test_range.GetText(AXTextConcatenationBehavior::kAsInnerText, 7));

  // Test passing -1 for max_count.
  EXPECT_EQ(LINE_1.substr().append(NEWLINE).append(LINE_2),
            test_range.GetText(AXTextConcatenationBehavior::kAsInnerText, -1));
}

TEST_F(AXRangeTest, GetTextWithList) {
  const std::u16string kListMarker1 = base::ASCIIToUTF16("1. ");
  const std::u16string kListItemContent = base::ASCIIToUTF16("List item 1");
  const std::u16string kListMarker2 = base::ASCIIToUTF16("2. ");
  const std::u16string kAfterList = base::ASCIIToUTF16("After list");
  const std::u16string kAllText = kListMarker1.substr()
                                      .append(kListItemContent)
                                      .append(NEWLINE)
                                      .append(kListMarker2)
                                      .append(NEWLINE)
                                      .append(kAfterList);
  // This test expects:
  // "1. List item 1
  //  2.
  //  After list"
  // for the following AXTree:
  // ++1 kRootWebArea
  // ++++2 kList
  // ++++++3 kListItem
  // ++++++++4 kListMarker
  // ++++++++++5 kStaticText
  // ++++++++++++6 kInlineTextBox "1. "
  // ++++++++7 kStaticText
  // ++++++++++8 kInlineTextBox "List item 1"
  // ++++++9 kListItem
  // ++++++++10 kListMarker
  // +++++++++++11 kStaticText
  // ++++++++++++++12 kInlineTextBox "2. "
  // ++++13 kStaticText
  // +++++++14 kInlineTextBox "After list"
  AXNodeData root;
  AXNodeData list;
  AXNodeData list_item1;
  AXNodeData list_item2;
  AXNodeData list_marker1;
  AXNodeData list_marker2;
  AXNodeData inline_box1;
  AXNodeData inline_box2;
  AXNodeData inline_box3;
  AXNodeData inline_box4;
  AXNodeData static_text1;
  AXNodeData static_text2;
  AXNodeData static_text3;
  AXNodeData static_text4;

  root.id = 1;
  list.id = 2;
  list_item1.id = 3;
  list_marker1.id = 4;
  static_text1.id = 5;
  inline_box1.id = 6;
  static_text2.id = 7;
  inline_box2.id = 8;
  list_item2.id = 9;
  list_marker2.id = 10;
  static_text3.id = 11;
  inline_box3.id = 12;
  static_text4.id = 13;
  inline_box4.id = 14;

  root.role = ax::mojom::Role::kRootWebArea;
  root.child_ids = {list.id, static_text4.id};

  list.role = ax::mojom::Role::kList;
  list.child_ids = {list_item1.id, list_item2.id};

  list_item1.role = ax::mojom::Role::kListItem;
  list_item1.child_ids = {list_marker1.id, static_text2.id};
  list_item1.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);

  list_marker1.role = ax::mojom::Role::kListMarker;
  list_marker1.child_ids = {static_text1.id};

  static_text1.role = ax::mojom::Role::kStaticText;
  static_text1.SetName(kListMarker1);
  static_text1.child_ids = {inline_box1.id};

  inline_box1.role = ax::mojom::Role::kInlineTextBox;
  inline_box1.SetName(kListMarker1);

  static_text2.role = ax::mojom::Role::kStaticText;
  static_text2.SetName(kListItemContent);
  static_text2.child_ids = {inline_box2.id};

  inline_box2.role = ax::mojom::Role::kInlineTextBox;
  inline_box2.SetName(kListItemContent);

  list_item2.role = ax::mojom::Role::kListItem;
  list_item2.child_ids = {list_marker2.id};
  list_item2.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);

  list_marker2.role = ax::mojom::Role::kListMarker;
  list_marker2.child_ids = {static_text3.id};

  static_text3.role = ax::mojom::Role::kStaticText;
  static_text3.SetName(kListMarker2);
  static_text3.child_ids = {inline_box3.id};

  inline_box3.role = ax::mojom::Role::kInlineTextBox;
  inline_box3.SetName(kListMarker2);

  static_text4.role = ax::mojom::Role::kStaticText;
  static_text4.SetName(kAfterList);
  static_text4.child_ids = {inline_box4.id};

  inline_box4.role = ax::mojom::Role::kInlineTextBox;
  inline_box4.SetName(kAfterList);

  AXTreeUpdate initial_state;
  initial_state.root_id = root.id;
  initial_state.nodes = {root,         list,         list_item1,   list_marker1,
                         static_text1, inline_box1,  static_text2, inline_box2,
                         list_item2,   list_marker2, static_text3, inline_box3,
                         static_text4, inline_box4};
  initial_state.has_tree_data = true;
  initial_state.tree_data.tree_id = AXTreeID::CreateNewAXTreeID();
  initial_state.tree_data.title = "Dialog title";

  SetTree(std::make_unique<AXTree>(initial_state));

  TestPositionInstance start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(start->IsTextPosition());
  TestPositionInstance end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box4.id, 10 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(end->IsTextPosition());
  TestPositionRange forward_range(start->Clone(), end->Clone());
  EXPECT_EQ(kAllText,
            forward_range.GetText(AXTextConcatenationBehavior::kAsInnerText));
  TestPositionRange backward_range(std::move(end), std::move(start));
  EXPECT_EQ(kAllText,
            backward_range.GetText(AXTextConcatenationBehavior::kAsInnerText));
}

TEST_F(AXRangeTest, GetRects) {
  TestAXRangeScreenRectDelegate delegate(this);

  // Setting up ax ranges for testing.
  TestPositionInstance button = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance check_box1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance check_box2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance line1_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line1_second_char = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line1_middle = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line1_second_to_last_char =
      AXNodePosition::CreateTextPosition(GetTreeID(), inline_box1_.id,
                                         5 /* text_offset */,
                                         ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line1_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance line2_start = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line2_second_char = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line2_middle = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line2_second_to_last_char =
      AXNodePosition::CreateTextPosition(GetTreeID(), inline_box2_.id,
                                         5 /* text_offset */,
                                         ax::mojom::TextAffinity::kDownstream);
  TestPositionInstance line2_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance after_line_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box3_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  // Since a button is not visible to the text representation, it spans an
  // empty anchor whose start and end positions are the same.
  TestPositionRange button_range(button->Clone(), button->Clone());
  std::vector<gfx::Rect> expected_screen_rects = {gfx::Rect(20, 20, 100, 30)};
  EXPECT_TRUE(
      ContainerEQ(button_range.GetRects(&delegate), expected_screen_rects));

  // Since a check box is not visible to the text representation, it spans an
  // empty anchor whose start and end positions are the same.
  TestPositionRange check_box1_range(check_box1->Clone(), check_box1->Clone());
  expected_screen_rects = {gfx::Rect(120, 20, 30, 30)};
  EXPECT_TRUE(
      ContainerEQ(check_box1_range.GetRects(&delegate), expected_screen_rects));

  // Retrieving bounding boxes of the button and both checkboxes.
  TestPositionRange button_check_box2_range(button->Clone(),
                                            check_box2->Clone());
  expected_screen_rects = {gfx::Rect(20, 20, 100, 30),
                           gfx::Rect(120, 20, 30, 30),
                           gfx::Rect(150, 20, 30, 30)};
  EXPECT_TRUE(ContainerEQ(button_check_box2_range.GetRects(&delegate),
                          expected_screen_rects));

  // Retrieving bounding box of text line 1, its whole range.
  //  0 1 2 3 4 5
  // |L|i|n|e| |1|
  // |-----------|
  TestPositionRange line1_whole_range(line1_start->Clone(), line1_end->Clone());
  expected_screen_rects = {gfx::Rect(20, 50, 30, 30)};
  EXPECT_TRUE(ContainerEQ(line1_whole_range.GetRects(&delegate),
                          expected_screen_rects));

  // Retrieving bounding box of text line 1, its first half range.
  //  0 1 2 3 4 5
  // |L|i|n|e| |1|
  // |-----|
  TestPositionRange line1_first_half_range(line1_start->Clone(),
                                           line1_middle->Clone());
  expected_screen_rects = {gfx::Rect(20, 50, 15, 30)};
  EXPECT_TRUE(ContainerEQ(line1_first_half_range.GetRects(&delegate),
                          expected_screen_rects));

  // Retrieving bounding box of text line 1, its second half range.
  //  0 1 2 3 4 5
  // |L|i|n|e| |1|
  //       |-----|
  TestPositionRange line1_second_half_range(line1_middle->Clone(),
                                            line1_end->Clone());
  expected_screen_rects = {gfx::Rect(35, 50, 15, 30)};
  EXPECT_TRUE(ContainerEQ(line1_second_half_range.GetRects(&delegate),
                          expected_screen_rects));

  // Retrieving bounding box of text line 1, its mid range.
  //  0 1 2 3 4 5
  // |L|i|n|e| |1|
  //   |-------|
  TestPositionRange line1_mid_range(line1_second_char->Clone(),
                                    line1_second_to_last_char->Clone());
  expected_screen_rects = {gfx::Rect(25, 50, 20, 30)};
  EXPECT_TRUE(
      ContainerEQ(line1_mid_range.GetRects(&delegate), expected_screen_rects));

  // Retrieving bounding box of text line 2, its whole range.
  //  0 1 2 3 4 5
  // |L|i|n|e| |2|
  // |-----------|
  TestPositionRange line2_whole_range(line2_start->Clone(), line2_end->Clone());
  expected_screen_rects = {gfx::Rect(20, 80, 42, 30)};
  EXPECT_TRUE(ContainerEQ(line2_whole_range.GetRects(&delegate),
                          expected_screen_rects));

  // Retrieving bounding box of text line 2, its first half range.
  //  0 1 2 3 4 5
  // |L|i|n|e| |2|
  // |-----|
  TestPositionRange line2_first_half_range(line2_start->Clone(),
                                           line2_middle->Clone());
  expected_screen_rects = {gfx::Rect(20, 80, 21, 30)};
  EXPECT_TRUE(ContainerEQ(line2_first_half_range.GetRects(&delegate),
                          expected_screen_rects));

  // Retrieving bounding box of text line 2, its second half range.
  //  0 1 2 3 4 5
  // |L|i|n|e| |2|
  //       |-----|
  TestPositionRange line2_second_half_range(line2_middle->Clone(),
                                            line2_end->Clone());
  expected_screen_rects = {gfx::Rect(41, 80, 21, 30)};
  EXPECT_TRUE(ContainerEQ(line2_second_half_range.GetRects(&delegate),
                          expected_screen_rects));

  // Retrieving bounding box of text line 2, its mid range.
  //  0 1 2 3 4 5
  // |L|i|n|e| |2|
  //   |-------|
  TestPositionRange line2_mid_range(line2_second_char->Clone(),
                                    line2_second_to_last_char->Clone());
  expected_screen_rects = {gfx::Rect(27, 80, 28, 30)};
  EXPECT_TRUE(
      ContainerEQ(line2_mid_range.GetRects(&delegate), expected_screen_rects));

  // Retrieving bounding boxes of text line 1 and line 2, the entire range.
  // |L|i|n|e| |1|\n|L|i|n|e| |2|\n|
  // |--------------------------|
  TestPositionRange line1_line2_whole_range(line1_start->Clone(),
                                            line2_end->Clone());
  expected_screen_rects = {gfx::Rect(20, 50, 30, 30),
                           gfx::Rect(20, 80, 42, 30)};
  EXPECT_TRUE(ContainerEQ(line1_line2_whole_range.GetRects(&delegate),
                          expected_screen_rects));

  // Retrieving bounding boxes of the range that spans from the middle of text
  // line 1 to the middle of text line 2.
  // |L|i|n|e| |1|\n|L|i|n|e| |2|\n|
  //       |--------------|
  TestPositionRange line1_line2_mid_range(line1_middle->Clone(),
                                          line2_middle->Clone());
  expected_screen_rects = {gfx::Rect(35, 50, 15, 30),
                           gfx::Rect(20, 80, 21, 30)};
  EXPECT_TRUE(ContainerEQ(line1_line2_mid_range.GetRects(&delegate),
                          expected_screen_rects));

  // Retrieving bounding boxes of the range that spans from the checkbox 2
  // ("invisible" in the text representation) to the middle of text line 2.
  // |[Button][Checkbox 1][Checkbox 2]L|i|n|e| |1|\n|L|i|n|e| |2|\n|A|f|t|e|r|
  //                      |-------------------------------|
  TestPositionRange check_box2_line2_mid_range(check_box2->Clone(),
                                               line2_middle->Clone());
  expected_screen_rects = {gfx::Rect(150, 20, 30, 30),
                           gfx::Rect(20, 50, 30, 30),
                           gfx::Rect(20, 80, 21, 30)};
  EXPECT_TRUE(ContainerEQ(check_box2_line2_mid_range.GetRects(&delegate),
                          expected_screen_rects));

  // Retrieving bounding boxes of the range spanning the entire document.
  // |[Button][Checkbox 1][Checkbox 2]L|i|n|e| |1|\n|L|i|n|e| |2|\n|A|f|t|e|r|
  // |-----------------------------------------------------------------------|
  TestPositionRange entire_test_range(button->Clone(), after_line_end->Clone());
  expected_screen_rects = {
      gfx::Rect(20, 20, 100, 30), gfx::Rect(120, 20, 30, 30),
      gfx::Rect(150, 20, 30, 30), gfx::Rect(20, 50, 30, 30),
      gfx::Rect(20, 80, 42, 30),  gfx::Rect(20, 110, 50, 30)};
  EXPECT_TRUE(ContainerEQ(entire_test_range.GetRects(&delegate),
                          expected_screen_rects));
}

TEST_F(AXRangeTest, GetRectsOffscreen) {
  // Set up root node bounds/viewport size  to {0, 50, 800x60}, so that only
  // some text will be onscreen the rest will be offscreen.
  AXNodeData old_root_node_data = GetRootAsAXNode()->data();
  AXNodeData new_root_node_data = old_root_node_data;
  new_root_node_data.relative_bounds.bounds = gfx::RectF(0, 50, 800, 60);
  GetRootAsAXNode()->SetData(new_root_node_data);

  TestAXRangeScreenRectDelegate delegate(this);

  TestPositionInstance button = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionInstance after_line_end = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box3_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  // [Button]           [Checkbox 1]         [Checkbox 2]
  // {20, 20, 100x30},  {120, 20, 30x30}     {150, 20, 30x30}
  //                                              ---
  // [Line 1]                                     |
  // {20, 50, 30x30}                              | view port, onscreen
  //                                              | {0, 50, 800x60}
  // [Line 2]                                     |
  // {20, 80, 42x30}                              |
  //                                              ---
  // [After]
  // {20, 110, 50x30}
  //
  // Retrieving bounding boxes of the range spanning the entire document.
  // |[Button][Checkbox 1][Checkbox 2]L|i|n|e| |1|\n|L|i|n|e| |2|\n|A|f|t|e|r|
  // |-----------------------------------------------------------------------|
  TestPositionRange entire_test_range(button->Clone(), after_line_end->Clone());
  std::vector<gfx::Rect> expected_screen_rects = {gfx::Rect(20, 50, 30, 30),
                                                  gfx::Rect(20, 80, 42, 30)};
  EXPECT_TRUE(ContainerEQ(entire_test_range.GetRects(&delegate),
                          expected_screen_rects));

  // Reset the root node bounds/viewport size back to {0, 0, 800x600}, and
  // verify all elements should be onscreen.
  GetRootAsAXNode()->SetData(old_root_node_data);
  expected_screen_rects = {
      gfx::Rect(20, 20, 100, 30), gfx::Rect(120, 20, 30, 30),
      gfx::Rect(150, 20, 30, 30), gfx::Rect(20, 50, 30, 30),
      gfx::Rect(20, 80, 42, 30),  gfx::Rect(20, 110, 50, 30)};
  EXPECT_TRUE(ContainerEQ(entire_test_range.GetRects(&delegate),
                          expected_screen_rects));
}

}  // namespace ui

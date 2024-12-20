// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <algorithm>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ax/ax_enums.h"
#include "ax/ax_node.h"
#include "ax/ax_node_data.h"
#include "ax/ax_node_position.h"
#include "ax/ax_range.h"
#include "ax/ax_tree.h"
#include "ax/ax_tree_data.h"
#include "ax/ax_tree_id.h"
#include "ax/ax_tree_update.h"
#include "ax/test_ax_tree_manager.h"
#include "flutter/fml/platform/win/wstring_conversion.h"
#include "gtest/gtest.h"

namespace ui {

using TestPositionType = std::unique_ptr<AXPosition<AXNodePosition, AXNode>>;
using TestPositionRange = AXRange<AXPosition<AXNodePosition, AXNode>>;

namespace {

constexpr AXNode::AXID ROOT_ID = 1;
constexpr AXNode::AXID BUTTON_ID = 2;
constexpr AXNode::AXID CHECK_BOX_ID = 3;
constexpr AXNode::AXID TEXT_FIELD_ID = 4;
constexpr AXNode::AXID STATIC_TEXT1_ID = 5;
constexpr AXNode::AXID INLINE_BOX1_ID = 6;
constexpr AXNode::AXID LINE_BREAK_ID = 7;
constexpr AXNode::AXID STATIC_TEXT2_ID = 8;
constexpr AXNode::AXID INLINE_BOX2_ID = 9;

// A group of basic and extended characters.
constexpr const char16_t* kGraphemeClusters[] = {
    // The English word "hey" consisting of four ASCII characters.
    u"h",
    u"e",
    u"y",
    // A Hindi word (which means "Hindi") consisting of two Devanagari
    // grapheme clusters.
    u"\x0939\x093F",
    u"\x0928\x094D\x0926\x0940",
    // A Thai word (which means "feel") consisting of three Thai grapheme
    // clusters.
    u"\x0E23\x0E39\x0E49",
    u"\x0E2A\x0E36",
    u"\x0E01",
};

class AXPositionTest : public testing::Test, public TestAXTreeManager {
 public:
  AXPositionTest() = default;
  ~AXPositionTest() override = default;

 protected:
  static const char* TEXT_VALUE;

  void SetUp() override;

  // Creates a document with three pages, adding any extra information to this
  // basic document structure that has been provided as arguments.
  std::unique_ptr<AXTree> CreateMultipageDocument(
      AXNodeData& root_data,
      AXNodeData& page_1_data,
      AXNodeData& page_1_text_data,
      AXNodeData& page_2_data,
      AXNodeData& page_2_text_data,
      AXNodeData& page_3_data,
      AXNodeData& page_3_text_data) const;

  // Creates a document with three static text objects each containing text in a
  // different language.
  std::unique_ptr<AXTree> CreateMultilingualDocument(
      std::vector<int>* text_offsets) const;

  void AssertTextLengthEquals(const AXTree* tree,
                              AXNode::AXID node_id,
                              int expected_text_length) const;

  // Creates a new AXTree from a vector of nodes.
  // Assumes the first node in the vector is the root.
  std::unique_ptr<AXTree> CreateAXTree(
      const std::vector<AXNodeData>& nodes) const;

  AXNodeData root_;
  AXNodeData button_;
  AXNodeData check_box_;
  AXNodeData text_field_;
  AXNodeData static_text1_;
  AXNodeData line_break_;
  AXNodeData static_text2_;
  AXNodeData inline_box1_;
  AXNodeData inline_box2_;

 private:
  BASE_DISALLOW_COPY_AND_ASSIGN(AXPositionTest);
};

// Used by AXPositionExpandToEnclosingTextBoundaryTestWithParam.
//
// Every test instance starts from a pre-determined position and calls the
// ExpandToEnclosingTextBoundary method with the arguments provided in this
// struct.
struct ExpandToEnclosingTextBoundaryTestParam {
  ExpandToEnclosingTextBoundaryTestParam() = default;

  // Required by GTest framework.
  ExpandToEnclosingTextBoundaryTestParam(
      const ExpandToEnclosingTextBoundaryTestParam& other) = default;
  ExpandToEnclosingTextBoundaryTestParam& operator=(
      const ExpandToEnclosingTextBoundaryTestParam& other) = default;

  ~ExpandToEnclosingTextBoundaryTestParam() = default;

  // The text boundary to expand to.
  ax::mojom::TextBoundary boundary;

  // Determines how to expand to the enclosing range when the starting position
  // is already at a text boundary.
  AXRangeExpandBehavior expand_behavior;

  // The text position that should be returned for the anchor of the range.
  std::string expected_anchor_position;

  // The text position that should be returned for the focus of the range.
  std::string expected_focus_position;
};

// This is a fixture for a set of parameterized tests that test the
// |ExpandToEnclosingTextBoundary| method with all possible input arguments.
class AXPositionExpandToEnclosingTextBoundaryTestWithParam
    : public AXPositionTest,
      public testing::WithParamInterface<
          ExpandToEnclosingTextBoundaryTestParam> {
 public:
  AXPositionExpandToEnclosingTextBoundaryTestWithParam() = default;
  ~AXPositionExpandToEnclosingTextBoundaryTestWithParam() override = default;

  BASE_DISALLOW_COPY_AND_ASSIGN(
      AXPositionExpandToEnclosingTextBoundaryTestWithParam);
};

// Used by AXPositionCreatePositionAtTextBoundaryTestWithParam.
//
// Every test instance starts from a pre-determined position and calls the
// CreatePositionAtTextBoundary method with the arguments provided in this
// struct.
struct CreatePositionAtTextBoundaryTestParam {
  CreatePositionAtTextBoundaryTestParam() = default;

  // Required by GTest framework.
  CreatePositionAtTextBoundaryTestParam(
      const CreatePositionAtTextBoundaryTestParam& other) = default;
  CreatePositionAtTextBoundaryTestParam& operator=(
      const CreatePositionAtTextBoundaryTestParam& other) = default;

  ~CreatePositionAtTextBoundaryTestParam() = default;

  // The text boundary to move to.
  ax::mojom::TextBoundary boundary;

  // The direction to move to.
  ax::mojom::MoveDirection direction;

  // What to do when the starting position is already at a text boundary, or
  // when the movement operation will cause us to cross the starting object's
  // boundary.
  AXBoundaryBehavior boundary_behavior;

  // The text position that should be returned, if the method was called on a
  // text position instance.
  std::string expected_text_position;
};

// This is a fixture for a set of parameterized tests that test the
// |CreatePositionAtTextBoundary| method with all possible input arguments.
class AXPositionCreatePositionAtTextBoundaryTestWithParam
    : public AXPositionTest,
      public testing::WithParamInterface<
          CreatePositionAtTextBoundaryTestParam> {
 public:
  AXPositionCreatePositionAtTextBoundaryTestWithParam() = default;
  ~AXPositionCreatePositionAtTextBoundaryTestWithParam() override = default;

  BASE_DISALLOW_COPY_AND_ASSIGN(
      AXPositionCreatePositionAtTextBoundaryTestWithParam);
};

// Used by |AXPositionTextNavigationTestWithParam|.
//
// The test starts from a pre-determined position and repeats a text navigation
// operation, such as |CreateNextWordStartPosition|, until it runs out of
// expectations.
struct TextNavigationTestParam {
  TextNavigationTestParam() = default;

  // Required by GTest framework.
  TextNavigationTestParam(const TextNavigationTestParam& other) = default;
  TextNavigationTestParam& operator=(const TextNavigationTestParam& other) =
      default;

  ~TextNavigationTestParam() = default;

  // Stores the method that should be called repeatedly by the test to create
  // the next position.
  std::function<TestPositionType(const TestPositionType&)> TestMethod;

  // The node at which the test should start.
  AXNode::AXID start_node_id;

  // The text offset at which the test should start.
  int start_offset;

  // A list of positions that should be returned from the method being tested,
  // in stringified form.
  std::vector<std::string> expectations;
};

// This is a fixture for a set of parameterized tests that ensure that text
// navigation operations, such as |CreateNextWordStartPosition|, work properly.
//
// Starting from a given position, test instances call a given text navigation
// method repeatedly and compare the return values to a set of expectations.
//
// TODO(nektar): Only text positions are tested for now.
class AXPositionTextNavigationTestWithParam
    : public AXPositionTest,
      public testing::WithParamInterface<TextNavigationTestParam> {
 public:
  AXPositionTextNavigationTestWithParam() = default;
  ~AXPositionTextNavigationTestWithParam() override = default;

  BASE_DISALLOW_COPY_AND_ASSIGN(AXPositionTextNavigationTestWithParam);
};

const char* AXPositionTest::TEXT_VALUE = "Line 1\nLine 2";

void AXPositionTest::SetUp() {
  // Most tests use kSuppressCharacter behavior.
  g_ax_embedded_object_behavior = AXEmbeddedObjectBehavior::kSuppressCharacter;

  // root_
  //  |
  //  +------------+-----------+
  //  |            |           |
  // button_  check_box_  text_field_
  //                           |
  //               +-----------+------------+
  //               |           |            |
  //        static_text1_  line_break_   static_text2_
  //               |                        |
  //        inline_box1_                 inline_box2_

  root_.id = ROOT_ID;
  button_.id = BUTTON_ID;
  check_box_.id = CHECK_BOX_ID;
  text_field_.id = TEXT_FIELD_ID;
  static_text1_.id = STATIC_TEXT1_ID;
  inline_box1_.id = INLINE_BOX1_ID;
  line_break_.id = LINE_BREAK_ID;
  static_text2_.id = STATIC_TEXT2_ID;
  inline_box2_.id = INLINE_BOX2_ID;

  root_.role = ax::mojom::Role::kRootWebArea;
  root_.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  button_.role = ax::mojom::Role::kButton;
  button_.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                           true);
  button_.SetHasPopup(ax::mojom::HasPopup::kMenu);
  button_.SetName("Button");
  button_.relative_bounds.bounds = gfx::RectF(20, 20, 200, 30);
  root_.child_ids.push_back(button_.id);

  check_box_.role = ax::mojom::Role::kCheckBox;
  check_box_.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);
  check_box_.SetCheckedState(ax::mojom::CheckedState::kTrue);
  check_box_.SetName("Check box");
  check_box_.relative_bounds.bounds = gfx::RectF(20, 50, 200, 30);
  root_.child_ids.push_back(check_box_.id);

  text_field_.role = ax::mojom::Role::kTextField;
  text_field_.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                               true);
  text_field_.AddState(ax::mojom::State::kEditable);
  text_field_.SetValue(TEXT_VALUE);
  text_field_.SetName(TEXT_VALUE);
  text_field_.AddIntListAttribute(
      ax::mojom::IntListAttribute::kCachedLineStarts,
      std::vector<int32_t>{0, 7});
  text_field_.child_ids.push_back(static_text1_.id);
  text_field_.child_ids.push_back(line_break_.id);
  text_field_.child_ids.push_back(static_text2_.id);
  root_.child_ids.push_back(text_field_.id);

  static_text1_.role = ax::mojom::Role::kStaticText;
  static_text1_.AddState(ax::mojom::State::kEditable);
  static_text1_.SetName("Line 1");
  static_text1_.child_ids.push_back(inline_box1_.id);
  static_text1_.AddIntAttribute(
      ax::mojom::IntAttribute::kTextStyle,
      static_cast<int32_t>(ax::mojom::TextStyle::kBold));

  inline_box1_.role = ax::mojom::Role::kInlineTextBox;
  inline_box1_.AddState(ax::mojom::State::kEditable);
  inline_box1_.SetName("Line 1");
  inline_box1_.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                   std::vector<int32_t>{0, 5});
  inline_box1_.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                   std::vector<int32_t>{4, 6});
  inline_box1_.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                               line_break_.id);

  line_break_.role = ax::mojom::Role::kLineBreak;
  line_break_.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                               true);
  line_break_.AddState(ax::mojom::State::kEditable);
  line_break_.SetName("\n");
  line_break_.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                              inline_box1_.id);

  static_text2_.role = ax::mojom::Role::kStaticText;
  static_text2_.AddState(ax::mojom::State::kEditable);
  static_text2_.SetName("Line 2");
  static_text2_.child_ids.push_back(inline_box2_.id);
  static_text2_.AddFloatAttribute(ax::mojom::FloatAttribute::kFontSize, 1.0f);

  inline_box2_.role = ax::mojom::Role::kInlineTextBox;
  inline_box2_.AddState(ax::mojom::State::kEditable);
  inline_box2_.SetName("Line 2");
  inline_box2_.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                   std::vector<int32_t>{0, 5});
  inline_box2_.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                   std::vector<int32_t>{4, 6});

  AXTreeUpdate initial_state;
  initial_state.root_id = 1;
  initial_state.nodes = {root_,       button_,       check_box_,
                         text_field_, static_text1_, inline_box1_,
                         line_break_, static_text2_, inline_box2_};
  initial_state.has_tree_data = true;
  initial_state.tree_data.tree_id = AXTreeID::CreateNewAXTreeID();
  initial_state.tree_data.title = "Dialog title";

  // "SetTree" is defined in "TestAXTreeManager" and it passes ownership of the
  // created AXTree to the manager.
  SetTree(std::make_unique<AXTree>(initial_state));
}

std::unique_ptr<AXTree> AXPositionTest::CreateMultipageDocument(
    AXNodeData& root_data,
    AXNodeData& page_1_data,
    AXNodeData& page_1_text_data,
    AXNodeData& page_2_data,
    AXNodeData& page_2_text_data,
    AXNodeData& page_3_data,
    AXNodeData& page_3_text_data) const {
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kDocument;

  page_1_data.id = 2;
  page_1_data.role = ax::mojom::Role::kRegion;
  page_1_data.AddBoolAttribute(ax::mojom::BoolAttribute::kIsPageBreakingObject,
                               true);

  page_1_text_data.id = 3;
  page_1_text_data.role = ax::mojom::Role::kStaticText;
  page_1_text_data.SetName("some text on page 1");
  page_1_text_data.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
  page_1_data.child_ids = {3};

  page_2_data.id = 4;
  page_2_data.role = ax::mojom::Role::kRegion;
  page_2_data.AddBoolAttribute(ax::mojom::BoolAttribute::kIsPageBreakingObject,
                               true);

  page_2_text_data.id = 5;
  page_2_text_data.role = ax::mojom::Role::kStaticText;
  page_2_text_data.SetName("some text on page 2");
  page_2_text_data.AddIntAttribute(
      ax::mojom::IntAttribute::kTextStyle,
      static_cast<int32_t>(ax::mojom::TextStyle::kBold));
  page_2_data.child_ids = {5};

  page_3_data.id = 6;
  page_3_data.role = ax::mojom::Role::kRegion;
  page_3_data.AddBoolAttribute(ax::mojom::BoolAttribute::kIsPageBreakingObject,
                               true);

  page_3_text_data.id = 7;
  page_3_text_data.role = ax::mojom::Role::kStaticText;
  page_3_text_data.SetName("some more text on page 3");
  page_3_data.child_ids = {7};

  root_data.child_ids = {2, 4, 6};

  return CreateAXTree({root_data, page_1_data, page_1_text_data, page_2_data,
                       page_2_text_data, page_3_data, page_3_text_data});
}

std::unique_ptr<AXTree> AXPositionTest::CreateMultilingualDocument(
    std::vector<int>* text_offsets) const {
  EXPECT_NE(nullptr, text_offsets);
  text_offsets->push_back(0);

  std::u16string english_text;
  for (int i = 0; i < 3; ++i) {
    std::u16string grapheme = kGraphemeClusters[i];
    EXPECT_EQ(1u, grapheme.length())
        << "All English characters should be one UTF16 code unit in length.";
    text_offsets->push_back(text_offsets->back() +
                            static_cast<int>(grapheme.length()));
    english_text.append(grapheme);
  }

  std::u16string hindi_text;
  for (int i = 3; i < 5; ++i) {
    std::u16string grapheme = kGraphemeClusters[i];
    EXPECT_LE(2u, grapheme.length()) << "All Hindi characters should be two "
                                        "or more UTF16 code units in length.";
    text_offsets->push_back(text_offsets->back() +
                            static_cast<int>(grapheme.length()));
    hindi_text.append(grapheme);
  }

  std::u16string thai_text;
  for (int i = 5; i < 8; ++i) {
    std::u16string grapheme = kGraphemeClusters[i];
    EXPECT_LT(0u, grapheme.length())
        << "One of the Thai characters should be one UTF16 code unit, "
           "whilst others should be two or more.";
    text_offsets->push_back(text_offsets->back() +
                            static_cast<int>(grapheme.length()));
    thai_text.append(grapheme);
  }

  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData text_data1;
  text_data1.id = 2;
  text_data1.role = ax::mojom::Role::kStaticText;
  text_data1.SetName(english_text);

  AXNodeData text_data2;
  text_data2.id = 3;
  text_data2.role = ax::mojom::Role::kStaticText;
  text_data2.SetName(hindi_text);

  AXNodeData text_data3;
  text_data3.id = 4;
  text_data3.role = ax::mojom::Role::kStaticText;
  text_data3.SetName(thai_text);

  root_data.child_ids = {text_data1.id, text_data2.id, text_data3.id};

  return CreateAXTree({root_data, text_data1, text_data2, text_data3});
}

void AXPositionTest::AssertTextLengthEquals(const AXTree* tree,
                                            AXNode::AXID node_id,
                                            int expected_text_length) const {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      tree->data().tree_id, node_id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(expected_text_length, text_position->MaxTextOffset());
  ASSERT_EQ(expected_text_length,
            static_cast<int>(text_position->GetText().length()));
}

std::unique_ptr<AXTree> AXPositionTest::CreateAXTree(
    const std::vector<AXNodeData>& nodes) const {
  EXPECT_FALSE(nodes.empty());
  AXTreeUpdate update;
  update.tree_data.tree_id = AXTreeID::CreateNewAXTreeID();
  update.has_tree_data = true;
  update.root_id = nodes[0].id;
  update.nodes = nodes;
  return std::make_unique<AXTree>(update);
}

}  // namespace

TEST_F(AXPositionTest, Clone) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType copy_position = null_position->Clone();
  ASSERT_NE(nullptr, copy_position);
  EXPECT_TRUE(copy_position->IsNullPosition());

  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  copy_position = tree_position->Clone();
  ASSERT_NE(nullptr, copy_position);
  EXPECT_TRUE(copy_position->IsTreePosition());
  EXPECT_EQ(root_.id, copy_position->anchor_id());
  EXPECT_EQ(1, copy_position->child_index());
  EXPECT_EQ(AXNodePosition::INVALID_OFFSET, copy_position->text_offset());

  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_NE(nullptr, tree_position);
  copy_position = tree_position->Clone();
  ASSERT_NE(nullptr, copy_position);
  EXPECT_TRUE(copy_position->IsTreePosition());
  EXPECT_EQ(root_.id, copy_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, copy_position->child_index());
  EXPECT_EQ(AXNodePosition::INVALID_OFFSET, copy_position->text_offset());

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  copy_position = text_position->Clone();
  ASSERT_NE(nullptr, copy_position);
  EXPECT_TRUE(copy_position->IsTextPosition());
  EXPECT_EQ(text_field_.id, copy_position->anchor_id());
  EXPECT_EQ(0, copy_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kUpstream, copy_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  copy_position = text_position->Clone();
  ASSERT_NE(nullptr, copy_position);
  EXPECT_TRUE(copy_position->IsTextPosition());
  EXPECT_EQ(text_field_.id, copy_position->anchor_id());
  EXPECT_EQ(0, copy_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, copy_position->affinity());
  EXPECT_EQ(AXNodePosition::INVALID_INDEX, copy_position->child_index());
}

TEST_F(AXPositionTest, Serialize) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType copy_position =
      AXNodePosition::Unserialize(null_position->Serialize());
  ASSERT_NE(nullptr, copy_position);
  EXPECT_TRUE(copy_position->IsNullPosition());

  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  copy_position = AXNodePosition::Unserialize(tree_position->Serialize());
  ASSERT_NE(nullptr, copy_position);
  EXPECT_TRUE(copy_position->IsTreePosition());
  EXPECT_EQ(root_.id, copy_position->anchor_id());
  EXPECT_EQ(1, copy_position->child_index());
  EXPECT_EQ(AXNodePosition::INVALID_OFFSET, copy_position->text_offset());

  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_NE(nullptr, tree_position);
  copy_position = AXNodePosition::Unserialize(tree_position->Serialize());
  ASSERT_NE(nullptr, copy_position);
  EXPECT_TRUE(copy_position->IsTreePosition());
  EXPECT_EQ(root_.id, copy_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, copy_position->child_index());
  EXPECT_EQ(AXNodePosition::INVALID_OFFSET, copy_position->text_offset());

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  copy_position = AXNodePosition::Unserialize(text_position->Serialize());
  ASSERT_NE(nullptr, copy_position);
  EXPECT_TRUE(copy_position->IsTextPosition());
  EXPECT_EQ(text_field_.id, copy_position->anchor_id());
  EXPECT_EQ(0, copy_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kUpstream, copy_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  copy_position = AXNodePosition::Unserialize(text_position->Serialize());
  ASSERT_NE(nullptr, copy_position);
  EXPECT_TRUE(copy_position->IsTextPosition());
  EXPECT_EQ(text_field_.id, copy_position->anchor_id());
  EXPECT_EQ(0, copy_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, copy_position->affinity());
  EXPECT_EQ(AXNodePosition::INVALID_INDEX, copy_position->child_index());
}

TEST_F(AXPositionTest, ToString) {
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData static_text_data_1;
  static_text_data_1.id = 2;
  static_text_data_1.role = ax::mojom::Role::kStaticText;
  static_text_data_1.SetName("some text");

  AXNodeData static_text_data_2;
  static_text_data_2.id = 3;
  static_text_data_2.role = ax::mojom::Role::kStaticText;
  static_text_data_2.SetName(u"\xfffc");

  AXNodeData static_text_data_3;
  static_text_data_3.id = 4;
  static_text_data_3.role = ax::mojom::Role::kStaticText;
  static_text_data_3.SetName("more text");

  root_data.child_ids = {static_text_data_1.id, static_text_data_2.id,
                         static_text_data_3.id};

  SetTree(CreateAXTree(
      {root_data, static_text_data_1, static_text_data_2, static_text_data_3}));

  TestPositionType text_position_1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_1->IsTextPosition());
  EXPECT_EQ(
      "TextPosition anchor_id=1 text_offset=0 affinity=downstream "
      "annotated_text=<s>ome text\xEF\xBF\xBCmore text",
      text_position_1->ToString());

  TestPositionType text_position_2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_2->IsTextPosition());
  EXPECT_EQ(
      "TextPosition anchor_id=1 text_offset=5 affinity=downstream "
      "annotated_text=some <t>ext\xEF\xBF\xBCmore text",
      text_position_2->ToString());

  TestPositionType text_position_3 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 9 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_3->IsTextPosition());
  EXPECT_EQ(
      "TextPosition anchor_id=1 text_offset=9 affinity=downstream "
      "annotated_text=some text<\xEF\xBF\xBC>more text",
      text_position_3->ToString());

  TestPositionType text_position_4 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 10 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_4->IsTextPosition());
  EXPECT_EQ(
      "TextPosition anchor_id=1 text_offset=10 affinity=downstream "
      "annotated_text=some text\xEF\xBF\xBC<m>ore text",
      text_position_4->ToString());

  TestPositionType text_position_5 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 19 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_5->IsTextPosition());
  EXPECT_EQ(
      "TextPosition anchor_id=1 text_offset=19 affinity=downstream "
      "annotated_text=some text\xEF\xBF\xBCmore text<>",
      text_position_5->ToString());

  TestPositionType text_position_6 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text_data_2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_6->IsTextPosition());
  EXPECT_EQ(
      "TextPosition anchor_id=3 text_offset=0 affinity=downstream "
      "annotated_text=<\xEF\xBF\xBC>",
      text_position_6->ToString());

  TestPositionType text_position_7 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text_data_2.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_7->IsTextPosition());
  EXPECT_EQ(
      "TextPosition anchor_id=3 text_offset=1 affinity=downstream "
      "annotated_text=\xEF\xBF\xBC<>",
      text_position_7->ToString());

  TestPositionType text_position_8 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text_data_3.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_8->IsTextPosition());
  EXPECT_EQ(
      "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
      "annotated_text=<m>ore text",
      text_position_8->ToString());

  TestPositionType text_position_9 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text_data_3.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_9->IsTextPosition());
  EXPECT_EQ(
      "TextPosition anchor_id=4 text_offset=5 affinity=downstream "
      "annotated_text=more <t>ext",
      text_position_9->ToString());

  TestPositionType text_position_10 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text_data_3.id, 9 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_10->IsTextPosition());
  EXPECT_EQ(
      "TextPosition anchor_id=4 text_offset=9 affinity=downstream "
      "annotated_text=more text<>",
      text_position_10->ToString());
}

TEST_F(AXPositionTest, IsIgnored) {
  EXPECT_FALSE(AXNodePosition::CreateNullPosition()->IsIgnored());

  // We now need to update the tree structure to test ignored tree and text
  // positions.
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData static_text_data_1;
  static_text_data_1.id = 2;
  static_text_data_1.role = ax::mojom::Role::kStaticText;
  static_text_data_1.SetName("One");

  AXNodeData inline_box_data_1;
  inline_box_data_1.id = 3;
  inline_box_data_1.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_1.SetName("One");
  inline_box_data_1.AddState(ax::mojom::State::kIgnored);

  AXNodeData container_data;
  container_data.id = 4;
  container_data.role = ax::mojom::Role::kGenericContainer;
  container_data.AddState(ax::mojom::State::kIgnored);

  AXNodeData static_text_data_2;
  static_text_data_2.id = 5;
  static_text_data_2.role = ax::mojom::Role::kStaticText;
  static_text_data_2.SetName("Two");

  AXNodeData inline_box_data_2;
  inline_box_data_2.id = 6;
  inline_box_data_2.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_2.SetName("Two");

  static_text_data_1.child_ids = {inline_box_data_1.id};
  container_data.child_ids = {static_text_data_2.id};
  static_text_data_2.child_ids = {inline_box_data_2.id};
  root_data.child_ids = {static_text_data_1.id, container_data.id};

  SetTree(
      CreateAXTree({root_data, static_text_data_1, inline_box_data_1,
                    container_data, static_text_data_2, inline_box_data_2}));

  //
  // Text positions.
  //

  TestPositionType text_position_1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_1->IsTextPosition());
  // Since the leaf node containing the text that is pointed to is ignored, this
  // position should be ignored.
  EXPECT_TRUE(text_position_1->IsIgnored());

  // Create a text position before the letter "e" in "One".
  TestPositionType text_position_2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 2 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_2->IsTextPosition());
  // Same as above.
  EXPECT_TRUE(text_position_2->IsIgnored());

  // Create a text position before the letter "T" in "Two".
  TestPositionType text_position_3 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_3->IsTextPosition());
  // Since the leaf node containing the text that is pointed to is not ignored,
  // but only a generic container that is in between this position and the leaf
  // node, this position should not be ignored.
  EXPECT_FALSE(text_position_3->IsIgnored());

  // Create a text position before the letter "w" in "Two".
  TestPositionType text_position_4 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_4->IsTextPosition());
  // Same as above.
  EXPECT_FALSE(text_position_4->IsIgnored());

  // But a text position on the ignored generic container itself, should be
  // ignored.
  TestPositionType text_position_5 = AXNodePosition::CreateTextPosition(
      GetTreeID(), container_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_5->IsTextPosition());
  EXPECT_TRUE(text_position_5->IsIgnored());

  // Whilst a text position on its static text child should not be ignored since
  // there is nothing ignore below the generic container.
  TestPositionType text_position_6 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text_data_2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_6->IsTextPosition());
  EXPECT_FALSE(text_position_6->IsIgnored());

  // A text position on an ignored leaf node should be ignored.
  TestPositionType text_position_7 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_data_1.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position_7->IsTextPosition());
  EXPECT_TRUE(text_position_7->IsIgnored());

  //
  // Tree positions.
  //

  // A "before children" position on the root should not be ignored, despite the
  // fact that the leaf equivalent position is, because we can always adjust to
  // an unignored position if asked to find the leaf equivalent unignored
  // position.
  TestPositionType tree_position_1 = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_data.id, 0 /* child_index */);
  ASSERT_TRUE(tree_position_1->IsTreePosition());
  EXPECT_FALSE(tree_position_1->IsIgnored());

  // A tree position pointing to an ignored child node should be ignored.
  TestPositionType tree_position_2 = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_data.id, 1 /* child_index */);
  ASSERT_TRUE(tree_position_2->IsTreePosition());
  EXPECT_TRUE(tree_position_2->IsIgnored());

  // An "after text" tree position on an ignored leaf node should be ignored.
  TestPositionType tree_position_3 = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box_data_1.id, 0 /* child_index */);
  ASSERT_TRUE(tree_position_3->IsTreePosition());
  EXPECT_TRUE(tree_position_3->IsIgnored());

  // A "before text" tree position on an ignored leaf node should be ignored.
  TestPositionType tree_position_4 = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box_data_1.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_TRUE(tree_position_4->IsTreePosition());
  EXPECT_TRUE(tree_position_4->IsIgnored());

  // An "after children" tree position on the root node, where the last child is
  // ignored, should be ignored.
  TestPositionType tree_position_5 = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_data.id, 2 /* child_index */);
  ASSERT_TRUE(tree_position_5->IsTreePosition());
  EXPECT_TRUE(tree_position_5->IsIgnored());

  // A "before text" position on an unignored node should not be ignored.
  TestPositionType tree_position_6 = AXNodePosition::CreateTreePosition(
      GetTreeID(), static_text_data_1.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_TRUE(tree_position_6->IsTreePosition());
  EXPECT_FALSE(tree_position_6->IsIgnored());
}

TEST_F(AXPositionTest, GetTextFromNullPosition) {
  TestPositionType text_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsNullPosition());
  ASSERT_EQ(u"", text_position->GetText());
}

TEST_F(AXPositionTest, GetTextFromRoot) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(u"Line 1\nLine 2", text_position->GetText());
}

TEST_F(AXPositionTest, GetTextFromButton) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(u"", text_position->GetText());
}

TEST_F(AXPositionTest, GetTextFromCheckbox) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(u"", text_position->GetText());
}

TEST_F(AXPositionTest, GetTextFromTextField) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(u"Line 1\nLine 2", text_position->GetText());
}

TEST_F(AXPositionTest, GetTextFromStaticText) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(u"Line 1", text_position->GetText());
}

TEST_F(AXPositionTest, GetTextFromInlineTextBox) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(u"Line 1", text_position->GetText());
}

TEST_F(AXPositionTest, GetTextFromLineBreak) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(u"\n", text_position->GetText());
}

TEST_F(AXPositionTest, GetMaxTextOffsetFromNullPosition) {
  TestPositionType text_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsNullPosition());
  ASSERT_EQ(AXNodePosition::INVALID_OFFSET, text_position->MaxTextOffset());
}

TEST_F(AXPositionTest, GetMaxTextOffsetFromRoot) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(13, text_position->MaxTextOffset());
}

TEST_F(AXPositionTest, GetMaxTextOffsetFromButton) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(0, text_position->MaxTextOffset());
}

TEST_F(AXPositionTest, GetMaxTextOffsetFromCheckbox) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(0, text_position->MaxTextOffset());
}

TEST_F(AXPositionTest, GetMaxTextOffsetFromTextfield) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(13, text_position->MaxTextOffset());
}

TEST_F(AXPositionTest, GetMaxTextOffsetFromStaticText) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(6, text_position->MaxTextOffset());
}

TEST_F(AXPositionTest, GetMaxTextOffsetFromInlineTextBox) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(6, text_position->MaxTextOffset());
}

TEST_F(AXPositionTest, GetMaxTextOffsetFromLineBreak) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(1, text_position->MaxTextOffset());
}

TEST_F(AXPositionTest, GetMaxTextOffsetUpdate) {
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData text_data;
  text_data.id = 2;
  text_data.role = ax::mojom::Role::kStaticText;
  text_data.SetName("some text");

  AXNodeData more_text_data;
  more_text_data.id = 3;
  more_text_data.role = ax::mojom::Role::kStaticText;
  more_text_data.SetName("more text");

  root_data.child_ids = {2, 3};

  SetTree(CreateAXTree({root_data, text_data, more_text_data}));

  AssertTextLengthEquals(GetTree(), text_data.id, 9);
  AssertTextLengthEquals(GetTree(), root_data.id, 18);

  text_data.SetName("Adjusted line 1");
  SetTree(CreateAXTree({root_data, text_data, more_text_data}));

  AssertTextLengthEquals(GetTree(), text_data.id, 15);
  AssertTextLengthEquals(GetTree(), root_data.id, 24);

  // Value should override name
  text_data.SetValue("Value should override name");
  SetTree(CreateAXTree({root_data, text_data, more_text_data}));

  AssertTextLengthEquals(GetTree(), text_data.id, 26);
  AssertTextLengthEquals(GetTree(), root_data.id, 35);

  // An empty value should fall back to name
  text_data.SetValue("");
  SetTree(CreateAXTree({root_data, text_data, more_text_data}));

  AssertTextLengthEquals(GetTree(), text_data.id, 15);
  AssertTextLengthEquals(GetTree(), root_data.id, 24);
}

TEST_F(AXPositionTest, GetMaxTextOffsetAndGetTextWithGeneratedContent) {
  // ++1 kRootWebArea
  // ++++2 kTextField
  // ++++++3 kStaticText
  // ++++++++4 kInlineTextBox
  // ++++++5 kStaticText
  // ++++++++6 kInlineTextBox
  AXNodeData root_1;
  AXNodeData text_field_2;
  AXNodeData static_text_3;
  AXNodeData inline_box_4;
  AXNodeData static_text_5;
  AXNodeData inline_box_6;

  root_1.id = 1;
  text_field_2.id = 2;
  static_text_3.id = 3;
  inline_box_4.id = 4;
  static_text_5.id = 5;
  inline_box_6.id = 6;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {text_field_2.id};

  text_field_2.role = ax::mojom::Role::kGroup;
  text_field_2.SetValue("3.14");
  text_field_2.child_ids = {static_text_3.id, static_text_5.id};

  static_text_3.role = ax::mojom::Role::kStaticText;
  static_text_3.SetName("Placeholder from generated content");
  static_text_3.child_ids = {inline_box_4.id};

  inline_box_4.role = ax::mojom::Role::kInlineTextBox;
  inline_box_4.SetName("Placeholder from generated content");

  static_text_5.role = ax::mojom::Role::kStaticText;
  static_text_5.SetName("3.14");
  static_text_5.child_ids = {inline_box_6.id};

  inline_box_6.role = ax::mojom::Role::kInlineTextBox;
  inline_box_6.SetName("3.14");

  SetTree(CreateAXTree({root_1, text_field_2, static_text_3, inline_box_4,
                        static_text_5, inline_box_6}));

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->IsTextPosition());
  EXPECT_EQ(38, text_position->MaxTextOffset());
  EXPECT_EQ(u"Placeholder from generated content3.14",
            text_position->GetText());
}

TEST_F(AXPositionTest, AtStartOfAnchorWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  EXPECT_FALSE(null_position->AtStartOfAnchor());
}

TEST_F(AXPositionTest, AtStartOfAnchorWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  EXPECT_TRUE(tree_position->AtStartOfAnchor());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                     1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  EXPECT_FALSE(tree_position->AtStartOfAnchor());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                     3 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  EXPECT_FALSE(tree_position->AtStartOfAnchor());

  // A "before text" position.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_NE(nullptr, tree_position);
  EXPECT_TRUE(tree_position->AtStartOfAnchor());

  // An "after text" position.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  EXPECT_FALSE(tree_position->AtStartOfAnchor());
}

TEST_F(AXPositionTest, AtStartOfAnchorWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfAnchor());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfAnchor());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfAnchor());
}

TEST_F(AXPositionTest, AtEndOfAnchorWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  EXPECT_FALSE(null_position->AtEndOfAnchor());
}

TEST_F(AXPositionTest, AtEndOfAnchorWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 3 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  EXPECT_TRUE(tree_position->AtEndOfAnchor());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                     2 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  EXPECT_FALSE(tree_position->AtEndOfAnchor());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                     0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  EXPECT_FALSE(tree_position->AtEndOfAnchor());
}

TEST_F(AXPositionTest, AtEndOfAnchorWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtEndOfAnchor());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtEndOfAnchor());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtEndOfAnchor());
}

TEST_F(AXPositionTest, AtStartOfLineWithTextPosition) {
  // An upstream affinity should not affect the outcome since there is no soft
  // line break.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfLine());

  // An "after text" position anchored at the line break should be equivalent to
  // a "before text" position at the start of the next line.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfLine());

  // An upstream affinity should not affect the outcome since there is no soft
  // line break.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfLine());
}

TEST_F(AXPositionTest, AtStartOfLineStaticTextExtraPrecedingSpace) {
  // Consider the following web content:
  //   <style>
  //     .required-label::after {
  //       content: " *";
  //     }
  //   </style>
  //   <label class="required-label">Required </label>
  //
  // Which has the following AXTree, where the static text (#3)
  // contains an extra preceding space compared to its inline text (#4).
  // ++1 kRootWebArea
  // ++++2 kLabelText
  // ++++++3 kStaticText      name=" *"
  // ++++++++4 kInlineTextBox name="*"
  // This test ensures that this difference between static text and its inline
  // text box does not cause a hang when AtStartOfLine is called on static text
  // with text position " <*>".

  AXNodeData root;
  root.id = 1;
  root.role = ax::mojom::Role::kRootWebArea;
  // "kIsLineBreakingObject" is not strictly necessary but is added for
  // completeness.
  root.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
  AXNodeData label_text;
  label_text.id = 2;
  label_text.role = ax::mojom::Role::kLabelText;

  AXNodeData static_text1;
  static_text1.id = 3;
  static_text1.role = ax::mojom::Role::kStaticText;
  static_text1.SetName(" *");

  AXNodeData inline_text1;
  inline_text1.id = 4;
  inline_text1.role = ax::mojom::Role::kInlineTextBox;
  inline_text1.SetName("*");

  static_text1.child_ids = {inline_text1.id};
  root.child_ids = {static_text1.id};

  SetTree(CreateAXTree({root, static_text1, inline_text1}));

  // Calling AtStartOfLine on |static_text1| with position " <*>",
  // text_offset_=1, should not get into an infinite loop; it should be
  // guaranteed to terminate.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1.id, 1 /* child_index */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_FALSE(text_position->AtStartOfLine());
}

TEST_F(AXPositionTest, AtEndOfLineWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtEndOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtEndOfLine());

  // A "before text" position anchored at the line break should visually be the
  // same as a text position at the end of the previous line.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtEndOfLine());

  // The following position comes after the soft line break, so it should not be
  // marked as the end of the line.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtEndOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtEndOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtEndOfLine());
}

TEST_F(AXPositionTest, AtStartOfBlankLine) {
  // Modify the test tree so that the line break will appear on a line of its
  // own, i.e. as creating a blank line.
  inline_box1_.RemoveIntAttribute(ax::mojom::IntAttribute::kNextOnLineId);
  line_break_.RemoveIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId);
  AXTreeUpdate update;
  update.nodes = {inline_box1_, line_break_};
  ASSERT_TRUE(GetTree()->Unserialize(update));

  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());
  EXPECT_TRUE(tree_position->AtStartOfLine());

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfLine());

  // A text position after a blank line should be equivalent to a "before text"
  // position at the line that comes after it.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfLine());
}

TEST_F(AXPositionTest, AtEndOfBlankLine) {
  // Modify the test tree so that the line break will appear on a line of its
  // own, i.e. as creating a blank line.
  inline_box1_.RemoveIntAttribute(ax::mojom::IntAttribute::kNextOnLineId);
  line_break_.RemoveIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId);
  AXTreeUpdate update;
  update.nodes = {inline_box1_, line_break_};
  ASSERT_TRUE(GetTree()->Unserialize(update));

  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());
  EXPECT_FALSE(tree_position->AtEndOfLine());

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtEndOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtEndOfLine());
}

TEST_F(AXPositionTest, AtStartAndEndOfLineWhenAtEndOfTextSpan) {
  // This test ensures that the "AtStartOfLine" and the "AtEndOfLine" methods
  // return false and true respectively when we are at the end of a text span.
  //
  // A text span is defined by a series of inline text boxes that make up a
  // single static text object. Lines always end at the end of static text
  // objects, so there would never arise a situation when a position at the end
  // of a text span would be at start of line. It should always be at end of
  // line. On the contrary, if a position is at the end of an inline text box
  // and the equivalent parent position is in the middle of a static text
  // object, then the position would sometimes be at start of line, i.e., when
  // the inline text box contains only white space that is used to separate
  // lines in the case of lines being wrapped by a soft line break.
  //
  // Example accessibility tree:
  // 0:kRootWebArea
  // ++1:kStaticText "Hello testing "
  // ++++2:kInlineTextBox "Hello" kNextOnLine=2
  // ++++3:kInlineTextBox " " kPreviousOnLine=2
  // ++++4:kInlineTextBox "testing" kNextOnLine=5
  // ++++5:kInlineTextBox " " kPreviousOnLine=4
  // ++6:kStaticText "here."
  // ++++7:kInlineTextBox "here."
  //
  // Resulting text representation:
  // "Hello<soft_line_break>testing <hard_line_break>here."
  // Notice the extra space after the word "testing". This is not a line break.
  // The hard line break is caused by the presence of the second static text
  // object.
  //
  // A position at the end of inline text box 3 should be at start of line,
  // whilst a position at the end of inline text box 5 should not.

  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;
  // "kIsLineBreakingObject" is not strictly necessary but is added for
  // completeness.
  root_data.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                             true);

  AXNodeData static_text_data_1;
  static_text_data_1.id = 2;
  static_text_data_1.role = ax::mojom::Role::kStaticText;
  static_text_data_1.SetName("Hello testing ");

  AXNodeData inline_box_data_1;
  inline_box_data_1.id = 3;
  inline_box_data_1.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_1.SetName("hello");

  AXNodeData inline_box_data_2;
  inline_box_data_2.id = 4;
  inline_box_data_2.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_1.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                                    inline_box_data_2.id);
  inline_box_data_2.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                                    inline_box_data_1.id);
  // The name is a space character that we assume it turns into a soft line
  // break by the layout engine.
  inline_box_data_2.SetName(" ");

  AXNodeData inline_box_data_3;
  inline_box_data_3.id = 5;
  inline_box_data_3.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_3.SetName("testing");

  AXNodeData inline_box_data_4;
  inline_box_data_4.id = 6;
  inline_box_data_4.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_3.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                                    inline_box_data_4.id);
  inline_box_data_4.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                                    inline_box_data_3.id);
  inline_box_data_4.SetName(" ");  // Just a space character - not a line break.

  AXNodeData static_text_data_2;
  static_text_data_2.id = 7;
  static_text_data_2.role = ax::mojom::Role::kStaticText;
  static_text_data_2.SetName("here.");

  AXNodeData inline_box_data_5;
  inline_box_data_5.id = 8;
  inline_box_data_5.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_5.SetName("here.");

  static_text_data_1.child_ids = {inline_box_data_1.id, inline_box_data_2.id,
                                  inline_box_data_3.id, inline_box_data_4.id};
  static_text_data_2.child_ids = {inline_box_data_5.id};
  root_data.child_ids = {static_text_data_1.id, static_text_data_2.id};

  SetTree(CreateAXTree({root_data, static_text_data_1, inline_box_data_1,
                        inline_box_data_2, inline_box_data_3, inline_box_data_4,
                        static_text_data_2, inline_box_data_5}));

  // An "after text" tree position - after the soft line break.
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box_data_2.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());
  EXPECT_TRUE(tree_position->AtStartOfLine());
  EXPECT_FALSE(tree_position->AtEndOfLine());

  // An "after text" tree position - after the space character and before the
  // hard line break caused by the second static text object.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box_data_4.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());
  EXPECT_FALSE(tree_position->AtStartOfLine());
  EXPECT_TRUE(tree_position->AtEndOfLine());

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_data_2.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfLine());
  EXPECT_FALSE(text_position->AtEndOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_data_4.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfLine());
  EXPECT_TRUE(text_position->AtEndOfLine());
}

TEST_F(AXPositionTest, AtStartAndEndOfLineInsideTextField) {
  // This test ensures that "AtStart/EndOfLine" methods work properly when at
  // the start or end of a text field.
  //
  // We set up a test tree with two text fields. The first one has one line of
  // text, and the second one three. There are inline text boxes containing only
  // white space at the start and end of both text fields, which is a valid
  // AXTree that might be generated by our renderer.
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;
  // "kIsLineBreakingObject" is not strictly necessary but is added for
  // completeness.
  root_data.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                             true);

  AXNodeData text_field_data_1;
  text_field_data_1.id = 2;
  text_field_data_1.role = ax::mojom::Role::kGroup;
  // "kIsLineBreakingObject" and the "kEditable" state are not strictly
  // necessary but are added for completeness.
  text_field_data_1.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
  text_field_data_1.AddState(ax::mojom::State::kEditable);
  // Notice that there is one space at the start and one at the end of the text
  // field's value.
  text_field_data_1.SetValue(" Text field one ");

  AXNodeData static_text_data_1;
  static_text_data_1.id = 3;
  static_text_data_1.role = ax::mojom::Role::kStaticText;
  static_text_data_1.SetName(" Text field one ");

  AXNodeData inline_box_data_1;
  inline_box_data_1.id = 4;
  inline_box_data_1.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_1.SetName(" ");

  AXNodeData inline_box_data_2;
  inline_box_data_2.id = 5;
  inline_box_data_2.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_1.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                                    inline_box_data_2.id);
  inline_box_data_2.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                                    inline_box_data_1.id);
  inline_box_data_2.SetName("Text field one");

  AXNodeData inline_box_data_3;
  inline_box_data_3.id = 6;
  inline_box_data_3.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_2.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                                    inline_box_data_3.id);
  inline_box_data_3.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                                    inline_box_data_2.id);
  inline_box_data_3.SetName(" ");

  AXNodeData text_field_data_2;
  text_field_data_2.id = 7;
  text_field_data_2.role = ax::mojom::Role::kGroup;
  // "kIsLineBreakingObject" and the "kEditable" state are not strictly
  // necessary but are added for completeness.
  text_field_data_2.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
  text_field_data_1.AddState(ax::mojom::State::kEditable);
  // Notice that there are three lines, the first and the last one include only
  // a single space.
  text_field_data_2.SetValue(" Text field two ");

  AXNodeData static_text_data_2;
  static_text_data_2.id = 8;
  static_text_data_2.role = ax::mojom::Role::kStaticText;
  static_text_data_2.SetName(" Text field two ");

  AXNodeData inline_box_data_4;
  inline_box_data_4.id = 9;
  inline_box_data_4.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_4.SetName(" ");

  AXNodeData inline_box_data_5;
  inline_box_data_5.id = 10;
  inline_box_data_5.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_5.SetName("Text field two");

  AXNodeData inline_box_data_6;
  inline_box_data_6.id = 11;
  inline_box_data_6.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_6.SetName(" ");

  static_text_data_1.child_ids = {inline_box_data_1.id, inline_box_data_2.id,
                                  inline_box_data_3.id};
  static_text_data_2.child_ids = {inline_box_data_4.id, inline_box_data_5.id,
                                  inline_box_data_6.id};
  text_field_data_1.child_ids = {static_text_data_1.id};
  text_field_data_2.child_ids = {static_text_data_2.id};
  root_data.child_ids = {text_field_data_1.id, text_field_data_2.id};

  SetTree(
      CreateAXTree({root_data, text_field_data_1, static_text_data_1,
                    inline_box_data_1, inline_box_data_2, inline_box_data_3,
                    text_field_data_2, static_text_data_2, inline_box_data_4,
                    inline_box_data_5, inline_box_data_6}));

  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_data_1.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());
  EXPECT_TRUE(tree_position->AtStartOfLine());
  EXPECT_FALSE(tree_position->AtEndOfLine());

  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_data_1.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());
  EXPECT_FALSE(tree_position->AtStartOfLine());
  EXPECT_TRUE(tree_position->AtEndOfLine());

  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_data_2.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());
  EXPECT_TRUE(tree_position->AtStartOfLine());
  EXPECT_FALSE(tree_position->AtEndOfLine());

  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_data_2.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());
  EXPECT_FALSE(tree_position->AtStartOfLine());
  EXPECT_TRUE(tree_position->AtEndOfLine());

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_data_1.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfLine());
  EXPECT_FALSE(text_position->AtEndOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_data_1.id, 16 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfLine());
  EXPECT_TRUE(text_position->AtEndOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_data_2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfLine());
  EXPECT_FALSE(text_position->AtEndOfLine());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_data_2.id, 16 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfLine());
  EXPECT_TRUE(text_position->AtEndOfLine());
}

TEST_F(AXPositionTest, AtStartOfParagraphWithTextPosition) {
  // An upstream affinity should not affect the outcome since there is no soft
  // line break.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfParagraph());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfParagraph());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfParagraph());

  // An "after text" position anchored at the line break should not be the same
  // as a text position at the start of the next paragraph because in practice
  // they should have resulted from two different ancestor positions. The former
  // should have been an upstream position, whilst the latter a downstream one.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfParagraph());

  // An upstream affinity should not affect the outcome since there is no soft
  // line break.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtStartOfParagraph());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtStartOfParagraph());
}

TEST_F(AXPositionTest, AtEndOfParagraphWithTextPosition) {
  // End of |inline_box1_| is not the end of paragraph since it's
  // followed by a whitespace-only line breaking object
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // The start of |line_break_| is not the end of paragraph since it's
  // not the end of its anchor.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // The end of |line_break_| is the end of paragraph since it's
  // a line breaking object without additional trailing whitespace.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtEndOfParagraph());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // The end of |inline_box2_| is the end of paragraph since it's
  // followed by the end of document.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->AtEndOfParagraph());
}

TEST_F(AXPositionTest, ParagraphEdgesWithPreservedNewLine) {
  // This test ensures that "At{Start|End}OfParagraph" work correctly when a
  // text position is on a preserved newline character.
  //
  // Newline characters are used to separate paragraphs. If there is a series of
  // newline characters, a paragraph should start after the last newline
  // character.
  // ++1 kRootWebArea isLineBreakingObject
  // ++++2 kStaticText "some text"
  // ++++++3 kInlineTextBox "some text"
  // ++++4 kGenericContainer isLineBreakingObject
  // ++++++5 kStaticText "\nmore text"
  // ++++++++6 kInlineTextBox "\n" isLineBreakingObject
  // ++++++++7 kInlineTextBox "more text"

  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;
  root_data.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                             true);

  AXNodeData static_text_data_1;
  static_text_data_1.id = 2;
  static_text_data_1.role = ax::mojom::Role::kStaticText;
  static_text_data_1.SetName("some text");

  AXNodeData some_text_data;
  some_text_data.id = 3;
  some_text_data.role = ax::mojom::Role::kInlineTextBox;
  some_text_data.SetName("some text");

  AXNodeData container_data;
  container_data.id = 4;
  container_data.role = ax::mojom::Role::kGenericContainer;
  container_data.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData static_text_data_2;
  static_text_data_2.id = 5;
  static_text_data_2.role = ax::mojom::Role::kStaticText;
  static_text_data_2.SetName("\nmore text");

  AXNodeData preserved_newline_data;
  preserved_newline_data.id = 6;
  preserved_newline_data.role = ax::mojom::Role::kInlineTextBox;
  preserved_newline_data.SetName("\n");
  preserved_newline_data.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData more_text_data;
  more_text_data.id = 7;
  more_text_data.role = ax::mojom::Role::kInlineTextBox;
  more_text_data.SetName("more text");

  static_text_data_1.child_ids = {some_text_data.id};
  container_data.child_ids = {static_text_data_2.id};
  static_text_data_2.child_ids = {preserved_newline_data.id, more_text_data.id};
  root_data.child_ids = {static_text_data_1.id, container_data.id};

  SetTree(CreateAXTree({root_data, static_text_data_1, some_text_data,
                        container_data, static_text_data_2,
                        preserved_newline_data, more_text_data}));

  // Text position "some tex<t>\nmore text".
  TestPositionType text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 8 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position1->AtEndOfParagraph());
  EXPECT_FALSE(text_position1->AtStartOfParagraph());

  // Text position "some text<\n>more text".
  TestPositionType text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 9 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position2->AtEndOfParagraph());
  EXPECT_FALSE(text_position2->AtStartOfParagraph());

  // Text position "some text<\n>more text".
  TestPositionType text_position3 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 9 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  EXPECT_FALSE(text_position3->AtEndOfParagraph());
  EXPECT_FALSE(text_position3->AtStartOfParagraph());

  // Text position "some text\n<m>ore text".
  TestPositionType text_position4 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 10 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position4->AtEndOfParagraph());
  EXPECT_TRUE(text_position4->AtStartOfParagraph());

  // Text position "some text\n<m>ore text".
  TestPositionType text_position5 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 10 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  EXPECT_TRUE(text_position5->AtEndOfParagraph());
  EXPECT_FALSE(text_position5->AtStartOfParagraph());

  // Text position "<\n>more text".
  TestPositionType text_position6 = AXNodePosition::CreateTextPosition(
      GetTreeID(), container_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position6->AtEndOfParagraph());
  EXPECT_FALSE(text_position6->AtStartOfParagraph());

  // Text position "\n<m>ore text".
  TestPositionType text_position7 = AXNodePosition::CreateTextPosition(
      GetTreeID(), container_data.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position7->AtEndOfParagraph());
  EXPECT_TRUE(text_position7->AtStartOfParagraph());

  // Text position "\n<m>ore text".
  TestPositionType text_position8 = AXNodePosition::CreateTextPosition(
      GetTreeID(), container_data.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  EXPECT_TRUE(text_position8->AtEndOfParagraph());
  EXPECT_FALSE(text_position8->AtStartOfParagraph());

  // Text position "\n<m>ore text".
  TestPositionType text_position9 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text_data_2.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position9->AtEndOfParagraph());
  EXPECT_TRUE(text_position9->AtStartOfParagraph());

  // Text position "\n<m>ore text".
  TestPositionType text_position10 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text_data_2.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  EXPECT_TRUE(text_position10->AtEndOfParagraph());
  EXPECT_FALSE(text_position10->AtStartOfParagraph());

  TestPositionType text_position11 = AXNodePosition::CreateTextPosition(
      GetTreeID(), preserved_newline_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position11->AtEndOfParagraph());
  EXPECT_FALSE(text_position11->AtStartOfParagraph());

  TestPositionType text_position12 = AXNodePosition::CreateTextPosition(
      GetTreeID(), preserved_newline_data.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_TRUE(text_position12->AtEndOfParagraph());
  EXPECT_FALSE(text_position12->AtStartOfParagraph());

  TestPositionType text_position13 = AXNodePosition::CreateTextPosition(
      GetTreeID(), more_text_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position13->AtEndOfParagraph());
  EXPECT_TRUE(text_position13->AtStartOfParagraph());

  TestPositionType text_position14 = AXNodePosition::CreateTextPosition(
      GetTreeID(), more_text_data.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position14->AtEndOfParagraph());
  EXPECT_FALSE(text_position14->AtStartOfParagraph());
}

TEST_F(
    AXPositionTest,
    PreviousParagraphEndStopAtAnchorBoundaryWithConsecutiveParentChildLineBreakingObjects) {
  // This test updates the tree structure to test a specific edge case -
  // CreatePreviousParagraphEndPosition(), stopping at an anchor boundary,
  // with consecutive parent-child line breaking objects.
  // ++1 rootWebArea
  // ++++2 staticText name="first"
  // ++++3 genericContainer isLineBreakingObject
  // ++++++4 genericContainer isLineBreakingObject
  // ++++++5 staticText name="second"
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData static_text_data_a;
  static_text_data_a.id = 2;
  static_text_data_a.role = ax::mojom::Role::kStaticText;
  static_text_data_a.SetName("first");

  AXNodeData container_data_a;
  container_data_a.id = 3;
  container_data_a.role = ax::mojom::Role::kGenericContainer;
  container_data_a.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData container_data_b;
  container_data_b.id = 4;
  container_data_b.role = ax::mojom::Role::kGenericContainer;
  container_data_b.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData static_text_data_b;
  static_text_data_b.id = 5;
  static_text_data_b.role = ax::mojom::Role::kStaticText;
  static_text_data_b.SetName("second");

  root_data.child_ids = {static_text_data_a.id, container_data_a.id};
  container_data_a.child_ids = {container_data_b.id, static_text_data_b.id};

  SetTree(CreateAXTree({root_data, static_text_data_a, container_data_a,
                        container_data_b, static_text_data_b}));

  TestPositionType test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 11 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  test_position = test_position->CreatePreviousParagraphEndPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(root_data.id, test_position->anchor_id());
  EXPECT_EQ(5, test_position->text_offset());
}

TEST_F(AXPositionTest, AtStartOrEndOfParagraphOnAListMarker) {
  // "AtStartOfParagraph" should return true before a list marker, either a
  // Legacy Layout or an NG Layout one. It should return false on the next
  // sibling of the list marker, i.e., before the list item's actual text
  // contents.
  //
  // There are two list markers in the following test tree. The first one is a
  // Legacy Layout one and the second an NG Layout one.
  // ++1 kRootWebArea
  // ++++2 kStaticText "Before list."
  // ++++++3 kInlineTextBox "Before list."
  // ++++4 kList
  // ++++++5 kListItem
  // ++++++++6 kListMarker
  // ++++++++++7 kStaticText "1. "
  // ++++++++++++8 kInlineTextBox "1. "
  // ++++++++9 kStaticText "First item."
  // ++++++++++10 kInlineTextBox "First item."
  // ++++++11 kListItem
  // ++++++++12 kListMarker "2. "
  // ++++++++13 kStaticText "Second item."
  // ++++++++++14 kInlineTextBox "Second item."
  // ++15 kStaticText "After list."
  // ++++16 kInlineTextBox "After list."

  AXNodeData root;
  AXNodeData list;
  AXNodeData list_item1;
  AXNodeData list_item2;
  AXNodeData list_marker_legacy;
  AXNodeData list_marker_ng;
  AXNodeData static_text1;
  AXNodeData static_text2;
  AXNodeData static_text3;
  AXNodeData static_text4;
  AXNodeData static_text5;
  AXNodeData inline_box1;
  AXNodeData inline_box2;
  AXNodeData inline_box3;
  AXNodeData inline_box4;
  AXNodeData inline_box5;

  root.id = 1;
  static_text1.id = 2;
  inline_box1.id = 3;
  list.id = 4;
  list_item1.id = 5;
  list_marker_legacy.id = 6;
  static_text2.id = 7;
  inline_box2.id = 8;
  static_text3.id = 9;
  inline_box3.id = 10;
  list_item2.id = 11;
  list_marker_ng.id = 12;
  static_text4.id = 13;
  inline_box4.id = 14;
  static_text5.id = 15;
  inline_box5.id = 16;

  root.role = ax::mojom::Role::kRootWebArea;
  root.child_ids = {static_text1.id, list.id, static_text5.id};
  root.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  static_text1.role = ax::mojom::Role::kStaticText;
  static_text1.child_ids = {inline_box1.id};
  static_text1.SetName("Before list.");

  inline_box1.role = ax::mojom::Role::kInlineTextBox;
  inline_box1.SetName("Before list.");

  list.role = ax::mojom::Role::kList;
  list.child_ids = {list_item1.id, list_item2.id};

  list_item1.role = ax::mojom::Role::kListItem;
  list_item1.child_ids = {list_marker_legacy.id, static_text3.id};
  list_item1.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);

  list_marker_legacy.role = ax::mojom::Role::kListMarker;
  list_marker_legacy.child_ids = {static_text2.id};

  static_text2.role = ax::mojom::Role::kStaticText;
  static_text2.child_ids = {inline_box2.id};
  static_text2.SetName("1. ");

  inline_box2.role = ax::mojom::Role::kInlineTextBox;
  inline_box2.SetName("1. ");
  inline_box2.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                              inline_box3.id);

  static_text3.role = ax::mojom::Role::kStaticText;
  static_text3.child_ids = {inline_box3.id};
  static_text3.SetName("First item.");

  inline_box3.role = ax::mojom::Role::kInlineTextBox;
  inline_box3.SetName("First item.");
  inline_box3.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                              inline_box2.id);

  list_item2.role = ax::mojom::Role::kListItem;
  list_item2.child_ids = {list_marker_ng.id, static_text4.id};
  list_item2.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);

  list_marker_ng.role = ax::mojom::Role::kListMarker;
  list_marker_ng.SetName("2. ");
  list_marker_ng.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                                 inline_box4.id);

  static_text4.role = ax::mojom::Role::kStaticText;
  static_text4.child_ids = {inline_box4.id};
  static_text4.SetName("Second item.");

  inline_box4.role = ax::mojom::Role::kInlineTextBox;
  inline_box4.SetName("Second item.");
  inline_box4.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                              list_marker_ng.id);

  static_text5.role = ax::mojom::Role::kStaticText;
  static_text5.child_ids = {inline_box5.id};
  static_text5.SetName("After list.");

  inline_box5.role = ax::mojom::Role::kInlineTextBox;
  inline_box5.SetName("After list.");

  SetTree(CreateAXTree({root, static_text1, inline_box1, list, list_item1,
                        list_marker_legacy, static_text2, inline_box2,
                        static_text3, inline_box3, list_item2, list_marker_ng,
                        static_text4, inline_box4, static_text5, inline_box5}));

  // A text position after the text "Before list.". It should not be equivalent
  // to a position that is before the list itself, or before the first list
  // bullet / item.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1.id, 12 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_TRUE(text_position->AtEndOfParagraph());

  // A text position after the text "Before list.". It should not be equivalent
  // to a position that is before the list itself, or before the first list
  // bullet / item.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1.id, 12 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_TRUE(text_position->AtEndOfParagraph());

  // A text position before the list.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), list.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // A downstream text position after the list. It should resolve to a leaf
  // position before the paragraph that comes after the list, so it should be
  // "AtStartOfParagraph".
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), list.id, 14 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // An upstream text position after the list. It should be "AtEndOfParagraph".
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), list.id, 14 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_TRUE(text_position->AtEndOfParagraph());

  // A text position before the first list bullet (the Legacy Layout one).
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), list_marker_legacy.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), list_marker_legacy.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // A text position before the first list bullet (the Legacy Layout one).
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2.id, 2 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // A text position before the first list bullet (the Legacy Layout one).
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // A text position before the second list bullet (the NG Layout one).
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), list_marker_ng.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), list_marker_ng.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // A text position before the text contents of the first list item - not the
  // bullet.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text3.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // A text position before the text contents of the first list item - not the
  // bullet.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box3.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // A text position after the text contents of the first list item.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text3.id, 11 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_TRUE(text_position->AtEndOfParagraph());

  // A text position after the text contents of the first list item.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box3.id, 11 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_TRUE(text_position->AtEndOfParagraph());

  // A text position before the text contents of the second list item - not the
  // bullet.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text4.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // A text position before the text contents of the second list item - not the
  // bullet.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box4.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());

  // A text position after the text contents of the second list item.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text4.id, 12 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_TRUE(text_position->AtEndOfParagraph());

  // A text position after the text contents of the second list item.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box4.id, 12 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtStartOfParagraph());
  EXPECT_TRUE(text_position->AtEndOfParagraph());

  // A text position before the text "After list.".
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box5.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->AtStartOfParagraph());
  EXPECT_FALSE(text_position->AtEndOfParagraph());
}

TEST_F(AXPositionTest,
       AtStartOrEndOfParagraphWithLeadingAndTrailingDocumentWhitespace) {
  // This test ensures that "At{Start|End}OfParagraph" work correctly when a
  // text position is on a preserved newline character.
  //
  // Newline characters are used to separate paragraphs. If there is a series of
  // newline characters, a paragraph should start after the last newline
  // character.
  // ++1 kRootWebArea isLineBreakingObject
  // ++++2 kGenericContainer isLineBreakingObject
  // ++++++3 kStaticText "\n"
  // ++++++++4 kInlineTextBox "\n" isLineBreakingObject
  // ++++5 kGenericContainer isLineBreakingObject
  // ++++++6 kStaticText "some text"
  // ++++++++7 kInlineTextBox "some"
  // ++++++++8 kInlineTextBox " "
  // ++++++++9 kInlineTextBox "text"
  // ++++10 kGenericContainer isLineBreakingObject
  // ++++++11 kStaticText "\n"
  // ++++++++12 kInlineTextBox "\n" isLineBreakingObject

  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;
  root_data.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                             true);

  AXNodeData container_data_a;
  container_data_a.id = 2;
  container_data_a.role = ax::mojom::Role::kGenericContainer;
  container_data_a.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData static_text_data_a;
  static_text_data_a.id = 3;
  static_text_data_a.role = ax::mojom::Role::kStaticText;
  static_text_data_a.SetName("\n");

  AXNodeData inline_text_data_a;
  inline_text_data_a.id = 4;
  inline_text_data_a.role = ax::mojom::Role::kInlineTextBox;
  inline_text_data_a.SetName("\n");
  inline_text_data_a.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData container_data_b;
  container_data_b.id = 5;
  container_data_b.role = ax::mojom::Role::kGenericContainer;
  container_data_b.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData static_text_data_b;
  static_text_data_b.id = 6;
  static_text_data_b.role = ax::mojom::Role::kStaticText;
  static_text_data_b.SetName("some text");

  AXNodeData inline_text_data_b_1;
  inline_text_data_b_1.id = 7;
  inline_text_data_b_1.role = ax::mojom::Role::kInlineTextBox;
  inline_text_data_b_1.SetName("some");

  AXNodeData inline_text_data_b_2;
  inline_text_data_b_2.id = 8;
  inline_text_data_b_2.role = ax::mojom::Role::kInlineTextBox;
  inline_text_data_b_2.SetName(" ");

  AXNodeData inline_text_data_b_3;
  inline_text_data_b_3.id = 9;
  inline_text_data_b_3.role = ax::mojom::Role::kInlineTextBox;
  inline_text_data_b_3.SetName("text");

  AXNodeData container_data_c;
  container_data_c.id = 10;
  container_data_c.role = ax::mojom::Role::kGenericContainer;
  container_data_c.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData static_text_data_c;
  static_text_data_c.id = 11;
  static_text_data_c.role = ax::mojom::Role::kStaticText;
  static_text_data_c.SetName("\n");

  AXNodeData inline_text_data_c;
  inline_text_data_c.id = 12;
  inline_text_data_c.role = ax::mojom::Role::kInlineTextBox;
  inline_text_data_c.SetName("\n");
  inline_text_data_c.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  root_data.child_ids = {container_data_a.id, container_data_b.id,
                         container_data_c.id};
  container_data_a.child_ids = {static_text_data_a.id};
  static_text_data_a.child_ids = {inline_text_data_a.id};
  container_data_b.child_ids = {static_text_data_b.id};
  static_text_data_b.child_ids = {inline_text_data_b_1.id,
                                  inline_text_data_b_2.id,
                                  inline_text_data_b_3.id};
  container_data_c.child_ids = {static_text_data_c.id};
  static_text_data_c.child_ids = {inline_text_data_c.id};

  SetTree(CreateAXTree(
      {root_data, container_data_a, container_data_b, container_data_c,
       static_text_data_a, static_text_data_b, static_text_data_c,
       inline_text_data_a, inline_text_data_b_1, inline_text_data_b_2,
       inline_text_data_b_3, inline_text_data_c}));

  // Before the first "\n".
  TestPositionType text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_a.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position1->AtEndOfParagraph());
  EXPECT_TRUE(text_position1->AtStartOfParagraph());

  // After the first "\n".
  //
  // Since the position is an "after text" position, it is similar to pressing
  // the End key, (or Cmd-Right on Mac), while the caret is on the line break,
  // so it should not be "AtStartOfParagraph".
  TestPositionType text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_a.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_TRUE(text_position2->AtEndOfParagraph());
  EXPECT_FALSE(text_position2->AtStartOfParagraph());

  // Before "some".
  TestPositionType text_position3 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_1.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position3->AtEndOfParagraph());
  EXPECT_TRUE(text_position3->AtStartOfParagraph());

  // After "some".
  TestPositionType text_position4 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_1.id, 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position4->AtEndOfParagraph());
  EXPECT_FALSE(text_position4->AtStartOfParagraph());

  // Before " ".
  TestPositionType text_position5 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position5->AtEndOfParagraph());
  EXPECT_FALSE(text_position5->AtStartOfParagraph());

  // After " ".
  TestPositionType text_position6 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_2.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position6->AtEndOfParagraph());
  EXPECT_FALSE(text_position6->AtStartOfParagraph());

  // Before "text".
  TestPositionType text_position7 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_3.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position7->AtEndOfParagraph());
  EXPECT_FALSE(text_position7->AtStartOfParagraph());

  // After "text".
  TestPositionType text_position8 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_3.id, 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position8->AtEndOfParagraph());
  EXPECT_FALSE(text_position8->AtStartOfParagraph());

  // Before the second "\n".
  TestPositionType text_position9 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_c.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position9->AtEndOfParagraph());
  EXPECT_FALSE(text_position9->AtStartOfParagraph());

  // After the second "\n".
  TestPositionType text_position10 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_c.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_TRUE(text_position10->AtEndOfParagraph());
  EXPECT_FALSE(text_position10->AtStartOfParagraph());
}

TEST_F(AXPositionTest, AtStartOrEndOfParagraphWithIgnoredNodes) {
  // This test ensures that "At{Start|End}OfParagraph" work correctly when there
  // are ignored nodes present near a paragraph boundary.
  //
  // An ignored node that is between a given position and a paragraph boundary
  // should not be taken into consideration. The position should be interpreted
  // as being on the boundary.
  // ++1 kRootWebArea isLineBreakingObject
  // ++++2 kGenericContainer ignored isLineBreakingObject
  // ++++++3 kStaticText ignored "ignored text"
  // ++++++++4 kInlineTextBox ignored "ignored text"
  // ++++5 kGenericContainer isLineBreakingObject
  // ++++++6 kStaticText "some text"
  // ++++++++7 kInlineTextBox "some"
  // ++++++++8 kInlineTextBox " "
  // ++++++++9 kInlineTextBox "text"
  // ++++10 kGenericContainer ignored isLineBreakingObject
  // ++++++11 kStaticText ignored "ignored text"
  // ++++++++12 kInlineTextBox ignored "ignored text"

  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;
  root_data.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                             true);

  AXNodeData container_data_a;
  container_data_a.id = 2;
  container_data_a.role = ax::mojom::Role::kGenericContainer;
  container_data_a.AddState(ax::mojom::State::kIgnored);
  container_data_a.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData static_text_data_a;
  static_text_data_a.id = 3;
  static_text_data_a.role = ax::mojom::Role::kStaticText;
  static_text_data_a.SetName("ignored text");
  static_text_data_a.AddState(ax::mojom::State::kIgnored);

  AXNodeData inline_text_data_a;
  inline_text_data_a.id = 4;
  inline_text_data_a.role = ax::mojom::Role::kInlineTextBox;
  inline_text_data_a.SetName("ignored text");
  inline_text_data_a.AddState(ax::mojom::State::kIgnored);

  AXNodeData container_data_b;
  container_data_b.id = 5;
  container_data_b.role = ax::mojom::Role::kGenericContainer;
  container_data_b.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData static_text_data_b;
  static_text_data_b.id = 6;
  static_text_data_b.role = ax::mojom::Role::kStaticText;
  static_text_data_b.SetName("some text");

  AXNodeData inline_text_data_b_1;
  inline_text_data_b_1.id = 7;
  inline_text_data_b_1.role = ax::mojom::Role::kInlineTextBox;
  inline_text_data_b_1.SetName("some");

  AXNodeData inline_text_data_b_2;
  inline_text_data_b_2.id = 8;
  inline_text_data_b_2.role = ax::mojom::Role::kInlineTextBox;
  inline_text_data_b_2.SetName(" ");

  AXNodeData inline_text_data_b_3;
  inline_text_data_b_3.id = 9;
  inline_text_data_b_3.role = ax::mojom::Role::kInlineTextBox;
  inline_text_data_b_3.SetName("text");

  AXNodeData container_data_c;
  container_data_c.id = 10;
  container_data_c.role = ax::mojom::Role::kGenericContainer;
  container_data_c.AddState(ax::mojom::State::kIgnored);
  container_data_c.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  AXNodeData static_text_data_c;
  static_text_data_c.id = 11;
  static_text_data_c.role = ax::mojom::Role::kStaticText;
  static_text_data_c.SetName("ignored text");
  static_text_data_c.AddState(ax::mojom::State::kIgnored);

  AXNodeData inline_text_data_c;
  inline_text_data_c.id = 12;
  inline_text_data_c.role = ax::mojom::Role::kInlineTextBox;
  inline_text_data_c.SetName("ignored text");
  inline_text_data_c.AddState(ax::mojom::State::kIgnored);

  root_data.child_ids = {container_data_a.id, container_data_b.id,
                         container_data_c.id};
  container_data_a.child_ids = {static_text_data_a.id};
  static_text_data_a.child_ids = {inline_text_data_a.id};
  container_data_b.child_ids = {static_text_data_b.id};
  static_text_data_b.child_ids = {inline_text_data_b_1.id,
                                  inline_text_data_b_2.id,
                                  inline_text_data_b_3.id};
  container_data_c.child_ids = {static_text_data_c.id};
  static_text_data_c.child_ids = {inline_text_data_c.id};

  SetTree(CreateAXTree(
      {root_data, container_data_a, container_data_b, container_data_c,
       static_text_data_a, static_text_data_b, static_text_data_c,
       inline_text_data_a, inline_text_data_b_1, inline_text_data_b_2,
       inline_text_data_b_3, inline_text_data_c}));

  // Before "ignored text".
  TestPositionType text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_a.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position1->AtEndOfParagraph());
  EXPECT_FALSE(text_position1->AtStartOfParagraph());

  // After "ignored text".
  //
  // Since the position is an "after text" position, it is similar to pressing
  // the End key, (or Cmd-Right on Mac), while the caret is on "ignored text",
  // so it should not be "AtStartOfParagraph". In practice, this situation
  // should not arise in accessibility, because the node is ignored.
  TestPositionType text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_a.id, 12 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position2->AtEndOfParagraph());
  EXPECT_FALSE(text_position2->AtStartOfParagraph());

  // Before "some".
  TestPositionType text_position3 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_1.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position3->AtEndOfParagraph());
  EXPECT_TRUE(text_position3->AtStartOfParagraph());

  // After "some".
  TestPositionType text_position4 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_1.id, 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position4->AtEndOfParagraph());
  EXPECT_FALSE(text_position4->AtStartOfParagraph());

  // Before " ".
  TestPositionType text_position5 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position5->AtEndOfParagraph());
  EXPECT_FALSE(text_position5->AtStartOfParagraph());

  // After " ".
  TestPositionType text_position6 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_2.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position6->AtEndOfParagraph());
  EXPECT_FALSE(text_position6->AtStartOfParagraph());

  // Before "text".
  TestPositionType text_position7 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_3.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position7->AtEndOfParagraph());
  EXPECT_FALSE(text_position7->AtStartOfParagraph());

  // After "text".
  TestPositionType text_position8 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_b_3.id, 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_TRUE(text_position8->AtEndOfParagraph());
  EXPECT_FALSE(text_position8->AtStartOfParagraph());

  // Before "ignored text" - the second version.
  TestPositionType text_position9 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_c.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position9->AtEndOfParagraph());
  EXPECT_FALSE(text_position9->AtStartOfParagraph());

  // After "ignored text" - the second version.
  TestPositionType text_position10 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_text_data_c.id, 12 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position10->AtEndOfParagraph());
  EXPECT_FALSE(text_position10->AtStartOfParagraph());
}

TEST_F(AXPositionTest, AtStartOrEndOfParagraphWithEmbeddedObjectCharacter) {
  g_ax_embedded_object_behavior = AXEmbeddedObjectBehavior::kExposeCharacter;

  // This test ensures that "At{Start|End}OfParagraph" work correctly when there
  // are embedded objects present near a paragraph boundary.
  //
  // Nodes represented by an embedded object character, such as a plain text
  // field or a check box, should create an implicit paragraph boundary for
  // assistive software.
  // ++1 kRootWebArea isLineBreakingObject
  // ++++2 kLink
  // ++++++3 kStaticText "hello"
  // ++++++++4 kInlineTextBox "hello"
  // ++++++5 kImage
  // ++++++6 kStaticText "world"
  // ++++++++7 kInlineTextBox "world"

  AXNodeData root_1;
  AXNodeData link_2;
  AXNodeData static_text_3;
  AXNodeData inline_box_4;
  AXNodeData image_5;
  AXNodeData static_text_6;
  AXNodeData inline_box_7;

  root_1.id = 1;
  link_2.id = 2;
  static_text_3.id = 3;
  inline_box_4.id = 4;
  image_5.id = 5;
  static_text_6.id = 6;
  inline_box_7.id = 7;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {link_2.id};
  root_1.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                          true);

  link_2.role = ax::mojom::Role::kLink;
  link_2.child_ids = {static_text_3.id, image_5.id, static_text_6.id};

  static_text_3.role = ax::mojom::Role::kStaticText;
  static_text_3.child_ids = {inline_box_4.id};
  static_text_3.SetName("Hello");

  inline_box_4.role = ax::mojom::Role::kInlineTextBox;
  inline_box_4.SetName("Hello");

  image_5.role = ax::mojom::Role::kImage;
  // The image's inner text should be an embedded object character.

  static_text_6.role = ax::mojom::Role::kStaticText;
  static_text_6.child_ids = {inline_box_7.id};
  static_text_6.SetName("world");

  inline_box_7.role = ax::mojom::Role::kInlineTextBox;
  inline_box_7.SetName("world");

  SetTree(CreateAXTree({root_1, link_2, static_text_3, inline_box_4, image_5,
                        static_text_6, inline_box_7}));

  // Before "hello".
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_4.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position->AtEndOfParagraph());
  EXPECT_TRUE(text_position->AtStartOfParagraph());

  // After "hello".
  //
  // Note that even though this position and a position before the image's
  // embedded object character are conceptually equivalent, in practice they
  // should result from two different ancestor positions. The former should have
  // been an upstream position, whilst the latter a downstream one.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_4.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_TRUE(text_position->AtEndOfParagraph());
  EXPECT_FALSE(text_position->AtStartOfParagraph());

  // Before the image's embedded object character.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), image_5.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position->AtEndOfParagraph());
  EXPECT_TRUE(text_position->AtStartOfParagraph());

  // After the image's embedded object character.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), image_5.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_TRUE(text_position->AtEndOfParagraph());
  EXPECT_FALSE(text_position->AtStartOfParagraph());

  // Before "world".
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_7.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_FALSE(text_position->AtEndOfParagraph());
  EXPECT_TRUE(text_position->AtStartOfParagraph());

  // After "world".
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_7.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_TRUE(text_position->AtEndOfParagraph());
  EXPECT_FALSE(text_position->AtStartOfParagraph());
}

TEST_F(AXPositionTest, LowestCommonAncestor) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  // An "after children" position.
  TestPositionType root_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 3 /* child_index */);
  ASSERT_NE(nullptr, root_position);
  // A "before text" position.
  TestPositionType button_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), button_.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_NE(nullptr, button_position);
  TestPositionType text_field_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 2 /* child_index */);
  ASSERT_NE(nullptr, text_field_position);
  TestPositionType static_text1_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), static_text1_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, static_text1_position);
  TestPositionType static_text2_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), static_text2_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, static_text2_position);
  TestPositionType inline_box1_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, inline_box1_position);
  ASSERT_TRUE(inline_box1_position->IsTextPosition());
  TestPositionType inline_box2_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, inline_box2_position);
  ASSERT_TRUE(inline_box2_position->IsTextPosition());

  TestPositionType test_position =
      root_position->LowestCommonAncestor(*null_position.get());
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  test_position = root_position->LowestCommonAncestor(*root_position.get());
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(root_.id, test_position->anchor_id());
  // The child index should be for an "after children" position, i.e. it should
  // be unchanged.
  EXPECT_EQ(3, test_position->child_index());

  test_position =
      button_position->LowestCommonAncestor(*text_field_position.get());
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(root_.id, test_position->anchor_id());
  // The child index should point to the button.
  EXPECT_EQ(0, test_position->child_index());

  test_position =
      static_text2_position->LowestCommonAncestor(*static_text1_position.get());
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  // The child index should point to the second static text node.
  EXPECT_EQ(2, test_position->child_index());

  test_position =
      static_text1_position->LowestCommonAncestor(*text_field_position.get());
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  // The child index should point to the first static text node.
  EXPECT_EQ(0, test_position->child_index());

  test_position =
      inline_box1_position->LowestCommonAncestor(*inline_box2_position.get());
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position =
      inline_box2_position->LowestCommonAncestor(*inline_box1_position.get());
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  // The text offset should point to the second line.
  EXPECT_EQ(7, test_position->text_offset());
}

TEST_F(AXPositionTest, AsTreePositionWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->AsTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, AsTreePositionWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  TestPositionType test_position = tree_position->AsTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(root_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->child_index());
  EXPECT_EQ(AXNodePosition::INVALID_OFFSET, test_position->text_offset());
}

TEST_F(AXPositionTest, AsTreePositionWithTextPosition) {
  // Create a text position pointing to the last character in the text field.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 12 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position = text_position->AsTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  // The created tree position should point to the second static text node
  // inside the text field.
  EXPECT_EQ(2, test_position->child_index());
  // But its text offset should be unchanged.
  EXPECT_EQ(12, test_position->text_offset());

  // Test for a "before text" position.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());
  EXPECT_EQ(0, test_position->text_offset());

  // Test for an "after text" position.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());
  EXPECT_EQ(6, test_position->text_offset());
}

TEST_F(AXPositionTest, AsTextPositionWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->AsTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, AsTextPositionWithTreePosition) {
  // Create a tree position pointing to the line break node inside the text
  // field.
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  TestPositionType test_position = tree_position->AsTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  // The created text position should point to the 6th character inside the text
  // field, i.e. the line break.
  EXPECT_EQ(6, test_position->text_offset());
  // But its child index should be unchanged.
  EXPECT_EQ(1, test_position->child_index());
  // And the affinity cannot be anything other than downstream because we
  // haven't moved up the tree and so there was no opportunity to introduce any
  // ambiguity regarding the new position.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Test for a "before text" position.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->AsTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Test for an "after text" position.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->AsTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  EXPECT_EQ(0, test_position->child_index());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, AsTextPositionWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position = text_position->AsTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
  EXPECT_EQ(AXNodePosition::INVALID_INDEX, test_position->child_index());
}

TEST_F(AXPositionTest, AsLeafTreePositionWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, AsLeafTreePositionWithTreePosition) {
  // Create a tree position pointing to the first static text node inside the
  // text field: a "before children" position.
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  TestPositionType test_position = tree_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // Create a tree position pointing to the line break node inside the text
  // field.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // Create a text position pointing to the second static text node inside the
  // text field.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 2 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());
}

TEST_F(AXPositionTest, AsLeafTreePositionWithTextPosition) {
  // Create a text position pointing to the end of the root (an "after text"
  // position).
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 13 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position = text_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // Create a text position on the root, pointing to the line break character
  // inside the text field but with an upstream affinity which will cause the
  // leaf text position to be placed after the text of the first inline text
  // box.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // Create a text position pointing to the line break character inside the text
  // field but with an upstream affinity which will cause the leaf text position
  // to be placed after the text of the first inline text box.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // Create a text position on the root, pointing to the line break character
  // inside the text field.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // Create a text position pointing to the line break character inside the text
  // field.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // Create a text position pointing to the offset after the last character in
  // the text field, (an "after text" position).
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 13 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // Create a root text position that points to the middle of an equivalent leaf
  // text position.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 10 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTreePosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTreePosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());
}

TEST_F(AXPositionTest, AsLeafTextPositionWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, AsLeafTextPositionWithTreePosition) {
  // Create a tree position pointing to the first static text node inside the
  // text field.
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  TestPositionType test_position = tree_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Create a tree position pointing to the line break node inside the text
  // field.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Create a text position pointing to the second static text node inside the
  // text field.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 2 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, AsLeafTextPositionWithTextPosition) {
  // Create a text position pointing to the end of the root (an "after text"
  // position).
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 13 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_FALSE(text_position->IsLeafTextPosition());
  TestPositionType test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Create a text position on the root, pointing to the line break character
  // inside the text field but with an upstream affinity which will cause the
  // leaf text position to be placed after the text of the first inline text
  // box.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Create a text position pointing to the line break character inside the text
  // field but with an upstream affinity which will cause the leaf text position
  // to be placed after the text of the first inline text box.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Create a text position on the root, pointing to the line break character
  // inside the text field.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Create a text position pointing to the line break character inside the text
  // field.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Create a text position pointing to the offset after the last character in
  // the text field, (an "after text" position).
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 13 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Create a root text position that points to the middle of a leaf text
  // position, should maintain its relative text_offset ("Lin<e> 2")
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 10 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(3, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // Create a root text position that points to the middle of an equivalent leaf
  // text position. It should maintain its relative text_offset ("Lin<e> 2")
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 10 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(3, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, AsLeafTextPositionWithTextPositionAndEmptyTextSandwich) {
  // This test updates the tree structure to test a specific edge case -
  // AsLeafTextPosition when there is an empty leaf text node between
  // two non-empty text nodes.
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData text_data;
  text_data.id = 2;
  text_data.role = ax::mojom::Role::kInlineTextBox;
  text_data.SetName("some text");

  AXNodeData button_data;
  button_data.id = 3;
  button_data.role = ax::mojom::Role::kButton;
  button_data.SetName("");

  AXNodeData more_text_data;
  more_text_data.id = 4;
  more_text_data.role = ax::mojom::Role::kInlineTextBox;
  more_text_data.SetName("more text");

  root_data.child_ids = {text_data.id, button_data.id, more_text_data.id};

  SetTree(CreateAXTree({root_data, text_data, button_data, more_text_data}));

  // Create a text position on the root pointing to just after the
  // first static text leaf node.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 9 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_FALSE(text_position->IsLeafTextPosition());
  TestPositionType test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(button_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 9 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->AsLeafTextPosition();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsLeafTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(text_data.id, test_position->anchor_id());
  EXPECT_EQ(9, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, AsUnignoredPosition) {
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData static_text_data_1;
  static_text_data_1.id = 2;
  static_text_data_1.role = ax::mojom::Role::kStaticText;
  static_text_data_1.SetName("12");

  AXNodeData inline_box_data_1;
  inline_box_data_1.id = 3;
  inline_box_data_1.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_1.SetName("1");

  AXNodeData inline_box_data_2;
  inline_box_data_2.id = 4;
  inline_box_data_2.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_2.SetName("2");
  inline_box_data_2.AddState(ax::mojom::State::kIgnored);

  AXNodeData container_data;
  container_data.id = 5;
  container_data.role = ax::mojom::Role::kGenericContainer;
  container_data.AddState(ax::mojom::State::kIgnored);

  AXNodeData static_text_data_2;
  static_text_data_2.id = 6;
  static_text_data_2.role = ax::mojom::Role::kStaticText;
  static_text_data_2.SetName("3");

  AXNodeData inline_box_data_3;
  inline_box_data_3.id = 7;
  inline_box_data_3.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_3.SetName("3");

  static_text_data_1.child_ids = {inline_box_data_1.id, inline_box_data_2.id};
  container_data.child_ids = {static_text_data_2.id};
  static_text_data_2.child_ids = {inline_box_data_3.id};
  root_data.child_ids = {static_text_data_1.id, container_data.id};

  SetTree(CreateAXTree({root_data, static_text_data_1, inline_box_data_1,
                        inline_box_data_2, container_data, static_text_data_2,
                        inline_box_data_3}));

  // 1. In the case of a text position, we move up the parent positions until we
  // find the next unignored equivalent parent position. We don't do this for
  // tree positions because, unlike text positions which maintain the
  // corresponding text offset in the inner text of the parent node, tree
  // positions would lose some information every time a parent position is
  // computed. In other words, the parent position of a tree position is, in
  // most cases, non-equivalent to the child position.

  // "Before text" position.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), container_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position->IsIgnored());
  TestPositionType test_position = text_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveForward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(root_data.id, test_position->anchor_id());
  EXPECT_EQ(2, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // "After text" position.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), container_data.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position->IsIgnored());
  // Changing the adjustment behavior should not affect the outcome.
  test_position = text_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveBackward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(root_data.id, test_position->anchor_id());
  EXPECT_EQ(3, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  // "Before children" position.
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), container_data.id, 0 /* child_index */);
  ASSERT_TRUE(tree_position->IsIgnored());
  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveForward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // "After children" position.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), container_data.id, 1 /* child_index */);
  ASSERT_TRUE(tree_position->IsIgnored());
  // Changing the adjustment behavior should not affect the outcome.
  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveBackward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // "After children" tree positions that are anchored to an unignored node
  // whose last child is ignored.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), static_text_data_1.id, 2 /* child_index */);
  ASSERT_TRUE(tree_position->IsIgnored());
  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveBackward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_1.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // 2. If no equivalent and unignored parent position can be computed, we try
  // computing the leaf equivalent position. If this is unignored, we return it.
  // This can happen both for tree and text positions, provided that the leaf
  // node and its inner text is visible to platform APIs, i.e. it's unignored.

  root_data.AddState(ax::mojom::State::kIgnored);
  SetTree(CreateAXTree({root_data, static_text_data_1, inline_box_data_1,
                        inline_box_data_2, container_data, static_text_data_2,
                        inline_box_data_3}));

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position->IsIgnored());
  test_position = text_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveForward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box_data_1.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position->IsIgnored());
  // Changing the adjustment behavior should not change the outcome.
  test_position = text_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveBackward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box_data_1.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_data.id,
                                                     1 /* child_index */);
  ASSERT_TRUE(tree_position->IsIgnored());
  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveForward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // Changing the adjustment behavior should not affect the outcome.
  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveBackward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // "After children" position.
  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_data.id,
                                                     2 /* child_index */);
  ASSERT_TRUE(tree_position->IsIgnored());
  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveForward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // Changing the adjustment behavior should not affect the outcome.
  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveBackward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // "Before children" position.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), container_data.id, 0 /* child_index */);
  ASSERT_TRUE(tree_position->IsIgnored());
  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveForward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // "After children" position.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), container_data.id, 1 /* child_index */);
  ASSERT_TRUE(tree_position->IsIgnored());
  // Changing the adjustment behavior should not affect the outcome.
  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveBackward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // 3. As a last resort, we move either to the next or previous unignored
  // position in the accessibility tree, based on the "adjustment_behavior".

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_data.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position->IsIgnored());
  test_position = text_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveForward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_data_2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position->IsIgnored());
  test_position = text_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveForward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_data_2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position->IsIgnored());
  test_position = text_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveBackward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box_data_1.id, test_position->anchor_id());
  // This should be an "after text" position.
  EXPECT_EQ(1, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box_data_2.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_TRUE(tree_position->IsIgnored());
  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveForward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());
  ASSERT_TRUE(tree_position->IsIgnored());

  test_position = tree_position->AsUnignoredPosition(
      AXPositionAdjustmentBehavior::kMoveBackward);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box_data_1.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());
}

TEST_F(AXPositionTest, CreatePositionAtTextBoundaryDocumentStartEndIsIgnored) {
  // +-root_data
  //   +-static_text_data_1
  //   | +-inline_box_data_1 IGNORED
  //   +-static_text_data_2
  //   | +-inline_box_data_2
  //   +-static_text_data_3
  //   | +-inline_box_data_3
  //   +-static_text_data_4
  //     +-inline_box_data_4 IGNORED
  constexpr AXNode::AXID ROOT_ID = 1;
  constexpr AXNode::AXID STATIC_TEXT1_ID = 2;
  constexpr AXNode::AXID STATIC_TEXT2_ID = 3;
  constexpr AXNode::AXID STATIC_TEXT3_ID = 4;
  constexpr AXNode::AXID STATIC_TEXT4_ID = 5;
  constexpr AXNode::AXID INLINE_BOX1_ID = 6;
  constexpr AXNode::AXID INLINE_BOX2_ID = 7;
  constexpr AXNode::AXID INLINE_BOX3_ID = 8;
  constexpr AXNode::AXID INLINE_BOX4_ID = 9;

  AXNodeData root_data;
  root_data.id = ROOT_ID;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData static_text_data_1;
  static_text_data_1.id = STATIC_TEXT1_ID;
  static_text_data_1.role = ax::mojom::Role::kStaticText;
  static_text_data_1.SetName("One");

  AXNodeData inline_box_data_1;
  inline_box_data_1.id = INLINE_BOX1_ID;
  inline_box_data_1.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_1.SetName("One");
  inline_box_data_1.AddState(ax::mojom::State::kIgnored);
  inline_box_data_1.AddIntListAttribute(
      ax::mojom::IntListAttribute::kWordStarts, std::vector<int32_t>{0});
  inline_box_data_1.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                        std::vector<int32_t>{3});
  inline_box_data_1.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                                    INLINE_BOX2_ID);

  AXNodeData static_text_data_2;
  static_text_data_2.id = STATIC_TEXT2_ID;
  static_text_data_2.role = ax::mojom::Role::kStaticText;
  static_text_data_2.SetName("Two");

  AXNodeData inline_box_data_2;
  inline_box_data_2.id = INLINE_BOX2_ID;
  inline_box_data_2.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_2.SetName("Two");
  inline_box_data_2.AddIntListAttribute(
      ax::mojom::IntListAttribute::kWordStarts, std::vector<int32_t>{0});
  inline_box_data_2.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                        std::vector<int32_t>{3});
  inline_box_data_2.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                                    INLINE_BOX1_ID);
  inline_box_data_2.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                                    INLINE_BOX3_ID);

  AXNodeData static_text_data_3;
  static_text_data_3.id = STATIC_TEXT3_ID;
  static_text_data_3.role = ax::mojom::Role::kStaticText;
  static_text_data_3.SetName("Three");

  AXNodeData inline_box_data_3;
  inline_box_data_3.id = INLINE_BOX3_ID;
  inline_box_data_3.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_3.SetName("Three");
  inline_box_data_3.AddIntListAttribute(
      ax::mojom::IntListAttribute::kWordStarts, std::vector<int32_t>{0});
  inline_box_data_3.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                        std::vector<int32_t>{5});
  inline_box_data_3.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                                    INLINE_BOX2_ID);
  inline_box_data_3.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                                    INLINE_BOX4_ID);

  AXNodeData static_text_data_4;
  static_text_data_4.id = STATIC_TEXT4_ID;
  static_text_data_4.role = ax::mojom::Role::kStaticText;
  static_text_data_4.SetName("Four");

  AXNodeData inline_box_data_4;
  inline_box_data_4.id = INLINE_BOX4_ID;
  inline_box_data_4.role = ax::mojom::Role::kInlineTextBox;
  inline_box_data_4.SetName("Four");
  inline_box_data_4.AddState(ax::mojom::State::kIgnored);
  inline_box_data_3.AddIntListAttribute(
      ax::mojom::IntListAttribute::kWordStarts, std::vector<int32_t>{0});
  inline_box_data_3.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                        std::vector<int32_t>{4});
  inline_box_data_3.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                                    INLINE_BOX3_ID);

  root_data.child_ids = {static_text_data_1.id, static_text_data_2.id,
                         static_text_data_3.id, static_text_data_4.id};
  static_text_data_1.child_ids = {inline_box_data_1.id};
  static_text_data_2.child_ids = {inline_box_data_2.id};
  static_text_data_3.child_ids = {inline_box_data_3.id};
  static_text_data_4.child_ids = {inline_box_data_4.id};

  SetTree(
      CreateAXTree({root_data, static_text_data_1, static_text_data_2,
                    static_text_data_3, static_text_data_4, inline_box_data_1,
                    inline_box_data_2, inline_box_data_3, inline_box_data_4}));

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_data_2.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_FALSE(text_position->IsIgnored());
  TestPositionType test_position = text_position->CreatePositionAtTextBoundary(
      ax::mojom::TextBoundary::kWordStart, ax::mojom::MoveDirection::kForward,
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
  test_position = text_position->CreatePositionAtTextBoundary(
      ax::mojom::TextBoundary::kWordStart, ax::mojom::MoveDirection::kBackward,
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box_data_2.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_data_3.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_FALSE(text_position->IsIgnored());
  test_position = text_position->CreatePositionAtTextBoundary(
      ax::mojom::TextBoundary::kWordStart, ax::mojom::MoveDirection::kForward,
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box_data_3.id, test_position->anchor_id());
  EXPECT_EQ(5, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
  test_position = text_position->CreatePositionAtTextBoundary(
      ax::mojom::TextBoundary::kWordStart, ax::mojom::MoveDirection::kBackward,
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box_data_2.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, CreatePositionAtInvalidGraphemeBoundary) {
  std::vector<int> text_offsets;
  SetTree(CreateMultilingualDocument(&text_offsets));

  TestPositionType test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
  EXPECT_EQ(4, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 10 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
  EXPECT_EQ(10, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kUpstream, test_position->affinity());
}

TEST_F(AXPositionTest, CreatePositionAtStartOfAnchorWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position =
      null_position->CreatePositionAtStartOfAnchor();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtStartOfAnchorWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  TestPositionType test_position =
      tree_position->CreatePositionAtStartOfAnchor();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(root_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                     1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->CreatePositionAtStartOfAnchor();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(root_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // An "after text" position.
  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->CreatePositionAtStartOfAnchor();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());
}

TEST_F(AXPositionTest, CreatePositionAtStartOfAnchorWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position =
      text_position->CreatePositionAtStartOfAnchor();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->CreatePositionAtStartOfAnchor();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  // Affinity should have been reset to the default value.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, CreatePositionAtEndOfAnchorWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->CreatePositionAtEndOfAnchor();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtEndOfAnchorWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 3 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  TestPositionType test_position = tree_position->CreatePositionAtEndOfAnchor();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(root_.id, test_position->anchor_id());
  EXPECT_EQ(3, test_position->child_index());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                     1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->CreatePositionAtEndOfAnchor();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(root_.id, test_position->anchor_id());
  EXPECT_EQ(3, test_position->child_index());
}

TEST_F(AXPositionTest, CreatePositionAtEndOfAnchorWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position = text_position->CreatePositionAtEndOfAnchor();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->CreatePositionAtEndOfAnchor();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  // Affinity should have been reset to the default value.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, CreatePositionAtPreviousFormatStartWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position =
      null_position->CreatePreviousFormatStartPosition(
          AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = null_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = null_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtPreviousFormatStartWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), static_text1_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());

  TestPositionType test_position =
      tree_position->CreatePreviousFormatStartPosition(
          AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(static_text1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  test_position = test_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  test_position = test_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // StopIfAlreadyAtBoundary shouldn't move, since it's already at a boundary.
  test_position = test_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // StopAtLastAnchorBoundary should stop at the start of the document while
  // CrossBoundary should return a null position when crossing it.
  test_position = test_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  test_position = test_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtPreviousFormatStartWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 2 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  TestPositionType test_position =
      text_position->CreatePreviousFormatStartPosition(
          AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  test_position = test_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // StopIfAlreadyAtBoundary shouldn't move, since it's already at a boundary.
  test_position = test_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // StopAtLastAnchorBoundary should stop at the start of the document while
  // CrossBoundary should return a null position when crossing it.
  test_position = test_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtNextFormatEndWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = null_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtNextFormatEndWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), button_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());

  TestPositionType test_position = tree_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // StopIfAlreadyAtBoundary shouldn't move, since it's already at a boundary.
  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // StopAtLastAnchorBoundary should stop at the end of the document while
  // CrossBoundary should return a null position when crossing it.
  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtNextFormatEndWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  TestPositionType test_position = text_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());

  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());

  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());

  // StopIfAlreadyAtBoundary shouldn't move, since it's already at a boundary.
  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());

  // StopAtLastAnchorBoundary should stop at the end of the document while
  // CrossBoundary should return a null position when crossing it.
  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());

  test_position = test_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtFormatBoundaryWithTextPosition) {
  // This test updates the tree structure to test a specific edge case -
  // CreatePositionAtFormatBoundary when text lies at the beginning and end
  // of the AX tree.
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData text_data;
  text_data.id = 2;
  text_data.role = ax::mojom::Role::kStaticText;
  text_data.SetName("some text");

  AXNodeData more_text_data;
  more_text_data.id = 3;
  more_text_data.role = ax::mojom::Role::kStaticText;
  more_text_data.SetName("more text");

  root_data.child_ids = {text_data.id, more_text_data.id};

  SetTree(CreateAXTree({root_data, text_data, more_text_data}));

  // Test CreatePreviousFormatStartPosition at the start of the document.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_data.id, 8 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  TestPositionType test_position =
      text_position->CreatePreviousFormatStartPosition(
          AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // Test CreateNextFormatEndPosition at the end of the document.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), more_text_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->CreateNextFormatEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(more_text_data.id, test_position->anchor_id());
  EXPECT_EQ(9, test_position->text_offset());
}

TEST_F(AXPositionTest, MoveByFormatWithIgnoredNodes) {
  // ++1 kRootWebArea
  // ++++2 kGenericContainer
  // ++++++3 kButton
  // ++++++++4 kStaticText
  // ++++++++++5 kInlineTextBox
  // ++++++++6 kSvgRoot ignored
  // ++++++++++7 kGenericContainer ignored
  // ++++8 kGenericContainer
  // ++++++9 kHeading
  // ++++++++10 kStaticText
  // ++++++++++11 kInlineTextBox
  // ++++++12 kStaticText
  // ++++++++13 kInlineTextBox
  // ++++++14 kGenericContainer ignored
  // ++++15 kGenericContainer
  // ++++++16 kHeading
  // ++++++++17 kStaticText
  // ++++++++++18 kInlineTextBox
  // ++++19 kGenericContainer
  // ++++++20 kGenericContainer ignored
  // ++++++21 kStaticText
  // ++++++++22 kInlineTextBox
  // ++++++23 kHeading
  // ++++++++24 kStaticText
  // ++++++++++25 kInlineTextBox
  AXNodeData root_1;
  AXNodeData generic_container_2;
  AXNodeData button_3;
  AXNodeData static_text_4;
  AXNodeData inline_box_5;
  AXNodeData svg_root_6;
  AXNodeData generic_container_7;
  AXNodeData generic_container_8;
  AXNodeData heading_9;
  AXNodeData static_text_10;
  AXNodeData inline_box_11;
  AXNodeData static_text_12;
  AXNodeData inline_box_13;
  AXNodeData generic_container_14;
  AXNodeData generic_container_15;
  AXNodeData heading_16;
  AXNodeData static_text_17;
  AXNodeData inline_box_18;
  AXNodeData generic_container_19;
  AXNodeData generic_container_20;
  AXNodeData static_text_21;
  AXNodeData inline_box_22;
  AXNodeData heading_23;
  AXNodeData static_text_24;
  AXNodeData inline_box_25;

  root_1.id = 1;
  generic_container_2.id = 2;
  button_3.id = 3;
  static_text_4.id = 4;
  inline_box_5.id = 5;
  svg_root_6.id = 6;
  generic_container_7.id = 7;
  generic_container_8.id = 8;
  heading_9.id = 9;
  static_text_10.id = 10;
  inline_box_11.id = 11;
  static_text_12.id = 12;
  inline_box_13.id = 13;
  generic_container_14.id = 14;
  generic_container_15.id = 15;
  heading_16.id = 16;
  static_text_17.id = 17;
  inline_box_18.id = 18;
  generic_container_19.id = 19;
  generic_container_20.id = 20;
  static_text_21.id = 21;
  inline_box_22.id = 22;
  heading_23.id = 23;
  static_text_24.id = 24;
  inline_box_25.id = 25;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {generic_container_2.id, generic_container_8.id,
                      generic_container_15.id, generic_container_19.id};

  generic_container_2.role = ax::mojom::Role::kGenericContainer;
  generic_container_2.child_ids = {button_3.id};

  button_3.role = ax::mojom::Role::kButton;
  button_3.child_ids = {static_text_4.id, svg_root_6.id};

  static_text_4.role = ax::mojom::Role::kStaticText;
  static_text_4.child_ids = {inline_box_5.id};
  static_text_4.SetName("Button");

  inline_box_5.role = ax::mojom::Role::kInlineTextBox;
  inline_box_5.SetName("Button");

  svg_root_6.role = ax::mojom::Role::kSvgRoot;
  svg_root_6.child_ids = {generic_container_7.id};
  svg_root_6.AddState(ax::mojom::State::kIgnored);

  generic_container_7.role = ax::mojom::Role::kGenericContainer;
  generic_container_7.AddState(ax::mojom::State::kIgnored);

  generic_container_8.role = ax::mojom::Role::kGenericContainer;
  generic_container_8.child_ids = {heading_9.id, static_text_12.id,
                                   generic_container_14.id};

  heading_9.role = ax::mojom::Role::kHeading;
  heading_9.child_ids = {static_text_10.id};

  static_text_10.role = ax::mojom::Role::kStaticText;
  static_text_10.child_ids = {inline_box_11.id};
  static_text_10.SetName("Heading");

  inline_box_11.role = ax::mojom::Role::kInlineTextBox;
  inline_box_11.SetName("Heading");

  static_text_12.role = ax::mojom::Role::kStaticText;
  static_text_12.child_ids = {inline_box_13.id};
  static_text_12.SetName("3.14");

  inline_box_13.role = ax::mojom::Role::kInlineTextBox;
  inline_box_13.SetName("3.14");

  generic_container_14.role = ax::mojom::Role::kGenericContainer;
  generic_container_14.AddState(ax::mojom::State::kIgnored);

  generic_container_15.role = ax::mojom::Role::kGenericContainer;
  generic_container_15.child_ids = {heading_16.id};

  heading_16.role = ax::mojom::Role::kHeading;
  heading_16.child_ids = {static_text_17.id};

  static_text_17.role = ax::mojom::Role::kStaticText;
  static_text_17.child_ids = {inline_box_18.id};
  static_text_17.SetName("Heading");

  inline_box_18.role = ax::mojom::Role::kInlineTextBox;
  inline_box_18.SetName("Heading");

  generic_container_19.role = ax::mojom::Role::kGenericContainer;
  generic_container_19.child_ids = {generic_container_20.id, static_text_21.id,
                                    heading_23.id};

  generic_container_20.role = ax::mojom::Role::kGenericContainer;
  generic_container_20.AddState(ax::mojom::State::kIgnored);

  static_text_21.role = ax::mojom::Role::kStaticText;
  static_text_21.child_ids = {inline_box_22.id};
  static_text_21.SetName("3.14");

  inline_box_22.role = ax::mojom::Role::kInlineTextBox;
  inline_box_22.SetName("3.14");

  heading_23.role = ax::mojom::Role::kHeading;
  heading_23.child_ids = {static_text_24.id};

  static_text_24.role = ax::mojom::Role::kStaticText;
  static_text_24.child_ids = {inline_box_25.id};
  static_text_24.SetName("Heading");

  inline_box_25.role = ax::mojom::Role::kInlineTextBox;
  inline_box_25.SetName("Heading");

  SetTree(CreateAXTree({root_1,
                        generic_container_2,
                        button_3,
                        static_text_4,
                        inline_box_5,
                        svg_root_6,
                        generic_container_7,
                        generic_container_8,
                        heading_9,
                        static_text_10,
                        inline_box_11,
                        static_text_12,
                        inline_box_13,
                        generic_container_14,
                        generic_container_15,
                        heading_16,
                        static_text_17,
                        inline_box_18,
                        generic_container_19,
                        generic_container_20,
                        static_text_21,
                        inline_box_22,
                        heading_23,
                        static_text_24,
                        inline_box_25}));

  // There are two major cases to consider for format boundaries with ignored
  // nodes:
  // Case 1: When the ignored node is directly next to the current position.
  // Case 2: When the ignored node is directly next to the next/previous format
  // boundary.

  // Case 1
  // This test case spans nodes 2 to 11, inclusively.
  {
    // Forward movement
    TestPositionType text_position = AXNodePosition::CreateTextPosition(
        GetTreeID(), inline_box_5.id, 6 /* text_offset */,
        ax::mojom::TextAffinity::kDownstream);
    ASSERT_NE(nullptr, text_position);
    EXPECT_TRUE(text_position->IsTextPosition());
    EXPECT_EQ(inline_box_5.id, text_position->anchor_id());
    EXPECT_EQ(6, text_position->text_offset());

    text_position = text_position->CreateNextFormatEndPosition(
        AXBoundaryBehavior::StopAtLastAnchorBoundary);
    ASSERT_NE(nullptr, text_position);
    EXPECT_TRUE(text_position->IsTextPosition());
    EXPECT_EQ(inline_box_11.id, text_position->anchor_id());
    EXPECT_EQ(7, text_position->text_offset());

    // Backward movement
    text_position = AXNodePosition::CreateTextPosition(
        GetTreeID(), inline_box_11.id, 0 /* text_offset */,
        ax::mojom::TextAffinity::kDownstream);
    ASSERT_NE(nullptr, text_position);
    EXPECT_TRUE(text_position->IsTextPosition());
    EXPECT_EQ(inline_box_11.id, text_position->anchor_id());
    EXPECT_EQ(0, text_position->text_offset());

    text_position = text_position->CreatePreviousFormatStartPosition(
        AXBoundaryBehavior::StopAtLastAnchorBoundary);
    ASSERT_NE(nullptr, text_position);
    EXPECT_TRUE(text_position->IsTextPosition());
    EXPECT_EQ(inline_box_5.id, text_position->anchor_id());
    EXPECT_EQ(0, text_position->text_offset());
  }

  // Case 2
  // This test case spans nodes 8 to 25.
  {
    // Forward movement
    TestPositionType text_position = AXNodePosition::CreateTextPosition(
        GetTreeID(), inline_box_11.id, 7 /* text_offset */,
        ax::mojom::TextAffinity::kDownstream);
    ASSERT_NE(nullptr, text_position);
    EXPECT_TRUE(text_position->IsTextPosition());
    EXPECT_EQ(inline_box_11.id, text_position->anchor_id());
    EXPECT_EQ(7, text_position->text_offset());

    text_position = text_position->CreateNextFormatEndPosition(
        AXBoundaryBehavior::StopAtLastAnchorBoundary);
    ASSERT_NE(nullptr, text_position);
    EXPECT_TRUE(text_position->IsTextPosition());
    EXPECT_EQ(inline_box_13.id, text_position->anchor_id());
    EXPECT_EQ(4, text_position->text_offset());

    // Backward movement
    text_position = AXNodePosition::CreateTextPosition(
        GetTreeID(), inline_box_25.id, 0 /* text_offset */,
        ax::mojom::TextAffinity::kDownstream);
    ASSERT_NE(nullptr, text_position);
    EXPECT_TRUE(text_position->IsTextPosition());
    EXPECT_EQ(inline_box_25.id, text_position->anchor_id());
    EXPECT_EQ(0, text_position->text_offset());

    text_position = text_position->CreatePreviousFormatStartPosition(
        AXBoundaryBehavior::StopAtLastAnchorBoundary);
    ASSERT_NE(nullptr, text_position);
    EXPECT_TRUE(text_position->IsTextPosition());
    EXPECT_EQ(inline_box_22.id, text_position->anchor_id());
    EXPECT_EQ(0, text_position->text_offset());
  }
}

TEST_F(AXPositionTest, CreatePositionAtPageBoundaryWithTextPosition) {
  AXNodeData root_data, page_1_data, page_1_text_data, page_2_data,
      page_2_text_data, page_3_data, page_3_text_data;
  SetTree(CreateMultipageDocument(root_data, page_1_data, page_1_text_data,
                                  page_2_data, page_2_text_data, page_3_data,
                                  page_3_text_data));

  // Test CreateNextPageStartPosition at the start of the document.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), page_1_text_data.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  // StopIfAlreadyAtBoundary shouldn't move at all since it's at a boundary.
  TestPositionType test_position = text_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_1_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = text_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = text_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // Test CreateNextPageEndPosition until the end of document is reached.
  test_position = test_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(19, test_position->text_offset());

  test_position = test_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_3_text_data.id, test_position->anchor_id());
  EXPECT_EQ(24, test_position->text_offset());

  test_position = test_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_3_text_data.id, test_position->anchor_id());
  EXPECT_EQ(24, test_position->text_offset());

  // StopAtLastAnchorBoundary shouldn't move past the end of the document.
  test_position = test_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_3_text_data.id, test_position->anchor_id());
  EXPECT_EQ(24, test_position->text_offset());

  test_position = test_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_3_text_data.id, test_position->anchor_id());
  EXPECT_EQ(24, test_position->text_offset());

  // Moving forward past the end should return a null position.
  TestPositionType null_position = test_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, null_position);
  EXPECT_TRUE(null_position->IsNullPosition());

  null_position = test_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, null_position);
  EXPECT_TRUE(null_position->IsNullPosition());

  // Now move backward through the document.
  text_position = test_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->IsTextPosition());
  EXPECT_EQ(page_3_text_data.id, text_position->anchor_id());
  EXPECT_EQ(24, text_position->text_offset());

  test_position = text_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(19, test_position->text_offset());

  test_position = text_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(19, test_position->text_offset());

  test_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_1_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_1_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // StopAtLastAnchorBoundary shouldn't move past the start of the document.
  test_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_1_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(page_1_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // Moving before the start should return a null position.
  null_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, null_position);
  EXPECT_TRUE(null_position->IsNullPosition());

  null_position = test_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, null_position);
  EXPECT_TRUE(null_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtPageBoundaryWithTreePosition) {
  AXNodeData root_data, page_1_data, page_1_text_data, page_2_data,
      page_2_text_data, page_3_data, page_3_text_data;
  SetTree(CreateMultipageDocument(root_data, page_1_data, page_1_text_data,
                                  page_2_data, page_2_text_data, page_3_data,
                                  page_3_text_data));

  // Test CreateNextPageStartPosition at the start of the document.
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), page_1_data.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  ASSERT_TRUE(tree_position->IsTreePosition());

  // StopIfAlreadyAtBoundary shouldn't move at all since it's at a boundary.
  TestPositionType test_position = tree_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_1_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  test_position = tree_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  test_position = tree_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // Test CreateNextPageEndPosition until the end of document is reached.
  test_position = tree_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_1_data.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->child_index());

  test_position = test_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  test_position = test_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // StopAtLastAnchorBoundary shouldn't move past the end of the document.
  test_position = test_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_3_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  test_position = test_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_3_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  // Moving forward past the end should return a null position.
  TestPositionType null_position = test_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, null_position);
  EXPECT_TRUE(null_position->IsNullPosition());

  null_position = test_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, null_position);
  EXPECT_TRUE(null_position->IsNullPosition());

  // Now move backward through the document.
  tree_position = test_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, tree_position);
  EXPECT_TRUE(tree_position->IsTreePosition());
  EXPECT_EQ(page_3_text_data.id, tree_position->anchor_id());
  EXPECT_EQ(0, tree_position->child_index());

  test_position = tree_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  test_position = tree_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->child_index());

  test_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_2_text_data.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  test_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_1_text_data.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  test_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_1_text_data.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // StopAtLastAnchorBoundary shouldn't move past the start of the document.
  test_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_1_text_data.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  test_position = test_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(page_1_text_data.id, test_position->anchor_id());
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  // Moving before the start should return a null position.
  null_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, null_position);
  EXPECT_TRUE(null_position->IsNullPosition());

  null_position = test_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, null_position);
  EXPECT_TRUE(null_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePagePositionWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position =
      null_position->CreatePreviousPageStartPosition(
          AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  test_position = null_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  test_position = null_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  test_position = null_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtStartOfDocumentWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position =
      null_position->CreatePositionAtStartOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePagePositionWithNonPaginatedDocument) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);

  // Non-paginated documents should move to the start of the document for
  // CreatePreviousPageStartPosition (treating the entire document as a single
  // page)
  TestPositionType test_position =
      text_position->CreatePreviousPageStartPosition(
          AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // Since there is no next page, CreateNextPageStartPosition should return a
  // null position
  test_position = text_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  // Since there is no previous page, CreatePreviousPageEndPosition should
  // return a null position
  test_position = text_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  // Since there are no distinct pages, CreateNextPageEndPosition should move
  // to the end of the document, as if it's one large page.
  test_position = text_position->CreateNextPageEndPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());

  // CreatePreviousPageStartPosition should move back to the beginning of the
  // document
  test_position = test_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // Since there's no next page, CreateNextPageStartPosition should return a
  // null position
  test_position = test_position->CreateNextPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  // Since there's no previous page, CreatePreviousPageEndPosition should return
  // a null position
  test_position = text_position->CreatePreviousPageEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  // Since there's no previous page, CreatePreviousPageStartPosition should
  // return a null position
  test_position = text_position->CreatePreviousPageStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtStartOfDocumentWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  TestPositionType test_position =
      tree_position->CreatePositionAtStartOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_EQ(root_.id, test_position->anchor_id());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                     1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->CreatePositionAtStartOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_EQ(root_.id, test_position->anchor_id());

  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->CreatePositionAtStartOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_EQ(root_.id, test_position->anchor_id());
}

TEST_F(AXPositionTest, CreatePositionAtStartOfDocumentWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  TestPositionType test_position =
      text_position->CreatePositionAtStartOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_EQ(root_.id, test_position->anchor_id());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->CreatePositionAtStartOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_EQ(root_.id, test_position->anchor_id());
  // Affinity should have been reset to the default value.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, CreatePositionAtEndOfDocumentWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position =
      null_position->CreatePositionAtEndOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreatePositionAtEndOfDocumentWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 3 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  TestPositionType test_position =
      tree_position->CreatePositionAtEndOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                     1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->CreatePositionAtEndOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());

  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->CreatePositionAtEndOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
}

TEST_F(AXPositionTest, CreatePositionAtEndOfDocumentWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  TestPositionType test_position =
      text_position->CreatePositionAtEndOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  test_position = text_position->CreatePositionAtEndOfDocument();
  EXPECT_NE(nullptr, test_position);
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  // Affinity should have been reset to the default value.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, AtLastNodeInTree) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_FALSE(text_position->AtLastNodeInTree());
  EXPECT_FALSE(text_position->AsTreePosition()->AtLastNodeInTree());

  TestPositionType test_position =
      text_position->CreatePositionAtEndOfDocument();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->AtLastNodeInTree());
  EXPECT_TRUE(test_position->AsTreePosition()->AtLastNodeInTree());
  EXPECT_FALSE(text_position->CreateNullPosition()->AtLastNodeInTree());

  TestPositionType on_last_node_but_not_at_maxtextoffset =
      AXNodePosition::CreateTextPosition(GetTreeID(), inline_box2_.id,
                                         1 /* text_offset */,
                                         ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, on_last_node_but_not_at_maxtextoffset);
  EXPECT_TRUE(on_last_node_but_not_at_maxtextoffset->AtLastNodeInTree());
  EXPECT_TRUE(on_last_node_but_not_at_maxtextoffset->AsTreePosition()
                  ->AtLastNodeInTree());
}

TEST_F(AXPositionTest, CreateChildPositionAtWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->CreateChildPositionAt(0);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreateChildPositionAtWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 2 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  TestPositionType test_position = tree_position->CreateChildPositionAt(1);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  // Since the anchor is a leaf node, |child_index| should signify that this is
  // a "before text" position.
  EXPECT_EQ(AXNodePosition::BEFORE_TEXT, test_position->child_index());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), button_.id,
                                                     0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->CreateChildPositionAt(0);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreateChildPositionAtWithTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position = text_position->CreateChildPositionAt(0);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->CreateChildPositionAt(1);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreateParentPositionWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->CreateParentPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreateParentPositionWithTreePosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), check_box_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  TestPositionType test_position = tree_position->CreateParentPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(root_.id, test_position->anchor_id());
  // |child_index| should point to the check box node.
  EXPECT_EQ(1, test_position->child_index());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  tree_position = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                     1 /* child_index */);
  ASSERT_NE(nullptr, tree_position);
  test_position = tree_position->CreateParentPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreateParentPositionWithTextPosition) {
  // Create a position that points at the end of the first line, right after the
  // check box.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position = text_position->CreateParentPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(root_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  // Since the same text offset in the root could be used to point to the
  // beginning of the second line, affinity should have been adjusted to
  // upstream.
  EXPECT_EQ(ax::mojom::TextAffinity::kUpstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->CreateParentPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(static_text2_.id, test_position->anchor_id());
  EXPECT_EQ(5, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  test_position = test_position->CreateParentPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  // |text_offset| should point to the same offset on the second line where the
  // static text node position was pointing at.
  EXPECT_EQ(12, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, CreateNextAndPreviousLeafTextPositionWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->CreateNextLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = null_position->CreatePreviousLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreateNextLeafTextPosition) {
  TestPositionType check_box_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, check_box_position);
  TestPositionType test_position =
      check_box_position->CreateNextLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // The text offset on the root points to the button since it is the first
  // available leaf text position, even though it has no text content.
  TestPositionType root_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, root_position);
  ASSERT_TRUE(root_position->IsTextPosition());
  test_position = root_position->CreateNextLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  TestPositionType button_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, button_position);
  ASSERT_TRUE(button_position->IsTextPosition());
  test_position = button_position->CreateNextLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreateNextLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreateNextLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreateNextLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreateNextLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  TestPositionType text_field_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 2 /* child_index */);
  ASSERT_NE(nullptr, text_field_position);
  test_position = text_field_position->CreateNextLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // The root text position should resolve to its leaf text position,
  // maintaining its text_offset
  TestPositionType root_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 10 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, root_position2);
  ASSERT_TRUE(root_position2->IsTextPosition());
  test_position = root_position2->CreateNextLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(3, test_position->text_offset());
}

TEST_F(AXPositionTest, CreatePreviousLeafTextPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position =
      text_position->CreatePreviousLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // Create a "before text" tree position on the second line of the text box.
  TestPositionType before_text_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box2_.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_NE(nullptr, before_text_position);
  test_position = before_text_position->CreatePreviousLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreatePreviousLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreatePreviousLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreatePreviousLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  test_position = test_position->CreatePreviousLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  TestPositionType text_field_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 2 /* child_index */);
  ASSERT_NE(nullptr, text_field_position);
  test_position = text_field_position->CreatePreviousLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // The text offset on the root points to the text coming from inside the check
  // box.
  TestPositionType check_box_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, check_box_position);
  ASSERT_TRUE(check_box_position->IsTextPosition());
  test_position = check_box_position->CreatePreviousLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(button_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // The root text position should resolve to its leaf text position,
  // maintaining its text_offset
  TestPositionType root_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 10 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, root_position2);
  ASSERT_TRUE(root_position2->IsTextPosition());
  test_position = root_position2->CreatePreviousLeafTextPosition();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTreeID(), test_position->tree_id());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(3, test_position->text_offset());
}

TEST_F(AXPositionTest, CreateNextLeafTreePosition) {
  TestPositionType root_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  ASSERT_TRUE(root_position->IsTreePosition());

  TestPositionType button_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), button_.id, AXNodePosition::BEFORE_TEXT);
  TestPositionType checkbox_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), check_box_.id, AXNodePosition::BEFORE_TEXT);
  TestPositionType inline_box1_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, AXNodePosition::BEFORE_TEXT);
  TestPositionType line_break_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), line_break_.id, AXNodePosition::BEFORE_TEXT);
  TestPositionType inline_box2_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box2_.id, AXNodePosition::BEFORE_TEXT);

  TestPositionType test_position = root_position->CreateNextLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *button_position);

  test_position = test_position->CreateNextLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *checkbox_position);

  test_position = test_position->CreateNextLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *inline_box1_position);

  test_position = test_position->CreateNextLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *line_break_position);

  test_position = test_position->CreateNextLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *inline_box2_position);

  test_position = test_position->CreateNextLeafTreePosition();
  EXPECT_TRUE(test_position->IsNullPosition());

  TestPositionType root_text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 2 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  EXPECT_TRUE(root_text_position->IsTextPosition());

  test_position = root_text_position->CreateNextLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *inline_box1_position);

  TestPositionType inline_box1_text_position =
      AXNodePosition::CreateTextPosition(GetTreeID(), inline_box1_.id,
                                         2 /* text_offset */,
                                         ax::mojom::TextAffinity::kDownstream);
  EXPECT_TRUE(inline_box1_text_position->IsTextPosition());

  test_position = inline_box1_text_position->CreateNextLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *line_break_position);
}

TEST_F(AXPositionTest, CreatePreviousLeafTreePosition) {
  TestPositionType inline_box2_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box2_.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_TRUE(inline_box2_position->IsTreePosition());

  TestPositionType line_break_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), line_break_.id, AXNodePosition::BEFORE_TEXT);
  TestPositionType inline_box1_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, AXNodePosition::BEFORE_TEXT);
  TestPositionType checkbox_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), check_box_.id, AXNodePosition::BEFORE_TEXT);
  TestPositionType button_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), button_.id, AXNodePosition::BEFORE_TEXT);

  TestPositionType test_position =
      inline_box2_position->CreatePreviousLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *line_break_position);

  test_position = test_position->CreatePreviousLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *inline_box1_position);

  test_position = test_position->CreatePreviousLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *checkbox_position);

  test_position = test_position->CreatePreviousLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *button_position);

  test_position = test_position->CreatePreviousLeafTreePosition();
  EXPECT_TRUE(test_position->IsNullPosition());

  TestPositionType inline_box2_text_position =
      AXNodePosition::CreateTextPosition(GetTreeID(), inline_box2_.id,
                                         2 /* text_offset */,
                                         ax::mojom::TextAffinity::kDownstream);
  EXPECT_TRUE(inline_box2_text_position->IsTextPosition());

  test_position = inline_box2_text_position->CreatePreviousLeafTreePosition();
  EXPECT_TRUE(test_position->IsTreePosition());
  EXPECT_EQ(*test_position, *line_break_position);
}

TEST_F(AXPositionTest,
       AsLeafTextPositionBeforeAndAfterCharacterWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  ASSERT_TRUE(null_position->IsNullPosition());
  TestPositionType test_position =
      null_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = null_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest,
       AsLeafTextPositionBeforeAndAfterCharacterAtInvalidGraphemeBoundary) {
#if true
  GTEST_SKIP()
      << "Skipping, current accessibility library cannot handle grapheme";
#else
  std::vector<int> text_offsets;
  SetTree(CreateMultilingualDocument(&text_offsets));

  TestPositionType test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  test_position = test_position->AsLeafTextPositionAfterCharacter();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->children()[1]->id(), test_position->anchor_id());
  // "text_offset_" should have been adjusted to the next grapheme boundary.
  EXPECT_EQ(2, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 10 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  test_position = test_position->AsLeafTextPositionBeforeCharacter();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->children()[2]->id(), test_position->anchor_id());
  // "text_offset_" should have been adjusted to the previous grapheme boundary.
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 10 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  test_position = test_position->AsLeafTextPositionBeforeCharacter();
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->children()[2]->id(), test_position->anchor_id());
  // The same as above, "text_offset_" should have been adjusted to the previous
  // grapheme boundary.
  EXPECT_EQ(0, test_position->text_offset());
  // An upstream affinity should have had no effect on the outcome and so, it
  // should have been reset in order to provide consistent output from the
  // method regardless of input affinity.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
#endif  // true
}

TEST_F(AXPositionTest, AsLeafTextPositionBeforeCharacterNoAdjustment) {
  // A text offset that is on the line break right after "Line 1".
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position =
      text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  // A text offset that is before the line break right after "Line 1".
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 13 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
}

TEST_F(AXPositionTest, AsLeafTextPositionAfterCharacterNoAdjustment) {
  // A text offset that is after "Line 2".
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 13 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position =
      text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());

  // A text offset that is before "Line 2".
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 7 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());

  // A text offset that is on the line break right after "Line 1".
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 13 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
}

TEST_F(AXPositionTest, AsLeafTextPositionBeforeCharacter) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position =
      text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(3, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 13 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionBeforeCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, AsLeafTextPositionAfterCharacter) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionType test_position =
      text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(5, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  test_position = text_position->AsLeafTextPositionAfterCharacter();
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreateNextAndPreviousCharacterPositionWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = null_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, AsValidPosition) {
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData text_data;
  text_data.id = 2;
  text_data.role = ax::mojom::Role::kStaticText;
  text_data.SetName("some text");

  root_data.child_ids = {text_data.id};

  SetTree(CreateAXTree({root_data, text_data}));

  // Create a text position at MaxTextOffset.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_data.id, 9 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->IsValid());
  EXPECT_EQ(9, text_position->text_offset());

  // Test basic cases with static MaxTextOffset
  TestPositionType test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_TRUE(test_position->IsValid());
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(text_data.id, test_position->anchor_id());
  EXPECT_EQ(9, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  // AsValidPosition should not change any fields on already-valid positions.
  EXPECT_TRUE(text_position->IsValid());
  test_position = text_position->AsValidPosition();
  EXPECT_TRUE(test_position->IsValid());
  EXPECT_EQ(*test_position, *text_position);

  // Now make a change to shorten MaxTextOffset. Ensure that this position is
  // invalid, then call AsValidPosition and ensure that it is now valid.
  text_data.SetName("some tex");
  AXTreeUpdate shorten_text_update;
  shorten_text_update.nodes = {text_data};
  ASSERT_TRUE(GetTree()->Unserialize(shorten_text_update));

  EXPECT_FALSE(text_position->IsValid());
  text_position = text_position->AsValidPosition();
  EXPECT_TRUE(text_position->IsValid());
  EXPECT_EQ(8, text_position->text_offset());

  // Now repeat the prior tests and ensure that we can create next character
  // positions with the new, valid MaxTextOffset (8).
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_TRUE(test_position->IsValid());
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(text_data.id, test_position->anchor_id());
  EXPECT_EQ(8, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());

  // AsValidPosition should create a NullPosition if a position's anchor is
  // removed. This is true for both tree positions and text positions.
  EXPECT_TRUE(text_position->IsValid());
  TestPositionType tree_position = text_position->AsTreePosition();
  ASSERT_NE(nullptr, tree_position);
  EXPECT_TRUE(tree_position->IsTreePosition());
  EXPECT_TRUE(tree_position->IsValid());
  EXPECT_EQ(0, tree_position->child_index());

  AXTreeUpdate remove_node_update;
  root_data.child_ids = {};
  remove_node_update.nodes = {root_data};
  ASSERT_TRUE(GetTree()->Unserialize(remove_node_update));
  EXPECT_FALSE(text_position->IsValid());
  EXPECT_FALSE(tree_position->IsValid());

  text_position = text_position->AsValidPosition();
  EXPECT_TRUE(text_position->IsValid());
  tree_position = tree_position->AsValidPosition();
  EXPECT_TRUE(tree_position->IsValid());

  EXPECT_TRUE(text_position->IsNullPosition());
  EXPECT_TRUE(tree_position->IsNullPosition());
}

TEST_F(AXPositionTest, AsValidPositionInDescendantOfEmptyObject) {
  g_ax_embedded_object_behavior = AXEmbeddedObjectBehavior::kExposeCharacter;

  // ++1 kRootWebArea
  // ++++2 kButton
  // ++++++3 kStaticText "3.14" ignored
  // ++++++++4 kInlineTextBox "3.14" ignored
  AXNodeData root_1;
  AXNodeData button_2;
  AXNodeData static_text_3;
  AXNodeData inline_box_4;

  root_1.id = 1;
  button_2.id = 2;
  static_text_3.id = 3;
  inline_box_4.id = 4;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {button_2.id};

  button_2.role = ax::mojom::Role::kButton;
  button_2.child_ids = {static_text_3.id};

  static_text_3.role = ax::mojom::Role::kStaticText;
  static_text_3.SetName("3.14");
  static_text_3.child_ids = {inline_box_4.id};

  inline_box_4.role = ax::mojom::Role::kInlineTextBox;
  inline_box_4.SetName("3.14");

  SetTree(CreateAXTree({root_1, button_2, static_text_3, inline_box_4}));

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_4.id, 3, ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->IsValid());
  EXPECT_EQ(*text_position, *text_position->AsValidPosition());

  TestPositionType tree_position =
      AXNodePosition::CreateTreePosition(GetTreeID(), inline_box_4.id, 0);
  ASSERT_NE(nullptr, tree_position);
  EXPECT_TRUE(tree_position->IsTreePosition());
  EXPECT_TRUE(tree_position->IsValid());
  EXPECT_EQ(*tree_position, *tree_position->AsValidPosition());

  static_text_3.AddState(ax::mojom::State::kIgnored);
  inline_box_4.AddState(ax::mojom::State::kIgnored);
  AXTreeUpdate update;
  update.nodes = {static_text_3, inline_box_4};
  ASSERT_TRUE(GetTree()->Unserialize(update));

  EXPECT_FALSE(text_position->IsValid());
  text_position = text_position->AsValidPosition();
  EXPECT_TRUE(text_position->IsValid());
  EXPECT_EQ(1, text_position->text_offset());

  EXPECT_FALSE(tree_position->IsValid());
  tree_position = tree_position->AsValidPosition();
  EXPECT_TRUE(tree_position->IsValid());
  EXPECT_EQ(0, tree_position->child_index());
}

TEST_F(AXPositionTest, CreateNextCharacterPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  TestPositionType test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(4, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(5, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(5, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(5, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(5, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(6, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());
  // Affinity should have been reset to downstream.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 12 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  EXPECT_EQ(13, test_position->text_offset());
  // Affinity should have been reset to downstream.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, CreatePreviousCharacterPosition) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  TestPositionType test_position =
      text_position->CreatePreviousCharacterPosition(
          AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(5, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(4, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(4, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(4, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(1, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box2_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(line_break_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(inline_box1_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopAtAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(check_box_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());

  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  test_position = text_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(text_field_.id, test_position->anchor_id());
  EXPECT_EQ(0, test_position->text_offset());
  // Affinity should have been reset to downstream.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
}

TEST_F(AXPositionTest, CreateNextCharacterPositionAtGraphemeBoundary) {
#if true
  GTEST_SKIP()
      << "Skipping, current accessibility library cannot handle grapheme";
#else
  std::vector<int> text_offsets;
  SetTree(CreateMultilingualDocument(&text_offsets));

  TestPositionType test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, test_position);
  ASSERT_TRUE(test_position->IsTextPosition());

  for (auto iter = (text_offsets.begin() + 1); iter != text_offsets.end();
       ++iter) {
    const int text_offset = *iter;
    test_position = test_position->CreateNextCharacterPosition(
        AXBoundaryBehavior::CrossBoundary);
    ASSERT_NE(nullptr, test_position);
    EXPECT_TRUE(test_position->IsTextPosition());

    testing::Message message;
    message << "Expecting character boundary at " << text_offset << " in\n"
            << *test_position;
    SCOPED_TRACE(message);

    EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
    EXPECT_EQ(text_offset, test_position->text_offset());
    EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
  }

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  test_position = test_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
  EXPECT_EQ(3, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  test_position = test_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
  EXPECT_EQ(5, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 9 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  test_position = test_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
  EXPECT_EQ(9, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kUpstream, test_position->affinity());

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 10 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  test_position = test_position->CreateNextCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
  EXPECT_EQ(12, test_position->text_offset());
  // Affinity should have been reset to downstream because there was a move.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
#endif  // true
}

TEST_F(AXPositionTest, CreatePreviousCharacterPositionAtGraphemeBoundary) {
#if true
  GTEST_SKIP()
      << "Skipping, current accessibility library cannot handle grapheme";
#else
  std::vector<int> text_offsets;
  SetTree(CreateMultilingualDocument(&text_offsets));

  TestPositionType test_position =
      AXNodePosition::CreateTextPosition(GetTreeID(), GetTree()->root()->id(),
                                         text_offsets.back() /* text_offset */,
                                         ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, test_position);
  ASSERT_TRUE(test_position->IsTextPosition());

  for (auto iter = (text_offsets.rbegin() + 1); iter != text_offsets.rend();
       ++iter) {
    const int text_offset = *iter;
    test_position = test_position->CreatePreviousCharacterPosition(
        AXBoundaryBehavior::CrossBoundary);
    ASSERT_NE(nullptr, test_position);
    EXPECT_TRUE(test_position->IsTextPosition());

    testing::Message message;
    message << "Expecting character boundary at " << text_offset << " in\n"
            << *test_position;
    SCOPED_TRACE(message);

    EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
    EXPECT_EQ(text_offset, test_position->text_offset());
    EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
  }

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  test_position = test_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
  EXPECT_EQ(3, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 4 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  test_position = test_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
  EXPECT_EQ(3, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 9 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  test_position = test_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
  EXPECT_EQ(9, test_position->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kUpstream, test_position->affinity());

  test_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetTree()->root()->id(), 10 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  test_position = test_position->CreatePreviousCharacterPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  ASSERT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsTextPosition());
  EXPECT_EQ(GetTree()->root()->id(), test_position->anchor_id());
  EXPECT_EQ(9, test_position->text_offset());
  // Affinity should have been reset to downstream because there was a move.
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, test_position->affinity());
#endif  // true
}

TEST_F(AXPositionTest, ReciprocalCreateNextAndPreviousCharacterPosition) {
  TestPositionType tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  TestPositionType text_position = tree_position->AsTextPosition();
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  size_t next_character_moves = 0;
  while (!text_position->IsNullPosition()) {
    TestPositionType moved_position =
        text_position->CreateNextCharacterPosition(
            AXBoundaryBehavior::CrossBoundary);
    ASSERT_NE(nullptr, moved_position);

    text_position = std::move(moved_position);
    ++next_character_moves;
  }

  tree_position = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, root_.child_ids.size() /* child_index */);
  text_position = tree_position->AsTextPosition();
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  size_t previous_character_moves = 0;
  while (!text_position->IsNullPosition()) {
    TestPositionType moved_position =
        text_position->CreatePreviousCharacterPosition(
            AXBoundaryBehavior::CrossBoundary);
    ASSERT_NE(nullptr, moved_position);

    text_position = std::move(moved_position);
    ++previous_character_moves;
  }

  EXPECT_EQ(next_character_moves, previous_character_moves);
  EXPECT_EQ(strlen(TEXT_VALUE), next_character_moves - 1);
}

TEST_F(AXPositionTest, CreateNextAndPreviousWordStartPositionWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->CreateNextWordStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = null_position->CreatePreviousWordStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, CreateNextAndPreviousWordEndPositionWithNullPosition) {
  TestPositionType null_position = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position);
  TestPositionType test_position = null_position->CreateNextWordEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
  test_position = null_position->CreatePreviousWordEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  EXPECT_NE(nullptr, test_position);
  EXPECT_TRUE(test_position->IsNullPosition());
}

TEST_F(AXPositionTest, OperatorEquals) {
  TestPositionType null_position1 = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position1);
  TestPositionType null_position2 = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position2);
  EXPECT_EQ(*null_position1, *null_position2);

  // Child indices must match.
  TestPositionType button_position1 = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, button_position1);
  TestPositionType button_position2 = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, button_position2);
  EXPECT_EQ(*button_position1, *button_position2);

  // Both child indices are invalid. It should result in equivalent null
  // positions.
  TestPositionType tree_position1 = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 4 /* child_index */);
  ASSERT_NE(nullptr, tree_position1);
  TestPositionType tree_position2 = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, AXNodePosition::INVALID_INDEX);
  ASSERT_NE(nullptr, tree_position2);
  EXPECT_EQ(*tree_position1, *tree_position2);

  // An invalid position should not be equivalent to an "after children"
  // position.
  tree_position1 = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                      3 /* child_index */);
  ASSERT_NE(nullptr, tree_position1);
  tree_position2 = AXNodePosition::CreateTreePosition(GetTreeID(), root_.id,
                                                      -1 /* child_index */);
  ASSERT_NE(nullptr, tree_position2);
  EXPECT_NE(*tree_position1, *tree_position2);

  // Two "after children" positions on the same node should be equivalent.
  tree_position1 = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 3 /* child_index */);
  ASSERT_NE(nullptr, tree_position1);
  tree_position2 = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 3 /* child_index */);
  ASSERT_NE(nullptr, tree_position2);
  EXPECT_EQ(*tree_position1, *tree_position2);

  // Two "before text" positions on the same node should be equivalent.
  tree_position1 = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_NE(nullptr, tree_position1);
  tree_position2 = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_NE(nullptr, tree_position2);
  EXPECT_EQ(*tree_position1, *tree_position2);

  // Both text offsets are invalid. It should result in equivalent null
  // positions.
  TestPositionType text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 15 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsNullPosition());
  TestPositionType text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, -1 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsNullPosition());
  EXPECT_EQ(*text_position1, *text_position2);

  text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_EQ(*text_position1, *text_position2);

  // Affinities should not matter.
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_EQ(*text_position1, *text_position2);

  // Text offsets should match.
  text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  EXPECT_NE(*text_position1, *text_position2);

  // Two "after text" positions on the same node should be equivalent.
  text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_EQ(*text_position1, *text_position2);

  // Two text positions that are consecutive, one "before text" and one "after
  // text".
  text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_EQ(*text_position1, *text_position2);

  // Two "after text" positions on a parent and child should be equivalent, in
  // the middle of the document...
  text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_EQ(*text_position1, *text_position2);

  // ...and at the end of the document.
  text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  // Validate that we're actually at the end of the document by normalizing to
  // the equivalent "before character" position.
  EXPECT_TRUE(
      text_position1->AsLeafTextPositionBeforeCharacter()->IsNullPosition());
  EXPECT_TRUE(
      text_position2->AsLeafTextPositionBeforeCharacter()->IsNullPosition());
  // Now compare the positions.
  EXPECT_EQ(*text_position1, *text_position2);
}

TEST_F(AXPositionTest, OperatorEqualsSameTextOffsetSameAnchorId) {
  TestPositionType text_position_one = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position_one);
  ASSERT_TRUE(text_position_one->IsTextPosition());

  TestPositionType text_position_two = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position_two);
  ASSERT_TRUE(text_position_two->IsTextPosition());

  ASSERT_TRUE(*text_position_one == *text_position_two);
  ASSERT_TRUE(*text_position_two == *text_position_one);
}

TEST_F(AXPositionTest, OperatorEqualsSameTextOffsetDifferentAnchorIdRoot) {
  TestPositionType text_position_one = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position_one);
  ASSERT_TRUE(text_position_one->IsTextPosition());

  TestPositionType text_position_two = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position_two);
  ASSERT_TRUE(text_position_two->IsTextPosition());

  ASSERT_TRUE(*text_position_one == *text_position_two);
  ASSERT_TRUE(*text_position_two == *text_position_one);
}

TEST_F(AXPositionTest, OperatorEqualsSameTextOffsetDifferentAnchorIdLeaf) {
  TestPositionType text_position_one = AXNodePosition::CreateTextPosition(
      GetTreeID(), button_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position_one);
  ASSERT_TRUE(text_position_one->IsTextPosition());

  TestPositionType text_position_two = AXNodePosition::CreateTextPosition(
      GetTreeID(), check_box_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position_two);
  ASSERT_TRUE(text_position_two->IsTextPosition());

  ASSERT_TRUE(*text_position_one == *text_position_two);
  ASSERT_TRUE(*text_position_two == *text_position_one);
}

TEST_F(AXPositionTest, OperatorsLessThanAndGreaterThan) {
  TestPositionType null_position1 = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position1);
  TestPositionType null_position2 = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position2);
  EXPECT_FALSE(*null_position1 < *null_position2);
  EXPECT_FALSE(*null_position1 > *null_position2);

  TestPositionType button_position1 = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, button_position1);
  TestPositionType button_position2 = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 1 /* child_index */);
  ASSERT_NE(nullptr, button_position2);
  EXPECT_LT(*button_position1, *button_position2);
  EXPECT_GT(*button_position2, *button_position1);

  TestPositionType tree_position1 = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 2 /* child_index */);
  ASSERT_NE(nullptr, tree_position1);
  // An "after children" position.
  TestPositionType tree_position2 = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 3 /* child_index */);
  ASSERT_NE(nullptr, tree_position2);
  EXPECT_LT(*tree_position1, *tree_position2);
  EXPECT_GT(*tree_position2, *tree_position1);

  // A "before text" position.
  tree_position1 = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, AXNodePosition::BEFORE_TEXT);
  ASSERT_NE(nullptr, tree_position1);
  // An "after text" position.
  tree_position2 = AXNodePosition::CreateTreePosition(
      GetTreeID(), inline_box1_.id, 0 /* child_index */);
  ASSERT_NE(nullptr, tree_position2);
  EXPECT_LT(*tree_position1, *tree_position2);
  EXPECT_GT(*tree_position2, *tree_position1);

  // Two text positions that share a common anchor.
  TestPositionType text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 2 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  TestPositionType text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_GT(*text_position1, *text_position2);
  EXPECT_LT(*text_position2, *text_position1);

  // Affinities should not matter.
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_GT(*text_position1, *text_position2);
  EXPECT_LT(*text_position2, *text_position1);

  // An "after text" position.
  text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  // A "before text" position.
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_GT(*text_position1, *text_position2);
  EXPECT_LT(*text_position2, *text_position1);

  // A text position that is an ancestor of another.
  text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1_.id, 5 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_GT(*text_position1, *text_position2);
  EXPECT_LT(*text_position2, *text_position1);

  // Two text positions that share a common ancestor.
  text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_GT(*text_position1, *text_position2);
  EXPECT_LT(*text_position2, *text_position1);

  // Two consecutive positions. One "before text" and one "after text".
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_EQ(*text_position1, *text_position2);

  // A text position at the end of the document versus one that isn't.
  text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box2_.id, 6 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_TRUE(text_position1->IsTextPosition());
  // Validate that we're actually at the end of the document by normalizing to
  // the equivalent "before character" position.
  EXPECT_TRUE(
      text_position1->AsLeafTextPositionBeforeCharacter()->IsNullPosition());
  // Now create the not-at-end-of-document position and compare.
  text_position2 = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position2);
  ASSERT_TRUE(text_position2->IsTextPosition());
  EXPECT_GT(*text_position1, *text_position2);
  EXPECT_LT(*text_position2, *text_position1);
}

TEST_F(AXPositionTest, Swap) {
  TestPositionType null_position1 = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position1);
  TestPositionType null_position2 = AXNodePosition::CreateNullPosition();
  ASSERT_NE(nullptr, null_position2);

  swap(*null_position1, *null_position2);
  EXPECT_TRUE(null_position1->IsNullPosition());
  EXPECT_TRUE(null_position2->IsNullPosition());

  TestPositionType tree_position1 = AXNodePosition::CreateTreePosition(
      GetTreeID(), root_.id, 2 /* child_index */);
  ASSERT_NE(nullptr, tree_position1);
  TestPositionType tree_position2 = AXNodePosition::CreateTreePosition(
      GetTreeID(), text_field_.id, 3 /* child_index */);
  ASSERT_NE(nullptr, tree_position2);

  swap(*tree_position1, *tree_position2);
  EXPECT_TRUE(tree_position1->IsTreePosition());
  EXPECT_EQ(GetTreeID(), tree_position1->tree_id());
  EXPECT_EQ(text_field_.id, tree_position1->anchor_id());
  EXPECT_EQ(3, tree_position1->child_index());
  EXPECT_TRUE(tree_position1->IsTreePosition());
  EXPECT_EQ(GetTreeID(), tree_position2->tree_id());
  EXPECT_EQ(root_.id, tree_position2->anchor_id());
  EXPECT_EQ(2, tree_position2->child_index());

  swap(*tree_position1, *null_position1);
  EXPECT_TRUE(tree_position1->IsNullPosition());
  EXPECT_TRUE(null_position1->IsTreePosition());
  EXPECT_EQ(GetTreeID(), null_position1->tree_id());
  EXPECT_EQ(text_field_.id, null_position1->anchor_id());
  EXPECT_EQ(3, null_position1->child_index());

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), line_break_.id, 1 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);

  swap(*text_position, *null_position1);
  EXPECT_TRUE(null_position1->IsTextPosition());
  EXPECT_EQ(GetTreeID(), text_position->tree_id());
  EXPECT_EQ(line_break_.id, null_position1->anchor_id());
  EXPECT_EQ(1, null_position1->text_offset());
  EXPECT_EQ(ax::mojom::TextAffinity::kDownstream, null_position1->affinity());
  EXPECT_TRUE(text_position->IsTreePosition());
  EXPECT_EQ(GetTreeID(), text_position->tree_id());
  EXPECT_EQ(text_field_.id, text_position->anchor_id());
  EXPECT_EQ(3, text_position->child_index());
}

TEST_F(AXPositionTest, CreateNextAnchorPosition) {
  // This test updates the tree structure to test a specific edge case -
  // CreateNextAnchorPosition on an empty text field.
  AXNodeData root_data;
  root_data.id = 1;
  root_data.role = ax::mojom::Role::kRootWebArea;

  AXNodeData text_data;
  text_data.id = 2;
  text_data.role = ax::mojom::Role::kStaticText;
  text_data.SetName("some text");

  AXNodeData text_field_data;
  text_field_data.id = 3;
  text_field_data.role = ax::mojom::Role::kTextField;

  AXNodeData empty_text_data;
  empty_text_data.id = 4;
  empty_text_data.role = ax::mojom::Role::kStaticText;
  empty_text_data.SetName("");

  AXNodeData more_text_data;
  more_text_data.id = 5;
  more_text_data.role = ax::mojom::Role::kStaticText;
  more_text_data.SetName("more text");

  root_data.child_ids = {text_data.id, text_field_data.id, more_text_data.id};
  text_field_data.child_ids = {empty_text_data.id};

  SetTree(CreateAXTree({root_data, text_data, text_field_data, empty_text_data,
                        more_text_data}));

  // Test that CreateNextAnchorPosition will successfully navigate past the
  // empty text field.
  TestPositionType text_position1 = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_data.id, 8 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position1);
  ASSERT_FALSE(text_position1->CreateNextAnchorPosition()
                   ->CreateNextAnchorPosition()
                   ->IsNullPosition());
}

TEST_F(AXPositionTest, CreateLinePositionsMultipleAnchorsInSingleLine) {
  // This test updates the tree structure to test a specific edge case -
  // Create next and previous line start/end positions on a single line composed
  // by multiple anchors; only two line boundaries should be resolved: either
  // the start of the "before" text or at the end of "after".
  // ++1 kRootWebArea
  // ++++2 kStaticText
  // ++++++3 kInlineTextBox "before" kNextOnLineId=6
  // ++++4 kGenericContainer
  // ++++++5 kStaticText
  // ++++++++6 kInlineTextBox "inside" kPreviousOnLineId=3 kNextOnLineId=8
  // ++++7 kStaticText
  // ++++++8 kInlineTextBox "after" kPreviousOnLineId=6
  AXNodeData root;
  AXNodeData inline_box1;
  AXNodeData inline_box2;
  AXNodeData inline_box3;
  AXNodeData inline_block;
  AXNodeData static_text1;
  AXNodeData static_text2;
  AXNodeData static_text3;

  root.id = 1;
  static_text1.id = 2;
  inline_box1.id = 3;
  inline_block.id = 4;
  static_text2.id = 5;
  inline_box2.id = 6;
  static_text3.id = 7;
  inline_box3.id = 8;

  root.role = ax::mojom::Role::kRootWebArea;
  root.child_ids = {static_text1.id, inline_block.id, static_text3.id};

  static_text1.role = ax::mojom::Role::kStaticText;
  static_text1.SetName("before");
  static_text1.child_ids = {inline_box1.id};

  inline_box1.role = ax::mojom::Role::kInlineTextBox;
  inline_box1.SetName("before");
  inline_box1.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                              inline_box2.id);

  inline_block.role = ax::mojom::Role::kGenericContainer;
  inline_block.child_ids = {static_text2.id};

  static_text2.role = ax::mojom::Role::kStaticText;
  static_text2.SetName("inside");
  static_text2.child_ids = {inline_box2.id};

  inline_box2.role = ax::mojom::Role::kInlineTextBox;
  inline_box2.SetName("inside");
  inline_box2.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                              inline_box1.id);
  inline_box2.AddIntAttribute(ax::mojom::IntAttribute::kNextOnLineId,
                              inline_box3.id);

  static_text3.role = ax::mojom::Role::kStaticText;
  static_text3.SetName("after");
  static_text3.child_ids = {inline_box3.id};

  inline_box3.role = ax::mojom::Role::kInlineTextBox;
  inline_box3.SetName("after");
  inline_box3.AddIntAttribute(ax::mojom::IntAttribute::kPreviousOnLineId,
                              inline_box2.id);

  SetTree(CreateAXTree({root, static_text1, inline_box1, inline_block,
                        static_text2, inline_box2, static_text3, inline_box3}));

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_block.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());

  TestPositionType next_line_start_position =
      text_position->CreateNextLineStartPosition(
          AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, next_line_start_position);
  EXPECT_TRUE(next_line_start_position->IsTextPosition());
  EXPECT_EQ(inline_box3.id, next_line_start_position->anchor_id());
  EXPECT_EQ(5, next_line_start_position->text_offset());

  TestPositionType previous_line_start_position =
      text_position->CreatePreviousLineStartPosition(
          AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, previous_line_start_position);
  EXPECT_TRUE(previous_line_start_position->IsTextPosition());
  EXPECT_EQ(inline_box1.id, previous_line_start_position->anchor_id());
  EXPECT_EQ(0, previous_line_start_position->text_offset());

  TestPositionType next_line_end_position =
      text_position->CreateNextLineEndPosition(
          AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, next_line_end_position);
  EXPECT_TRUE(next_line_end_position->IsTextPosition());
  EXPECT_EQ(inline_box3.id, next_line_end_position->anchor_id());
  EXPECT_EQ(5, next_line_end_position->text_offset());

  TestPositionType previous_line_end_position =
      text_position->CreatePreviousLineEndPosition(
          AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, previous_line_end_position);
  EXPECT_TRUE(previous_line_end_position->IsTextPosition());
  EXPECT_EQ(inline_box1.id, previous_line_end_position->anchor_id());
  EXPECT_EQ(0, previous_line_end_position->text_offset());
}

TEST_F(AXPositionTest, CreateNextWordPositionInList) {
  // This test updates the tree structure to test a specific edge case -
  // next word navigation inside a list with AXListMarkers nodes.
  // ++1 kRootWebArea
  // ++++2 kList
  // ++++++3 kListItem
  // ++++++++4 kListMarker
  // ++++++++++5 kStaticText
  // ++++++++++++6 kInlineTextBox "1. "
  // ++++++++7 kStaticText
  // ++++++++++8 kInlineTextBox "first item"
  // ++++++9 kListItem
  // ++++++++10 kListMarker
  // +++++++++++11 kStaticText
  // ++++++++++++++12 kInlineTextBox "2. "
  // ++++++++13 kStaticText
  // ++++++++++14 kInlineTextBox "second item"
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
  root.child_ids = {list.id};

  list.role = ax::mojom::Role::kList;
  list.child_ids = {list_item1.id, list_item2.id};

  list_item1.role = ax::mojom::Role::kListItem;
  list_item1.child_ids = {list_marker1.id, static_text2.id};
  list_item1.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);

  list_marker1.role = ax::mojom::Role::kListMarker;
  list_marker1.child_ids = {static_text1.id};

  static_text1.role = ax::mojom::Role::kStaticText;
  static_text1.SetName("1. ");
  static_text1.child_ids = {inline_box1.id};

  inline_box1.role = ax::mojom::Role::kInlineTextBox;
  inline_box1.SetName("1. ");
  inline_box1.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                  std::vector<int32_t>{0});
  inline_box1.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                  std::vector<int32_t>{3});

  static_text2.role = ax::mojom::Role::kStaticText;
  static_text2.SetName("first item");
  static_text2.child_ids = {inline_box2.id};

  inline_box2.role = ax::mojom::Role::kInlineTextBox;
  inline_box2.SetName("first item");
  inline_box2.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                  std::vector<int32_t>{0, 6});
  inline_box2.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                  std::vector<int32_t>{5});

  list_item2.role = ax::mojom::Role::kListItem;
  list_item2.child_ids = {list_marker2.id, static_text4.id};
  list_item2.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);

  list_marker2.role = ax::mojom::Role::kListMarker;
  list_marker2.child_ids = {static_text3.id};

  static_text3.role = ax::mojom::Role::kStaticText;
  static_text3.SetName("2. ");
  static_text3.child_ids = {inline_box3.id};

  inline_box3.role = ax::mojom::Role::kInlineTextBox;
  inline_box3.SetName("2. ");
  inline_box3.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                  std::vector<int32_t>{0});
  inline_box3.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                  std::vector<int32_t>{3});

  static_text4.role = ax::mojom::Role::kStaticText;
  static_text4.SetName("second item");
  static_text4.child_ids = {inline_box4.id};

  inline_box4.role = ax::mojom::Role::kInlineTextBox;
  inline_box4.SetName("second item");
  inline_box4.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                  std::vector<int32_t>{0, 7});
  inline_box4.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                  std::vector<int32_t>{6});

  SetTree(CreateAXTree({root, list, list_item1, list_marker1, static_text1,
                        inline_box1, static_text2, inline_box2, list_item2,
                        list_marker2, static_text3, inline_box3, static_text4,
                        inline_box4}));

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box1.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box1.id, text_position->anchor_id());
  ASSERT_EQ(0, text_position->text_offset());

  // "1. <f>irst item\n2. second item"
  text_position = text_position->CreateNextWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box2.id, text_position->anchor_id());
  ASSERT_EQ(0, text_position->text_offset());

  // "1. first <i>tem\n2. second item"
  text_position = text_position->CreateNextWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box2.id, text_position->anchor_id());
  ASSERT_EQ(6, text_position->text_offset());

  // "1. first item\n<2>. second item"
  text_position = text_position->CreateNextWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box3.id, text_position->anchor_id());
  ASSERT_EQ(0, text_position->text_offset());

  // "1. first item\n2. <s>econd item"
  text_position = text_position->CreateNextWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box4.id, text_position->anchor_id());
  ASSERT_EQ(0, text_position->text_offset());

  // "1. first item\n2. second <i>tem"
  text_position = text_position->CreateNextWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box4.id, text_position->anchor_id());
  ASSERT_EQ(7, text_position->text_offset());
}

TEST_F(AXPositionTest, CreatePreviousWordPositionInList) {
  // This test updates the tree structure to test a specific edge case -
  // previous word navigation inside a list with AXListMarkers nodes.
  // ++1 kRootWebArea
  // ++++2 kList
  // ++++++3 kListItem
  // ++++++++4 kListMarker
  // ++++++++++5 kStaticText
  // ++++++++++++6 kInlineTextBox "1. "
  // ++++++++7 kStaticText
  // ++++++++++8 kInlineTextBox "first item"
  // ++++++9 kListItem
  // ++++++++10 kListMarker
  // +++++++++++11 kStaticText
  // ++++++++++++++12 kInlineTextBox "2. "
  // ++++++++13 kStaticText
  // ++++++++++14 kInlineTextBox "second item"
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
  root.child_ids = {list.id};

  list.role = ax::mojom::Role::kList;
  list.child_ids = {list_item1.id, list_item2.id};

  list_item1.role = ax::mojom::Role::kListItem;
  list_item1.child_ids = {list_marker1.id, static_text2.id};
  list_item1.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);

  list_marker1.role = ax::mojom::Role::kListMarker;
  list_marker1.child_ids = {static_text1.id};

  static_text1.role = ax::mojom::Role::kStaticText;
  static_text1.SetName("1. ");
  static_text1.child_ids = {inline_box1.id};

  inline_box1.role = ax::mojom::Role::kInlineTextBox;
  inline_box1.SetName("1. ");
  inline_box1.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                  std::vector<int32_t>{0});
  inline_box1.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                  std::vector<int32_t>{3});

  static_text2.role = ax::mojom::Role::kStaticText;
  static_text2.SetName("first item");
  static_text2.child_ids = {inline_box2.id};

  inline_box2.role = ax::mojom::Role::kInlineTextBox;
  inline_box2.SetName("first item");
  inline_box2.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                  std::vector<int32_t>{0, 6});
  inline_box2.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                  std::vector<int32_t>{5});

  list_item2.role = ax::mojom::Role::kListItem;
  list_item2.child_ids = {list_marker2.id, static_text4.id};
  list_item2.AddBoolAttribute(ax::mojom::BoolAttribute::kIsLineBreakingObject,
                              true);

  list_marker2.role = ax::mojom::Role::kListMarker;
  list_marker2.child_ids = {static_text3.id};

  static_text3.role = ax::mojom::Role::kStaticText;
  static_text3.SetName("2. ");
  static_text3.child_ids = {inline_box3.id};

  inline_box3.role = ax::mojom::Role::kInlineTextBox;
  inline_box3.SetName("2. ");
  inline_box3.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                  std::vector<int32_t>{0});
  inline_box3.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                  std::vector<int32_t>{3});

  static_text4.role = ax::mojom::Role::kStaticText;
  static_text4.SetName("second item");
  static_text4.child_ids = {inline_box4.id};

  inline_box4.role = ax::mojom::Role::kInlineTextBox;
  inline_box4.SetName("second item");
  inline_box4.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                  std::vector<int32_t>{0, 7});
  inline_box4.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                  std::vector<int32_t>{6});

  SetTree(CreateAXTree({root, list, list_item1, list_marker1, static_text1,
                        inline_box1, static_text2, inline_box2, list_item2,
                        list_marker2, static_text3, inline_box3, static_text4,
                        inline_box4}));

  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box4.id, 11 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box4.id, text_position->anchor_id());
  ASSERT_EQ(11, text_position->text_offset());

  // "1. first item\n2. second <i>tem"
  text_position = text_position->CreatePreviousWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box4.id, text_position->anchor_id());
  ASSERT_EQ(7, text_position->text_offset());

  // "1. first item\n2. <s>econd item"
  text_position = text_position->CreatePreviousWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box4.id, text_position->anchor_id());
  ASSERT_EQ(0, text_position->text_offset());

  // "1. first item\n<2>. second item"
  text_position = text_position->CreatePreviousWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box3.id, text_position->anchor_id());
  ASSERT_EQ(0, text_position->text_offset());

  // "1. first <i>tem\n2. <s>econd item"
  text_position = text_position->CreatePreviousWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box2.id, text_position->anchor_id());
  ASSERT_EQ(6, text_position->text_offset());

  // "1. <f>irst item\n2. second item"
  text_position = text_position->CreatePreviousWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box2.id, text_position->anchor_id());
  ASSERT_EQ(0, text_position->text_offset());

  // "<1>. first item\n2. second item"
  text_position = text_position->CreatePreviousWordStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, text_position);
  ASSERT_TRUE(text_position->IsTextPosition());
  ASSERT_EQ(inline_box1.id, text_position->anchor_id());
  ASSERT_EQ(0, text_position->text_offset());
}

TEST_F(AXPositionTest, EmptyObjectReplacedByCharacterTextNavigation) {
  g_ax_embedded_object_behavior = AXEmbeddedObjectBehavior::kExposeCharacter;

  // ++1 kRootWebArea
  // ++++2 kStaticText
  // ++++++3 kInlineTextBox
  // ++++4 kTextField
  // ++++++5 kGenericContainer
  // ++++6 kStaticText
  // ++++++7 kInlineTextBox
  // ++++8 kHeading
  // ++++++9 kStaticText
  // ++++++++10 kInlineTextBox
  // ++++11 kGenericContainer ignored
  // ++++12 kGenericContainer
  // ++++13 kStaticText
  // ++++14 kButton
  // ++++++15 kGenericContainer ignored
  AXNodeData root_1;
  AXNodeData static_text_2;
  AXNodeData inline_box_3;
  AXNodeData text_field_4;
  AXNodeData generic_container_5;
  AXNodeData static_text_6;
  AXNodeData inline_box_7;
  AXNodeData heading_8;
  AXNodeData static_text_9;
  AXNodeData inline_box_10;
  AXNodeData generic_container_11;
  AXNodeData generic_container_12;
  AXNodeData static_text_13;
  AXNodeData button_14;
  AXNodeData generic_container_15;

  root_1.id = 1;
  static_text_2.id = 2;
  inline_box_3.id = 3;
  text_field_4.id = 4;
  generic_container_5.id = 5;
  static_text_6.id = 6;
  inline_box_7.id = 7;
  heading_8.id = 8;
  static_text_9.id = 9;
  inline_box_10.id = 10;
  generic_container_11.id = 11;
  generic_container_12.id = 12;
  static_text_13.id = 13;
  button_14.id = 14;
  generic_container_15.id = 15;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {static_text_2.id,        text_field_4.id,
                      static_text_6.id,        heading_8.id,
                      generic_container_11.id, generic_container_12.id,
                      static_text_13.id,       button_14.id};

  static_text_2.role = ax::mojom::Role::kStaticText;
  static_text_2.SetName("Hello ");
  static_text_2.child_ids = {inline_box_3.id};

  inline_box_3.role = ax::mojom::Role::kInlineTextBox;
  inline_box_3.SetName("Hello ");
  inline_box_3.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                   std::vector<int32_t>{0});
  inline_box_3.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                   std::vector<int32_t>{6});

  text_field_4.role = ax::mojom::Role::kGroup;
  text_field_4.child_ids = {generic_container_5.id};

  generic_container_5.role = ax::mojom::Role::kGenericContainer;

  static_text_6.role = ax::mojom::Role::kStaticText;
  static_text_6.SetName(" world");
  static_text_6.child_ids = {inline_box_7.id};

  inline_box_7.role = ax::mojom::Role::kInlineTextBox;
  inline_box_7.SetName(" world");
  inline_box_7.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                   std::vector<int32_t>{1});
  inline_box_7.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds,
                                   std::vector<int32_t>{6});

  heading_8.role = ax::mojom::Role::kHeading;
  heading_8.child_ids = {static_text_9.id};

  static_text_9.role = ax::mojom::Role::kStaticText;
  static_text_9.child_ids = {inline_box_10.id};
  static_text_9.SetName("3.14");

  inline_box_10.role = ax::mojom::Role::kInlineTextBox;
  inline_box_10.SetName("3.14");

  generic_container_11.role = ax::mojom::Role::kGenericContainer;
  generic_container_11.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);
  generic_container_11.AddState(ax::mojom::State::kIgnored);

  generic_container_12.role = ax::mojom::Role::kGenericContainer;
  generic_container_12.AddBoolAttribute(
      ax::mojom::BoolAttribute::kIsLineBreakingObject, true);

  static_text_13.role = ax::mojom::Role::kStaticText;
  static_text_13.SetName("hey");

  button_14.role = ax::mojom::Role::kButton;
  button_14.child_ids = {generic_container_15.id};

  generic_container_15.role = ax::mojom::Role::kGenericContainer;
  generic_container_15.AddState(ax::mojom::State::kIgnored);

  SetTree(CreateAXTree({root_1, static_text_2, inline_box_3, text_field_4,
                        generic_container_5, static_text_6, inline_box_7,
                        heading_8, static_text_9, inline_box_10,
                        generic_container_11, generic_container_12,
                        static_text_13, button_14, generic_container_15}));

  // CreateNextWordStartPosition tests.
  TestPositionType position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_3.id, 0 /* child_index_or_text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  TestPositionType result_position =
      position->CreateNextWordStartPosition(AXBoundaryBehavior::CrossBoundary);
  std::string expectations =
      "TextPosition anchor_id=5 text_offset=0 affinity=downstream "
      "annotated_text=<\xEF\xBF\xBC>";
  ASSERT_EQ(result_position->ToString(), expectations);

  position = std::move(result_position);
  result_position =
      position->CreateNextWordStartPosition(AXBoundaryBehavior::CrossBoundary);
  expectations =
      "TextPosition anchor_id=7 text_offset=1 affinity=downstream "
      "annotated_text= <w>orld";
  ASSERT_EQ(result_position->ToString(), expectations);

  // CreatePreviousWordStartPosition tests.
  position = std::move(result_position);
  result_position = position->CreatePreviousWordStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  expectations =
      "TextPosition anchor_id=5 text_offset=0 affinity=downstream "
      "annotated_text=<\xEF\xBF\xBC>";
  ASSERT_EQ(result_position->ToString(), expectations);

  position = std::move(result_position);
  result_position = position->CreatePreviousWordStartPosition(
      AXBoundaryBehavior::CrossBoundary);
  expectations =
      "TextPosition anchor_id=3 text_offset=0 affinity=downstream "
      "annotated_text=<H>ello ";
  ASSERT_EQ(result_position->ToString(), expectations);

  // CreateNextWordEndPosition tests.
  position = std::move(result_position);
  result_position =
      position->CreateNextWordEndPosition(AXBoundaryBehavior::CrossBoundary);
  expectations =
      "TextPosition anchor_id=3 text_offset=6 affinity=downstream "
      "annotated_text=Hello <>";
  ASSERT_EQ(result_position->ToString(), expectations);

  position = std::move(result_position);
  result_position =
      position->CreateNextWordEndPosition(AXBoundaryBehavior::CrossBoundary);
  expectations =
      "TextPosition anchor_id=5 text_offset=1 affinity=downstream "
      "annotated_text=\xEF\xBF\xBC<>";
  ASSERT_EQ(result_position->ToString(), expectations);

  position = std::move(result_position);
  result_position =
      position->CreateNextWordEndPosition(AXBoundaryBehavior::CrossBoundary);
  expectations =
      "TextPosition anchor_id=7 text_offset=6 affinity=downstream "
      "annotated_text= world<>";
  ASSERT_EQ(result_position->ToString(), expectations);

  // CreatePreviousWordEndPosition tests.
  position = std::move(result_position);
  result_position = position->CreatePreviousWordEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  expectations =
      "TextPosition anchor_id=5 text_offset=1 affinity=downstream "
      "annotated_text=\xEF\xBF\xBC<>";
  ASSERT_EQ(result_position->ToString(), expectations);

  position = std::move(result_position);
  result_position = position->CreatePreviousWordEndPosition(
      AXBoundaryBehavior::CrossBoundary);
  expectations =
      "TextPosition anchor_id=3 text_offset=6 affinity=downstream "
      "annotated_text=Hello <>";
  ASSERT_EQ(result_position->ToString(), expectations);

  // GetText() with embedded object replacement character test.
  position = AXNodePosition::CreateTextPosition(
      GetTreeID(), generic_container_5.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  std::u16string expected_text;
  expected_text += AXNodePosition::kEmbeddedCharacter;
  ASSERT_EQ(expected_text, position->GetText());

  // GetText() on a node parent of text nodes and an embedded object replacement
  // character.
  position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_1.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  expected_text =
      std::u16string(u"Hello ") + AXNodePosition::kEmbeddedCharacter +
      std::u16string(u" world3.14") + AXNodePosition::kEmbeddedCharacter +
      std::u16string(u"hey") + AXNodePosition::kEmbeddedCharacter;
  ASSERT_EQ(expected_text, position->GetText());

  // MaxTextOffset() with an embedded object replacement character.
  position = AXNodePosition::CreateTextPosition(
      GetTreeID(), generic_container_5.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);

  ASSERT_EQ(1, position->MaxTextOffset());

  // Parent positions created from a position inside a node represented by an
  // embedded object replacement character.
  position = position->CreateParentPosition();
  expectations =
      "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
      "annotated_text=<\xEF\xBF\xBC>";
  ASSERT_EQ(position->ToString(), expectations);
  ASSERT_EQ(1, position->MaxTextOffset());

  position = position->CreateParentPosition();
  expectations =
      "TextPosition anchor_id=1 text_offset=6 affinity=downstream "
      "annotated_text=Hello <\xEF\xBF\xBC> "
      "world3.14\xEF\xBF\xBChey\xEF\xBF\xBC";
  ASSERT_EQ(position->ToString(), expectations);
  ASSERT_EQ(22, position->MaxTextOffset());

  // MaxTextOffset() on a node parent of text nodes and an embedded object
  // replacement character.
  position = AXNodePosition::CreateTextPosition(
      GetTreeID(), root_1.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_EQ(22, position->MaxTextOffset());

  // The following is to test a specific edge case with heading navigation,
  // occurring in AXPosition::CreatePreviousFormatStartPosition.
  //
  // When the position is at the beginning of an unignored empty object,
  // preceded by an ignored empty object itself preceded by an heading node, the
  // previous format start position should stay on this unignored empty object.
  // It shouldn't move to the beginning of the heading.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), generic_container_12.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);

  text_position = text_position->CreatePreviousFormatStartPosition(
      AXBoundaryBehavior::StopIfAlreadyAtBoundary);
  EXPECT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->IsTextPosition());
  EXPECT_EQ(generic_container_12.id, text_position->anchor_id());
  EXPECT_EQ(0, text_position->text_offset());

  // The following is to test a specific edge case that occurs when all the
  // children of a node are ignored and that node could be considered as an
  // empty object replaced by character (e.g., a button).
  //
  // The button element should be treated as a leaf node even though it has a
  // child. Because its only child is ignored, the button should be considered
  // as an empty object replaced by character and we should be able to create a
  // leaf position in the button node.
  text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text_13.id, 3 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, text_position);

  text_position = text_position->CreateNextParagraphEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  EXPECT_NE(nullptr, text_position);
  EXPECT_TRUE(text_position->IsTextPosition());
  EXPECT_TRUE(text_position->IsLeafTextPosition());
  EXPECT_EQ(button_14.id, text_position->anchor_id());
  EXPECT_EQ(1, text_position->text_offset());
}

TEST_F(AXPositionTest, TextNavigationWithCollapsedCombobox) {
  // On Windows, a <select> element is replaced by a combobox that contains
  // an AXMenuListPopup parent of AXMenuListOptions. When the select dropdown is
  // collapsed, the subtree of that combobox needs to be hidden and, when
  // expanded, it must be accessible in the tree. This test ensures we can't
  // navigate into the options of a collapsed menu list popup.
  g_ax_embedded_object_behavior = AXEmbeddedObjectBehavior::kExposeCharacter;

  // ++1 kRootWebArea
  // ++++2 kStaticText "Hi"
  // ++++++3 kInlineTextBox "Hi"
  // ++++4 kPopUpButton
  // ++++++5 kMenuListPopup
  // ++++++++6 kMenuListOption "Option"
  // ++++7 kStaticText "3.14"
  // ++++++8 kInlineTextBox "3.14"
  AXNodeData root_1;
  AXNodeData static_text_2;
  AXNodeData inline_box_3;
  AXNodeData popup_button_4;
  AXNodeData menu_list_popup_5;
  AXNodeData menu_list_option_6;
  AXNodeData static_text_7;
  AXNodeData inline_box_8;

  root_1.id = 1;
  static_text_2.id = 2;
  inline_box_3.id = 3;
  popup_button_4.id = 4;
  menu_list_popup_5.id = 5;
  menu_list_option_6.id = 6;
  static_text_7.id = 7;
  inline_box_8.id = 8;

  root_1.role = ax::mojom::Role::kRootWebArea;
  root_1.child_ids = {static_text_2.id, popup_button_4.id, static_text_7.id};

  static_text_2.role = ax::mojom::Role::kStaticText;
  static_text_2.SetName("Hi");
  static_text_2.child_ids = {inline_box_3.id};

  inline_box_3.role = ax::mojom::Role::kInlineTextBox;
  inline_box_3.SetName("Hi");
  inline_box_3.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                   {0});
  inline_box_3.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds, {2});

  popup_button_4.role = ax::mojom::Role::kPopUpButton;
  popup_button_4.child_ids = {menu_list_popup_5.id};
  popup_button_4.AddState(ax::mojom::State::kCollapsed);

  menu_list_popup_5.role = ax::mojom::Role::kMenuListPopup;
  menu_list_popup_5.child_ids = {menu_list_option_6.id};

  menu_list_option_6.role = ax::mojom::Role::kMenuListOption;
  menu_list_option_6.SetName("Option");

  static_text_7.role = ax::mojom::Role::kStaticText;
  static_text_7.SetName("3.14");
  static_text_7.child_ids = {inline_box_8.id};

  inline_box_8.role = ax::mojom::Role::kInlineTextBox;
  inline_box_8.SetName("3.14");
  inline_box_8.AddIntListAttribute(ax::mojom::IntListAttribute::kWordStarts,
                                   {0});
  inline_box_8.AddIntListAttribute(ax::mojom::IntListAttribute::kWordEnds, {4});

  SetTree(CreateAXTree({root_1, static_text_2, inline_box_3, popup_button_4,
                        menu_list_popup_5, menu_list_option_6, static_text_7,
                        inline_box_8}));

  // Collapsed - Forward navigation.
  TestPositionType position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_3.id, 0, ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, position);

  position = position->CreateNextParagraphStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, position);
  EXPECT_EQ(popup_button_4.id, position->anchor_id());
  EXPECT_EQ(0, position->text_offset());

  position = position->CreateNextParagraphStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, position);
  EXPECT_EQ(inline_box_8.id, position->anchor_id());
  EXPECT_EQ(0, position->text_offset());

  // Collapsed - Backward navigation.
  position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_8.id, 4, ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, position);

  position = position->CreatePreviousParagraphEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, position);
  EXPECT_EQ(popup_button_4.id, position->anchor_id());
  // The content of this popup button should be replaced with the empty object
  // character of length 1.
  EXPECT_EQ(1, position->text_offset());

  position = position->CreatePreviousParagraphEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, position);
  EXPECT_EQ(inline_box_3.id, position->anchor_id());
  EXPECT_EQ(2, position->text_offset());

  // Expand the combobox for the rest of the test.
  popup_button_4.RemoveState(ax::mojom::State::kCollapsed);
  popup_button_4.AddState(ax::mojom::State::kExpanded);
  AXTreeUpdate update;
  update.nodes = {popup_button_4};
  ASSERT_TRUE(GetTree()->Unserialize(update));

  // Expanded - Forward navigation.
  position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_3.id, 0, ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, position);

  position = position->CreateNextParagraphStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, position);
  EXPECT_EQ(menu_list_option_6.id, position->anchor_id());
  EXPECT_EQ(0, position->text_offset());

  position = position->CreateNextParagraphStartPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, position);
  EXPECT_EQ(inline_box_8.id, position->anchor_id());
  EXPECT_EQ(0, position->text_offset());

  // Expanded- Backward navigation.
  position = AXNodePosition::CreateTextPosition(
      GetTreeID(), inline_box_8.id, 4, ax::mojom::TextAffinity::kDownstream);
  ASSERT_NE(nullptr, position);

  position = position->CreatePreviousParagraphEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, position);
  EXPECT_EQ(menu_list_option_6.id, position->anchor_id());
  EXPECT_EQ(1, position->text_offset());

  position = position->CreatePreviousParagraphEndPosition(
      AXBoundaryBehavior::StopAtLastAnchorBoundary);
  ASSERT_NE(nullptr, position);
  EXPECT_EQ(inline_box_3.id, position->anchor_id());
  EXPECT_EQ(2, position->text_offset());
}

//
// Parameterized tests.
//

TEST_P(AXPositionExpandToEnclosingTextBoundaryTestWithParam,
       TextPositionBeforeLine2) {
  // Create a text position right before "Line 2". This should be at the start
  // of many text boundaries, e.g. line, paragraph and word.
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), text_field_.id, 7 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position->IsTextPosition());
  TestPositionRange range = text_position->ExpandToEnclosingTextBoundary(
      GetParam().boundary, GetParam().expand_behavior);
  EXPECT_EQ(GetParam().expected_anchor_position, range.anchor()->ToString());
  EXPECT_EQ(GetParam().expected_focus_position, range.focus()->ToString());
}

TEST_P(AXPositionCreatePositionAtTextBoundaryTestWithParam,
       TextPositionBeforeStaticText) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), static_text2_.id, 0 /* text_offset */,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position->IsTextPosition());
  text_position = text_position->CreatePositionAtTextBoundary(
      GetParam().boundary, GetParam().direction, GetParam().boundary_behavior);
  EXPECT_NE(nullptr, text_position);
  EXPECT_EQ(GetParam().expected_text_position, text_position->ToString());
}

TEST_P(AXPositionTextNavigationTestWithParam,
       TraverseTreeStartingWithAffinityDownstream) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetParam().start_node_id, GetParam().start_offset,
      ax::mojom::TextAffinity::kDownstream);
  ASSERT_TRUE(text_position->IsTextPosition());
  for (const std::string& expectation : GetParam().expectations) {
    text_position = GetParam().TestMethod(text_position);
    EXPECT_NE(nullptr, text_position);
    EXPECT_EQ(expectation, text_position->ToString());
  }
}

TEST_P(AXPositionTextNavigationTestWithParam,
       TraverseTreeStartingWithAffinityUpstream) {
  TestPositionType text_position = AXNodePosition::CreateTextPosition(
      GetTreeID(), GetParam().start_node_id, GetParam().start_offset,
      ax::mojom::TextAffinity::kUpstream);
  ASSERT_TRUE(text_position->IsTextPosition());
  for (const std::string& expectation : GetParam().expectations) {
    text_position = GetParam().TestMethod(text_position);
    EXPECT_NE(nullptr, text_position);
    EXPECT_EQ(expectation, text_position->ToString());
  }
}

//
// Instantiations of parameterized tests.
//

INSTANTIATE_TEST_SUITE_P(
    ExpandToEnclosingTextBoundary,
    AXPositionExpandToEnclosingTextBoundaryTestWithParam,
    testing::Values(
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kCharacter,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=6 affinity=downstream "
            "annotated_text=Line 1<\n>Line 2",
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kCharacter,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2",
            "TextPosition anchor_id=4 text_offset=8 affinity=downstream "
            "annotated_text=Line 1\nL<i>ne 2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kFormat, AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kFormat,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineEnd,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=6 affinity=downstream "
            "annotated_text=Line 1<\n>Line 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineEnd,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=6 affinity=downstream "
            "annotated_text=Line 1<\n>Line 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineStart,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineStart,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineStartOrEnd,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=4 text_offset=6 affinity=downstream "
            "annotated_text=Line 1<\n>Line 2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineStartOrEnd,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kObject, AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kObject,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphEnd,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=4 text_offset=7 affinity=upstream "
            "annotated_text=Line 1\n<L>ine 2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphEnd,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=7 affinity=upstream "
            "annotated_text=Line 1\n<L>ine 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphStart,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphStart,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphStartOrEnd,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=4 text_offset=7 affinity=upstream "
            "annotated_text=Line 1\n<L>ine 2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphStartOrEnd,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2",
            "TextPosition anchor_id=4 text_offset=13 affinity=downstream "
            "annotated_text=Line 1\nLine 2<>"},
        // TODO(accessibility): Add tests for sentence boundary.
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWebPage,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=1 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=9 text_offset=6 affinity=downstream "
            "annotated_text=Line 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWebPage,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=1 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2",
            "TextPosition anchor_id=9 text_offset=6 affinity=downstream "
            "annotated_text=Line 2<>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordEnd,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=6 affinity=downstream "
            "annotated_text=Line 1<\n>Line 2",
            "TextPosition anchor_id=4 text_offset=11 affinity=downstream "
            "annotated_text=Line 1\nLine< >2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordEnd,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=6 affinity=downstream "
            "annotated_text=Line 1<\n>Line 2",
            "TextPosition anchor_id=4 text_offset=11 affinity=downstream "
            "annotated_text=Line 1\nLine< >2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordStart,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=5 affinity=downstream "
            "annotated_text=Line <1>\nLine 2",
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordStart,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2",
            "TextPosition anchor_id=4 text_offset=12 affinity=downstream "
            "annotated_text=Line 1\nLine <2>"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordStartOrEnd,
            AXRangeExpandBehavior::kLeftFirst,
            "TextPosition anchor_id=4 text_offset=5 affinity=downstream "
            "annotated_text=Line <1>\nLine 2",
            "TextPosition anchor_id=4 text_offset=6 affinity=downstream "
            "annotated_text=Line 1<\n>Line 2"},
        ExpandToEnclosingTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordStartOrEnd,
            AXRangeExpandBehavior::kRightFirst,
            "TextPosition anchor_id=4 text_offset=7 affinity=downstream "
            "annotated_text=Line 1\n<L>ine 2",
            "TextPosition anchor_id=4 text_offset=11 affinity=downstream "
            "annotated_text=Line 1\nLine< >2"}));

// Only test with AXBoundaryBehavior::CrossBoundary for now.
// TODO(accessibility): Add more tests for other boundary behaviors if needed.
INSTANTIATE_TEST_SUITE_P(
    CreatePositionAtTextBoundary,
    AXPositionCreatePositionAtTextBoundaryTestWithParam,
    testing::Values(
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kCharacter,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=7 text_offset=0 affinity=downstream "
            "annotated_text=<\n>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kCharacter,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=1 affinity=downstream "
            "annotated_text=L<i>ne 2"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kFormat,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=7 text_offset=0 affinity=downstream "
            "annotated_text=<\n>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kFormat,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=6 affinity=downstream "
            "annotated_text=Line 2<>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineEnd,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=7 text_offset=0 affinity=downstream "
            "annotated_text=<\n>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineEnd,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=6 affinity=downstream "
            "annotated_text=Line 2<>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineStart,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=6 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineStart,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary, "NullPosition"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineStartOrEnd,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=6 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kLineStartOrEnd,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=6 affinity=downstream "
            "annotated_text=Line 2<>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kObject,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 2"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kObject,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=6 affinity=downstream "
            "annotated_text=Line 2<>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphEnd,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=3 text_offset=0 affinity=downstream "
            "annotated_text=<>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphEnd,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=6 affinity=downstream "
            "annotated_text=Line 2<>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphStart,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=6 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphStart,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary, "NullPosition"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphStartOrEnd,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=6 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kParagraphStartOrEnd,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=6 affinity=downstream "
            "annotated_text=Line 2<>"},
        // TODO(accessibility): Add tests for sentence boundary.
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWebPage,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=1 text_offset=0 affinity=downstream "
            "annotated_text=<L>ine 1\nLine 2"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWebPage,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=9 text_offset=6 affinity=downstream "
            "annotated_text=Line 2<>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordEnd,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=6 text_offset=6 affinity=downstream "
            "annotated_text=Line 1<>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordEnd,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=4 affinity=downstream "
            "annotated_text=Line< >2"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordStart,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=6 text_offset=5 affinity=downstream "
            "annotated_text=Line <1>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordStart,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=5 affinity=downstream "
            "annotated_text=Line <2>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordStartOrEnd,
            ax::mojom::MoveDirection::kBackward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=6 text_offset=5 affinity=downstream "
            "annotated_text=Line <1>"},
        CreatePositionAtTextBoundaryTestParam{
            ax::mojom::TextBoundary::kWordStartOrEnd,
            ax::mojom::MoveDirection::kForward,
            AXBoundaryBehavior::CrossBoundary,
            "TextPosition anchor_id=8 text_offset=4 affinity=downstream "
            "annotated_text=Line< >2"}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextWordStartPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "NullPosition"}},
        TextNavigationTestParam{[](const TestPositionType& position) {
                                  return position->CreateNextWordStartPosition(
                                      AXBoundaryBehavior::CrossBoundary);
                                },
                                STATIC_TEXT1_ID,
                                1 /* text_offset */,
                                {"TextPosition anchor_id=5 text_offset=5 "
                                 "affinity=downstream annotated_text=Line <1>",
                                 "TextPosition anchor_id=9 text_offset=0 "
                                 "affinity=downstream annotated_text=<L>ine 2",
                                 "TextPosition anchor_id=9 text_offset=5 "
                                 "affinity=downstream annotated_text=Line <2>",
                                 "NullPosition"}},
        TextNavigationTestParam{[](const TestPositionType& position) {
                                  return position->CreateNextWordStartPosition(
                                      AXBoundaryBehavior::CrossBoundary);
                                },
                                INLINE_BOX2_ID,
                                4 /* text_offset */,
                                {"TextPosition anchor_id=9 text_offset=5 "
                                 "affinity=downstream annotated_text=Line <2>",
                                 "NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextWordStartPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>",
             "TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=5 "
             "affinity=downstream annotated_text=Line <2>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextWordStartPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>",
             "TextPosition anchor_id=5 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=5 "
             "affinity=downstream annotated_text=Line <2>",
             "TextPosition anchor_id=9 text_offset=5 "
             "affinity=downstream annotated_text=Line <2>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextWordStartPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=5 "
             "affinity=downstream annotated_text=Line <2>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=5 "
             "affinity=downstream annotated_text=Line <2>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousWordStartPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=6 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>",
             "TextPosition anchor_id=6 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousWordStartPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousWordStartPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=1 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=4 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousWordStartPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=12 "
             "affinity=downstream annotated_text=Line 1\nLine <2>",
             "TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=6 text_offset=5 "
             "affinity=downstream annotated_text=Line <1>",
             "TextPosition anchor_id=6 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=6 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextWordEndPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "NullPosition"}},
        TextNavigationTestParam{[](const TestPositionType& position) {
                                  return position->CreateNextWordEndPosition(
                                      AXBoundaryBehavior::CrossBoundary);
                                },
                                STATIC_TEXT1_ID,
                                1 /* text_offset */,
                                {"TextPosition anchor_id=5 text_offset=4 "
                                 "affinity=downstream annotated_text=Line< >1",
                                 "TextPosition anchor_id=5 text_offset=6 "
                                 "affinity=downstream annotated_text=Line 1<>",
                                 "TextPosition anchor_id=9 text_offset=4 "
                                 "affinity=downstream annotated_text=Line< >2",
                                 "TextPosition anchor_id=9 text_offset=6 "
                                 "affinity=downstream annotated_text=Line 2<>",
                                 "NullPosition"}},
        TextNavigationTestParam{[](const TestPositionType& position) {
                                  return position->CreateNextWordEndPosition(
                                      AXBoundaryBehavior::CrossBoundary);
                                },
                                INLINE_BOX2_ID,
                                4 /* text_offset */,
                                {"TextPosition anchor_id=9 text_offset=6 "
                                 "affinity=downstream annotated_text=Line 2<>",
                                 "NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextWordEndPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1",
             "TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextWordEndPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=1 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=4 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1",
             "TextPosition anchor_id=5 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=4 "
             "affinity=downstream annotated_text=Line< >2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextWordEndPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1",
             "TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=9 text_offset=4 "
             "affinity=downstream annotated_text=Line< >2",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextWordEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousWordEndPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=6 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=6 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1",
             "NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousWordEndPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {
                "TextPosition anchor_id=1 text_offset=11 "
                "affinity=downstream annotated_text=Line 1\nLine< >2",
                "TextPosition anchor_id=1 text_offset=6 "
                "affinity=downstream annotated_text=Line 1<\n>Line 2",
                "TextPosition anchor_id=1 text_offset=4 "
                "affinity=downstream annotated_text=Line< >1\nLine 2",
                "TextPosition anchor_id=1 text_offset=0 "
                "affinity=downstream annotated_text=<L>ine 1\nLine 2",
            }},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1",
             "TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousWordEndPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1",
             "TextPosition anchor_id=5 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=4 "
             "affinity=downstream annotated_text=Line< >2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousWordEndPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=11 "
             "affinity=downstream annotated_text=Line 1\nLine< >2",
             "TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1\nLine 2",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousWordEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=6 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=6 text_offset=4 "
             "affinity=downstream annotated_text=Line< >1",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextLineStartPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "NullPosition"}},
        TextNavigationTestParam{[](const TestPositionType& position) {
                                  return position->CreateNextLineStartPosition(
                                      AXBoundaryBehavior::CrossBoundary);
                                },
                                STATIC_TEXT1_ID,
                                1 /* text_offset */,
                                {"TextPosition anchor_id=9 text_offset=0 "
                                 "affinity=downstream annotated_text=<L>ine 2",
                                 "NullPosition"}},
        TextNavigationTestParam{[](const TestPositionType& position) {
                                  return position->CreateNextLineStartPosition(
                                      AXBoundaryBehavior::CrossBoundary);
                                },
                                INLINE_BOX2_ID,
                                4 /* text_offset */,
                                {"NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextLineStartPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextLineStartPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextLineStartPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousLineStartPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            13 /* text_offset at the end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=6 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousLineStartPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at the end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousLineStartPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            13 /* text_offset at the end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousLineStartPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at the end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=6 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=6 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextLineEndPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "NullPosition"}},
        TextNavigationTestParam{[](const TestPositionType& position) {
                                  return position->CreateNextLineEndPosition(
                                      AXBoundaryBehavior::CrossBoundary);
                                },
                                STATIC_TEXT1_ID,
                                1 /* text_offset */,
                                {"TextPosition anchor_id=5 text_offset=6 "
                                 "affinity=downstream annotated_text=Line 1<>",
                                 "TextPosition anchor_id=9 text_offset=6 "
                                 "affinity=downstream annotated_text=Line 2<>",
                                 "NullPosition"}},
        TextNavigationTestParam{[](const TestPositionType& position) {
                                  return position->CreateNextLineEndPosition(
                                      AXBoundaryBehavior::CrossBoundary);
                                },
                                INLINE_BOX2_ID,
                                4 /* text_offset */,
                                {"TextPosition anchor_id=9 text_offset=6 "
                                 "affinity=downstream annotated_text=Line 2<>",
                                 "NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextLineEndPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextLineEndPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextLineEndPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextLineEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousLineEndPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=6 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            INLINE_BOX2_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=0 "
             "affinity=downstream annotated_text=<\n>",
             "NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousLineEndPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousLineEndPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            12 /* text_offset one before the end of root. */,
            {"TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            12 /* text_offset one before the end of text field */,
            {"TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX1_ID,
            2 /* text_offset */,
            {"NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=6 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=6 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=6 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=6 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousLineEndPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<\n>Line 2",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=6 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousLineEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=0 "
             "affinity=downstream annotated_text=<\n>",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=2 text_offset=0 "
             "affinity=downstream annotated_text=<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextParagraphStartPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextParagraphStartPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextParagraphStartPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextParagraphStartPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousParagraphStartPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            13 /* text_offset at the end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=6 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousParagraphStartPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at the end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousParagraphStartPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            13 /* text_offset at the end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousParagraphStartPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at the end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=downstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            5 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=5 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphStartPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=6 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1",
             "TextPosition anchor_id=6 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextParagraphEndPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextParagraphEndPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>",
             "TextPosition anchor_id=5 text_offset=6 "
             "affinity=downstream annotated_text=Line 1<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextParagraphEndPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>",
             "TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            LINE_BREAK_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>",
             "TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            LINE_BREAK_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>",
             "TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreateNextParagraphEndPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=1 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>",
             "TextPosition anchor_id=4 text_offset=13 "
             "affinity=downstream annotated_text=Line 1\nLine 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            STATIC_TEXT1_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreateNextParagraphEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>",
             "TextPosition anchor_id=9 text_offset=6 "
             "affinity=downstream annotated_text=Line 2<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousParagraphEndPositionWithBoundaryBehaviorCrossBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            ROOT_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            TEXT_FIELD_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>",
             "TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "NullPosition"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::CrossBoundary);
            },
            INLINE_BOX2_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "NullPosition"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousParagraphEndPositionWithBoundaryBehaviorStopAtAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            ROOT_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            TEXT_FIELD_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=4 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtAnchorBoundary);
            },
            INLINE_BOX2_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2",
             "TextPosition anchor_id=9 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 2"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousParagraphEndPositionWithBoundaryBehaviorStopIfAlreadyAtBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            ROOT_ID,
            12 /* text_offset one before the end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            TEXT_FIELD_ID,
            12 /* text_offset one before the end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=4 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX1_ID,
            2 /* text_offset */,
            {"TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>",
             "TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            INLINE_BOX2_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>",
             "TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            LINE_BREAK_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopIfAlreadyAtBoundary);
            },
            LINE_BREAK_ID,
            1 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>",
             "TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>"}}));

INSTANTIATE_TEST_SUITE_P(
    CreatePreviousParagraphEndPositionWithBoundaryBehaviorStopAtLastAnchorBoundary,
    AXPositionTextNavigationTestWithParam,
    testing::Values(
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            13 /* text_offset at end of root. */,
            {"TextPosition anchor_id=1 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            13 /* text_offset at end of text field */,
            {"TextPosition anchor_id=4 text_offset=7 "
             "affinity=upstream annotated_text=Line 1\n<L>ine 2",
             "TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            ROOT_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2",
             "TextPosition anchor_id=1 text_offset=0 "
             "affinity=downstream annotated_text=<L>ine 1\nLine 2"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            TEXT_FIELD_ID,
            5 /* text_offset on the last character of "Line 1". */,
            {"TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            4 /* text_offset */,
            {"TextPosition anchor_id=7 text_offset=1 "
             "affinity=downstream annotated_text=\n<>",
             "TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>"}},
        TextNavigationTestParam{
            [](const TestPositionType& position) {
              return position->CreatePreviousParagraphEndPosition(
                  AXBoundaryBehavior::StopAtLastAnchorBoundary);
            },
            INLINE_BOX2_ID,
            0 /* text_offset */,
            {"TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>",
             "TextPosition anchor_id=3 text_offset=0 "
             "affinity=downstream annotated_text=<>"}}));

}  // namespace ui

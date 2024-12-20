// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/text_input_model.h"

#include <limits>
#include <map>
#include <vector>

#include "gtest/gtest.h"

namespace flutter {

TEST(TextInputModel, SetText) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetTextWideCharacters) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("😄🙃🤪🧐");
  EXPECT_STREQ(model->GetText().c_str(), "😄🙃🤪🧐");
}

TEST(TextInputModel, SetTextEmpty) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("");
  EXPECT_STREQ(model->GetText().c_str(), "");
}

TEST(TextInputModel, SetTextReplaceText) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
  model->SetText("");
  EXPECT_STREQ(model->GetText().c_str(), "");
}

TEST(TextInputModel, SetTextResetsSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(3)));
  EXPECT_EQ(model->selection(), TextRange(3));
  model->SetText("FGHJI");
  EXPECT_EQ(model->selection(), TextRange(0));
}

TEST(TextInputModel, SetSelectionStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(0)));
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetSelectionComposingStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->SetSelection(TextRange(1)));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetSelectionMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetSelectionComposingMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetSelectionEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(5)));
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetSelectionComposingEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->SetSelection(TextRange(4)));
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetSelectionWthExtent) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  EXPECT_EQ(model->selection(), TextRange(1, 4));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetSelectionWthExtentComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_FALSE(model->SetSelection(TextRange(1, 4)));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetSelectionReverseExtent) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  EXPECT_EQ(model->selection(), TextRange(4, 1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetSelectionReverseExtentComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_FALSE(model->SetSelection(TextRange(4, 1)));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetSelectionOutsideString) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_FALSE(model->SetSelection(TextRange(4, 6)));
  EXPECT_FALSE(model->SetSelection(TextRange(5, 6)));
  EXPECT_FALSE(model->SetSelection(TextRange(6)));
}

TEST(TextInputModel, SetSelectionOutsideComposingRange) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_FALSE(model->SetSelection(TextRange(0)));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_FALSE(model->SetSelection(TextRange(5)));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
}

TEST(TextInputModel, SetComposingRangeStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(0, 0), 0));
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetComposingRangeMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(2, 2), 0));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(2));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetComposingRangeEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(5, 5), 0));
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(5));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetComposingRangeWithExtent) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 3));
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetComposingRangeReverseExtent) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 3));
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SetComposingRangeOutsideString) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_FALSE(model->SetComposingRange(TextRange(4, 6), 0));
  EXPECT_FALSE(model->SetComposingRange(TextRange(5, 6), 0));
  EXPECT_FALSE(model->SetComposingRange(TextRange(6, 6), 0));
}

// Composing sequence with no initial selection and no text input.
TEST(TextInputModel, CommitComposingNoTextWithNoSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->SetSelection(TextRange(0));

  // Verify no changes on BeginComposing.
  model->BeginComposing();
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");

  // Verify no changes on CommitComposing.
  model->CommitComposing();
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");

  // Verify no changes on CommitComposing.
  model->EndComposing();
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

// Composing sequence with an initial selection and no text input.
TEST(TextInputModel, CommitComposingNoTextWithSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->SetSelection(TextRange(1, 3));

  // Verify no changes on BeginComposing.
  model->BeginComposing();
  EXPECT_EQ(model->selection(), TextRange(1, 3));
  EXPECT_EQ(model->composing_range(), TextRange(1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");

  // Verify no changes on CommitComposing.
  model->CommitComposing();
  EXPECT_EQ(model->selection(), TextRange(1, 3));
  EXPECT_EQ(model->composing_range(), TextRange(1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");

  // Verify no changes on CommitComposing.
  model->EndComposing();
  EXPECT_EQ(model->selection(), TextRange(1, 3));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

// Composing sequence with no initial selection.
TEST(TextInputModel, CommitComposingTextWithNoSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->SetSelection(TextRange(1));

  // Verify no changes on BeginComposing.
  model->BeginComposing();
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");

  // Verify selection base, extent and composing extent increment as text is
  // entered. Verify composing base does not change.
  model->UpdateComposingText("つ");
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(1, 2));
  EXPECT_STREQ(model->GetText().c_str(), "AつBCDE");
  model->UpdateComposingText("つる");
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(1, 3));
  EXPECT_STREQ(model->GetText().c_str(), "AつるBCDE");

  // Verify that cursor position is set to correct offset from composing base.
  model->UpdateComposingText("鶴");
  EXPECT_TRUE(model->SetSelection(TextRange(1)));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 2));
  EXPECT_STREQ(model->GetText().c_str(), "A鶴BCDE");

  // Verify composing base is set to composing extent on commit.
  model->CommitComposing();
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(2));
  EXPECT_STREQ(model->GetText().c_str(), "A鶴BCDE");

  // Verify that further text entry increments the selection base, extent and
  // the composing extent. Verify that composing base does not change.
  model->UpdateComposingText("が");
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(2, 3));
  EXPECT_STREQ(model->GetText().c_str(), "A鶴がBCDE");

  // Verify composing base is set to composing extent on commit.
  model->CommitComposing();
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(3));
  EXPECT_STREQ(model->GetText().c_str(), "A鶴がBCDE");

  // Verify no changes on EndComposing.
  model->EndComposing();
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "A鶴がBCDE");
}

// Composing sequence with an initial selection.
TEST(TextInputModel, CommitComposingTextWithSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->SetSelection(TextRange(1, 3));

  // Verify no changes on BeginComposing.
  model->BeginComposing();
  EXPECT_EQ(model->selection(), TextRange(1, 3));
  EXPECT_EQ(model->composing_range(), TextRange(1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");

  // Verify selection is replaced and selection base, extent and composing
  // extent increment to the position immediately after the composing text.
  // Verify composing base does not change.
  model->UpdateComposingText("つ");
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(1, 2));
  EXPECT_STREQ(model->GetText().c_str(), "AつDE");

  // Verify that further text entry increments the selection base, extent and
  // the composing extent. Verify that composing base does not change.
  model->UpdateComposingText("つる");
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(1, 3));
  EXPECT_STREQ(model->GetText().c_str(), "AつるDE");

  // Verify that cursor position is set to correct offset from composing base.
  model->UpdateComposingText("鶴");
  EXPECT_TRUE(model->SetSelection(TextRange(1)));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 2));
  EXPECT_STREQ(model->GetText().c_str(), "A鶴DE");

  // Verify composing base is set to composing extent on commit.
  model->CommitComposing();
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(2));
  EXPECT_STREQ(model->GetText().c_str(), "A鶴DE");

  // Verify that further text entry increments the selection base, extent and
  // the composing extent. Verify that composing base does not change.
  model->UpdateComposingText("が");
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(2, 3));
  EXPECT_STREQ(model->GetText().c_str(), "A鶴がDE");

  // Verify composing base is set to composing extent on commit.
  model->CommitComposing();
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(3));
  EXPECT_STREQ(model->GetText().c_str(), "A鶴がDE");

  // Verify no changes on EndComposing.
  model->EndComposing();
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "A鶴がDE");
}

TEST(TextInputModel, UpdateComposingRemovesLastComposingCharacter) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  model->SetComposingRange(TextRange(1, 2), 1);
  model->UpdateComposingText("");
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1));
  model->SetText("ACDE");
}

TEST(TextInputModel, UpdateSelectionWhileComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  model->SetComposingRange(TextRange(4, 5), 1);
  model->UpdateComposingText(u"ぴょんぴょん", TextRange(3, 6));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDぴょんぴょん");
  EXPECT_EQ(model->selection(), TextRange(7, 10));
  EXPECT_EQ(model->composing_range(), TextRange(4, 10));
}

TEST(TextInputModel, AddCodePoint) {
  auto model = std::make_unique<TextInputModel>();
  model->AddCodePoint('A');
  model->AddCodePoint('B');
  model->AddCodePoint(0x1f604);
  model->AddCodePoint('D');
  model->AddCodePoint('E');
  EXPECT_EQ(model->selection(), TextRange(6));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AB😄DE");
}

TEST(TextInputModel, AddCodePointSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  model->AddCodePoint('x');
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AxE");
}

TEST(TextInputModel, AddCodePointReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  model->AddCodePoint('x');
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AxE");
}

TEST(TextInputModel, AddCodePointSelectionWideCharacter) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  model->AddCodePoint(0x1f604);
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "A😄E");
}

TEST(TextInputModel, AddCodePointReverseSelectionWideCharacter) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  model->AddCodePoint(0x1f604);
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "A😄E");
}

TEST(TextInputModel, AddText) {
  auto model = std::make_unique<TextInputModel>();
  model->AddText(u"ABCDE");
  model->AddText("😄");
  model->AddText("FGHIJ");
  EXPECT_EQ(model->selection(), TextRange(12));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE😄FGHIJ");
}

TEST(TextInputModel, AddTextSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  model->AddText("xy");
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AxyE");
}

TEST(TextInputModel, AddTextReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  model->AddText("xy");
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AxyE");
}

TEST(TextInputModel, AddTextSelectionWideCharacter) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  model->AddText(u"😄🙃");
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "A😄🙃E");
}

TEST(TextInputModel, AddTextReverseSelectionWideCharacter) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  model->AddText(u"😄🙃");
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "A😄🙃E");
}

TEST(TextInputModel, DeleteStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(0)));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "BCDE");
}

TEST(TextInputModel, DeleteMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABDE");
}

TEST(TextInputModel, DeleteEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(5)));
  ASSERT_FALSE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, DeleteWideCharacters) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("😄🙃🤪🧐");
  EXPECT_TRUE(model->SetSelection(TextRange(4)));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "😄🙃🧐");
}

TEST(TextInputModel, DeleteSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AE");
}

TEST(TextInputModel, DeleteReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AE");
}

TEST(TextInputModel, DeleteStartComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 3));
  EXPECT_STREQ(model->GetText().c_str(), "ACDE");
}

TEST(TextInputModel, DeleteStartReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 0));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(3, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ACDE");
}

TEST(TextInputModel, DeleteMiddleComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(1, 3));
  EXPECT_STREQ(model->GetText().c_str(), "ABDE");
}

TEST(TextInputModel, DeleteMiddleReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 1));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(3, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABDE");
}

TEST(TextInputModel, DeleteEndComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 3));
  ASSERT_FALSE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, DeleteEndReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 3));
  ASSERT_FALSE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, DeleteSurroundingAtCursor) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->DeleteSurrounding(0, 1));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABDE");
}

TEST(TextInputModel, DeleteSurroundingAtCursorComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  EXPECT_TRUE(model->DeleteSurrounding(0, 1));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(1, 3));
  EXPECT_STREQ(model->GetText().c_str(), "ABDE");
}

TEST(TextInputModel, DeleteSurroundingAtCursorAll) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->DeleteSurrounding(0, 3));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AB");
}

TEST(TextInputModel, DeleteSurroundingAtCursorAllComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  EXPECT_TRUE(model->DeleteSurrounding(0, 2));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(1, 2));
  EXPECT_STREQ(model->GetText().c_str(), "ABE");
}

TEST(TextInputModel, DeleteSurroundingAtCursorGreedy) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->DeleteSurrounding(0, 4));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AB");
}

TEST(TextInputModel, DeleteSurroundingAtCursorGreedyComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  EXPECT_TRUE(model->DeleteSurrounding(0, 4));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(1, 2));
  EXPECT_STREQ(model->GetText().c_str(), "ABE");
}

TEST(TextInputModel, DeleteSurroundingBeforeCursor) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->DeleteSurrounding(-1, 1));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ACDE");
}

TEST(TextInputModel, DeleteSurroundingBeforeCursorComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 2));
  EXPECT_TRUE(model->DeleteSurrounding(-1, 1));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(1, 3));
  EXPECT_STREQ(model->GetText().c_str(), "ABDE");
}

TEST(TextInputModel, DeleteSurroundingBeforeCursorAll) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->DeleteSurrounding(-2, 2));
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "CDE");
}

TEST(TextInputModel, DeleteSurroundingBeforeCursorAllComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 2));
  EXPECT_TRUE(model->DeleteSurrounding(-2, 2));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 2));
  EXPECT_STREQ(model->GetText().c_str(), "ADE");
}

TEST(TextInputModel, DeleteSurroundingBeforeCursorGreedy) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->DeleteSurrounding(-3, 3));
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "CDE");
}

TEST(TextInputModel, DeleteSurroundingBeforeCursorGreedyComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 2));
  EXPECT_TRUE(model->DeleteSurrounding(-3, 3));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 2));
  EXPECT_STREQ(model->GetText().c_str(), "ADE");
}

TEST(TextInputModel, DeleteSurroundingAfterCursor) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->DeleteSurrounding(1, 1));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCE");
}

TEST(TextInputModel, DeleteSurroundingAfterCursorComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->DeleteSurrounding(1, 1));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 3));
  EXPECT_STREQ(model->GetText().c_str(), "ABDE");
}

TEST(TextInputModel, DeleteSurroundingAfterCursorAll) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->DeleteSurrounding(1, 2));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABC");
}

TEST(TextInputModel, DeleteSurroundingAfterCursorAllComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->DeleteSurrounding(1, 2));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 2));
  EXPECT_STREQ(model->GetText().c_str(), "ABE");
}

TEST(TextInputModel, DeleteSurroundingAfterCursorGreedy) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->DeleteSurrounding(1, 3));
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABC");
}

TEST(TextInputModel, DeleteSurroundingAfterCursorGreedyComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->DeleteSurrounding(1, 3));
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 2));
  EXPECT_STREQ(model->GetText().c_str(), "ABE");
}

TEST(TextInputModel, DeleteSurroundingSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2, 3)));
  EXPECT_TRUE(model->DeleteSurrounding(0, 1));
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCE");
}

TEST(TextInputModel, DeleteSurroundingReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 3)));
  EXPECT_TRUE(model->DeleteSurrounding(0, 1));
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCE");
}

TEST(TextInputModel, BackspaceStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(0)));
  ASSERT_FALSE(model->Backspace());
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, BackspaceMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  ASSERT_TRUE(model->Backspace());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ACDE");
}

TEST(TextInputModel, BackspaceEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(5)));
  ASSERT_TRUE(model->Backspace());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCD");
}

TEST(TextInputModel, BackspaceWideCharacters) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("😄🙃🤪🧐");
  EXPECT_TRUE(model->SetSelection(TextRange(4)));
  ASSERT_TRUE(model->Backspace());
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "😄🤪🧐");
}

TEST(TextInputModel, BackspaceSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AE");
}

TEST(TextInputModel, BackspaceReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  ASSERT_TRUE(model->Delete());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "AE");
}

TEST(TextInputModel, BackspaceStartComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  ASSERT_FALSE(model->Backspace());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, BackspaceStartReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 0));
  ASSERT_FALSE(model->Backspace());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, BackspaceMiddleComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  ASSERT_TRUE(model->Backspace());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 3));
  EXPECT_STREQ(model->GetText().c_str(), "ACDE");
}

TEST(TextInputModel, BackspaceMiddleReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 1));
  ASSERT_TRUE(model->Backspace());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(3, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ACDE");
}

TEST(TextInputModel, BackspaceEndComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 3));
  ASSERT_TRUE(model->Backspace());
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(1, 3));
  EXPECT_STREQ(model->GetText().c_str(), "ABCE");
}

TEST(TextInputModel, BackspaceEndReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 3));
  ASSERT_TRUE(model->Backspace());
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(3, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCE");
}

TEST(TextInputModel, MoveCursorForwardStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(0)));
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorForwardMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorForwardEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(5)));
  EXPECT_FALSE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorForwardWideCharacters) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("😄🙃🤪🧐");
  EXPECT_TRUE(model->SetSelection(TextRange(4)));
  ASSERT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(6));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "😄🙃🤪🧐");
}

TEST(TextInputModel, MoveCursorForwardSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorForwardReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorForwardStartComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorForwardStartReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 0));
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorForwardMiddleComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorForwardMiddleReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 1));
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorForwardEndComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 3));
  EXPECT_FALSE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorForwardEndReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 3));
  EXPECT_FALSE(model->MoveCursorForward());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(0)));
  EXPECT_FALSE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(5)));
  EXPECT_TRUE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackWideCharacters) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("😄🙃🤪🧐");
  EXPECT_TRUE(model->SetSelection(TextRange(4)));
  ASSERT_TRUE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(2));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "😄🙃🤪🧐");
}

TEST(TextInputModel, MoveCursorBackSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  EXPECT_TRUE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  EXPECT_TRUE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackStartComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->SetSelection(TextRange(1)));
  EXPECT_FALSE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackStartReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 0));
  EXPECT_TRUE(model->SetSelection(TextRange(1)));
  EXPECT_FALSE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackMiddleComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  EXPECT_TRUE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackMiddleReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 1));
  EXPECT_TRUE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackEndComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 3));
  EXPECT_TRUE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorBackEndReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 3));
  EXPECT_TRUE(model->MoveCursorBack());
  EXPECT_EQ(model->selection(), TextRange(3));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(0)));
  EXPECT_FALSE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(0)));
  EXPECT_FALSE(model->SelectToBeginning());
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->SelectToBeginning());
  EXPECT_EQ(model->selection(), TextRange(2, 0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(5)));
  EXPECT_TRUE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(5)));
  EXPECT_TRUE(model->SelectToBeginning());
  EXPECT_EQ(model->selection(), TextRange(5, 0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  EXPECT_TRUE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  EXPECT_TRUE(model->SelectToBeginning());
  EXPECT_EQ(model->selection(), TextRange(1, 0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  EXPECT_TRUE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  EXPECT_TRUE(model->SelectToBeginning());
  EXPECT_EQ(model->selection(), TextRange(4, 0));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningStartComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_FALSE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningStartComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_FALSE(model->SelectToBeginning());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningStartReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 0));
  EXPECT_FALSE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningStartReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 0));
  EXPECT_FALSE(model->SelectToBeginning());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningMiddleComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  EXPECT_TRUE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningMiddleComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  EXPECT_TRUE(model->SelectToBeginning());
  EXPECT_EQ(model->selection(), TextRange(2, 1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningMiddleReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 1));
  EXPECT_TRUE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningMiddleReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 1));
  EXPECT_TRUE(model->SelectToBeginning());
  EXPECT_EQ(model->selection(), TextRange(2, 1));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningEndComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 3));
  EXPECT_TRUE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningEndComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 3));
  EXPECT_TRUE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToBeginningEndReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 3));
  EXPECT_TRUE(model->MoveCursorToBeginning());
  EXPECT_EQ(model->selection(), TextRange(1));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToBeginningEndReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 3));
  EXPECT_TRUE(model->SelectToBeginning());
  EXPECT_EQ(model->selection(), TextRange(4, 1));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(0)));
  EXPECT_TRUE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndStart) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(0)));
  EXPECT_TRUE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(0, 5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndMiddle) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(2)));
  EXPECT_TRUE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(2, 5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(5)));
  EXPECT_FALSE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndEnd) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(5)));
  EXPECT_FALSE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  EXPECT_TRUE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  EXPECT_TRUE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(1, 5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  EXPECT_TRUE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  EXPECT_TRUE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(4, 5));
  EXPECT_EQ(model->composing_range(), TextRange(0));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndStartComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndStartComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(1, 4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndStartReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndStartReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 0));
  EXPECT_TRUE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(1, 4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndMiddleComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  EXPECT_TRUE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndMiddleComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 1));
  EXPECT_TRUE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(2, 4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndMiddleReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 1));
  EXPECT_TRUE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndMiddleReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 1));
  EXPECT_TRUE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(2, 4));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndEndComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 3));
  EXPECT_FALSE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndEndComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(1, 4), 3));
  EXPECT_FALSE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(1, 4));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, MoveCursorToEndEndReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 3));
  EXPECT_FALSE(model->MoveCursorToEnd());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, SelectToEndEndReverseComposing) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  model->BeginComposing();
  EXPECT_TRUE(model->SetComposingRange(TextRange(4, 1), 3));
  EXPECT_FALSE(model->SelectToEnd());
  EXPECT_EQ(model->selection(), TextRange(4));
  EXPECT_EQ(model->composing_range(), TextRange(4, 1));
  EXPECT_STREQ(model->GetText().c_str(), "ABCDE");
}

TEST(TextInputModel, GetCursorOffset) {
  auto model = std::make_unique<TextInputModel>();
  // These characters take 1, 2, 3 and 4 bytes in UTF-8.
  model->SetText("$¢€𐍈");
  EXPECT_TRUE(model->SetSelection(TextRange(0)));
  EXPECT_EQ(model->GetCursorOffset(), 0);
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->GetCursorOffset(), 1);
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->GetCursorOffset(), 3);
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->GetCursorOffset(), 6);
  EXPECT_TRUE(model->MoveCursorForward());
  EXPECT_EQ(model->GetCursorOffset(), 10);
}

TEST(TextInputModel, GetCursorOffsetSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(1, 4)));
  EXPECT_EQ(model->GetCursorOffset(), 4);
}

TEST(TextInputModel, GetCursorOffsetReverseSelection) {
  auto model = std::make_unique<TextInputModel>();
  model->SetText("ABCDE");
  EXPECT_TRUE(model->SetSelection(TextRange(4, 1)));
  EXPECT_EQ(model->GetCursorOffset(), 1);
}

}  // namespace flutter

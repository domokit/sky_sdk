// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/text_editing_delta.h"

#include "gtest/gtest.h"

namespace flutter {

TEST(TextEditingDeltaTest, TestTextEditingDeltaConstructor) {
  // Here we are simulating inserting an "o" at the end of "hell".
  std::string old_text = "hell";
  std::string replacement_text = "hello";
  TextRange range(0, 4);

  TextEditingDelta delta = TextEditingDelta(old_text, range, replacement_text);

  EXPECT_EQ(delta.old_text(), old_text);
  EXPECT_EQ(delta.delta_text(), "hello");
  EXPECT_EQ(delta.delta_start(), 0);
  EXPECT_EQ(delta.delta_end(), 4);
}

TEST(TextEditingDeltaTest, TestTextEditingDeltaNonTextConstructor) {
  // Here we are simulating inserting an "o" at the end of "hell".
  std::string old_text = "hello";

  TextEditingDelta delta = TextEditingDelta(old_text);

  EXPECT_EQ(delta.old_text(), old_text);
  EXPECT_EQ(delta.delta_text(), "");
  EXPECT_EQ(delta.delta_start(), -1);
  EXPECT_EQ(delta.delta_end(), -1);
}

}  // namespace flutter

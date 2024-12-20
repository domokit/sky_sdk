// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/ascii_trie.h"

#include "gtest/gtest.h"

using fml::AsciiTrie;

TEST(AsciiTableTest, Simple) {
  AsciiTrie trie;
  auto entries = std::vector<std::string>{"foo"};
  trie.Fill(entries);
  ASSERT_TRUE(trie.Query("foobar"));
  ASSERT_FALSE(trie.Query("google"));
}

TEST(AsciiTableTest, ExactMatch) {
  AsciiTrie trie;
  auto entries = std::vector<std::string>{"foo"};
  trie.Fill(entries);
  ASSERT_TRUE(trie.Query("foo"));
}

TEST(AsciiTableTest, Empty) {
  AsciiTrie trie;
  ASSERT_TRUE(trie.Query("foo"));
}

TEST(AsciiTableTest, MultipleEntries) {
  AsciiTrie trie;
  auto entries = std::vector<std::string>{"foo", "bar"};
  trie.Fill(entries);
  ASSERT_TRUE(trie.Query("foozzz"));
  ASSERT_TRUE(trie.Query("barzzz"));
}

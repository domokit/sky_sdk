// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/ascii_trie.h"

#include "flutter/fml/logging.h"

namespace fml {
typedef AsciiTrie::TrieNode TrieNode;
typedef AsciiTrie::TrieNodePtr TrieNodePtr;

namespace {
void Add(TrieNodePtr* trie, const char* entry) {
  int ch = entry[0];
  FML_DCHECK(ch < AsciiTrie::kMaxAsciiValue);
  if (ch != 0) {
    if (!*trie) {
      *trie = std::make_unique<TrieNode>();
    }
    Add(&(*trie)->children[ch], entry + 1);
  }
}

TrieNodePtr MakeTrie(const std::vector<std::string>& entries) {
  TrieNodePtr result;
  for (const std::string& entry : entries) {
    Add(&result, entry.c_str());
  }
  return result;
}
}  // namespace

void AsciiTrie::Fill(const std::vector<std::string>& entries) {
  node_ = MakeTrie(entries);
}

bool AsciiTrie::Query(TrieNode* trie, const char* query) {
  FML_DCHECK(trie);
  const char* char_position = query;
  TrieNode* trie_position = trie;
  TrieNode* child = nullptr;
  int ch;
  while ((ch = *char_position) && (child = trie_position->children[ch].get())) {
    char_position++;
    trie_position = child;
  }
  return !child && trie_position != trie;
}
}  // namespace fml

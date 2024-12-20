// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "simple_token.h"

#include <ostream>
#include <random>

namespace base {

constexpr size_t kRandomTokenLength = 10;

SimpleToken::SimpleToken(const std::string& token) : token_(token) {}

// static
SimpleToken SimpleToken::Create() {
  const char charset[] =
      "0123456789"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz";
  const size_t max_index = (sizeof(charset) - 1);

  std::string str;
  for (size_t i = 0; i < kRandomTokenLength; i++) {
    str.push_back(charset[rand() % max_index]);
  }
  return SimpleToken(str);
}

std::ostream& operator<<(std::ostream& out, const SimpleToken& token) {
  return out << "(" << token.ToString() << ")";
}

std::optional<base::SimpleToken> ValueToSimpleToken(std::string str) {
  return std::make_optional<base::SimpleToken>(str);
}

std::string SimpleTokenToValue(const SimpleToken& token) {
  return token.ToString();
}

size_t SimpleTokenHash(const SimpleToken& SimpleToken) {
  return std::hash<std::string>()(SimpleToken.ToString());
}

}  // namespace base

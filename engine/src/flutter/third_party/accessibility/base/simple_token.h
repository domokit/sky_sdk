// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_SIMPLE_TOKEN_H_
#define BASE_SIMPLE_TOKEN_H_

#include <functional>
#include <optional>
#include <string>

namespace base {

// A simple string wrapping token.
class SimpleToken {
 public:
  // Create a random SimpleToken.
  static SimpleToken Create();

  // Creates an empty SimpleToken.
  // Assign to it with Create() before using it.
  SimpleToken() = default;
  ~SimpleToken() = default;
  // Creates an token that wrapps the input string.
  SimpleToken(const std::string& token);

  // Hex representation of the unguessable token.
  std::string ToString() const { return token_; }

  explicit constexpr operator bool() const { return !token_.empty(); }

  constexpr bool operator<(const SimpleToken& other) const {
    return token_.compare(other.token_) < 0;
  }

  constexpr bool operator==(const SimpleToken& other) const {
    return token_.compare(other.token_) == 0;
  }

  constexpr bool operator!=(const SimpleToken& other) const {
    return !(*this == other);
  }

 private:
  std::string token_;
};

std::ostream& operator<<(std::ostream& out, const SimpleToken& token);

std::optional<base::SimpleToken> ValueToSimpleToken(std::string str);

std::string SimpleTokenToValue(const SimpleToken& token);

size_t SimpleTokenHash(const SimpleToken& SimpleToken);

}  // namespace base

#endif  // BASE_SIMPLE_TOKEN_H_

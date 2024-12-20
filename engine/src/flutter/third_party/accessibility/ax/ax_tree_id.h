// Copyright 2018 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_TREE_ID_H_
#define UI_ACCESSIBILITY_AX_TREE_ID_H_

#include <string>

#include "ax_base_export.h"
#include "ax_enums.h"
#include "base/no_destructor.h"
#include "base/simple_token.h"

namespace ui {

// A unique ID representing an accessibility tree.
class AX_BASE_EXPORT AXTreeID {
 public:
  // Create an Unknown AXTreeID.
  AXTreeID();

  // Copy constructor.
  AXTreeID(const AXTreeID& other);

  // Create a new unique AXTreeID.
  static AXTreeID CreateNewAXTreeID();

  // Unserialize an AXTreeID from a string. This is used so that tree IDs
  // can be stored compactly as a string attribute in an AXNodeData, and
  // so that AXTreeIDs can be passed to JavaScript bindings in the
  // automation API.
  static AXTreeID FromString(const std::string& string);

  // Convenience method to unserialize an AXTreeID from an SimpleToken.
  static AXTreeID FromToken(const base::SimpleToken& token);

  AXTreeID& operator=(const AXTreeID& other);

  std::string ToString() const;

  ax::mojom::AXTreeIDType type() const { return type_; }
  const std::optional<base::SimpleToken>& token() const { return token_; }

  bool operator==(const AXTreeID& rhs) const;
  bool operator!=(const AXTreeID& rhs) const;
  bool operator<(const AXTreeID& rhs) const;
  bool operator<=(const AXTreeID& rhs) const;
  bool operator>(const AXTreeID& rhs) const;
  bool operator>=(const AXTreeID& rhs) const;

 private:
  explicit AXTreeID(ax::mojom::AXTreeIDType type);
  explicit AXTreeID(const std::string& string);

  friend class base::NoDestructor<AXTreeID>;
  friend void swap(AXTreeID& first, AXTreeID& second);

  ax::mojom::AXTreeIDType type_;
  std::optional<base::SimpleToken> token_ = std::nullopt;
};

// For use in std::unordered_map.
struct AX_BASE_EXPORT AXTreeIDHash {
  size_t operator()(const ui::AXTreeID& tree_id) const;
};

AX_BASE_EXPORT std::ostream& operator<<(std::ostream& stream,
                                        const AXTreeID& value);

// The value to use when an AXTreeID is unknown.
AX_BASE_EXPORT extern const AXTreeID& AXTreeIDUnknown();

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_TREE_ID_H_

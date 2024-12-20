// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_EVENT_INTENT_H_
#define UI_ACCESSIBILITY_AX_EVENT_INTENT_H_

#include <string>

#include "ax_base_export.h"
#include "ax_enums.h"

namespace ui {

// Describes what caused an accessibility event to be raised. For example, a
// character could have been typed, a word replaced, or a line deleted. Or, the
// selection could have been extended to the beginning of the previous word, or
// it could have been moved to the end of the next line.
struct AX_BASE_EXPORT AXEventIntent final {
  AXEventIntent();
  AXEventIntent(ax::mojom::Command command,
                ax::mojom::TextBoundary text_boundary,
                ax::mojom::MoveDirection move_direction);
  virtual ~AXEventIntent();
  AXEventIntent(const AXEventIntent& intent);
  AXEventIntent& operator=(const AXEventIntent& intent);

  friend AX_BASE_EXPORT bool operator==(const AXEventIntent& a,
                                        const AXEventIntent& b);
  friend AX_BASE_EXPORT bool operator!=(const AXEventIntent& a,
                                        const AXEventIntent& b);

  ax::mojom::Command command = ax::mojom::Command::kType;
  // TODO(nektar): Split TextBoundary into TextUnit and TextBoundary.
  ax::mojom::TextBoundary text_boundary = ax::mojom::TextBoundary::kCharacter;
  ax::mojom::MoveDirection move_direction = ax::mojom::MoveDirection::kForward;

  // Returns a string representation of this data, for debugging.
  std::string ToString() const;
};

}  // namespace ui

#endif  // UI_ACCESSIBILITY_AX_EVENT_INTENT_H_

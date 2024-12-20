// Copyright 2020 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "ax_event_intent.h"

#include "ax_enum_util.h"

namespace ui {

AXEventIntent::AXEventIntent() = default;

AXEventIntent::AXEventIntent(ax::mojom::Command command,
                             ax::mojom::TextBoundary text_boundary,
                             ax::mojom::MoveDirection move_direction)
    : command(command),
      text_boundary(text_boundary),
      move_direction(move_direction) {}

AXEventIntent::~AXEventIntent() = default;

AXEventIntent::AXEventIntent(const AXEventIntent& intent) = default;

AXEventIntent& AXEventIntent::operator=(const AXEventIntent& intent) = default;

bool operator==(const AXEventIntent& a, const AXEventIntent& b) {
  return a.command == b.command && a.text_boundary == b.text_boundary &&
         a.move_direction == b.move_direction;
}

bool operator!=(const AXEventIntent& a, const AXEventIntent& b) {
  return !(a == b);
}

std::string AXEventIntent::ToString() const {
  return std::string("AXEventIntent(") + ui::ToString(command) + ',' +
         ui::ToString(text_boundary) + ',' + ui::ToString(move_direction) + ')';
}

}  // namespace ui

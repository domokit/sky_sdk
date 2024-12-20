// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/semantics/semantics_node.h"

#include <cstring>

namespace flutter {

constexpr int32_t kMinPlatformViewId = -1;

SemanticsNode::SemanticsNode() = default;

SemanticsNode::SemanticsNode(const SemanticsNode& other) = default;

SemanticsNode::~SemanticsNode() = default;

bool SemanticsNode::HasAction(SemanticsAction action) const {
  return (actions & static_cast<int32_t>(action)) != 0;
}

bool SemanticsNode::HasFlag(SemanticsFlags flag) const {
  return (flags & static_cast<int32_t>(flag)) != 0;
}

bool SemanticsNode::IsPlatformViewNode() const {
  return platformViewId > kMinPlatformViewId;
}

}  // namespace flutter

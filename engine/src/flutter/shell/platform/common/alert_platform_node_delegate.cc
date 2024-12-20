// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "alert_platform_node_delegate.h"

namespace flutter {

AlertPlatformNodeDelegate::AlertPlatformNodeDelegate(
    ui::AXPlatformNodeDelegate& parent_delegate)
    : parent_delegate_(parent_delegate) {
  data_.role = ax::mojom::Role::kAlert;
  data_.id = id_.Get();
}

AlertPlatformNodeDelegate::~AlertPlatformNodeDelegate() {}

gfx::AcceleratedWidget
AlertPlatformNodeDelegate::GetTargetForNativeAccessibilityEvent() {
  return parent_delegate_.GetTargetForNativeAccessibilityEvent();
}

gfx::NativeViewAccessible AlertPlatformNodeDelegate::GetParent() {
  return parent_delegate_.GetNativeViewAccessible();
}

const ui::AXUniqueId& AlertPlatformNodeDelegate::GetUniqueId() const {
  return id_;
}

const ui::AXNodeData& AlertPlatformNodeDelegate::GetData() const {
  return data_;
}

void AlertPlatformNodeDelegate::SetText(const std::u16string& text) {
  data_.SetName(text);
  data_.SetDescription(text);
  data_.SetValue(text);
}

}  // namespace flutter

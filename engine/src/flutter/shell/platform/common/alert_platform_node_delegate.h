// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_ALERT_PLATFORM_NODE_DELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_ALERT_PLATFORM_NODE_DELEGATE_H_

#include "flutter/fml/macros.h"
#include "flutter/third_party/accessibility/ax/ax_node_data.h"
#include "flutter/third_party/accessibility/ax/platform/ax_platform_node_delegate_base.h"

namespace flutter {

// A delegate for a node that holds the text of an a11y alert that a
// screen-reader should announce. The delegate is used to construct an
// AXPlatformNode, and in order to serve as an alert, only needs to be able to
// hold a text announcement and make that text available to the platform node.
class AlertPlatformNodeDelegate : public ui::AXPlatformNodeDelegateBase {
 public:
  explicit AlertPlatformNodeDelegate(
      ui::AXPlatformNodeDelegate& parent_delegate);
  ~AlertPlatformNodeDelegate();

  // Set the alert text of the node for which this is the delegate.
  void SetText(const std::u16string& text);

  // |AXPlatformNodeDelegate|
  gfx::NativeViewAccessible GetParent() override;

 private:
  // AXPlatformNodeDelegate overrides.
  gfx::AcceleratedWidget GetTargetForNativeAccessibilityEvent() override;
  const ui::AXUniqueId& GetUniqueId() const override;
  const ui::AXNodeData& GetData() const override;

  // Delegate of the parent of this node. Returned by GetParent.
  ui::AXPlatformNodeDelegate& parent_delegate_;

  // Node Data that contains the alert text. Returned by GetData.
  ui::AXNodeData data_;

  // A unique ID used to identify this node. Returned by GetUniqueId.
  ui::AXUniqueId id_;

  FML_DISALLOW_COPY_AND_ASSIGN(AlertPlatformNodeDelegate);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_ALERT_PLATFORM_NODE_DELEGATE_H_

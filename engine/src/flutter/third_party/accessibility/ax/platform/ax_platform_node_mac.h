// Copyright 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_MAC_H_
#define UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_MAC_H_

#import <Cocoa/Cocoa.h>

#include "base/macros.h"

#include "ax/ax_export.h"

#include "ax_platform_node_base.h"

@class AXPlatformNodeCocoa;

namespace ui {

class AXPlatformNodeMac : public AXPlatformNodeBase {
 public:
  AXPlatformNodeMac();

  // AXPlatformNode.
  gfx::NativeViewAccessible GetNativeViewAccessible() override;
  void NotifyAccessibilityEvent(ax::mojom::Event event_type) override;
  void AnnounceText(const std::u16string& text) override;

  // AXPlatformNodeBase.
  void Destroy() override;
  bool IsPlatformCheckable() const override;

 protected:
  void AddAttributeToList(const char* name,
                          const char* value,
                          PlatformAttributeList* attributes) override;

 private:
  ~AXPlatformNodeMac() override;

  AXPlatformNodeCocoa* native_node_;

  BASE_DISALLOW_COPY_AND_ASSIGN(AXPlatformNodeMac);
};

// Convenience function to determine whether an internal object role should
// expose its accessible name in AXValue (as opposed to AXTitle/AXDescription).
AX_EXPORT bool IsNameExposedInAXValueForRole(ax::mojom::Role role);

}  // namespace ui

AX_EXPORT
@interface AXPlatformNodeCocoa : NSAccessibilityElement <NSAccessibility>

// Maps AX roles to native roles. Returns NSAccessibilityUnknownRole if not
// found.
+ (NSString*)nativeRoleFromAXRole:(ax::mojom::Role)role;

// Maps AX roles to native subroles. Returns nil if not found.
+ (NSString*)nativeSubroleFromAXRole:(ax::mojom::Role)role;

// Maps AX events to native notifications. Returns nil if not found.
+ (NSString*)nativeNotificationFromAXEvent:(ax::mojom::Event)event;

- (instancetype)initWithNode:(ui::AXPlatformNodeBase*)node;
- (void)detach;

@property(nonatomic, readonly) NSRect boundsInScreen;
@property(nonatomic, readonly) ui::AXPlatformNodeBase* node;

@end

#endif  // UI_ACCESSIBILITY_PLATFORM_AX_PLATFORM_NODE_MAC_H_

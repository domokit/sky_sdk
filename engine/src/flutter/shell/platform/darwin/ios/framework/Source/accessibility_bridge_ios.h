// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_ACCESSIBILITY_BRIDGE_IOS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_ACCESSIBILITY_BRIDGE_IOS_H_

#include <memory>
#include <vector>

#import "flutter/fml/mapping.h"
#include "flutter/lib/ui/semantics/semantics_node.h"

@class UIView;
@class FlutterPlatformViewsController;

namespace flutter {

/// Interface that represents an accessibility bridge for iOS.
class AccessibilityBridgeIos {
 public:
  virtual ~AccessibilityBridgeIos() = default;
  virtual UIView* view() const = 0;
  virtual bool isVoiceOverRunning() const = 0;
  virtual UIView<UITextInput>* textInputView() = 0;
  virtual void DispatchSemanticsAction(int32_t id, flutter::SemanticsAction action) = 0;
  virtual void DispatchSemanticsAction(int32_t id,
                                       flutter::SemanticsAction action,
                                       fml::MallocMapping args) = 0;
  /**
   * A callback that is called when a SemanticObject receives focus.
   *
   * The input id is the uid of the newly focused SemanticObject.
   */
  virtual void AccessibilityObjectDidBecomeFocused(int32_t id) = 0;
  /**
   * A callback that is called when a SemanticObject loses focus
   *
   * The input id is the uid of the newly focused SemanticObject.
   */
  virtual void AccessibilityObjectDidLoseFocus(int32_t id) = 0;
  virtual FlutterPlatformViewsController* GetPlatformViewsController() const = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_ACCESSIBILITY_BRIDGE_IOS_H_

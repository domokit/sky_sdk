// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_ACCESSIBILITY_BRIDGE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_ACCESSIBILITY_BRIDGE_H_

#import <UIKit/UIKit.h>

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/lib/ui/semantics/custom_accessibility_action.h"
#include "flutter/lib/ui/semantics/semantics_node.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterView.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/SemanticsObject.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/accessibility_bridge_ios.h"
#include "third_party/skia/include/core/SkRect.h"

namespace flutter {
class PlatformViewIOS;

/**
 * An accessibility instance is bound to one `FlutterViewController` and
 * `FlutterView` instance.
 *
 * It helps populate the UIView's accessibilityElements property from Flutter's
 * semantics nodes.
 */
class AccessibilityBridge final : public AccessibilityBridgeIos {
 public:
  /** Delegate for handling iOS operations. */
  class IosDelegate {
   public:
    virtual ~IosDelegate() = default;
    /// Returns true when the FlutterViewController associated with the `view`
    /// is presenting a modal view controller.
    virtual bool IsFlutterViewControllerPresentingModalViewController(
        FlutterViewController* view_controller) = 0;
    virtual void PostAccessibilityNotification(UIAccessibilityNotifications notification,
                                               id argument) = 0;
  };

  AccessibilityBridge(FlutterViewController* view_controller,
                      PlatformViewIOS* platform_view,
                      __weak FlutterPlatformViewsController* platform_views_controller,
                      std::unique_ptr<IosDelegate> ios_delegate = nullptr);
  ~AccessibilityBridge();

  void UpdateSemantics(flutter::SemanticsNodeUpdates nodes,
                       const flutter::CustomAccessibilityActionUpdates& actions);
  void HandleEvent(NSDictionary<NSString*, id>* annotatedEvent);
  void DispatchSemanticsAction(int32_t id, flutter::SemanticsAction action) override;
  void DispatchSemanticsAction(int32_t id,
                               flutter::SemanticsAction action,
                               fml::MallocMapping args) override;
  void AccessibilityObjectDidBecomeFocused(int32_t id) override;
  void AccessibilityObjectDidLoseFocus(int32_t id) override;

  UIView<UITextInput>* textInputView() override;

  UIView* view() const override { return view_controller_.view; }

  bool isVoiceOverRunning() const override { return view_controller_.isVoiceOverRunning; }

  fml::WeakPtr<AccessibilityBridge> GetWeakPtr();

  FlutterPlatformViewsController* GetPlatformViewsController() const override {
    return platform_views_controller_;
  };

  void clearState();

 private:
  SemanticsObject* GetOrCreateObject(int32_t id, flutter::SemanticsNodeUpdates& updates);
  SemanticsObject* FindNextFocusableIfNecessary();
  // Finds the first focusable SemanticsObject rooted at the parent. This includes the parent itself
  // if it is a focusable SemanticsObject.
  //
  // If the parent is nil, this function use the root SemanticsObject as the parent.
  SemanticsObject* FindFirstFocusable(SemanticsObject* parent);
  void VisitObjectsRecursivelyAndRemove(SemanticsObject* object,
                                        NSMutableArray<NSNumber*>* doomed_uids);

  FlutterViewController* view_controller_;
  PlatformViewIOS* platform_view_;
  __weak FlutterPlatformViewsController* platform_views_controller_;
  // If the this id is kSemanticObjectIdInvalid, it means either nothing has
  // been focused or the focus is currently outside of the flutter application
  // (i.e. the status bar or keyboard)
  int32_t last_focused_semantics_object_id_;

  NSMutableDictionary<NSNumber*, SemanticsObject*>* objects_;
  FlutterBasicMessageChannel* accessibility_channel_;
  int32_t previous_route_id_ = 0;
  std::unordered_map<int32_t, flutter::CustomAccessibilityAction> actions_;
  std::vector<int32_t> previous_routes_;
  std::unique_ptr<IosDelegate> ios_delegate_;
  fml::WeakPtrFactory<AccessibilityBridge> weak_factory_;  // Must be the last member.
  FML_DISALLOW_COPY_AND_ASSIGN(AccessibilityBridge);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_ACCESSIBILITY_BRIDGE_H_

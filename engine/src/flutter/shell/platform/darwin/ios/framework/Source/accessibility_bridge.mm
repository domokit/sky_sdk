// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/accessibility_bridge.h"

#include <utility>

#include "flutter/fml/logging.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/TextInputSemanticsObject.h"
#import "flutter/shell/platform/darwin/ios/platform_view_ios.h"

#pragma GCC diagnostic error "-Wundeclared-selector"

FLUTTER_ASSERT_ARC

namespace flutter {
namespace {

constexpr int32_t kSemanticObjectIdInvalid = -1;

class DefaultIosDelegate : public AccessibilityBridge::IosDelegate {
 public:
  bool IsFlutterViewControllerPresentingModalViewController(
      FlutterViewController* view_controller) override {
    if (view_controller) {
      return view_controller.isPresentingViewController;
    } else {
      return false;
    }
  }

  void PostAccessibilityNotification(UIAccessibilityNotifications notification,
                                     id argument) override {
    UIAccessibilityPostNotification(notification, argument);
  }
};
}  // namespace

AccessibilityBridge::AccessibilityBridge(
    FlutterViewController* view_controller,
    PlatformViewIOS* platform_view,
    __weak FlutterPlatformViewsController* platform_views_controller,
    std::unique_ptr<IosDelegate> ios_delegate)
    : view_controller_(view_controller),
      platform_view_(platform_view),
      platform_views_controller_(platform_views_controller),
      last_focused_semantics_object_id_(kSemanticObjectIdInvalid),
      objects_([[NSMutableDictionary alloc] init]),
      previous_routes_({}),
      ios_delegate_(ios_delegate ? std::move(ios_delegate)
                                 : std::make_unique<DefaultIosDelegate>()),
      weak_factory_(this) {
  accessibility_channel_ = [[FlutterBasicMessageChannel alloc]
         initWithName:@"flutter/accessibility"
      binaryMessenger:platform_view->GetOwnerViewController().engine.binaryMessenger
                codec:[FlutterStandardMessageCodec sharedInstance]];
  [accessibility_channel_ setMessageHandler:^(id message, FlutterReply reply) {
    HandleEvent((NSDictionary*)message);
  }];
}

AccessibilityBridge::~AccessibilityBridge() {
  [accessibility_channel_ setMessageHandler:nil];
  clearState();
  view_controller_.viewIfLoaded.accessibilityElements = nil;
}

UIView<UITextInput>* AccessibilityBridge::textInputView() {
  return [[platform_view_->GetOwnerViewController().engine textInputPlugin] textInputView];
}

void AccessibilityBridge::AccessibilityObjectDidBecomeFocused(int32_t id) {
  last_focused_semantics_object_id_ = id;
  [accessibility_channel_ sendMessage:@{@"type" : @"didGainFocus", @"nodeId" : @(id)}];
}

void AccessibilityBridge::AccessibilityObjectDidLoseFocus(int32_t id) {
  if (last_focused_semantics_object_id_ == id) {
    last_focused_semantics_object_id_ = kSemanticObjectIdInvalid;
  }
}

void AccessibilityBridge::UpdateSemantics(
    flutter::SemanticsNodeUpdates nodes,
    const flutter::CustomAccessibilityActionUpdates& actions) {
  BOOL layoutChanged = NO;
  BOOL scrollOccured = NO;
  BOOL needsAnnouncement = NO;
  for (const auto& entry : actions) {
    const flutter::CustomAccessibilityAction& action = entry.second;
    actions_[action.id] = action;
  }
  for (const auto& entry : nodes) {
    const flutter::SemanticsNode& node = entry.second;
    SemanticsObject* object = GetOrCreateObject(node.id, nodes);
    layoutChanged = layoutChanged || [object nodeWillCauseLayoutChange:&node];
    scrollOccured = scrollOccured || [object nodeWillCauseScroll:&node];
    needsAnnouncement = [object nodeShouldTriggerAnnouncement:&node];
    [object setSemanticsNode:&node];
    NSUInteger newChildCount = node.childrenInTraversalOrder.size();
    NSMutableArray* newChildren = [[NSMutableArray alloc] initWithCapacity:newChildCount];
    for (NSUInteger i = 0; i < newChildCount; ++i) {
      SemanticsObject* child = GetOrCreateObject(node.childrenInTraversalOrder[i], nodes);
      [newChildren addObject:child];
    }
    NSMutableArray* newChildrenInHitTestOrder =
        [[NSMutableArray alloc] initWithCapacity:newChildCount];
    for (NSUInteger i = 0; i < newChildCount; ++i) {
      SemanticsObject* child = GetOrCreateObject(node.childrenInHitTestOrder[i], nodes);
      [newChildrenInHitTestOrder addObject:child];
    }
    object.children = newChildren;
    object.childrenInHitTestOrder = newChildrenInHitTestOrder;
    if (!node.customAccessibilityActions.empty()) {
      NSMutableArray<FlutterCustomAccessibilityAction*>* accessibilityCustomActions =
          [[NSMutableArray alloc] init];
      for (int32_t action_id : node.customAccessibilityActions) {
        flutter::CustomAccessibilityAction& action = actions_[action_id];
        if (action.overrideId != -1) {
          // iOS does not support overriding standard actions, so we ignore any
          // custom actions that have an override id provided.
          continue;
        }
        NSString* label = @(action.label.data());
        SEL selector = @selector(onCustomAccessibilityAction:);
        FlutterCustomAccessibilityAction* customAction =
            [[FlutterCustomAccessibilityAction alloc] initWithName:label
                                                            target:object
                                                          selector:selector];
        customAction.uid = action_id;
        [accessibilityCustomActions addObject:customAction];
      }
      object.accessibilityCustomActions = accessibilityCustomActions;
    }

    if (needsAnnouncement) {
      // Try to be more polite - iOS 11+ supports
      // UIAccessibilitySpeechAttributeQueueAnnouncement which should avoid
      // interrupting system notifications or other elements.
      // Expectation: roughly match the behavior of polite announcements on
      // Android.
      NSString* announcement = [[NSString alloc] initWithUTF8String:object.node.label.c_str()];
      UIAccessibilityPostNotification(
          UIAccessibilityAnnouncementNotification,
          [[NSAttributedString alloc] initWithString:announcement
                                          attributes:@{
                                            UIAccessibilitySpeechAttributeQueueAnnouncement : @YES
                                          }]);
    }
  }

  SemanticsObject* root = objects_[@(kRootNodeId)];

  bool routeChanged = false;
  SemanticsObject* lastAdded = nil;

  if (root) {
    if (!view_controller_.view.accessibilityElements) {
      view_controller_.view.accessibilityElements =
          @[ [root accessibilityContainer] ?: [NSNull null] ];
    }
    NSMutableArray<SemanticsObject*>* newRoutes = [[NSMutableArray alloc] init];
    [root collectRoutes:newRoutes];
    // Finds the last route that is not in the previous routes.
    for (SemanticsObject* route in newRoutes) {
      if (std::find(previous_routes_.begin(), previous_routes_.end(), [route uid]) ==
          previous_routes_.end()) {
        lastAdded = route;
      }
    }
    // If all the routes are in the previous route, get the last route.
    if (lastAdded == nil && [newRoutes count] > 0) {
      int index = [newRoutes count] - 1;
      lastAdded = [newRoutes objectAtIndex:index];
    }
    // There are two cases if lastAdded != nil
    // 1. lastAdded is not in previous routes. In this case,
    //    [lastAdded uid] != previous_route_id_
    // 2. All new routes are in previous routes and
    //    lastAdded = newRoutes.last.
    // In the first case, we need to announce new route. In the second case,
    // we need to announce if one list is shorter than the other.
    if (lastAdded != nil &&
        ([lastAdded uid] != previous_route_id_ || [newRoutes count] != previous_routes_.size())) {
      previous_route_id_ = [lastAdded uid];
      routeChanged = true;
    }
    previous_routes_.clear();
    for (SemanticsObject* route in newRoutes) {
      previous_routes_.push_back([route uid]);
    }
  } else {
    view_controller_.viewIfLoaded.accessibilityElements = nil;
  }

  NSMutableArray<NSNumber*>* doomed_uids = [NSMutableArray arrayWithArray:objects_.allKeys];
  if (root) {
    VisitObjectsRecursivelyAndRemove(root, doomed_uids);
  }
  [objects_ removeObjectsForKeys:doomed_uids];

  for (SemanticsObject* object in objects_.allValues) {
    [object accessibilityBridgeDidFinishUpdate];
  }

  if (!ios_delegate_->IsFlutterViewControllerPresentingModalViewController(view_controller_)) {
    layoutChanged = layoutChanged || [doomed_uids count] > 0;

    if (routeChanged) {
      NSString* routeName = [lastAdded routeName];
      ios_delegate_->PostAccessibilityNotification(UIAccessibilityScreenChangedNotification,
                                                   routeName);
    }

    if (layoutChanged) {
      SemanticsObject* next = FindNextFocusableIfNecessary();
      SemanticsObject* lastFocused = [objects_ objectForKey:@(last_focused_semantics_object_id_)];
      // Only specify the focus item if the new focus is different, avoiding double focuses on the
      // same item. See: https://github.com/flutter/flutter/issues/104176. If there is a route
      // change, we always refocus.
      ios_delegate_->PostAccessibilityNotification(
          UIAccessibilityLayoutChangedNotification,
          (routeChanged || next != lastFocused) ? next.nativeAccessibility : NULL);
    } else if (scrollOccured) {
      // TODO(chunhtai): figure out what string to use for notification. At this
      // point, it is guarantee the previous focused object is still in the tree
      // so that we don't need to worry about focus lost. (e.g. "Screen 0 of 3")
      ios_delegate_->PostAccessibilityNotification(
          UIAccessibilityPageScrolledNotification,
          FindNextFocusableIfNecessary().nativeAccessibility);
    }
  }
}

void AccessibilityBridge::DispatchSemanticsAction(int32_t uid, flutter::SemanticsAction action) {
  platform_view_->DispatchSemanticsAction(uid, action, {});
}

void AccessibilityBridge::DispatchSemanticsAction(int32_t uid,
                                                  flutter::SemanticsAction action,
                                                  fml::MallocMapping args) {
  platform_view_->DispatchSemanticsAction(uid, action, std::move(args));
}

static void ReplaceSemanticsObject(SemanticsObject* oldObject,
                                   SemanticsObject* newObject,
                                   NSMutableDictionary<NSNumber*, SemanticsObject*>* objects) {
  // `newObject` should represent the same id as `oldObject`.
  FML_DCHECK(oldObject.node.id == newObject.uid);
  NSNumber* nodeId = @(oldObject.node.id);
  NSUInteger positionInChildlist = [oldObject.parent.children indexOfObject:oldObject];
  oldObject.children = @[];
  [oldObject.parent replaceChildAtIndex:positionInChildlist withChild:newObject];
  [objects removeObjectForKey:nodeId];
  objects[nodeId] = newObject;
}

static SemanticsObject* CreateObject(const flutter::SemanticsNode& node,
                                     const fml::WeakPtr<AccessibilityBridge>& weak_ptr) {
  if (node.HasFlag(flutter::SemanticsFlags::kIsTextField) &&
      !node.HasFlag(flutter::SemanticsFlags::kIsReadOnly)) {
    // Text fields are backed by objects that implement UITextInput.
    return [[TextInputSemanticsObject alloc] initWithBridge:weak_ptr uid:node.id];
  } else if (!node.HasFlag(flutter::SemanticsFlags::kIsInMutuallyExclusiveGroup) &&
             (node.HasFlag(flutter::SemanticsFlags::kHasToggledState) ||
              node.HasFlag(flutter::SemanticsFlags::kHasCheckedState))) {
    return [[FlutterSwitchSemanticsObject alloc] initWithBridge:weak_ptr uid:node.id];
  } else if (node.HasFlag(flutter::SemanticsFlags::kHasImplicitScrolling)) {
    return [[FlutterScrollableSemanticsObject alloc] initWithBridge:weak_ptr uid:node.id];
  } else if (node.IsPlatformViewNode()) {
    FlutterPlatformViewsController* platformViewsController =
        weak_ptr->GetPlatformViewsController();
    FlutterTouchInterceptingView* touchInterceptingView =
        [platformViewsController flutterTouchInterceptingViewForId:node.platformViewId];
    return [[FlutterPlatformViewSemanticsContainer alloc] initWithBridge:weak_ptr
                                                                     uid:node.id
                                                            platformView:touchInterceptingView];
  } else {
    return [[FlutterSemanticsObject alloc] initWithBridge:weak_ptr uid:node.id];
  }
}

static bool DidFlagChange(const flutter::SemanticsNode& oldNode,
                          const flutter::SemanticsNode& newNode,
                          SemanticsFlags flag) {
  return oldNode.HasFlag(flag) != newNode.HasFlag(flag);
}

SemanticsObject* AccessibilityBridge::GetOrCreateObject(int32_t uid,
                                                        flutter::SemanticsNodeUpdates& updates) {
  SemanticsObject* object = objects_[@(uid)];
  if (!object) {
    object = CreateObject(updates[uid], GetWeakPtr());
    objects_[@(uid)] = object;
  } else {
    // Existing node case
    auto nodeEntry = updates.find(object.node.id);
    if (nodeEntry != updates.end()) {
      // There's an update for this node
      flutter::SemanticsNode node = nodeEntry->second;
      if (DidFlagChange(object.node, node, flutter::SemanticsFlags::kIsTextField) ||
          DidFlagChange(object.node, node, flutter::SemanticsFlags::kIsReadOnly) ||
          DidFlagChange(object.node, node, flutter::SemanticsFlags::kHasCheckedState) ||
          DidFlagChange(object.node, node, flutter::SemanticsFlags::kHasToggledState) ||
          DidFlagChange(object.node, node, flutter::SemanticsFlags::kHasImplicitScrolling)) {
        // The node changed its type. In this case, we cannot reuse the existing
        // SemanticsObject implementation. Instead, we replace it with a new
        // instance.
        SemanticsObject* newSemanticsObject = CreateObject(node, GetWeakPtr());
        ReplaceSemanticsObject(object, newSemanticsObject, objects_);
        object = newSemanticsObject;
      }
    }
  }
  return object;
}

void AccessibilityBridge::VisitObjectsRecursivelyAndRemove(SemanticsObject* object,
                                                           NSMutableArray<NSNumber*>* doomed_uids) {
  [doomed_uids removeObject:@(object.uid)];
  for (SemanticsObject* child in [object children])
    VisitObjectsRecursivelyAndRemove(child, doomed_uids);
}

SemanticsObject* AccessibilityBridge::FindNextFocusableIfNecessary() {
  // This property will be -1 if the focus is outside of the flutter
  // application. In this case, we should not refocus anything.
  if (last_focused_semantics_object_id_ == kSemanticObjectIdInvalid) {
    return nil;
  }

  // Tries to refocus the previous focused semantics object to avoid random jumps.
  return FindFirstFocusable(objects_[@(last_focused_semantics_object_id_)]);
}

SemanticsObject* AccessibilityBridge::FindFirstFocusable(SemanticsObject* parent) {
  SemanticsObject* currentObject = parent ?: objects_[@(kRootNodeId)];
  if (!currentObject) {
    return nil;
  }
  if (currentObject.isAccessibilityElement) {
    return currentObject;
  }

  for (SemanticsObject* child in [currentObject children]) {
    SemanticsObject* candidate = FindFirstFocusable(child);
    if (candidate) {
      return candidate;
    }
  }
  return nil;
}

void AccessibilityBridge::HandleEvent(NSDictionary<NSString*, id>* annotatedEvent) {
  NSString* type = annotatedEvent[@"type"];
  if ([type isEqualToString:@"announce"]) {
    NSString* message = annotatedEvent[@"data"][@"message"];
    ios_delegate_->PostAccessibilityNotification(UIAccessibilityAnnouncementNotification, message);
  }
  if ([type isEqualToString:@"focus"]) {
    SemanticsObject* node = objects_[annotatedEvent[@"nodeId"]];
    ios_delegate_->PostAccessibilityNotification(UIAccessibilityLayoutChangedNotification, node);
  }
}

fml::WeakPtr<AccessibilityBridge> AccessibilityBridge::GetWeakPtr() {
  return weak_factory_.GetWeakPtr();
}

void AccessibilityBridge::clearState() {
  [objects_ removeAllObjects];
  previous_route_id_ = 0;
  previous_routes_.clear();
}

}  // namespace flutter

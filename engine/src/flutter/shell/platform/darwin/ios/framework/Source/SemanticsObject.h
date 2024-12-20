// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_SEMANTICSOBJECT_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_SEMANTICSOBJECT_H_

#import <UIKit/UIKit.h>

#include "flutter/fml/macros.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/lib/ui/semantics/semantics_node.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterSemanticsScrollView.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/accessibility_bridge_ios.h"

constexpr int32_t kRootNodeId = 0;
// This can be arbitrary number as long as it is bigger than 0.
constexpr float kScrollExtentMaxForInf = 1000;

@class FlutterCustomAccessibilityAction;
@class FlutterPlatformViewSemanticsContainer;
@class FlutterTouchInterceptingView;

/**
 * A node in the iOS semantics tree. This object is a wrapper over a native accessibiliy
 * object, which is stored in the property `nativeAccessibility`. In the most case, the
 * `nativeAccessibility` directly returns this object. Some subclasses such as the
 * `FlutterScrollableSemanticsObject` creates a native `UIScrollView` as its `nativeAccessibility`
 * so that it can interact with iOS.
 */
@interface SemanticsObject : UIAccessibilityElement

/**
 * The globally unique identifier for this node.
 */
@property(nonatomic, readonly) int32_t uid;

/**
 * The parent of this node in the node tree. Will be nil for the root node and
 * during transient state changes.
 */
@property(nonatomic, weak, readonly) SemanticsObject* parent;

/**
 * The accessibility bridge that this semantics object is attached to. This
 * object may use the bridge to access contextual application information. A weak
 * pointer is used because the platform view owns the accessibility bridge.
 * If you are referencing this property from an iOS callback, be sure to
 * use `isAccessibilityBridgeActive` to protect against the case where this
 * node may be orphaned.
 */
@property(nonatomic, readonly) fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge;

/**
 * The semantics node used to produce this semantics object.
 */
@property(nonatomic, readonly) flutter::SemanticsNode node;

/**
 * Whether this semantics object has child semantics objects.
 */
@property(nonatomic, readonly) BOOL hasChildren;

/**
 * Direct children of this semantics object. Each child's `parent` property must
 * be equal to this object.
 */
@property(nonatomic, copy) NSArray<SemanticsObject*>* children;

/**
 * Direct children of this semantics object in hit test order. Each child's `parent` property
 * must be equal to this object.
 */
@property(nonatomic, copy) NSArray<SemanticsObject*>* childrenInHitTestOrder;

/**
 * The UIAccessibility that represents this object.
 *
 * By default, this return self. Subclasses can override to return different
 * objects to represent them. For example, FlutterScrollableSemanticsObject[s]
 * maintain UIScrollView[s] to represent their UIAccessibility[s].
 */
@property(nonatomic, readonly) id nativeAccessibility;

/**
 * Due to the fact that VoiceOver may hold onto SemanticObjects even after it shuts down,
 * there can be situations where the AccessibilityBridge is shutdown, but the SemanticObject
 * will still be alive. If VoiceOver is turned on again, it may try to access this orphaned
 * SemanticObject. Methods that are called from the accessiblity framework should use
 * this to guard against this case by just returning early if its bridge has been shutdown.
 *
 * See https://github.com/flutter/flutter/issues/43795 for more information.
 */
- (BOOL)isAccessibilityBridgeAlive;

/**
 * Updates this semantics object using data from the `node` argument.
 */
- (void)setSemanticsNode:(const flutter::SemanticsNode*)node NS_REQUIRES_SUPER;

- (void)replaceChildAtIndex:(NSInteger)index withChild:(SemanticsObject*)child;

- (BOOL)nodeWillCauseLayoutChange:(const flutter::SemanticsNode*)node;

- (BOOL)nodeWillCauseScroll:(const flutter::SemanticsNode*)node;

- (BOOL)nodeShouldTriggerAnnouncement:(const flutter::SemanticsNode*)node;

- (void)collectRoutes:(NSMutableArray<SemanticsObject*>*)edges;

- (NSString*)routeName;

- (BOOL)onCustomAccessibilityAction:(FlutterCustomAccessibilityAction*)action;

/**
 * Called after accessibility bridge finishes a semantics update.
 *
 * Subclasses can override this method if they contain states that can only be
 * updated once every node in the accessibility tree has finished updating.
 */
- (void)accessibilityBridgeDidFinishUpdate;

#pragma mark - Designated initializers

- (instancetype)init __attribute__((unavailable("Use initWithBridge instead")));
- (instancetype)initWithBridge:(fml::WeakPtr<flutter::AccessibilityBridgeIos>)bridge
                           uid:(int32_t)uid NS_DESIGNATED_INITIALIZER;

@end

/**
 * An implementation of UIAccessibilityCustomAction which also contains the
 * Flutter uid.
 */
@interface FlutterCustomAccessibilityAction : UIAccessibilityCustomAction

/**
 * The uid of the action defined by the flutter application.
 */
@property(nonatomic) int32_t uid;

@end

/**
 * The default implementation of `SemanticsObject` for most accessibility elements
 * in the iOS accessibility tree.
 *
 * Use this implementation for nodes that do not need to be expressed via UIKit-specific
 * protocols (it only implements NSObject).
 *
 * See also:
 *  * TextInputSemanticsObject, which implements `UITextInput` protocol to expose
 *    editable text widgets to a11y.
 */
@interface FlutterSemanticsObject : SemanticsObject
@end

/**
 * Designated to act as an accessibility container of a platform view.
 *
 * This object does not take any accessibility actions on its own, nor has any accessibility
 * label/value/trait/hint... on its own. The accessibility data will be handled by the platform
 * view.
 *
 * See also:
 * * `SemanticsObject` for the other type of semantics objects.
 * * `FlutterSemanticsObject` for default implementation of `SemanticsObject`.
 */
@interface FlutterPlatformViewSemanticsContainer : SemanticsObject

- (instancetype)initWithBridge:(fml::WeakPtr<flutter::AccessibilityBridgeIos>)bridge
                           uid:(int32_t)uid NS_UNAVAILABLE;

- (instancetype)initWithBridge:(fml::WeakPtr<flutter::AccessibilityBridgeIos>)bridge
                           uid:(int32_t)uid
                  platformView:(FlutterTouchInterceptingView*)platformView
    NS_DESIGNATED_INITIALIZER;

@end

/// The semantics object for switch buttons. This class creates an UISwitch to interact with the
/// iOS.
@interface FlutterSwitchSemanticsObject : SemanticsObject

@end

/// The semantics object for scrollable. This class creates an UIScrollView to interact with the
/// iOS.
@interface FlutterScrollableSemanticsObject : SemanticsObject
@property(nonatomic, readonly) FlutterSemanticsScrollView* scrollView;
@end

/**
 * Represents a semantics object that has children and hence has to be presented to the OS as a
 * UIAccessibilityContainer.
 *
 * The SemanticsObject class cannot implement the UIAccessibilityContainer protocol because an
 * object that returns YES for isAccessibilityElement cannot also implement
 * UIAccessibilityContainer.
 *
 * With the help of SemanticsObjectContainer, the hierarchy of semantic objects received from
 * the framework, such as:
 *
 * SemanticsObject1
 *     SemanticsObject2
 *         SemanticsObject3
 *         SemanticsObject4
 *
 * is translated into the following hierarchy, which is understood by iOS:
 *
 * SemanticsObjectContainer1
 *     SemanticsObject1
 *     SemanticsObjectContainer2
 *         SemanticsObject2
 *         SemanticsObject3
 *         SemanticsObject4
 *
 * From Flutter's view of the world (the first tree seen above), we construct iOS's view of the
 * world (second tree) as follows: We replace each SemanticsObjects that has children with a
 * SemanticsObjectContainer, which has the original SemanticsObject and its children as children.
 *
 * SemanticsObjects have semantic information attached to them which is interpreted by
 * VoiceOver (they return YES for isAccessibilityElement). The SemanticsObjectContainers are just
 * there for structure and they don't provide any semantic information to VoiceOver (they return
 * NO for isAccessibilityElement).
 */
@interface SemanticsObjectContainer : UIAccessibilityElement
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (instancetype)initWithAccessibilityContainer:(id)container NS_UNAVAILABLE;
- (instancetype)initWithSemanticsObject:(SemanticsObject*)semanticsObject
                                 bridge:(fml::WeakPtr<flutter::AccessibilityBridgeIos>)bridge
    NS_DESIGNATED_INITIALIZER;

@property(nonatomic, weak) SemanticsObject* semanticsObject;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_SEMANTICSOBJECT_H_

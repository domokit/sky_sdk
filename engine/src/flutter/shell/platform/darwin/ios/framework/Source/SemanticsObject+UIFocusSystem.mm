// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "SemanticsObject.h"
#include "flutter/lib/ui/semantics/semantics_node.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterCodecs.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterSemanticsScrollView.h"

FLUTTER_ASSERT_ARC

// The SemanticsObject class conforms to UIFocusItem and UIFocusItemContainer
// protocols, so the SemanticsObject tree can also be used to represent
// interactive UI components on screen that can receive UIFocusSystem focus.
//
// Typically, physical key events received by the FlutterViewController is
// first delivered to the framework, but that stopped working for navigation keys
// since iOS 15 when full keyboard access (FKA) is on, because those events are
// consumed by the UIFocusSystem and never dispatched to the UIResponders in the
// application (see
// https://developer.apple.com/documentation/uikit/uikeycommand/3780513-wantspriorityoversystembehavior
// ). FKA relies on the iOS focus engine, to enable FKA on iOS 15+, we use
// SemanticsObject to provide the iOS focus engine with the required hierarchical
// information and geometric context.
//
// The focus engine focus is different from accessibility focus, or even the
// currentFocus of the Flutter FocusManager in the framework. On iOS 15+, FKA
// key events are dispatched to the current iOS focus engine focus (and
// translated to calls such as -[NSObject accessibilityActivate]), while most
// other key events are dispatched to the framework.
@interface SemanticsObject (UIFocusSystem) <UIFocusItem, UIFocusItemContainer>
/// The `UIFocusItem` that represents this SemanticsObject.
///
///  For regular `SemanticsObject`s, this method returns `self`,
///  for `FlutterScrollableSemanticsObject`s, this method returns its scroll view.
- (id<UIFocusItem>)focusItem;
@end

@implementation SemanticsObject (UIFocusSystem)

- (id<UIFocusItem>)focusItem {
  return self;
}

#pragma mark - UIFocusEnvironment Conformance

- (void)setNeedsFocusUpdate {
}

- (void)updateFocusIfNeeded {
}

- (BOOL)shouldUpdateFocusInContext:(UIFocusUpdateContext*)context {
  return YES;
}

- (void)didUpdateFocusInContext:(UIFocusUpdateContext*)context
       withAnimationCoordinator:(UIFocusAnimationCoordinator*)coordinator {
}

- (id<UIFocusEnvironment>)parentFocusEnvironment {
  // The root SemanticsObject node's parent is the FlutterView.
  return self.parent.focusItem ?: self.bridge->view();
}

- (NSArray<id<UIFocusEnvironment>>*)preferredFocusEnvironments {
  return nil;
}

- (id<UIFocusItemContainer>)focusItemContainer {
  return self;
}

#pragma mark - UIFocusItem Conformance

- (BOOL)canBecomeFocused {
  if ((self.node.flags & static_cast<int32_t>(flutter::SemanticsFlags::kIsHidden)) != 0) {
    return NO;
  }
  // Currently only supports SemanticsObjects that handle
  // -[NSObject accessibilityActivate].
  return self.node.HasAction(flutter::SemanticsAction::kTap);
}

// The frame is described in the `coordinateSpace` of the
// `parentFocusEnvironment` (all `parentFocusEnvironment`s are `UIFocusItem`s).
//
// See also the `coordinateSpace` implementation.
// TODO(LongCatIsLooong): use CoreGraphics types.
- (CGRect)frame {
  SkPoint quad[4] = {SkPoint::Make(self.node.rect.left(), self.node.rect.top()),
                     SkPoint::Make(self.node.rect.left(), self.node.rect.bottom()),
                     SkPoint::Make(self.node.rect.right(), self.node.rect.top()),
                     SkPoint::Make(self.node.rect.right(), self.node.rect.bottom())};

  SkM44 transform = self.node.transform;
  FlutterSemanticsScrollView* scrollView;
  for (SemanticsObject* ancestor = self.parent; ancestor; ancestor = ancestor.parent) {
    if ([ancestor isKindOfClass:[FlutterScrollableSemanticsObject class]]) {
      scrollView = ((FlutterScrollableSemanticsObject*)ancestor).scrollView;
      break;
    }
    transform = ancestor.node.transform * transform;
  }

  for (auto& vertex : quad) {
    SkV4 vector = transform.map(vertex.x(), vertex.y(), 0, 1);
    vertex = SkPoint::Make(vector.x / vector.w, vector.y / vector.w);
  }

  SkRect rect;
  rect.setBounds(quad, 4);
  // If this UIFocusItemContainer's coordinateSpace is a UIScrollView, offset
  // the rect by `contentOffset` because the contentOffset translation is
  // incorporated into the paint transform at different node depth in UIKit
  // and Flutter. In Flutter, the translation is added to the cells
  // while in UIKit the viewport's bounds is manipulated (IOW, each cell's frame
  // in the UIScrollView coordinateSpace does not change when the UIScrollView
  // scrolls).
  CGRect unscaledRect =
      CGRectMake(rect.x() + scrollView.bounds.origin.x, rect.y() + scrollView.bounds.origin.y,
                 rect.width(), rect.height());
  if (scrollView) {
    return unscaledRect;
  }
  // `rect` could be in physical pixels since the root RenderObject ("RenderView")
  // applies a transform that turns logical pixels to physical pixels. Undo the
  // transform by dividing the coordinates by the screen's scale factor, if this
  // UIFocusItem's reported `coordinateSpace` is the root view (which means this
  // UIFocusItem is not inside of a scroll view).
  //
  // Screen can be nil if the FlutterView is covered by another native view.
  CGFloat scale = (self.bridge->view().window.screen ?: UIScreen.mainScreen).scale;
  return CGRectMake(unscaledRect.origin.x / scale, unscaledRect.origin.y / scale,
                    unscaledRect.size.width / scale, unscaledRect.size.height / scale);
}

#pragma mark - UIFocusItemContainer Conformance

- (NSArray<id<UIFocusItem>>*)focusItemsInRect:(CGRect)rect {
  // It seems the iOS focus system relies heavily on focusItemsInRect
  // (instead of preferredFocusEnvironments) for directional navigation.
  //
  // The order of the items seems to be important, menus and dialogs become
  // unreachable via FKA if the returned children are organized
  // in hit-test order.
  //
  // This method is only supposed to return items within the given
  // rect but returning everything in the subtree seems to work fine.
  NSMutableArray<id<UIFocusItem>>* reversedItems =
      [[NSMutableArray alloc] initWithCapacity:self.childrenInHitTestOrder.count];
  for (NSUInteger i = 0; i < self.childrenInHitTestOrder.count; ++i) {
    SemanticsObject* child = self.childrenInHitTestOrder[self.childrenInHitTestOrder.count - 1 - i];
    [reversedItems addObject:child.focusItem];
  }
  return reversedItems;
}

- (id<UICoordinateSpace>)coordinateSpace {
  // A regular SemanticsObject uses the same coordinate space as its parent.
  return self.parent.coordinateSpace ?: self.bridge->view();
}

@end

/// Scrollable containers interact with the iOS focus engine using the
/// `UIFocusItemScrollableContainer` protocol. The said protocol (and other focus-related protocols)
/// does not provide means to inform the focus system of layout changes. In order for the focus
/// highlight to update properly as the scroll view scrolls, this implementation incorporates a
/// UIScrollView into the focus hierarchy to workaround the highlight update problem.
///
///  As a result, in the current implementation only scrollable containers and the root node
///  establish their own `coordinateSpace`s. All other `UIFocusItemContainter`s use the same
///  `coordinateSpace` as the containing UIScrollView, or the root `FlutterView`, whichever is
///  closer.
///
/// See also the `frame` method implementation.
#pragma mark - Scrolling

@interface FlutterScrollableSemanticsObject (CoordinateSpace)
@end

@implementation FlutterScrollableSemanticsObject (CoordinateSpace)
- (id<UICoordinateSpace>)coordinateSpace {
  // A scrollable SemanticsObject uses the same coordinate space as the scroll view.
  // This may not work very well in nested scroll views.
  return self.scrollView;
}

- (id<UIFocusItem>)focusItem {
  return self.scrollView;
}

@end

@interface FlutterSemanticsScrollView (UIFocusItemScrollableContainer) <
    UIFocusItemScrollableContainer>
@end

@implementation FlutterSemanticsScrollView (UIFocusItemScrollableContainer)

#pragma mark - FlutterSemanticsScrollView UIFocusItemScrollableContainer Conformance

- (CGSize)visibleSize {
  return self.frame.size;
}

- (void)setContentOffset:(CGPoint)contentOffset {
  [super setContentOffset:contentOffset];
  // Do no send flutter::SemanticsAction::kScrollToOffset if it's triggered
  // by a framework update.
  if (![self.semanticsObject isAccessibilityBridgeAlive] || !self.isDoingSystemScrolling) {
    return;
  }

  double offset[2] = {contentOffset.x, contentOffset.y};
  FlutterStandardTypedData* offsetData = [FlutterStandardTypedData
      typedDataWithFloat64:[NSData dataWithBytes:&offset length:sizeof(offset)]];
  NSData* encoded = [[FlutterStandardMessageCodec sharedInstance] encode:offsetData];
  self.semanticsObject.bridge->DispatchSemanticsAction(
      self.semanticsObject.uid, flutter::SemanticsAction::kScrollToOffset,
      fml::MallocMapping::Copy(encoded.bytes, encoded.length));
}

- (BOOL)canBecomeFocused {
  return NO;
}

- (id<UIFocusEnvironment>)parentFocusEnvironment {
  return self.semanticsObject.parentFocusEnvironment;
}

- (NSArray<id<UIFocusEnvironment>>*)preferredFocusEnvironments {
  return nil;
}

- (NSArray<id<UIFocusItem>>*)focusItemsInRect:(CGRect)rect {
  return [self.semanticsObject focusItemsInRect:rect];
}
@end

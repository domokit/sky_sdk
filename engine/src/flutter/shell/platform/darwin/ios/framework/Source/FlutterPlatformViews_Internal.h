// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMVIEWS_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMVIEWS_INTERNAL_H_

#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterPlatformViews.h"

#include <Metal/Metal.h>

#include "flutter/flow/surface.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/task_runner.h"
#include "flutter/fml/trace_event.h"
#include "flutter/impeller/base/thread_safety.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformViewsController.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewResponder.h"
#include "flutter/shell/platform/darwin/ios/ios_context.h"
#include "third_party/skia/include/core/SkRect.h"

// A UIView that acts as a clipping mask for the |ChildClippingView|.
//
// On the [UIView drawRect:] method, this view performs a series of clipping operations and sets the
// alpha channel to the final resulting area to be 1; it also sets the "clipped out" area's alpha
// channel to be 0.
//
// When a UIView sets a |FlutterClippingMaskView| as its `maskView`, the alpha channel of the UIView
// is replaced with the alpha channel of the |FlutterClippingMaskView|.
@interface FlutterClippingMaskView : UIView

- (instancetype)initWithFrame:(CGRect)frame screenScale:(CGFloat)screenScale;

- (void)reset;

// Adds a clip rect operation to the queue.
//
// The `clipSkRect` is transformed with the `matrix` before adding to the queue.
- (void)clipRect:(const SkRect&)clipSkRect matrix:(const SkMatrix&)matrix;

// Adds a clip rrect operation to the queue.
//
// The `clipSkRRect` is transformed with the `matrix` before adding to the queue.
- (void)clipRRect:(const SkRRect&)clipSkRRect matrix:(const SkMatrix&)matrix;

// Adds a clip path operation to the queue.
//
// The `path` is transformed with the `matrix` before adding to the queue.
- (void)clipPath:(const SkPath&)path matrix:(const SkMatrix&)matrix;

@end

// A pool that provides |FlutterClippingMaskView|s.
//
// The pool has a capacity that can be set in the initializer.
// When requesting a FlutterClippingMaskView, the pool will first try to reuse an available maskView
// in the pool. If there are none available, a new FlutterClippingMaskView is constructed. If the
// capacity is reached, the newly constructed FlutterClippingMaskView is not added to the pool.
//
// Call |insertViewToPoolIfNeeded:| to return a maskView to the pool.
@interface FlutterClippingMaskViewPool : NSObject

// Initialize the pool with `capacity`. When the `capacity` is reached, a FlutterClippingMaskView is
// constructed when requested, and it is not added to the pool.
- (instancetype)initWithCapacity:(NSInteger)capacity;

// Reuse a maskView from the pool, or allocate a new one.
- (FlutterClippingMaskView*)getMaskViewWithFrame:(CGRect)frame;

// Insert the `maskView` into the pool.
- (void)insertViewToPoolIfNeeded:(FlutterClippingMaskView*)maskView;

@end

// An object represents a blur filter.
//
// This object produces a `backdropFilterView`.
// To blur a View, add `backdropFilterView` as a subView of the View.
@interface PlatformViewFilter : NSObject

// Determines the rect of the blur effect in the coordinate system of `backdropFilterView`'s
// parentView.
@property(nonatomic, readonly) CGRect frame;

// Determines the blur intensity.
//
// It is set as the value of `inputRadius` of the `gaussianFilter` that is internally used.
@property(nonatomic, readonly) CGFloat blurRadius;

// This is the view to use to blur the PlatformView.
//
// It is a modified version of UIKit's `UIVisualEffectView`.
// The inputRadius can be customized and it doesn't add any color saturation to the blurred view.
@property(nonatomic, readonly) UIVisualEffectView* backdropFilterView;

// For testing only.
+ (void)resetPreparation;

- (instancetype)init NS_UNAVAILABLE;

// Initialize the filter object.
//
// The `frame` determines the rect of the blur effect in the coordinate system of
// `backdropFilterView`'s parentView. The `blurRadius` determines the blur intensity. It is set as
// the value of `inputRadius` of the `gaussianFilter` that is internally used. The
// `UIVisualEffectView` is the view that is used to add the blur effects. It is modified to become
// `backdropFilterView`, which better supports the need of Flutter.
//
// Note: if the implementation of UIVisualEffectView changes in a way that affects the
// implementation in `PlatformViewFilter`, this method will return nil.
- (instancetype)initWithFrame:(CGRect)frame
                   blurRadius:(CGFloat)blurRadius
             visualEffectView:(UIVisualEffectView*)visualEffectView NS_DESIGNATED_INITIALIZER;

@end

// The parent view handles clipping to its subViews.
@interface ChildClippingView : UIView

// Applies blur backdrop filters to the ChildClippingView with blur values from
// filters.
- (void)applyBlurBackdropFilters:(NSArray<PlatformViewFilter*>*)filters;

// For testing only.
- (NSMutableArray*)backdropFilterSubviews;
@end

// A UIView that is used as the parent for embedded UIViews.
//
// This view has 2 roles:
// 1. Delay or prevent touch events from arriving the embedded view.
// 2. Dispatching all events that are hittested to the embedded view to the FlutterView.
@interface FlutterTouchInterceptingView : UIView
- (instancetype)initWithEmbeddedView:(UIView*)embeddedView
             platformViewsController:(FlutterPlatformViewsController*)platformViewsController
    gestureRecognizersBlockingPolicy:
        (FlutterPlatformViewGestureRecognizersBlockingPolicy)blockingPolicy;

// Stop delaying any active touch sequence (and let it arrive the embedded view).
- (void)releaseGesture;

// Prevent the touch sequence from ever arriving to the embedded view.
- (void)blockGesture;

// Get embedded view
- (UIView*)embeddedView;

// Sets flutterAccessibilityContainer as this view's accessibilityContainer.
@property(nonatomic, retain) id flutterAccessibilityContainer;
@end

@interface UIView (FirstResponder)
// Returns YES if a view or any of its descendant view is the first responder. Returns NO otherwise.
@property(nonatomic, readonly) BOOL flt_hasFirstResponderInViewHierarchySubtree;
@end

// This recognizer delays touch events from being dispatched to the responder chain until it failed
// recognizing a gesture.
//
// We only fail this recognizer when asked to do so by the Flutter framework (which does so by
// invoking an acceptGesture method on the platform_views channel). And this is how we allow the
// Flutter framework to delay or prevent the embedded view from getting a touch sequence.
@interface FlutterDelayingGestureRecognizer : UIGestureRecognizer <UIGestureRecognizerDelegate>

// Indicates that if the `FlutterDelayingGestureRecognizer`'s state should be set to
// `UIGestureRecognizerStateEnded` during next `touchesEnded` call.
@property(nonatomic) BOOL shouldEndInNextTouchesEnded;

// Indicates that the `FlutterDelayingGestureRecognizer`'s `touchesEnded` has been invoked without
// setting the state to `UIGestureRecognizerStateEnded`.
@property(nonatomic) BOOL touchedEndedWithoutBlocking;

@property(nonatomic) UIGestureRecognizer* forwardingRecognizer;

- (instancetype)initWithTarget:(id)target
                        action:(SEL)action
          forwardingRecognizer:(UIGestureRecognizer*)forwardingRecognizer;
@end

// While the FlutterDelayingGestureRecognizer is preventing touches from hitting the responder chain
// the touch events are not arriving to the FlutterView (and thus not arriving to the Flutter
// framework). We use this gesture recognizer to dispatch the events directly to the FlutterView
// while during this phase.
//
// If the Flutter framework decides to dispatch events to the embedded view, we fail the
// FlutterDelayingGestureRecognizer which sends the events up the responder chain. But since the
// events are handled by the embedded view they are not delivered to the Flutter framework in this
// phase as well. So during this phase as well the ForwardingGestureRecognizer dispatched the events
// directly to the FlutterView.
@interface ForwardingGestureRecognizer : UIGestureRecognizer <UIGestureRecognizerDelegate>
- (instancetype)initWithTarget:(id)target
       platformViewsController:(FlutterPlatformViewsController*)platformViewsController;
- (ForwardingGestureRecognizer*)recreateRecognizerWithTarget:(id)target;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMVIEWS_INTERNAL_H_

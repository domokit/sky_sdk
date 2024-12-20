// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMVIEWSCONTROLLER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMVIEWSCONTROLLER_H_

#include <Metal/Metal.h>
#include <memory>
#include <unordered_map>
#include <unordered_set>

#include "flutter/flow/surface.h"
#include "flutter/fml/task_runner.h"
#include "flutter/fml/trace_event.h"
#include "impeller/base/thread_safety.h"
#include "third_party/skia/include/core/SkRect.h"

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterPlatformViews.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewResponder.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/overlay_layer_pool.h"
#import "flutter/shell/platform/darwin/ios/ios_context.h"

NS_ASSUME_NONNULL_BEGIN

@class FlutterTouchInterceptingView;
@class FlutterClippingMaskViewPool;

@interface FlutterPlatformViewsController : NSObject

- (instancetype)init NS_DESIGNATED_INITIALIZER;

/// The task runner used to post rendering tasks to the platform thread.
@property(nonatomic, assign) const fml::RefPtr<fml::TaskRunner>& taskRunner;

/// The flutter view.
@property(nonatomic, weak) UIView* _Nullable flutterView;

/// @brief The flutter view controller.
@property(nonatomic, weak) UIViewController<FlutterViewResponder>* _Nullable flutterViewController;

/// @brief set the factory used to construct embedded UI Views.
- (void)registerViewFactory:(NSObject<FlutterPlatformViewFactory>*)factory
                              withId:(NSString*)factoryId
    gestureRecognizersBlockingPolicy:
        (FlutterPlatformViewGestureRecognizersBlockingPolicy)gestureRecognizerBlockingPolicy;

/// @brief Mark the beginning of a frame and record the size of the onscreen.
- (void)beginFrameWithSize:(SkISize)frameSize;

/// @brief Cancel the current frame, indicating that no platform views are composited.
///
/// Additionally, reverts the composition order to its original state at the beginning of the
/// frame.
- (void)cancelFrame;

/// @brief Record a platform view in the layer tree to be rendered, along with the positioning and
///        mutator parameters.
///
/// Called from the raster thread.
- (void)prerollCompositeEmbeddedView:(int64_t)viewId
                          withParams:(std::unique_ptr<flutter::EmbeddedViewParams>)params;

/// @brief Returns the`FlutterTouchInterceptingView` with the provided view_id.
///
/// Returns nil if there is no platform view with the provided id. Called
/// from the platform thread.
- (FlutterTouchInterceptingView*)flutterTouchInterceptingViewForId:(int64_t)viewId;

/// @brief Determine if thread merging is required after prerolling platform views.
///
/// Called from the raster thread.
- (flutter::PostPrerollResult)postPrerollActionWithThreadMerger:
                                  (const fml::RefPtr<fml::RasterThreadMerger>&)rasterThreadMerger
                                                impellerEnabled:(BOOL)impellerEnabled;

/// @brief Mark the end of a compositor frame.
///
/// May determine changes are required to the thread merging state.
/// Called from the raster thread.
- (void)endFrameWithResubmit:(BOOL)shouldResubmitFrame
                threadMerger:(const fml::RefPtr<fml::RasterThreadMerger>&)rasterThreadMerger
             impellerEnabled:(BOOL)impellerEnabled;

/// @brief Returns the Canvas for the overlay slice for the given platform view.
///
/// Called from the raster thread.
- (flutter::DlCanvas*)compositeEmbeddedViewWithId:(int64_t)viewId;

/// @brief Discards all platform views instances and auxiliary resources.
///
/// Called from the raster thread.
- (void)reset;

/// @brief Encode rendering for the Flutter overlay views and queue up perform platform view
/// mutations.
///
/// Called from the raster thread.
- (BOOL)submitFrame:(std::unique_ptr<flutter::SurfaceFrame>)frame
     withIosContext:(const std::shared_ptr<flutter::IOSContext>&)iosContext
          grContext:(GrDirectContext* _Nullable)grContext;

/// @brief Handler for platform view message channels.
- (void)onMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;

/// @brief Returns the platform view id if the platform view (or any of its descendant view) is
/// the first responder.
///
/// Returns -1 if no such platform view is found.
- (long)firstResponderPlatformViewId;

/// @brief Pushes backdrop filter mutation to the mutator stack of each visited platform view.
- (void)pushFilterToVisitedPlatformViews:(const std::shared_ptr<flutter::DlImageFilter>&)filter
                                withRect:(const SkRect&)filterRect;

/// @brief Pushes the view id of a visted platform view to the list of visied platform views.
- (void)pushVisitedPlatformViewId:(int64_t)viewId;

@end

@interface FlutterPlatformViewsController (Testing)

- (size_t)embeddedViewCount;

// Returns the `FlutterPlatformView`'s `view` object associated with the view_id.
//
// If the `PlatformViewsController` does not contain any `FlutterPlatformView` object or
// a `FlutterPlatformView` object associated with the view_id cannot be found, the method
// returns nil.
- (UIView* _Nullable)platformViewForId:(int64_t)viewId;

// Composite the PlatformView with `viewId`.
//
// Every frame, during the paint traversal of the layer tree, this method is called for all
// the PlatformViews in `_viewsToRecomposite`.
//
// Note that `_viewsToRecomposite` does not represent all the views in the view hierarchy,
// if a PlatformView does not change its composition parameter from last frame, it is not
// included in the `views_to_recomposite_`.
- (void)compositeView:(int64_t)viewId withParams:(const flutter::EmbeddedViewParams&)params;

- (const flutter::EmbeddedViewParams&)compositionParamsForView:(int64_t)viewId;

@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMVIEWSCONTROLLER_H_

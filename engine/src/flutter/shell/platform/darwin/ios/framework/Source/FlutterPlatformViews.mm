// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformViews_Internal.h"

#import <WebKit/WebKit.h>

#include "flutter/display_list/effects/dl_image_filter.h"
#include "flutter/fml/platform/darwin/cf_utils.h"
#import "flutter/shell/platform/darwin/ios/ios_surface.h"

FLUTTER_ASSERT_ARC

static constexpr int kMaxPointsInVerb = 4;

namespace {
CGRect GetCGRectFromSkRect(const SkRect& clipSkRect) {
  return CGRectMake(clipSkRect.fLeft, clipSkRect.fTop, clipSkRect.fRight - clipSkRect.fLeft,
                    clipSkRect.fBottom - clipSkRect.fTop);
}

CATransform3D GetCATransform3DFromSkMatrix(const SkMatrix& matrix) {
  // Skia only supports 2D transform so we don't map z.
  CATransform3D transform = CATransform3DIdentity;
  transform.m11 = matrix.getScaleX();
  transform.m21 = matrix.getSkewX();
  transform.m41 = matrix.getTranslateX();
  transform.m14 = matrix.getPerspX();

  transform.m12 = matrix.getSkewY();
  transform.m22 = matrix.getScaleY();
  transform.m42 = matrix.getTranslateY();
  transform.m24 = matrix.getPerspY();
  return transform;
}
}  // namespace

@interface PlatformViewFilter ()

// `YES` if the backdropFilterView has been configured at least once.
@property(nonatomic) BOOL backdropFilterViewConfigured;
@property(nonatomic) UIVisualEffectView* backdropFilterView;

// Updates the `visualEffectView` with the current filter parameters.
// Also sets `self.backdropFilterView` to the updated visualEffectView.
- (void)updateVisualEffectView:(UIVisualEffectView*)visualEffectView;

@end

@implementation PlatformViewFilter

static NSObject* _gaussianBlurFilter = nil;
// The index of "_UIVisualEffectBackdropView" in UIVisualEffectView's subViews.
static NSInteger _indexOfBackdropView = -1;
// The index of "_UIVisualEffectSubview" in UIVisualEffectView's subViews.
static NSInteger _indexOfVisualEffectSubview = -1;
static BOOL _preparedOnce = NO;

- (instancetype)initWithFrame:(CGRect)frame
                   blurRadius:(CGFloat)blurRadius
             visualEffectView:(UIVisualEffectView*)visualEffectView {
  if (self = [super init]) {
    _frame = frame;
    _blurRadius = blurRadius;
    [PlatformViewFilter prepareOnce:visualEffectView];
    if (![PlatformViewFilter isUIVisualEffectViewImplementationValid]) {
      FML_DLOG(ERROR) << "Apple's API for UIVisualEffectView changed. Update the implementation to "
                         "access the gaussianBlur CAFilter.";
      return nil;
    }
    _backdropFilterView = visualEffectView;
    _backdropFilterViewConfigured = NO;
  }
  return self;
}

+ (void)resetPreparation {
  _preparedOnce = NO;
  _gaussianBlurFilter = nil;
  _indexOfBackdropView = -1;
  _indexOfVisualEffectSubview = -1;
}

+ (void)prepareOnce:(UIVisualEffectView*)visualEffectView {
  if (_preparedOnce) {
    return;
  }
  for (NSUInteger i = 0; i < visualEffectView.subviews.count; i++) {
    UIView* view = visualEffectView.subviews[i];
    if ([NSStringFromClass([view class]) hasSuffix:@"BackdropView"]) {
      _indexOfBackdropView = i;
      for (NSObject* filter in view.layer.filters) {
        if ([[filter valueForKey:@"name"] isEqual:@"gaussianBlur"] &&
            [[filter valueForKey:@"inputRadius"] isKindOfClass:[NSNumber class]]) {
          _gaussianBlurFilter = filter;
          break;
        }
      }
    } else if ([NSStringFromClass([view class]) hasSuffix:@"VisualEffectSubview"]) {
      _indexOfVisualEffectSubview = i;
    }
  }
  _preparedOnce = YES;
}

+ (BOOL)isUIVisualEffectViewImplementationValid {
  return _indexOfBackdropView > -1 && _indexOfVisualEffectSubview > -1 && _gaussianBlurFilter;
}

- (UIVisualEffectView*)backdropFilterView {
  FML_DCHECK(_backdropFilterView);
  if (!self.backdropFilterViewConfigured) {
    [self updateVisualEffectView:_backdropFilterView];
    self.backdropFilterViewConfigured = YES;
  }
  return _backdropFilterView;
}

- (void)updateVisualEffectView:(UIVisualEffectView*)visualEffectView {
  NSObject* gaussianBlurFilter = [_gaussianBlurFilter copy];
  FML_DCHECK(gaussianBlurFilter);
  UIView* backdropView = visualEffectView.subviews[_indexOfBackdropView];
  [gaussianBlurFilter setValue:@(_blurRadius) forKey:@"inputRadius"];
  backdropView.layer.filters = @[ gaussianBlurFilter ];

  UIView* visualEffectSubview = visualEffectView.subviews[_indexOfVisualEffectSubview];
  visualEffectSubview.layer.backgroundColor = UIColor.clearColor.CGColor;
  visualEffectView.frame = _frame;

  self.backdropFilterView = visualEffectView;
}

@end

@interface ChildClippingView ()

@property(nonatomic, copy) NSArray<PlatformViewFilter*>* filters;
@property(nonatomic) NSMutableArray<UIVisualEffectView*>* backdropFilterSubviews;

@end

@implementation ChildClippingView

// The ChildClippingView's frame is the bounding rect of the platform view. we only want touches to
// be hit tested and consumed by this view if they are inside the embedded platform view which could
// be smaller the embedded platform view is rotated.
- (BOOL)pointInside:(CGPoint)point withEvent:(UIEvent*)event {
  for (UIView* view in self.subviews) {
    if ([view pointInside:[self convertPoint:point toView:view] withEvent:event]) {
      return YES;
    }
  }
  return NO;
}

- (void)applyBlurBackdropFilters:(NSArray<PlatformViewFilter*>*)filters {
  FML_DCHECK(self.filters.count == self.backdropFilterSubviews.count);
  if (self.filters.count == 0 && filters.count == 0) {
    return;
  }
  self.filters = filters;
  NSUInteger index = 0;
  for (index = 0; index < self.filters.count; index++) {
    UIVisualEffectView* backdropFilterView;
    PlatformViewFilter* filter = self.filters[index];
    if (self.backdropFilterSubviews.count <= index) {
      backdropFilterView = filter.backdropFilterView;
      [self addSubview:backdropFilterView];
      [self.backdropFilterSubviews addObject:backdropFilterView];
    } else {
      [filter updateVisualEffectView:self.backdropFilterSubviews[index]];
    }
  }
  for (NSUInteger i = self.backdropFilterSubviews.count; i > index; i--) {
    [self.backdropFilterSubviews[i - 1] removeFromSuperview];
    [self.backdropFilterSubviews removeLastObject];
  }
}

- (NSMutableArray*)backdropFilterSubviews {
  if (!_backdropFilterSubviews) {
    _backdropFilterSubviews = [[NSMutableArray alloc] init];
  }
  return _backdropFilterSubviews;
}

@end

@interface FlutterClippingMaskView ()

// A `CATransform3D` matrix represnts a scale transform that revese UIScreen.scale.
//
// The transform matrix passed in clipRect/clipRRect/clipPath methods are in device coordinate
// space. The transfrom matrix concats `reverseScreenScale` to create a transform matrix in the iOS
// logical coordinates (points).
//
// See https://developer.apple.com/documentation/uikit/uiscreen/1617836-scale?language=objc for
// information about screen scale.
@property(nonatomic) CATransform3D reverseScreenScale;

- (fml::CFRef<CGPathRef>)getTransformedPath:(CGPathRef)path matrix:(CATransform3D)matrix;

@end

@implementation FlutterClippingMaskView {
  std::vector<fml::CFRef<CGPathRef>> paths_;
  BOOL containsNonRectPath_;
  CGRect rectSoFar_;
}

- (instancetype)initWithFrame:(CGRect)frame {
  return [self initWithFrame:frame screenScale:[UIScreen mainScreen].scale];
}

- (instancetype)initWithFrame:(CGRect)frame screenScale:(CGFloat)screenScale {
  if (self = [super initWithFrame:frame]) {
    self.backgroundColor = UIColor.clearColor;
    _reverseScreenScale = CATransform3DMakeScale(1 / screenScale, 1 / screenScale, 1);
    rectSoFar_ = self.bounds;
    containsNonRectPath_ = NO;
  }
  return self;
}

+ (Class)layerClass {
  return [CAShapeLayer class];
}

- (CAShapeLayer*)shapeLayer {
  return (CAShapeLayer*)self.layer;
}

- (void)reset {
  paths_.clear();
  rectSoFar_ = self.bounds;
  containsNonRectPath_ = NO;
  [self shapeLayer].path = nil;
  [self setNeedsDisplay];
}

// In some scenarios, when we add this view as a maskView of the ChildClippingView, iOS added
// this view as a subview of the ChildClippingView.
// This results this view blocking touch events on the ChildClippingView.
// So we should always ignore any touch events sent to this view.
// See https://github.com/flutter/flutter/issues/66044
- (BOOL)pointInside:(CGPoint)point withEvent:(UIEvent*)event {
  return NO;
}

- (void)drawRect:(CGRect)rect {
  // It's hard to compute intersection of arbitrary non-rect paths.
  // So we fallback to software rendering.
  if (containsNonRectPath_ && paths_.size() > 1) {
    CGContextRef context = UIGraphicsGetCurrentContext();
    CGContextSaveGState(context);

    // For mask view, only the alpha channel is used.
    CGContextSetAlpha(context, 1);

    for (size_t i = 0; i < paths_.size(); i++) {
      CGContextAddPath(context, paths_.at(i));
      CGContextClip(context);
    }
    CGContextFillRect(context, rect);
    CGContextRestoreGState(context);
  } else {
    // Either a single path, or multiple rect paths.
    // Use hardware rendering with CAShapeLayer.
    [super drawRect:rect];
    if (![self shapeLayer].path) {
      if (paths_.size() == 1) {
        // A single path, either rect or non-rect.
        [self shapeLayer].path = paths_.at(0);
      } else {
        // Multiple paths, all paths must be rects.
        CGPathRef pathSoFar = CGPathCreateWithRect(rectSoFar_, nil);
        [self shapeLayer].path = pathSoFar;
        CGPathRelease(pathSoFar);
      }
    }
  }
}

- (void)clipRect:(const SkRect&)clipSkRect matrix:(const SkMatrix&)matrix {
  CGRect clipRect = GetCGRectFromSkRect(clipSkRect);
  CGPathRef path = CGPathCreateWithRect(clipRect, nil);
  // The `matrix` is based on the physical pixels, convert it to UIKit points.
  CATransform3D matrixInPoints =
      CATransform3DConcat(GetCATransform3DFromSkMatrix(matrix), _reverseScreenScale);
  paths_.push_back([self getTransformedPath:path matrix:matrixInPoints]);
  CGAffineTransform affine = [self affineWithMatrix:matrixInPoints];
  // Make sure the rect is not rotated (only translated or scaled).
  if (affine.b == 0 && affine.c == 0) {
    rectSoFar_ = CGRectIntersection(rectSoFar_, CGRectApplyAffineTransform(clipRect, affine));
  } else {
    containsNonRectPath_ = YES;
  }
}

- (void)clipRRect:(const SkRRect&)clipSkRRect matrix:(const SkMatrix&)matrix {
  containsNonRectPath_ = YES;
  CGPathRef pathRef = nullptr;
  switch (clipSkRRect.getType()) {
    case SkRRect::kEmpty_Type: {
      break;
    }
    case SkRRect::kRect_Type: {
      [self clipRect:clipSkRRect.rect() matrix:matrix];
      return;
    }
    case SkRRect::kOval_Type:
    case SkRRect::kSimple_Type: {
      CGRect clipRect = GetCGRectFromSkRect(clipSkRRect.rect());
      pathRef = CGPathCreateWithRoundedRect(clipRect, clipSkRRect.getSimpleRadii().x(),
                                            clipSkRRect.getSimpleRadii().y(), nil);
      break;
    }
    case SkRRect::kNinePatch_Type:
    case SkRRect::kComplex_Type: {
      CGMutablePathRef mutablePathRef = CGPathCreateMutable();
      // Complex types, we manually add each corner.
      SkRect clipSkRect = clipSkRRect.rect();
      SkVector topLeftRadii = clipSkRRect.radii(SkRRect::kUpperLeft_Corner);
      SkVector topRightRadii = clipSkRRect.radii(SkRRect::kUpperRight_Corner);
      SkVector bottomRightRadii = clipSkRRect.radii(SkRRect::kLowerRight_Corner);
      SkVector bottomLeftRadii = clipSkRRect.radii(SkRRect::kLowerLeft_Corner);

      // Start drawing RRect
      // Move point to the top left corner adding the top left radii's x.
      CGPathMoveToPoint(mutablePathRef, nil, clipSkRect.fLeft + topLeftRadii.x(), clipSkRect.fTop);
      // Move point horizontally right to the top right corner and add the top right curve.
      CGPathAddLineToPoint(mutablePathRef, nil, clipSkRect.fRight - topRightRadii.x(),
                           clipSkRect.fTop);
      CGPathAddCurveToPoint(mutablePathRef, nil, clipSkRect.fRight, clipSkRect.fTop,
                            clipSkRect.fRight, clipSkRect.fTop + topRightRadii.y(),
                            clipSkRect.fRight, clipSkRect.fTop + topRightRadii.y());
      // Move point vertically down to the bottom right corner and add the bottom right curve.
      CGPathAddLineToPoint(mutablePathRef, nil, clipSkRect.fRight,
                           clipSkRect.fBottom - bottomRightRadii.y());
      CGPathAddCurveToPoint(mutablePathRef, nil, clipSkRect.fRight, clipSkRect.fBottom,
                            clipSkRect.fRight - bottomRightRadii.x(), clipSkRect.fBottom,
                            clipSkRect.fRight - bottomRightRadii.x(), clipSkRect.fBottom);
      // Move point horizontally left to the bottom left corner and add the bottom left curve.
      CGPathAddLineToPoint(mutablePathRef, nil, clipSkRect.fLeft + bottomLeftRadii.x(),
                           clipSkRect.fBottom);
      CGPathAddCurveToPoint(mutablePathRef, nil, clipSkRect.fLeft, clipSkRect.fBottom,
                            clipSkRect.fLeft, clipSkRect.fBottom - bottomLeftRadii.y(),
                            clipSkRect.fLeft, clipSkRect.fBottom - bottomLeftRadii.y());
      // Move point vertically up to the top left corner and add the top left curve.
      CGPathAddLineToPoint(mutablePathRef, nil, clipSkRect.fLeft,
                           clipSkRect.fTop + topLeftRadii.y());
      CGPathAddCurveToPoint(mutablePathRef, nil, clipSkRect.fLeft, clipSkRect.fTop,
                            clipSkRect.fLeft + topLeftRadii.x(), clipSkRect.fTop,
                            clipSkRect.fLeft + topLeftRadii.x(), clipSkRect.fTop);
      CGPathCloseSubpath(mutablePathRef);

      pathRef = mutablePathRef;
      break;
    }
  }
  // The `matrix` is based on the physical pixels, convert it to UIKit points.
  CATransform3D matrixInPoints =
      CATransform3DConcat(GetCATransform3DFromSkMatrix(matrix), _reverseScreenScale);
  // TODO(cyanglaz): iOS does not seem to support hard edge on CAShapeLayer. It clearly stated that
  // the CAShaperLayer will be drawn antialiased. Need to figure out a way to do the hard edge
  // clipping on iOS.
  paths_.push_back([self getTransformedPath:pathRef matrix:matrixInPoints]);
}

- (void)clipPath:(const SkPath&)path matrix:(const SkMatrix&)matrix {
  if (!path.isValid()) {
    return;
  }
  if (path.isEmpty()) {
    return;
  }
  containsNonRectPath_ = YES;
  CGMutablePathRef pathRef = CGPathCreateMutable();

  // Loop through all verbs and translate them into CGPath
  SkPath::Iter iter(path, true);
  SkPoint pts[kMaxPointsInVerb];
  SkPath::Verb verb = iter.next(pts);
  SkPoint last_pt_from_last_verb = SkPoint::Make(0, 0);
  while (verb != SkPath::kDone_Verb) {
    if (verb == SkPath::kLine_Verb || verb == SkPath::kQuad_Verb || verb == SkPath::kConic_Verb ||
        verb == SkPath::kCubic_Verb) {
      FML_DCHECK(last_pt_from_last_verb == pts[0]);
    }
    switch (verb) {
      case SkPath::kMove_Verb: {
        CGPathMoveToPoint(pathRef, nil, pts[0].x(), pts[0].y());
        last_pt_from_last_verb = pts[0];
        break;
      }
      case SkPath::kLine_Verb: {
        CGPathAddLineToPoint(pathRef, nil, pts[1].x(), pts[1].y());
        last_pt_from_last_verb = pts[1];
        break;
      }
      case SkPath::kQuad_Verb: {
        CGPathAddQuadCurveToPoint(pathRef, nil, pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
        last_pt_from_last_verb = pts[2];
        break;
      }
      case SkPath::kConic_Verb: {
        // Conic is not available in quartz, we use quad to approximate.
        // TODO(cyanglaz): Better approximate the conic path.
        // https://github.com/flutter/flutter/issues/35062
        CGPathAddQuadCurveToPoint(pathRef, nil, pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y());
        last_pt_from_last_verb = pts[2];
        break;
      }
      case SkPath::kCubic_Verb: {
        CGPathAddCurveToPoint(pathRef, nil, pts[1].x(), pts[1].y(), pts[2].x(), pts[2].y(),
                              pts[3].x(), pts[3].y());
        last_pt_from_last_verb = pts[3];
        break;
      }
      case SkPath::kClose_Verb: {
        CGPathCloseSubpath(pathRef);
        break;
      }
      case SkPath::kDone_Verb: {
        break;
      }
    }
    verb = iter.next(pts);
  }
  // The `matrix` is based on the physical pixels, convert it to UIKit points.
  CATransform3D matrixInPoints =
      CATransform3DConcat(GetCATransform3DFromSkMatrix(matrix), _reverseScreenScale);
  paths_.push_back([self getTransformedPath:pathRef matrix:matrixInPoints]);
}

- (CGAffineTransform)affineWithMatrix:(CATransform3D)matrix {
  return CGAffineTransformMake(matrix.m11, matrix.m12, matrix.m21, matrix.m22, matrix.m41,
                               matrix.m42);
}

- (fml::CFRef<CGPathRef>)getTransformedPath:(CGPathRef)path matrix:(CATransform3D)matrix {
  CGAffineTransform affine = [self affineWithMatrix:matrix];
  CGPathRef transformedPath = CGPathCreateCopyByTransformingPath(path, &affine);

  CGPathRelease(path);
  return fml::CFRef<CGPathRef>(transformedPath);
}

@end

@interface FlutterClippingMaskViewPool ()

// The maximum number of `FlutterClippingMaskView` the pool can contain.
// This prevents the pool to grow infinately and limits the maximum memory a pool can use.
@property(nonatomic) NSUInteger capacity;

// The pool contains the views that are available to use.
// The number of items in the pool must not excceds `capacity`.
@property(nonatomic) NSMutableSet<FlutterClippingMaskView*>* pool;

@end

@implementation FlutterClippingMaskViewPool : NSObject

- (instancetype)initWithCapacity:(NSInteger)capacity {
  if (self = [super init]) {
    // Most of cases, there are only one PlatformView in the scene.
    // Thus init with the capacity of 1.
    _pool = [[NSMutableSet alloc] initWithCapacity:1];
    _capacity = capacity;
  }
  return self;
}

- (FlutterClippingMaskView*)getMaskViewWithFrame:(CGRect)frame {
  FML_DCHECK(self.pool.count <= self.capacity);
  if (self.pool.count == 0) {
    // The pool is empty, alloc a new one.
    return [[FlutterClippingMaskView alloc] initWithFrame:frame
                                              screenScale:UIScreen.mainScreen.scale];
  }
  FlutterClippingMaskView* maskView = [self.pool anyObject];
  maskView.frame = frame;
  [maskView reset];
  [self.pool removeObject:maskView];
  return maskView;
}

- (void)insertViewToPoolIfNeeded:(FlutterClippingMaskView*)maskView {
  FML_DCHECK(![self.pool containsObject:maskView]);
  FML_DCHECK(self.pool.count <= self.capacity);
  if (self.pool.count == self.capacity) {
    return;
  }
  [self.pool addObject:maskView];
}

@end

@implementation UIView (FirstResponder)
- (BOOL)flt_hasFirstResponderInViewHierarchySubtree {
  if (self.isFirstResponder) {
    return YES;
  }
  for (UIView* subview in self.subviews) {
    if (subview.flt_hasFirstResponderInViewHierarchySubtree) {
      return YES;
    }
  }
  return NO;
}
@end

@interface FlutterTouchInterceptingView ()
@property(nonatomic, weak, readonly) UIView* embeddedView;
@property(nonatomic, readonly) FlutterDelayingGestureRecognizer* delayingRecognizer;
@property(nonatomic, readonly) FlutterPlatformViewGestureRecognizersBlockingPolicy blockingPolicy;
@end

@implementation FlutterTouchInterceptingView
- (instancetype)initWithEmbeddedView:(UIView*)embeddedView
             platformViewsController:(FlutterPlatformViewsController*)platformViewsController
    gestureRecognizersBlockingPolicy:
        (FlutterPlatformViewGestureRecognizersBlockingPolicy)blockingPolicy {
  self = [super initWithFrame:embeddedView.frame];
  if (self) {
    self.multipleTouchEnabled = YES;
    _embeddedView = embeddedView;
    embeddedView.autoresizingMask =
        (UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight);

    [self addSubview:embeddedView];

    ForwardingGestureRecognizer* forwardingRecognizer =
        [[ForwardingGestureRecognizer alloc] initWithTarget:self
                                    platformViewsController:platformViewsController];

    _delayingRecognizer =
        [[FlutterDelayingGestureRecognizer alloc] initWithTarget:self
                                                          action:nil
                                            forwardingRecognizer:forwardingRecognizer];
    _blockingPolicy = blockingPolicy;

    [self addGestureRecognizer:_delayingRecognizer];
    [self addGestureRecognizer:forwardingRecognizer];
  }
  return self;
}

- (void)forceResetForwardingGestureRecognizerState {
  // When iPad pencil is involved in a finger touch gesture, the gesture is not reset to "possible"
  // state and is stuck on "failed" state, which causes subsequent touches to be blocked. As a
  // workaround, we force reset the state by recreating the forwarding gesture recognizer. See:
  // https://github.com/flutter/flutter/issues/136244
  ForwardingGestureRecognizer* oldForwardingRecognizer =
      (ForwardingGestureRecognizer*)self.delayingRecognizer.forwardingRecognizer;
  ForwardingGestureRecognizer* newForwardingRecognizer =
      [oldForwardingRecognizer recreateRecognizerWithTarget:self];
  self.delayingRecognizer.forwardingRecognizer = newForwardingRecognizer;
  [self removeGestureRecognizer:oldForwardingRecognizer];
  [self addGestureRecognizer:newForwardingRecognizer];
}

- (void)releaseGesture {
  self.delayingRecognizer.state = UIGestureRecognizerStateFailed;
}

- (BOOL)containsWebView:(UIView*)view remainingSubviewDepth:(int)remainingSubviewDepth {
  if (remainingSubviewDepth < 0) {
    return NO;
  }
  if ([view isKindOfClass:[WKWebView class]]) {
    return YES;
  }
  for (UIView* subview in view.subviews) {
    if ([self containsWebView:subview remainingSubviewDepth:remainingSubviewDepth - 1]) {
      return YES;
    }
  }
  return NO;
}

- (void)blockGesture {
  switch (_blockingPolicy) {
    case FlutterPlatformViewGestureRecognizersBlockingPolicyEager:
      // We block all other gesture recognizers immediately in this policy.
      self.delayingRecognizer.state = UIGestureRecognizerStateEnded;

      // On iOS 18.2, WKWebView's internal recognizer likely caches the old state of its blocking
      // recognizers (i.e. delaying recognizer), resulting in non-tappable links. See
      // https://github.com/flutter/flutter/issues/158961. Removing and adding back the delaying
      // recognizer solves the problem, possibly because UIKit notifies all the recognizers related
      // to (blocking or blocked by) this recognizer. It is not possible to inject this workaround
      // from the web view plugin level. Right now we only observe this issue for
      // FlutterPlatformViewGestureRecognizersBlockingPolicyEager, but we should try it if a similar
      // issue arises for the other policy.
      if (@available(iOS 18.2, *)) {
        // This workaround is designed for WKWebView only. The 1P web view plugin provides a
        // WKWebView itself as the platform view. However, some 3P plugins provide wrappers of
        // WKWebView instead. So we perform DFS to search the view hierarchy (with a depth limit).
        // Passing a limit of 0 means only searching for platform view itself; Pass 1 to include its
        // children as well, and so on. We should be conservative and start with a small number. The
        // AdMob banner has a WKWebView at depth 7.
        if ([self containsWebView:self.embeddedView remainingSubviewDepth:1]) {
          [self removeGestureRecognizer:self.delayingRecognizer];
          [self addGestureRecognizer:self.delayingRecognizer];
        }
      }

      break;
    case FlutterPlatformViewGestureRecognizersBlockingPolicyWaitUntilTouchesEnded:
      if (self.delayingRecognizer.touchedEndedWithoutBlocking) {
        // If touchesEnded of the `DelayingGesureRecognizer` has been already invoked,
        // we want to set the state of the `DelayingGesureRecognizer` to
        // `UIGestureRecognizerStateEnded` as soon as possible.
        self.delayingRecognizer.state = UIGestureRecognizerStateEnded;
      } else {
        // If touchesEnded of the `DelayingGesureRecognizer` has not been invoked,
        // We will set a flag to notify the `DelayingGesureRecognizer` to set the state to
        // `UIGestureRecognizerStateEnded` when touchesEnded is called.
        self.delayingRecognizer.shouldEndInNextTouchesEnded = YES;
      }
      break;
    default:
      break;
  }
}

// We want the intercepting view to consume the touches and not pass the touches up to the parent
// view. Make the touch event method not call super will not pass the touches up to the parent view.
// Hence we overide the touch event methods and do nothing.
- (void)touchesBegan:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
}

- (void)touchesMoved:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
}

- (void)touchesCancelled:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
}

- (void)touchesEnded:(NSSet*)touches withEvent:(UIEvent*)event {
}

- (id)accessibilityContainer {
  return self.flutterAccessibilityContainer;
}

@end

@implementation FlutterDelayingGestureRecognizer

- (instancetype)initWithTarget:(id)target
                        action:(SEL)action
          forwardingRecognizer:(UIGestureRecognizer*)forwardingRecognizer {
  self = [super initWithTarget:target action:action];
  if (self) {
    self.delaysTouchesBegan = YES;
    self.delaysTouchesEnded = YES;
    self.delegate = self;
    _shouldEndInNextTouchesEnded = NO;
    _touchedEndedWithoutBlocking = NO;
    _forwardingRecognizer = forwardingRecognizer;
  }
  return self;
}

- (BOOL)gestureRecognizer:(UIGestureRecognizer*)gestureRecognizer
    shouldBeRequiredToFailByGestureRecognizer:(UIGestureRecognizer*)otherGestureRecognizer {
  // The forwarding gesture recognizer should always get all touch events, so it should not be
  // required to fail by any other gesture recognizer.
  return otherGestureRecognizer != _forwardingRecognizer && otherGestureRecognizer != self;
}

- (BOOL)gestureRecognizer:(UIGestureRecognizer*)gestureRecognizer
    shouldRequireFailureOfGestureRecognizer:(UIGestureRecognizer*)otherGestureRecognizer {
  return otherGestureRecognizer == self;
}

- (void)touchesBegan:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
  self.touchedEndedWithoutBlocking = NO;
  [super touchesBegan:touches withEvent:event];
}

- (void)touchesEnded:(NSSet<UITouch*>*)touches withEvent:(UIEvent*)event {
  if (self.shouldEndInNextTouchesEnded) {
    self.state = UIGestureRecognizerStateEnded;
    self.shouldEndInNextTouchesEnded = NO;
  } else {
    self.touchedEndedWithoutBlocking = YES;
  }
  [super touchesEnded:touches withEvent:event];
}

- (void)touchesCancelled:(NSSet*)touches withEvent:(UIEvent*)event {
  self.state = UIGestureRecognizerStateFailed;
}
@end

@implementation ForwardingGestureRecognizer {
  // Weak reference to PlatformViewsController. The PlatformViewsController has
  // a reference to the FlutterViewController, where we can dispatch pointer events to.
  //
  // The lifecycle of PlatformViewsController is bind to FlutterEngine, which should always
  // outlives the FlutterViewController. And ForwardingGestureRecognizer is owned by a subview of
  // FlutterView, so the ForwardingGestureRecognizer never out lives FlutterViewController.
  // Therefore, `_platformViewsController` should never be nullptr.
  __weak FlutterPlatformViewsController* _platformViewsController;
  // Counting the pointers that has started in one touch sequence.
  NSInteger _currentTouchPointersCount;
  // We can't dispatch events to the framework without this back pointer.
  // This gesture recognizer retains the `FlutterViewController` until the
  // end of a gesture sequence, that is all the touches in touchesBegan are concluded
  // with |touchesCancelled| or |touchesEnded|.
  UIViewController<FlutterViewResponder>* _flutterViewController;
}

- (instancetype)initWithTarget:(id)target
       platformViewsController:(FlutterPlatformViewsController*)platformViewsController {
  self = [super initWithTarget:target action:nil];
  if (self) {
    self.delegate = self;
    FML_DCHECK(platformViewsController);
    _platformViewsController = platformViewsController;
    _currentTouchPointersCount = 0;
  }
  return self;
}

- (ForwardingGestureRecognizer*)recreateRecognizerWithTarget:(id)target {
  return [[ForwardingGestureRecognizer alloc] initWithTarget:target
                                     platformViewsController:_platformViewsController];
}

- (void)touchesBegan:(NSSet*)touches withEvent:(UIEvent*)event {
  FML_DCHECK(_currentTouchPointersCount >= 0);
  if (_currentTouchPointersCount == 0) {
    // At the start of each gesture sequence, we reset the `_flutterViewController`,
    // so that all the touch events in the same sequence are forwarded to the same
    // `_flutterViewController`.
    _flutterViewController = _platformViewsController.flutterViewController;
  }
  [_flutterViewController touchesBegan:touches withEvent:event];
  _currentTouchPointersCount += touches.count;
}

- (void)touchesMoved:(NSSet*)touches withEvent:(UIEvent*)event {
  [_flutterViewController touchesMoved:touches withEvent:event];
}

- (void)touchesEnded:(NSSet*)touches withEvent:(UIEvent*)event {
  [_flutterViewController touchesEnded:touches withEvent:event];
  _currentTouchPointersCount -= touches.count;
  // Touches in one touch sequence are sent to the touchesEnded method separately if different
  // fingers stop touching the screen at different time. So one touchesEnded method triggering does
  // not necessarially mean the touch sequence has ended. We Only set the state to
  // UIGestureRecognizerStateFailed when all the touches in the current touch sequence is ended.
  if (_currentTouchPointersCount == 0) {
    self.state = UIGestureRecognizerStateFailed;
    _flutterViewController = nil;
    [self forceResetStateIfNeeded];
  }
}

- (void)touchesCancelled:(NSSet*)touches withEvent:(UIEvent*)event {
  // In the event of platform view is removed, iOS generates a "stationary" change type instead of
  // "cancelled" change type.
  // Flutter needs all the cancelled touches to be "cancelled" change types in order to correctly
  // handle gesture sequence.
  // We always override the change type to "cancelled".
  [_flutterViewController forceTouchesCancelled:touches];
  _currentTouchPointersCount -= touches.count;
  if (_currentTouchPointersCount == 0) {
    self.state = UIGestureRecognizerStateFailed;
    _flutterViewController = nil;
    [self forceResetStateIfNeeded];
  }
}

- (void)forceResetStateIfNeeded {
  __weak ForwardingGestureRecognizer* weakSelf = self;
  dispatch_async(dispatch_get_main_queue(), ^{
    ForwardingGestureRecognizer* strongSelf = weakSelf;
    if (!strongSelf) {
      return;
    }
    if (strongSelf.state != UIGestureRecognizerStatePossible) {
      [(FlutterTouchInterceptingView*)strongSelf.view forceResetForwardingGestureRecognizerState];
    }
  });
}

- (BOOL)gestureRecognizer:(UIGestureRecognizer*)gestureRecognizer
    shouldRecognizeSimultaneouslyWithGestureRecognizer:
        (UIGestureRecognizer*)otherGestureRecognizer {
  return YES;
}
@end

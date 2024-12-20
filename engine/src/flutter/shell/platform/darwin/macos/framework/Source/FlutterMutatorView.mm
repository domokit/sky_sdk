// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterMutatorView.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterView.h"

#include <QuartzCore/QuartzCore.h>

#include "flutter/fml/logging.h"
#include "flutter/shell/platform/embedder/embedder.h"

#import "flutter/shell/platform/darwin/macos/framework/Source/NSView+ClipsToBounds.h"

namespace flutter {
PlatformViewLayer::PlatformViewLayer(const FlutterLayer* layer) {
  FML_CHECK(layer->type == kFlutterLayerContentTypePlatformView);
  const auto* platform_view = layer->platform_view;
  identifier_ = platform_view->identifier;
  for (size_t i = 0; i < platform_view->mutations_count; i++) {
    mutations_.push_back(*platform_view->mutations[i]);
  }
  offset_ = layer->offset;
  size_ = layer->size;
}
PlatformViewLayer::PlatformViewLayer(FlutterPlatformViewIdentifier identifier,
                                     const std::vector<FlutterPlatformViewMutation>& mutations,
                                     FlutterPoint offset,
                                     FlutterSize size)
    : identifier_(identifier), mutations_(mutations), offset_(offset), size_(size) {}
}  // namespace flutter

@implementation FlutterCursorCoordinator {
  __weak FlutterView* _flutterView;
  BOOL _cleanupScheduled;
  BOOL _mouseMoveHandled;
}

- (FlutterCursorCoordinator*)initWithFlutterView:(FlutterView*)flutterView {
  if (self = [super init]) {
    _flutterView = flutterView;
  }
  return self;
}

- (void)frameCleanup {
  _cleanupScheduled = NO;
  _mouseMoveHandled = NO;
}

- (BOOL)cleanupScheduled {
  return _cleanupScheduled;
}

// Processes the mouse event from given mutator view. This is called for each mutator view, in
// z-order, from the top most down.
- (void)processMouseMoveEvent:(NSEvent*)event
               forMutatorView:(FlutterMutatorView*)view
                overlayRegion:(const std::vector<CGRect>&)region {
  // [self frameCleanup] will be called once after current run loop turn.
  if (!_cleanupScheduled) {
    [[NSRunLoop mainRunLoop] performBlock:^{
      [self frameCleanup];
    }];
    _cleanupScheduled = YES;
  }

  // Mouse move was already handled by a mutator view above.
  if (_mouseMoveHandled) {
    return;
  }

  NSPoint point = [view convertPoint:event.locationInWindow fromView:nil];

  // If the mouse is above overlay region restore current Flutter cursor.
  for (const auto& r : region) {
    if (CGRectContainsPoint(r, point)) {
      [_flutterView cursorUpdate:event];
      _mouseMoveHandled = YES;
      return;
    }
  }
  NSView* platformView = view.platformView;
  // It is possible that Flutter changed mouse cursor while the mouse was inside
  // cursor rect. Unfocused NSTextField uses legacy cursor rects for changing
  // its cursor.
  [platformView.window invalidateCursorRectsForView:platformView];
  _mouseMoveHandled = YES;
}
@end

@interface FlutterMutatorView () {
  // Each of these views clips to a CGPathRef. These views, if present,
  // are nested (first is child of FlutterMutatorView and last is parent of
  // _platformView).
  NSMutableArray* _pathClipViews;

  // View right above the platform view. Used to apply the final transform
  // (sans the translation) to the platform view.
  NSView* _platformViewContainer;

  NSView* _platformView;

  FlutterCursorCoordinator* _cursorCoordinator;

  // Container view that hosts the tracking area. Must be above platform view
  // so that it gets the mouseMove event first.
  NSView* _trackingAreaContainer;

  // Tracking area used to update cursor when moving over overlay region.
  NSTrackingArea* _trackingArea;

  // Region of the overlay that should be ignored for hit testing.
  std::vector<CGRect> _hitTestIgnoreRegion;
}

@end

/// Superview container for platform views, to which sublayer transforms are applied.
@interface FlutterPlatformViewContainer : NSView
@end

@implementation FlutterPlatformViewContainer

- (NSView*)hitTest:(NSPoint)point {
  NSView* res = [super hitTest:point];
  return res != self ? res : nil;
}

- (BOOL)isFlipped {
  // Flutter transforms assume a coordinate system with an upper-left corner origin, with y
  // coordinate values increasing downwards. This affects the view, view transforms, and
  // sublayerTransforms.
  return YES;
}

@end

/// View that clips that content to a specific CGPathRef.
/// Clipping is done through a CAShapeLayer mask, which avoids the need to
/// rasterize the mask.
@interface FlutterPathClipView : NSView

@end

@implementation FlutterPathClipView

- (instancetype)initWithFrame:(NSRect)frameRect {
  if (self = [super initWithFrame:frameRect]) {
    self.wantsLayer = YES;
  }
  return self;
}

- (BOOL)isFlipped {
  // Flutter transforms assume a coordinate system with an upper-left corner origin, with y
  // coordinate values increasing downwards. This affects the view, view transforms, and
  // sublayerTransforms.
  return YES;
}

- (NSView*)hitTest:(NSPoint)point {
  NSView* res = [super hitTest:point];
  return res != self ? res : nil;
}

/// Clip the view to the given path. Offset top left corner of platform view
/// in global logical coordinates.
- (void)maskToPath:(CGPathRef)path withOrigin:(CGPoint)origin {
  CAShapeLayer* maskLayer = self.layer.mask;
  if (maskLayer == nil) {
    maskLayer = [CAShapeLayer layer];
    self.layer.mask = maskLayer;
  }
  maskLayer.path = path;
  maskLayer.transform = CATransform3DMakeTranslation(-origin.x, -origin.y, 0);
}

@end

namespace {
CATransform3D ToCATransform3D(const FlutterTransformation& t) {
  CATransform3D transform = CATransform3DIdentity;
  transform.m11 = t.scaleX;
  transform.m21 = t.skewX;
  transform.m41 = t.transX;
  transform.m14 = t.pers0;
  transform.m12 = t.skewY;
  transform.m22 = t.scaleY;
  transform.m42 = t.transY;
  transform.m24 = t.pers1;
  return transform;
}

bool AffineTransformIsOnlyScaleOrTranslate(const CGAffineTransform& transform) {
  return transform.b == 0 && transform.c == 0;
}

bool IsZeroSize(const FlutterSize size) {
  return size.width == 0 && size.height == 0;
}

CGRect FromFlutterRect(const FlutterRect& rect) {
  return CGRectMake(rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top);
}

FlutterRect ToFlutterRect(const CGRect& rect) {
  return FlutterRect{
      .left = rect.origin.x,
      .top = rect.origin.y,
      .right = rect.origin.x + rect.size.width,
      .bottom = rect.origin.y + rect.size.height,

  };
}

/// Returns whether the point is inside ellipse with given radius (centered at 0, 0).
bool PointInsideEllipse(const CGPoint& point, const FlutterSize& radius) {
  return (point.x * point.x) / (radius.width * radius.width) +
             (point.y * point.y) / (radius.height * radius.height) <
         1.0;
}

bool RoundRectCornerIntersects(const FlutterRoundedRect& roundRect, const FlutterRect& rect) {
  // Inner coordinate of the top left corner of the round rect.
  CGPoint inner_top_left =
      CGPointMake(roundRect.rect.left + roundRect.upper_left_corner_radius.width,
                  roundRect.rect.top + roundRect.upper_left_corner_radius.height);

  // Position of `rect` corner relative to inner_top_left.
  CGPoint relative_top_left =
      CGPointMake(rect.left - inner_top_left.x, rect.top - inner_top_left.y);

  // `relative_top_left` is in upper left quadrant.
  if (relative_top_left.x < 0 && relative_top_left.y < 0) {
    if (!PointInsideEllipse(relative_top_left, roundRect.upper_left_corner_radius)) {
      return true;
    }
  }

  // Inner coordinate of the top right corner of the round rect.
  CGPoint inner_top_right =
      CGPointMake(roundRect.rect.right - roundRect.upper_right_corner_radius.width,
                  roundRect.rect.top + roundRect.upper_right_corner_radius.height);

  // Positon of `rect` corner relative to inner_top_right.
  CGPoint relative_top_right =
      CGPointMake(rect.right - inner_top_right.x, rect.top - inner_top_right.y);

  // `relative_top_right` is in top right quadrant.
  if (relative_top_right.x > 0 && relative_top_right.y < 0) {
    if (!PointInsideEllipse(relative_top_right, roundRect.upper_right_corner_radius)) {
      return true;
    }
  }

  // Inner coordinate of the bottom left corner of the round rect.
  CGPoint inner_bottom_left =
      CGPointMake(roundRect.rect.left + roundRect.lower_left_corner_radius.width,
                  roundRect.rect.bottom - roundRect.lower_left_corner_radius.height);

  // Position of `rect` corner relative to inner_bottom_left.
  CGPoint relative_bottom_left =
      CGPointMake(rect.left - inner_bottom_left.x, rect.bottom - inner_bottom_left.y);

  // `relative_bottom_left` is in bottom left quadrant.
  if (relative_bottom_left.x < 0 && relative_bottom_left.y > 0) {
    if (!PointInsideEllipse(relative_bottom_left, roundRect.lower_left_corner_radius)) {
      return true;
    }
  }

  // Inner coordinate of the bottom right corner of the round rect.
  CGPoint inner_bottom_right =
      CGPointMake(roundRect.rect.right - roundRect.lower_right_corner_radius.width,
                  roundRect.rect.bottom - roundRect.lower_right_corner_radius.height);

  // Position of `rect` corner relative to inner_bottom_right.
  CGPoint relative_bottom_right =
      CGPointMake(rect.right - inner_bottom_right.x, rect.bottom - inner_bottom_right.y);

  // `relative_bottom_right` is in bottom right quadrant.
  if (relative_bottom_right.x > 0 && relative_bottom_right.y > 0) {
    if (!PointInsideEllipse(relative_bottom_right, roundRect.lower_right_corner_radius)) {
      return true;
    }
  }

  return false;
}

CGPathRef PathFromRoundedRect(const FlutterRoundedRect& roundedRect) {
  if (IsZeroSize(roundedRect.lower_left_corner_radius) &&
      IsZeroSize(roundedRect.lower_right_corner_radius) &&
      IsZeroSize(roundedRect.upper_left_corner_radius) &&
      IsZeroSize(roundedRect.upper_right_corner_radius)) {
    return CGPathCreateWithRect(FromFlutterRect(roundedRect.rect), nullptr);
  }

  CGMutablePathRef path = CGPathCreateMutable();

  const auto& rect = roundedRect.rect;
  const auto& topLeft = roundedRect.upper_left_corner_radius;
  const auto& topRight = roundedRect.upper_right_corner_radius;
  const auto& bottomLeft = roundedRect.lower_left_corner_radius;
  const auto& bottomRight = roundedRect.lower_right_corner_radius;

  CGPathMoveToPoint(path, nullptr, rect.left + topLeft.width, rect.top);
  CGPathAddLineToPoint(path, nullptr, rect.right - topRight.width, rect.top);
  CGPathAddCurveToPoint(path, nullptr, rect.right, rect.top, rect.right, rect.top + topRight.height,
                        rect.right, rect.top + topRight.height);
  CGPathAddLineToPoint(path, nullptr, rect.right, rect.bottom - bottomRight.height);
  CGPathAddCurveToPoint(path, nullptr, rect.right, rect.bottom, rect.right - bottomRight.width,
                        rect.bottom, rect.right - bottomRight.width, rect.bottom);
  CGPathAddLineToPoint(path, nullptr, rect.left + bottomLeft.width, rect.bottom);
  CGPathAddCurveToPoint(path, nullptr, rect.left, rect.bottom, rect.left,
                        rect.bottom - bottomLeft.height, rect.left,
                        rect.bottom - bottomLeft.height);
  CGPathAddLineToPoint(path, nullptr, rect.left, rect.top + topLeft.height);
  CGPathAddCurveToPoint(path, nullptr, rect.left, rect.top, rect.left + topLeft.width, rect.top,
                        rect.left + topLeft.width, rect.top);
  CGPathCloseSubpath(path);
  return path;
}

using MutationVector = std::vector<FlutterPlatformViewMutation>;

/// Returns a vector of FlutterPlatformViewMutation object pointers associated with a platform view.
/// The transforms sent from the engine include a transform from logical to physical coordinates.
/// Since Cocoa deals only in logical points, this function prepends a scale transform that scales
/// back from physical to logical coordinates to compensate.
MutationVector MutationsForPlatformView(const MutationVector& mutationsIn, float scale) {
  MutationVector mutations(mutationsIn);

  mutations.insert(mutations.begin(), {
                                          .type = kFlutterPlatformViewMutationTypeTransformation,
                                          .transformation{
                                              .scaleX = 1.0 / scale,
                                              .scaleY = 1.0 / scale,
                                          },
                                      });
  return mutations;
}

/// Returns the composition of all transformation mutations in the mutations vector.
CATransform3D CATransformFromMutations(const MutationVector& mutations) {
  CATransform3D transform = CATransform3DIdentity;
  for (auto mutation : mutations) {
    switch (mutation.type) {
      case kFlutterPlatformViewMutationTypeTransformation: {
        CATransform3D mutationTransform = ToCATransform3D(mutation.transformation);
        transform = CATransform3DConcat(mutationTransform, transform);
        break;
      }
      case kFlutterPlatformViewMutationTypeClipRect:
      case kFlutterPlatformViewMutationTypeClipRoundedRect:
      case kFlutterPlatformViewMutationTypeOpacity:
        break;
    }
  }
  return transform;
}

/// Returns the opacity for all opacity mutations in the mutations vector.
float OpacityFromMutations(const MutationVector& mutations) {
  float opacity = 1.0;
  for (auto mutation : mutations) {
    switch (mutation.type) {
      case kFlutterPlatformViewMutationTypeOpacity:
        opacity *= mutation.opacity;
        break;
      case kFlutterPlatformViewMutationTypeClipRect:
      case kFlutterPlatformViewMutationTypeClipRoundedRect:
      case kFlutterPlatformViewMutationTypeTransformation:
        break;
    }
  }
  return opacity;
}

/// Returns the clip rect generated by the intersection of clips in the mutations vector.
CGRect MasterClipFromMutations(CGRect bounds, const MutationVector& mutations) {
  // Master clip in global logical coordinates. This is intersection of all clip rectangles
  // present in mutators.
  CGRect master_clip = bounds;

  // Create the initial transform.
  CATransform3D transform = CATransform3DIdentity;
  for (auto mutation : mutations) {
    switch (mutation.type) {
      case kFlutterPlatformViewMutationTypeClipRect: {
        CGRect rect = CGRectApplyAffineTransform(FromFlutterRect(mutation.clip_rect),
                                                 CATransform3DGetAffineTransform(transform));
        master_clip = CGRectIntersection(rect, master_clip);
        break;
      }
      case kFlutterPlatformViewMutationTypeClipRoundedRect: {
        CGAffineTransform affineTransform = CATransform3DGetAffineTransform(transform);
        CGRect rect = CGRectApplyAffineTransform(FromFlutterRect(mutation.clip_rounded_rect.rect),
                                                 affineTransform);
        master_clip = CGRectIntersection(rect, master_clip);
        break;
      }
      case kFlutterPlatformViewMutationTypeTransformation:
        transform = CATransform3DConcat(ToCATransform3D(mutation.transformation), transform);
        break;
      case kFlutterPlatformViewMutationTypeOpacity:
        break;
    }
  }
  return master_clip;
}

/// A rounded rectangle and transform associated with it.
typedef struct {
  FlutterRoundedRect rrect;
  CGAffineTransform transform;
} ClipRoundedRect;

/// Returns the set of all rounded rect paths generated by clips in the mutations vector.
NSMutableArray* ClipPathFromMutations(CGRect master_clip, const MutationVector& mutations) {
  std::vector<ClipRoundedRect> rounded_rects;

  CATransform3D transform = CATransform3DIdentity;
  for (auto mutation : mutations) {
    switch (mutation.type) {
      case kFlutterPlatformViewMutationTypeClipRoundedRect: {
        CGAffineTransform affineTransform = CATransform3DGetAffineTransform(transform);
        rounded_rects.push_back({mutation.clip_rounded_rect, affineTransform});
        break;
      }
      case kFlutterPlatformViewMutationTypeTransformation:
        transform = CATransform3DConcat(ToCATransform3D(mutation.transformation), transform);
        break;
      case kFlutterPlatformViewMutationTypeClipRect: {
        CGAffineTransform affineTransform = CATransform3DGetAffineTransform(transform);
        // Shearing or rotation requires path clipping.
        if (!AffineTransformIsOnlyScaleOrTranslate(affineTransform)) {
          rounded_rects.push_back(
              {FlutterRoundedRect{mutation.clip_rect, FlutterSize{0, 0}, FlutterSize{0, 0},
                                  FlutterSize{0, 0}, FlutterSize{0, 0}},
               affineTransform});
        }
        break;
      }
      case kFlutterPlatformViewMutationTypeOpacity:
        break;
    }
  }

  NSMutableArray* paths = [NSMutableArray array];
  for (const auto& r : rounded_rects) {
    bool requiresPath = !AffineTransformIsOnlyScaleOrTranslate(r.transform);
    if (!requiresPath) {
      CGAffineTransform inverse = CGAffineTransformInvert(r.transform);
      // Transform master clip to clip rect coordinates and check if this view intersects one of the
      // corners, which means we need to use path clipping.
      CGRect localMasterClip = CGRectApplyAffineTransform(master_clip, inverse);
      requiresPath = RoundRectCornerIntersects(r.rrect, ToFlutterRect(localMasterClip));
    }

    // Only clip to rounded rectangle path if the view intersects some of the round corners. If
    // not, clipping to masterClip is enough.
    if (requiresPath) {
      CGPathRef path = PathFromRoundedRect(r.rrect);
      CGPathRef transformedPath = CGPathCreateCopyByTransformingPath(path, &r.transform);
      [paths addObject:(__bridge id)transformedPath];
      CGPathRelease(transformedPath);
      CGPathRelease(path);
    }
  }
  return paths;
}
}  // namespace

@interface FlutterTrackingAreaContainer : NSView
@end

@implementation FlutterTrackingAreaContainer
- (NSView*)hitTest:(NSPoint)point {
  return nil;
}
@end

@implementation FlutterMutatorView

- (NSView*)platformView {
  return _platformView;
}

- (NSMutableArray*)pathClipViews {
  return _pathClipViews;
}

- (NSView*)platformViewContainer {
  return _platformViewContainer;
}

- (instancetype)initWithPlatformView:(NSView*)platformView {
  return [self initWithPlatformView:platformView cursorCoordiator:nil];
}

- (instancetype)initWithPlatformView:(NSView*)platformView
                    cursorCoordiator:(FlutterCursorCoordinator*)coordinator {
  if (self = [super initWithFrame:NSZeroRect]) {
    _platformView = platformView;
    _pathClipViews = [NSMutableArray array];
    _cursorCoordinator = coordinator;
    self.wantsLayer = YES;
    self.clipsToBounds = YES;

    _trackingAreaContainer = [[FlutterTrackingAreaContainer alloc] initWithFrame:NSZeroRect];
    [self addSubview:_trackingAreaContainer];

    NSTrackingAreaOptions options = NSTrackingMouseMoved | NSTrackingInVisibleRect |
                                    NSTrackingEnabledDuringMouseDrag | NSTrackingActiveAlways;
    _trackingArea = [[NSTrackingArea alloc] initWithRect:NSZeroRect
                                                 options:options
                                                   owner:self
                                                userInfo:nil];
    [_trackingAreaContainer addTrackingArea:_trackingArea];
  }
  return self;
}

- (void)resetHitTestRegion {
  self->_hitTestIgnoreRegion.clear();
}

- (void)addHitTestIgnoreRegion:(CGRect)region {
  self->_hitTestIgnoreRegion.push_back(region);
}

- (void)mouseMoved:(NSEvent*)event {
  [_cursorCoordinator processMouseMoveEvent:event
                             forMutatorView:self
                              overlayRegion:_hitTestIgnoreRegion];
}

- (NSView*)hitTest:(NSPoint)point {
  CGPoint localPoint = point;
  localPoint.x -= self.frame.origin.x;
  localPoint.y -= self.frame.origin.y;
  for (const auto& region : _hitTestIgnoreRegion) {
    if (CGRectContainsPoint(region, localPoint)) {
      return nil;
    }
  }
  return [super hitTest:point];
}

- (BOOL)isFlipped {
  return YES;
}

/// Returns the scale factor to translate logical pixels to physical pixels for this view.
- (CGFloat)contentsScale {
  return self.superview != nil ? self.superview.layer.contentsScale : 1.0;
}

/// Updates the nested stack of clip views that host the platform view.
- (void)updatePathClipViewsWithPaths:(NSArray*)paths {
  // Remove path clip views depending on the number of paths.
  while (_pathClipViews.count > paths.count) {
    NSView* view = _pathClipViews.lastObject;
    [view removeFromSuperview];
    [_pathClipViews removeLastObject];
  }
  // Otherwise, add path clip views to the end.
  for (size_t i = _pathClipViews.count; i < paths.count; ++i) {
    NSView* superView = _pathClipViews.count == 0 ? self : _pathClipViews.lastObject;
    FlutterPathClipView* pathClipView = [[FlutterPathClipView alloc] initWithFrame:self.bounds];
    [_pathClipViews addObject:pathClipView];
    [superView addSubview:pathClipView];
  }
  // Update bounds and apply clip paths.
  for (size_t i = 0; i < _pathClipViews.count; ++i) {
    FlutterPathClipView* pathClipView = _pathClipViews[i];
    pathClipView.frame = self.bounds;
    [pathClipView maskToPath:(__bridge CGPathRef)[paths objectAtIndex:i]
                  withOrigin:self.frame.origin];
  }
}

/// Updates the PlatformView and PlatformView container views.
///
/// Re-nests _platformViewContainer in the innermost clip view, applies transforms to the underlying
/// CALayer, adds the platform view as a subview of the container, and sets the axis-aligned clip
/// rect around the tranformed view.
- (void)updatePlatformViewWithBounds:(CGRect)untransformedBounds
                   transformedBounds:(CGRect)transformedBounds
                           transform:(CATransform3D)transform
                            clipRect:(CGRect)clipRect {
  // Create the PlatformViewContainer view if necessary.
  if (_platformViewContainer == nil) {
    _platformViewContainer = [[FlutterPlatformViewContainer alloc] initWithFrame:self.bounds];
    _platformViewContainer.wantsLayer = YES;
  }

  // Nest the PlatformViewContainer view in the innermost path clip view.
  NSView* containerSuperview = _pathClipViews.count == 0 ? self : _pathClipViews.lastObject;
  [containerSuperview addSubview:_platformViewContainer];
  _platformViewContainer.frame = self.bounds;

  // Nest the platform view in the PlatformViewContainer, but only if the view doesn't have a
  // superview yet. Sometimes the platform view reparents itself (WKWebView entering FullScreen)
  // in which case do not forcefully move it back.
  if (_platformView.superview == nil) {
    [_platformViewContainer addSubview:_platformView];
  }

  // Originally first subview would be the _platformView. However during WKWebView full screen
  // the platform view gets replaced with a placeholder. Given that _platformViewContainer does
  // not contain any other views it is safe to assume that any subview found can be treated
  // as the platform view.
  _platformViewContainer.subviews.firstObject.frame = untransformedBounds;

  // Transform for the platform view is finalTransform adjusted for bounding rect origin.
  CATransform3D translation =
      CATransform3DMakeTranslation(-transformedBounds.origin.x, -transformedBounds.origin.y, 0);
  transform = CATransform3DConcat(transform, translation);
  _platformViewContainer.layer.sublayerTransform = transform;

  // By default NSView clips children to frame. If masterClip is tighter than mutator view frame,
  // the frame is set to masterClip and child offset adjusted to compensate for the difference.
  if (!CGRectEqualToRect(clipRect, transformedBounds)) {
    NSMutableArray<NSView*>* subviews = [NSMutableArray arrayWithArray:self.subviews];
    [subviews removeObject:_trackingAreaContainer];
    FML_DCHECK(subviews.count == 1);
    auto subview = subviews.firstObject;
    FML_DCHECK(subview.frame.origin.x == 0 && subview.frame.origin.y == 0);
    subview.frame = CGRectMake(transformedBounds.origin.x - clipRect.origin.x,
                               transformedBounds.origin.y - clipRect.origin.y,
                               subview.frame.size.width, subview.frame.size.height);
    self.frame = clipRect;
  }
}

/// Whenever possible view will be clipped using layer bounds.
/// If clipping to path is needed, CAShapeLayer(s) will be used as mask.
/// Clipping to round rect only clips to path if round corners are intersected.
- (void)applyFlutterLayer:(const flutter::PlatformViewLayer*)layer {
  // Compute the untransformed bounding rect for the platform view in logical pixels.
  // FlutterLayer.size is in physical pixels but Cocoa uses logical points.
  CGFloat scale = [self contentsScale];
  MutationVector mutations = MutationsForPlatformView(layer->mutations(), scale);

  CATransform3D finalTransform = CATransformFromMutations(mutations);

  // Compute the untransformed bounding rect for the platform view in logical pixels.
  // FlutterLayer.size is in physical pixels but Cocoa uses logical points.
  CGRect untransformedBoundingRect =
      CGRectMake(0, 0, layer->size().width / scale, layer->size().height / scale);
  CGRect finalBoundingRect = CGRectApplyAffineTransform(
      untransformedBoundingRect, CATransform3DGetAffineTransform(finalTransform));
  self.frame = finalBoundingRect;

  // Compute the layer opacity.
  self.layer.opacity = OpacityFromMutations(mutations);

  // Compute the master clip in global logical coordinates.
  CGRect masterClip = MasterClipFromMutations(finalBoundingRect, mutations);
  if (CGRectIsNull(masterClip)) {
    self.hidden = YES;
    return;
  }
  self.hidden = NO;

  /// Paths in global logical coordinates that need to be clipped to.
  NSMutableArray* paths = ClipPathFromMutations(masterClip, mutations);
  [self updatePathClipViewsWithPaths:paths];

  /// Update PlatformViewContainer, PlatformView, and apply transforms and axis-aligned clip rect.
  [self updatePlatformViewWithBounds:untransformedBoundingRect
                   transformedBounds:finalBoundingRect
                           transform:finalTransform
                            clipRect:masterClip];

  [self addSubview:_trackingAreaContainer positioned:(NSWindowAbove)relativeTo:nil];
  _trackingAreaContainer.frame = self.bounds;
}

@end

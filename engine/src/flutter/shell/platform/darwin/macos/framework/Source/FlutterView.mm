// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterView.h"

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterResizeSynchronizer.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterSurfaceManager.h"

#import <QuartzCore/QuartzCore.h>

@interface FlutterView () <FlutterSurfaceManagerDelegate> {
  FlutterViewIdentifier _viewIdentifier;
  __weak id<FlutterViewDelegate> _viewDelegate;
  FlutterResizeSynchronizer* _resizeSynchronizer;
  FlutterSurfaceManager* _surfaceManager;
  NSCursor* _lastCursor;
}

@end

@implementation FlutterView

- (instancetype)initWithMTLDevice:(id<MTLDevice>)device
                     commandQueue:(id<MTLCommandQueue>)commandQueue
                         delegate:(id<FlutterViewDelegate>)delegate
                   viewIdentifier:(FlutterViewIdentifier)viewIdentifier {
  self = [super initWithFrame:NSZeroRect];
  if (self) {
    [self setWantsLayer:YES];
    [self setBackgroundColor:[NSColor blackColor]];
    [self setLayerContentsRedrawPolicy:NSViewLayerContentsRedrawDuringViewResize];
    _viewIdentifier = viewIdentifier;
    _viewDelegate = delegate;
    _surfaceManager = [[FlutterSurfaceManager alloc] initWithDevice:device
                                                       commandQueue:commandQueue
                                                              layer:self.layer
                                                           delegate:self];
    _resizeSynchronizer = [[FlutterResizeSynchronizer alloc] init];
  }
  return self;
}

- (void)onPresent:(CGSize)frameSize withBlock:(dispatch_block_t)block delay:(NSTimeInterval)delay {
  [_resizeSynchronizer performCommitForSize:frameSize notify:block delay:delay];
}

- (FlutterSurfaceManager*)surfaceManager {
  return _surfaceManager;
}

- (void)shutDown {
  [_resizeSynchronizer shutDown];
}

- (void)setBackgroundColor:(NSColor*)color {
  self.layer.backgroundColor = color.CGColor;
}

#pragma mark - NSView overrides

- (void)setFrameSize:(NSSize)newSize {
  [super setFrameSize:newSize];
  CGSize scaledSize = [self convertSizeToBacking:self.bounds.size];
  [_resizeSynchronizer beginResizeForSize:scaledSize
                                   notify:^{
                                     [_viewDelegate viewDidReshape:self];
                                   }];
}

/**
 * Declares that the view uses a flipped coordinate system, consistent with Flutter conventions.
 */
- (BOOL)isFlipped {
  return YES;
}

- (BOOL)isOpaque {
  return YES;
}

/**
 * Declares that the initial mouse-down when the view is not in focus will send an event to the
 * view.
 */
- (BOOL)acceptsFirstMouse:(NSEvent*)event {
  return YES;
}

- (BOOL)acceptsFirstResponder {
  // This is to ensure that FlutterView does not take first responder status from TextInputPlugin
  // on mouse clicks.
  return [_viewDelegate viewShouldAcceptFirstResponder:self];
}

- (void)didUpdateMouseCursor:(NSCursor*)cursor {
  _lastCursor = cursor;
}

// Restores mouse cursor. There are few cases when this is needed and framework will not handle
// this automatically:
// - When mouse cursor leaves subview of FlutterView (technically still within bound of
// FlutterView tracking area so the framework won't be notified)
// - When context menu above FlutterView is closed. Context menu will change current cursor to
// arrow and will not restore it back.
- (void)cursorUpdate:(NSEvent*)event {
  // Make sure to not override cursor when over a platform view.
  NSPoint mouseLocation = [[self superview] convertPoint:event.locationInWindow fromView:nil];
  NSView* hitTestView = [self hitTest:mouseLocation];
  if (hitTestView != self) {
    return;
  }
  [_lastCursor set];
  // It is possible that there is a platform view with NSTrackingArea below flutter content.
  // This could override the mouse cursor as a result of mouse move event. There is no good way
  // to prevent that short of swizzling [NSCursor set], so as a workaround force flutter cursor
  // in next runloop turn. This is not ideal, as it may cause the cursor flicker a bit.
  [[NSRunLoop currentRunLoop] performBlock:^{
    [_lastCursor set];
  }];
}

- (void)viewDidChangeBackingProperties {
  [super viewDidChangeBackingProperties];
  // Force redraw
  [_viewDelegate viewDidReshape:self];
}

- (BOOL)layer:(CALayer*)layer
    shouldInheritContentsScale:(CGFloat)newScale
                    fromWindow:(NSWindow*)window {
  return YES;
}

#pragma mark - NSAccessibility overrides

- (BOOL)isAccessibilityElement {
  return YES;
}

- (NSAccessibilityRole)accessibilityRole {
  return NSAccessibilityGroupRole;
}

- (NSString*)accessibilityLabel {
  // TODO(chunhtai): Provides a way to let developer customize the accessibility
  // label.
  // https://github.com/flutter/flutter/issues/75446
  NSString* applicationName =
      [NSBundle.mainBundle objectForInfoDictionaryKey:@"CFBundleDisplayName"];
  if (!applicationName) {
    applicationName = [NSBundle.mainBundle objectForInfoDictionaryKey:@"CFBundleName"];
  }
  return applicationName;
}

@end

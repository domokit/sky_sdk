// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <objc/message.h>

#import "FlutterMouseCursorPlugin.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterCodecs.h"

static NSString* const kMouseCursorChannel = @"flutter/mousecursor";

static NSString* const kActivateSystemCursorMethod = @"activateSystemCursor";
static NSString* const kKindKey = @"kind";

static NSString* const kKindValueNone = @"none";

static NSDictionary* systemCursors;

/**
 * Maps a Flutter's constant to a platform's cursor object.
 *
 * Returns the arrow cursor for unknown constants, including kSystemShapeNone.
 */
static NSCursor* GetCursorForKind(NSString* kind) {
  // The following mapping must be kept in sync with Flutter framework's
  // mouse_cursor.dart

  if ([kind isEqualToString:kKindValueNone]) {
    NSImage* image = [[NSImage alloc] initWithSize:NSMakeSize(1, 1)];
    return [[NSCursor alloc] initWithImage:image hotSpot:NSMakePoint(0, 0)];
  }

  if (systemCursors == nil) {
    systemCursors = @{
      @"alias" : [NSCursor dragLinkCursor],
      @"basic" : [NSCursor arrowCursor],
      @"click" : [NSCursor pointingHandCursor],
      @"contextMenu" : [NSCursor contextualMenuCursor],
      @"copy" : [NSCursor dragCopyCursor],
      @"disappearing" : [NSCursor disappearingItemCursor],
      @"forbidden" : [NSCursor operationNotAllowedCursor],
      @"grab" : [NSCursor openHandCursor],
      @"grabbing" : [NSCursor closedHandCursor],
      @"noDrop" : [NSCursor operationNotAllowedCursor],
      @"precise" : [NSCursor crosshairCursor],
      @"text" : [NSCursor IBeamCursor],
      @"resizeColumn" : [NSCursor resizeLeftRightCursor],
      @"resizeDown" : [NSCursor resizeDownCursor],
      @"resizeLeft" : [NSCursor resizeLeftCursor],
      @"resizeLeftRight" : [NSCursor resizeLeftRightCursor],
      @"resizeRight" : [NSCursor resizeRightCursor],
      @"resizeRow" : [NSCursor resizeUpDownCursor],
      @"resizeUp" : [NSCursor resizeUpCursor],
      @"resizeUpDown" : [NSCursor resizeUpDownCursor],
      @"verticalText" : [NSCursor IBeamCursorForVerticalLayout],
    };
  }
  NSCursor* result = [systemCursors objectForKey:kind];
  if (result == nil) {
    return [NSCursor arrowCursor];
  }
  return result;
}

@interface FlutterMouseCursorPlugin ()

@property(nonatomic, weak) id<FlutterMouseCursorPluginDelegate> delegate;

/**
 * Handles the method call that activates a system cursor.
 *
 * Returns a FlutterError if the arguments can not be recognized. Otherwise
 * returns nil.
 */
- (FlutterError*)activateSystemCursor:(nonnull NSDictionary*)arguments;

/**
 * Displays the specified cursor.
 *
 * Unhides the cursor before displaying the cursor, and updates
 * internal states.
 */
- (void)displayCursorObject:(nonnull NSCursor*)cursorObject;

/**
 * Handles all method calls from Flutter.
 */
- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;

@end

@implementation FlutterMouseCursorPlugin

#pragma mark - Private

NSMutableDictionary* cachedSystemCursors;

- (instancetype)init {
  self = [super init];
  if (self) {
    cachedSystemCursors = [NSMutableDictionary dictionary];
  }
  return self;
}

- (FlutterError*)activateSystemCursor:(nonnull NSDictionary*)arguments {
  NSString* kindArg = arguments[kKindKey];
  if (!kindArg) {
    return [FlutterError errorWithCode:@"error"
                               message:@"Missing argument"
                               details:@"Missing argument while trying to activate system cursor"];
  }

  NSCursor* cursorObject = [FlutterMouseCursorPlugin cursorFromKind:kindArg];
  [self displayCursorObject:cursorObject];
  return nil;
}

- (void)displayCursorObject:(nonnull NSCursor*)cursorObject {
  [cursorObject set];
  [self.delegate didUpdateMouseCursor:cursorObject];
}

+ (NSCursor*)cursorFromKind:(NSString*)kind {
  NSCursor* cachedValue = [cachedSystemCursors objectForKey:kind];
  if (!cachedValue) {
    cachedValue = GetCursorForKind(kind);
    [cachedSystemCursors setValue:cachedValue forKey:kind];
  }
  return cachedValue;
}

#pragma mark - FlutterPlugin implementation

+ (void)registerWithRegistrar:(id<FlutterPluginRegistrar>)registrar {
  [self registerWithRegistrar:registrar delegate:nil];
}

+ (void)registerWithRegistrar:(id<FlutterPluginRegistrar>)registrar
                     delegate:(id<FlutterMouseCursorPluginDelegate>)delegate {
  FlutterMethodChannel* channel = [FlutterMethodChannel methodChannelWithName:kMouseCursorChannel
                                                              binaryMessenger:registrar.messenger];
  FlutterMouseCursorPlugin* instance = [[FlutterMouseCursorPlugin alloc] init];
  instance.delegate = delegate;
  [registrar addMethodCallDelegate:instance channel:channel];
}

- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result {
  NSString* method = call.method;
  if ([method isEqualToString:kActivateSystemCursorMethod]) {
    result([self activateSystemCursor:call.arguments]);
  } else {
    result(FlutterMethodNotImplemented);
  }
}

@end

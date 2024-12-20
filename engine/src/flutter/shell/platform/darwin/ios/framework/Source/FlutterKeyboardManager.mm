// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterKeyboardManager.h"

#include "flutter/fml/platform/darwin/message_loop_darwin.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

FLUTTER_ASSERT_ARC

static constexpr CFTimeInterval kDistantFuture = 1.0e10;

@interface FlutterKeyboardManager ()

/**
 * The primary responders added by addPrimaryResponder.
 */
@property(nonatomic, copy, readonly)
    NSMutableArray<id<FlutterKeyPrimaryResponder>>* primaryResponders;

/**
 * The secondary responders added by addSecondaryResponder.
 */
@property(nonatomic, copy, readonly)
    NSMutableArray<id<FlutterKeySecondaryResponder>>* secondaryResponders;

- (void)dispatchToSecondaryResponders:(nonnull FlutterUIPressProxy*)press
                             complete:(nonnull KeyEventCompleteCallback)callback
    API_AVAILABLE(ios(13.4));

@end

@implementation FlutterKeyboardManager

- (nonnull instancetype)init {
  self = [super init];
  if (self != nil) {
    _primaryResponders = [[NSMutableArray alloc] init];
    _secondaryResponders = [[NSMutableArray alloc] init];
  }
  return self;
}

- (void)addPrimaryResponder:(nonnull id<FlutterKeyPrimaryResponder>)responder {
  [_primaryResponders addObject:responder];
}

- (void)addSecondaryResponder:(nonnull id<FlutterKeySecondaryResponder>)responder {
  [_secondaryResponders addObject:responder];
}

- (void)handlePress:(nonnull FlutterUIPressProxy*)press
         nextAction:(nonnull void (^)())next API_AVAILABLE(ios(13.4)) {
  if (@available(iOS 13.4, *)) {
    // no-op
  } else {
    return;
  }

  bool __block wasHandled = false;
  KeyEventCompleteCallback completeCallback = ^void(bool handled, FlutterUIPressProxy* press) {
    wasHandled = handled;
    CFRunLoopStop(CFRunLoopGetCurrent());
  };
  switch (press.phase) {
    case UIPressPhaseBegan:
    case UIPressPhaseEnded: {
      // Having no primary responders requires extra logic, but Flutter hard-codes
      // all primary responders, so this is a situation that Flutter will never
      // encounter.
      NSAssert([_primaryResponders count] >= 0, @"At least one primary responder must be added.");

      __block __weak __typeof(self) weakSelf = self;
      __block NSUInteger unreplied = [self.primaryResponders count];
      __block BOOL anyHandled = false;
      FlutterAsyncKeyCallback replyCallback = ^(BOOL handled) {
        unreplied--;
        NSAssert(unreplied >= 0, @"More primary responders replied than expected.");
        anyHandled = anyHandled || handled;
        if (unreplied == 0) {
          if (!anyHandled && weakSelf) {
            [weakSelf dispatchToSecondaryResponders:press complete:completeCallback];
          } else {
            completeCallback(true, press);
          }
        }
      };
      for (id<FlutterKeyPrimaryResponder> responder in _primaryResponders) {
        [responder handlePress:press callback:replyCallback];
      }
      // Create a nested run loop while we wait for a response from the
      // framework. Once the completeCallback is called, this run loop will exit
      // and the main one will resume. The completeCallback MUST be called, or
      // the app will get stuck in this run loop indefinitely.
      //
      // We need to run in this mode so that UIKit doesn't give us new
      // events until we are done processing this one.
      CFRunLoopRunInMode(fml::MessageLoopDarwin::kMessageLoopCFRunLoopMode, kDistantFuture, NO);
      break;
    }
    case UIPressPhaseChanged:
    case UIPressPhaseCancelled:
    case UIPressPhaseStationary:
      break;
  }
  if (!wasHandled) {
    next();
  }
}

#pragma mark - Private

- (void)dispatchToSecondaryResponders:(nonnull FlutterUIPressProxy*)press
                             complete:(nonnull KeyEventCompleteCallback)callback
    API_AVAILABLE(ios(13.4)) {
  if (@available(iOS 13.4, *)) {
    // no-op
  } else {
    callback(false, press);
    return;
  }

  for (id<FlutterKeySecondaryResponder> responder in _secondaryResponders) {
    if ([responder handlePress:press]) {
      callback(true, press);
      return;
    }
  }
  callback(false, press);
}

@end

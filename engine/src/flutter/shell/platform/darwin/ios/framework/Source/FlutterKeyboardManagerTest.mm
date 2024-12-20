// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Foundation/Foundation.h>
#import <OCMock/OCMock.h>
#import <UIKit/UIKit.h>
#import <XCTest/XCTest.h>
#include <_types/_uint32_t.h>

#include "flutter/fml/platform/darwin/message_loop_darwin.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterFakeKeyEvents.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterKeyboardManager.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUIPressProxy.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewController_Internal.h"

FLUTTER_ASSERT_ARC;

namespace flutter {
class PointerDataPacket {};
}  // namespace flutter

using namespace flutter::testing;

namespace {

typedef void (^KeyCallbackSetter)(FlutterUIPressProxy* press, FlutterAsyncKeyCallback callback)
    API_AVAILABLE(ios(13.4));
typedef BOOL (^BoolGetter)();

}  // namespace

// These tests were designed to run on iOS 13.4 or later.
API_AVAILABLE(ios(13.4))
@interface FlutterKeyboardManagerTest : XCTestCase
@end

@implementation FlutterKeyboardManagerTest

- (id<FlutterKeyPrimaryResponder>)mockPrimaryResponder:(KeyCallbackSetter)callbackSetter {
  id<FlutterKeyPrimaryResponder> mock =
      OCMStrictProtocolMock(@protocol(FlutterKeyPrimaryResponder));
  OCMStub([mock handlePress:[OCMArg any] callback:[OCMArg any]])
      .andDo((^(NSInvocation* invocation) {
        __unsafe_unretained FlutterUIPressProxy* pressUnsafe;
        __unsafe_unretained FlutterAsyncKeyCallback callbackUnsafe;

        [invocation getArgument:&pressUnsafe atIndex:2];
        [invocation getArgument:&callbackUnsafe atIndex:3];

        // Retain the unretained parameters so they can
        // be run in the perform block when this invocation goes out of scope.
        FlutterUIPressProxy* press = pressUnsafe;
        FlutterAsyncKeyCallback callback = callbackUnsafe;
        CFRunLoopPerformBlock(CFRunLoopGetCurrent(),
                              fml::MessageLoopDarwin::kMessageLoopCFRunLoopMode, ^() {
                                callbackSetter(press, callback);
                              });
      }));
  return mock;
}

- (id<FlutterKeySecondaryResponder>)mockSecondaryResponder:(BoolGetter)resultGetter {
  id<FlutterKeySecondaryResponder> mock =
      OCMStrictProtocolMock(@protocol(FlutterKeySecondaryResponder));
  OCMStub([mock handlePress:[OCMArg any]]).andDo((^(NSInvocation* invocation) {
    BOOL result = resultGetter();
    [invocation setReturnValue:&result];
  }));
  return mock;
}

- (void)testNextResponderShouldThrowOnPressesEnded {
  // The nextResponder is a strict mock and hasn't stubbed pressesEnded.
  // An error will be thrown on pressesEnded.
  UIResponder* nextResponder = OCMStrictClassMock([UIResponder class]);
  OCMStub([nextResponder pressesBegan:OCMOCK_ANY withEvent:OCMOCK_ANY]);

  id mockEngine = OCMClassMock([FlutterEngine class]);
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:mockEngine
                                                                                nibName:nil
                                                                                 bundle:nil];
  FlutterViewController* owner = OCMPartialMock(viewController);
  OCMStub([owner nextResponder]).andReturn(nextResponder);

  XCTAssertThrowsSpecificNamed([owner.nextResponder pressesEnded:[[NSSet alloc] init]
                                                       withEvent:[[UIPressesEvent alloc] init]],
                               NSException, NSInternalInconsistencyException);

  [mockEngine stopMocking];
}

- (void)testSinglePrimaryResponder {
  FlutterKeyboardManager* manager = [[FlutterKeyboardManager alloc] init];
  __block BOOL primaryResponse = FALSE;
  __block int callbackCount = 0;
  [manager addPrimaryResponder:[self mockPrimaryResponder:^(FlutterUIPressProxy* press,
                                                            FlutterAsyncKeyCallback callback) {
             callbackCount++;
             callback(primaryResponse);
           }]];
  constexpr UIKeyboardHIDUsage keyId = (UIKeyboardHIDUsage)0x50;
  // Case: The responder reports TRUE
  __block bool completeHandled = true;
  primaryResponse = TRUE;
  [manager handlePress:keyDownEvent(keyId)
            nextAction:^() {
              completeHandled = false;
            }];
  XCTAssertEqual(callbackCount, 1);
  XCTAssertTrue(completeHandled);
  completeHandled = true;
  callbackCount = 0;

  // Case: The responder reports FALSE
  primaryResponse = FALSE;
  [manager handlePress:keyUpEvent(keyId)
            nextAction:^() {
              completeHandled = false;
            }];
  XCTAssertEqual(callbackCount, 1);
  XCTAssertFalse(completeHandled);
}

- (void)testDoublePrimaryResponder {
  FlutterKeyboardManager* manager = [[FlutterKeyboardManager alloc] init];

  __block BOOL callback1Response = FALSE;
  __block int callback1Count = 0;
  [manager addPrimaryResponder:[self mockPrimaryResponder:^(FlutterUIPressProxy* press,
                                                            FlutterAsyncKeyCallback callback) {
             callback1Count++;
             callback(callback1Response);
           }]];

  __block BOOL callback2Response = FALSE;
  __block int callback2Count = 0;
  [manager addPrimaryResponder:[self mockPrimaryResponder:^(FlutterUIPressProxy* press,
                                                            FlutterAsyncKeyCallback callback) {
             callback2Count++;
             callback(callback2Response);
           }]];

  // Case: Both responders report TRUE.
  __block bool somethingWasHandled = true;
  constexpr UIKeyboardHIDUsage keyId = (UIKeyboardHIDUsage)0x50;
  callback1Response = TRUE;
  callback2Response = TRUE;
  [manager handlePress:keyUpEvent(keyId)
            nextAction:^() {
              somethingWasHandled = false;
            }];
  XCTAssertEqual(callback1Count, 1);
  XCTAssertEqual(callback2Count, 1);
  XCTAssertTrue(somethingWasHandled);

  somethingWasHandled = true;
  callback1Count = 0;
  callback2Count = 0;

  // Case: One responder reports TRUE.
  callback1Response = TRUE;
  callback2Response = FALSE;
  [manager handlePress:keyUpEvent(keyId)
            nextAction:^() {
              somethingWasHandled = false;
            }];
  XCTAssertEqual(callback1Count, 1);
  XCTAssertEqual(callback2Count, 1);
  XCTAssertTrue(somethingWasHandled);

  somethingWasHandled = true;
  callback1Count = 0;
  callback2Count = 0;

  // Case: Both responders report FALSE.
  callback1Response = FALSE;
  callback2Response = FALSE;
  [manager handlePress:keyDownEvent(keyId)
            nextAction:^() {
              somethingWasHandled = false;
            }];
  XCTAssertEqual(callback1Count, 1);
  XCTAssertEqual(callback2Count, 1);
  XCTAssertFalse(somethingWasHandled);
}

- (void)testSingleSecondaryResponder {
  FlutterKeyboardManager* manager = [[FlutterKeyboardManager alloc] init];

  __block BOOL primaryResponse = FALSE;
  __block int callbackCount = 0;
  [manager addPrimaryResponder:[self mockPrimaryResponder:^(FlutterUIPressProxy* press,
                                                            FlutterAsyncKeyCallback callback) {
             callbackCount++;
             callback(primaryResponse);
           }]];

  __block BOOL secondaryResponse;
  [manager addSecondaryResponder:[self mockSecondaryResponder:^() {
             return secondaryResponse;
           }]];

  // Case: Primary responder responds TRUE. The event shouldn't be handled by
  // the secondary responder.
  constexpr UIKeyboardHIDUsage keyId = (UIKeyboardHIDUsage)0x50;
  secondaryResponse = FALSE;
  primaryResponse = TRUE;
  __block bool completeHandled = true;
  [manager handlePress:keyUpEvent(keyId)
            nextAction:^() {
              completeHandled = false;
            }];
  XCTAssertEqual(callbackCount, 1);
  XCTAssertTrue(completeHandled);
  completeHandled = true;
  callbackCount = 0;

  // Case: Primary responder responds FALSE. The secondary responder returns
  // TRUE.
  secondaryResponse = TRUE;
  primaryResponse = FALSE;
  [manager handlePress:keyUpEvent(keyId)
            nextAction:^() {
              completeHandled = false;
            }];
  XCTAssertEqual(callbackCount, 1);
  XCTAssertTrue(completeHandled);
  completeHandled = true;
  callbackCount = 0;

  // Case: Primary responder responds FALSE. The secondary responder returns FALSE.
  secondaryResponse = FALSE;
  primaryResponse = FALSE;
  [manager handlePress:keyDownEvent(keyId)
            nextAction:^() {
              completeHandled = false;
            }];
  XCTAssertEqual(callbackCount, 1);
  XCTAssertFalse(completeHandled);
}

- (void)testEventsProcessedSequentially {
  constexpr UIKeyboardHIDUsage keyId1 = (UIKeyboardHIDUsage)0x50;
  constexpr UIKeyboardHIDUsage keyId2 = (UIKeyboardHIDUsage)0x51;
  FlutterUIPressProxy* event1 = keyDownEvent(keyId1);
  FlutterUIPressProxy* event2 = keyDownEvent(keyId2);
  __block FlutterAsyncKeyCallback key1Callback;
  __block FlutterAsyncKeyCallback key2Callback;
  __block bool key1Handled = true;
  __block bool key2Handled = true;

  FlutterKeyboardManager* manager = [[FlutterKeyboardManager alloc] init];
  [manager addPrimaryResponder:[self mockPrimaryResponder:^(FlutterUIPressProxy* press,
                                                            FlutterAsyncKeyCallback callback) {
             if (press == event1) {
               key1Callback = callback;
             } else if (press == event2) {
               key2Callback = callback;
             }
           }]];

  // Add both presses into the main CFRunLoop queue
  CFRunLoopTimerRef timer0 = CFRunLoopTimerCreateWithHandler(
      kCFAllocatorDefault, CFAbsoluteTimeGetCurrent(), 0, 0, 0, ^(CFRunLoopTimerRef timerRef) {
        [manager handlePress:event1
                  nextAction:^() {
                    key1Handled = false;
                  }];
      });
  CFRunLoopAddTimer(CFRunLoopGetCurrent(), timer0, kCFRunLoopCommonModes);
  CFRunLoopTimerRef timer1 = CFRunLoopTimerCreateWithHandler(
      kCFAllocatorDefault, CFAbsoluteTimeGetCurrent() + 1, 0, 0, 0, ^(CFRunLoopTimerRef timerRef) {
        // key1 should be completely finished by now
        XCTAssertFalse(key1Handled);
        [manager handlePress:event2
                  nextAction:^() {
                    key2Handled = false;
                  }];
        // End the nested CFRunLoop
        CFRunLoopStop(CFRunLoopGetCurrent());
      });
  CFRunLoopAddTimer(CFRunLoopGetCurrent(), timer1, kCFRunLoopCommonModes);

  // Add the callbacks to the CFRunLoop with mode kMessageLoopCFRunLoopMode
  // This allows them to interrupt the loop started within handlePress
  CFRunLoopTimerRef timer2 = CFRunLoopTimerCreateWithHandler(
      kCFAllocatorDefault, CFAbsoluteTimeGetCurrent() + 2, 0, 0, 0, ^(CFRunLoopTimerRef timerRef) {
        // No processing should be done on key2 yet
        XCTAssertTrue(key1Callback != nil);
        XCTAssertTrue(key2Callback == nil);
        key1Callback(false);
      });
  CFRunLoopAddTimer(CFRunLoopGetCurrent(), timer2,
                    fml::MessageLoopDarwin::kMessageLoopCFRunLoopMode);
  CFRunLoopTimerRef timer3 = CFRunLoopTimerCreateWithHandler(
      kCFAllocatorDefault, CFAbsoluteTimeGetCurrent() + 3, 0, 0, 0, ^(CFRunLoopTimerRef timerRef) {
        // Both keys should be processed by now
        XCTAssertTrue(key1Callback != nil);
        XCTAssertTrue(key2Callback != nil);
        key2Callback(false);
      });
  CFRunLoopAddTimer(CFRunLoopGetCurrent(), timer3,
                    fml::MessageLoopDarwin::kMessageLoopCFRunLoopMode);

  // Start a nested CFRunLoop so we can wait for both presses to complete before exiting the test
  CFRunLoopRun();
  XCTAssertFalse(key2Handled);
  XCTAssertFalse(key1Handled);
}

@end

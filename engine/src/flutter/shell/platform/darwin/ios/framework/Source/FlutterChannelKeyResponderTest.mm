// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Foundation/Foundation.h>
#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterChannelKeyResponder.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterFakeKeyEvents.h"

FLUTTER_ASSERT_ARC;

#define XCTAssertStrEqual(value, expected)        \
  XCTAssertTrue([value isEqualToString:expected], \
                @"String \"%@\" not equal to the expected value of \"%@\"", value, expected)

using namespace flutter::testing;

API_AVAILABLE(ios(13.4))
@interface FlutterChannelKeyResponderTest : XCTestCase
@property(copy, nonatomic) FlutterUIPressProxy* testKeyDownEvent API_AVAILABLE(ios(13.4));
@property(copy, nonatomic) FlutterUIPressProxy* testKeyUpEvent API_AVAILABLE(ios(13.4));
@end

@implementation FlutterChannelKeyResponderTest

- (void)setUp {
  // All of these tests were designed to run on iOS 13.4 or later.
  if (@available(iOS 13.4, *)) {
  } else {
    XCTSkip(@"Required API not present for test.");
  }
  _testKeyDownEvent = keyDownEvent(UIKeyboardHIDUsageKeyboardA, 0x0, 0.0f, "a", "a");
  _testKeyUpEvent = keyUpEvent(UIKeyboardHIDUsageKeyboardA, 0x0, 0.0f);
}

- (void)tearDown API_AVAILABLE(ios(13.4)) {
  _testKeyDownEvent = nil;
  _testKeyUpEvent = nil;
}

- (void)testBasicKeyEvent API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<id>* messages = [[NSMutableArray<id> alloc] init];
  __block BOOL next_response = TRUE;
  __block NSMutableArray<NSNumber*>* responses = [[NSMutableArray<NSNumber*> alloc] init];

  id mockKeyEventChannel = OCMStrictClassMock([FlutterBasicMessageChannel class]);
  OCMStub([mockKeyEventChannel sendMessage:[OCMArg any] reply:[OCMArg any]])
      .andDo((^(NSInvocation* invocation) {
        [invocation retainArguments];
        NSDictionary* message;
        [invocation getArgument:&message atIndex:2];
        [messages addObject:message];

        FlutterReply callback;
        [invocation getArgument:&callback atIndex:3];
        NSDictionary* keyMessage = @{
          @"handled" : @(next_response),
        };
        callback(keyMessage);
      }));

  // Key down
  FlutterChannelKeyResponder* responder =
      [[FlutterChannelKeyResponder alloc] initWithChannel:mockKeyEventChannel];
  [responder handlePress:_testKeyDownEvent
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  XCTAssertEqual([messages count], 1u);
  XCTAssertStrEqual([messages lastObject][@"keymap"], @"ios");
  XCTAssertStrEqual([messages lastObject][@"type"], @"keydown");
  XCTAssertEqual([[messages lastObject][@"keyCode"] intValue], UIKeyboardHIDUsageKeyboardA);
  XCTAssertEqual([[messages lastObject][@"modifiers"] intValue], 0x0);
  XCTAssertStrEqual([messages lastObject][@"characters"], @"a");
  XCTAssertStrEqual([messages lastObject][@"charactersIgnoringModifiers"], @"a");

  XCTAssertEqual([responses count], 1u);
  XCTAssertEqual([[responses lastObject] boolValue], TRUE);

  [messages removeAllObjects];
  [responses removeAllObjects];

  // Key up
  next_response = FALSE;
  [responder handlePress:_testKeyUpEvent
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  XCTAssertEqual([messages count], 1u);
  XCTAssertStrEqual([messages lastObject][@"keymap"], @"ios");
  XCTAssertStrEqual([messages lastObject][@"type"], @"keyup");
  XCTAssertEqual([[messages lastObject][@"keyCode"] intValue], UIKeyboardHIDUsageKeyboardA);
  XCTAssertEqual([[messages lastObject][@"modifiers"] intValue], 0x0);

  XCTAssertEqual([responses count], 1u);
  XCTAssertEqual([[responses lastObject] boolValue], FALSE);

  [messages removeAllObjects];
  [responses removeAllObjects];
}

- (void)testEmptyResponseIsTakenAsHandled API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<id>* messages = [[NSMutableArray<id> alloc] init];
  __block NSMutableArray<NSNumber*>* responses = [[NSMutableArray<NSNumber*> alloc] init];

  id mockKeyEventChannel = OCMStrictClassMock([FlutterBasicMessageChannel class]);
  OCMStub([mockKeyEventChannel sendMessage:[OCMArg any] reply:[OCMArg any]])
      .andDo((^(NSInvocation* invocation) {
        [invocation retainArguments];
        NSDictionary* message;
        [invocation getArgument:&message atIndex:2];
        [messages addObject:message];

        FlutterReply callback;
        [invocation getArgument:&callback atIndex:3];
        callback(nullptr);
      }));

  FlutterChannelKeyResponder* responder =
      [[FlutterChannelKeyResponder alloc] initWithChannel:mockKeyEventChannel];
  [responder handlePress:_testKeyDownEvent
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  XCTAssertEqual([messages count], 1u);
  XCTAssertStrEqual([messages lastObject][@"keymap"], @"ios");
  XCTAssertStrEqual([messages lastObject][@"type"], @"keydown");
  XCTAssertEqual([[messages lastObject][@"keyCode"] intValue], UIKeyboardHIDUsageKeyboardA);
  XCTAssertEqual([[messages lastObject][@"modifiers"] intValue], 0x0);
  XCTAssertStrEqual([messages lastObject][@"characters"], @"a");
  XCTAssertStrEqual([messages lastObject][@"charactersIgnoringModifiers"], @"a");

  XCTAssertEqual([responses count], 1u);
  XCTAssertEqual([[responses lastObject] boolValue], TRUE);
}

@end

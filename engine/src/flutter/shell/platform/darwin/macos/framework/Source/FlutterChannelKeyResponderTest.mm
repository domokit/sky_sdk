// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterChannelKeyResponder.h"

#include <Carbon/Carbon.h>
#import <Foundation/Foundation.h>

#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#include "flutter/testing/autoreleasepool_test.h"
#include "flutter/testing/testing.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

// FlutterBasicMessageChannel fake instance that records Flutter key event messages.
//
// When a sendMessage:reply: callback is specified, it is invoked with the value stored in the
// nextResponse property.
@interface FakeMessageChannel : FlutterBasicMessageChannel
@property(nonatomic, readonly) NSMutableArray<id>* messages;
@property(nonatomic) NSDictionary* nextResponse;

- (instancetype)init;
- (void)sendMessage:(id _Nullable)message;
- (void)sendMessage:(id _Nullable)message reply:(FlutterReply _Nullable)callback;
@end

@implementation FakeMessageChannel
- (instancetype)init {
  self = [super init];
  if (self != nil) {
    _messages = [[NSMutableArray<id> alloc] init];
  }
  return self;
}

- (void)sendMessage:(id _Nullable)message {
  [self sendMessage:message reply:nil];
}

- (void)sendMessage:(id _Nullable)message reply:(FlutterReply _Nullable)callback {
  [_messages addObject:message];
  if (callback) {
    callback(_nextResponse);
  }
}
@end

namespace flutter::testing {

namespace {
using flutter::testing::keycodes::kLogicalKeyQ;

NSEvent* CreateKeyEvent(NSEventType type,
                        NSEventModifierFlags modifierFlags,
                        NSString* characters,
                        NSString* charactersIgnoringModifiers,
                        BOOL isARepeat,
                        unsigned short keyCode) {
  return [NSEvent keyEventWithType:type
                          location:NSZeroPoint
                     modifierFlags:modifierFlags
                         timestamp:0
                      windowNumber:0
                           context:nil
                        characters:characters
       charactersIgnoringModifiers:charactersIgnoringModifiers
                         isARepeat:isARepeat
                           keyCode:keyCode];
}
}  // namespace

using FlutterChannelKeyResponderTest = AutoreleasePoolTest;

TEST_F(FlutterChannelKeyResponderTest, BasicKeyEvent) {
  __block NSMutableArray<NSNumber*>* responses = [[NSMutableArray<NSNumber*> alloc] init];
  FakeMessageChannel* channel = [[FakeMessageChannel alloc] init];
  FlutterChannelKeyResponder* responder =
      [[FlutterChannelKeyResponder alloc] initWithChannel:channel];

  // Initial empty modifiers.
  //
  // This can happen when user opens window while modifier key is pressed and then releases the
  // modifier. No events should be sent, but the callback should still be called.
  // Regression test for https://github.com/flutter/flutter/issues/87339.
  channel.nextResponse = @{@"handled" : @YES};
  [responder handleEvent:CreateKeyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", NO, 60)
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  EXPECT_EQ([channel.messages count], 0u);
  ASSERT_EQ([responses count], 1u);
  EXPECT_EQ([responses[0] boolValue], YES);
  [responses removeAllObjects];

  // Key down
  channel.nextResponse = @{@"handled" : @YES};
  [responder handleEvent:CreateKeyEvent(NSEventTypeKeyDown, 0x100, @"a", @"a", NO, 0)
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  ASSERT_EQ([channel.messages count], 1u);
  EXPECT_STREQ([[channel.messages lastObject][@"keymap"] UTF8String], "macos");
  EXPECT_STREQ([[channel.messages lastObject][@"type"] UTF8String], "keydown");
  EXPECT_EQ([[channel.messages lastObject][@"keyCode"] intValue], 0);
  EXPECT_EQ([[channel.messages lastObject][@"modifiers"] intValue], 0x0);
  EXPECT_STREQ([[channel.messages lastObject][@"characters"] UTF8String], "a");
  EXPECT_STREQ([[channel.messages lastObject][@"charactersIgnoringModifiers"] UTF8String], "a");

  ASSERT_EQ([responses count], 1u);
  EXPECT_EQ([[responses lastObject] boolValue], YES);

  [channel.messages removeAllObjects];
  [responses removeAllObjects];

  // Key up
  channel.nextResponse = @{@"handled" : @NO};
  [responder handleEvent:CreateKeyEvent(NSEventTypeKeyUp, 0x100, @"a", @"a", NO, 0)
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  ASSERT_EQ([channel.messages count], 1u);
  EXPECT_STREQ([[channel.messages lastObject][@"keymap"] UTF8String], "macos");
  EXPECT_STREQ([[channel.messages lastObject][@"type"] UTF8String], "keyup");
  EXPECT_EQ([[channel.messages lastObject][@"keyCode"] intValue], 0);
  EXPECT_EQ([[channel.messages lastObject][@"modifiers"] intValue], 0);
  EXPECT_STREQ([[channel.messages lastObject][@"characters"] UTF8String], "a");
  EXPECT_STREQ([[channel.messages lastObject][@"charactersIgnoringModifiers"] UTF8String], "a");

  ASSERT_EQ([responses count], 1u);
  EXPECT_EQ([[responses lastObject] boolValue], NO);

  [channel.messages removeAllObjects];
  [responses removeAllObjects];

  // LShift down
  channel.nextResponse = @{@"handled" : @YES};
  [responder handleEvent:CreateKeyEvent(NSEventTypeFlagsChanged, 0x20102, @"", @"", NO, 56)
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  ASSERT_EQ([channel.messages count], 1u);
  EXPECT_STREQ([[channel.messages lastObject][@"keymap"] UTF8String], "macos");
  EXPECT_STREQ([[channel.messages lastObject][@"type"] UTF8String], "keydown");
  EXPECT_EQ([[channel.messages lastObject][@"keyCode"] intValue], 56);
  EXPECT_EQ([[channel.messages lastObject][@"modifiers"] intValue], 0x20002);

  ASSERT_EQ([responses count], 1u);
  EXPECT_EQ([[responses lastObject] boolValue], YES);

  [channel.messages removeAllObjects];
  [responses removeAllObjects];

  // RShift down
  channel.nextResponse = @{@"handled" : @NO};
  [responder handleEvent:CreateKeyEvent(NSEventTypeFlagsChanged, 0x20006, @"", @"", NO, 60)
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  ASSERT_EQ([channel.messages count], 1u);
  EXPECT_STREQ([[channel.messages lastObject][@"keymap"] UTF8String], "macos");
  EXPECT_STREQ([[channel.messages lastObject][@"type"] UTF8String], "keydown");
  EXPECT_EQ([[channel.messages lastObject][@"keyCode"] intValue], 60);
  EXPECT_EQ([[channel.messages lastObject][@"modifiers"] intValue], 0x20006);

  ASSERT_EQ([responses count], 1u);
  EXPECT_EQ([[responses lastObject] boolValue], NO);

  [channel.messages removeAllObjects];
  [responses removeAllObjects];

  // LShift up
  channel.nextResponse = @{@"handled" : @NO};
  [responder handleEvent:CreateKeyEvent(NSEventTypeFlagsChanged, 0x20104, @"", @"", NO, 56)
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  ASSERT_EQ([channel.messages count], 1u);
  EXPECT_STREQ([[channel.messages lastObject][@"keymap"] UTF8String], "macos");
  EXPECT_STREQ([[channel.messages lastObject][@"type"] UTF8String], "keyup");
  EXPECT_EQ([[channel.messages lastObject][@"keyCode"] intValue], 56);
  EXPECT_EQ([[channel.messages lastObject][@"modifiers"] intValue], 0x20004);

  ASSERT_EQ([responses count], 1u);
  EXPECT_EQ([[responses lastObject] boolValue], NO);

  [channel.messages removeAllObjects];
  [responses removeAllObjects];

  // RShift up
  channel.nextResponse = @{@"handled" : @NO};
  [responder handleEvent:CreateKeyEvent(NSEventTypeFlagsChanged, 0, @"", @"", NO, 60)
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  ASSERT_EQ([channel.messages count], 1u);
  EXPECT_STREQ([[channel.messages lastObject][@"keymap"] UTF8String], "macos");
  EXPECT_STREQ([[channel.messages lastObject][@"type"] UTF8String], "keyup");
  EXPECT_EQ([[channel.messages lastObject][@"keyCode"] intValue], 60);
  EXPECT_EQ([[channel.messages lastObject][@"modifiers"] intValue], 0);

  ASSERT_EQ([responses count], 1u);
  EXPECT_EQ([[responses lastObject] boolValue], NO);

  [channel.messages removeAllObjects];
  [responses removeAllObjects];

  // RShift up again, should be ignored and not produce a keydown event, but the
  // callback should be called.
  channel.nextResponse = @{@"handled" : @NO};
  [responder handleEvent:CreateKeyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", NO, 60)
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  EXPECT_EQ([channel.messages count], 0u);
  ASSERT_EQ([responses count], 1u);
  EXPECT_EQ([responses[0] boolValue], YES);
}

TEST_F(FlutterChannelKeyResponderTest, EmptyResponseIsTakenAsHandled) {
  __block NSMutableArray<NSNumber*>* responses = [[NSMutableArray<NSNumber*> alloc] init];
  FakeMessageChannel* channel = [[FakeMessageChannel alloc] init];
  FlutterChannelKeyResponder* responder =
      [[FlutterChannelKeyResponder alloc] initWithChannel:channel];

  channel.nextResponse = nil;
  [responder handleEvent:CreateKeyEvent(NSEventTypeKeyDown, 0x100, @"a", @"a", NO, 0)
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  ASSERT_EQ([channel.messages count], 1u);
  EXPECT_STREQ([[channel.messages lastObject][@"keymap"] UTF8String], "macos");
  EXPECT_STREQ([[channel.messages lastObject][@"type"] UTF8String], "keydown");
  EXPECT_EQ([[channel.messages lastObject][@"keyCode"] intValue], 0);
  EXPECT_EQ([[channel.messages lastObject][@"modifiers"] intValue], 0);
  EXPECT_STREQ([[channel.messages lastObject][@"characters"] UTF8String], "a");
  EXPECT_STREQ([[channel.messages lastObject][@"charactersIgnoringModifiers"] UTF8String], "a");

  ASSERT_EQ([responses count], 1u);
  EXPECT_EQ([[responses lastObject] boolValue], YES);
}

TEST_F(FlutterChannelKeyResponderTest, FollowsLayoutMap) {
  __block NSMutableArray<NSNumber*>* responses = [[NSMutableArray<NSNumber*> alloc] init];
  FakeMessageChannel* channel = [[FakeMessageChannel alloc] init];
  FlutterChannelKeyResponder* responder =
      [[FlutterChannelKeyResponder alloc] initWithChannel:channel];

  NSMutableDictionary<NSNumber*, NSNumber*>* layoutMap =
      [NSMutableDictionary<NSNumber*, NSNumber*> dictionary];
  responder.layoutMap = layoutMap;
  // French layout
  layoutMap[@(kVK_ANSI_A)] = @(kLogicalKeyQ);

  channel.nextResponse = @{@"handled" : @YES};
  [responder handleEvent:CreateKeyEvent(NSEventTypeKeyDown, kVK_ANSI_A, @"q", @"q", NO, 0)
                callback:^(BOOL handled) {
                  [responses addObject:@(handled)];
                }];

  ASSERT_EQ([channel.messages count], 1u);
  EXPECT_STREQ([[channel.messages lastObject][@"keymap"] UTF8String], "macos");
  EXPECT_STREQ([[channel.messages lastObject][@"type"] UTF8String], "keydown");
  EXPECT_EQ([[channel.messages lastObject][@"keyCode"] intValue], 0);
  EXPECT_EQ([[channel.messages lastObject][@"modifiers"] intValue], 0x0);
  EXPECT_STREQ([[channel.messages lastObject][@"characters"] UTF8String], "q");
  EXPECT_STREQ([[channel.messages lastObject][@"charactersIgnoringModifiers"] UTF8String], "q");
  EXPECT_EQ([channel.messages lastObject][@"specifiedLogicalKey"], @(kLogicalKeyQ));

  ASSERT_EQ([responses count], 1u);
  EXPECT_EQ([[responses lastObject] boolValue], YES);

  [channel.messages removeAllObjects];
  [responses removeAllObjects];
}

}  // namespace flutter::testing

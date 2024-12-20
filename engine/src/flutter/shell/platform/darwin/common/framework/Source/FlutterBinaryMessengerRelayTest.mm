// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/framework/Source/FlutterBinaryMessengerRelay.h"

#import <OCMock/OCMock.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/common/framework/Source/FlutterTestUtils.h"
#import "flutter/shell/platform/darwin/ios/flutter_task_queue_dispatch.h"
#import "flutter/testing/testing.h"
#include "gtest/gtest.h"

FLUTTER_ASSERT_ARC

@interface FlutterBinaryMessengerRelayTest : NSObject
@end

@implementation FlutterBinaryMessengerRelayTest

- (void)testCreate {
  id messenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  FlutterBinaryMessengerRelay* relay =
      [[FlutterBinaryMessengerRelay alloc] initWithParent:messenger];
  EXPECT_NE(relay, nil);
  EXPECT_EQ(messenger, relay.parent);
}

- (void)testPassesCallOn {
  id messenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  FlutterBinaryMessengerRelay* relay =
      [[FlutterBinaryMessengerRelay alloc] initWithParent:messenger];
  char messageData[] = {'a', 'a', 'r', 'o', 'n'};
  NSData* message = [NSData dataWithBytes:messageData length:sizeof(messageData)];
  NSString* channel = @"foobar";
  [relay sendOnChannel:channel message:message binaryReply:nil];
  OCMVerify([messenger sendOnChannel:channel message:message binaryReply:nil]);
}

- (void)testDoesntPassCallOn {
  id messenger = OCMStrictProtocolMock(@protocol(FlutterBinaryMessenger));
  FlutterBinaryMessengerRelay* relay =
      [[FlutterBinaryMessengerRelay alloc] initWithParent:messenger];
  char messageData[] = {'a', 'a', 'r', 'o', 'n'};
  NSData* message = [NSData dataWithBytes:messageData length:sizeof(messageData)];
  NSString* channel = @"foobar";
  relay.parent = nil;
  [relay sendOnChannel:channel message:message binaryReply:nil];
}

- (void)testSetMessageHandlerWithTaskQueue {
  id messenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  FlutterBinaryMessengerRelay* relay =
      [[FlutterBinaryMessengerRelay alloc] initWithParent:messenger];
  NSString* channel = @"foobar";
  NSObject<FlutterTaskQueue>* taskQueue = OCMProtocolMock(@protocol(FlutterTaskQueueDispatch));
  FlutterBinaryMessageHandler handler = ^(NSData* _Nullable, FlutterBinaryReply _Nonnull) {
  };
  [relay setMessageHandlerOnChannel:channel binaryMessageHandler:handler taskQueue:taskQueue];
  OCMVerify([messenger setMessageHandlerOnChannel:channel
                             binaryMessageHandler:handler
                                        taskQueue:taskQueue]);
}

- (void)testMakeBackgroundTaskQueue {
  id messenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  FlutterBinaryMessengerRelay* relay =
      [[FlutterBinaryMessengerRelay alloc] initWithParent:messenger];
  [relay makeBackgroundTaskQueue];
  OCMVerify([messenger makeBackgroundTaskQueue]);
}

@end

TEST(FlutterBinaryMessengerRelayTest, Create) {
  ASSERT_FALSE(FLTThrowsObjcException(^{
    [[FlutterBinaryMessengerRelayTest alloc] testCreate];
  }));
}

TEST(FlutterBinaryMessengerRelayTest, PassesCallOn) {
  ASSERT_FALSE(FLTThrowsObjcException(^{
    [[FlutterBinaryMessengerRelayTest alloc] testPassesCallOn];
  }));
}

TEST(FlutterBinaryMessengerRelayTest, DoesntPassCallOn) {
  ASSERT_FALSE(FLTThrowsObjcException(^{
    [[FlutterBinaryMessengerRelayTest alloc] testDoesntPassCallOn];
  }));
}

TEST(FlutterBinaryMessengerRelayTest, SetMessageHandlerWithTaskQueue) {
  ASSERT_FALSE(FLTThrowsObjcException(^{
    [[FlutterBinaryMessengerRelayTest alloc] testSetMessageHandlerWithTaskQueue];
  }));
}

TEST(FlutterBinaryMessengerRelayTest, SetMakeBackgroundTaskQueue) {
  ASSERT_FALSE(FLTThrowsObjcException(^{
    [[FlutterBinaryMessengerRelayTest alloc] testMakeBackgroundTaskQueue];
  }));
}

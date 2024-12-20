// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"
#import "flutter/shell/platform/darwin/ios/flutter_task_queue_dispatch.h"

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

FLUTTER_ASSERT_ARC

@interface MockBinaryMessenger : NSObject <FlutterBinaryMessenger>
@property(nonatomic, copy) NSString* channel;
@property(nonatomic, strong) NSData* message;
@property(nonatomic, strong) NSMutableDictionary<NSString*, FlutterBinaryMessageHandler>* handlers;
@end
@implementation MockBinaryMessenger
- (instancetype)init {
  self = [super init];
  if (self) {
    _handlers = [[NSMutableDictionary<NSString*, FlutterBinaryMessageHandler> alloc] init];
  }
  return self;
}

- (void)sendOnChannel:(NSString*)channel message:(NSData* _Nullable)message {
  [self sendOnChannel:channel message:message binaryReply:nil];
}

- (void)sendOnChannel:(NSString*)channel
              message:(NSData* _Nullable)message
          binaryReply:(FlutterBinaryReply _Nullable)callback {
  self.channel = channel;
  self.message = message;
}

- (FlutterBinaryMessengerConnection)setMessageHandlerOnChannel:(NSString*)channel
                                          binaryMessageHandler:
                                              (FlutterBinaryMessageHandler _Nullable)handler {
  [self.handlers setObject:handler forKey:channel];
  return 0;
}

- (void)cleanUpConnection:(FlutterBinaryMessengerConnection)connection {
}

@end

@interface FlutterChannelsTest : XCTestCase
@end

@implementation FlutterChannelsTest

- (void)testMethodInvoke {
  NSString* channelName = @"foo";
  id binaryMessenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  FlutterMethodChannel* channel = [[FlutterMethodChannel alloc] initWithName:channelName
                                                             binaryMessenger:binaryMessenger
                                                                       codec:codec];
  XCTAssertNotNil(channel);
  NSData* encodedMethodCall = [@"hey" dataUsingEncoding:NSUTF8StringEncoding];
  OCMStub([codec encodeMethodCall:[OCMArg any]]).andReturn(encodedMethodCall);
  [channel invokeMethod:@"foo" arguments:@[ @(1) ]];
  OCMVerify([binaryMessenger sendOnChannel:channelName message:encodedMethodCall]);
}

- (void)testMethodInvokeWithReply {
  NSString* channelName = @"foo";
  id binaryMessenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  FlutterMethodChannel* channel = [[FlutterMethodChannel alloc] initWithName:channelName
                                                             binaryMessenger:binaryMessenger
                                                                       codec:codec];
  XCTAssertNotNil(channel);
  NSData* encodedMethodCall = [@"hey" dataUsingEncoding:NSUTF8StringEncoding];
  OCMStub([codec encodeMethodCall:[OCMArg any]]).andReturn(encodedMethodCall);
  XCTestExpectation* didCallReply = [self expectationWithDescription:@"didCallReply"];
  OCMExpect([binaryMessenger sendOnChannel:channelName
                                   message:encodedMethodCall
                               binaryReply:[OCMArg checkWithBlock:^BOOL(id obj) {
                                 FlutterBinaryReply reply = obj;
                                 reply(nil);
                                 return YES;
                               }]]);
  [channel invokeMethod:@"foo"
              arguments:@[ @1 ]
                 result:^(id _Nullable result) {
                   [didCallReply fulfill];
                   XCTAssertEqual(FlutterMethodNotImplemented, result);
                 }];
  OCMVerifyAll(binaryMessenger);
  [self waitForExpectationsWithTimeout:1.0 handler:nil];
}

- (void)testMethodMessageHandler {
  NSString* channelName = @"foo";
  id binaryMessenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  FlutterMethodChannel* channel = [[FlutterMethodChannel alloc] initWithName:channelName
                                                             binaryMessenger:binaryMessenger
                                                                       codec:codec];
  XCTAssertNotNil(channel);

  NSData* encodedMethodCall = [@"hey" dataUsingEncoding:NSUTF8StringEncoding];
  OCMStub([codec encodeMethodCall:[OCMArg any]]).andReturn(encodedMethodCall);
  FlutterMethodCallHandler handler =
      ^(FlutterMethodCall* _Nonnull call, FlutterResult _Nonnull result) {
      };
  [channel setMethodCallHandler:handler];
  OCMVerify([binaryMessenger setMessageHandlerOnChannel:channelName
                                   binaryMessageHandler:[OCMArg isNotNil]]);
}

- (void)testCallMethodHandler {
  NSString* channelName = @"foo";
  MockBinaryMessenger* binaryMessenger = [[MockBinaryMessenger alloc] init];
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  FlutterMethodChannel* channel = [[FlutterMethodChannel alloc] initWithName:channelName
                                                             binaryMessenger:binaryMessenger
                                                                       codec:codec];
  XCTAssertNotNil(channel);

  NSData* encodedMethodCall = [@"encoded" dataUsingEncoding:NSUTF8StringEncoding];
  NSData* replyData = [@"reply" dataUsingEncoding:NSUTF8StringEncoding];
  NSData* replyEnvelopeData = [@"reply-envelope" dataUsingEncoding:NSUTF8StringEncoding];
  FlutterMethodCall* methodCall = [[FlutterMethodCall alloc] init];
  OCMStub([codec decodeMethodCall:encodedMethodCall]).andReturn(methodCall);
  OCMStub([codec encodeSuccessEnvelope:replyData]).andReturn(replyEnvelopeData);
  XCTestExpectation* didCallHandler = [self expectationWithDescription:@"didCallHandler"];
  XCTestExpectation* didCallReply = [self expectationWithDescription:@"didCallReply"];
  FlutterMethodCallHandler handler =
      ^(FlutterMethodCall* _Nonnull call, FlutterResult _Nonnull result) {
        XCTAssertEqual(methodCall, call);
        [didCallHandler fulfill];
        result(replyData);
      };
  [channel setMethodCallHandler:handler];
  binaryMessenger.handlers[channelName](encodedMethodCall, ^(NSData* reply) {
    [didCallReply fulfill];
    XCTAssertEqual(replyEnvelopeData, reply);
  });
  [self waitForExpectationsWithTimeout:1.0 handler:nil];
}

- (void)testResize {
  NSString* channelName = @"flutter/test";
  id binaryMessenger = OCMStrictProtocolMock(@protocol(FlutterBinaryMessenger));
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  FlutterBasicMessageChannel* channel =
      [[FlutterBasicMessageChannel alloc] initWithName:channelName
                                       binaryMessenger:binaryMessenger
                                                 codec:codec];
  XCTAssertNotNil(channel);

  // The expected content was created from the following Dart code:
  //   MethodCall call = MethodCall('resize', ['flutter/test',3]);
  //   StandardMethodCodec().encodeMethodCall(call).buffer.asUint8List();
  const unsigned char bytes[] = {7,   6,   114, 101, 115, 105, 122, 101, 12,  2,
                                 7,   12,  102, 108, 117, 116, 116, 101, 114, 47,
                                 116, 101, 115, 116, 3,   3,   0,   0,   0};
  NSData* expectedMessage = [NSData dataWithBytes:bytes length:sizeof(bytes)];

  OCMExpect([binaryMessenger sendOnChannel:@"dev.flutter/channel-buffers" message:expectedMessage]);
  [channel resizeChannelBuffer:3];
  OCMVerifyAll(binaryMessenger);
  [binaryMessenger stopMocking];
}

- (void)testSetWarnsOnOverflow {
  NSString* channelName = @"flutter/test";
  id binaryMessenger = OCMStrictProtocolMock(@protocol(FlutterBinaryMessenger));
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  FlutterBasicMessageChannel* channel =
      [[FlutterBasicMessageChannel alloc] initWithName:channelName
                                       binaryMessenger:binaryMessenger
                                                 codec:codec];
  XCTAssertNotNil(channel);

  // The expected content was created from the following Dart code:
  //   MethodCall call = MethodCall('overflow',['flutter/test', true]);
  //   StandardMethodCodec().encodeMethodCall(call).buffer.asUint8List();
  const unsigned char bytes[] = {7,   8,   111, 118, 101, 114, 102, 108, 111, 119, 12,  2,   7, 12,
                                 102, 108, 117, 116, 116, 101, 114, 47,  116, 101, 115, 116, 1};
  NSData* expectedMessage = [NSData dataWithBytes:bytes length:sizeof(bytes)];

  OCMExpect([binaryMessenger sendOnChannel:@"dev.flutter/channel-buffers" message:expectedMessage]);
  [channel setWarnsOnOverflow:NO];
  OCMVerifyAll(binaryMessenger);
  [binaryMessenger stopMocking];
}

- (void)testBasicMessageChannelCleanup {
  NSString* channelName = @"foo";
  FlutterBinaryMessengerConnection connection = 123;
  id binaryMessenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  FlutterBasicMessageChannel* channel =
      [[FlutterBasicMessageChannel alloc] initWithName:channelName
                                       binaryMessenger:binaryMessenger
                                                 codec:codec];
  FlutterMessageHandler handler = ^(id _Nullable message, FlutterReply callback) {
  };
  OCMStub([binaryMessenger setMessageHandlerOnChannel:channelName
                                 binaryMessageHandler:[OCMArg any]])
      .andReturn(connection);
  [channel setMessageHandler:handler];
  OCMVerify([binaryMessenger setMessageHandlerOnChannel:channelName
                                   binaryMessageHandler:[OCMArg isNotNil]]);
  [channel setMessageHandler:nil];
  OCMVerify([binaryMessenger cleanUpConnection:connection]);
}

- (void)testMethodChannelCleanup {
  NSString* channelName = @"foo";
  FlutterBinaryMessengerConnection connection = 123;
  id binaryMessenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  FlutterMethodChannel* channel = [[FlutterMethodChannel alloc] initWithName:channelName
                                                             binaryMessenger:binaryMessenger
                                                                       codec:codec];
  XCTAssertNotNil(channel);

  OCMStub([binaryMessenger setMessageHandlerOnChannel:channelName
                                 binaryMessageHandler:[OCMArg any]])
      .andReturn(connection);

  FlutterMethodCallHandler handler =
      ^(FlutterMethodCall* _Nonnull call, FlutterResult _Nonnull result) {
      };
  [channel setMethodCallHandler:handler];
  OCMVerify([binaryMessenger setMessageHandlerOnChannel:channelName
                                   binaryMessageHandler:[OCMArg isNotNil]]);
  [channel setMethodCallHandler:nil];
  OCMVerify([binaryMessenger cleanUpConnection:connection]);
}

- (void)testBasicMessageChannelTaskQueue {
  NSString* channelName = @"foo";
  FlutterBinaryMessengerConnection connection = 123;
  id binaryMessenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  id taskQueue = OCMProtocolMock(@protocol(FlutterTaskQueueDispatch));
  FlutterBasicMessageChannel* channel =
      [[FlutterBasicMessageChannel alloc] initWithName:channelName
                                       binaryMessenger:binaryMessenger
                                                 codec:codec
                                             taskQueue:taskQueue];
  FlutterMessageHandler handler = ^(id _Nullable message, FlutterReply callback) {
  };
  OCMStub([binaryMessenger setMessageHandlerOnChannel:channelName
                                 binaryMessageHandler:[OCMArg any]
                                            taskQueue:taskQueue])
      .andReturn(connection);
  [channel setMessageHandler:handler];
  OCMVerify([binaryMessenger setMessageHandlerOnChannel:channelName
                                   binaryMessageHandler:[OCMArg isNotNil]
                                              taskQueue:taskQueue]);
  [channel setMessageHandler:nil];
  OCMVerify([binaryMessenger cleanUpConnection:connection]);
}

- (void)testBasicMessageChannelInvokeHandlerAfterChannelReleased {
  NSString* channelName = @"foo";
  __block NSString* handlerMessage;
  __block FlutterBinaryMessageHandler messageHandler;
  @autoreleasepool {
    FlutterBinaryMessengerConnection connection = 123;
    id binaryMessenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
    id codec = OCMProtocolMock(@protocol(FlutterMessageCodec));
    id taskQueue = OCMProtocolMock(@protocol(FlutterTaskQueueDispatch));
    FlutterBasicMessageChannel* channel =
        [[FlutterBasicMessageChannel alloc] initWithName:channelName
                                         binaryMessenger:binaryMessenger
                                                   codec:codec
                                               taskQueue:taskQueue];

    FlutterMessageHandler handler = ^(id _Nullable message, FlutterReply callback) {
      handlerMessage = message;
    };
    OCMStub([binaryMessenger
                setMessageHandlerOnChannel:channelName
                      binaryMessageHandler:[OCMArg checkWithBlock:^BOOL(
                                                       FlutterBinaryMessageHandler handler) {
                        messageHandler = handler;
                        return YES;
                      }]
                                 taskQueue:taskQueue])
        .andReturn(connection);
    OCMStub([codec decode:[OCMArg any]]).andReturn(@"decoded message");
    [channel setMessageHandler:handler];
  }
  // Channel is released, messageHandler should still retain the codec. The codec
  // internally makes a `decode` call and updates the `handlerMessage` to "decoded message".
  messageHandler([NSData data], ^(NSData* data){
                 });
  XCTAssertEqualObjects(handlerMessage, @"decoded message");
}

- (void)testMethodChannelInvokeHandlerAfterChannelReleased {
  NSString* channelName = @"foo";
  FlutterBinaryMessengerConnection connection = 123;
  __block FlutterMethodCall* decodedMethodCall;
  __block FlutterBinaryMessageHandler messageHandler;
  @autoreleasepool {
    id binaryMessenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
    id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
    id taskQueue = OCMProtocolMock(@protocol(FlutterTaskQueueDispatch));
    FlutterMethodChannel* channel = [[FlutterMethodChannel alloc] initWithName:channelName
                                                               binaryMessenger:binaryMessenger
                                                                         codec:codec
                                                                     taskQueue:taskQueue];
    FlutterMethodCallHandler handler = ^(FlutterMethodCall* call, FlutterResult result) {
      decodedMethodCall = call;
    };
    OCMStub([binaryMessenger
                setMessageHandlerOnChannel:channelName
                      binaryMessageHandler:[OCMArg checkWithBlock:^BOOL(
                                                       FlutterBinaryMessageHandler handler) {
                        messageHandler = handler;
                        return YES;
                      }]
                                 taskQueue:taskQueue])
        .andReturn(connection);
    OCMStub([codec decodeMethodCall:[OCMArg any]])
        .andReturn([FlutterMethodCall methodCallWithMethodName:@"decoded method call"
                                                     arguments:nil]);
    [channel setMethodCallHandler:handler];
  }
  messageHandler([NSData data], ^(NSData* data){
                 });
  XCTAssertEqualObjects(decodedMethodCall.method, @"decoded method call");
}

- (void)testMethodChannelTaskQueue {
  NSString* channelName = @"foo";
  FlutterBinaryMessengerConnection connection = 123;
  id binaryMessenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  id taskQueue = OCMProtocolMock(@protocol(FlutterTaskQueueDispatch));
  FlutterMethodChannel* channel = [[FlutterMethodChannel alloc] initWithName:channelName
                                                             binaryMessenger:binaryMessenger
                                                                       codec:codec
                                                                   taskQueue:taskQueue];
  XCTAssertNotNil(channel);
  FlutterMethodCallHandler handler = ^(FlutterMethodCall* call, FlutterResult result) {
  };
  OCMStub([binaryMessenger setMessageHandlerOnChannel:channelName
                                 binaryMessageHandler:[OCMArg any]
                                            taskQueue:taskQueue])
      .andReturn(connection);
  [channel setMethodCallHandler:handler];
  OCMVerify([binaryMessenger setMessageHandlerOnChannel:channelName
                                   binaryMessageHandler:[OCMArg isNotNil]
                                              taskQueue:taskQueue]);
  [channel setMethodCallHandler:nil];
  OCMVerify([binaryMessenger cleanUpConnection:connection]);
}

- (void)testEventChannelTaskQueue {
  NSString* channelName = @"foo";
  FlutterBinaryMessengerConnection connection = 123;
  id binaryMessenger = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  id codec = OCMProtocolMock(@protocol(FlutterMethodCodec));
  id taskQueue = OCMProtocolMock(@protocol(FlutterTaskQueueDispatch));
  id handler = OCMProtocolMock(@protocol(FlutterStreamHandler));
  FlutterEventChannel* channel = [[FlutterEventChannel alloc] initWithName:channelName
                                                           binaryMessenger:binaryMessenger
                                                                     codec:codec
                                                                 taskQueue:taskQueue];
  XCTAssertNotNil(channel);
  OCMStub([binaryMessenger setMessageHandlerOnChannel:channelName
                                 binaryMessageHandler:[OCMArg any]
                                            taskQueue:taskQueue])
      .andReturn(connection);
  [channel setStreamHandler:handler];
  OCMVerify([binaryMessenger setMessageHandlerOnChannel:channelName
                                   binaryMessageHandler:[OCMArg isNotNil]
                                              taskQueue:taskQueue]);
  [channel setStreamHandler:nil];
  OCMVerify([binaryMessenger cleanUpConnection:connection]);
}

@end

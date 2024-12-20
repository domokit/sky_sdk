// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/framework/Source/FlutterBinaryMessengerRelay.h"

#include "flutter/fml/logging.h"

FLUTTER_ASSERT_ARC

@implementation FlutterBinaryMessengerRelay
#pragma mark - FlutterBinaryMessenger

- (instancetype)initWithParent:(NSObject<FlutterBinaryMessenger>*)parent {
  self = [super init];
  if (self != nil) {
    _parent = parent;
  }
  return self;
}

- (void)sendOnChannel:(NSString*)channel message:(NSData*)message {
  if (self.parent) {
    [self.parent sendOnChannel:channel message:message binaryReply:nil];
  } else {
    FML_LOG(WARNING) << "Communicating on a dead channel.";
  }
}

- (void)sendOnChannel:(NSString*)channel
              message:(NSData*)message
          binaryReply:(FlutterBinaryReply)callback {
  if (self.parent) {
    [self.parent sendOnChannel:channel message:message binaryReply:callback];
  } else {
    FML_LOG(WARNING) << "Communicating on a dead channel.";
  }
}

- (NSObject<FlutterTaskQueue>*)makeBackgroundTaskQueue {
  if (self.parent) {
    return [self.parent makeBackgroundTaskQueue];
  } else {
    return nil;
  };
}

- (FlutterBinaryMessengerConnection)setMessageHandlerOnChannel:(NSString*)channel
                                          binaryMessageHandler:
                                              (FlutterBinaryMessageHandler)handler {
  if (self.parent) {
    return [self.parent setMessageHandlerOnChannel:channel binaryMessageHandler:handler];
  } else {
    FML_LOG(WARNING) << "Communicating on a dead channel.";
    return -1;
  }
}

- (FlutterBinaryMessengerConnection)setMessageHandlerOnChannel:(NSString*)channel
                                          binaryMessageHandler:(FlutterBinaryMessageHandler)handler
                                                     taskQueue:
                                                         (NSObject<FlutterTaskQueue>*)taskQueue {
  if (self.parent) {
    return [self.parent setMessageHandlerOnChannel:channel
                              binaryMessageHandler:handler
                                         taskQueue:taskQueue];
  } else {
    FML_LOG(WARNING) << "Communicating on a dead channel.";
    return -1;
  }
}

- (void)cleanUpConnection:(FlutterBinaryMessengerConnection)connection {
  if (self.parent) {
    return [self.parent cleanUpConnection:connection];
  } else {
    FML_LOG(WARNING) << "Communicating on a dead channel.";
  }
}

@end

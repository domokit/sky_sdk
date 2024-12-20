// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"

FLUTTER_ASSERT_ARC

#pragma mark - Basic message channel

static NSString* const kFlutterChannelBuffersChannel = @"dev.flutter/channel-buffers";
static NSString* const kResizeMethod = @"resize";
static NSString* const kOverflowMethod = @"overflow";

static void ResizeChannelBuffer(NSObject<FlutterBinaryMessenger>* binaryMessenger,
                                NSString* channel,
                                NSInteger newSize) {
  NSCAssert(newSize >= 0, @"Channel buffer size must be non-negative");
  // Cast newSize to int because the deserialization logic handles only 32 bits values,
  // see
  // https://github.com/flutter/engine/blob/93e8901490e78c7ba7e319cce4470d9c6478c6dc/lib/ui/channel_buffers.dart#L495.
  NSArray* args = @[ channel, @(static_cast<int>(newSize)) ];
  FlutterMethodCall* resizeMethodCall = [FlutterMethodCall methodCallWithMethodName:kResizeMethod
                                                                          arguments:args];
  NSObject<FlutterMethodCodec>* codec = [FlutterStandardMethodCodec sharedInstance];
  NSData* message = [codec encodeMethodCall:resizeMethodCall];
  [binaryMessenger sendOnChannel:kFlutterChannelBuffersChannel message:message];
}

/**
 * Defines whether a channel should show warning messages when discarding messages
 * due to overflow.
 *
 * @param binaryMessenger The binary messenger.
 * @param channel The channel name.
 * @param warns When false, the channel is expected to overflow and warning messages
 *              will not be shown.
 */
static void SetWarnsOnOverflow(NSObject<FlutterBinaryMessenger>* binaryMessenger,
                               NSString* channel,
                               BOOL warns) {
  FlutterMethodCall* overflowMethodCall =
      [FlutterMethodCall methodCallWithMethodName:kOverflowMethod
                                        arguments:@[ channel, @(!warns) ]];
  NSObject<FlutterMethodCodec>* codec = [FlutterStandardMethodCodec sharedInstance];
  NSData* message = [codec encodeMethodCall:overflowMethodCall];
  [binaryMessenger sendOnChannel:kFlutterChannelBuffersChannel message:message];
}

static FlutterBinaryMessengerConnection SetMessageHandler(
    NSObject<FlutterBinaryMessenger>* messenger,
    NSString* name,
    FlutterBinaryMessageHandler handler,
    NSObject<FlutterTaskQueue>* taskQueue) {
  if (taskQueue) {
    NSCAssert([messenger respondsToSelector:@selector(setMessageHandlerOnChannel:
                                                            binaryMessageHandler:taskQueue:)],
              @"");
    return [messenger setMessageHandlerOnChannel:name
                            binaryMessageHandler:handler
                                       taskQueue:taskQueue];
  } else {
    return [messenger setMessageHandlerOnChannel:name binaryMessageHandler:handler];
  }
}

////////////////////////////////////////////////////////////////////////////////
@implementation FlutterBasicMessageChannel {
  NSObject<FlutterBinaryMessenger>* _messenger;
  NSString* _name;
  NSObject<FlutterMessageCodec>* _codec;
  FlutterBinaryMessengerConnection _connection;
  NSObject<FlutterTaskQueue>* _taskQueue;
}
+ (instancetype)messageChannelWithName:(NSString*)name
                       binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger {
  NSObject<FlutterMessageCodec>* codec = [FlutterStandardMessageCodec sharedInstance];
  return [FlutterBasicMessageChannel messageChannelWithName:name
                                            binaryMessenger:messenger
                                                      codec:codec];
}
+ (instancetype)messageChannelWithName:(NSString*)name
                       binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                                 codec:(NSObject<FlutterMessageCodec>*)codec {
  return [[FlutterBasicMessageChannel alloc] initWithName:name
                                          binaryMessenger:messenger
                                                    codec:codec];
}

- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMessageCodec>*)codec {
  self = [self initWithName:name binaryMessenger:messenger codec:codec taskQueue:nil];
  return self;
}

- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMessageCodec>*)codec
                   taskQueue:(NSObject<FlutterTaskQueue>*)taskQueue {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _name = [name copy];
  _messenger = messenger;
  _codec = codec;
  _taskQueue = taskQueue;
  return self;
}

- (void)sendMessage:(id)message {
  [_messenger sendOnChannel:_name message:[_codec encode:message]];
}

- (void)sendMessage:(id)message reply:(FlutterReply)callback {
  FlutterBinaryReply reply = ^(NSData* data) {
    if (callback) {
      callback([_codec decode:data]);
    }
  };
  [_messenger sendOnChannel:_name message:[_codec encode:message] binaryReply:reply];
}

- (void)setMessageHandler:(FlutterMessageHandler)handler {
  if (!handler) {
    if (_connection > 0) {
      [_messenger cleanUpConnection:_connection];
      _connection = 0;
    } else {
      [_messenger setMessageHandlerOnChannel:_name binaryMessageHandler:nil];
    }
    return;
  }

  // Grab reference to avoid retain on self.
  // `self` might be released before the block, so the block needs to retain the codec to
  // make sure it is not released with `self`
  NSObject<FlutterMessageCodec>* codec = _codec;
  FlutterBinaryMessageHandler messageHandler = ^(NSData* message, FlutterBinaryReply callback) {
    handler([codec decode:message], ^(id reply) {
      callback([codec encode:reply]);
    });
  };
  _connection = SetMessageHandler(_messenger, _name, messageHandler, _taskQueue);
}

+ (void)resizeChannelWithName:(NSString*)name
              binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                         size:(NSInteger)newSize {
  ResizeChannelBuffer(messenger, name, newSize);
}

- (void)resizeChannelBuffer:(NSInteger)newSize {
  ResizeChannelBuffer(_messenger, _name, newSize);
}

+ (void)setWarnsOnOverflow:(BOOL)warns
        forChannelWithName:(NSString*)name
           binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger {
  SetWarnsOnOverflow(messenger, name, warns);
}

- (void)setWarnsOnOverflow:(BOOL)warns {
  SetWarnsOnOverflow(_messenger, _name, warns);
}

@end

#pragma mark - Method channel

////////////////////////////////////////////////////////////////////////////////
@implementation FlutterError
+ (instancetype)errorWithCode:(NSString*)code message:(NSString*)message details:(id)details {
  return [[FlutterError alloc] initWithCode:code message:message details:details];
}

- (instancetype)initWithCode:(NSString*)code message:(NSString*)message details:(id)details {
  NSAssert(code, @"Code cannot be nil");
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _code = [code copy];
  _message = [message copy];
  _details = details;
  return self;
}

- (BOOL)isEqual:(id)object {
  if (self == object) {
    return YES;
  }
  if (![object isKindOfClass:[FlutterError class]]) {
    return NO;
  }
  FlutterError* other = (FlutterError*)object;
  return [self.code isEqual:other.code] &&
         ((!self.message && !other.message) || [self.message isEqual:other.message]) &&
         ((!self.details && !other.details) || [self.details isEqual:other.details]);
}

- (NSUInteger)hash {
  return [self.code hash] ^ [self.message hash] ^ [self.details hash];
}
@end

////////////////////////////////////////////////////////////////////////////////
@implementation FlutterMethodCall
+ (instancetype)methodCallWithMethodName:(NSString*)method arguments:(id)arguments {
  return [[FlutterMethodCall alloc] initWithMethodName:method arguments:arguments];
}

- (instancetype)initWithMethodName:(NSString*)method arguments:(id)arguments {
  NSAssert(method, @"Method name cannot be nil");
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _method = [method copy];
  _arguments = arguments;
  return self;
}

- (BOOL)isEqual:(id)object {
  if (self == object) {
    return YES;
  }
  if (![object isKindOfClass:[FlutterMethodCall class]]) {
    return NO;
  }
  FlutterMethodCall* other = (FlutterMethodCall*)object;
  return [self.method isEqual:[other method]] &&
         ((!self.arguments && !other.arguments) || [self.arguments isEqual:other.arguments]);
}

- (NSUInteger)hash {
  return [self.method hash] ^ [self.arguments hash];
}
@end

NSObject const* FlutterMethodNotImplemented = [[NSObject alloc] init];

////////////////////////////////////////////////////////////////////////////////
@implementation FlutterMethodChannel {
  NSObject<FlutterBinaryMessenger>* _messenger;
  NSString* _name;
  NSObject<FlutterMethodCodec>* _codec;
  FlutterBinaryMessengerConnection _connection;
  NSObject<FlutterTaskQueue>* _taskQueue;
}

+ (instancetype)methodChannelWithName:(NSString*)name
                      binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger {
  NSObject<FlutterMethodCodec>* codec = [FlutterStandardMethodCodec sharedInstance];
  return [FlutterMethodChannel methodChannelWithName:name binaryMessenger:messenger codec:codec];
}

+ (instancetype)methodChannelWithName:(NSString*)name
                      binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                                codec:(NSObject<FlutterMethodCodec>*)codec {
  return [[FlutterMethodChannel alloc] initWithName:name binaryMessenger:messenger codec:codec];
}

- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMethodCodec>*)codec {
  self = [self initWithName:name binaryMessenger:messenger codec:codec taskQueue:nil];
  return self;
}
- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMethodCodec>*)codec
                   taskQueue:(NSObject<FlutterTaskQueue>*)taskQueue {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _name = [name copy];
  _messenger = messenger;
  _codec = codec;
  _taskQueue = taskQueue;
  return self;
}

- (void)invokeMethod:(NSString*)method arguments:(id)arguments {
  FlutterMethodCall* methodCall = [FlutterMethodCall methodCallWithMethodName:method
                                                                    arguments:arguments];
  NSData* message = [_codec encodeMethodCall:methodCall];
  [_messenger sendOnChannel:_name message:message];
}

- (void)invokeMethod:(NSString*)method arguments:(id)arguments result:(FlutterResult)callback {
  FlutterMethodCall* methodCall = [FlutterMethodCall methodCallWithMethodName:method
                                                                    arguments:arguments];
  NSData* message = [_codec encodeMethodCall:methodCall];
  FlutterBinaryReply reply = ^(NSData* data) {
    if (callback) {
      callback((data == nil) ? FlutterMethodNotImplemented : [_codec decodeEnvelope:data]);
    }
  };
  [_messenger sendOnChannel:_name message:message binaryReply:reply];
}

- (void)setMethodCallHandler:(FlutterMethodCallHandler)handler {
  if (!handler) {
    if (_connection > 0) {
      [_messenger cleanUpConnection:_connection];
      _connection = 0;
    } else {
      [_messenger setMessageHandlerOnChannel:_name binaryMessageHandler:nil];
    }
    return;
  }
  // Make sure the block captures the codec, not self.
  // `self` might be released before the block, so the block needs to retain the codec to
  // make sure it is not released with `self`
  NSObject<FlutterMethodCodec>* codec = _codec;
  FlutterBinaryMessageHandler messageHandler = ^(NSData* message, FlutterBinaryReply callback) {
    FlutterMethodCall* call = [codec decodeMethodCall:message];
    handler(call, ^(id result) {
      if (result == FlutterMethodNotImplemented) {
        callback(nil);
      } else if ([result isKindOfClass:[FlutterError class]]) {
        callback([codec encodeErrorEnvelope:(FlutterError*)result]);
      } else {
        callback([codec encodeSuccessEnvelope:result]);
      }
    });
  };
  _connection = SetMessageHandler(_messenger, _name, messageHandler, _taskQueue);
}

- (void)resizeChannelBuffer:(NSInteger)newSize {
  ResizeChannelBuffer(_messenger, _name, newSize);
}

@end

#pragma mark - Event channel

NSObject const* FlutterEndOfEventStream = [[NSObject alloc] init];

////////////////////////////////////////////////////////////////////////////////
@implementation FlutterEventChannel {
  NSObject<FlutterBinaryMessenger>* _messenger;
  NSString* _name;
  NSObject<FlutterMethodCodec>* _codec;
  NSObject<FlutterTaskQueue>* _taskQueue;
  FlutterBinaryMessengerConnection _connection;
}
+ (instancetype)eventChannelWithName:(NSString*)name
                     binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger {
  NSObject<FlutterMethodCodec>* codec = [FlutterStandardMethodCodec sharedInstance];
  return [FlutterEventChannel eventChannelWithName:name binaryMessenger:messenger codec:codec];
}

+ (instancetype)eventChannelWithName:(NSString*)name
                     binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                               codec:(NSObject<FlutterMethodCodec>*)codec {
  return [[FlutterEventChannel alloc] initWithName:name binaryMessenger:messenger codec:codec];
}

- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMethodCodec>*)codec {
  return [self initWithName:name binaryMessenger:messenger codec:codec taskQueue:nil];
}

- (instancetype)initWithName:(NSString*)name
             binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger
                       codec:(NSObject<FlutterMethodCodec>*)codec
                   taskQueue:(NSObject<FlutterTaskQueue>* _Nullable)taskQueue {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _name = [name copy];
  _messenger = messenger;
  _codec = codec;
  _taskQueue = taskQueue;
  return self;
}

static FlutterBinaryMessengerConnection SetStreamHandlerMessageHandlerOnChannel(
    NSObject<FlutterStreamHandler>* handler,
    NSString* name,
    NSObject<FlutterBinaryMessenger>* messenger,
    NSObject<FlutterMethodCodec>* codec,
    NSObject<FlutterTaskQueue>* taskQueue) {
  __block FlutterEventSink currentSink = nil;
  FlutterBinaryMessageHandler messageHandler = ^(NSData* message, FlutterBinaryReply callback) {
    FlutterMethodCall* call = [codec decodeMethodCall:message];
    if ([call.method isEqual:@"listen"]) {
      if (currentSink) {
        FlutterError* error = [handler onCancelWithArguments:nil];
        if (error) {
          NSLog(@"Failed to cancel existing stream: %@. %@ (%@)", error.code, error.message,
                error.details);
        }
      }
      currentSink = ^(id event) {
        if (event == FlutterEndOfEventStream) {
          [messenger sendOnChannel:name message:nil];
        } else if ([event isKindOfClass:[FlutterError class]]) {
          [messenger sendOnChannel:name message:[codec encodeErrorEnvelope:(FlutterError*)event]];
        } else {
          [messenger sendOnChannel:name message:[codec encodeSuccessEnvelope:event]];
        }
      };
      FlutterError* error = [handler onListenWithArguments:call.arguments eventSink:currentSink];
      if (error) {
        callback([codec encodeErrorEnvelope:error]);
      } else {
        callback([codec encodeSuccessEnvelope:nil]);
      }
    } else if ([call.method isEqual:@"cancel"]) {
      if (!currentSink) {
        callback(
            [codec encodeErrorEnvelope:[FlutterError errorWithCode:@"error"
                                                           message:@"No active stream to cancel"
                                                           details:nil]]);
        return;
      }
      currentSink = nil;
      FlutterError* error = [handler onCancelWithArguments:call.arguments];
      if (error) {
        callback([codec encodeErrorEnvelope:error]);
      } else {
        callback([codec encodeSuccessEnvelope:nil]);
      }
    } else {
      callback(nil);
    }
  };
  return SetMessageHandler(messenger, name, messageHandler, taskQueue);
}

- (void)setStreamHandler:(NSObject<FlutterStreamHandler>*)handler {
  if (!handler) {
    [_messenger cleanUpConnection:_connection];
    _connection = 0;
    return;
  }
  _connection =
      SetStreamHandlerMessageHandlerOnChannel(handler, _name, _messenger, _codec, _taskQueue);
}
@end

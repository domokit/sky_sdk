// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterCodecs.h"

#include <cstring>

FLUTTER_ASSERT_ARC

@implementation FlutterBinaryCodec
+ (instancetype)sharedInstance {
  static id _sharedInstance = nil;
  if (!_sharedInstance) {
    _sharedInstance = [[FlutterBinaryCodec alloc] init];
  }
  return _sharedInstance;
}

- (NSData*)encode:(id)message {
  NSAssert(!message || [message isKindOfClass:[NSData class]], @"");
  return message;
}

- (NSData*)decode:(NSData*)message {
  return message;
}
@end

@implementation FlutterStringCodec
+ (instancetype)sharedInstance {
  static id _sharedInstance = nil;
  if (!_sharedInstance) {
    _sharedInstance = [[FlutterStringCodec alloc] init];
  }
  return _sharedInstance;
}

- (NSData*)encode:(id)message {
  if (message == nil) {
    return nil;
  }
  NSAssert([message isKindOfClass:[NSString class]], @"");
  NSString* stringMessage = message;
  const char* utf8 = stringMessage.UTF8String;
  return [NSData dataWithBytes:utf8 length:strlen(utf8)];
}

- (NSString*)decode:(NSData*)message {
  if (message == nil) {
    return nil;
  }
  return [[NSString alloc] initWithData:message encoding:NSUTF8StringEncoding];
}
@end

@implementation FlutterJSONMessageCodec
+ (instancetype)sharedInstance {
  static id _sharedInstance = nil;
  if (!_sharedInstance) {
    _sharedInstance = [[FlutterJSONMessageCodec alloc] init];
  }
  return _sharedInstance;
}

- (NSData*)encode:(id)message {
  if (message == nil) {
    return nil;
  }
  NSData* encoding;
  NSError* error;
  if ([message isKindOfClass:[NSArray class]] || [message isKindOfClass:[NSDictionary class]]) {
    encoding = [NSJSONSerialization dataWithJSONObject:message options:0 error:&error];
  } else {
    // NSJSONSerialization does not support top-level simple values.
    // We encode as singleton array, then extract the relevant bytes.
    encoding = [NSJSONSerialization dataWithJSONObject:@[ message ] options:0 error:&error];
    if (encoding) {
      encoding = [encoding subdataWithRange:NSMakeRange(1, encoding.length - 2)];
    }
  }
  NSAssert(encoding, @"Invalid JSON message, encoding failed: %@", error);
  return encoding;
}

- (id)decode:(NSData*)message {
  if ([message length] == 0) {
    return nil;
  }
  BOOL isSimpleValue = NO;
  id decoded = nil;
  NSError* error;
  if (0 < message.length) {
    UInt8 first;
    [message getBytes:&first length:1];
    isSimpleValue = first != '{' && first != '[';
    if (isSimpleValue) {
      // NSJSONSerialization does not support top-level simple values.
      // We expand encoding to singleton array, then decode that and extract
      // the single entry.
      UInt8 begin = '[';
      UInt8 end = ']';
      NSMutableData* expandedMessage = [NSMutableData dataWithLength:message.length + 2];
      [expandedMessage replaceBytesInRange:NSMakeRange(0, 1) withBytes:&begin];
      [expandedMessage replaceBytesInRange:NSMakeRange(1, message.length) withBytes:message.bytes];
      [expandedMessage replaceBytesInRange:NSMakeRange(message.length + 1, 1) withBytes:&end];
      message = expandedMessage;
    }
    decoded = [NSJSONSerialization JSONObjectWithData:message options:0 error:&error];
  }
  NSAssert(decoded, @"Invalid JSON message, decoding failed: %@", error);
  return isSimpleValue ? ((NSArray*)decoded)[0] : decoded;
}
@end

@implementation FlutterJSONMethodCodec
+ (instancetype)sharedInstance {
  static id _sharedInstance = nil;
  if (!_sharedInstance) {
    _sharedInstance = [[FlutterJSONMethodCodec alloc] init];
  }
  return _sharedInstance;
}

- (NSData*)encodeMethodCall:(FlutterMethodCall*)call {
  return [[FlutterJSONMessageCodec sharedInstance] encode:@{
    @"method" : call.method,
    @"args" : [self wrapNil:call.arguments],
  }];
}

- (NSData*)encodeSuccessEnvelope:(id)result {
  return [[FlutterJSONMessageCodec sharedInstance] encode:@[ [self wrapNil:result] ]];
}

- (NSData*)encodeErrorEnvelope:(FlutterError*)error {
  return [[FlutterJSONMessageCodec sharedInstance] encode:@[
    error.code,
    [self wrapNil:error.message],
    [self wrapNil:error.details],
  ]];
}

- (FlutterMethodCall*)decodeMethodCall:(NSData*)message {
  NSDictionary* dictionary = [[FlutterJSONMessageCodec sharedInstance] decode:message];
  id method = dictionary[@"method"];
  id arguments = [self unwrapNil:dictionary[@"args"]];
  NSAssert([method isKindOfClass:[NSString class]], @"Invalid JSON method call");
  return [FlutterMethodCall methodCallWithMethodName:method arguments:arguments];
}

- (id)decodeEnvelope:(NSData*)envelope {
  NSArray* array = [[FlutterJSONMessageCodec sharedInstance] decode:envelope];
  if (array.count == 1) {
    return [self unwrapNil:array[0]];
  }
  NSAssert(array.count == 3, @"Invalid JSON envelope");
  id code = array[0];
  id message = [self unwrapNil:array[1]];
  id details = [self unwrapNil:array[2]];
  NSAssert([code isKindOfClass:[NSString class]], @"Invalid JSON envelope");
  NSAssert(message == nil || [message isKindOfClass:[NSString class]], @"Invalid JSON envelope");
  return [FlutterError errorWithCode:code message:message details:details];
}

- (id)wrapNil:(id)value {
  return value == nil ? [NSNull null] : value;
}
- (id)unwrapNil:(id)value {
  return value == [NSNull null] ? nil : value;
}
@end

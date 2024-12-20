// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterCodecs.h"

#include <CoreFoundation/CoreFoundation.h>

#include "gtest/gtest.h"

FLUTTER_ASSERT_ARC

@interface Pair : NSObject
@property(atomic, readonly, strong, nullable) NSObject* left;
@property(atomic, readonly, strong, nullable) NSObject* right;
- (instancetype)initWithLeft:(NSObject*)first right:(NSObject*)right;
@end

@implementation Pair
- (instancetype)initWithLeft:(NSObject*)left right:(NSObject*)right {
  self = [super init];
  if (self) {
    _left = left;
    _right = right;
  }
  return self;
}
@end

static const UInt8 kDATE = 128;
static const UInt8 kPAIR = 129;

@interface ExtendedWriter : FlutterStandardWriter
- (void)writeValue:(id)value;
@end

@implementation ExtendedWriter
- (void)writeValue:(id)value {
  if ([value isKindOfClass:[NSDate class]]) {
    [self writeByte:kDATE];
    NSDate* date = value;
    NSTimeInterval time = date.timeIntervalSince1970;
    SInt64 ms = (SInt64)(time * 1000.0);
    [self writeBytes:&ms length:8];
  } else if ([value isKindOfClass:[Pair class]]) {
    Pair* pair = value;
    [self writeByte:kPAIR];
    [self writeValue:pair.left];
    [self writeValue:pair.right];
  } else {
    [super writeValue:value];
  }
}
@end

@interface ExtendedReader : FlutterStandardReader
- (id)readValueOfType:(UInt8)type;
@end

@implementation ExtendedReader
- (id)readValueOfType:(UInt8)type {
  switch (type) {
    case kDATE: {
      SInt64 value;
      [self readBytes:&value length:8];
      NSTimeInterval time = [NSNumber numberWithLong:value].doubleValue / 1000.0;
      return [NSDate dateWithTimeIntervalSince1970:time];
    }
    case kPAIR: {
      return [[Pair alloc] initWithLeft:[self readValue] right:[self readValue]];
    }
    default:
      return [super readValueOfType:type];
  }
}
@end

@interface ExtendedReaderWriter : FlutterStandardReaderWriter
- (FlutterStandardWriter*)writerWithData:(NSMutableData*)data;
- (FlutterStandardReader*)readerWithData:(NSData*)data;
@end

@implementation ExtendedReaderWriter
- (FlutterStandardWriter*)writerWithData:(NSMutableData*)data {
  return [[ExtendedWriter alloc] initWithData:data];
}
- (FlutterStandardReader*)readerWithData:(NSData*)data {
  return [[ExtendedReader alloc] initWithData:data];
}
@end

static void CheckEncodeDecode(id value, NSData* expectedEncoding) {
  FlutterStandardMessageCodec* codec = [FlutterStandardMessageCodec sharedInstance];
  NSData* encoded = [codec encode:value];
  if (expectedEncoding == nil) {
    ASSERT_TRUE(encoded == nil);
  } else {
    ASSERT_TRUE([encoded isEqual:expectedEncoding]);
  }
  id decoded = [codec decode:encoded];
  if (value == nil || value == [NSNull null]) {
    ASSERT_TRUE(decoded == nil);
  } else {
    ASSERT_TRUE([value isEqual:decoded]);
  }
}

static void CheckEncodeDecode(id value) {
  FlutterStandardMessageCodec* codec = [FlutterStandardMessageCodec sharedInstance];
  NSData* encoded = [codec encode:value];
  id decoded = [codec decode:encoded];
  if (value == nil || value == [NSNull null]) {
    ASSERT_TRUE(decoded == nil);
  } else {
    ASSERT_TRUE([value isEqual:decoded]);
  }
}

TEST(FlutterStandardCodec, CanDecodeZeroLength) {
  FlutterStandardMessageCodec* codec = [FlutterStandardMessageCodec sharedInstance];
  id decoded = [codec decode:[NSData data]];
  ASSERT_TRUE(decoded == nil);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeNil) {
  CheckEncodeDecode(nil, nil);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeNSNull) {
  uint8_t bytes[1] = {0x00};
  CheckEncodeDecode([NSNull null], [NSData dataWithBytes:bytes length:1]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeYes) {
  uint8_t bytes[1] = {0x01};
  CheckEncodeDecode(@YES, [NSData dataWithBytes:bytes length:1]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeNo) {
  uint8_t bytes[1] = {0x02};
  CheckEncodeDecode(@NO, [NSData dataWithBytes:bytes length:1]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeUInt8) {
  uint8_t bytes[5] = {0x03, 0xfe, 0x00, 0x00, 0x00};
  UInt8 value = 0xfe;
  CheckEncodeDecode(@(value), [NSData dataWithBytes:bytes length:5]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeUInt16) {
  uint8_t bytes[5] = {0x03, 0xdc, 0xfe, 0x00, 0x00};
  UInt16 value = 0xfedc;
  CheckEncodeDecode(@(value), [NSData dataWithBytes:bytes length:5]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeUInt32) {
  uint8_t bytes[9] = {0x04, 0x09, 0xba, 0xdc, 0xfe, 0x00, 0x00, 0x00, 0x00};
  UInt32 value = 0xfedcba09;
  CheckEncodeDecode(@(value), [NSData dataWithBytes:bytes length:9]);
}

TEST(FlutterStandardCodec, CanEncodeUInt64) {
  FlutterStandardMessageCodec* codec = [FlutterStandardMessageCodec sharedInstance];
  UInt64 u64 = 0xfffffffffffffffa;
  uint8_t bytes[9] = {0x04, 0xfa, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  NSData* encoded = [codec encode:@(u64)];
  ASSERT_TRUE([encoded isEqual:[NSData dataWithBytes:bytes length:9]]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeSInt8) {
  uint8_t bytes[5] = {0x03, 0xfe, 0xff, 0xff, 0xff};
  SInt8 value = 0xfe;
  CheckEncodeDecode(@(value), [NSData dataWithBytes:bytes length:5]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeSInt16) {
  uint8_t bytes[5] = {0x03, 0xdc, 0xfe, 0xff, 0xff};
  SInt16 value = 0xfedc;
  CheckEncodeDecode(@(value), [NSData dataWithBytes:bytes length:5]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeSInt32) {
  uint8_t bytes[5] = {0x03, 0x78, 0x56, 0x34, 0x12};
  CheckEncodeDecode(@(0x12345678), [NSData dataWithBytes:bytes length:5]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeSInt64) {
  uint8_t bytes[9] = {0x04, 0xef, 0xcd, 0xab, 0x90, 0x78, 0x56, 0x34, 0x12};
  CheckEncodeDecode(@(0x1234567890abcdef), [NSData dataWithBytes:bytes length:9]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeFloat32) {
  uint8_t bytes[16] = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x60, 0xfb, 0x21, 0x09, 0x40};
  CheckEncodeDecode(@3.1415927f, [NSData dataWithBytes:bytes length:16]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeFloat64) {
  uint8_t bytes[16] = {0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x18, 0x2d, 0x44, 0x54, 0xfb, 0x21, 0x09, 0x40};
  CheckEncodeDecode(@3.14159265358979311599796346854, [NSData dataWithBytes:bytes length:16]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeString) {
  uint8_t bytes[13] = {0x07, 0x0b, 0x68, 0x65, 0x6c, 0x6c, 0x6f,
                       0x20, 0x77, 0x6f, 0x72, 0x6c, 0x64};
  CheckEncodeDecode(@"hello world", [NSData dataWithBytes:bytes length:13]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeStringWithNonAsciiCodePoint) {
  uint8_t bytes[7] = {0x07, 0x05, 0x68, 0xe2, 0x98, 0xba, 0x77};
  CheckEncodeDecode(@"h\u263Aw", [NSData dataWithBytes:bytes length:7]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeStringWithNonBMPCodePoint) {
  uint8_t bytes[8] = {0x07, 0x06, 0x68, 0xf0, 0x9f, 0x98, 0x82, 0x77};
  CheckEncodeDecode(@"h\U0001F602w", [NSData dataWithBytes:bytes length:8]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeIndirectString) {
  // This test ensures that an indirect NSString, whose internal string buffer
  // can't be simply returned by `CFStringGetCStringPtr`, can be encoded without
  // violating the memory sanitizer. This test only works with `--asan` flag.
  // See https://github.com/flutter/flutter/issues/142101
  uint8_t bytes[7] = {0x07, 0x05, 0x68, 0xe2, 0x98, 0xba, 0x77};
  NSString* target = @"h\u263Aw";
  // Ensures that this is an indirect string so that this test makes sense.
  ASSERT_TRUE(CFStringGetCStringPtr((__bridge CFStringRef)target, kCFStringEncodingUTF8) ==
              nullptr);
  CheckEncodeDecode(target, [NSData dataWithBytes:bytes length:7]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeArray) {
  NSArray* value = @[ [NSNull null], @"hello", @3.14, @47, @{@42 : @"nested"} ];
  CheckEncodeDecode(value);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeDictionary) {
  NSDictionary* value =
      @{@"a" : @3.14,
        @"b" : @47,
        [NSNull null] : [NSNull null],
        @3.14 : @[ @"nested" ]};
  CheckEncodeDecode(value);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeByteArray) {
  uint8_t bytes[4] = {0xBA, 0x5E, 0xBA, 0x11};
  NSData* data = [NSData dataWithBytes:bytes length:4];
  FlutterStandardTypedData* value = [FlutterStandardTypedData typedDataWithBytes:data];
  CheckEncodeDecode(value);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeNSData) {
  FlutterStandardMessageCodec* codec = [FlutterStandardMessageCodec sharedInstance];
  uint8_t bytes[4] = {0xBA, 0x5E, 0xBA, 0x11};
  NSData* data = [NSData dataWithBytes:bytes length:4];
  FlutterStandardTypedData* standardData = [FlutterStandardTypedData typedDataWithBytes:data];

  NSData* encoded = [codec encode:data];
  ASSERT_TRUE([encoded isEqual:[codec encode:standardData]]);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeInt32Array) {
  uint8_t bytes[8] = {0xBA, 0x5E, 0xBA, 0x11, 0xff, 0xff, 0xff, 0xff};
  NSData* data = [NSData dataWithBytes:bytes length:8];
  FlutterStandardTypedData* value = [FlutterStandardTypedData typedDataWithInt32:data];
  CheckEncodeDecode(value);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeInt64Array) {
  uint8_t bytes[8] = {0xBA, 0x5E, 0xBA, 0x11, 0xff, 0xff, 0xff, 0xff};
  NSData* data = [NSData dataWithBytes:bytes length:8];
  FlutterStandardTypedData* value = [FlutterStandardTypedData typedDataWithInt64:data];
  CheckEncodeDecode(value);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeFloat32Array) {
  uint8_t bytes[8] = {0xd8, 0x0f, 0x49, 0x40, 0x00, 0x00, 0x7a, 0x44};
  NSData* data = [NSData dataWithBytes:bytes length:8];
  FlutterStandardTypedData* value = [FlutterStandardTypedData typedDataWithFloat32:data];
  CheckEncodeDecode(value);
}

TEST(FlutterStandardCodec, CanEncodeAndDecodeFloat64Array) {
  uint8_t bytes[16] = {0xBA, 0x5E, 0xBA, 0x11, 0xff, 0xff, 0xff, 0xff,
                       0xBA, 0x5E, 0xBA, 0x11, 0xff, 0xff, 0xff, 0xff};
  NSData* data = [NSData dataWithBytes:bytes length:16];
  FlutterStandardTypedData* value = [FlutterStandardTypedData typedDataWithFloat64:data];
  CheckEncodeDecode(value);
}

TEST(FlutterStandardCodec, HandlesMethodCallsWithNilArguments) {
  FlutterStandardMethodCodec* codec = [FlutterStandardMethodCodec sharedInstance];
  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"hello" arguments:nil];
  NSData* encoded = [codec encodeMethodCall:call];
  FlutterMethodCall* decoded = [codec decodeMethodCall:encoded];
  ASSERT_TRUE([decoded isEqual:call]);
}

TEST(FlutterStandardCodec, HandlesMethodCallsWithSingleArgument) {
  FlutterStandardMethodCodec* codec = [FlutterStandardMethodCodec sharedInstance];
  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"hello" arguments:@42];
  NSData* encoded = [codec encodeMethodCall:call];
  FlutterMethodCall* decoded = [codec decodeMethodCall:encoded];
  ASSERT_TRUE([decoded isEqual:call]);
}

TEST(FlutterStandardCodec, HandlesMethodCallsWithArgumentList) {
  FlutterStandardMethodCodec* codec = [FlutterStandardMethodCodec sharedInstance];
  NSArray* arguments = @[ @42, @"world" ];
  FlutterMethodCall* call = [FlutterMethodCall methodCallWithMethodName:@"hello"
                                                              arguments:arguments];
  NSData* encoded = [codec encodeMethodCall:call];
  FlutterMethodCall* decoded = [codec decodeMethodCall:encoded];
  ASSERT_TRUE([decoded isEqual:call]);
}

TEST(FlutterStandardCodec, HandlesSuccessEnvelopesWithNilResult) {
  FlutterStandardMethodCodec* codec = [FlutterStandardMethodCodec sharedInstance];
  NSData* encoded = [codec encodeSuccessEnvelope:nil];
  id decoded = [codec decodeEnvelope:encoded];
  ASSERT_TRUE(decoded == nil);
}

TEST(FlutterStandardCodec, HandlesSuccessEnvelopesWithSingleResult) {
  FlutterStandardMethodCodec* codec = [FlutterStandardMethodCodec sharedInstance];
  NSData* encoded = [codec encodeSuccessEnvelope:@42];
  id decoded = [codec decodeEnvelope:encoded];
  ASSERT_TRUE([decoded isEqual:@42]);
}

TEST(FlutterStandardCodec, HandlesSuccessEnvelopesWithResultMap) {
  FlutterStandardMethodCodec* codec = [FlutterStandardMethodCodec sharedInstance];
  NSDictionary* result = @{@"a" : @42, @42 : @"a"};
  NSData* encoded = [codec encodeSuccessEnvelope:result];
  id decoded = [codec decodeEnvelope:encoded];
  ASSERT_TRUE([decoded isEqual:result]);
}

TEST(FlutterStandardCodec, HandlesErrorEnvelopes) {
  FlutterStandardMethodCodec* codec = [FlutterStandardMethodCodec sharedInstance];
  NSDictionary* details = @{@"a" : @42, @42 : @"a"};
  FlutterError* error = [FlutterError errorWithCode:@"errorCode"
                                            message:@"something failed"
                                            details:details];
  NSData* encoded = [codec encodeErrorEnvelope:error];
  id decoded = [codec decodeEnvelope:encoded];
  ASSERT_TRUE([decoded isEqual:error]);
}

TEST(FlutterStandardCodec, HandlesSubclasses) {
  ExtendedReaderWriter* extendedReaderWriter = [[ExtendedReaderWriter alloc] init];
  FlutterStandardMessageCodec* codec =
      [FlutterStandardMessageCodec codecWithReaderWriter:extendedReaderWriter];
  Pair* pair = [[Pair alloc] initWithLeft:@1 right:@2];
  NSData* encoded = [codec encode:pair];
  Pair* decoded = [codec decode:encoded];
  ASSERT_TRUE([pair.left isEqual:decoded.left]);
  ASSERT_TRUE([pair.right isEqual:decoded.right]);
}

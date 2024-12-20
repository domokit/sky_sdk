// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/framework/Source/FlutterStandardCodecHelper.h"
#import "flutter/shell/platform/darwin/common/framework/Source/FlutterStandardCodec_Internal.h"

FLUTTER_ASSERT_ARC

#pragma mark - Codec for basic message channel

@implementation FlutterStandardMessageCodec {
  FlutterStandardReaderWriter* _readerWriter;
}
+ (instancetype)sharedInstance {
  static id _sharedInstance = nil;
  if (!_sharedInstance) {
    FlutterStandardReaderWriter* readerWriter = [[FlutterStandardReaderWriter alloc] init];
    _sharedInstance = [[FlutterStandardMessageCodec alloc] initWithReaderWriter:readerWriter];
  }
  return _sharedInstance;
}

+ (instancetype)codecWithReaderWriter:(FlutterStandardReaderWriter*)readerWriter {
  return [[FlutterStandardMessageCodec alloc] initWithReaderWriter:readerWriter];
}

- (instancetype)initWithReaderWriter:(FlutterStandardReaderWriter*)readerWriter {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _readerWriter = readerWriter;
  return self;
}

- (NSData*)encode:(id)message {
  if (message == nil) {
    return nil;
  }
  NSMutableData* data = [NSMutableData dataWithCapacity:32];
  FlutterStandardWriter* writer = [_readerWriter writerWithData:data];
  [writer writeValue:message];
  return data;
}

- (id)decode:(NSData*)message {
  if ([message length] == 0) {
    return nil;
  }
  FlutterStandardReader* reader = [_readerWriter readerWithData:message];
  id value = [reader readValue];
  NSAssert(![reader hasMore], @"Corrupted standard message");
  return value;
}
@end

#pragma mark - Codec for method channel

@implementation FlutterStandardMethodCodec {
  FlutterStandardReaderWriter* _readerWriter;
}
+ (instancetype)sharedInstance {
  static id _sharedInstance = nil;
  if (!_sharedInstance) {
    FlutterStandardReaderWriter* readerWriter = [[FlutterStandardReaderWriter alloc] init];
    _sharedInstance = [[FlutterStandardMethodCodec alloc] initWithReaderWriter:readerWriter];
  }
  return _sharedInstance;
}

+ (instancetype)codecWithReaderWriter:(FlutterStandardReaderWriter*)readerWriter {
  return [[FlutterStandardMethodCodec alloc] initWithReaderWriter:readerWriter];
}

- (instancetype)initWithReaderWriter:(FlutterStandardReaderWriter*)readerWriter {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _readerWriter = readerWriter;
  return self;
}

- (NSData*)encodeMethodCall:(FlutterMethodCall*)call {
  NSMutableData* data = [NSMutableData dataWithCapacity:32];
  FlutterStandardWriter* writer = [_readerWriter writerWithData:data];
  [writer writeValue:call.method];
  [writer writeValue:call.arguments];
  return data;
}

- (NSData*)encodeSuccessEnvelope:(id)result {
  NSMutableData* data = [NSMutableData dataWithCapacity:32];
  FlutterStandardWriter* writer = [_readerWriter writerWithData:data];
  [writer writeByte:0];
  [writer writeValue:result];
  return data;
}

- (NSData*)encodeErrorEnvelope:(FlutterError*)error {
  NSMutableData* data = [NSMutableData dataWithCapacity:32];
  FlutterStandardWriter* writer = [_readerWriter writerWithData:data];
  [writer writeByte:1];
  [writer writeValue:error.code];
  [writer writeValue:error.message];
  [writer writeValue:error.details];
  return data;
}

- (FlutterMethodCall*)decodeMethodCall:(NSData*)message {
  FlutterStandardReader* reader = [_readerWriter readerWithData:message];
  id value1 = [reader readValue];
  id value2 = [reader readValue];
  NSAssert(![reader hasMore], @"Corrupted standard method call");
  NSAssert([value1 isKindOfClass:[NSString class]], @"Corrupted standard method call");
  return [FlutterMethodCall methodCallWithMethodName:value1 arguments:value2];
}

- (id)decodeEnvelope:(NSData*)envelope {
  FlutterStandardReader* reader = [_readerWriter readerWithData:envelope];
  UInt8 flag = [reader readByte];
  NSAssert(flag <= 1, @"Corrupted standard envelope");
  id result;
  switch (flag) {
    case 0: {
      result = [reader readValue];
      NSAssert(![reader hasMore], @"Corrupted standard envelope");
    } break;
    case 1: {
      id code = [reader readValue];
      id message = [reader readValue];
      id details = [reader readValue];
      NSAssert(![reader hasMore], @"Corrupted standard envelope");
      NSAssert([code isKindOfClass:[NSString class]], @"Invalid standard envelope");
      NSAssert(message == nil || [message isKindOfClass:[NSString class]],
               @"Invalid standard envelope");
      result = [FlutterError errorWithCode:code message:message details:details];
    } break;
  }
  return result;
}
@end

using namespace flutter;

#pragma mark - Standard serializable types

@implementation FlutterStandardTypedData
+ (instancetype)typedDataWithBytes:(NSData*)data {
  return [FlutterStandardTypedData typedDataWithData:data type:FlutterStandardDataTypeUInt8];
}

+ (instancetype)typedDataWithInt32:(NSData*)data {
  return [FlutterStandardTypedData typedDataWithData:data type:FlutterStandardDataTypeInt32];
}

+ (instancetype)typedDataWithInt64:(NSData*)data {
  return [FlutterStandardTypedData typedDataWithData:data type:FlutterStandardDataTypeInt64];
}

+ (instancetype)typedDataWithFloat32:(NSData*)data {
  return [FlutterStandardTypedData typedDataWithData:data type:FlutterStandardDataTypeFloat32];
}

+ (instancetype)typedDataWithFloat64:(NSData*)data {
  return [FlutterStandardTypedData typedDataWithData:data type:FlutterStandardDataTypeFloat64];
}

+ (instancetype)typedDataWithData:(NSData*)data type:(FlutterStandardDataType)type {
  return [[FlutterStandardTypedData alloc] initWithData:data type:type];
}

- (instancetype)initWithData:(NSData*)data type:(FlutterStandardDataType)type {
  UInt8 elementSize = elementSizeForFlutterStandardDataType(type);
  NSAssert(data, @"Data cannot be nil");
  NSAssert(data.length % elementSize == 0, @"Data must contain integral number of elements");
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _data = [data copy];
  _type = type;
  _elementSize = elementSize;
  _elementCount = data.length / elementSize;
  return self;
}

- (BOOL)isEqual:(id)object {
  if (self == object) {
    return YES;
  }
  if (![object isKindOfClass:[FlutterStandardTypedData class]]) {
    return NO;
  }
  FlutterStandardTypedData* other = (FlutterStandardTypedData*)object;
  return self.type == other.type && self.elementCount == other.elementCount &&
         [self.data isEqual:other.data];
}

- (NSUInteger)hash {
  return [self.data hash] ^ self.type;
}
@end

#pragma mark - Writer and reader of standard codec

@implementation FlutterStandardWriter {
  NSMutableData* _data;
}

- (instancetype)initWithData:(NSMutableData*)data {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _data = data;
  return self;
}

- (void)writeByte:(UInt8)value {
  FlutterStandardCodecHelperWriteByte((__bridge CFMutableDataRef)_data, value);
}

- (void)writeBytes:(const void*)bytes length:(NSUInteger)length {
  FlutterStandardCodecHelperWriteBytes((__bridge CFMutableDataRef)_data, bytes, length);
}

- (void)writeData:(NSData*)data {
  FlutterStandardCodecHelperWriteData((__bridge CFMutableDataRef)_data, (__bridge CFDataRef)data);
}

- (void)writeSize:(UInt32)size {
  FlutterStandardCodecHelperWriteSize((__bridge CFMutableDataRef)_data, size);
}

- (void)writeAlignment:(UInt8)alignment {
  FlutterStandardCodecHelperWriteAlignment((__bridge CFMutableDataRef)_data, alignment);
}

- (void)writeUTF8:(NSString*)value {
  FlutterStandardCodecHelperWriteUTF8((__bridge CFMutableDataRef)_data,
                                      (__bridge CFStringRef)value);
}

static FlutterStandardCodecObjcType GetWriteType(id value) {
  if (value == nil || (__bridge CFNullRef)value == kCFNull) {
    return FlutterStandardCodecObjcTypeNil;
  } else if ([value isKindOfClass:[NSNumber class]]) {
    return FlutterStandardCodecObjcTypeNSNumber;
  } else if ([value isKindOfClass:[NSString class]]) {
    return FlutterStandardCodecObjcTypeNSString;
  } else if ([value isKindOfClass:[FlutterStandardTypedData class]]) {
    return FlutterStandardCodecObjcTypeFlutterStandardTypedData;
  } else if ([value isKindOfClass:[NSData class]]) {
    return FlutterStandardCodecObjcTypeNSData;
  } else if ([value isKindOfClass:[NSArray class]]) {
    return FlutterStandardCodecObjcTypeNSArray;
  } else if ([value isKindOfClass:[NSDictionary class]]) {
    return FlutterStandardCodecObjcTypeNSDictionary;
  } else {
    return FlutterStandardCodecObjcTypeUnknown;
  }
}

struct WriteKeyValuesInfo {
  CFTypeRef writer;
  CFMutableDataRef data;
};

static void WriteKeyValues(CFTypeRef key, CFTypeRef value, void* context) {
  WriteKeyValuesInfo* info = (WriteKeyValuesInfo*)context;
  FastWriteValueOfType(info->writer, info->data, key);
  FastWriteValueOfType(info->writer, info->data, value);
}

// Recurses into WriteValueOfType directly if it is writing a known type,
// otherwise recurses with objc_msgSend.
static void FastWriteValueOfType(CFTypeRef writer, CFMutableDataRef data, CFTypeRef value) {
  FlutterStandardCodecObjcType type = GetWriteType((__bridge id)value);
  if (type != FlutterStandardCodecObjcTypeUnknown) {
    WriteValueOfType(writer, data, type, value);
  } else {
    [(__bridge FlutterStandardWriter*)writer writeValue:(__bridge id)value];
  }
}

static void WriteValueOfType(CFTypeRef writer,
                             CFMutableDataRef data,
                             FlutterStandardCodecObjcType type,
                             CFTypeRef value) {
  switch (type) {
    case FlutterStandardCodecObjcTypeNil:
      FlutterStandardCodecHelperWriteByte(data, FlutterStandardFieldNil);
      break;
    case FlutterStandardCodecObjcTypeNSNumber: {
      CFNumberRef number = (CFNumberRef)value;
      BOOL success = FlutterStandardCodecHelperWriteNumber(data, number);
      if (!success) {
        NSLog(@"Unsupported value: %@ of number type %ld", value, CFNumberGetType(number));
        NSCAssert(NO, @"Unsupported value for standard codec");
      }
      break;
    }
    case FlutterStandardCodecObjcTypeNSString: {
      CFStringRef string = (CFStringRef)value;
      FlutterStandardCodecHelperWriteByte(data, FlutterStandardFieldString);
      FlutterStandardCodecHelperWriteUTF8(data, string);
      break;
    }
    case FlutterStandardCodecObjcTypeFlutterStandardTypedData: {
      FlutterStandardTypedData* typedData = (__bridge FlutterStandardTypedData*)value;
      FlutterStandardCodecHelperWriteByte(data, FlutterStandardFieldForDataType(typedData.type));
      FlutterStandardCodecHelperWriteSize(data, typedData.elementCount);
      FlutterStandardCodecHelperWriteAlignment(data, typedData.elementSize);
      FlutterStandardCodecHelperWriteData(data, (__bridge CFDataRef)typedData.data);
      break;
    }
    case FlutterStandardCodecObjcTypeNSData:
      WriteValueOfType(writer, data, FlutterStandardCodecObjcTypeFlutterStandardTypedData,
                       (__bridge CFTypeRef)
                           [FlutterStandardTypedData typedDataWithBytes:(__bridge NSData*)value]);
      break;
    case FlutterStandardCodecObjcTypeNSArray: {
      CFArrayRef array = (CFArrayRef)value;
      FlutterStandardCodecHelperWriteByte(data, FlutterStandardFieldList);
      CFIndex count = CFArrayGetCount(array);
      FlutterStandardCodecHelperWriteSize(data, count);
      for (CFIndex i = 0; i < count; ++i) {
        FastWriteValueOfType(writer, data, CFArrayGetValueAtIndex(array, i));
      }
      break;
    }
    case FlutterStandardCodecObjcTypeNSDictionary: {
      CFDictionaryRef dict = (CFDictionaryRef)value;
      FlutterStandardCodecHelperWriteByte(data, FlutterStandardFieldMap);
      CFIndex count = CFDictionaryGetCount(dict);
      FlutterStandardCodecHelperWriteSize(data, count);
      WriteKeyValuesInfo info = {
          .writer = writer,
          .data = data,
      };
      CFDictionaryApplyFunction(dict, WriteKeyValues, (void*)&info);
      break;
    }
    case FlutterStandardCodecObjcTypeUnknown: {
      id objc_value = (__bridge id)value;
      NSLog(@"Unsupported value: %@ of type %@", objc_value, [objc_value class]);
      NSCAssert(NO, @"Unsupported value for standard codec");
      break;
    }
  }
}

- (void)writeValue:(id)value {
  FlutterStandardCodecObjcType type = GetWriteType(value);
  WriteValueOfType((__bridge CFTypeRef)self, (__bridge CFMutableDataRef)self->_data, type,
                   (__bridge CFTypeRef)value);
}
@end

@implementation FlutterStandardReader {
  NSData* _data;
  NSRange _range;
}

- (instancetype)initWithData:(NSData*)data {
  self = [super init];
  NSAssert(self, @"Super init cannot be nil");
  _data = [data copy];
  _range = NSMakeRange(0, 0);
  return self;
}

- (BOOL)hasMore {
  return _range.location < _data.length;
}

- (void)readBytes:(void*)destination length:(NSUInteger)length {
  FlutterStandardCodecHelperReadBytes(&_range.location, length, destination,
                                      (__bridge CFDataRef)_data);
}

- (UInt8)readByte {
  return FlutterStandardCodecHelperReadByte(&_range.location, (__bridge CFDataRef)_data);
}

- (UInt32)readSize {
  return FlutterStandardCodecHelperReadSize(&_range.location, (__bridge CFDataRef)_data);
}

- (NSData*)readData:(NSUInteger)length {
  _range.length = length;
  NSData* data = [_data subdataWithRange:_range];
  _range.location += _range.length;
  return data;
}

- (NSString*)readUTF8 {
  return (__bridge NSString*)FlutterStandardCodecHelperReadUTF8(&_range.location,
                                                                (__bridge CFDataRef)_data);
}

- (void)readAlignment:(UInt8)alignment {
  FlutterStandardCodecHelperReadAlignment(&_range.location, alignment);
}

- (nullable id)readValue {
  return (__bridge id)ReadValue((__bridge CFTypeRef)self);
}

static CFTypeRef ReadValue(CFTypeRef user_data) {
  FlutterStandardReader* reader = (__bridge FlutterStandardReader*)user_data;
  uint8_t type = FlutterStandardCodecHelperReadByte(&reader->_range.location,
                                                    (__bridge CFDataRef)reader->_data);
  return (__bridge CFTypeRef)[reader readValueOfType:type];
}

static CFTypeRef ReadTypedDataOfType(FlutterStandardField field, CFTypeRef user_data) {
  FlutterStandardReader* reader = (__bridge FlutterStandardReader*)user_data;
  unsigned long* location = &reader->_range.location;
  CFDataRef data = (__bridge CFDataRef)reader->_data;
  FlutterStandardDataType type = FlutterStandardDataTypeForField(field);

  UInt64 elementCount = FlutterStandardCodecHelperReadSize(location, data);
  UInt64 elementSize = elementSizeForFlutterStandardDataType(type);
  FlutterStandardCodecHelperReadAlignment(location, elementSize);
  UInt64 length = elementCount * elementSize;
  NSRange range = NSMakeRange(*location, length);
  // Note: subdataWithRange performs better than CFDataCreate and
  // CFDataCreateBytesNoCopy crashes.
  NSData* bytes = [(__bridge NSData*)data subdataWithRange:range];
  *location += length;
  return (__bridge CFTypeRef)[FlutterStandardTypedData typedDataWithData:bytes type:type];
}

- (nullable id)readValueOfType:(UInt8)type {
  return (__bridge id)FlutterStandardCodecHelperReadValueOfType(
      &_range.location, (__bridge CFDataRef)_data, type, ReadValue, ReadTypedDataOfType,
      (__bridge CFTypeRef)self);
}
@end

@implementation FlutterStandardReaderWriter
- (FlutterStandardWriter*)writerWithData:(NSMutableData*)data {
  return [[FlutterStandardWriter alloc] initWithData:data];
}

- (FlutterStandardReader*)readerWithData:(NSData*)data {
  return [[FlutterStandardReader alloc] initWithData:data];
}
@end

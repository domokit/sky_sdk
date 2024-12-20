// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/darwin/common/framework/Source/FlutterStandardCodecHelper.h"
#include <stdint.h>

#include <vector>

#include "flutter/fml/logging.h"

// The google-runtime-int lint suggests uint64_t in place of unsigned long,
// however these functions are frequently used with NSUInteger, which is
// defined as an unsigned long.
//
// NOLINTBEGIN(google-runtime-int)

void FlutterStandardCodecHelperReadAlignment(unsigned long* location,
                                             uint8_t alignment) {
  uint8_t mod = *location % alignment;
  if (mod) {
    *location += (alignment - mod);
  }
}

static uint8_t PeekByte(unsigned long location, CFDataRef data) {
  uint8_t result;
  CFRange range = CFRangeMake(location, 1);
  CFDataGetBytes(data, range, &result);
  return result;
}

void FlutterStandardCodecHelperReadBytes(unsigned long* location,
                                         unsigned long length,
                                         void* destination,
                                         CFDataRef data) {
  CFRange range = CFRangeMake(*location, length);
  CFDataGetBytes(data, range, static_cast<UInt8*>(destination));
  *location += length;
}

uint8_t FlutterStandardCodecHelperReadByte(unsigned long* location,
                                           CFDataRef data) {
  uint8_t value;
  FlutterStandardCodecHelperReadBytes(location, 1, &value, data);
  return value;
}

uint32_t FlutterStandardCodecHelperReadSize(unsigned long* location,
                                            CFDataRef data) {
  uint8_t byte = FlutterStandardCodecHelperReadByte(location, data);
  if (byte < 254) {
    return static_cast<uint32_t>(byte);
  } else if (byte == 254) {
    UInt16 value;
    FlutterStandardCodecHelperReadBytes(location, 2, &value, data);
    return value;
  } else {
    UInt32 value;
    FlutterStandardCodecHelperReadBytes(location, 4, &value, data);
    return value;
  }
}

static CFDataRef ReadDataNoCopy(unsigned long* location,
                                unsigned long length,
                                CFDataRef data) {
  CFDataRef result = CFDataCreateWithBytesNoCopy(
      kCFAllocatorDefault, CFDataGetBytePtr(data) + *location, length,
      kCFAllocatorNull);
  *location += length;
  return static_cast<CFDataRef>(CFAutorelease(result));
}

CFStringRef FlutterStandardCodecHelperReadUTF8(unsigned long* location,
                                               CFDataRef data) {
  uint32_t size = FlutterStandardCodecHelperReadSize(location, data);
  CFDataRef bytes = ReadDataNoCopy(location, size, data);
  CFStringRef result = CFStringCreateFromExternalRepresentation(
      kCFAllocatorDefault, bytes, kCFStringEncodingUTF8);
  return static_cast<CFStringRef>(CFAutorelease(result));
}

// Peeks ahead to see if we are reading a standard type.  If so, recurse
// directly to FlutterStandardCodecHelperReadValueOfType, otherwise recurse to
// objc.
static inline CFTypeRef FastReadValue(
    unsigned long* location,
    CFDataRef data,
    CFTypeRef (*ReadValue)(CFTypeRef),
    CFTypeRef (*ReadTypedDataOfType)(FlutterStandardField, CFTypeRef),
    CFTypeRef user_data) {
  uint8_t type = PeekByte(*location, data);
  if (FlutterStandardFieldIsStandardType(type)) {
    *location += 1;
    return FlutterStandardCodecHelperReadValueOfType(
        location, data, type, ReadValue, ReadTypedDataOfType, user_data);
  } else {
    return ReadValue(user_data);
  }
}

CFTypeRef FlutterStandardCodecHelperReadValueOfType(
    unsigned long* location,
    CFDataRef data,
    uint8_t type,
    CFTypeRef (*ReadValue)(CFTypeRef),
    CFTypeRef (*ReadTypedDataOfType)(FlutterStandardField, CFTypeRef),
    CFTypeRef user_data) {
  FlutterStandardField field = static_cast<FlutterStandardField>(type);
  switch (field) {
    case FlutterStandardFieldNil:
      return nil;
    case FlutterStandardFieldTrue:
      return kCFBooleanTrue;
    case FlutterStandardFieldFalse:
      return kCFBooleanFalse;
    case FlutterStandardFieldInt32: {
      int32_t value;
      FlutterStandardCodecHelperReadBytes(location, 4, &value, data);
      return CFAutorelease(
          CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &value));
    }
    case FlutterStandardFieldInt64: {
      int64_t value;
      FlutterStandardCodecHelperReadBytes(location, 8, &value, data);
      return CFAutorelease(
          CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt64Type, &value));
    }
    case FlutterStandardFieldFloat64: {
      Float64 value;
      FlutterStandardCodecHelperReadAlignment(location, 8);
      FlutterStandardCodecHelperReadBytes(location, 8, &value, data);
      return CFAutorelease(
          CFNumberCreate(kCFAllocatorDefault, kCFNumberDoubleType, &value));
    }
    case FlutterStandardFieldIntHex:
    case FlutterStandardFieldString:
      return FlutterStandardCodecHelperReadUTF8(location, data);
    case FlutterStandardFieldUInt8Data:
    case FlutterStandardFieldInt32Data:
    case FlutterStandardFieldInt64Data:
    case FlutterStandardFieldFloat32Data:
    case FlutterStandardFieldFloat64Data:
      return ReadTypedDataOfType(field, user_data);
    case FlutterStandardFieldList: {
      UInt32 length = FlutterStandardCodecHelperReadSize(location, data);
      CFMutableArrayRef array = CFArrayCreateMutable(
          kCFAllocatorDefault, length, &kCFTypeArrayCallBacks);
      for (UInt32 i = 0; i < length; i++) {
        CFTypeRef value = FastReadValue(location, data, ReadValue,
                                        ReadTypedDataOfType, user_data);
        CFArrayAppendValue(array, (value == nil ? kCFNull : value));
      }
      return CFAutorelease(array);
    }
    case FlutterStandardFieldMap: {
      UInt32 size = FlutterStandardCodecHelperReadSize(location, data);
      CFMutableDictionaryRef dict = CFDictionaryCreateMutable(
          kCFAllocatorDefault, size, &kCFTypeDictionaryKeyCallBacks,
          &kCFTypeDictionaryValueCallBacks);
      for (UInt32 i = 0; i < size; i++) {
        CFTypeRef key = FastReadValue(location, data, ReadValue,
                                      ReadTypedDataOfType, user_data);
        CFTypeRef val = FastReadValue(location, data, ReadValue,
                                      ReadTypedDataOfType, user_data);
        CFDictionaryAddValue(dict, (key == nil ? kCFNull : key),
                             (val == nil ? kCFNull : val));
      }
      return CFAutorelease(dict);
    }
    default:
      // Malformed message.
      FML_DCHECK(false);
  }
}

void FlutterStandardCodecHelperWriteByte(CFMutableDataRef data, uint8_t value) {
  CFDataAppendBytes(data, &value, 1);
}

void FlutterStandardCodecHelperWriteBytes(CFMutableDataRef data,
                                          const void* bytes,
                                          unsigned long length) {
  CFDataAppendBytes(data, static_cast<const UInt8*>(bytes), length);
}

void FlutterStandardCodecHelperWriteSize(CFMutableDataRef data, uint32_t size) {
  if (size < 254) {
    FlutterStandardCodecHelperWriteByte(data, size);
  } else if (size <= 0xffff) {
    FlutterStandardCodecHelperWriteByte(data, 254);
    UInt16 value = static_cast<UInt16>(size);
    FlutterStandardCodecHelperWriteBytes(data, &value, 2);
  } else {
    FlutterStandardCodecHelperWriteByte(data, 255);
    FlutterStandardCodecHelperWriteBytes(data, &size, 4);
  }
}

void FlutterStandardCodecHelperWriteAlignment(CFMutableDataRef data,
                                              uint8_t alignment) {
  uint8_t mod = CFDataGetLength(data) % alignment;
  if (mod) {
    for (int i = 0; i < (alignment - mod); i++) {
      FlutterStandardCodecHelperWriteByte(data, 0);
    }
  }
}

void FlutterStandardCodecHelperWriteUTF8(CFMutableDataRef data,
                                         CFStringRef value) {
  const char* utf8 = CFStringGetCStringPtr(value, kCFStringEncodingUTF8);
  if (utf8) {
    size_t length = strlen(utf8);
    FlutterStandardCodecHelperWriteSize(data, length);
    FlutterStandardCodecHelperWriteBytes(data, utf8, length);
  } else {
    CFIndex length = CFStringGetLength(value);
    CFIndex used_length = 0;
    // UTF16 length times 3 will fit all UTF8.
    CFIndex buffer_length = length * 3;
    std::vector<UInt8> buffer;
    buffer.resize(buffer_length);
    CFStringGetBytes(value, CFRangeMake(0, length), kCFStringEncodingUTF8, 0,
                     false, buffer.data(), buffer_length, &used_length);
    FlutterStandardCodecHelperWriteSize(data, used_length);
    FlutterStandardCodecHelperWriteBytes(data, buffer.data(), used_length);
  }
}

void FlutterStandardCodecHelperWriteData(CFMutableDataRef data,
                                         CFDataRef value) {
  const UInt8* bytes = CFDataGetBytePtr(value);
  CFIndex length = CFDataGetLength(value);
  FlutterStandardCodecHelperWriteBytes(data, bytes, length);
}

bool FlutterStandardCodecHelperWriteNumber(CFMutableDataRef data,
                                           CFNumberRef number) {
  bool success = false;
  if (CFGetTypeID(number) == CFBooleanGetTypeID()) {
    bool b = CFBooleanGetValue(reinterpret_cast<CFBooleanRef>(number));
    FlutterStandardCodecHelperWriteByte(
        data, (b ? FlutterStandardFieldTrue : FlutterStandardFieldFalse));
    success = true;
  } else if (CFNumberIsFloatType(number)) {
    Float64 f;
    success = CFNumberGetValue(number, kCFNumberFloat64Type, &f);
    if (success) {
      FlutterStandardCodecHelperWriteByte(data, FlutterStandardFieldFloat64);
      FlutterStandardCodecHelperWriteAlignment(data, 8);
      FlutterStandardCodecHelperWriteBytes(data, &f, 8);
    }
  } else if (CFNumberGetByteSize(number) <= 4) {
    SInt32 n;
    success = CFNumberGetValue(number, kCFNumberSInt32Type, &n);
    if (success) {
      FlutterStandardCodecHelperWriteByte(data, FlutterStandardFieldInt32);
      FlutterStandardCodecHelperWriteBytes(data, &n, 4);
    }
  } else if (CFNumberGetByteSize(number) <= 8) {
    SInt64 n;
    success = CFNumberGetValue(number, kCFNumberSInt64Type, &n);
    if (success) {
      FlutterStandardCodecHelperWriteByte(data, FlutterStandardFieldInt64);
      FlutterStandardCodecHelperWriteBytes(data, &n, 8);
    }
  }
  return success;
}

// NOLINTEND(google-runtime-int)

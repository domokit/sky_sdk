// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_SOURCE_FLUTTERSTANDARDCODECHELPER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_SOURCE_FLUTTERSTANDARDCODECHELPER_H_

#include <CoreFoundation/CoreFoundation.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

// NOLINTBEGIN(google-runtime-int)

// Note: Update FlutterStandardFieldIsStandardType if this changes.
typedef enum {
  // NOLINTBEGIN(readability-identifier-naming)
  FlutterStandardFieldNil,
  FlutterStandardFieldTrue,
  FlutterStandardFieldFalse,
  FlutterStandardFieldInt32,
  FlutterStandardFieldInt64,
  FlutterStandardFieldIntHex,
  FlutterStandardFieldFloat64,
  FlutterStandardFieldString,
  FlutterStandardFieldUInt8Data,
  FlutterStandardFieldInt32Data,
  FlutterStandardFieldInt64Data,
  FlutterStandardFieldFloat64Data,
  FlutterStandardFieldList,
  FlutterStandardFieldMap,
  FlutterStandardFieldFloat32Data,
  // NOLINTEND(readability-identifier-naming)
} FlutterStandardField;

static inline bool FlutterStandardFieldIsStandardType(uint8_t field) {
  return field <= FlutterStandardFieldFloat32Data &&
         field >= FlutterStandardFieldNil;
}

typedef enum {
  // NOLINTBEGIN(readability-identifier-naming)
  FlutterStandardCodecObjcTypeNil,
  FlutterStandardCodecObjcTypeNSNumber,
  FlutterStandardCodecObjcTypeNSString,
  FlutterStandardCodecObjcTypeFlutterStandardTypedData,
  FlutterStandardCodecObjcTypeNSData,
  FlutterStandardCodecObjcTypeNSArray,
  FlutterStandardCodecObjcTypeNSDictionary,
  FlutterStandardCodecObjcTypeUnknown,
  // NOLINTEND(readability-identifier-naming)
} FlutterStandardCodecObjcType;

// NOLINTBEGIN(google-objc-function-naming)

///////////////////////////////////////////////////////////////////////////////
///\name Reader Helpers
///@{

void FlutterStandardCodecHelperReadAlignment(unsigned long* location,
                                             uint8_t alignment);

void FlutterStandardCodecHelperReadBytes(unsigned long* location,
                                         unsigned long length,
                                         void* destination,
                                         CFDataRef data);

uint8_t FlutterStandardCodecHelperReadByte(unsigned long* location,
                                           CFDataRef data);

uint32_t FlutterStandardCodecHelperReadSize(unsigned long* location,
                                            CFDataRef data);

CFStringRef FlutterStandardCodecHelperReadUTF8(unsigned long* location,
                                               CFDataRef data);

CFTypeRef FlutterStandardCodecHelperReadValueOfType(
    unsigned long* location,
    CFDataRef data,
    uint8_t type,
    CFTypeRef (*ReadValue)(CFTypeRef),
    CFTypeRef (*ReadTypedDataOfType)(FlutterStandardField, CFTypeRef),
    CFTypeRef user_data);

///@}

///////////////////////////////////////////////////////////////////////////////
///\name Writer Helpers
///@{

void FlutterStandardCodecHelperWriteByte(CFMutableDataRef data, uint8_t value);

void FlutterStandardCodecHelperWriteBytes(CFMutableDataRef data,
                                          const void* bytes,
                                          unsigned long length);

void FlutterStandardCodecHelperWriteSize(CFMutableDataRef data, uint32_t size);

void FlutterStandardCodecHelperWriteAlignment(CFMutableDataRef data,
                                              uint8_t alignment);

void FlutterStandardCodecHelperWriteUTF8(CFMutableDataRef data,
                                         CFStringRef value);

void FlutterStandardCodecHelperWriteData(CFMutableDataRef data,
                                         CFDataRef value);

bool FlutterStandardCodecHelperWriteNumber(CFMutableDataRef data,
                                           CFNumberRef number);

///@}

// NOLINTEND(google-objc-function-naming)
// NOLINTEND(google-runtime-int)

#if defined(__cplusplus)
}
#endif

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_SOURCE_FLUTTERSTANDARDCODECHELPER_H_

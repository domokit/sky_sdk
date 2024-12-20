// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// These declarations are an amalgamation of different headers whose
// symbols exist in IOKit.framework.  The headers have been removed
// from the iOS SDKs but all the functions are documented here:
//  * https://developer.apple.com/documentation/iokit/iokitlib_h?language=objc
//  * https://developer.apple.com/documentation/iokit/iokit_functions?language=objc
//  * file:///Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/System/Library/Frameworks/IOKit.framework/Versions/A/Headers/IOKitLib.h

#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG || \
    FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_PROFILE
#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_IOKIT_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_IOKIT_H_

#if defined(__cplusplus)
extern "C" {
#endif  // defined(__cplusplus)

#include <CoreFoundation/CoreFoundation.h>
#include <mach/mach.h>
#include <stdint.h>

#define IOKIT
#include <device/device_types.h>

constexpr const char* kIOServicePlane = "IOService";

typedef io_object_t io_registry_entry_t;
typedef io_object_t io_service_t;
typedef io_object_t io_connect_t;
typedef io_object_t io_iterator_t;

enum {
  kIOReturnSuccess = 0,
};

extern const mach_port_t kIOMasterPortDefault;

kern_return_t IOObjectRetain(io_object_t object);
kern_return_t IOObjectRelease(io_object_t object);
boolean_t IOObjectConformsTo(io_object_t object, const io_name_t name);
uint32_t IOObjectGetKernelRetainCount(io_object_t object);
kern_return_t IOObjectGetClass(io_object_t object, io_name_t name);
CFStringRef IOObjectCopyClass(io_object_t object);
CFStringRef IOObjectCopySuperclassForClass(CFStringRef name);
CFStringRef IOObjectCopyBundleIdentifierForClass(CFStringRef name);

io_registry_entry_t IORegistryGetRootEntry(mach_port_t master);
kern_return_t IORegistryEntryGetName(io_registry_entry_t entry, io_name_t name);
kern_return_t IORegistryEntryGetRegistryEntryID(io_registry_entry_t entry,
                                                uint64_t* entryID);
kern_return_t IORegistryEntryGetPath(io_registry_entry_t entry,
                                     const io_name_t plane,
                                     io_string_t path);
kern_return_t IORegistryEntryGetProperty(io_registry_entry_t entry,
                                         const io_name_t name,
                                         io_struct_inband_t buffer,
                                         uint32_t* size);
kern_return_t IORegistryEntryCreateCFProperties(
    io_registry_entry_t entry,
    CFMutableDictionaryRef* properties,
    CFAllocatorRef allocator,
    uint32_t options);
CFTypeRef IORegistryEntryCreateCFProperty(io_registry_entry_t entry,
                                          CFStringRef key,
                                          CFAllocatorRef allocator,
                                          uint32_t options);
kern_return_t IORegistryEntrySetCFProperties(io_registry_entry_t entry,
                                             CFTypeRef properties);

kern_return_t IORegistryCreateIterator(mach_port_t master,
                                       const io_name_t plane,
                                       uint32_t options,
                                       io_iterator_t* it);
kern_return_t IORegistryEntryCreateIterator(io_registry_entry_t entry,
                                            const io_name_t plane,
                                            uint32_t options,
                                            io_iterator_t* it);
kern_return_t IORegistryEntryGetChildIterator(io_registry_entry_t entry,
                                              const io_name_t plane,
                                              io_iterator_t* it);
kern_return_t IORegistryEntryGetParentIterator(io_registry_entry_t entry,
                                               const io_name_t plane,
                                               io_iterator_t* it);
io_object_t IOIteratorNext(io_iterator_t it);
boolean_t IOIteratorIsValid(io_iterator_t it);
void IOIteratorReset(io_iterator_t it);

CFMutableDictionaryRef IOServiceMatching(const char* name) CF_RETURNS_RETAINED;
CFMutableDictionaryRef IOServiceNameMatching(const char* name)
    CF_RETURNS_RETAINED;
io_service_t IOServiceGetMatchingService(
    mach_port_t master,
    CFDictionaryRef matching CF_RELEASES_ARGUMENT);
kern_return_t IOServiceGetMatchingServices(
    mach_port_t master,
    CFDictionaryRef matching CF_RELEASES_ARGUMENT,
    io_iterator_t* it);

#if __cplusplus
}
#endif  // __cplusplus

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_IOKIT_H_
#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_IOKIT_H_
        // defined(FLUTTER_RUNTIME_MODE_PROFILE)

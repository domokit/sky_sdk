// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERCALLBACKCACHE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERCALLBACKCACHE_H_

#import <Foundation/Foundation.h>

#import "FlutterMacros.h"

/**
 * An object containing the result of `FlutterCallbackCache`'s `lookupCallbackInformation`
 * method.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterCallbackInformation : NSObject
/**
 * The name of the callback.
 */
@property(copy) NSString* callbackName;
/**
 * The class name of the callback.
 */
@property(copy) NSString* callbackClassName;
/**
 * The library path of the callback.
 */
@property(copy) NSString* callbackLibraryPath;
@end

/**
 * The cache containing callback information for spawning a
 * `FlutterHeadlessDartRunner`.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterCallbackCache : NSObject
/**
 * Returns the callback information for the given callback handle.
 * This callback information can be used when spawning a
 * `FlutterHeadlessDartRunner`.
 *
 * @param handle The handle for a callback, provided by the
 *   Dart method `PluginUtilities.getCallbackHandle`.
 * @return A `FlutterCallbackInformation` object which contains the name of the
 *   callback, the name of the class in which the callback is defined, and the
 *   path of the library which contains the callback. If the provided handle is
 *   invalid, nil is returned.
 */
+ (FlutterCallbackInformation*)lookupCallbackInformation:(int64_t)handle;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERCALLBACKCACHE_H_

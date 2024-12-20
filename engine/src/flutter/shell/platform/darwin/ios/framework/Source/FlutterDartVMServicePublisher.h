// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERDARTVMSERVICEPUBLISHER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERDARTVMSERVICEPUBLISHER_H_

#import <Foundation/Foundation.h>

@interface FlutterDartVMServicePublisher : NSObject

- (instancetype)initWithEnableVMServicePublication:(BOOL)enableVMServicePublication
    NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@property(nonatomic, readonly) NSURL* url;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERDARTVMSERVICEPUBLISHER_H_

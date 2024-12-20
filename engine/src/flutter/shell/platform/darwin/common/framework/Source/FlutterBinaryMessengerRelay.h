// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_SOURCE_FLUTTERBINARYMESSENGERRELAY_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_SOURCE_FLUTTERBINARYMESSENGERRELAY_H_

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterBinaryMessenger.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG
FLUTTER_DARWIN_EXPORT
#endif
@interface FlutterBinaryMessengerRelay : NSObject <FlutterBinaryMessenger>
@property(nonatomic, weak) NSObject<FlutterBinaryMessenger>* parent;
- (instancetype)initWithParent:(NSObject<FlutterBinaryMessenger>*)parent;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_SOURCE_FLUTTERBINARYMESSENGERRELAY_H_

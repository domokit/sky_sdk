// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTIMECONVERTER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTIMECONVERTER_H_

#import <CoreGraphics/CoreGraphics.h>

@class FlutterEngine;

/// Converts between the time representation used by Flutter Engine and CAMediaTime.
@interface FlutterTimeConverter : NSObject

- (instancetype)initWithEngine:(FlutterEngine*)engine;

- (uint64_t)CAMediaTimeToEngineTime:(CFTimeInterval)time;
- (CFTimeInterval)engineTimeToCAMediaTime:(uint64_t)time;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTIMECONVERTER_H_

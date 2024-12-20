// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTER_TIME_CONVERTER_MM_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTER_TIME_CONVERTER_MM_

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTimeConverter.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"

@interface FlutterTimeConverter () {
  __weak FlutterEngine* _engine;
}
@end

@implementation FlutterTimeConverter

- (instancetype)initWithEngine:(FlutterEngine*)engine {
  self = [super init];
  if (self) {
    _engine = engine;
  }
  return self;
}

- (uint64_t)CAMediaTimeToEngineTime:(CFTimeInterval)time {
  FlutterEngine* engine = _engine;
  if (!engine) {
    return 0;
  }
  return (time - CACurrentMediaTime()) * NSEC_PER_SEC + engine.embedderAPI.GetCurrentTime();
}

- (CFTimeInterval)engineTimeToCAMediaTime:(uint64_t)time {
  FlutterEngine* engine = _engine;
  if (!engine) {
    return 0;
  }
  return (static_cast<int64_t>(time) - static_cast<int64_t>(engine.embedderAPI.GetCurrentTime())) /
             static_cast<double>(NSEC_PER_SEC) +
         CACurrentMediaTime();
}

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTER_TIME_CONVERTER_MM_-

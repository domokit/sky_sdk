// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewEngineProvider.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"

@interface FlutterViewEngineProvider () {
  __weak FlutterEngine* _engine;
}

@end

@implementation FlutterViewEngineProvider

- (instancetype)initWithEngine:(FlutterEngine*)engine {
  self = [super init];
  if (self != nil) {
    _engine = engine;
  }
  return self;
}

- (nullable FlutterView*)viewForIdentifier:(FlutterViewIdentifier)viewIdentifier {
  return [_engine viewControllerForIdentifier:viewIdentifier].flutterView;
}

@end

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Cocoa/Cocoa.h>
#import <Foundation/Foundation.h>

#import "flutter/shell/platform/darwin/macos/framework/Source/TestFlutterPlatformView.h"

@implementation TestFlutterPlatformView

- (instancetype)initWithFrame:(CGRect)frame arguments:(nullable NSDictionary*)args {
  self = [super initWithFrame:frame];
  _args = args;
  return self;
}

@end

@implementation TestFlutterPlatformViewFactory
- (NSView*)createWithViewIdentifier:(int64_t)viewId arguments:(nullable id)args {
  return [[TestFlutterPlatformView alloc] initWithFrame:CGRectZero arguments:args];
}

- (NSObject<FlutterMessageCodec>*)createArgsCodec {
  return [FlutterStandardMessageCodec sharedInstance];
}

@end

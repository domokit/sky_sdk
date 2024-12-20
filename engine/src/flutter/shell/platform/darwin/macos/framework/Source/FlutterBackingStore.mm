// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterBackingStore.h"

@implementation FlutterRenderBackingStore

- (instancetype)initWithTexture:(id<MTLTexture>)texture {
  self = [super init];
  if (self) {
    _texture = texture;
  }
  return self;
}

@end

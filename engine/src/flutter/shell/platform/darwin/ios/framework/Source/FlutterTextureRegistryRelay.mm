// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextureRegistryRelay.h"

#include "flutter/fml/logging.h"

FLUTTER_ASSERT_ARC

@implementation FlutterTextureRegistryRelay : NSObject

#pragma mark - FlutterTextureRegistry

- (instancetype)initWithParent:(NSObject<FlutterTextureRegistry>*)parent {
  if (self = [super init]) {
    _parent = parent;
  }
  return self;
}

- (int64_t)registerTexture:(NSObject<FlutterTexture>*)texture {
  if (!self.parent) {
    FML_LOG(WARNING) << "Using on an empty registry.";
    return 0;
  }
  return [self.parent registerTexture:texture];
}

- (void)textureFrameAvailable:(int64_t)textureId {
  if (!self.parent) {
    FML_LOG(WARNING) << "Using on an empty registry.";
  }
  return [self.parent textureFrameAvailable:textureId];
}

- (void)unregisterTexture:(int64_t)textureId {
  if (!self.parent) {
    FML_LOG(WARNING) << "Using on an empty registry.";
  }
  return [self.parent unregisterTexture:textureId];
}

@end

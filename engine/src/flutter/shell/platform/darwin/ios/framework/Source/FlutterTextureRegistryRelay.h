// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERTEXTUREREGISTRYRELAY_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERTEXTUREREGISTRYRELAY_H_

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterTexture.h"

#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG
FLUTTER_DARWIN_EXPORT
#endif

/**
 * Wrapper around a weakly held collection of registered textures.
 *
 * Avoids a retain cycle between plugins and the engine.
 */
@interface FlutterTextureRegistryRelay : NSObject <FlutterTextureRegistry>

/**
 * A weak reference to a FlutterEngine that will be passed texture registration.
 */
@property(nonatomic, weak) NSObject<FlutterTextureRegistry>* parent;
- (instancetype)initWithParent:(NSObject<FlutterTextureRegistry>*)parent;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERTEXTUREREGISTRYRELAY_H_

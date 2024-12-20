// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTEREXTERNALTEXTURE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTEREXTERNALTEXTURE_H_

#import <Foundation/Foundation.h>

#import "flutter/shell/platform/darwin/graphics/FlutterDarwinContextMetalSkia.h"
#include "flutter/shell/platform/embedder/embedder.h"

/**
 * Embedding side texture wrappers for Metal external textures.
 * Used to bridge FlutterTexture object and handle the texture copy request the
 * Flutter engine.
 */
@interface FlutterExternalTexture : NSObject

/**
 * Initializes a texture adapter with |texture|.
 */
- (nonnull instancetype)initWithFlutterTexture:(nonnull id<FlutterTexture>)texture
                            darwinMetalContext:(nonnull FlutterDarwinContextMetalSkia*)context;

/**
 * Returns the ID for the FlutterExternalTexture instance.
 */
- (int64_t)textureID;

/**
 * Accepts texture buffer copy request from the Flutter engine.
 * When the user side marks the textureID as available, the Flutter engine will
 * callback to this method and ask for populate the |metalTexture| object,
 * such as the texture type and the format of the pixel buffer and the texture object.
 */
- (BOOL)populateTexture:(nonnull FlutterMetalExternalTexture*)metalTexture;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTEREXTERNALTEXTURE_H_

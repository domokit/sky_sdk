// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTEREMBEDDERKEYRESPONDER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTEREMBEDDERKEYRESPONDER_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyPrimaryResponder.h"
#include "flutter/shell/platform/embedder/embedder.h"

typedef void (^FlutterSendEmbedderKeyEvent)(const FlutterKeyEvent& /* event */,
                                            _Nullable FlutterKeyEventCallback /* callback */,
                                            void* _Nullable /* user_data */);

/**
 * A primary responder of |FlutterKeyboardManager| that handles events by
 * sending the converted events through the embedder API.
 *
 * This class communicates with the HardwareKeyboard API in the framework.
 */
@interface FlutterEmbedderKeyResponder : NSObject <FlutterKeyPrimaryResponder>

/**
 * Create an instance by specifying the function to send converted events to.
 *
 * The |sendEvent| is typically |FlutterEngine|'s |sendKeyEvent|.
 */
- (nonnull instancetype)initWithSendEvent:(_Nonnull FlutterSendEmbedderKeyEvent)sendEvent;

/**
 * Synthesize modifier keys events.
 *
 * If needed, synthesize modifier keys up and down events by comparing their
 * current pressing states with the given modifier flags.
 */
- (void)syncModifiersIfNeeded:(NSEventModifierFlags)modifierFlags
                    timestamp:(NSTimeInterval)timestamp;

/**
 * Returns the keyboard pressed state.
 *
 * Returns the keyboard pressed state. The dictionary contains one entry per
 * pressed keys, mapping from the logical key to the physical key.
 */
- (nonnull NSDictionary*)getPressedState;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTEREMBEDDERKEYRESPONDER_H_

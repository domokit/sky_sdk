// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTEREMBEDDERKEYRESPONDER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTEREMBEDDERKEYRESPONDER_H_

#import <Foundation/Foundation.h>

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterKeyPrimaryResponder.h"
#import "flutter/shell/platform/embedder/embedder.h"

typedef void (^FlutterSendKeyEvent)(const FlutterKeyEvent& /* event */,
                                    _Nullable FlutterKeyEventCallback /* callback */,
                                    void* _Nullable /* user_data */);

/**
 * A primary responder of |FlutterKeyboardManager| that handles events by
 * sending the converted events through a Dart hook to the framework.
 *
 * This class interfaces with the HardwareKeyboard API in the framework.
 */
@interface FlutterEmbedderKeyResponder : NSObject <FlutterKeyPrimaryResponder>

/**
 * Create an instance by specifying the function to send converted events to.
 *
 * The |sendEvent| is typically |FlutterEngine|'s |sendKeyEvent|.
 */
- (nonnull instancetype)initWithSendEvent:(nonnull FlutterSendKeyEvent)sendEvent;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTEREMBEDDERKEYRESPONDER_H_

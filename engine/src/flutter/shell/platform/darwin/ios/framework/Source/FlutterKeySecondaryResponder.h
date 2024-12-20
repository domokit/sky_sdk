// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERKEYSECONDARYRESPONDER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERKEYSECONDARYRESPONDER_H_

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUIPressProxy.h"

/**
 * An interface for a responder that can process a key event and synchronously
 * decide whether to handle the event.
 *
 * To use this class, add it to a |FlutterKeyboardManager| with
 * |addSecondaryResponder|.
 */
@protocol FlutterKeySecondaryResponder

/**
 * Informs the receiver that the user has interacted with a key.
 *
 * The return value indicates whether it has handled the given event.
 *
 * Default implementation returns NO.
 */
@required

- (BOOL)handlePress:(nonnull FlutterUIPressProxy*)press API_AVAILABLE(ios(13.4));
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERKEYSECONDARYRESPONDER_H_

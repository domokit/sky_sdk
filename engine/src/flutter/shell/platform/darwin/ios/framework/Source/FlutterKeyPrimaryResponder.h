// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERKEYPRIMARYRESPONDER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERKEYPRIMARYRESPONDER_H_

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUIPressProxy.h"

typedef void (^FlutterAsyncKeyCallback)(BOOL handled);

/**
 * An interface for a responder that can process a key press event and decides
 * whether to handle the event asynchronously.
 *
 * To use this class, add it to a |FlutterKeyboardManager| with
 * |addPrimaryResponder|.
 */
@protocol FlutterKeyPrimaryResponder

/**
 * Process the event.
 *
 * The |callback| should be called with a value that indicates whether the
 * responder has handled the given press event. The |callback| must be called
 * exactly once, and can be called before the return of this method, or after.
 */
@required
- (void)handlePress:(nonnull FlutterUIPressProxy*)press
           callback:(nonnull FlutterAsyncKeyCallback)callback API_AVAILABLE(ios(13.4));

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERKEYPRIMARYRESPONDER_H_

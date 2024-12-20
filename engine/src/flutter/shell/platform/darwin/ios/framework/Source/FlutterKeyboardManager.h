// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDMANAGER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDMANAGER_H_

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterKeyPrimaryResponder.h"

#import <Foundation/NSObject.h>
#import <UIKit/UIKit.h>

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterKeyPrimaryResponder.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterKeySecondaryResponder.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUIPressProxy.h"

typedef void (^KeyEventCompleteCallback)(bool, FlutterUIPressProxy* _Nonnull)
    API_AVAILABLE(ios(13.4));

/**
 * A hub that manages how key events are dispatched to various Flutter key
 * responders, and propagates it to the superclass if the Flutter key responders
 * do not handle it.
 *
 * This class manages one or more primary responders, as well as zero or more
 * secondary responders.
 *
 * An event that is received by |handlePresses| is first dispatched to *all*
 * primary responders. Each primary responder responds *asynchronously* with a
 * boolean, indicating whether it handles the event.
 *
 * An event that is not handled by any primary responders is then passed to to
 * the first secondary responder (in the chronological order of addition),
 * which responds *synchronously* with a boolean, indicating whether it handles
 * the event. If not, the event is passed to the next secondary responder, and
 * so on.
 *
 * The event is then handed back to the |completeCallback| from the original
 * call to |handlePresses| so that it can respond synchronously to the OS if the
 * event was not handled by the responders. The |completeCallback| is called on
 * the platform thread because the platform thread is blocked by a nested event
 * loop while the response from the framework is being collected, and it needs
 * to be called on the platform thread to unblock the thread by exiting the
 * nested event loop.
 *
 * Preventing primary responders from receiving events is not supported, because
 * in reality this class only supports two hardcoded responders
 * (FlutterChannelKeyResponder and FlutterEmbedderKeyResponder), where the only purpose
 * of supporting two is to maintain the legacy channel API during the
 * deprecation window, after which the channel responder should be removed, and
 * only one primary responder will exist.
 */
@interface FlutterKeyboardManager : NSObject
/**
 * Add a primary responder, which asynchronously decides whether to handle an
 * event.
 */
- (void)addPrimaryResponder:(nonnull id<FlutterKeyPrimaryResponder>)responder;

/**
 * Add a secondary responder, which synchronously decides whether to handle an
 * event in order if no earlier responders handle.
 */
- (void)addSecondaryResponder:(nonnull id<FlutterKeySecondaryResponder>)responder;

/**
 * Dispatches a key press event to all responders, gathering their responses,
 * and then calls the |nextAction| if the event was not handled.
 */
- (void)handlePress:(nonnull FlutterUIPressProxy*)press
         nextAction:(nonnull void (^)())next API_AVAILABLE(ios(13.4));
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDMANAGER_H_

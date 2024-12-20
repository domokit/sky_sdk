// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDMANAGER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDMANAGER_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyboardViewDelegate.h"

/**
 * Processes keyboard events and cooperate with |TextInputPlugin|.
 *
 * A keyboard event goes through a few sections, each can choose to handled the
 * event, and only unhandled events can move to the next section:
 *
 * - Pre-filtering: Events during IME are sent to the system immediately.
 * - Keyboard: Dispatch to the embedder responder and the channel responder
 *   simultaneously. After both responders have responded (asynchronously), the
 *   event is considered handled if either responder handles.
 * - Text input: Events are sent to |TextInputPlugin| and are handled
 *   synchronously.
 * - Next responder: Events are sent to the next responder as specified by
 *   |viewDelegate|.
 */
@interface FlutterKeyboardManager : NSObject

/**
 * Create a keyboard manager.
 *
 * The |viewDelegate| is a weak reference, typically implemented by
 * |FlutterViewController|.
 */
- (nonnull instancetype)initWithViewDelegate:(nonnull id<FlutterKeyboardViewDelegate>)viewDelegate;

/**
 * Processes a key event.
 *
 * Unhandled events will be dispatched to the text input system, and possibly
 * the next responder afterwards.
 */
- (void)handleEvent:(nonnull NSEvent*)event;

/**
 * Returns yes if is event currently being redispatched.
 *
 * In some instances (i.e. emoji shortcut) the event may be redelivered by cocoa
 * as key equivalent to FlutterTextInput, in which case it shouldn't be
 * processed again.
 */
- (BOOL)isDispatchingKeyEvent:(nonnull NSEvent*)event;

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

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDMANAGER_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYPRIMARYRESPONDER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYPRIMARYRESPONDER_H_

#import <Cocoa/Cocoa.h>

typedef void (^FlutterAsyncKeyCallback)(BOOL handled);

/**
 * An interface for a responder that can process a key event and decides whether
 * to handle an event asynchronously.
 *
 * To use this class, add it to a |FlutterKeyboardManager| with |addPrimaryResponder|.
 */
@protocol FlutterKeyPrimaryResponder

/**
 * Process the event.
 *
 * The |callback| should be called with a value that indicates whether the
 * responder has handled the given event. The |callback| must be called exactly
 * once, and can be called before the return of this method, or after.
 */
@required
- (void)handleEvent:(nonnull NSEvent*)event callback:(nonnull FlutterAsyncKeyCallback)callback;

/**
 * Synchronize the modifier flags if necessary. The new modifier flag would usually come from mouse
 * event and may be out of sync with current keyboard state if the modifier flags have changed while
 * window was not key.
 */
@required
- (void)syncModifiersIfNeeded:(NSEventModifierFlags)modifierFlags
                    timestamp:(NSTimeInterval)timestamp;

/* A map from macOS key code to logical keyboard.
 *
 * The map is assigned on initialization, and updated when the user changes
 * keyboard type or layout. The responder should prioritize this map when
 * deriving logical keys.
 */
@required
@property(nonatomic, nullable, strong) NSMutableDictionary<NSNumber*, NSNumber*>* layoutMap;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYPRIMARYRESPONDER_H_

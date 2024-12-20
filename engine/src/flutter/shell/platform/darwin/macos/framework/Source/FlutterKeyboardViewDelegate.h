// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDVIEWDELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDVIEWDELEGATE_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterBinaryMessenger.h"
#import "flutter/shell/platform/embedder/embedder.h"

namespace flutter {

// Signature used to notify that a keyboard layout has changed.
typedef void (^KeyboardLayoutNotifier)();

// The printable result of a key under certain modifiers, used to derive key
// mapping.
typedef struct {
  // The printable character.
  //
  // If `isDeadKey` is true, then this is the character when pressing the same
  // dead key twice.
  uint32_t character;

  // Whether this character is a dead key.
  //
  // A dead key is a key that is not counted as text per se, but holds a
  // diacritics to be added to the next key.
  bool isDeadKey;
} LayoutClue;

}  // namespace flutter

/**
 * An interface for a class that can provides |FlutterKeyboardManager| with
 * platform-related features.
 *
 * This protocol is typically implemented by |FlutterViewController|.
 */
@protocol FlutterKeyboardViewDelegate

@required
/**
 * Get the next responder to dispatch events that the keyboard system
 * (including text input) do not handle.
 *
 * If the |nextResponder| is null, then those events will be discarded.
 */
@property(nonatomic, readonly, nullable) NSResponder* nextResponder;

/**
 * Dispatch events to the framework to be processed by |HardwareKeyboard|.
 *
 * This method typically forwards events to
 * |FlutterEngine.sendKeyEvent:callback:userData:|.
 */
- (void)sendKeyEvent:(const FlutterKeyEvent&)event
            callback:(nullable FlutterKeyEventCallback)callback
            userData:(nullable void*)userData;

/**
 * Get a binary messenger to send channel messages with.
 *
 * This method is used to create the key data channel and typically
 * forwards to |FlutterEngine.binaryMessenger|.
 */
- (nonnull id<FlutterBinaryMessenger>)getBinaryMessenger;

/**
 * Dispatch events that are not handled by the keyboard event handlers
 * to the text input handler.
 *
 * This method typically forwards events to |TextInputPlugin.handleKeyEvent|.
 */
- (BOOL)onTextInputKeyEvent:(nonnull NSEvent*)event;

/**
 * Add a listener that is called whenever the user changes keyboard layout.
 *
 * Only one listeners is supported. Adding new ones overwrites the current one.
 * Assigning nil unsubscribes.
 */
- (void)subscribeToKeyboardLayoutChange:(nullable flutter::KeyboardLayoutNotifier)callback;

/**
 * Querying the printable result of a key under the given modifier state.
 */
- (flutter::LayoutClue)lookUpLayoutForKeyCode:(uint16_t)keyCode shift:(BOOL)shift;

/**
 * Returns the keyboard pressed state.
 *
 * Returns the keyboard pressed state. The dictionary contains one entry per
 * pressed keys, mapping from the logical key to the physical key.
 */
- (nonnull NSDictionary*)getPressedState;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDVIEWDELEGATE_H_

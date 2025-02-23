// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDLAYOUT_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDLAYOUT_H_

#import <AppKit/AppKit.h>

namespace flutter {

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

@protocol FlutterKeyboardLayoutDelegate

/**
 * Called when the active keyboard input source changes.
 *
 * Input sources may be simple keyboard layouts, or more complex input methods involving an IME,
 * such as Chinese, Japanese, and Korean.
 */
- (void)keyboardLayoutDidChange;

@end

@interface FlutterKeyboardLayout : NSObject

@property(readwrite, nonatomic, weak) id<FlutterKeyboardLayoutDelegate> delegate;

/**
 * Querying the printable result of a key under the given modifier state.
 */
- (flutter::LayoutClue)lookUpLayoutForKeyCode:(uint16_t)keyCode shift:(BOOL)shift;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERKEYBOARDLAYOUT_H_

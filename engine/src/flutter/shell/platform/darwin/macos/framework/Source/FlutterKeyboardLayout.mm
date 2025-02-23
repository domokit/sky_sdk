// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyboardLayout.h"

#include <Carbon/Carbon.h>
#include <cctype>
#include "flutter/fml/platform/darwin/cf_utils.h"

@implementation FlutterKeyboardLayout {
  NSData* _keyboardLayoutData;
}

@synthesize delegate = _delegate;

/**
 * Returns the current Unicode layout data (kTISPropertyUnicodeKeyLayoutData).
 *
 * To use the returned data, convert it to CFDataRef first, finds its bytes
 * with CFDataGetBytePtr, then reinterpret it into const UCKeyboardLayout*.
 * It's returned in NSData* to enable auto reference count.
 */
static NSData* CurrentKeyboardLayoutData() {
  fml::CFRef<TISInputSourceRef> source(TISCopyCurrentKeyboardInputSource());
  CFTypeRef layout_data = TISGetInputSourceProperty(source, kTISPropertyUnicodeKeyLayoutData);
  if (layout_data == nil) {
    // TISGetInputSourceProperty returns null with Japanese keyboard layout.
    // Using TISCopyCurrentKeyboardLayoutInputSource to fix NULL return.
    // https://github.com/microsoft/node-native-keymap/blob/5f0699ded00179410a14c0e1b0e089fe4df8e130/src/keyboard_mac.mm#L91
    source.Reset(TISCopyCurrentKeyboardLayoutInputSource());
    layout_data = TISGetInputSourceProperty(source, kTISPropertyUnicodeKeyLayoutData);
  }
  return (__bridge NSData*)layout_data;
}

/**
 * NotificationCenter callback invoked on kTISNotifySelectedKeyboardInputSourceChanged events.
 */
static void OnKeyboardLayoutChanged(CFNotificationCenterRef center,
                                    void* observer,
                                    CFStringRef name,
                                    const void* object,
                                    CFDictionaryRef userInfo) {
  FlutterKeyboardLayout* controller = (__bridge FlutterKeyboardLayout*)observer;
  if (controller != nil) {
    [controller onKeyboardLayoutChanged];
  }
}

- (instancetype)initWithDelegate:(id<FlutterKeyboardLayoutDelegate>)delegate {
  self = [super init];
  if (self) {
    _delegate = delegate;
    // macOS fires this message when changing IMEs.
    CFNotificationCenterRef cfCenter = CFNotificationCenterGetDistributedCenter();
    __weak FlutterKeyboardLayout* weakSelf = self;
    CFNotificationCenterAddObserver(cfCenter, (__bridge void*)weakSelf, OnKeyboardLayoutChanged,
                                    kTISNotifySelectedKeyboardInputSourceChanged, NULL,
                                    CFNotificationSuspensionBehaviorDeliverImmediately);
  }
  return self;
}

- (void)dealloc {
  CFNotificationCenterRef cfCenter = CFNotificationCenterGetDistributedCenter();
  CFNotificationCenterRemoveEveryObserver(cfCenter, (__bridge void*)self);
}

- (void)onKeyboardLayoutChanged {
  _keyboardLayoutData = nil;
  [_delegate keyboardLayoutDidChange];
}

- (flutter::LayoutClue)lookUpLayoutForKeyCode:(uint16_t)keyCode shift:(BOOL)shift {
  if (_keyboardLayoutData == nil) {
    _keyboardLayoutData = CurrentKeyboardLayoutData();
  }
  const UCKeyboardLayout* layout = reinterpret_cast<const UCKeyboardLayout*>(
      CFDataGetBytePtr((__bridge CFDataRef)_keyboardLayoutData));

  UInt32 deadKeyState = 0;
  UniCharCount stringLength = 0;
  UniChar resultChar;

  UInt32 modifierState = ((shift ? shiftKey : 0) >> 8) & 0xFF;
  UInt32 keyboardType = LMGetKbdLast();

  bool isDeadKey = false;
  OSStatus status =
      UCKeyTranslate(layout, keyCode, kUCKeyActionDown, modifierState, keyboardType,
                     kUCKeyTranslateNoDeadKeysBit, &deadKeyState, 1, &stringLength, &resultChar);
  // For dead keys, press the same key again to get the printable representation of the key.
  if (status == noErr && stringLength == 0 && deadKeyState != 0) {
    isDeadKey = true;
    status =
        UCKeyTranslate(layout, keyCode, kUCKeyActionDown, modifierState, keyboardType,
                       kUCKeyTranslateNoDeadKeysBit, &deadKeyState, 1, &stringLength, &resultChar);
  }

  if (status == noErr && stringLength == 1 && !std::iscntrl(resultChar)) {
    return flutter::LayoutClue{resultChar, isDeadKey};
  }
  return flutter::LayoutClue{0, false};
}

@end

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Foundation/Foundation.h>
#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>
#include <_types/_uint64_t.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEmbedderKeyResponder.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterFakeKeyEvents.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/KeyCodeMap_Internal.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"

using namespace flutter::testing::keycodes;
using namespace flutter::testing;

FLUTTER_ASSERT_ARC;

#define XCTAssertStrEqual(value, expected)    \
  XCTAssertTrue(strcmp(value, expected) == 0, \
                @"String \"%s\" not equal to the expected value of \"%s\"", value, expected)

// A wrap to convert FlutterKeyEvent to a ObjC class.
@interface TestKeyEvent : NSObject
@property(nonatomic) FlutterKeyEvent* data;
@property(nonatomic) FlutterKeyEventCallback callback;
@property(nonatomic, nullable) void* userData;
@end

@implementation TestKeyEvent
- (instancetype)initWithEvent:(const FlutterKeyEvent*)event
                     callback:(nullable FlutterKeyEventCallback)callback
                     userData:(nullable void*)userData {
  self = [super init];
  _data = new FlutterKeyEvent(*event);
  if (event->character != nullptr) {
    size_t len = strlen(event->character);
    char* character = new char[len + 1];
    strlcpy(character, event->character, len + 1);
    _data->character = character;
  }
  _callback = callback;
  _userData = userData;
  return self;
}

- (BOOL)hasCallback {
  return _callback != nil;
}

- (void)respond:(BOOL)handled {
  NSAssert(
      _callback != nil,
      @"Improper call to `respond` that does not have a callback.");  // Caller's responsibility
  _callback(handled, _userData);
}

- (void)dealloc {
  if (_data->character != nullptr) {
    delete[] _data->character;
  }
  delete _data;
}
@end

namespace {
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeUndefined = (UIKeyboardHIDUsage)0x03;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeKeyA = (UIKeyboardHIDUsage)0x04;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodePeriod = (UIKeyboardHIDUsage)0x37;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeKeyW = (UIKeyboardHIDUsage)0x1a;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeShiftLeft = (UIKeyboardHIDUsage)0xe1;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeShiftRight = (UIKeyboardHIDUsage)0xe5;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeNumpad1 = (UIKeyboardHIDUsage)0x59;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeCapsLock = (UIKeyboardHIDUsage)0x39;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeF1 = (UIKeyboardHIDUsage)0x3a;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeCommandLeft = (UIKeyboardHIDUsage)0xe3;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeAltRight = (UIKeyboardHIDUsage)0xe6;
API_AVAILABLE(ios(13.4))
constexpr UIKeyboardHIDUsage kKeyCodeEject = (UIKeyboardHIDUsage)0xb8;

constexpr uint64_t kPhysicalKeyUndefined = 0x00070003;

constexpr uint64_t kLogicalKeyUndefined = 0x1300000003;

constexpr uint64_t kModifierFlagNone = 0x0;

typedef void (^ResponseCallback)(bool handled);
}  // namespace

@interface FlutterEmbedderKeyResponderTest : XCTestCase
@end

@implementation FlutterEmbedderKeyResponderTest

- (void)setUp {
  // All of these tests were designed to run on iOS 13.4 or later.
  if (@available(iOS 13.4, *)) {
  } else {
    XCTSkip(@"Required API not present for test.");
  }
}

- (void)tearDown {
}

// Test the most basic key events.
//
// Press, hold, and release key A on an US keyboard.
- (void)testBasicKeyEvent API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  __block BOOL last_handled = TRUE;
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  last_handled = FALSE;
  [responder handlePress:keyDownEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f, "a", "a")
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->timestamp, 123000000.0f);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertStrEqual(event->character, "a");
  XCTAssertEqual(event->synthesized, false);

  XCTAssertEqual(last_handled, FALSE);
  XCTAssert([[events lastObject] hasCallback]);
  [[events lastObject] respond:TRUE];
  XCTAssertEqual(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = TRUE;
  [responder handlePress:keyUpEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->timestamp, 123000000.0f);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);

  XCTAssertEqual(last_handled, TRUE);
  XCTAssert([[events lastObject] hasCallback]);
  [[events lastObject] respond:FALSE];  // Check if responding FALSE works
  XCTAssertEqual(last_handled, FALSE);

  [events removeAllObjects];
}

- (void)testIosKeyPlane API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  __block BOOL last_handled = TRUE;
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  last_handled = FALSE;
  // Verify that the eject key (keycode 0xb8, which is not present in the keymap)
  // should be translated to the right logical and physical keys.
  [responder handlePress:keyDownEvent(kKeyCodeEject, kModifierFlagNone, 123.0f)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kKeyCodeEject | kIosPlane);
  XCTAssertEqual(event->logical, kKeyCodeEject | kIosPlane);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);

  XCTAssertEqual(last_handled, FALSE);
  XCTAssert([[events lastObject] hasCallback]);
  [[events lastObject] respond:TRUE];
  XCTAssertEqual(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = TRUE;
  [responder handlePress:keyUpEvent(kKeyCodeEject, kModifierFlagNone, 123.0f)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kKeyCodeEject | kIosPlane);
  XCTAssertEqual(event->logical, kKeyCodeEject | kIosPlane);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);

  XCTAssertEqual(last_handled, TRUE);
  XCTAssert([[events lastObject] hasCallback]);
  [[events lastObject] respond:FALSE];  // Check if responding FALSE works
  XCTAssertEqual(last_handled, FALSE);

  [events removeAllObjects];
}

- (void)testOutOfOrderModifiers API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  // This tests that we synthesize the correct modifier keys when we release the
  // modifier key that created the letter before we release the letter.
  [responder handlePress:keyDownEvent(kKeyCodeAltRight, kModifierFlagAltAny, 123.0f)
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalAltRight);
  XCTAssertEqual(event->logical, kLogicalAltRight);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);

  [events removeAllObjects];

  // Test non-ASCII characters being produced.
  [responder handlePress:keyDownEvent(kKeyCodeKeyW, kModifierFlagAltAny, 123.0f, "∑", "w")
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyW);
  XCTAssertEqual(event->logical, kLogicalKeyW);
  XCTAssertStrEqual(event->character, "∑");
  XCTAssertEqual(event->synthesized, false);

  [events removeAllObjects];

  // Releasing the modifier key before the letter should send the key up to the
  // framework.
  [responder handlePress:keyUpEvent(kKeyCodeAltRight, kModifierFlagAltAny, 123.0f)
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalAltRight);
  XCTAssertEqual(event->logical, kLogicalAltRight);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);

  [events removeAllObjects];

  // Yes, iOS sends a modifier flag for the Alt key being down on this event,
  // even though the Alt (Option) key has already been released. This means that
  // for the framework to be in the correct state, we must synthesize a key down
  // event for the modifier key here, and another key up before the next key
  // event.
  [responder handlePress:keyUpEvent(kKeyCodeKeyW, kModifierFlagAltAny, 123.0f)
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyW);
  XCTAssertEqual(event->logical, kLogicalKeyW);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);

  [events removeAllObjects];

  // Here we should simulate a key up for the Alt key, since it is no longer
  // shown as down in the modifier flags.
  [responder handlePress:keyDownEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f, "å", "a")
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertStrEqual(event->character, "å");
  XCTAssertEqual(event->synthesized, false);
}

- (void)testIgnoreDuplicateDownEvent API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  __block BOOL last_handled = TRUE;
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  last_handled = FALSE;
  [responder handlePress:keyDownEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f, "a", "a")
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertStrEqual(event->character, "a");
  XCTAssertEqual(event->synthesized, false);
  XCTAssertEqual(last_handled, FALSE);
  [[events lastObject] respond:TRUE];
  XCTAssertEqual(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = FALSE;
  [responder handlePress:keyDownEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f, "a", "a")
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->physical, 0ull);
  XCTAssertEqual(event->logical, 0ull);
  XCTAssertEqual(event->synthesized, false);
  XCTAssertFalse([[events lastObject] hasCallback]);
  XCTAssertEqual(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = FALSE;
  [responder handlePress:keyUpEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  XCTAssertEqual(last_handled, FALSE);
  [[events lastObject] respond:TRUE];
  XCTAssertEqual(last_handled, TRUE);

  [events removeAllObjects];
}

- (void)testIgnoreAbruptUpEvent API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  __block BOOL last_handled = TRUE;
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  last_handled = FALSE;
  [responder handlePress:keyUpEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->physical, 0ull);
  XCTAssertEqual(event->logical, 0ull);
  XCTAssertEqual(event->synthesized, false);
  XCTAssertFalse([[events lastObject] hasCallback]);
  XCTAssertEqual(last_handled, TRUE);

  [events removeAllObjects];
}

// Press R-Shift, A, then release R-Shift then A, on a US keyboard.
//
// This is special because the characters for the A key will change in this
// process.
- (void)testToggleModifiersDuringKeyTap API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  [responder handlePress:keyDownEvent(kKeyCodeShiftRight, kModifierFlagShiftAny, 123.0f)
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);

  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->timestamp, 123000000.0f);
  XCTAssertEqual(event->physical, kPhysicalShiftRight);
  XCTAssertEqual(event->logical, kLogicalShiftRight);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder handlePress:keyDownEvent(kKeyCodeKeyA, kModifierFlagShiftAny, 123.0f, "A", "A")
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertStrEqual(event->character, "A");
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder handlePress:keyUpEvent(kKeyCodeShiftRight, kModifierFlagNone, 123.0f)
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalShiftRight);
  XCTAssertEqual(event->logical, kLogicalShiftRight);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder handlePress:keyUpEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f)
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];
}

// Special modifier flags.
//
// Some keys in modifierFlags are not to indicate modifier state, but to mark
// the key area that the key belongs to, such as numpad keys or function keys.
// Ensure these flags do not obstruct other keys.
- (void)testSpecialModiferFlags API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  FlutterKeyEvent* event;
  __block BOOL last_handled = TRUE;
  id keyEventCallback = ^(BOOL handled) {
    last_handled = handled;
  };

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  // Keydown:    Numpad1, Fn (undefined), F1, KeyA, ShiftLeft
  // Then KeyUp: Numpad1, Fn (undefined), F1, KeyA, ShiftLeft

  // Numpad 1
  // OS provides: char: "1", code: 0x59, modifiers: 0x200000
  [responder handlePress:keyDownEvent(kKeyCodeNumpad1, kModifierFlagNumPadKey, 123.0, "1", "1")
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalNumpad1);
  XCTAssertEqual(event->logical, kLogicalNumpad1);
  XCTAssertStrEqual(event->character, "1");
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // Fn Key (sends HID undefined)
  // OS provides: char: nil, keycode: 0x3, modifiers: 0x0
  [responder handlePress:keyDownEvent(kKeyCodeUndefined, kModifierFlagNone, 123.0)
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyUndefined);
  XCTAssertEqual(event->logical, kLogicalKeyUndefined);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // F1 Down
  // OS provides: char: UIKeyInputF1, code: 0x3a, modifiers: 0x0
  [responder handlePress:keyDownEvent(kKeyCodeF1, kModifierFlagNone, 123.0f, "\\^P", "\\^P")
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalF1);
  XCTAssertEqual(event->logical, kLogicalF1);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // KeyA Down
  // OS provides: char: "q", code: 0x4, modifiers: 0x0
  [responder handlePress:keyDownEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f, "a", "a")
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertStrEqual(event->character, "a");
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // ShiftLeft Down
  // OS Provides: char: nil, code: 0xe1, modifiers: 0x20000
  [responder handlePress:keyDownEvent(kKeyCodeShiftLeft, kModifierFlagShiftAny, 123.0f)
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalShiftLeft);
  XCTAssertEqual(event->logical, kLogicalShiftLeft);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);

  [events removeAllObjects];

  // Numpad 1 Up
  // OS provides: char: "1", code: 0x59, modifiers: 0x200000
  [responder handlePress:keyUpEvent(kKeyCodeNumpad1, kModifierFlagNumPadKey, 123.0f)
                callback:keyEventCallback];

  XCTAssertEqual([events count], 2u);

  // Because the OS no longer provides the 0x20000 (kModifierFlagShiftAny), we
  // have to simulate a keyup.
  event = [events firstObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalShiftLeft);
  XCTAssertEqual(event->logical, kLogicalShiftLeft);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, true);

  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalNumpad1);
  XCTAssertEqual(event->logical, kLogicalNumpad1);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // F1 Up
  // OS provides: char: UIKeyInputF1, code: 0x3a, modifiers: 0x0
  [responder handlePress:keyUpEvent(kKeyCodeF1, kModifierFlagNone, 123.0f)
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalF1);
  XCTAssertEqual(event->logical, kLogicalF1);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // Fn Key (sends HID undefined)
  // OS provides: char: nil, code: 0x3, modifiers: 0x0
  [responder handlePress:keyUpEvent(kKeyCodeUndefined, kModifierFlagNone, 123.0)
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyUndefined);
  XCTAssertEqual(event->logical, kLogicalKeyUndefined);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // KeyA Up
  // OS provides: char: "a", code: 0x4, modifiers: 0x0
  [responder handlePress:keyUpEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f)
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // ShiftLeft Up
  // OS provides: char: nil, code: 0xe1, modifiers: 0x20000
  [responder handlePress:keyUpEvent(kKeyCodeShiftLeft, kModifierFlagShiftAny, 123.0f)
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->physical, 0ull);
  XCTAssertEqual(event->logical, 0ull);
  XCTAssertEqual(event->synthesized, false);
  XCTAssertFalse([[events lastObject] hasCallback]);
  XCTAssertEqual(last_handled, TRUE);

  [events removeAllObjects];
}

- (void)testIdentifyLeftAndRightModifiers API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  [responder handlePress:keyDownEvent(kKeyCodeShiftLeft, kModifierFlagShiftAny, 123.0f)
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalShiftLeft);
  XCTAssertEqual(event->logical, kLogicalShiftLeft);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder handlePress:keyDownEvent(kKeyCodeShiftRight, kModifierFlagShiftAny, 123.0f)
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalShiftRight);
  XCTAssertEqual(event->logical, kLogicalShiftRight);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder handlePress:keyUpEvent(kKeyCodeShiftLeft, kModifierFlagShiftAny, 123.0f)
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalShiftLeft);
  XCTAssertEqual(event->logical, kLogicalShiftLeft);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder handlePress:keyUpEvent(kKeyCodeShiftRight, kModifierFlagShiftAny, 123.0f)
                callback:^(BOOL handled){
                }];

  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalShiftRight);
  XCTAssertEqual(event->logical, kLogicalShiftRight);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];
}

// Press the CapsLock key when CapsLock state is desynchronized
- (void)testSynchronizeCapsLockStateOnCapsLock API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  __block BOOL last_handled = TRUE;
  id keyEventCallback = ^(BOOL handled) {
    last_handled = handled;
  };
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  last_handled = FALSE;
  [responder handlePress:keyDownEvent(kKeyCodeKeyA, kModifierFlagCapsLock, 123.0f, "A", "A")
                callback:keyEventCallback];

  XCTAssertEqual([events count], 3u);

  event = events[0].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalCapsLock);
  XCTAssertEqual(event->logical, kLogicalCapsLock);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, true);
  XCTAssertFalse([events[0] hasCallback]);

  event = events[1].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalCapsLock);
  XCTAssertEqual(event->logical, kLogicalCapsLock);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, true);
  XCTAssertFalse([events[1] hasCallback]);

  event = events[2].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertStrEqual(event->character, "A");
  XCTAssertEqual(event->synthesized, false);
  XCTAssert([events[2] hasCallback]);

  XCTAssertEqual(last_handled, FALSE);
  [[events lastObject] respond:TRUE];
  XCTAssertEqual(last_handled, TRUE);

  [events removeAllObjects];

  // Release the "A" key.
  [responder handlePress:keyUpEvent(kKeyCodeKeyA, kModifierFlagCapsLock, 123.0f)
                callback:keyEventCallback];
  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertEqual(event->synthesized, false);

  [events removeAllObjects];

  // In:  CapsLock down
  // Out: CapsLock down
  last_handled = FALSE;
  [responder handlePress:keyDownEvent(kKeyCodeCapsLock, kModifierFlagCapsLock, 123.0f)
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events firstObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalCapsLock);
  XCTAssertEqual(event->logical, kLogicalCapsLock);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  XCTAssert([[events firstObject] hasCallback]);

  [events removeAllObjects];

  // In:  CapsLock up
  // Out: CapsLock up
  // This turns off the caps lock, triggering a synthesized up/down to tell the
  // framework that.
  last_handled = FALSE;
  [responder handlePress:keyUpEvent(kKeyCodeCapsLock, kModifierFlagCapsLock, 123.0f)
                callback:keyEventCallback];

  XCTAssertEqual([events count], 1u);
  event = [events firstObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalCapsLock);
  XCTAssertEqual(event->logical, kLogicalCapsLock);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  XCTAssert([[events firstObject] hasCallback]);

  [events removeAllObjects];

  last_handled = FALSE;
  [responder handlePress:keyDownEvent(kKeyCodeKeyA, kModifierFlagNone, 123.0f, "a", "a")
                callback:keyEventCallback];

  // Just to make sure that we aren't simulating events now, since the state is
  // consistent, and should be off.
  XCTAssertEqual([events count], 1u);
  event = [events lastObject].data;
  XCTAssertNotEqual(event, nullptr);
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertStrEqual(event->character, "a");
  XCTAssertEqual(event->synthesized, false);
  XCTAssert([[events firstObject] hasCallback]);
}

// Press the CapsLock key when CapsLock state is desynchronized
- (void)testSynchronizeCapsLockStateOnNormalKey API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  __block BOOL last_handled = TRUE;
  id keyEventCallback = ^(BOOL handled) {
    last_handled = handled;
  };
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  last_handled = FALSE;
  [responder handlePress:keyDownEvent(kKeyCodeKeyA, kModifierFlagCapsLock, 123.0f, "A", "a")
                callback:keyEventCallback];

  XCTAssertEqual([events count], 3u);

  event = events[0].data;
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalCapsLock);
  XCTAssertEqual(event->logical, kLogicalCapsLock);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, true);
  XCTAssertFalse([events[0] hasCallback]);

  event = events[1].data;
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalCapsLock);
  XCTAssertEqual(event->logical, kLogicalCapsLock);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, true);
  XCTAssertFalse([events[1] hasCallback]);

  event = events[2].data;
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalKeyA);
  XCTAssertEqual(event->logical, kLogicalKeyA);
  XCTAssertStrEqual(event->character, "A");
  XCTAssertEqual(event->synthesized, false);
  XCTAssert([events[2] hasCallback]);

  XCTAssertEqual(last_handled, FALSE);
  [[events lastObject] respond:TRUE];
  XCTAssertEqual(last_handled, TRUE);

  [events removeAllObjects];
}

// Press Cmd-. should correctly result in an Escape event.
- (void)testCommandPeriodKey API_AVAILABLE(ios(13.4)) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  id keyEventCallback = ^(BOOL handled) {
  };
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event callback:nil userData:nil]];
        callback(true, user_data);
      }];

  // MetaLeft down.
  [responder handlePress:keyDownEvent(kKeyCodeCommandLeft, kModifierFlagMetaAny, 123.0f, "", "")
                callback:keyEventCallback];
  XCTAssertEqual([events count], 1u);
  event = events[0].data;
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalMetaLeft);
  XCTAssertEqual(event->logical, kLogicalMetaLeft);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [events removeAllObjects];

  // Period down, which is logically Escape.
  [responder handlePress:keyDownEvent(kKeyCodePeriod, kModifierFlagMetaAny, 123.0f,
                                      "UIKeyInputEscape", "UIKeyInputEscape")
                callback:keyEventCallback];
  XCTAssertEqual([events count], 1u);
  event = events[0].data;
  XCTAssertEqual(event->type, kFlutterKeyEventTypeDown);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalPeriod);
  XCTAssertEqual(event->logical, kLogicalEscape);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [events removeAllObjects];

  // Period up, which unconventionally has characters.
  [responder handlePress:keyUpEvent(kKeyCodePeriod, kModifierFlagMetaAny, 123.0f,
                                    "UIKeyInputEscape", "UIKeyInputEscape")
                callback:keyEventCallback];
  XCTAssertEqual([events count], 1u);
  event = events[0].data;
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalPeriod);
  XCTAssertEqual(event->logical, kLogicalEscape);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [events removeAllObjects];

  // MetaLeft up.
  [responder handlePress:keyUpEvent(kKeyCodeCommandLeft, kModifierFlagMetaAny, 123.0f, "", "")
                callback:keyEventCallback];
  XCTAssertEqual([events count], 1u);
  event = events[0].data;
  XCTAssertEqual(event->type, kFlutterKeyEventTypeUp);
  XCTAssertEqual(event->device_type, kFlutterKeyEventDeviceTypeKeyboard);
  XCTAssertEqual(event->physical, kPhysicalMetaLeft);
  XCTAssertEqual(event->logical, kLogicalMetaLeft);
  XCTAssertEqual(event->character, nullptr);
  XCTAssertEqual(event->synthesized, false);
  [events removeAllObjects];
}

@end

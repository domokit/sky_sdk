// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Foundation/Foundation.h>

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEmbedderKeyResponder.h"
#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#import "flutter/testing/testing.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

// A wrap to convert FlutterKeyEvent to a ObjC class.
@interface TestKeyEvent : NSObject
@property(nonatomic) FlutterKeyEvent* data;
@property(nonatomic) FlutterKeyEventCallback callback;
@property(nonatomic) void* userData;
- (nonnull instancetype)initWithEvent:(const FlutterKeyEvent*)event
                             callback:(nullable FlutterKeyEventCallback)callback
                             userData:(nullable void*)userData;
- (BOOL)hasCallback;
- (void)respond:(BOOL)handled;
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
    strlcpy(character, event->character, sizeof(character));
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

namespace flutter::testing {

namespace {
constexpr uint64_t kKeyCodeKeyA = 0x00;
constexpr uint64_t kKeyCodeKeyW = 0x0d;
constexpr uint64_t kKeyCodeShiftLeft = 0x38;
constexpr uint64_t kKeyCodeShiftRight = 0x3c;
constexpr uint64_t kKeyCodeCapsLock = 0x39;
constexpr uint64_t kKeyCodeNumpad1 = 0x53;
constexpr uint64_t kKeyCodeF1 = 0x7a;
constexpr uint64_t kKeyCodeAltRight = 0x3d;

using namespace ::flutter::testing::keycodes;

typedef void (^ResponseCallback)(bool handled);

NSEvent* keyEvent(NSEventType type,
                  NSEventModifierFlags modifierFlags,
                  NSString* characters,
                  NSString* charactersIgnoringModifiers,
                  BOOL isARepeat,
                  unsigned short keyCode) {
  return [NSEvent keyEventWithType:type
                          location:NSZeroPoint
                     modifierFlags:modifierFlags
                         timestamp:0
                      windowNumber:0
                           context:nil
                        characters:characters
       charactersIgnoringModifiers:charactersIgnoringModifiers
                         isARepeat:isARepeat
                           keyCode:keyCode];
}

NSEvent* keyEvent(NSEventType type,
                  NSTimeInterval timestamp,
                  NSEventModifierFlags modifierFlags,
                  NSString* characters,
                  NSString* charactersIgnoringModifiers,
                  BOOL isARepeat,
                  unsigned short keyCode) {
  return [NSEvent keyEventWithType:type
                          location:NSZeroPoint
                     modifierFlags:modifierFlags
                         timestamp:timestamp
                      windowNumber:0
                           context:nil
                        characters:characters
       charactersIgnoringModifiers:charactersIgnoringModifiers
                         isARepeat:isARepeat
                           keyCode:keyCode];
}

}  // namespace

// Test the most basic key events.
//
// Press, hold, and release key A on an US keyboard.
TEST(FlutterEmbedderKeyResponderUnittests, BasicKeyEvent) {
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
  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 123.0f, 0x100, @"a", @"a", FALSE, 0)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->timestamp, 123000000.0f);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  EXPECT_EQ(last_handled, FALSE);
  EXPECT_TRUE([[events lastObject] hasCallback]);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = FALSE;
  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x100, @"a", @"a", TRUE, kKeyCodeKeyA)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeRepeat);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  EXPECT_EQ(last_handled, FALSE);
  EXPECT_TRUE([[events lastObject] hasCallback]);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = TRUE;
  [responder handleEvent:keyEvent(NSEventTypeKeyUp, 124.0f, 0x100, @"a", @"a", FALSE, kKeyCodeKeyA)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->timestamp, 124000000.0f);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_EQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  EXPECT_EQ(last_handled, TRUE);
  EXPECT_TRUE([[events lastObject] hasCallback]);
  [[events lastObject] respond:FALSE];  // Check if responding FALSE works
  EXPECT_EQ(last_handled, FALSE);

  [events removeAllObjects];
}

TEST(FlutterEmbedderKeyResponderUnittests, NonAsciiCharacters) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x80140, @"", @"", FALSE, kKeyCodeAltRight)
         callback:^(BOOL handled){
         }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalAltRight);
  EXPECT_EQ(event->logical, kLogicalAltRight);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  [events removeAllObjects];

  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x80140, @"∑", @"w", FALSE, kKeyCodeKeyW)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyW);
  EXPECT_EQ(event->logical, kLogicalKeyW);
  EXPECT_STREQ(event->character, "∑");
  EXPECT_EQ(event->synthesized, false);

  [events removeAllObjects];

  [responder handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", FALSE, kKeyCodeAltRight)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalAltRight);
  EXPECT_EQ(event->logical, kLogicalAltRight);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  [events removeAllObjects];

  [responder handleEvent:keyEvent(NSEventTypeKeyUp, 0x100, @"w", @"w", FALSE, kKeyCodeKeyW)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalKeyW);
  EXPECT_EQ(event->logical, kLogicalKeyW);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  [events removeAllObjects];
}

TEST(FlutterEmbedderKeyResponderUnittests, MultipleCharacters) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0, @"àn", @"àn", FALSE, kKeyCodeKeyA)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, 0x1400000000ull);
  EXPECT_STREQ(event->character, "àn");
  EXPECT_EQ(event->synthesized, false);

  [events removeAllObjects];

  [responder handleEvent:keyEvent(NSEventTypeKeyUp, 0, @"a", @"a", FALSE, kKeyCodeKeyA)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, 0x1400000000ull);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  [events removeAllObjects];
}

TEST(FlutterEmbedderKeyResponderUnittests, SynthesizeForDuplicateDownEvent) {
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

  last_handled = TRUE;
  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x100, @"a", @"a", FALSE, kKeyCodeKeyA)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);
  EXPECT_EQ(last_handled, TRUE);
  [[events lastObject] respond:FALSE];
  EXPECT_EQ(last_handled, FALSE);

  [events removeAllObjects];

  last_handled = TRUE;
  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x100, @"à", @"à", FALSE, kKeyCodeKeyA)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  EXPECT_EQ([events count], 2u);

  event = [events firstObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, NULL);
  EXPECT_EQ(event->synthesized, true);

  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, 0xE0ull /* à */);
  EXPECT_STREQ(event->character, "à");
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:FALSE];
  EXPECT_EQ(last_handled, FALSE);

  [events removeAllObjects];
}

TEST(FlutterEmbedderKeyResponderUnittests, IgnoreDuplicateUpEvent) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  FlutterKeyEvent* event;
  __block BOOL last_handled = TRUE;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  last_handled = FALSE;
  [responder handleEvent:keyEvent(NSEventTypeKeyUp, 0x100, @"a", @"a", FALSE, kKeyCodeKeyA)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  EXPECT_EQ([events count], 1u);
  EXPECT_EQ(last_handled, TRUE);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->physical, 0ull);
  EXPECT_EQ(event->logical, 0ull);
  EXPECT_FALSE([[events lastObject] hasCallback]);
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];
}

TEST(FlutterEmbedderKeyResponderUnittests, ConvertAbruptRepeatEventsToDown) {
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

  last_handled = TRUE;
  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x100, @"a", @"a", TRUE, kKeyCodeKeyA)
                callback:^(BOOL handled) {
                  last_handled = handled;
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);
  EXPECT_EQ(last_handled, TRUE);
  [[events lastObject] respond:FALSE];
  EXPECT_EQ(last_handled, FALSE);

  [events removeAllObjects];
}

// Press L shift, A, then release L shift then A, on an US keyboard.
//
// This is special because the characters for the A key will change in this
// process.
TEST(FlutterEmbedderKeyResponderUnittests, ToggleModifiersDuringKeyTap) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  [responder handleEvent:keyEvent(NSEventTypeFlagsChanged, 123.0f, 0x20104, @"", @"", FALSE,
                                  kKeyCodeShiftRight)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->timestamp, 123000000.0f);
  EXPECT_EQ(event->physical, kPhysicalShiftRight);
  EXPECT_EQ(event->logical, kLogicalShiftRight);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x20104, @"A", @"A", FALSE, kKeyCodeKeyA)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "A");
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x20104, @"A", @"A", TRUE, kKeyCodeKeyA)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeRepeat);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "A");
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", FALSE, kKeyCodeShiftRight)
         callback:^(BOOL handled){
         }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftRight);
  EXPECT_EQ(event->logical, kLogicalShiftRight);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x100, @"a", @"a", TRUE, kKeyCodeKeyA)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeRepeat);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);

  [events removeAllObjects];

  [responder handleEvent:keyEvent(NSEventTypeKeyUp, 0x100, @"a", @"a", FALSE, kKeyCodeKeyA)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];
}

// Special modifier flags.
//
// Some keys in modifierFlags are not to indicate modifier state, but to mark
// the key area that the key belongs to, such as numpad keys or function keys.
// Ensure these flags do not obstruct other keys.
TEST(FlutterEmbedderKeyResponderUnittests, SpecialModiferFlags) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  // Keydown:    Numpad1, F1, KeyA, ShiftLeft
  // Then KeyUp: Numpad1, F1, KeyA, ShiftLeft

  // Numpad 1
  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x200100, @"1", @"1", FALSE, kKeyCodeNumpad1)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalNumpad1);
  EXPECT_EQ(event->logical, kLogicalNumpad1);
  EXPECT_STREQ(event->character, "1");
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // F1
  [responder
      handleEvent:keyEvent(NSEventTypeKeyDown, 0x800100, @"\uf704", @"\uf704", FALSE, kKeyCodeF1)
         callback:^(BOOL handled){
         }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalF1);
  EXPECT_EQ(event->logical, kLogicalF1);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // KeyA
  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x100, @"a", @"a", FALSE, kKeyCodeKeyA)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "a");
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // ShiftLeft
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x20102, @"", @"", FALSE, kKeyCodeShiftLeft)
         callback:^(BOOL handled){
         }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // Numpad 1
  [responder handleEvent:keyEvent(NSEventTypeKeyUp, 0x220102, @"1", @"1", FALSE, kKeyCodeNumpad1)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalNumpad1);
  EXPECT_EQ(event->logical, kLogicalNumpad1);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // F1
  [responder
      handleEvent:keyEvent(NSEventTypeKeyUp, 0x820102, @"\uF704", @"\uF704", FALSE, kKeyCodeF1)
         callback:^(BOOL handled){
         }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalF1);
  EXPECT_EQ(event->logical, kLogicalF1);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // KeyA
  [responder handleEvent:keyEvent(NSEventTypeKeyUp, 0x20102, @"a", @"a", FALSE, kKeyCodeKeyA)
                callback:^(BOOL handled){
                }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  // ShiftLeft
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", FALSE, kKeyCodeShiftLeft)
         callback:^(BOOL handled){
         }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];
}

TEST(FlutterEmbedderKeyResponderUnittests, IdentifyLeftAndRightModifiers) {
  __block NSMutableArray<TestKeyEvent*>* events = [[NSMutableArray<TestKeyEvent*> alloc] init];
  FlutterKeyEvent* event;

  FlutterEmbedderKeyResponder* responder = [[FlutterEmbedderKeyResponder alloc]
      initWithSendEvent:^(const FlutterKeyEvent& event, _Nullable FlutterKeyEventCallback callback,
                          void* _Nullable user_data) {
        [events addObject:[[TestKeyEvent alloc] initWithEvent:&event
                                                     callback:callback
                                                     userData:user_data]];
      }];

  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x20102, @"", @"", FALSE, kKeyCodeShiftLeft)
         callback:^(BOOL handled){
         }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x20106, @"", @"", FALSE, kKeyCodeShiftRight)
         callback:^(BOOL handled){
         }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftRight);
  EXPECT_EQ(event->logical, kLogicalShiftRight);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x20104, @"", @"", FALSE, kKeyCodeShiftLeft)
         callback:^(BOOL handled){
         }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];

  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", FALSE, kKeyCodeShiftRight)
         callback:^(BOOL handled){
         }];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftRight);
  EXPECT_EQ(event->logical, kLogicalShiftRight);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  [[events lastObject] respond:TRUE];

  [events removeAllObjects];
}

// Process various cases where pair modifier key events are missed, and the
// responder has to "guess" how to synchronize states.
//
// In the following comments, parentheses indicate missed events, while
// asterisks indicate synthesized events.
TEST(FlutterEmbedderKeyResponderUnittests, SynthesizeMissedModifierEvents) {
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

  // Case 1:
  // In:  L down, (L up), L down, L up
  // Out: L down,                 L up
  last_handled = FALSE;
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x20102, @"", @"", FALSE, kKeyCodeShiftLeft)
         callback:keyEventCallback];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  EXPECT_EQ(last_handled, FALSE);
  EXPECT_TRUE([[events lastObject] hasCallback]);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = FALSE;
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x20102, @"", @"", FALSE, kKeyCodeShiftLeft)
         callback:keyEventCallback];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->physical, 0u);
  EXPECT_EQ(event->logical, 0u);
  EXPECT_FALSE([[events lastObject] hasCallback]);
  EXPECT_EQ(last_handled, TRUE);
  [events removeAllObjects];

  last_handled = FALSE;
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", FALSE, kKeyCodeShiftLeft)
         callback:keyEventCallback];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  EXPECT_EQ(last_handled, FALSE);
  EXPECT_TRUE([[events lastObject] hasCallback]);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];

  // Case 2:
  // In:  (L down), L up
  // Out:

  last_handled = FALSE;
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", FALSE, kKeyCodeShiftLeft)
         callback:keyEventCallback];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->physical, 0u);
  EXPECT_EQ(event->logical, 0u);
  EXPECT_FALSE([[events lastObject] hasCallback]);
  EXPECT_EQ(last_handled, TRUE);
  [events removeAllObjects];

  // Case 3:
  // In:  L down, (L up), (R down), R up
  // Out: L down,                   *L up

  last_handled = FALSE;
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x20102, @"", @"", FALSE, kKeyCodeShiftLeft)
         callback:keyEventCallback];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  EXPECT_EQ(last_handled, FALSE);
  EXPECT_TRUE([[events lastObject] hasCallback]);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = FALSE;
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", FALSE, kKeyCodeShiftRight)
         callback:keyEventCallback];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, true);

  // The primary event is automatically replied with TRUE, unrelated to the received event.
  EXPECT_EQ(last_handled, TRUE);
  EXPECT_FALSE([[events lastObject] hasCallback]);

  [events removeAllObjects];

  // Case 4:
  // In:  L down, R down, (L up), R up
  // Out: L down, R down          *L up & R up

  last_handled = FALSE;
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x20102, @"", @"", FALSE, kKeyCodeShiftLeft)
         callback:keyEventCallback];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  EXPECT_EQ(last_handled, FALSE);
  EXPECT_TRUE([[events lastObject] hasCallback]);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = FALSE;
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x20106, @"", @"", FALSE, kKeyCodeShiftRight)
         callback:keyEventCallback];

  EXPECT_EQ([events count], 1u);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftRight);
  EXPECT_EQ(event->logical, kLogicalShiftRight);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  EXPECT_EQ(last_handled, FALSE);
  EXPECT_TRUE([[events lastObject] hasCallback]);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = FALSE;
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", FALSE, kKeyCodeShiftRight)
         callback:keyEventCallback];

  EXPECT_EQ([events count], 2u);
  event = [events firstObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, true);

  EXPECT_FALSE([[events firstObject] hasCallback]);

  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftRight);
  EXPECT_EQ(event->logical, kLogicalShiftRight);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);

  EXPECT_EQ(last_handled, FALSE);
  EXPECT_TRUE([[events lastObject] hasCallback]);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];
}

TEST(FlutterEmbedderKeyResponderUnittests, SynthesizeMissedModifierEventsInNormalEvents) {
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

  // In:  (LShift down), A down,           (LShift up), A up
  // Out:               *LS down & A down,              *LS up & A up

  last_handled = FALSE;
  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x20102, @"A", @"A", FALSE, kKeyCodeKeyA)
                callback:keyEventCallback];

  EXPECT_EQ([events count], 2u);
  event = [events firstObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, true);
  EXPECT_FALSE([[events firstObject] hasCallback]);

  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "A");
  EXPECT_EQ(event->synthesized, false);
  EXPECT_TRUE([[events lastObject] hasCallback]);

  EXPECT_EQ(last_handled, FALSE);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];

  last_handled = FALSE;
  [responder handleEvent:keyEvent(NSEventTypeKeyUp, 0x100, @"a", @"a", FALSE, kKeyCodeKeyA)
                callback:keyEventCallback];

  EXPECT_EQ([events count], 2u);
  event = [events firstObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalShiftLeft);
  EXPECT_EQ(event->logical, kLogicalShiftLeft);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, true);
  EXPECT_FALSE([[events firstObject] hasCallback]);

  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  EXPECT_TRUE([[events lastObject] hasCallback]);

  EXPECT_EQ(last_handled, FALSE);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];
}

TEST(FlutterEmbedderKeyResponderUnittests, ConvertCapsLockEvents) {
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

  // In:  CapsLock down
  // Out: CapsLock down & *CapsLock Up
  last_handled = FALSE;
  [responder
      handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x10100, @"", @"", FALSE, kKeyCodeCapsLock)
         callback:keyEventCallback];

  EXPECT_EQ([events count], 2u);

  event = [events firstObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalCapsLock);
  EXPECT_EQ(event->logical, kLogicalCapsLock);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  EXPECT_TRUE([[events firstObject] hasCallback]);

  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalCapsLock);
  EXPECT_EQ(event->logical, kLogicalCapsLock);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, true);
  EXPECT_FALSE([[events lastObject] hasCallback]);

  EXPECT_EQ(last_handled, FALSE);
  [[events firstObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];

  // In:  CapsLock up
  // Out: CapsLock down & *CapsLock Up
  last_handled = FALSE;
  [responder handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", FALSE, kKeyCodeCapsLock)
                callback:keyEventCallback];

  EXPECT_EQ([events count], 2u);

  event = [events firstObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalCapsLock);
  EXPECT_EQ(event->logical, kLogicalCapsLock);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, false);
  EXPECT_TRUE([[events firstObject] hasCallback]);

  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalCapsLock);
  EXPECT_EQ(event->logical, kLogicalCapsLock);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, true);
  EXPECT_FALSE([[events lastObject] hasCallback]);

  EXPECT_EQ(last_handled, FALSE);
  [[events firstObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];
}

// Press the CapsLock key when CapsLock state is desynchronized
TEST(FlutterEmbedderKeyResponderUnittests, SynchronizeCapsLockStateOnCapsLock) {
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

  // In:  CapsLock down
  // Out: (empty)
  last_handled = FALSE;
  [responder handleEvent:keyEvent(NSEventTypeFlagsChanged, 0x100, @"", @"", FALSE, kKeyCodeCapsLock)
                callback:keyEventCallback];

  EXPECT_EQ([events count], 1u);
  EXPECT_EQ(last_handled, TRUE);
  event = [events lastObject].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->physical, 0ull);
  EXPECT_EQ(event->logical, 0ull);
  EXPECT_FALSE([[events lastObject] hasCallback]);
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];
}

// Press the CapsLock key when CapsLock state is desynchronized
TEST(FlutterEmbedderKeyResponderUnittests, SynchronizeCapsLockStateOnNormalKey) {
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
  [responder handleEvent:keyEvent(NSEventTypeKeyDown, 0x10100, @"A", @"a", FALSE, kKeyCodeKeyA)
                callback:keyEventCallback];

  EXPECT_EQ([events count], 3u);

  event = events[0].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalCapsLock);
  EXPECT_EQ(event->logical, kLogicalCapsLock);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, true);
  EXPECT_FALSE([events[0] hasCallback]);

  event = events[1].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
  EXPECT_EQ(event->physical, kPhysicalCapsLock);
  EXPECT_EQ(event->logical, kLogicalCapsLock);
  EXPECT_STREQ(event->character, nullptr);
  EXPECT_EQ(event->synthesized, true);
  EXPECT_FALSE([events[1] hasCallback]);

  event = events[2].data;
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(event->physical, kPhysicalKeyA);
  EXPECT_EQ(event->logical, kLogicalKeyA);
  EXPECT_STREQ(event->character, "A");
  EXPECT_EQ(event->synthesized, false);
  EXPECT_TRUE([events[2] hasCallback]);

  EXPECT_EQ(last_handled, FALSE);
  [[events lastObject] respond:TRUE];
  EXPECT_EQ(last_handled, TRUE);

  [events removeAllObjects];
}

}  // namespace flutter::testing

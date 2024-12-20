// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <Carbon/Carbon.h>
#import <Foundation/Foundation.h>
#import <OCMock/OCMock.h>

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyPrimaryResponder.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyboardManager.h"
#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#import "flutter/testing/testing.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

namespace {

using flutter::testing::keycodes::kLogicalBracketLeft;
using flutter::testing::keycodes::kLogicalDigit1;
using flutter::testing::keycodes::kLogicalDigit2;
using flutter::testing::keycodes::kLogicalKeyA;
using flutter::testing::keycodes::kLogicalKeyM;
using flutter::testing::keycodes::kLogicalKeyQ;
using flutter::testing::keycodes::kLogicalKeyT;
using flutter::testing::keycodes::kPhysicalKeyA;

using flutter::LayoutClue;

typedef BOOL (^BoolGetter)();
typedef void (^AsyncKeyCallbackHandler)(FlutterAsyncKeyCallback callback);
typedef void (^AsyncEmbedderCallbackHandler)(const FlutterKeyEvent* event,
                                             FlutterAsyncKeyCallback callback);
typedef BOOL (^TextInputCallback)(NSEvent*);

// When the Vietnamese IME converts messages into "pure text" messages, their
// key codes are set to "empty".
//
// The 0 also happens to be the key code for key A.
constexpr uint16_t kKeyCodeEmpty = 0x00;

// Constants used for `recordCallTypesTo:forTypes:`.
constexpr uint32_t kEmbedderCall = 0x1;
constexpr uint32_t kChannelCall = 0x2;
constexpr uint32_t kTextCall = 0x4;

// All key clues for a keyboard layout.
//
// The index is (keyCode * 2 + hasShift). The value is 0xMNNNN, where:
//
//  - M is whether the key is a dead key (0x1 for true, 0x0 for false).
//  - N is the character for this key. (It only supports UTF-16, but we don't
//    need full UTF-32 support for unit tests. Moreover, Carbon's UCKeyTranslate
//    only returns UniChar (UInt16) anyway.)
typedef const std::array<uint32_t, 256> MockLayoutData;

// The following layout data is generated using DEBUG_PRINT_LAYOUT.

MockLayoutData kUsLayout = {
    //         +0x0     Shift    +0x1     Shift    +0x2     Shift    +0x3     Shift
    /* 0x00 */ 0x00061, 0x00041, 0x00073, 0x00053, 0x00064, 0x00044, 0x00066, 0x00046,
    /* 0x04 */ 0x00068, 0x00048, 0x00067, 0x00047, 0x0007a, 0x0005a, 0x00078, 0x00058,
    /* 0x08 */ 0x00063, 0x00043, 0x00076, 0x00056, 0x000a7, 0x000b1, 0x00062, 0x00042,
    /* 0x0c */ 0x00071, 0x00051, 0x00077, 0x00057, 0x00065, 0x00045, 0x00072, 0x00052,
    /* 0x10 */ 0x00079, 0x00059, 0x00074, 0x00054, 0x00031, 0x00021, 0x00032, 0x00040,
    /* 0x14 */ 0x00033, 0x00023, 0x00034, 0x00024, 0x00036, 0x0005e, 0x00035, 0x00025,
    /* 0x18 */ 0x0003d, 0x0002b, 0x00039, 0x00028, 0x00037, 0x00026, 0x0002d, 0x0005f,
    /* 0x1c */ 0x00038, 0x0002a, 0x00030, 0x00029, 0x0005d, 0x0007d, 0x0006f, 0x0004f,
    /* 0x20 */ 0x00075, 0x00055, 0x0005b, 0x0007b, 0x00069, 0x00049, 0x00070, 0x00050,
    /* 0x24 */ 0x00000, 0x00000, 0x0006c, 0x0004c, 0x0006a, 0x0004a, 0x00027, 0x00022,
    /* 0x28 */ 0x0006b, 0x0004b, 0x0003b, 0x0003a, 0x0005c, 0x0007c, 0x0002c, 0x0003c,
    /* 0x2c */ 0x0002f, 0x0003f, 0x0006e, 0x0004e, 0x0006d, 0x0004d, 0x0002e, 0x0003e,
    /* 0x30 */ 0x00000, 0x00000, 0x00020, 0x00020, 0x00060, 0x0007e,
};

MockLayoutData kFrenchLayout = {
    //         +0x0     Shift    +0x1     Shift    +0x2     Shift    +0x3     Shift
    /* 0x00 */ 0x00071, 0x00051, 0x00073, 0x00053, 0x00064, 0x00044, 0x00066, 0x00046,
    /* 0x04 */ 0x00068, 0x00048, 0x00067, 0x00047, 0x00077, 0x00057, 0x00078, 0x00058,
    /* 0x08 */ 0x00063, 0x00043, 0x00076, 0x00056, 0x00040, 0x00023, 0x00062, 0x00042,
    /* 0x0c */ 0x00061, 0x00041, 0x0007a, 0x0005a, 0x00065, 0x00045, 0x00072, 0x00052,
    /* 0x10 */ 0x00079, 0x00059, 0x00074, 0x00054, 0x00026, 0x00031, 0x000e9, 0x00032,
    /* 0x14 */ 0x00022, 0x00033, 0x00027, 0x00034, 0x000a7, 0x00036, 0x00028, 0x00035,
    /* 0x18 */ 0x0002d, 0x0005f, 0x000e7, 0x00039, 0x000e8, 0x00037, 0x00029, 0x000b0,
    /* 0x1c */ 0x00021, 0x00038, 0x000e0, 0x00030, 0x00024, 0x0002a, 0x0006f, 0x0004f,
    /* 0x20 */ 0x00075, 0x00055, 0x1005e, 0x100a8, 0x00069, 0x00049, 0x00070, 0x00050,
    /* 0x24 */ 0x00000, 0x00000, 0x0006c, 0x0004c, 0x0006a, 0x0004a, 0x000f9, 0x00025,
    /* 0x28 */ 0x0006b, 0x0004b, 0x0006d, 0x0004d, 0x10060, 0x000a3, 0x0003b, 0x0002e,
    /* 0x2c */ 0x0003d, 0x0002b, 0x0006e, 0x0004e, 0x0002c, 0x0003f, 0x0003a, 0x0002f,
    /* 0x30 */ 0x00000, 0x00000, 0x00020, 0x00020, 0x0003c, 0x0003e,
};

MockLayoutData kRussianLayout = {
    //         +0x0     Shift    +0x1     Shift    +0x2     Shift    +0x3     Shift
    /* 0x00 */ 0x00444, 0x00424, 0x0044b, 0x0042b, 0x00432, 0x00412, 0x00430, 0x00410,
    /* 0x04 */ 0x00440, 0x00420, 0x0043f, 0x0041f, 0x0044f, 0x0042f, 0x00447, 0x00427,
    /* 0x08 */ 0x00441, 0x00421, 0x0043c, 0x0041c, 0x0003e, 0x0003c, 0x00438, 0x00418,
    /* 0x0c */ 0x00439, 0x00419, 0x00446, 0x00426, 0x00443, 0x00423, 0x0043a, 0x0041a,
    /* 0x10 */ 0x0043d, 0x0041d, 0x00435, 0x00415, 0x00031, 0x00021, 0x00032, 0x00022,
    /* 0x14 */ 0x00033, 0x02116, 0x00034, 0x00025, 0x00036, 0x0002c, 0x00035, 0x0003a,
    /* 0x18 */ 0x0003d, 0x0002b, 0x00039, 0x00028, 0x00037, 0x0002e, 0x0002d, 0x0005f,
    /* 0x1c */ 0x00038, 0x0003b, 0x00030, 0x00029, 0x0044a, 0x0042a, 0x00449, 0x00429,
    /* 0x20 */ 0x00433, 0x00413, 0x00445, 0x00425, 0x00448, 0x00428, 0x00437, 0x00417,
    /* 0x24 */ 0x00000, 0x00000, 0x00434, 0x00414, 0x0043e, 0x0041e, 0x0044d, 0x0042d,
    /* 0x28 */ 0x0043b, 0x0041b, 0x00436, 0x00416, 0x00451, 0x00401, 0x00431, 0x00411,
    /* 0x2c */ 0x0002f, 0x0003f, 0x00442, 0x00422, 0x0044c, 0x0042c, 0x0044e, 0x0042e,
    /* 0x30 */ 0x00000, 0x00000, 0x00020, 0x00020, 0x0005d, 0x0005b,
};

MockLayoutData kKhmerLayout = {
    //         +0x0     Shift    +0x1     Shift    +0x2     Shift    +0x3     Shift
    /* 0x00 */ 0x017b6, 0x017ab, 0x0179f, 0x017c3, 0x0178a, 0x0178c, 0x01790, 0x01792,
    /* 0x04 */ 0x017a0, 0x017c7, 0x01784, 0x017a2, 0x0178b, 0x0178d, 0x01781, 0x01783,
    /* 0x08 */ 0x01785, 0x01787, 0x0179c, 0x017c8, 0x00000, 0x00000, 0x01794, 0x01796,
    /* 0x0c */ 0x01786, 0x01788, 0x017b9, 0x017ba, 0x017c1, 0x017c2, 0x0179a, 0x017ac,
    /* 0x10 */ 0x01799, 0x017bd, 0x0178f, 0x01791, 0x017e1, 0x00021, 0x017e2, 0x017d7,
    /* 0x14 */ 0x017e3, 0x00022, 0x017e4, 0x017db, 0x017e6, 0x017cd, 0x017e5, 0x00025,
    /* 0x18 */ 0x017b2, 0x017ce, 0x017e9, 0x017b0, 0x017e7, 0x017d0, 0x017a5, 0x017cc,
    /* 0x1c */ 0x017e8, 0x017cf, 0x017e0, 0x017b3, 0x017aa, 0x017a7, 0x017c4, 0x017c5,
    /* 0x20 */ 0x017bb, 0x017bc, 0x017c0, 0x017bf, 0x017b7, 0x017b8, 0x01795, 0x01797,
    /* 0x24 */ 0x00000, 0x00000, 0x0179b, 0x017a1, 0x017d2, 0x01789, 0x017cb, 0x017c9,
    /* 0x28 */ 0x01780, 0x01782, 0x017be, 0x017d6, 0x017ad, 0x017ae, 0x017a6, 0x017b1,
    /* 0x2c */ 0x017ca, 0x017af, 0x01793, 0x0178e, 0x01798, 0x017c6, 0x017d4, 0x017d5,
    /* 0x30 */ 0x00000, 0x00000, 0x00020, 0x0200b, 0x000ab, 0x000bb,
};

NSEvent* keyDownEvent(unsigned short keyCode, NSString* chars = @"", NSString* charsUnmod = @"") {
  return [NSEvent keyEventWithType:NSEventTypeKeyDown
                          location:NSZeroPoint
                     modifierFlags:0x100
                         timestamp:0
                      windowNumber:0
                           context:nil
                        characters:chars
       charactersIgnoringModifiers:charsUnmod
                         isARepeat:NO
                           keyCode:keyCode];
}

NSEvent* keyUpEvent(unsigned short keyCode) {
  return [NSEvent keyEventWithType:NSEventTypeKeyUp
                          location:NSZeroPoint
                     modifierFlags:0x100
                         timestamp:0
                      windowNumber:0
                           context:nil
                        characters:@""
       charactersIgnoringModifiers:@""
                         isARepeat:NO
                           keyCode:keyCode];
}

id checkKeyDownEvent(unsigned short keyCode) {
  return [OCMArg checkWithBlock:^BOOL(id value) {
    if (![value isKindOfClass:[NSEvent class]]) {
      return NO;
    }
    NSEvent* event = value;
    return event.keyCode == keyCode;
  }];
}

// Clear a list of `FlutterKeyEvent` whose `character` is dynamically allocated.
void clearEvents(std::vector<FlutterKeyEvent>& events) {
  for (FlutterKeyEvent& event : events) {
    if (event.character != nullptr) {
      delete[] event.character;
    }
  }
  events.clear();
}

#define VERIFY_DOWN(OUT_LOGICAL, OUT_CHAR)                          \
  EXPECT_EQ(events[0].type, kFlutterKeyEventTypeDown);              \
  EXPECT_EQ(events[0].logical, static_cast<uint64_t>(OUT_LOGICAL)); \
  EXPECT_STREQ(events[0].character, (OUT_CHAR));                    \
  clearEvents(events);

}  // namespace

@interface KeyboardTester : NSObject
- (nonnull instancetype)init;

// Set embedder calls to respond immediately with the given response.
- (void)respondEmbedderCallsWith:(BOOL)response;

// Record embedder calls to the given storage.
//
// They are not responded to until the stored callbacks are manually called.
- (void)recordEmbedderCallsTo:(nonnull NSMutableArray<FlutterAsyncKeyCallback>*)storage;

- (void)recordEmbedderEventsTo:(nonnull std::vector<FlutterKeyEvent>*)storage
                     returning:(bool)handled;

// Set channel calls to respond immediately with the given response.
- (void)respondChannelCallsWith:(BOOL)response;

// Record channel calls to the given storage.
//
// They are not responded to until the stored callbacks are manually called.
- (void)recordChannelCallsTo:(nonnull NSMutableArray<FlutterAsyncKeyCallback>*)storage;

// Set text calls to respond with the given response.
- (void)respondTextInputWith:(BOOL)response;

// At the start of any kind of call, record the call type to the given storage.
//
// Only calls that are included in `typeMask` will be added. Options are
// kEmbedderCall, kChannelCall, and kTextCall.
//
// This method does not conflict with other call settings, and the recording
// takes place before the callbacks are (or are not) invoked.
- (void)recordCallTypesTo:(nonnull NSMutableArray<NSNumber*>*)typeStorage
                 forTypes:(uint32_t)typeMask;

- (id)lastKeyboardChannelResult;

- (void)sendKeyboardChannelMessage:(NSData* _Nullable)message;

@property(readonly, nonatomic, strong) FlutterKeyboardManager* manager;
@property(nonatomic, nullable, strong) NSResponder* nextResponder;

#pragma mark - Private

- (void)handleEmbedderEvent:(const FlutterKeyEvent&)event
                   callback:(nullable FlutterKeyEventCallback)callback
                   userData:(nullable void*)userData;

- (void)handleChannelMessage:(NSString*)channel
                     message:(NSData* _Nullable)message
                 binaryReply:(FlutterBinaryReply _Nullable)callback;

- (BOOL)handleTextInputKeyEvent:(NSEvent*)event;
@end

@implementation KeyboardTester {
  AsyncEmbedderCallbackHandler _embedderHandler;
  AsyncKeyCallbackHandler _channelHandler;
  TextInputCallback _textCallback;

  NSMutableArray<NSNumber*>* _typeStorage;
  uint32_t _typeStorageMask;

  flutter::KeyboardLayoutNotifier _keyboardLayoutNotifier;
  const MockLayoutData* _currentLayout;

  id _keyboardChannelResult;
  NSObject<FlutterBinaryMessenger>* _messengerMock;
  FlutterBinaryMessageHandler _keyboardHandler;
}

- (nonnull instancetype)init {
  self = [super init];
  if (self == nil) {
    return nil;
  }

  _nextResponder = OCMClassMock([NSResponder class]);
  [self respondChannelCallsWith:FALSE];
  [self respondEmbedderCallsWith:FALSE];
  [self respondTextInputWith:FALSE];

  _currentLayout = &kUsLayout;

  _messengerMock = OCMStrictProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub([_messengerMock sendOnChannel:@"flutter/keyevent"
                                message:[OCMArg any]
                            binaryReply:[OCMArg any]])
      .andCall(self, @selector(handleChannelMessage:message:binaryReply:));
  OCMStub([_messengerMock setMessageHandlerOnChannel:@"flutter/keyboard"
                                binaryMessageHandler:[OCMArg any]])
      .andCall(self, @selector(setKeyboardChannelHandler:handler:));
  OCMStub([_messengerMock sendOnChannel:@"flutter/keyboard" message:[OCMArg any]])
      .andCall(self, @selector(handleKeyboardChannelMessage:message:));
  id viewDelegateMock = OCMStrictProtocolMock(@protocol(FlutterKeyboardViewDelegate));
  OCMStub([viewDelegateMock nextResponder]).andReturn(_nextResponder);
  OCMStub([viewDelegateMock onTextInputKeyEvent:[OCMArg any]])
      .andCall(self, @selector(handleTextInputKeyEvent:));
  OCMStub([viewDelegateMock getBinaryMessenger]).andReturn(_messengerMock);
  OCMStub([viewDelegateMock sendKeyEvent:*(const FlutterKeyEvent*)[OCMArg anyPointer]
                                callback:nil
                                userData:nil])
      .ignoringNonObjectArgs()
      .andCall(self, @selector(handleEmbedderEvent:callback:userData:));
  OCMStub([viewDelegateMock subscribeToKeyboardLayoutChange:[OCMArg any]])
      .andCall(self, @selector(onSetKeyboardLayoutNotifier:));
  OCMStub([viewDelegateMock lookUpLayoutForKeyCode:0 shift:false])
      .ignoringNonObjectArgs()
      .andCall(self, @selector(lookUpLayoutForKeyCode:shift:));

  _manager = [[FlutterKeyboardManager alloc] initWithViewDelegate:viewDelegateMock];
  return self;
}

- (id)lastKeyboardChannelResult {
  return _keyboardChannelResult;
}

- (void)respondEmbedderCallsWith:(BOOL)response {
  _embedderHandler = ^(const FlutterKeyEvent* event, FlutterAsyncKeyCallback callback) {
    callback(response);
  };
}

- (void)recordEmbedderCallsTo:(nonnull NSMutableArray<FlutterAsyncKeyCallback>*)storage {
  _embedderHandler = ^(const FlutterKeyEvent* event, FlutterAsyncKeyCallback callback) {
    [storage addObject:callback];
  };
}

- (void)recordEmbedderEventsTo:(nonnull std::vector<FlutterKeyEvent>*)storage
                     returning:(bool)handled {
  _embedderHandler = ^(const FlutterKeyEvent* event, FlutterAsyncKeyCallback callback) {
    FlutterKeyEvent newEvent = *event;
    if (event->character != nullptr) {
      size_t charLen = strlen(event->character);
      char* newCharacter = new char[charLen + 1];
      strlcpy(newCharacter, event->character, charLen + 1);
      newEvent.character = newCharacter;
    }
    storage->push_back(newEvent);
    callback(handled);
  };
}

- (void)respondChannelCallsWith:(BOOL)response {
  _channelHandler = ^(FlutterAsyncKeyCallback callback) {
    callback(response);
  };
}

- (void)recordChannelCallsTo:(nonnull NSMutableArray<FlutterAsyncKeyCallback>*)storage {
  _channelHandler = ^(FlutterAsyncKeyCallback callback) {
    [storage addObject:callback];
  };
}

- (void)respondTextInputWith:(BOOL)response {
  _textCallback = ^(NSEvent* event) {
    return response;
  };
}

- (void)recordCallTypesTo:(nonnull NSMutableArray<NSNumber*>*)typeStorage
                 forTypes:(uint32_t)typeMask {
  _typeStorage = typeStorage;
  _typeStorageMask = typeMask;
}

- (void)sendKeyboardChannelMessage:(NSData* _Nullable)message {
  [_messengerMock sendOnChannel:@"flutter/keyboard" message:message];
}

- (void)setLayout:(const MockLayoutData&)layout {
  _currentLayout = &layout;
  if (_keyboardLayoutNotifier != nil) {
    _keyboardLayoutNotifier();
  }
}

#pragma mark - Private

- (void)handleEmbedderEvent:(const FlutterKeyEvent&)event
                   callback:(nullable FlutterKeyEventCallback)callback
                   userData:(nullable void*)userData {
  if (_typeStorage != nil && (_typeStorageMask & kEmbedderCall) != 0) {
    [_typeStorage addObject:@(kEmbedderCall)];
  }
  if (callback != nullptr) {
    _embedderHandler(&event, ^(BOOL handled) {
      callback(handled, userData);
    });
  }
}

- (void)handleChannelMessage:(NSString*)channel
                     message:(NSData* _Nullable)message
                 binaryReply:(FlutterBinaryReply _Nullable)callback {
  if (_typeStorage != nil && (_typeStorageMask & kChannelCall) != 0) {
    [_typeStorage addObject:@(kChannelCall)];
  }
  _channelHandler(^(BOOL handled) {
    NSDictionary* result = @{
      @"handled" : @(handled),
    };
    NSData* encodedKeyEvent = [[FlutterJSONMessageCodec sharedInstance] encode:result];
    callback(encodedKeyEvent);
  });
}

- (void)handleKeyboardChannelMessage:(NSString*)channel message:(NSData* _Nullable)message {
  _keyboardHandler(message, ^(id result) {
    _keyboardChannelResult = result;
  });
}

- (BOOL)handleTextInputKeyEvent:(NSEvent*)event {
  if (_typeStorage != nil && (_typeStorageMask & kTextCall) != 0) {
    [_typeStorage addObject:@(kTextCall)];
  }
  return _textCallback(event);
}

- (void)onSetKeyboardLayoutNotifier:(nullable flutter::KeyboardLayoutNotifier)callback {
  _keyboardLayoutNotifier = callback;
}

- (LayoutClue)lookUpLayoutForKeyCode:(uint16_t)keyCode shift:(BOOL)shift {
  uint32_t cluePair = (*_currentLayout)[(keyCode * 2) + (shift ? 1 : 0)];
  const uint32_t kCharMask = 0xffff;
  const uint32_t kDeadKeyMask = 0x10000;
  return LayoutClue{cluePair & kCharMask, (cluePair & kDeadKeyMask) != 0};
}

- (void)setKeyboardChannelHandler:(NSString*)channel handler:(FlutterBinaryMessageHandler)handler {
  _keyboardHandler = handler;
}

@end

@interface FlutterKeyboardManagerUnittestsObjC : NSObject
- (bool)singlePrimaryResponder;
- (bool)doublePrimaryResponder;
- (bool)textInputPlugin;
- (bool)emptyNextResponder;
- (bool)getPressedState;
- (bool)keyboardChannelGetPressedState;
- (bool)racingConditionBetweenKeyAndText;
- (bool)correctLogicalKeyForLayouts;
- (bool)shouldNotHoldStrongReferenceToViewDelegate;
@end

namespace flutter::testing {

TEST(FlutterKeyboardManagerUnittests, SinglePrimaryResponder) {
  ASSERT_TRUE([[FlutterKeyboardManagerUnittestsObjC alloc] singlePrimaryResponder]);
}

TEST(FlutterKeyboardManagerUnittests, DoublePrimaryResponder) {
  ASSERT_TRUE([[FlutterKeyboardManagerUnittestsObjC alloc] doublePrimaryResponder]);
}

TEST(FlutterKeyboardManagerUnittests, SingleFinalResponder) {
  ASSERT_TRUE([[FlutterKeyboardManagerUnittestsObjC alloc] textInputPlugin]);
}

TEST(FlutterKeyboardManagerUnittests, EmptyNextResponder) {
  ASSERT_TRUE([[FlutterKeyboardManagerUnittestsObjC alloc] emptyNextResponder]);
}

TEST(FlutterKeyboardManagerUnittests, GetPressedState) {
  ASSERT_TRUE([[FlutterKeyboardManagerUnittestsObjC alloc] getPressedState]);
}

TEST(FlutterKeyboardManagerUnittests, KeyboardChannelGetPressedState) {
  ASSERT_TRUE([[FlutterKeyboardManagerUnittestsObjC alloc] keyboardChannelGetPressedState]);
}

TEST(FlutterKeyboardManagerUnittests, RacingConditionBetweenKeyAndText) {
  ASSERT_TRUE([[FlutterKeyboardManagerUnittestsObjC alloc] racingConditionBetweenKeyAndText]);
}

TEST(FlutterKeyboardManagerUnittests, CorrectLogicalKeyForLayouts) {
  ASSERT_TRUE([[FlutterKeyboardManagerUnittestsObjC alloc] correctLogicalKeyForLayouts]);
}

TEST(FlutterKeyboardManagerUnittests, ShouldNotHoldStrongReferenceToViewDelegate) {
  ASSERT_TRUE(
      [[FlutterKeyboardManagerUnittestsObjC alloc] shouldNotHoldStrongReferenceToViewDelegate]);
}

}  // namespace flutter::testing

@implementation FlutterKeyboardManagerUnittestsObjC

- (bool)singlePrimaryResponder {
  KeyboardTester* tester = [[KeyboardTester alloc] init];
  NSMutableArray<FlutterAsyncKeyCallback>* embedderCallbacks =
      [NSMutableArray<FlutterAsyncKeyCallback> array];
  [tester recordEmbedderCallsTo:embedderCallbacks];

  // Case: The responder reports FALSE
  [tester.manager handleEvent:keyDownEvent(0x50)];
  EXPECT_EQ([embedderCallbacks count], 1u);
  embedderCallbacks[0](FALSE);
  OCMVerify([tester.nextResponder keyDown:checkKeyDownEvent(0x50)]);
  [embedderCallbacks removeAllObjects];

  // Case: The responder reports TRUE
  [tester.manager handleEvent:keyUpEvent(0x50)];
  EXPECT_EQ([embedderCallbacks count], 1u);
  embedderCallbacks[0](TRUE);
  // [owner.nextResponder keyUp:] should not be called, otherwise an error will be thrown.

  return true;
}

- (bool)doublePrimaryResponder {
  KeyboardTester* tester = [[KeyboardTester alloc] init];

  // Send a down event first so we can send an up event later.
  [tester respondEmbedderCallsWith:false];
  [tester respondChannelCallsWith:false];
  [tester.manager handleEvent:keyDownEvent(0x50)];

  NSMutableArray<FlutterAsyncKeyCallback>* embedderCallbacks =
      [NSMutableArray<FlutterAsyncKeyCallback> array];
  NSMutableArray<FlutterAsyncKeyCallback>* channelCallbacks =
      [NSMutableArray<FlutterAsyncKeyCallback> array];
  [tester recordEmbedderCallsTo:embedderCallbacks];
  [tester recordChannelCallsTo:channelCallbacks];

  // Case: Both responders report TRUE.
  [tester.manager handleEvent:keyUpEvent(0x50)];
  EXPECT_EQ([embedderCallbacks count], 1u);
  EXPECT_EQ([channelCallbacks count], 1u);
  embedderCallbacks[0](TRUE);
  channelCallbacks[0](TRUE);
  EXPECT_EQ([embedderCallbacks count], 1u);
  EXPECT_EQ([channelCallbacks count], 1u);
  // [tester.nextResponder keyUp:] should not be called, otherwise an error will be thrown.
  [embedderCallbacks removeAllObjects];
  [channelCallbacks removeAllObjects];

  // Case: One responder reports TRUE.
  [tester respondEmbedderCallsWith:false];
  [tester respondChannelCallsWith:false];
  [tester.manager handleEvent:keyDownEvent(0x50)];

  [tester recordEmbedderCallsTo:embedderCallbacks];
  [tester recordChannelCallsTo:channelCallbacks];
  [tester.manager handleEvent:keyUpEvent(0x50)];
  EXPECT_EQ([embedderCallbacks count], 1u);
  EXPECT_EQ([channelCallbacks count], 1u);
  embedderCallbacks[0](FALSE);
  channelCallbacks[0](TRUE);
  EXPECT_EQ([embedderCallbacks count], 1u);
  EXPECT_EQ([channelCallbacks count], 1u);
  // [tester.nextResponder keyUp:] should not be called, otherwise an error will be thrown.
  [embedderCallbacks removeAllObjects];
  [channelCallbacks removeAllObjects];

  // Case: Both responders report FALSE.
  [tester.manager handleEvent:keyDownEvent(0x53)];
  EXPECT_EQ([embedderCallbacks count], 1u);
  EXPECT_EQ([channelCallbacks count], 1u);
  embedderCallbacks[0](FALSE);
  channelCallbacks[0](FALSE);
  EXPECT_EQ([embedderCallbacks count], 1u);
  EXPECT_EQ([channelCallbacks count], 1u);
  OCMVerify([tester.nextResponder keyDown:checkKeyDownEvent(0x53)]);
  [embedderCallbacks removeAllObjects];
  [channelCallbacks removeAllObjects];

  return true;
}

- (bool)textInputPlugin {
  KeyboardTester* tester = [[KeyboardTester alloc] init];

  // Send a down event first so we can send an up event later.
  [tester respondEmbedderCallsWith:false];
  [tester respondChannelCallsWith:false];
  [tester.manager handleEvent:keyDownEvent(0x50)];

  NSMutableArray<FlutterAsyncKeyCallback>* callbacks =
      [NSMutableArray<FlutterAsyncKeyCallback> array];
  [tester recordEmbedderCallsTo:callbacks];

  // Case: Primary responder responds TRUE. The event shouldn't be handled by
  // the secondary responder.
  [tester respondTextInputWith:FALSE];
  [tester.manager handleEvent:keyUpEvent(0x50)];
  EXPECT_EQ([callbacks count], 1u);
  callbacks[0](TRUE);
  // [owner.nextResponder keyUp:] should not be called, otherwise an error will be thrown.
  [callbacks removeAllObjects];

  // Send a down event first so we can send an up event later.
  [tester respondEmbedderCallsWith:false];
  [tester.manager handleEvent:keyDownEvent(0x50)];

  // Case: Primary responder responds FALSE. The secondary responder returns
  // TRUE.
  [tester recordEmbedderCallsTo:callbacks];
  [tester respondTextInputWith:TRUE];
  [tester.manager handleEvent:keyUpEvent(0x50)];
  EXPECT_EQ([callbacks count], 1u);
  callbacks[0](FALSE);
  // [owner.nextResponder keyUp:] should not be called, otherwise an error will be thrown.
  [callbacks removeAllObjects];

  // Case: Primary responder responds FALSE. The secondary responder returns FALSE.
  [tester respondTextInputWith:FALSE];
  [tester.manager handleEvent:keyDownEvent(0x50)];
  EXPECT_EQ([callbacks count], 1u);
  callbacks[0](FALSE);
  OCMVerify([tester.nextResponder keyDown:checkKeyDownEvent(0x50)]);
  [callbacks removeAllObjects];

  return true;
}

- (bool)emptyNextResponder {
  KeyboardTester* tester = [[KeyboardTester alloc] init];
  tester.nextResponder = nil;

  [tester respondEmbedderCallsWith:false];
  [tester respondChannelCallsWith:false];
  [tester respondTextInputWith:false];
  [tester.manager handleEvent:keyDownEvent(0x50)];

  // Passes if no error is thrown.
  return true;
}

- (bool)getPressedState {
  KeyboardTester* tester = [[KeyboardTester alloc] init];

  [tester respondEmbedderCallsWith:false];
  [tester respondChannelCallsWith:false];
  [tester respondTextInputWith:false];
  [tester.manager handleEvent:keyDownEvent(kVK_ANSI_A)];

  NSDictionary* pressingRecords = [tester.manager getPressedState];
  EXPECT_EQ([pressingRecords count], 1u);
  EXPECT_EQ(pressingRecords[@(kPhysicalKeyA)], @(kLogicalKeyA));

  return true;
}

- (bool)keyboardChannelGetPressedState {
  KeyboardTester* tester = [[KeyboardTester alloc] init];

  [tester respondEmbedderCallsWith:false];
  [tester respondChannelCallsWith:false];
  [tester respondTextInputWith:false];
  [tester.manager handleEvent:keyDownEvent(kVK_ANSI_A)];

  FlutterMethodCall* getKeyboardStateMethodCall =
      [FlutterMethodCall methodCallWithMethodName:@"getKeyboardState" arguments:nil];
  NSData* getKeyboardStateMessage =
      [[FlutterStandardMethodCodec sharedInstance] encodeMethodCall:getKeyboardStateMethodCall];
  [tester sendKeyboardChannelMessage:getKeyboardStateMessage];

  id encodedResult = [tester lastKeyboardChannelResult];
  id decoded = [[FlutterStandardMethodCodec sharedInstance] decodeEnvelope:encodedResult];

  EXPECT_EQ([decoded count], 1u);
  EXPECT_EQ(decoded[@(kPhysicalKeyA)], @(kLogicalKeyA));

  return true;
}

// Regression test for https://github.com/flutter/flutter/issues/82673.
- (bool)racingConditionBetweenKeyAndText {
  KeyboardTester* tester = [[KeyboardTester alloc] init];

  // Use Vietnamese IME (GoTiengViet, Telex mode) to type "uco".

  // The events received by the framework. The engine might receive
  // a channel message "setEditingState" from the framework.
  NSMutableArray<FlutterAsyncKeyCallback>* keyCallbacks =
      [NSMutableArray<FlutterAsyncKeyCallback> array];
  [tester recordEmbedderCallsTo:keyCallbacks];

  NSMutableArray<NSNumber*>* allCalls = [NSMutableArray<NSNumber*> array];
  [tester recordCallTypesTo:allCalls forTypes:(kEmbedderCall | kTextCall)];

  // Tap key U, which is converted by IME into a pure text message "ư".

  [tester.manager handleEvent:keyDownEvent(kKeyCodeEmpty, @"ư", @"ư")];
  EXPECT_EQ([keyCallbacks count], 1u);
  EXPECT_EQ([allCalls count], 1u);
  EXPECT_EQ(allCalls[0], @(kEmbedderCall));
  keyCallbacks[0](false);
  EXPECT_EQ([keyCallbacks count], 1u);
  EXPECT_EQ([allCalls count], 2u);
  EXPECT_EQ(allCalls[1], @(kTextCall));
  [keyCallbacks removeAllObjects];
  [allCalls removeAllObjects];

  [tester.manager handleEvent:keyUpEvent(kKeyCodeEmpty)];
  EXPECT_EQ([keyCallbacks count], 1u);
  keyCallbacks[0](false);
  EXPECT_EQ([keyCallbacks count], 1u);
  EXPECT_EQ([allCalls count], 2u);
  [keyCallbacks removeAllObjects];
  [allCalls removeAllObjects];

  // Tap key O, which is converted to normal KeyO events, but the responses are
  // slow.

  [tester.manager handleEvent:keyDownEvent(kVK_ANSI_O, @"o", @"o")];
  [tester.manager handleEvent:keyUpEvent(kVK_ANSI_O)];
  EXPECT_EQ([keyCallbacks count], 1u);
  EXPECT_EQ([allCalls count], 1u);
  EXPECT_EQ(allCalls[0], @(kEmbedderCall));

  // Tap key C, which results in two Backspace messages first - and here they
  // arrive before the key O messages are responded.

  [tester.manager handleEvent:keyDownEvent(kVK_Delete)];
  [tester.manager handleEvent:keyUpEvent(kVK_Delete)];
  EXPECT_EQ([keyCallbacks count], 1u);
  EXPECT_EQ([allCalls count], 1u);

  // The key O down is responded, which releases a text call (for KeyO down) and
  // an embedder call (for KeyO up) immediately.
  keyCallbacks[0](false);
  EXPECT_EQ([keyCallbacks count], 2u);
  EXPECT_EQ([allCalls count], 3u);
  EXPECT_EQ(allCalls[1], @(kTextCall));  // The order is important!
  EXPECT_EQ(allCalls[2], @(kEmbedderCall));

  // The key O up is responded, which releases a text call (for KeyO up) and
  // an embedder call (for Backspace down) immediately.
  keyCallbacks[1](false);
  EXPECT_EQ([keyCallbacks count], 3u);
  EXPECT_EQ([allCalls count], 5u);
  EXPECT_EQ(allCalls[3], @(kTextCall));  // The order is important!
  EXPECT_EQ(allCalls[4], @(kEmbedderCall));

  // Finish up callbacks.
  keyCallbacks[2](false);
  keyCallbacks[3](false);

  return true;
}

- (bool)correctLogicalKeyForLayouts {
  KeyboardTester* tester = [[KeyboardTester alloc] init];
  tester.nextResponder = nil;

  std::vector<FlutterKeyEvent> events;
  [tester recordEmbedderEventsTo:&events returning:true];
  [tester respondChannelCallsWith:false];
  [tester respondTextInputWith:false];

  auto sendTap = [&](uint16_t keyCode, NSString* chars, NSString* charsUnmod) {
    [tester.manager handleEvent:keyDownEvent(keyCode, chars, charsUnmod)];
    [tester.manager handleEvent:keyUpEvent(keyCode)];
  };

  /* US keyboard layout */

  sendTap(kVK_ANSI_A, @"a", @"a");  // KeyA
  VERIFY_DOWN(kLogicalKeyA, "a");

  sendTap(kVK_ANSI_A, @"A", @"A");  // Shift-KeyA
  VERIFY_DOWN(kLogicalKeyA, "A");

  sendTap(kVK_ANSI_A, @"å", @"a");  // Option-KeyA
  VERIFY_DOWN(kLogicalKeyA, "å");

  sendTap(kVK_ANSI_T, @"t", @"t");  // KeyT
  VERIFY_DOWN(kLogicalKeyT, "t");

  sendTap(kVK_ANSI_1, @"1", @"1");  // Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  sendTap(kVK_ANSI_1, @"!", @"!");  // Shift-Digit1
  VERIFY_DOWN(kLogicalDigit1, "!");

  sendTap(kVK_ANSI_Minus, @"-", @"-");  // Minus
  VERIFY_DOWN('-', "-");

  sendTap(kVK_ANSI_Minus, @"=", @"=");  // Shift-Minus
  VERIFY_DOWN('=', "=");

  /* French keyboard layout */
  [tester setLayout:kFrenchLayout];

  sendTap(kVK_ANSI_A, @"q", @"q");  // KeyA
  VERIFY_DOWN(kLogicalKeyQ, "q");

  sendTap(kVK_ANSI_A, @"Q", @"Q");  // Shift-KeyA
  VERIFY_DOWN(kLogicalKeyQ, "Q");

  sendTap(kVK_ANSI_Semicolon, @"m", @"m");  // ; but prints M
  VERIFY_DOWN(kLogicalKeyM, "m");

  sendTap(kVK_ANSI_M, @",", @",");  // M but prints ,
  VERIFY_DOWN(',', ",");

  sendTap(kVK_ANSI_1, @"&", @"&");  // Digit1
  VERIFY_DOWN(kLogicalDigit1, "&");

  sendTap(kVK_ANSI_1, @"1", @"1");  // Shift-Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  sendTap(kVK_ANSI_Minus, @")", @")");  // Minus
  VERIFY_DOWN(')', ")");

  sendTap(kVK_ANSI_Minus, @"°", @"°");  // Shift-Minus
  VERIFY_DOWN(L'°', "°");

  /* Russian keyboard layout */
  [tester setLayout:kRussianLayout];

  sendTap(kVK_ANSI_A, @"ф", @"ф");  // KeyA
  VERIFY_DOWN(kLogicalKeyA, "ф");

  sendTap(kVK_ANSI_1, @"1", @"1");  // Digit1
  VERIFY_DOWN(kLogicalDigit1, "1");

  sendTap(kVK_ANSI_LeftBracket, @"х", @"х");
  VERIFY_DOWN(kLogicalBracketLeft, "х");

  /* Khmer keyboard layout */
  // Regression test for https://github.com/flutter/flutter/issues/108729
  [tester setLayout:kKhmerLayout];

  sendTap(kVK_ANSI_2, @"២", @"២");  // Digit2
  VERIFY_DOWN(kLogicalDigit2, "២");

  return TRUE;
}

- (bool)shouldNotHoldStrongReferenceToViewDelegate {
  __strong FlutterKeyboardManager* strongKeyboardManager;
  __weak id weakViewDelegate;

  @autoreleasepool {
    id binaryMessengerMock = OCMStrictProtocolMock(@protocol(FlutterBinaryMessenger));
    OCMStub([binaryMessengerMock setMessageHandlerOnChannel:[OCMArg any]
                                       binaryMessageHandler:[OCMArg any]]);

    id viewDelegateMock = OCMStrictProtocolMock(@protocol(FlutterKeyboardViewDelegate));
    OCMStub([viewDelegateMock getBinaryMessenger]).andReturn(binaryMessengerMock);
    OCMStub([viewDelegateMock subscribeToKeyboardLayoutChange:[OCMArg any]]);

    LayoutClue layoutClue;
    OCMStub([viewDelegateMock lookUpLayoutForKeyCode:0 shift:NO])
        .ignoringNonObjectArgs()
        .andReturn(layoutClue);
    FlutterKeyboardManager* keyboardManager =
        [[FlutterKeyboardManager alloc] initWithViewDelegate:viewDelegateMock];
    strongKeyboardManager = keyboardManager;
    weakViewDelegate = viewDelegateMock;
  }

  return weakViewDelegate == nil;
}

@end

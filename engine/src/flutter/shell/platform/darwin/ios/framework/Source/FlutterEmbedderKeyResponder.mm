// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEmbedderKeyResponder.h"
#include <objc/NSObjCRuntime.h>

#import <objc/message.h>
#include <map>
#include "fml/memory/weak_ptr.h"

#import "KeyCodeMap_Internal.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterCodecs.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

FLUTTER_ASSERT_ARC

namespace {

/**
 * Isolate the least significant 1-bit.
 *
 * For example,
 *
 *  * lowestSetBit(0x1010) returns 0x10.
 *  * lowestSetBit(0) returns 0.
 */
static NSUInteger lowestSetBit(NSUInteger bitmask) {
  // This utilizes property of two's complement (negation), which propagates a
  // carry bit from LSB to the lowest set bit.
  return bitmask & -bitmask;
}

/**
 * Whether a string represents a control character.
 */
static bool IsControlCharacter(NSUInteger length, NSString* label) {
  if (length > 1) {
    return false;
  }
  unichar codeUnit = [label characterAtIndex:0];
  return (codeUnit <= 0x1f && codeUnit >= 0x00) || (codeUnit >= 0x7f && codeUnit <= 0x9f);
}

/**
 * Whether a string represents an unprintable key.
 */
static bool IsUnprintableKey(NSUInteger length, NSString* label) {
  if (length > 1) {
    return false;
  }
  unichar codeUnit = [label characterAtIndex:0];
  return codeUnit >= 0xF700 && codeUnit <= 0xF8FF;
}

/**
 * Returns a key code composed with a base key and a plane.
 *
 * Examples of unprintable keys are "NSUpArrowFunctionKey = 0xF700" or
 * "NSHomeFunctionKey = 0xF729".
 *
 * See
 * https://developer.apple.com/documentation/appkit/1535851-function-key_unicodes?language=objc
 * for more information.
 */
static uint64_t KeyOfPlane(uint64_t baseKey, uint64_t plane) {
  return plane | (baseKey & kValueMask);
}

/**
 * Returns the physical key for a key code.
 */
static uint64_t GetPhysicalKeyForKeyCode(UInt32 keyCode) {
  auto physicalKey = keyCodeToPhysicalKey.find(keyCode);
  if (physicalKey == keyCodeToPhysicalKey.end()) {
    return KeyOfPlane(keyCode, kIosPlane);
  }
  return physicalKey->second;
}

/**
 * Returns the logical key for a modifier physical key.
 */
static uint64_t GetLogicalKeyForModifier(UInt32 keyCode, uint64_t hidCode) {
  auto fromKeyCode = keyCodeToLogicalKey.find(keyCode);
  if (fromKeyCode != keyCodeToLogicalKey.end()) {
    return fromKeyCode->second;
  }
  return KeyOfPlane(hidCode, kIosPlane);
}

/**
 * Converts upper letters to lower letters in ASCII and extended ASCII, and
 * returns as-is otherwise.
 *
 * Independent of locale.
 */
static uint64_t toLower(uint64_t n) {
  constexpr uint64_t lower_a = 0x61;
  constexpr uint64_t upper_a = 0x41;
  constexpr uint64_t upper_z = 0x5a;

  constexpr uint64_t lower_a_grave = 0xe0;
  constexpr uint64_t upper_a_grave = 0xc0;
  constexpr uint64_t upper_thorn = 0xde;
  constexpr uint64_t division = 0xf7;

  // ASCII range.
  if (n >= upper_a && n <= upper_z) {
    return n - upper_a + lower_a;
  }

  // EASCII range.
  if (n >= upper_a_grave && n <= upper_thorn && n != division) {
    return n - upper_a_grave + lower_a_grave;
  }

  return n;
}

/**
 * Filters out some special cases in the characters field on UIKey.
 */
static const char* getEventCharacters(NSString* characters, UIKeyboardHIDUsage keyCode)
    API_AVAILABLE(ios(13.4)) {
  if (characters == nil) {
    return nullptr;
  }
  if ([characters length] == 0) {
    return nullptr;
  }
  if (@available(iOS 13.4, *)) {
    // On iOS, function keys return the UTF8 string "\^P" (with a literal '/',
    // '^' and a 'P', not escaped ctrl-P) as their "characters" field. This
    // isn't a valid (single) UTF8 character. Looking at the only UTF16
    // character for a function key yields a value of "16", which is a Unicode
    // "SHIFT IN" character, which is just odd. UTF8 conversion of that string
    // is what generates the three characters "\^P".
    //
    // Anyhow, we strip them all out and replace them with empty strings, since
    // function keys shouldn't be printable.
    if (functionKeyCodes.find(keyCode) != functionKeyCodes.end()) {
      return nullptr;
    }
  }
  return [characters UTF8String];
}

/**
 * Returns the logical key of a KeyUp or KeyDown event.
 *
 * The `maybeSpecialKey` is a nullable integer, and if not nil, indicates
 * that the event key is a special key as defined by `specialKeyMapping`,
 * and is the corresponding logical key.
 *
 * For modifier keys, use GetLogicalKeyForModifier.
 */
static uint64_t GetLogicalKeyForEvent(FlutterUIPressProxy* press, NSNumber* maybeSpecialKey)
    API_AVAILABLE(ios(13.4)) {
  if (maybeSpecialKey != nil) {
    return [maybeSpecialKey unsignedLongLongValue];
  }
  // Look to see if the keyCode can be mapped from keycode.
  auto fromKeyCode = keyCodeToLogicalKey.find(press.key.keyCode);
  if (fromKeyCode != keyCodeToLogicalKey.end()) {
    return fromKeyCode->second;
  }
  const char* characters =
      getEventCharacters(press.key.charactersIgnoringModifiers, press.key.keyCode);
  NSString* keyLabel =
      characters == nullptr ? nil : [[NSString alloc] initWithUTF8String:characters];
  NSUInteger keyLabelLength = [keyLabel length];
  // If this key is printable, generate the logical key from its Unicode
  // value. Control keys such as ESC, CTRL, and SHIFT are not printable. HOME,
  // DEL, arrow keys, and function keys are considered modifier function keys,
  // which generate invalid Unicode scalar values.
  if (keyLabelLength != 0 && !IsControlCharacter(keyLabelLength, keyLabel) &&
      !IsUnprintableKey(keyLabelLength, keyLabel)) {
    // Given that charactersIgnoringModifiers can contain a string of arbitrary
    // length, limit to a maximum of two Unicode scalar values. It is unlikely
    // that a keyboard would produce a code point bigger than 32 bits, but it is
    // still worth defending against this case.
    NSCAssert((keyLabelLength < 2), @"Unexpected long key label: |%@|.", keyLabel);

    uint64_t codeUnit = (uint64_t)[keyLabel characterAtIndex:0];
    if (keyLabelLength == 2) {
      uint64_t secondCode = (uint64_t)[keyLabel characterAtIndex:1];
      codeUnit = (codeUnit << 16) | secondCode;
    }
    return KeyOfPlane(toLower(codeUnit), kUnicodePlane);
  }

  // This is a non-printable key that is unrecognized, so a new code is minted
  // with the autogenerated bit set.
  return KeyOfPlane(press.key.keyCode, kIosPlane);
}

/**
 * Converts NSEvent.timestamp to the timestamp for Flutter.
 */
static double GetFlutterTimestampFrom(NSTimeInterval timestamp) {
  // Timestamp in microseconds. The event.timestamp is in seconds with sub-ms precision.
  return timestamp * 1000000.0;
}

/**
 * Compute |modifierFlagOfInterestMask| out of |keyCodeToModifierFlag|.
 *
 * This is equal to the bitwise-or of all values of |keyCodeToModifierFlag|.
 */
static NSUInteger computeModifierFlagOfInterestMask() {
  NSUInteger modifierFlagOfInterestMask = kModifierFlagCapsLock | kModifierFlagShiftAny |
                                          kModifierFlagControlAny | kModifierFlagAltAny |
                                          kModifierFlagMetaAny;
  for (std::pair<UInt32, ModifierFlag> entry : keyCodeToModifierFlag) {
    modifierFlagOfInterestMask = modifierFlagOfInterestMask | entry.second;
  }
  return modifierFlagOfInterestMask;
}

static bool isKeyDown(FlutterUIPressProxy* press) API_AVAILABLE(ios(13.4)) {
  switch (press.phase) {
    case UIPressPhaseStationary:
    case UIPressPhaseChanged:
      // Not sure if this is the right thing to do for these two, but true seems
      // more correct than false.
      return true;
    case UIPressPhaseBegan:
      return true;
    case UIPressPhaseCancelled:
    case UIPressPhaseEnded:
      return false;
  }
  return false;
}

/**
 * The C-function sent to the engine's |sendKeyEvent|, wrapping
 * |FlutterEmbedderKeyResponder.handleResponse|.
 *
 * For the reason of this wrap, see |FlutterKeyPendingResponse|.
 */
void HandleResponse(bool handled, void* user_data);
}  // namespace

/**
 * The invocation context for |HandleResponse|, wrapping
 * |FlutterEmbedderKeyResponder.handleResponse|.
 *
 * The key responder's functions only accept C-functions as callbacks, as well
 * as arbitrary user_data. In order to send an instance method of
 * |FlutterEmbedderKeyResponder.handleResponse| to the engine's |SendKeyEvent|,
 * we wrap the invocation into a C-function |HandleResponse| and invocation
 * context |FlutterKeyPendingResponse|.
 */
@interface FlutterKeyPendingResponse : NSObject

@property(readonly, weak) FlutterEmbedderKeyResponder* responder;

@property(nonatomic) uint64_t responseId;

- (nonnull instancetype)initWithHandler:(nonnull FlutterEmbedderKeyResponder*)responder
                             responseId:(uint64_t)responseId;

@end

@implementation FlutterKeyPendingResponse
- (instancetype)initWithHandler:(FlutterEmbedderKeyResponder*)responder
                     responseId:(uint64_t)responseId {
  self = [super init];
  if (self != nil) {
    _responder = responder;
    _responseId = responseId;
  }
  return self;
}
@end

/**
 * Guards a |FlutterAsyncKeyCallback| to make sure it's handled exactly once
 * throughout the process of handling an event in |FlutterEmbedderKeyResponder|.
 *
 * A callback can either be handled with |pendTo:withId:|, or with |resolveTo:|.
 * Either way, the callback cannot be handled again, or an assertion will be
 * thrown.
 */
@interface FlutterKeyCallbackGuard : NSObject
- (nonnull instancetype)initWithCallback:(FlutterAsyncKeyCallback)callback;

/**
 * Handle the callback by storing it to pending responses.
 */
- (void)pendTo:(nonnull NSMutableDictionary<NSNumber*, FlutterAsyncKeyCallback>*)pendingResponses
        withId:(uint64_t)responseId;

/**
 * Handle the callback by calling it with a result.
 */
- (void)resolveTo:(BOOL)handled;

@property(nonatomic) BOOL handled;
/**
 * A string indicating how the callback is handled.
 *
 * Only set in debug mode. Nil in release mode, or if the callback has not been
 * handled.
 */
@property(readonly, copy) NSString* debugHandleSource;
@end

@implementation FlutterKeyCallbackGuard {
  // The callback is declared in the implementation block to avoid being
  // accessed directly.
  FlutterAsyncKeyCallback _callback;
}
- (nonnull instancetype)initWithCallback:(FlutterAsyncKeyCallback)callback {
  self = [super init];
  if (self != nil) {
    _callback = [callback copy];
    _handled = FALSE;
  }
  return self;
}

- (void)pendTo:(nonnull NSMutableDictionary<NSNumber*, FlutterAsyncKeyCallback>*)pendingResponses
        withId:(uint64_t)responseId {
  NSAssert(!_handled, @"This callback has been handled by %@.", _debugHandleSource);
  if (_handled) {
    return;
  }
  pendingResponses[@(responseId)] = _callback;
  _handled = TRUE;
  NSAssert(
      ((_debugHandleSource = [NSString stringWithFormat:@"pending event %llu", responseId]), TRUE),
      @"");
}

- (void)resolveTo:(BOOL)handled {
  NSAssert(!_handled, @"This callback has been handled by %@.", _debugHandleSource);
  if (_handled) {
    return;
  }
  _callback(handled);
  _handled = TRUE;
  NSAssert(((_debugHandleSource = [NSString stringWithFormat:@"resolved with %d", _handled]), TRUE),
           @"");
}
@end

@interface FlutterEmbedderKeyResponder ()

/**
 * The function to send converted events to.
 *
 * Set by the initializer.
 */
@property(nonatomic, copy, readonly) FlutterSendKeyEvent sendEvent;

/**
 * A map of pressed keys.
 *
 * The keys of the dictionary are physical keys, while the values are the logical keys
 * of the key down event.
 */
@property(nonatomic, copy, readonly) NSMutableDictionary<NSNumber*, NSNumber*>* pressingRecords;

/**
 * A constant mask for NSEvent.modifierFlags that Flutter synchronizes with.
 *
 * Flutter keeps track of the last |modifierFlags| and compares it with the
 * incoming one. Any bit within |modifierFlagOfInterestMask| that is different
 * (except for the one that corresponds to the event key) indicates that an
 * event for this modifier was missed, and Flutter synthesizes an event to make
 * up for the state difference.
 *
 * It is computed by computeModifierFlagOfInterestMask.
 */
@property(nonatomic) NSUInteger modifierFlagOfInterestMask;

/**
 * The modifier flags of the last received key event, excluding uninterested
 * bits.
 *
 * This should be kept synchronized with the last |NSEvent.modifierFlags|
 * after masking with |modifierFlagOfInterestMask|. This should also be kept
 * synchronized with the corresponding keys of |pressingRecords|.
 *
 * This is used by |synchronizeModifiers| to quickly find out modifier keys that
 * are desynchronized.
 */
@property(nonatomic) NSUInteger lastModifierFlagsOfInterest;

/**
 * A self-incrementing ID used to label key events sent to the framework.
 */
@property(nonatomic) uint64_t responseId;

/**
 * A map of unresponded key events sent to the framework.
 *
 * Its values are |responseId|s, and keys are the callback that was received
 * along with the event.
 */
@property(nonatomic, copy, readonly)
    NSMutableDictionary<NSNumber*, FlutterAsyncKeyCallback>* pendingResponses;

/**
 * Compare the last modifier flags and the current, and dispatch synthesized
 * key events for each different modifier flag bit.
 *
 * The flags compared are all flags after masking with
 * |modifierFlagOfInterestMask| and excluding |ignoringFlags|.
 */
- (void)synchronizeModifiers:(nonnull FlutterUIPressProxy*)press API_AVAILABLE(ios(13.4));

/**
 * Update the pressing state.
 *
 * If `logicalKey` is not 0, `physicalKey` is pressed as `logicalKey`.
 * Otherwise, `physicalKey` is released.
 */
- (void)updateKey:(uint64_t)physicalKey asPressed:(uint64_t)logicalKey;

/**
 * Synthesize a CapsLock down event, then a CapsLock up event.
 */
- (void)synthesizeCapsLockTapWithTimestamp:(NSTimeInterval)timestamp;

/**
 * Send an event to the framework, expecting its response.
 */
- (void)sendPrimaryFlutterEvent:(const FlutterKeyEvent&)event
                       callback:(nonnull FlutterKeyCallbackGuard*)callback;

/**
 * Send an empty key event.
 *
 * The event is never synthesized, and never expects an event result. An empty
 * event is sent when no other events should be sent, such as upon back-to-back
 * keydown events of the same key.
 */
- (void)sendEmptyEvent;

/**
 * Send a key event for a modifier key.
 */
- (void)synthesizeModifierEventOfType:(BOOL)isDownEvent
                            timestamp:(NSTimeInterval)timestamp
                              keyCode:(UInt32)keyCode;

/**
 * Processes a down event from the system.
 */
- (void)handlePressBegin:(nonnull FlutterUIPressProxy*)press
                callback:(nonnull FlutterKeyCallbackGuard*)callback API_AVAILABLE(ios(13.4));

/**
 * Processes an up event from the system.
 */
- (void)handlePressEnd:(nonnull FlutterUIPressProxy*)press
              callback:(nonnull FlutterKeyCallbackGuard*)callback API_AVAILABLE(ios(13.4));

/**
 * Processes the response from the framework.
 */
- (void)handleResponse:(BOOL)handled forId:(uint64_t)responseId;

/**
 * Fix up the modifiers for a particular type of modifier key.
 */
- (UInt32)fixSidedFlags:(ModifierFlag)anyFlag
           withLeftFlag:(ModifierFlag)leftSide
          withRightFlag:(ModifierFlag)rightSide
            withLeftKey:(UInt16)leftKeyCode
           withRightKey:(UInt16)rightKeyCode
            withKeyCode:(UInt16)keyCode
                keyDown:(BOOL)isKeyDown
               forFlags:(UInt32)modifiersPressed API_AVAILABLE(ios(13.4));

/**
 * Because iOS differs from other platforms in that the modifier flags still
 * contain the flag for the key that is being released on the keyup event, we
 * adjust the modifiers when the released key is a matching modifier key.
 */
- (UInt32)adjustModifiers:(nonnull FlutterUIPressProxy*)press API_AVAILABLE(ios(13.4));
@end

@implementation FlutterEmbedderKeyResponder

- (nonnull instancetype)initWithSendEvent:(FlutterSendKeyEvent)sendEvent {
  self = [super init];
  if (self != nil) {
    _sendEvent = [sendEvent copy];
    _pressingRecords = [[NSMutableDictionary alloc] init];
    _pendingResponses = [[NSMutableDictionary alloc] init];
    _responseId = 1;
    _lastModifierFlagsOfInterest = 0;
    _modifierFlagOfInterestMask = computeModifierFlagOfInterestMask();
  }
  return self;
}

- (void)handlePress:(nonnull FlutterUIPressProxy*)press
           callback:(FlutterAsyncKeyCallback)callback API_AVAILABLE(ios(13.4)) {
  if (@available(iOS 13.4, *)) {
  } else {
    return;
  }
  // The conversion algorithm relies on a non-nil callback to properly compute
  // `synthesized`.
  NSAssert(callback != nil, @"The callback must not be nil.");

  FlutterKeyCallbackGuard* guardedCallback = nil;
  switch (press.phase) {
    case UIPressPhaseBegan:
      guardedCallback = [[FlutterKeyCallbackGuard alloc] initWithCallback:callback];
      [self handlePressBegin:press callback:guardedCallback];
      break;
    case UIPressPhaseEnded:
      guardedCallback = [[FlutterKeyCallbackGuard alloc] initWithCallback:callback];
      [self handlePressEnd:press callback:guardedCallback];
      break;
    case UIPressPhaseChanged:
    case UIPressPhaseCancelled:
      // TODO(gspencergoog): Handle cancelled events as synthesized up events.
    case UIPressPhaseStationary:
      NSAssert(false, @"Unexpected press phase receieved in handlePress");
      return;
  }
  NSAssert(guardedCallback.handled, @"The callback returned without being handled.");
  NSAssert(
      (_lastModifierFlagsOfInterest & ~kModifierFlagCapsLock) ==
          ([self adjustModifiers:press] & (_modifierFlagOfInterestMask & ~kModifierFlagCapsLock)),
      @"The modifier flags are not properly updated: recorded 0x%lx, event with mask 0x%lx",
      static_cast<unsigned long>(_lastModifierFlagsOfInterest & ~kModifierFlagCapsLock),
      static_cast<unsigned long>([self adjustModifiers:press] &
                                 (_modifierFlagOfInterestMask & ~kModifierFlagCapsLock)));
}

#pragma mark - Private

- (void)synchronizeModifiers:(nonnull FlutterUIPressProxy*)press API_AVAILABLE(ios(13.4)) {
  if (@available(iOS 13.4, *)) {
  } else {
    return;
  }

  const UInt32 lastFlagsOfInterest = _lastModifierFlagsOfInterest & _modifierFlagOfInterestMask;
  const UInt32 pressedModifiers = [self adjustModifiers:press];
  const UInt32 currentFlagsOfInterest = pressedModifiers & _modifierFlagOfInterestMask;
  UInt32 flagDifference = currentFlagsOfInterest ^ lastFlagsOfInterest;
  if (flagDifference & kModifierFlagCapsLock) {
    // If the caps lock changed, and we didn't expect that, then send a
    // synthesized down and an up to simulate a toggle of the state.
    if (press.key.keyCode != UIKeyboardHIDUsageKeyboardCapsLock) {
      [self synthesizeCapsLockTapWithTimestamp:press.timestamp];
    }
    flagDifference &= ~kModifierFlagCapsLock;
  }
  while (true) {
    const UInt32 currentFlag = lowestSetBit(flagDifference);
    if (currentFlag == 0) {
      break;
    }
    flagDifference &= ~currentFlag;
    if (currentFlag & kModifierFlagAnyMask) {
      // Skip synthesizing keys for the "any" flags, since their synthesis will
      // be handled when we do the sided flags. We still want them in the flags
      // of interest, though, so we can keep their state.
      continue;
    }
    auto keyCode = modifierFlagToKeyCode.find(static_cast<ModifierFlag>(currentFlag));
    NSAssert(keyCode != modifierFlagToKeyCode.end(), @"Invalid modifier flag of interest 0x%lx",
             static_cast<unsigned long>(currentFlag));
    if (keyCode == modifierFlagToKeyCode.end()) {
      continue;
    }
    // If this press matches the modifier key in question, then don't synthesize
    // it, because it's already a "real" keypress.
    if (keyCode->second == static_cast<UInt32>(press.key.keyCode)) {
      continue;
    }
    BOOL isDownEvent = currentFlagsOfInterest & currentFlag;
    [self synthesizeModifierEventOfType:isDownEvent
                              timestamp:press.timestamp
                                keyCode:keyCode->second];
  }
  _lastModifierFlagsOfInterest =
      (_lastModifierFlagsOfInterest & ~_modifierFlagOfInterestMask) | currentFlagsOfInterest;
}

- (void)synthesizeCapsLockTapWithTimestamp:(NSTimeInterval)timestamp {
  // The assumption when the app starts is that caps lock is off, but if that
  // turns out to be untrue (according to the modifier flags), then this is used
  // to simulate a key down and a key up of the caps lock key, to simulate
  // toggling of that state in the framework.
  FlutterKeyEvent flutterEvent = {
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = GetFlutterTimestampFrom(timestamp),
      .type = kFlutterKeyEventTypeDown,
      .physical = kCapsLockPhysicalKey,
      .logical = kCapsLockLogicalKey,
      .character = nil,
      .synthesized = true,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };
  _sendEvent(flutterEvent, nullptr, nullptr);

  flutterEvent.type = kFlutterKeyEventTypeUp;
  _sendEvent(flutterEvent, nullptr, nullptr);
}

- (void)updateKey:(uint64_t)physicalKey asPressed:(uint64_t)logicalKey {
  if (logicalKey == 0) {
    [_pressingRecords removeObjectForKey:@(physicalKey)];
  } else {
    _pressingRecords[@(physicalKey)] = @(logicalKey);
  }
}

- (void)sendPrimaryFlutterEvent:(const FlutterKeyEvent&)event
                       callback:(FlutterKeyCallbackGuard*)callback {
  _responseId += 1;
  uint64_t responseId = _responseId;
  FlutterKeyPendingResponse* pending =
      [[FlutterKeyPendingResponse alloc] initWithHandler:self responseId:responseId];
  [callback pendTo:_pendingResponses withId:responseId];
  _sendEvent(event, HandleResponse, (__bridge_retained void* _Nullable)pending);
}

- (void)sendEmptyEvent {
  FlutterKeyEvent event = {
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = 0,
      .type = kFlutterKeyEventTypeDown,
      .physical = 0,
      .logical = 0,
      .character = nil,
      .synthesized = false,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };
  _sendEvent(event, nil, nil);
}

- (void)synthesizeModifierEventOfType:(BOOL)isDownEvent
                            timestamp:(NSTimeInterval)timestamp
                              keyCode:(UInt32)keyCode {
  uint64_t physicalKey = GetPhysicalKeyForKeyCode(keyCode);
  uint64_t logicalKey = GetLogicalKeyForModifier(keyCode, physicalKey);
  if (physicalKey == 0 || logicalKey == 0) {
    return;
  }
  FlutterKeyEvent flutterEvent = {
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = GetFlutterTimestampFrom(timestamp),
      .type = isDownEvent ? kFlutterKeyEventTypeDown : kFlutterKeyEventTypeUp,
      .physical = physicalKey,
      .logical = logicalKey,
      .character = nil,
      .synthesized = true,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };
  [self updateKey:physicalKey asPressed:isDownEvent ? logicalKey : 0];
  _sendEvent(flutterEvent, nullptr, nullptr);
}

- (void)handlePressBegin:(nonnull FlutterUIPressProxy*)press
                callback:(nonnull FlutterKeyCallbackGuard*)callback API_AVAILABLE(ios(13.4)) {
  if (@available(iOS 13.4, *)) {
  } else {
    return;
  }
  uint64_t physicalKey = GetPhysicalKeyForKeyCode(press.key.keyCode);
  // Some unprintable keys on iOS have literal names on their key label, such as
  // @"UIKeyInputEscape". They are called the "special keys" and have predefined
  // logical keys and empty characters.
  NSNumber* specialKey = [specialKeyMapping objectForKey:press.key.charactersIgnoringModifiers];
  uint64_t logicalKey = GetLogicalKeyForEvent(press, specialKey);
  [self synchronizeModifiers:press];

  NSNumber* pressedLogicalKey = nil;
  if ([_pressingRecords count] > 0) {
    pressedLogicalKey = _pressingRecords[@(physicalKey)];
    if (pressedLogicalKey != nil) {
      // Normally the key up events won't be missed since iOS always sends the
      // key up event to the view where the corresponding key down occurred.
      // However this might happen in add-to-app scenarios if the focus is changed
      // from the native view to the Flutter view amid the key tap.
      [callback resolveTo:TRUE];
      [self sendEmptyEvent];
      return;
    }
  }

  if (pressedLogicalKey == nil) {
    [self updateKey:physicalKey asPressed:logicalKey];
  }

  FlutterKeyEvent flutterEvent = {
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = GetFlutterTimestampFrom(press.timestamp),
      .type = kFlutterKeyEventTypeDown,
      .physical = physicalKey,
      .logical = pressedLogicalKey == nil ? logicalKey : [pressedLogicalKey unsignedLongLongValue],
      .character =
          specialKey != nil ? nil : getEventCharacters(press.key.characters, press.key.keyCode),
      .synthesized = false,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };
  [self sendPrimaryFlutterEvent:flutterEvent callback:callback];
}

- (void)handlePressEnd:(nonnull FlutterUIPressProxy*)press
              callback:(nonnull FlutterKeyCallbackGuard*)callback API_AVAILABLE(ios(13.4)) {
  if (@available(iOS 13.4, *)) {
  } else {
    return;
  }
  [self synchronizeModifiers:press];

  uint64_t physicalKey = GetPhysicalKeyForKeyCode(press.key.keyCode);
  NSNumber* pressedLogicalKey = _pressingRecords[@(physicalKey)];
  if (pressedLogicalKey == nil) {
    // Normally the key up events won't be missed since iOS always sends the
    // key up event to the view where the corresponding key down occurred.
    // However this might happen in add-to-app scenarios if the focus is changed
    // from the native view to the Flutter view amid the key tap.
    [callback resolveTo:TRUE];
    [self sendEmptyEvent];
    return;
  }
  [self updateKey:physicalKey asPressed:0];

  FlutterKeyEvent flutterEvent = {
      .struct_size = sizeof(FlutterKeyEvent),
      .timestamp = GetFlutterTimestampFrom(press.timestamp),
      .type = kFlutterKeyEventTypeUp,
      .physical = physicalKey,
      .logical = [pressedLogicalKey unsignedLongLongValue],
      .character = nil,
      .synthesized = false,
      .device_type = kFlutterKeyEventDeviceTypeKeyboard,
  };
  [self sendPrimaryFlutterEvent:flutterEvent callback:callback];
}

- (void)handleResponse:(BOOL)handled forId:(uint64_t)responseId {
  FlutterAsyncKeyCallback callback = _pendingResponses[@(responseId)];
  callback(handled);
  [_pendingResponses removeObjectForKey:@(responseId)];
}

- (UInt32)fixSidedFlags:(ModifierFlag)anyFlag
           withLeftFlag:(ModifierFlag)leftSide
          withRightFlag:(ModifierFlag)rightSide
            withLeftKey:(UInt16)leftKeyCode
           withRightKey:(UInt16)rightKeyCode
            withKeyCode:(UInt16)keyCode
                keyDown:(BOOL)isKeyDown
               forFlags:(UInt32)modifiersPressed API_AVAILABLE(ios(13.4)) {
  UInt32 newModifiers = modifiersPressed;
  if (isKeyDown) {
    // Add in the modifier flags that correspond to this key code, if any.
    if (keyCode == leftKeyCode) {
      newModifiers |= leftSide | anyFlag;
    } else if (keyCode == rightKeyCode) {
      newModifiers |= rightSide | anyFlag;
    }
  } else {
    // If this is a key up, then remove any modifier that matches the keycode in
    // the event from the flags, and the anyFlag if the other side isn't also
    // pressed.
    if (keyCode == leftKeyCode) {
      newModifiers &= ~leftSide;
      if (!(newModifiers & rightSide)) {
        newModifiers &= ~anyFlag;
      }
    } else if (keyCode == rightKeyCode) {
      newModifiers &= ~rightSide;
      if (!(newModifiers & leftSide)) {
        newModifiers &= ~anyFlag;
      }
    }
  }

  if (!(newModifiers & anyFlag)) {
    // Turn off any sided flags, since the "any" flag is gone.
    newModifiers &= ~(leftSide | rightSide);
  }

  return newModifiers;
}

// This fixes a few cases where iOS provides modifier flags differently from how
// the framework would like to receive them.
//
// 1) iOS turns off the flag associated with a modifier key AFTER the modifier
//    key up event, so when the key up event arrives, the flags must be modified
//    before synchronizing so they do not include the modifier that arrived in
//    the key up event.
// 2) Modifier flags can be set even when that modifier is not being pressed.
//    One example of this is when a special character is produced with the Alt
//    (Option) key, and the Alt key is released before the letter key: the
//    letter key's key up event still contains the Alt key flag.
// 3) iOS doesn't provide information about which side modifier was pressed,
//    except through the keycode of the pressed key, so we look at the pressed
//    key code to decide which side to indicate in the flags. If we can't know
//    (in the case of a non-modifier key event having an "any" modifier set, but
//    we don't know already that the modifier is down), then we just pick the
//    left one arbitrarily.
- (UInt32)adjustModifiers:(nonnull FlutterUIPressProxy*)press API_AVAILABLE(ios(13.4)) {
  if (@available(iOS 13.4, *)) {
    // no-op
  } else {
    return press.key.modifierFlags;
  }

  bool keyDown = isKeyDown(press);

  // Start with the current modifier flags, along with any sided flags that we
  // already know are down.
  UInt32 pressedModifiers =
      press.key.modifierFlags | (_lastModifierFlagsOfInterest & kModifierFlagSidedMask);

  pressedModifiers = [self fixSidedFlags:kModifierFlagShiftAny
                            withLeftFlag:kModifierFlagShiftLeft
                           withRightFlag:kModifierFlagShiftRight
                             withLeftKey:UIKeyboardHIDUsageKeyboardLeftShift
                            withRightKey:UIKeyboardHIDUsageKeyboardRightShift
                             withKeyCode:press.key.keyCode
                                 keyDown:keyDown
                                forFlags:pressedModifiers];
  pressedModifiers = [self fixSidedFlags:kModifierFlagControlAny
                            withLeftFlag:kModifierFlagControlLeft
                           withRightFlag:kModifierFlagControlRight
                             withLeftKey:UIKeyboardHIDUsageKeyboardLeftControl
                            withRightKey:UIKeyboardHIDUsageKeyboardRightControl
                             withKeyCode:press.key.keyCode
                                 keyDown:keyDown
                                forFlags:pressedModifiers];
  pressedModifiers = [self fixSidedFlags:kModifierFlagAltAny
                            withLeftFlag:kModifierFlagAltLeft
                           withRightFlag:kModifierFlagAltRight
                             withLeftKey:UIKeyboardHIDUsageKeyboardLeftAlt
                            withRightKey:UIKeyboardHIDUsageKeyboardRightAlt
                             withKeyCode:press.key.keyCode
                                 keyDown:keyDown
                                forFlags:pressedModifiers];
  pressedModifiers = [self fixSidedFlags:kModifierFlagMetaAny
                            withLeftFlag:kModifierFlagMetaLeft
                           withRightFlag:kModifierFlagMetaRight
                             withLeftKey:UIKeyboardHIDUsageKeyboardLeftGUI
                            withRightKey:UIKeyboardHIDUsageKeyboardRightGUI
                             withKeyCode:press.key.keyCode
                                 keyDown:keyDown
                                forFlags:pressedModifiers];

  if (press.key.keyCode == UIKeyboardHIDUsageKeyboardCapsLock) {
    // The caps lock modifier needs to be unset only if it was already on
    // and this is a key up. This is because it indicates the lock state, and
    // not the key press state. The caps lock state should be on between the
    // first down, and the second up (i.e. while the lock in effect), and
    // this code turns it off at the second up event. The OS leaves it on still
    // because of iOS's weird late processing of modifier states. Synthesis of
    // the appropriate synthesized key events happens in synchronizeModifiers.
    if (!keyDown && _lastModifierFlagsOfInterest & kModifierFlagCapsLock) {
      pressedModifiers &= ~kModifierFlagCapsLock;
    }
  }
  return pressedModifiers;
}

@end

namespace {
void HandleResponse(bool handled, void* user_data) {
  FlutterKeyPendingResponse* pending = (__bridge_transfer FlutterKeyPendingResponse*)user_data;
  [pending.responder handleResponse:handled forId:pending.responseId];
}
}  // namespace

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <objc/message.h>

#import "FlutterChannelKeyResponder.h"
#import "KeyCodeMap_Internal.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterCodecs.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/embedder/embedder.h"

@interface FlutterChannelKeyResponder ()

/**
 * The channel used to communicate with Flutter.
 */
@property(nonatomic) FlutterBasicMessageChannel* channel;

/**
 * The |NSEvent.modifierFlags| of the last event received.
 *
 * Used to determine whether a FlagsChanged event should count as a keydown or
 * a keyup event.
 */
@property(nonatomic) uint64_t previouslyPressedFlags;

@end

@implementation FlutterChannelKeyResponder

// Synthesize properties declared in FlutterKeyPrimaryResponder protocol.
@synthesize layoutMap;

- (nonnull instancetype)initWithChannel:(nonnull FlutterBasicMessageChannel*)channel {
  self = [super init];
  if (self != nil) {
    _channel = channel;
    _previouslyPressedFlags = 0;
  }
  return self;
}

/// Checks single modifier flag from event flags and sends appropriate key event
/// if it is different from the previous state.
- (void)checkModifierFlag:(NSUInteger)targetMask
            forEventFlags:(NSEventModifierFlags)eventFlags
                  keyCode:(NSUInteger)keyCode
                timestamp:(NSTimeInterval)timestamp {
  NSAssert((targetMask & (targetMask - 1)) == 0, @"targetMask must only have one bit set");
  if ((eventFlags & targetMask) != (_previouslyPressedFlags & targetMask)) {
    uint64_t newFlags = (_previouslyPressedFlags & ~targetMask) | (eventFlags & targetMask);

    // Sets combined flag if either left or right modifier is pressed, unsets otherwise.
    auto updateCombinedFlag = [&](uint64_t side1, uint64_t side2, NSEventModifierFlags flag) {
      if (newFlags & (side1 | side2)) {
        newFlags |= flag;
      } else {
        newFlags &= ~flag;
      }
    };
    updateCombinedFlag(flutter::kModifierFlagShiftLeft, flutter::kModifierFlagShiftRight,
                       NSEventModifierFlagShift);
    updateCombinedFlag(flutter::kModifierFlagControlLeft, flutter::kModifierFlagControlRight,
                       NSEventModifierFlagControl);
    updateCombinedFlag(flutter::kModifierFlagAltLeft, flutter::kModifierFlagAltRight,
                       NSEventModifierFlagOption);
    updateCombinedFlag(flutter::kModifierFlagMetaLeft, flutter::kModifierFlagMetaRight,
                       NSEventModifierFlagCommand);

    NSEvent* event = [NSEvent keyEventWithType:NSEventTypeFlagsChanged
                                      location:NSZeroPoint
                                 modifierFlags:newFlags
                                     timestamp:timestamp
                                  windowNumber:0
                                       context:nil
                                    characters:@""
                   charactersIgnoringModifiers:@""
                                     isARepeat:NO
                                       keyCode:keyCode];
    [self handleEvent:event
             callback:^(BOOL){
             }];
  };
}

- (void)syncModifiersIfNeeded:(NSEventModifierFlags)modifierFlags
                    timestamp:(NSTimeInterval)timestamp {
  modifierFlags = modifierFlags & ~0x100;
  if (_previouslyPressedFlags == modifierFlags) {
    return;
  }

  [flutter::modifierFlagToKeyCode
      enumerateKeysAndObjectsUsingBlock:^(NSNumber* flag, NSNumber* keyCode, BOOL* stop) {
        [self checkModifierFlag:[flag unsignedShortValue]
                  forEventFlags:modifierFlags
                        keyCode:[keyCode unsignedShortValue]
                      timestamp:timestamp];
      }];

  // Caps lock is not included in the modifierFlagToKeyCode map.
  [self checkModifierFlag:NSEventModifierFlagCapsLock
            forEventFlags:modifierFlags
                  keyCode:0x00000039  // kVK_CapsLock
                timestamp:timestamp];

  // At the end we should end up with the same modifier flags as the event.
  FML_DCHECK(_previouslyPressedFlags == modifierFlags);
}

- (void)handleEvent:(NSEvent*)event callback:(FlutterAsyncKeyCallback)callback {
  // Remove the modifier bits that Flutter is not interested in.
  NSEventModifierFlags modifierFlags = event.modifierFlags & ~0x100;
  NSString* type;
  switch (event.type) {
    case NSEventTypeKeyDown:
      type = @"keydown";
      break;
    case NSEventTypeKeyUp:
      type = @"keyup";
      break;
    case NSEventTypeFlagsChanged:
      if (modifierFlags < _previouslyPressedFlags) {
        type = @"keyup";
      } else if (modifierFlags > _previouslyPressedFlags) {
        type = @"keydown";
      } else {
        // ignore duplicate modifiers; This can happen in situations like switching
        // between application windows when MacOS only sends the up event to new window.
        callback(true);
        return;
      }
      break;
    default:
      [[unlikely]] {
        NSAssert(false, @"Unexpected key event type (got %lu).", event.type);
        callback(false);
        // This should not happen. Return to suppress clang-tidy warning on `type` being nil.
        return;
      }
  }
  _previouslyPressedFlags = modifierFlags;
  NSMutableDictionary* keyMessage = [@{
    @"keymap" : @"macos",
    @"type" : type,
    @"keyCode" : @(event.keyCode),
    @"modifiers" : @(modifierFlags),
  } mutableCopy];
  // Calling these methods on any other type of event
  // (e.g NSEventTypeFlagsChanged) will raise an exception.
  if (event.type == NSEventTypeKeyDown || event.type == NSEventTypeKeyUp) {
    keyMessage[@"characters"] = event.characters;
    keyMessage[@"charactersIgnoringModifiers"] = event.charactersIgnoringModifiers;
  }
  NSNumber* specifiedLogicalKey = layoutMap[@(event.keyCode)];
  if (specifiedLogicalKey != nil) {
    keyMessage[@"specifiedLogicalKey"] = specifiedLogicalKey;
  }
  [self.channel sendMessage:keyMessage
                      reply:^(id reply) {
                        if (!reply) {
                          return callback(true);
                        }
                        // Only propagate the event to other responders if the framework didn't
                        // handle it.
                        callback([[reply valueForKey:@"handled"] boolValue]);
                      }];
}

#pragma mark - Private

@end

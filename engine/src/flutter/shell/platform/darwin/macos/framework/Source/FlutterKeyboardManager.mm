// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyboardManager.h"

#include <cctype>
#include <map>

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterChannelKeyResponder.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEmbedderKeyResponder.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyPrimaryResponder.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyboardLayout.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/KeyCodeMap_Internal.h"

// Turn on this flag to print complete layout data when switching IMEs. The data
// is used in unit tests.
// #define DEBUG_PRINT_LAYOUT

namespace {
using flutter::LayoutClue;
using flutter::LayoutGoal;

#ifdef DEBUG_PRINT_LAYOUT
// Prints layout entries that will be parsed by `MockLayoutData`.
NSString* debugFormatLayoutData(NSString* debugLayoutData,
                                uint16_t keyCode,
                                LayoutClue clue1,
                                LayoutClue clue2) {
  return [NSString
      stringWithFormat:@"    %@%@0x%d%04x, 0x%d%04x,", debugLayoutData,
                       keyCode % 4 == 0 ? [NSString stringWithFormat:@"\n/* 0x%02x */ ", keyCode]
                                        : @" ",
                       clue1.isDeadKey, clue1.character, clue2.isDeadKey, clue2.character];
}
#endif

bool isEascii(const LayoutClue& clue) {
  return clue.character < 256 && !clue.isDeadKey;
}

typedef void (^VoidBlock)();

}  // namespace

@interface FlutterEventWithViewDelegate : NSObject

@property(nonatomic, readonly) NSEvent* event;
@property(nonatomic, readonly) id<FlutterKeyboardManagerViewDelegate> viewDelegate;

- (instancetype)initWithEvent:(NSEvent*)event
                 viewDelegate:(nonnull id<FlutterKeyboardManagerViewDelegate>)viewDelegate;

@end

@implementation FlutterEventWithViewDelegate {
  NSEvent* _event;
  id<FlutterKeyboardManagerViewDelegate> _viewDelegate;
}

- (instancetype)initWithEvent:(NSEvent*)event
                 viewDelegate:(id<FlutterKeyboardManagerViewDelegate>)viewDelegate {
  self = [super init];
  if (self) {
    _event = event;
    _viewDelegate = viewDelegate;
  }
  return self;
}

@end

@interface FlutterKeyboardManager () <FlutterKeyboardLayoutDelegate>

/**
 * The text input plugin set by initialization.
 */
@property(nonatomic, weak) id<FlutterKeyboardManagerDelegate> delegate;

/**
 * The primary responders added by addPrimaryResponder.
 */
@property(nonatomic) NSMutableArray<id<FlutterKeyPrimaryResponder>>* primaryResponders;

@property(nonatomic) NSMutableArray<FlutterEventWithViewDelegate*>* pendingEvents;

@property(nonatomic) BOOL processingEvent;

@property(nonatomic) NSMutableDictionary<NSNumber*, NSNumber*>* layoutMap;

@property(nonatomic, nullable) NSEvent* eventBeingDispatched;

/**
 * Add a primary responder, which asynchronously decides whether to handle an
 * event.
 */
- (void)addPrimaryResponder:(nonnull id<FlutterKeyPrimaryResponder>)responder;

/**
 * Start processing the next event if not started already.
 *
 * This function might initiate an async process, whose callback calls this
 * function again.
 */
- (void)processNextEvent;

/**
 * Implement how to process an event.
 *
 * The `onFinish` must be called eventually, either during this function or
 * asynchronously later, otherwise the event queue will be stuck.
 *
 * This function is called by processNextEvent.
 */
- (void)performProcessEvent:(NSEvent*)event
               viewDelegate:(nonnull id<FlutterKeyboardManagerViewDelegate>)viewDelegate
                   onFinish:(nonnull VoidBlock)onFinish;

/**
 * Dispatch an event that's not hadled by the responders to text input plugin,
 * and potentially to the next responder.
 */
- (void)dispatchTextEvent:(nonnull NSEvent*)pendingEvent
             viewDelegate:(nonnull id<FlutterKeyboardManagerViewDelegate>)viewDelegate;

/**
 * Clears the current layout and build a new one based on the current layout.
 */
- (void)buildLayout;

@end

@implementation FlutterKeyboardManager {
  FlutterKeyboardLayout* _keyboardLayout;
}

- (nonnull instancetype)initWithDelegate:(nonnull id<FlutterKeyboardManagerDelegate>)delegate {
  return [self initWithDelegate:delegate keyboardLayout:[[FlutterKeyboardLayout alloc] init]];
}

- (nonnull instancetype)initWithDelegate:(nonnull id<FlutterKeyboardManagerDelegate>)delegate
                          keyboardLayout:(nonnull FlutterKeyboardLayout*)keyboardLayout {
  self = [super init];
  if (self != nil) {
    _processingEvent = FALSE;
    _delegate = delegate;

    FlutterMethodChannel* keyboardChannel =
        [FlutterMethodChannel methodChannelWithName:@"flutter/keyboard"
                                    binaryMessenger:_delegate.binaryMessenger
                                              codec:[FlutterStandardMethodCodec sharedInstance]];

    [keyboardChannel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
      [self handleKeyboardMethodCall:call result:result];
    }];

    _primaryResponders = [[NSMutableArray alloc] init];

    __weak __typeof__(self) weakSelf = self;
    [self addPrimaryResponder:[[FlutterEmbedderKeyResponder alloc]
                                  initWithSendEvent:^(const FlutterKeyEvent& event,
                                                      FlutterKeyEventCallback callback,
                                                      void* userData) {
                                    __strong __typeof__(weakSelf) strongSelf = weakSelf;
                                    [strongSelf.delegate sendKeyEvent:event
                                                             callback:callback
                                                             userData:userData];
                                  }]];

    [self
        addPrimaryResponder:[[FlutterChannelKeyResponder alloc]
                                initWithChannel:[FlutterBasicMessageChannel
                                                    messageChannelWithName:@"flutter/keyevent"
                                                           binaryMessenger:_delegate.binaryMessenger
                                                                     codec:[FlutterJSONMessageCodec
                                                                               sharedInstance]]]];

    _pendingEvents = [[NSMutableArray alloc] init];
    _layoutMap = [NSMutableDictionary<NSNumber*, NSNumber*> dictionary];

    _keyboardLayout = keyboardLayout;
    _keyboardLayout.delegate = self;
    [self buildLayout];
    for (id<FlutterKeyPrimaryResponder> responder in _primaryResponders) {
      responder.layoutMap = _layoutMap;
    }
  }
  return self;
}

- (void)handleKeyboardMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result {
  if ([[call method] isEqualToString:@"getKeyboardState"]) {
    result([self getPressedState]);
  } else {
    result(FlutterMethodNotImplemented);
  }
}

- (void)addPrimaryResponder:(nonnull id<FlutterKeyPrimaryResponder>)responder {
  [_primaryResponders addObject:responder];
}

- (void)handleEvent:(nonnull NSEvent*)event
    withViewDelegate:(nonnull id<FlutterKeyboardManagerViewDelegate>)viewDelegate {
  // The `handleEvent` does not process the event immediately, but instead put
  // events into a queue. Events are processed one by one by `processNextEvent`.

  // Be sure to add a handling method in propagateKeyEvent when allowing more
  // event types here.
  if (event.type != NSEventTypeKeyDown && event.type != NSEventTypeKeyUp &&
      event.type != NSEventTypeFlagsChanged) {
    return;
  }

  [_pendingEvents addObject:[[FlutterEventWithViewDelegate alloc] initWithEvent:event
                                                                   viewDelegate:viewDelegate]];
  [self processNextEvent];
}

- (BOOL)isDispatchingKeyEvent:(NSEvent*)event {
  return _eventBeingDispatched == event;
}

#pragma mark - Private

- (void)processNextEvent {
  @synchronized(self) {
    if (_processingEvent || [_pendingEvents count] == 0) {
      return;
    }
    _processingEvent = TRUE;
  }

  FlutterEventWithViewDelegate* pendingEvent = [_pendingEvents firstObject];
  [_pendingEvents removeObjectAtIndex:0];

  __weak __typeof__(self) weakSelf = self;
  VoidBlock onFinish = ^() {
    weakSelf.processingEvent = FALSE;
    [weakSelf processNextEvent];
  };
  [self performProcessEvent:pendingEvent.event
               viewDelegate:pendingEvent.viewDelegate
                   onFinish:onFinish];
}

- (void)performProcessEvent:(NSEvent*)event
               viewDelegate:(id<FlutterKeyboardManagerViewDelegate>)viewDelegate
                   onFinish:(VoidBlock)onFinish {
  // Having no primary responders require extra logic, but Flutter hard-codes
  // all primary responders, so this is a situation that Flutter will never
  // encounter.
  NSAssert([_primaryResponders count] >= 0, @"At least one primary responder must be added.");

  __weak __typeof__(self) weakSelf = self;
  __block int unreplied = [_primaryResponders count];
  __block BOOL anyHandled = false;

  FlutterAsyncKeyCallback replyCallback = ^(BOOL handled) {
    unreplied -= 1;
    NSAssert(unreplied >= 0, @"More primary responders replied than possible.");
    anyHandled = anyHandled || handled;
    if (unreplied == 0) {
      if (!anyHandled) {
        [weakSelf dispatchTextEvent:event viewDelegate:viewDelegate];
      }
      onFinish();
    }
  };

  for (id<FlutterKeyPrimaryResponder> responder in _primaryResponders) {
    [responder handleEvent:event callback:replyCallback];
  }
}

- (void)dispatchTextEvent:(NSEvent*)event
             viewDelegate:(id<FlutterKeyboardManagerViewDelegate>)viewDelegate {
  if ([viewDelegate onTextInputKeyEvent:event]) {
    return;
  }
  NSResponder* nextResponder = viewDelegate.nextResponder;
  if (nextResponder == nil) {
    return;
  }
  NSAssert(_eventBeingDispatched == nil, @"An event is already being dispached.");
  _eventBeingDispatched = event;
  switch (event.type) {
    case NSEventTypeKeyDown:
      if ([nextResponder respondsToSelector:@selector(keyDown:)]) {
        [nextResponder keyDown:event];
      }
      break;
    case NSEventTypeKeyUp:
      if ([nextResponder respondsToSelector:@selector(keyUp:)]) {
        [nextResponder keyUp:event];
      }
      break;
    case NSEventTypeFlagsChanged:
      if ([nextResponder respondsToSelector:@selector(flagsChanged:)]) {
        [nextResponder flagsChanged:event];
      }
      break;
    default:
      NSAssert(false, @"Unexpected key event type (got %lu).", event.type);
  }
  NSAssert(_eventBeingDispatched != nil, @"_eventBeingDispatched was cleared unexpectedly.");
  _eventBeingDispatched = nil;
}

- (void)buildLayout {
  [_layoutMap removeAllObjects];

  std::map<uint32_t, LayoutGoal> mandatoryGoalsByChar;
  std::map<uint32_t, LayoutGoal> usLayoutGoalsByKeyCode;
  for (const LayoutGoal& goal : flutter::kLayoutGoals) {
    if (goal.mandatory) {
      mandatoryGoalsByChar[goal.keyChar] = goal;
    } else {
      usLayoutGoalsByKeyCode[goal.keyCode] = goal;
    }
  }

  // Derive key mapping for each key code based on their layout clues.
  // Key code 0x00 - 0x32 are typewriter keys (letters, digits, and symbols.)
  // See keyCodeToPhysicalKey.
  const uint16_t kMaxKeyCode = 0x32;
#ifdef DEBUG_PRINT_LAYOUT
  NSString* debugLayoutData = @"";
#endif
  for (uint16_t keyCode = 0; keyCode <= kMaxKeyCode; keyCode += 1) {
    std::vector<LayoutClue> thisKeyClues = {
        [_keyboardLayout lookUpLayoutForKeyCode:keyCode shift:false],
        [_keyboardLayout lookUpLayoutForKeyCode:keyCode shift:true]};
#ifdef DEBUG_PRINT_LAYOUT
    debugLayoutData =
        debugFormatLayoutData(debugLayoutData, keyCode, thisKeyClues[0], thisKeyClues[1]);
#endif
    // The logical key should be the first available clue from below:
    //
    //  - Mandatory goal, if it matches any clue. This ensures that all alnum
    //    keys can be found somewhere.
    //  - US layout, if neither clue of the key is EASCII. This ensures that
    //    there are no non-latin logical keys.
    //  - Derived on the fly from keyCode & characters.
    for (const LayoutClue& clue : thisKeyClues) {
      uint32_t keyChar = clue.isDeadKey ? 0 : clue.character;
      auto matchingGoal = mandatoryGoalsByChar.find(keyChar);
      if (matchingGoal != mandatoryGoalsByChar.end()) {
        // Found a key that produces a mandatory char. Use it.
        NSAssert(_layoutMap[@(keyCode)] == nil, @"Attempting to assign an assigned key code.");
        _layoutMap[@(keyCode)] = @(keyChar);
        mandatoryGoalsByChar.erase(matchingGoal);
        break;
      }
    }
    bool hasAnyEascii = isEascii(thisKeyClues[0]) || isEascii(thisKeyClues[1]);
    // See if any produced char meets the requirement as a logical key.
    auto foundUsLayoutGoal = usLayoutGoalsByKeyCode.find(keyCode);
    if (foundUsLayoutGoal != usLayoutGoalsByKeyCode.end() && _layoutMap[@(keyCode)] == nil &&
        !hasAnyEascii) {
      _layoutMap[@(keyCode)] = @(foundUsLayoutGoal->second.keyChar);
    }
  }
#ifdef DEBUG_PRINT_LAYOUT
  NSLog(@"%@", debugLayoutData);
#endif

  // Ensure all mandatory goals are assigned.
  for (auto mandatoryGoalIter : mandatoryGoalsByChar) {
    const LayoutGoal& goal = mandatoryGoalIter.second;
    _layoutMap[@(goal.keyCode)] = @(goal.keyChar);
  }
}

- (void)syncModifiersIfNeeded:(NSEventModifierFlags)modifierFlags
                    timestamp:(NSTimeInterval)timestamp {
  for (id<FlutterKeyPrimaryResponder> responder in _primaryResponders) {
    [responder syncModifiersIfNeeded:modifierFlags timestamp:timestamp];
  }
}

- (void)reset {
  _processingEvent = FALSE;
  [_pendingEvents removeAllObjects];
}

/**
 * Returns the keyboard pressed state.
 *
 * Returns the keyboard pressed state. The dictionary contains one entry per
 * pressed keys, mapping from the logical key to the physical key.
 */
- (nonnull NSDictionary*)getPressedState {
  // The embedder responder is the first element in _primaryResponders.
  FlutterEmbedderKeyResponder* embedderResponder =
      (FlutterEmbedderKeyResponder*)_primaryResponders[0];
  return [embedderResponder getPressedState];
}

- (void)keyboardLayoutDidChange {
  [self buildLayout];
}

- (void)replaceKeyboardLayout:(nonnull FlutterKeyboardLayout*)keyboardLayout {
  _keyboardLayout = keyboardLayout;
}

@end

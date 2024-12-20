// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERFAKEKEYEVENTS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERFAKEKEYEVENTS_H_

#import <Foundation/Foundation.h>
#import <OCMock/OCMock.h>
#import <UIKit/UIKit.h>

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUIPressProxy.h"

API_AVAILABLE(ios(13.4))
@interface FakeUIPressProxy : FlutterUIPressProxy
- (instancetype)initWithData:(UIPressPhase)phase
                         key:(UIKey*)key
                        type:(UIEventType)type
                   timestamp:(NSTimeInterval)timestamp API_AVAILABLE(ios(13.4));

- (UIPressPhase)phase API_AVAILABLE(ios(13.4));
- (UIKey*)key API_AVAILABLE(ios(13.4));
- (UIEventType)type API_AVAILABLE(ios(13.4));
- (NSTimeInterval)timestamp API_AVAILABLE(ios(13.4));

@property(nonatomic, readonly) UIPressPhase dataPhase;
@property(nonatomic, readonly) UIKey* dataKey;
@property(nonatomic, readonly) UIEventType dataType;
@property(nonatomic, readonly) NSTimeInterval dataTimestamp;
@end

API_AVAILABLE(ios(13.4))
@interface FakeUIKey : UIKey
- (instancetype)initWithData:(UIKeyboardHIDUsage)keyCode
                  modifierFlags:(UIKeyModifierFlags)modifierFlags
                     characters:(NSString*)characters
    charactersIgnoringModifiers:(NSString*)charactersIgnoringModifiers API_AVAILABLE(ios(13.4));

- (UIKeyboardHIDUsage)keyCode;
- (UIKeyModifierFlags)modifierFlags;
- (NSString*)characters;
- (NSString*)charactersIgnoringModifiers;

@property(assign, nonatomic) UIKeyboardHIDUsage dataKeyCode;
@property(assign, nonatomic) UIKeyModifierFlags dataModifierFlags;
@property(readwrite, nonatomic) NSString* dataCharacters;
@property(readwrite, nonatomic) NSString* dataCharactersIgnoringModifiers;
@end

namespace flutter {
namespace testing {
extern FlutterUIPressProxy* keyDownEvent(UIKeyboardHIDUsage keyCode,
                                         UIKeyModifierFlags modifierFlags = 0x0,
                                         NSTimeInterval timestamp = 0.0f,
                                         const char* characters = "",
                                         const char* charactersIgnoringModifiers = "")
    API_AVAILABLE(ios(13.4));

extern FlutterUIPressProxy* keyUpEvent(UIKeyboardHIDUsage keyCode,
                                       UIKeyModifierFlags modifierFlags = 0x0,
                                       NSTimeInterval timestamp = 0.0f,
                                       const char* characters = "",
                                       const char* charactersIgnoringModifiers = "")
    API_AVAILABLE(ios(13.4));

extern FlutterUIPressProxy* keyEventWithPhase(UIPressPhase phase,
                                              UIKeyboardHIDUsage keyCode,
                                              UIKeyModifierFlags modifierFlags = 0x0,
                                              NSTimeInterval timestamp = 0.0f,
                                              const char* characters = "",
                                              const char* charactersIgnoringModifiers = "")
    API_AVAILABLE(ios(13.4));
}  // namespace testing
}  // namespace flutter
#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERFAKEKEYEVENTS_H_

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERTEXTINPUTDELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERTEXTINPUTDELEGATE_H_

#import <Foundation/Foundation.h>

@class FlutterTextInputPlugin;
@class FlutterTextInputView;

typedef NS_ENUM(NSInteger, FlutterTextInputAction) {
  // NOLINTBEGIN(readability-identifier-naming)
  FlutterTextInputActionUnspecified,
  FlutterTextInputActionDone,
  FlutterTextInputActionGo,
  FlutterTextInputActionSend,
  FlutterTextInputActionSearch,
  FlutterTextInputActionNext,
  FlutterTextInputActionContinue,
  FlutterTextInputActionJoin,
  FlutterTextInputActionRoute,
  FlutterTextInputActionEmergencyCall,
  FlutterTextInputActionNewline,
  // NOLINTEND(readability-identifier-naming)
};

typedef NS_ENUM(NSInteger, FlutterFloatingCursorDragState) {
  // NOLINTBEGIN(readability-identifier-naming)
  FlutterFloatingCursorDragStateStart,
  FlutterFloatingCursorDragStateUpdate,
  FlutterFloatingCursorDragStateEnd,
  // NOLINTEND(readability-identifier-naming)
};

@protocol FlutterTextInputDelegate <NSObject>
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
         updateEditingClient:(int)client
                   withState:(NSDictionary*)state;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
         updateEditingClient:(int)client
                   withState:(NSDictionary*)state
                     withTag:(NSString*)tag;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
         updateEditingClient:(int)client
                   withDelta:(NSDictionary*)state;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
               performAction:(FlutterTextInputAction)action
                  withClient:(int)client;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
        updateFloatingCursor:(FlutterFloatingCursorDragState)state
                  withClient:(int)client
                withPosition:(NSDictionary*)point;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
    showAutocorrectionPromptRectForStart:(NSUInteger)start
                                     end:(NSUInteger)end
                              withClient:(int)client;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView showToolbar:(int)client;
- (void)flutterTextInputViewScribbleInteractionBegan:(FlutterTextInputView*)textInputView;
- (void)flutterTextInputViewScribbleInteractionFinished:(FlutterTextInputView*)textInputView;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
    insertTextPlaceholderWithSize:(CGSize)size
                       withClient:(int)client;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView removeTextPlaceholder:(int)client;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
    didResignFirstResponderWithTextInputClient:(int)client;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
    willDismissEditMenuWithTextInputClient:(int)client;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
           shareSelectedText:(NSString*)selectedText;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
    searchWebWithSelectedText:(NSString*)selectedText;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
          lookUpSelectedText:(NSString*)selectedText;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERTEXTINPUTDELEGATE_H_

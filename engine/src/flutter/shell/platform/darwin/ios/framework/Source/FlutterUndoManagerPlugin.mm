// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUndoManagerPlugin.h"

#pragma mark - UndoManager channel method names.
static NSString* const kSetUndoStateMethod = @"UndoManager.setUndoState";

#pragma mark - Undo State field names
static NSString* const kCanUndo = @"canUndo";
static NSString* const kCanRedo = @"canRedo";

@interface FlutterUndoManagerPlugin ()
@property(nonatomic, weak, readonly) id<FlutterUndoManagerDelegate> undoManagerDelegate;

// When the delegate is `FlutterEngine` this will be the `FlutterViewController`'s undo manager.
// Strongly retain to ensure this target's actions are completely removed during dealloc.
@property(nonatomic) NSUndoManager* undoManager;
@end

@implementation FlutterUndoManagerPlugin

- (instancetype)initWithDelegate:(id<FlutterUndoManagerDelegate>)undoManagerDelegate {
  self = [super init];

  if (self) {
    _undoManagerDelegate = undoManagerDelegate;
  }

  return self;
}

- (void)dealloc {
  [_undoManager removeAllActionsWithTarget:self];
}

- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result {
  NSString* method = call.method;
  id args = call.arguments;
  if ([method isEqualToString:kSetUndoStateMethod]) {
    self.undoManager = self.undoManagerDelegate.undoManager;
    [self setUndoState:args];
    result(nil);
  } else {
    result(FlutterMethodNotImplemented);
  }
}

- (void)resetUndoManager {
  [self.undoManager removeAllActionsWithTarget:self];
}

- (void)registerUndoWithDirection:(FlutterUndoRedoDirection)direction {
  NSUndoManager* undoManager = self.undoManager;
  [undoManager beginUndoGrouping];
  [undoManager registerUndoWithTarget:self
                              handler:^(FlutterUndoManagerPlugin* target) {
                                // Register undo with opposite direction.
                                FlutterUndoRedoDirection newDirection =
                                    (direction == FlutterUndoRedoDirectionRedo)
                                        ? FlutterUndoRedoDirectionUndo
                                        : FlutterUndoRedoDirectionRedo;
                                [target registerUndoWithDirection:newDirection];
                                // Invoke method on delegate.
                                [target.undoManagerDelegate handleUndoWithDirection:direction];
                              }];
  [undoManager endUndoGrouping];
}

- (void)registerRedo {
  NSUndoManager* undoManager = self.undoManager;
  [undoManager beginUndoGrouping];
  [undoManager registerUndoWithTarget:self
                              handler:^(id target) {
                                // Register undo with opposite direction.
                                [target registerUndoWithDirection:FlutterUndoRedoDirectionRedo];
                              }];
  [undoManager endUndoGrouping];
  [undoManager undo];
}

- (void)setUndoState:(NSDictionary*)dictionary {
  NSUndoManager* undoManager = self.undoManager;
  BOOL groupsByEvent = undoManager.groupsByEvent;
  undoManager.groupsByEvent = NO;
  BOOL canUndo = [dictionary[kCanUndo] boolValue];
  BOOL canRedo = [dictionary[kCanRedo] boolValue];

  [self resetUndoManager];

  if (canUndo) {
    [self registerUndoWithDirection:FlutterUndoRedoDirectionUndo];
  }
  if (canRedo) {
    [self registerRedo];
  }
  UIView<UITextInput>* textInputView = self.undoManagerDelegate.activeTextInputView;
  if (textInputView != nil) {
    // This is needed to notify the iPadOS keyboard that it needs to update the
    // state of the UIBarButtons. Otherwise, the state changes to NSUndoManager
    // will not show up until the next keystroke (or other trigger).
    UITextInputAssistantItem* assistantItem = textInputView.inputAssistantItem;
    assistantItem.leadingBarButtonGroups = assistantItem.leadingBarButtonGroups;
  }
  undoManager.groupsByEvent = groupsByEvent;
}

@end

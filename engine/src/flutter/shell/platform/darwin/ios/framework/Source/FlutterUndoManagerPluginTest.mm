// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUndoManagerPlugin.h"

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputPlugin.h"

FLUTTER_ASSERT_ARC

@interface FakeFlutterUndoManagerDelegate : NSObject <FlutterUndoManagerDelegate>

@property(readonly) NSUInteger undoCount;
@property(readonly) NSUInteger redoCount;
@property(nonatomic, nullable) NSUndoManager* undoManager;
@property(nonatomic, nullable) UIView<UITextInput>* activeTextInputView;

- (instancetype)initWithUndoManager:(NSUndoManager*)undoManager
                activeTextInputView:(UIView<UITextInput>*)activeTextInputView;

@end

@implementation FakeFlutterUndoManagerDelegate

- (instancetype)initWithUndoManager:(NSUndoManager*)undoManager
                activeTextInputView:(UIView<UITextInput>*)activeTextInputView {
  self = [super init];
  if (self) {
    _undoManager = undoManager;
    _activeTextInputView = activeTextInputView;
  }
  return self;
}

- (void)handleUndoWithDirection:(FlutterUndoRedoDirection)direction {
  if (direction == FlutterUndoRedoDirectionUndo) {
    _undoCount++;
  } else {
    _redoCount++;
  }
}

@end

@interface FlutterUndoManagerPluginTest : XCTestCase
@property(nonatomic) FakeFlutterUndoManagerDelegate* undoManagerDelegate;
@property(nonatomic) FlutterUndoManagerPlugin* undoManagerPlugin;
@property(nonatomic) UIView<UITextInput>* activeTextInputView;
@property(nonatomic) NSUndoManager* undoManager;
@end

@implementation FlutterUndoManagerPluginTest

- (void)setUp {
  [super setUp];

  self.undoManager = OCMClassMock([NSUndoManager class]);
  self.activeTextInputView = OCMClassMock([FlutterTextInputView class]);

  self.undoManagerDelegate =
      [[FakeFlutterUndoManagerDelegate alloc] initWithUndoManager:self.undoManager
                                              activeTextInputView:self.activeTextInputView];

  self.undoManagerPlugin =
      [[FlutterUndoManagerPlugin alloc] initWithDelegate:self.undoManagerDelegate];
}

- (void)testSetUndoState {
  __block int registerUndoCount = 0;
  __block void (^undoHandler)(id target);
  OCMStub([self.undoManager registerUndoWithTarget:self.undoManagerPlugin handler:[OCMArg any]])
      .andDo(^(NSInvocation* invocation) {
        registerUndoCount++;
        __weak void (^handler)(id target);
        [invocation retainArguments];
        [invocation getArgument:&handler atIndex:3];
        undoHandler = handler;
      });
  __block int removeAllActionsCount = 0;
  OCMStub([self.undoManager removeAllActionsWithTarget:self.undoManagerPlugin])
      .andDo(^(NSInvocation* invocation) {
        removeAllActionsCount++;
      });
  __block int undoCount = 0;
  OCMStub([self.undoManager undo]).andDo(^(NSInvocation* invocation) {
    undoCount++;
    undoHandler(self.undoManagerPlugin);
  });

  // If canUndo and canRedo are false, only removeAllActionsWithTarget is called.
  FlutterMethodCall* setUndoStateCall =
      [FlutterMethodCall methodCallWithMethodName:@"UndoManager.setUndoState"
                                        arguments:@{@"canUndo" : @NO, @"canRedo" : @NO}];
  [self.undoManagerPlugin handleMethodCall:setUndoStateCall
                                    result:^(id _Nullable result){
                                    }];
  XCTAssertEqual(1, removeAllActionsCount);
  XCTAssertEqual(0, registerUndoCount);

  // If canUndo is true, an undo will be registered.
  setUndoStateCall =
      [FlutterMethodCall methodCallWithMethodName:@"UndoManager.setUndoState"
                                        arguments:@{@"canUndo" : @YES, @"canRedo" : @NO}];
  [self.undoManagerPlugin handleMethodCall:setUndoStateCall
                                    result:^(id _Nullable result){
                                    }];
  XCTAssertEqual(2, removeAllActionsCount);
  XCTAssertEqual(1, registerUndoCount);

  // Invoking the undo handler will invoke the handleUndo delegate method with "undo".
  undoHandler(self.undoManagerPlugin);
  XCTAssertEqual(1UL, self.undoManagerDelegate.undoCount);
  XCTAssertEqual(0UL, self.undoManagerDelegate.redoCount);
  XCTAssertEqual(2, registerUndoCount);

  // Invoking the redo handler will invoke the handleUndo delegate method with "redo".
  undoHandler(self.undoManagerPlugin);
  XCTAssertEqual(1UL, self.undoManagerDelegate.undoCount);
  XCTAssertEqual(1UL, self.undoManagerDelegate.redoCount);
  XCTAssertEqual(3, registerUndoCount);

  // If canRedo is true, an undo will be registered and undo will be called.
  setUndoStateCall =
      [FlutterMethodCall methodCallWithMethodName:@"UndoManager.setUndoState"
                                        arguments:@{@"canUndo" : @NO, @"canRedo" : @YES}];
  [self.undoManagerPlugin handleMethodCall:setUndoStateCall
                                    result:^(id _Nullable result){
                                    }];
  XCTAssertEqual(3, removeAllActionsCount);
  XCTAssertEqual(5, registerUndoCount);
  XCTAssertEqual(1, undoCount);

  // Invoking the redo handler will invoke the handleUndo delegate method with "redo".
  undoHandler(self.undoManagerPlugin);
  XCTAssertEqual(1UL, self.undoManagerDelegate.undoCount);
  XCTAssertEqual(2UL, self.undoManagerDelegate.redoCount);
}

- (void)testSetUndoStateDoesInteractWithInputDelegate {
  // Regression test for https://github.com/flutter/flutter/issues/133424
  FlutterMethodCall* setUndoStateCall =
      [FlutterMethodCall methodCallWithMethodName:@"UndoManager.setUndoState"
                                        arguments:@{@"canUndo" : @NO, @"canRedo" : @NO}];
  [self.undoManagerPlugin handleMethodCall:setUndoStateCall
                                    result:^(id _Nullable result){
                                    }];

  OCMVerify(never(), [self.activeTextInputView inputDelegate]);
}

- (void)testDeallocRemovesAllUndoManagerActions {
  __weak FlutterUndoManagerPlugin* weakUndoManagerPlugin;
  // Use a real undo manager.
  NSUndoManager* undoManager = [[NSUndoManager alloc] init];
  @autoreleasepool {
    id activeTextInputView = OCMClassMock([FlutterTextInputView class]);

    FakeFlutterUndoManagerDelegate* undoManagerDelegate =
        [[FakeFlutterUndoManagerDelegate alloc] initWithUndoManager:undoManager
                                                activeTextInputView:activeTextInputView];

    FlutterUndoManagerPlugin* undoManagerPlugin =
        [[FlutterUndoManagerPlugin alloc] initWithDelegate:undoManagerDelegate];
    weakUndoManagerPlugin = undoManagerPlugin;

    FlutterMethodCall* setUndoStateCall =
        [FlutterMethodCall methodCallWithMethodName:@"UndoManager.setUndoState"
                                          arguments:@{@"canUndo" : @YES, @"canRedo" : @YES}];
    [undoManagerPlugin handleMethodCall:setUndoStateCall
                                 result:^(id _Nullable result){
                                 }];
    XCTAssertTrue(undoManager.canUndo);
    XCTAssertTrue(undoManager.canRedo);
    // Fake out the undoManager being nil, which happens when the FlutterViewController deallocs and
    // the undo manager can't be fetched from the FlutterEngine delegate.
    undoManagerDelegate.undoManager = nil;
  }
  XCTAssertNil(weakUndoManagerPlugin);
  // Regression test for https://github.com/flutter/flutter/issues/150408.
  // Undo manager undo and redo stack should be empty after the plugin deallocs.
  XCTAssertFalse(undoManager.canUndo);
  XCTAssertFalse(undoManager.canRedo);
}

@end

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "KeyCodeMap_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"

#import <OCMock/OCMock.h>

#include "flutter/fml/platform/darwin/cf_utils.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterBinaryMessenger.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterDartProject_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngineTestUtils.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterRenderer.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewControllerTestUtils.h"
#include "flutter/shell/platform/embedder/test_utils/key_codes.g.h"
#include "flutter/testing/autoreleasepool_test.h"
#include "flutter/testing/testing.h"

#pragma mark - Test Helper Classes

static const FlutterPointerEvent kDefaultFlutterPointerEvent = {};
static const FlutterKeyEvent kDefaultFlutterKeyEvent = {};

// A wrap to convert FlutterKeyEvent to a ObjC class.
@interface KeyEventWrapper : NSObject
@property(nonatomic) FlutterKeyEvent* data;
- (nonnull instancetype)initWithEvent:(const FlutterKeyEvent*)event;
@end

@implementation KeyEventWrapper
- (instancetype)initWithEvent:(const FlutterKeyEvent*)event {
  self = [super init];
  _data = new FlutterKeyEvent(*event);
  return self;
}

- (void)dealloc {
  delete _data;
}
@end

/// Responder wrapper that forwards key events to another responder. This is a necessary middle step
/// for mocking responder because when setting the responder to controller AppKit will access ivars
/// of the objects, which means it must extend NSResponder instead of just implementing the
/// selectors.
@interface FlutterResponderWrapper : NSResponder {
  NSResponder* _responder;
}
@end

@implementation FlutterResponderWrapper

- (instancetype)initWithResponder:(NSResponder*)responder {
  if (self = [super init]) {
    _responder = responder;
  }
  return self;
}

- (void)keyDown:(NSEvent*)event {
  [_responder keyDown:event];
}

- (void)keyUp:(NSEvent*)event {
  [_responder keyUp:event];
}

- (BOOL)performKeyEquivalent:(NSEvent*)event {
  return [_responder performKeyEquivalent:event];
}

- (void)flagsChanged:(NSEvent*)event {
  [_responder flagsChanged:event];
}

@end

// A FlutterViewController subclass for testing that mouseDown/mouseUp get called when
// mouse events are sent to the associated view.
@interface MouseEventFlutterViewController : FlutterViewController
@property(nonatomic, assign) BOOL mouseDownCalled;
@property(nonatomic, assign) BOOL mouseUpCalled;
@end

@implementation MouseEventFlutterViewController
- (void)mouseDown:(NSEvent*)event {
  self.mouseDownCalled = YES;
}

- (void)mouseUp:(NSEvent*)event {
  self.mouseUpCalled = YES;
}
@end

@interface FlutterViewControllerTestObjC : NSObject
- (bool)testKeyEventsAreSentToFramework:(id)mockEngine;
- (bool)testKeyEventsArePropagatedIfNotHandled:(id)mockEngine;
- (bool)testKeyEventsAreNotPropagatedIfHandled:(id)mockEngine;
- (bool)testCtrlTabKeyEventIsPropagated:(id)mockEngine;
- (bool)testKeyEquivalentIsPassedToTextInputPlugin:(id)mockEngine;
- (bool)testFlagsChangedEventsArePropagatedIfNotHandled:(id)mockEngine;
- (bool)testKeyboardIsRestartedOnEngineRestart:(id)mockEngine;
- (bool)testTrackpadGesturesAreSentToFramework:(id)mockEngine;
- (bool)mouseAndGestureEventsAreHandledSeparately:(id)engineMock;
- (bool)testMouseDownUpEventsSentToNextResponder:(id)mockEngine;
- (bool)testModifierKeysAreSynthesizedOnMouseMove:(id)mockEngine;
- (bool)testViewWillAppearCalledMultipleTimes:(id)mockEngine;
- (bool)testFlutterViewIsConfigured:(id)mockEngine;
- (bool)testLookupKeyAssets;
- (bool)testLookupKeyAssetsWithPackage;
- (bool)testViewControllerIsReleased;

+ (void)respondFalseForSendEvent:(const FlutterKeyEvent&)event
                        callback:(nullable FlutterKeyEventCallback)callback
                        userData:(nullable void*)userData;
@end

#pragma mark - Static helper functions

using namespace ::flutter::testing::keycodes;

namespace flutter::testing {

namespace {

id MockGestureEvent(NSEventType type, NSEventPhase phase, double magnification, double rotation) {
  id event = [OCMockObject mockForClass:[NSEvent class]];
  NSPoint locationInWindow = NSMakePoint(0, 0);
  CGFloat deltaX = 0;
  CGFloat deltaY = 0;
  NSTimeInterval timestamp = 1;
  NSUInteger modifierFlags = 0;
  [(NSEvent*)[[event stub] andReturnValue:OCMOCK_VALUE(type)] type];
  [(NSEvent*)[[event stub] andReturnValue:OCMOCK_VALUE(phase)] phase];
  [(NSEvent*)[[event stub] andReturnValue:OCMOCK_VALUE(locationInWindow)] locationInWindow];
  [(NSEvent*)[[event stub] andReturnValue:OCMOCK_VALUE(deltaX)] deltaX];
  [(NSEvent*)[[event stub] andReturnValue:OCMOCK_VALUE(deltaY)] deltaY];
  [(NSEvent*)[[event stub] andReturnValue:OCMOCK_VALUE(timestamp)] timestamp];
  [(NSEvent*)[[event stub] andReturnValue:OCMOCK_VALUE(modifierFlags)] modifierFlags];
  [(NSEvent*)[[event stub] andReturnValue:OCMOCK_VALUE(magnification)] magnification];
  [(NSEvent*)[[event stub] andReturnValue:OCMOCK_VALUE(rotation)] rotation];
  return event;
}

// Allocates and returns an engine configured for the test fixture resource configuration.
FlutterEngine* CreateTestEngine() {
  NSString* fixtures = @(testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  return [[FlutterEngine alloc] initWithName:@"test" project:project allowHeadlessExecution:true];
}

NSResponder* mockResponder() {
  NSResponder* mock = OCMStrictClassMock([NSResponder class]);
  OCMStub([mock keyDown:[OCMArg any]]).andDo(nil);
  OCMStub([mock keyUp:[OCMArg any]]).andDo(nil);
  OCMStub([mock flagsChanged:[OCMArg any]]).andDo(nil);
  return mock;
}

NSEvent* CreateMouseEvent(NSEventModifierFlags modifierFlags) {
  return [NSEvent mouseEventWithType:NSEventTypeMouseMoved
                            location:NSZeroPoint
                       modifierFlags:modifierFlags
                           timestamp:0
                        windowNumber:0
                             context:nil
                         eventNumber:0
                          clickCount:1
                            pressure:1.0];
}

}  // namespace

#pragma mark - gtest tests

// Test-specific names for AutoreleasePoolTest, MockFlutterEngineTest fixtures.
using FlutterViewControllerTest = AutoreleasePoolTest;
using FlutterViewControllerMockEngineTest = MockFlutterEngineTest;

TEST_F(FlutterViewControllerTest, HasViewThatHidesOtherViewsInAccessibility) {
  FlutterViewController* viewControllerMock = CreateMockViewController();

  [viewControllerMock loadView];
  auto subViews = [viewControllerMock.view subviews];

  EXPECT_EQ([subViews count], 1u);
  EXPECT_EQ(subViews[0], viewControllerMock.flutterView);

  NSTextField* textField = [[NSTextField alloc] initWithFrame:NSMakeRect(0, 0, 1, 1)];
  [viewControllerMock.view addSubview:textField];

  subViews = [viewControllerMock.view subviews];
  EXPECT_EQ([subViews count], 2u);

  auto accessibilityChildren = viewControllerMock.view.accessibilityChildren;
  // The accessibilityChildren should only contains the FlutterView.
  EXPECT_EQ([accessibilityChildren count], 1u);
  EXPECT_EQ(accessibilityChildren[0], viewControllerMock.flutterView);
}

TEST_F(FlutterViewControllerTest, FlutterViewAcceptsFirstMouse) {
  FlutterViewController* viewControllerMock = CreateMockViewController();
  [viewControllerMock loadView];
  EXPECT_EQ([viewControllerMock.flutterView acceptsFirstMouse:nil], YES);
}

TEST_F(FlutterViewControllerTest, ReparentsPluginWhenAccessibilityDisabled) {
  FlutterEngine* engine = CreateTestEngine();
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                nibName:nil
                                                                                 bundle:nil];
  [viewController loadView];
  // Creates a NSWindow so that sub view can be first responder.
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
  window.contentView = viewController.view;
  NSView* dummyView = [[NSView alloc] initWithFrame:CGRectZero];
  [viewController.view addSubview:dummyView];
  // Attaches FlutterTextInputPlugin to the view;
  [dummyView addSubview:viewController.textInputPlugin];
  // Makes sure the textInputPlugin can be the first responder.
  EXPECT_TRUE([window makeFirstResponder:viewController.textInputPlugin]);
  EXPECT_EQ([window firstResponder], viewController.textInputPlugin);
  EXPECT_FALSE(viewController.textInputPlugin.superview == viewController.view);
  [viewController onAccessibilityStatusChanged:NO];
  // FlutterView becomes child of view controller
  EXPECT_TRUE(viewController.textInputPlugin.superview == viewController.view);
}

TEST_F(FlutterViewControllerTest, CanSetMouseTrackingModeBeforeViewLoaded) {
  NSString* fixtures = @(testing::GetFixturesPath());
  FlutterDartProject* project = [[FlutterDartProject alloc]
      initWithAssetsPath:fixtures
             ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithProject:project];
  viewController.mouseTrackingMode = kFlutterMouseTrackingModeInActiveApp;
  ASSERT_EQ(viewController.mouseTrackingMode, kFlutterMouseTrackingModeInActiveApp);
}

TEST_F(FlutterViewControllerMockEngineTest, TestKeyEventsAreSentToFramework) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE([[FlutterViewControllerTestObjC alloc] testKeyEventsAreSentToFramework:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, TestKeyEventsArePropagatedIfNotHandled) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE(
      [[FlutterViewControllerTestObjC alloc] testKeyEventsArePropagatedIfNotHandled:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, TestKeyEventsAreNotPropagatedIfHandled) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE(
      [[FlutterViewControllerTestObjC alloc] testKeyEventsAreNotPropagatedIfHandled:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, TestCtrlTabKeyEventIsPropagated) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE([[FlutterViewControllerTestObjC alloc] testCtrlTabKeyEventIsPropagated:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, TestKeyEquivalentIsPassedToTextInputPlugin) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE([[FlutterViewControllerTestObjC alloc]
      testKeyEquivalentIsPassedToTextInputPlugin:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, TestFlagsChangedEventsArePropagatedIfNotHandled) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE([[FlutterViewControllerTestObjC alloc]
      testFlagsChangedEventsArePropagatedIfNotHandled:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, TestKeyboardIsRestartedOnEngineRestart) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE(
      [[FlutterViewControllerTestObjC alloc] testKeyboardIsRestartedOnEngineRestart:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, TestTrackpadGesturesAreSentToFramework) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE(
      [[FlutterViewControllerTestObjC alloc] testTrackpadGesturesAreSentToFramework:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, TestmouseAndGestureEventsAreHandledSeparately) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE(
      [[FlutterViewControllerTestObjC alloc] mouseAndGestureEventsAreHandledSeparately:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, TestMouseDownUpEventsSentToNextResponder) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE(
      [[FlutterViewControllerTestObjC alloc] testMouseDownUpEventsSentToNextResponder:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, TestModifierKeysAreSynthesizedOnMouseMove) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE(
      [[FlutterViewControllerTestObjC alloc] testModifierKeysAreSynthesizedOnMouseMove:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, testViewWillAppearCalledMultipleTimes) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE(
      [[FlutterViewControllerTestObjC alloc] testViewWillAppearCalledMultipleTimes:mockEngine]);
}

TEST_F(FlutterViewControllerMockEngineTest, testFlutterViewIsConfigured) {
  id mockEngine = GetMockEngine();
  ASSERT_TRUE([[FlutterViewControllerTestObjC alloc] testFlutterViewIsConfigured:mockEngine]);
}

TEST_F(FlutterViewControllerTest, testLookupKeyAssets) {
  ASSERT_TRUE([[FlutterViewControllerTestObjC alloc] testLookupKeyAssets]);
}

TEST_F(FlutterViewControllerTest, testLookupKeyAssetsWithPackage) {
  ASSERT_TRUE([[FlutterViewControllerTestObjC alloc] testLookupKeyAssetsWithPackage]);
}

TEST_F(FlutterViewControllerTest, testViewControllerIsReleased) {
  ASSERT_TRUE([[FlutterViewControllerTestObjC alloc] testViewControllerIsReleased]);
}

}  // namespace flutter::testing

#pragma mark - FlutterViewControllerTestObjC

@implementation FlutterViewControllerTestObjC

- (bool)testKeyEventsAreSentToFramework:(id)engineMock {
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:kDefaultFlutterKeyEvent
                                                  callback:nil
                                                  userData:nil])
      .andCall([FlutterViewControllerTestObjC class],
               @selector(respondFalseForSendEvent:callback:userData:));
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  NSDictionary* expectedEvent = @{
    @"keymap" : @"macos",
    @"type" : @"keydown",
    @"keyCode" : @(65),
    @"modifiers" : @(538968064),
    @"characters" : @".",
    @"charactersIgnoringModifiers" : @".",
  };
  NSData* encodedKeyEvent = [[FlutterJSONMessageCodec sharedInstance] encode:expectedEvent];
  CGEventRef cgEvent = CGEventCreateKeyboardEvent(NULL, 65, TRUE);
  NSEvent* event = [NSEvent eventWithCGEvent:cgEvent];
  [viewController viewWillAppear];  // Initializes the event channel.
  [viewController keyDown:event];
  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/keyevent"
                                   message:encodedKeyEvent
                               binaryReply:[OCMArg any]]);
  } @catch (...) {
    return false;
  }
  return true;
}

// Regression test for https://github.com/flutter/flutter/issues/122084.
- (bool)testCtrlTabKeyEventIsPropagated:(id)engineMock {
  __block bool called = false;
  __block FlutterKeyEvent last_event;
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:kDefaultFlutterKeyEvent
                                                  callback:nil
                                                  userData:nil])
      .andDo((^(NSInvocation* invocation) {
        FlutterKeyEvent* event;
        [invocation getArgument:&event atIndex:2];
        called = true;
        last_event = *event;
      }));
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  // Ctrl+tab
  NSEvent* event = [NSEvent keyEventWithType:NSEventTypeKeyDown
                                    location:NSZeroPoint
                               modifierFlags:0x40101
                                   timestamp:0
                                windowNumber:0
                                     context:nil
                                  characters:@""
                 charactersIgnoringModifiers:@""
                                   isARepeat:NO
                                     keyCode:48];
  const uint64_t kPhysicalKeyTab = 0x7002b;

  [viewController viewWillAppear];  // Initializes the event channel.
  // Creates a NSWindow so that FlutterView view can be first responder.
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
  window.contentView = viewController.view;
  [window makeFirstResponder:viewController.flutterView];
  [viewController.view performKeyEquivalent:event];

  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(last_event.physical, kPhysicalKeyTab);
  return true;
}

- (bool)testKeyEquivalentIsPassedToTextInputPlugin:(id)engineMock {
  __block bool called = false;
  __block FlutterKeyEvent last_event;
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:kDefaultFlutterKeyEvent
                                                  callback:nil
                                                  userData:nil])
      .andDo((^(NSInvocation* invocation) {
        FlutterKeyEvent* event;
        [invocation getArgument:&event atIndex:2];
        called = true;
        last_event = *event;
      }));
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  // Ctrl+tab
  NSEvent* event = [NSEvent keyEventWithType:NSEventTypeKeyDown
                                    location:NSZeroPoint
                               modifierFlags:0x40101
                                   timestamp:0
                                windowNumber:0
                                     context:nil
                                  characters:@""
                 charactersIgnoringModifiers:@""
                                   isARepeat:NO
                                     keyCode:48];
  const uint64_t kPhysicalKeyTab = 0x7002b;

  [viewController viewWillAppear];  // Initializes the event channel.

  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];
  window.contentView = viewController.view;

  [viewController.view addSubview:viewController.textInputPlugin];

  // Make the textInputPlugin first responder. This should still result in
  // view controller reporting the key event.
  [window makeFirstResponder:viewController.textInputPlugin];

  [viewController.view performKeyEquivalent:event];

  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(last_event.physical, kPhysicalKeyTab);
  return true;
}

- (bool)testKeyEventsArePropagatedIfNotHandled:(id)engineMock {
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:kDefaultFlutterKeyEvent
                                                  callback:nil
                                                  userData:nil])
      .andCall([FlutterViewControllerTestObjC class],
               @selector(respondFalseForSendEvent:callback:userData:));
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  id responderMock = flutter::testing::mockResponder();
  id responderWrapper = [[FlutterResponderWrapper alloc] initWithResponder:responderMock];
  viewController.nextResponder = responderWrapper;
  NSDictionary* expectedEvent = @{
    @"keymap" : @"macos",
    @"type" : @"keydown",
    @"keyCode" : @(65),
    @"modifiers" : @(538968064),
    @"characters" : @".",
    @"charactersIgnoringModifiers" : @".",
  };
  NSData* encodedKeyEvent = [[FlutterJSONMessageCodec sharedInstance] encode:expectedEvent];
  CGEventRef cgEvent = CGEventCreateKeyboardEvent(NULL, 65, TRUE);
  NSEvent* event = [NSEvent eventWithCGEvent:cgEvent];
  OCMExpect(  // NOLINT(google-objc-avoid-throwing-exception)
      [binaryMessengerMock sendOnChannel:@"flutter/keyevent"
                                 message:encodedKeyEvent
                             binaryReply:[OCMArg any]])
      .andDo((^(NSInvocation* invocation) {
        FlutterBinaryReply handler;
        [invocation getArgument:&handler atIndex:4];
        NSDictionary* reply = @{
          @"handled" : @(false),
        };
        NSData* encodedReply = [[FlutterJSONMessageCodec sharedInstance] encode:reply];
        handler(encodedReply);
      }));
  [viewController viewWillAppear];  // Initializes the event channel.
  [viewController keyDown:event];
  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [responderMock keyDown:[OCMArg any]]);
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/keyevent"
                                   message:encodedKeyEvent
                               binaryReply:[OCMArg any]]);
  } @catch (...) {
    return false;
  }
  return true;
}

- (bool)testFlutterViewIsConfigured:(id)engineMock {
  FlutterRenderer* renderer_ = [[FlutterRenderer alloc] initWithFlutterEngine:engineMock];
  OCMStub([engineMock renderer]).andReturn(renderer_);

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  [viewController loadView];

  @try {
    // Make sure "renderer" was called during "loadView", which means "flutterView" is created
    OCMVerify([engineMock renderer]);
  } @catch (...) {
    return false;
  }

  return true;
}

- (bool)testFlagsChangedEventsArePropagatedIfNotHandled:(id)engineMock {
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:kDefaultFlutterKeyEvent
                                                  callback:nil
                                                  userData:nil])
      .andCall([FlutterViewControllerTestObjC class],
               @selector(respondFalseForSendEvent:callback:userData:));
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  id responderMock = flutter::testing::mockResponder();
  id responderWrapper = [[FlutterResponderWrapper alloc] initWithResponder:responderMock];
  viewController.nextResponder = responderWrapper;
  NSDictionary* expectedEvent = @{
    @"keymap" : @"macos",
    @"type" : @"keydown",
    @"keyCode" : @(56),  // SHIFT key
    @"modifiers" : @(537001986),
  };
  NSData* encodedKeyEvent = [[FlutterJSONMessageCodec sharedInstance] encode:expectedEvent];
  CGEventRef cgEvent = CGEventCreateKeyboardEvent(NULL, 56, TRUE);  // SHIFT key
  CGEventSetType(cgEvent, kCGEventFlagsChanged);
  NSEvent* event = [NSEvent eventWithCGEvent:cgEvent];
  OCMExpect(  // NOLINT(google-objc-avoid-throwing-exception)
      [binaryMessengerMock sendOnChannel:@"flutter/keyevent"
                                 message:encodedKeyEvent
                             binaryReply:[OCMArg any]])
      .andDo((^(NSInvocation* invocation) {
        FlutterBinaryReply handler;
        [invocation getArgument:&handler atIndex:4];
        NSDictionary* reply = @{
          @"handled" : @(false),
        };
        NSData* encodedReply = [[FlutterJSONMessageCodec sharedInstance] encode:reply];
        handler(encodedReply);
      }));
  [viewController viewWillAppear];  // Initializes the event channel.
  [viewController flagsChanged:event];
  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/keyevent"
                                   message:encodedKeyEvent
                               binaryReply:[OCMArg any]]);
  } @catch (NSException* e) {
    NSLog(@"%@", e.reason);
    return false;
  }
  return true;
}

- (bool)testKeyEventsAreNotPropagatedIfHandled:(id)engineMock {
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:kDefaultFlutterKeyEvent
                                                  callback:nil
                                                  userData:nil])
      .andCall([FlutterViewControllerTestObjC class],
               @selector(respondFalseForSendEvent:callback:userData:));
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  id responderMock = flutter::testing::mockResponder();
  id responderWrapper = [[FlutterResponderWrapper alloc] initWithResponder:responderMock];
  viewController.nextResponder = responderWrapper;
  NSDictionary* expectedEvent = @{
    @"keymap" : @"macos",
    @"type" : @"keydown",
    @"keyCode" : @(65),
    @"modifiers" : @(538968064),
    @"characters" : @".",
    @"charactersIgnoringModifiers" : @".",
  };
  NSData* encodedKeyEvent = [[FlutterJSONMessageCodec sharedInstance] encode:expectedEvent];
  CGEventRef cgEvent = CGEventCreateKeyboardEvent(NULL, 65, TRUE);
  NSEvent* event = [NSEvent eventWithCGEvent:cgEvent];
  OCMExpect(  // NOLINT(google-objc-avoid-throwing-exception)
      [binaryMessengerMock sendOnChannel:@"flutter/keyevent"
                                 message:encodedKeyEvent
                             binaryReply:[OCMArg any]])
      .andDo((^(NSInvocation* invocation) {
        FlutterBinaryReply handler;
        [invocation getArgument:&handler atIndex:4];
        NSDictionary* reply = @{
          @"handled" : @(true),
        };
        NSData* encodedReply = [[FlutterJSONMessageCodec sharedInstance] encode:reply];
        handler(encodedReply);
      }));
  [viewController viewWillAppear];  // Initializes the event channel.
  [viewController keyDown:event];
  @try {
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        never(), [responderMock keyDown:[OCMArg any]]);
    OCMVerify(  // NOLINT(google-objc-avoid-throwing-exception)
        [binaryMessengerMock sendOnChannel:@"flutter/keyevent"
                                   message:encodedKeyEvent
                               binaryReply:[OCMArg any]]);
  } @catch (...) {
    return false;
  }
  return true;
}

- (bool)testKeyboardIsRestartedOnEngineRestart:(id)engineMock {
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);
  __block bool called = false;
  __block FlutterKeyEvent last_event;
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:kDefaultFlutterKeyEvent
                                                  callback:nil
                                                  userData:nil])
      .andDo((^(NSInvocation* invocation) {
        FlutterKeyEvent* event;
        [invocation getArgument:&event atIndex:2];
        called = true;
        last_event = *event;
      }));

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  [viewController viewWillAppear];
  NSEvent* keyADown = [NSEvent keyEventWithType:NSEventTypeKeyDown
                                       location:NSZeroPoint
                                  modifierFlags:0x100
                                      timestamp:0
                                   windowNumber:0
                                        context:nil
                                     characters:@"a"
                    charactersIgnoringModifiers:@"a"
                                      isARepeat:FALSE
                                        keyCode:0];
  const uint64_t kPhysicalKeyA = 0x70004;

  // Send KeyA key down event twice. Without restarting the keyboard during
  // onPreEngineRestart, the second event received will be an empty event with
  // physical key 0x0 because duplicate key down events are ignored.

  called = false;
  [viewController keyDown:keyADown];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(last_event.physical, kPhysicalKeyA);

  [viewController onPreEngineRestart];

  called = false;
  [viewController keyDown:keyADown];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.type, kFlutterKeyEventTypeDown);
  EXPECT_EQ(last_event.physical, kPhysicalKeyA);
  return true;
}

+ (void)respondFalseForSendEvent:(const FlutterKeyEvent&)event
                        callback:(nullable FlutterKeyEventCallback)callback
                        userData:(nullable void*)userData {
  if (callback != nullptr) {
    callback(false, userData);
  }
}

- (bool)testTrackpadGesturesAreSentToFramework:(id)engineMock {
  // Need to return a real renderer to allow view controller to load.
  FlutterRenderer* renderer_ = [[FlutterRenderer alloc] initWithFlutterEngine:engineMock];
  OCMStub([engineMock renderer]).andReturn(renderer_);
  __block bool called = false;
  __block FlutterPointerEvent last_event;
  OCMStub([[engineMock ignoringNonObjectArgs] sendPointerEvent:kDefaultFlutterPointerEvent])
      .andDo((^(NSInvocation* invocation) {
        FlutterPointerEvent* event;
        [invocation getArgument:&event atIndex:2];
        called = true;
        last_event = *event;
      }));

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  [viewController loadView];

  // Test for pan events.
  // Start gesture.
  CGEventRef cgEventStart = CGEventCreateScrollWheelEvent(NULL, kCGScrollEventUnitPixel, 1, 0);
  CGEventSetType(cgEventStart, kCGEventScrollWheel);
  CGEventSetIntegerValueField(cgEventStart, kCGScrollWheelEventScrollPhase, kCGScrollPhaseBegan);
  CGEventSetIntegerValueField(cgEventStart, kCGScrollWheelEventIsContinuous, 1);

  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventStart]];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomStart);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);

  // Update gesture.
  CGEventRef cgEventUpdate = CGEventCreateCopy(cgEventStart);
  CGEventSetIntegerValueField(cgEventUpdate, kCGScrollWheelEventScrollPhase, kCGScrollPhaseChanged);
  CGEventSetIntegerValueField(cgEventUpdate, kCGScrollWheelEventDeltaAxis2, 1);  // pan_x
  CGEventSetIntegerValueField(cgEventUpdate, kCGScrollWheelEventDeltaAxis1, 2);  // pan_y

  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventUpdate]];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomUpdate);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.pan_x, 8 * viewController.flutterView.layer.contentsScale);
  EXPECT_EQ(last_event.pan_y, 16 * viewController.flutterView.layer.contentsScale);

  // Make sure the pan values accumulate.
  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventUpdate]];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomUpdate);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.pan_x, 16 * viewController.flutterView.layer.contentsScale);
  EXPECT_EQ(last_event.pan_y, 32 * viewController.flutterView.layer.contentsScale);

  // End gesture.
  CGEventRef cgEventEnd = CGEventCreateCopy(cgEventStart);
  CGEventSetIntegerValueField(cgEventEnd, kCGScrollWheelEventScrollPhase, kCGScrollPhaseEnded);

  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventEnd]];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomEnd);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);

  // Start system momentum.
  CGEventRef cgEventMomentumStart = CGEventCreateCopy(cgEventStart);
  CGEventSetIntegerValueField(cgEventMomentumStart, kCGScrollWheelEventScrollPhase, 0);
  CGEventSetIntegerValueField(cgEventMomentumStart, kCGScrollWheelEventMomentumPhase,
                              kCGMomentumScrollPhaseBegin);

  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventMomentumStart]];
  EXPECT_FALSE(called);

  // Advance system momentum.
  CGEventRef cgEventMomentumUpdate = CGEventCreateCopy(cgEventStart);
  CGEventSetIntegerValueField(cgEventMomentumUpdate, kCGScrollWheelEventScrollPhase, 0);
  CGEventSetIntegerValueField(cgEventMomentumUpdate, kCGScrollWheelEventMomentumPhase,
                              kCGMomentumScrollPhaseContinue);

  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventMomentumUpdate]];
  EXPECT_FALSE(called);

  // Mock a touch on the trackpad.
  id touchMock = OCMClassMock([NSTouch class]);
  NSSet* touchSet = [NSSet setWithObject:touchMock];
  id touchEventMock1 = OCMClassMock([NSEvent class]);
  OCMStub([touchEventMock1 allTouches]).andReturn(touchSet);
  CGPoint touchLocation = {0, 0};
  OCMStub([touchEventMock1 locationInWindow]).andReturn(touchLocation);
  OCMStub([(NSEvent*)touchEventMock1 timestamp]).andReturn(0.150);  // 150 milliseconds.

  // Scroll inertia cancel event should not be issued (timestamp too far in the future).
  called = false;
  [viewController touchesBeganWithEvent:touchEventMock1];
  EXPECT_FALSE(called);

  // Mock another touch on the trackpad.
  id touchEventMock2 = OCMClassMock([NSEvent class]);
  OCMStub([touchEventMock2 allTouches]).andReturn(touchSet);
  OCMStub([touchEventMock2 locationInWindow]).andReturn(touchLocation);
  OCMStub([(NSEvent*)touchEventMock2 timestamp]).andReturn(0.005);  // 5 milliseconds.

  // Scroll inertia cancel event should be issued.
  called = false;
  [viewController touchesBeganWithEvent:touchEventMock2];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindScrollInertiaCancel);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);

  // End system momentum.
  CGEventRef cgEventMomentumEnd = CGEventCreateCopy(cgEventStart);
  CGEventSetIntegerValueField(cgEventMomentumEnd, kCGScrollWheelEventScrollPhase, 0);
  CGEventSetIntegerValueField(cgEventMomentumEnd, kCGScrollWheelEventMomentumPhase,
                              kCGMomentumScrollPhaseEnd);

  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventMomentumEnd]];
  EXPECT_FALSE(called);

  // May-begin and cancel are used while macOS determines which type of gesture to choose.
  CGEventRef cgEventMayBegin = CGEventCreateCopy(cgEventStart);
  CGEventSetIntegerValueField(cgEventMayBegin, kCGScrollWheelEventScrollPhase,
                              kCGScrollPhaseMayBegin);

  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventMayBegin]];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomStart);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);

  // Cancel gesture.
  CGEventRef cgEventCancel = CGEventCreateCopy(cgEventStart);
  CGEventSetIntegerValueField(cgEventCancel, kCGScrollWheelEventScrollPhase,
                              kCGScrollPhaseCancelled);

  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventCancel]];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomEnd);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);

  // A discrete scroll event should use the PointerSignal system.
  CGEventRef cgEventDiscrete = CGEventCreateScrollWheelEvent(NULL, kCGScrollEventUnitPixel, 1, 0);
  CGEventSetType(cgEventDiscrete, kCGEventScrollWheel);
  CGEventSetIntegerValueField(cgEventDiscrete, kCGScrollWheelEventIsContinuous, 0);
  CGEventSetIntegerValueField(cgEventDiscrete, kCGScrollWheelEventDeltaAxis2, 1);  // scroll_delta_x
  CGEventSetIntegerValueField(cgEventDiscrete, kCGScrollWheelEventDeltaAxis1, 2);  // scroll_delta_y

  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventDiscrete]];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindScroll);
  // pixelsPerLine is 40.0 and direction is reversed.
  EXPECT_EQ(last_event.scroll_delta_x, -40 * viewController.flutterView.layer.contentsScale);
  EXPECT_EQ(last_event.scroll_delta_y, -80 * viewController.flutterView.layer.contentsScale);

  // A discrete scroll event should use the PointerSignal system, and flip the
  // direction when shift is pressed.
  CGEventRef cgEventDiscreteShift =
      CGEventCreateScrollWheelEvent(NULL, kCGScrollEventUnitPixel, 1, 0);
  CGEventSetType(cgEventDiscreteShift, kCGEventScrollWheel);
  CGEventSetFlags(cgEventDiscreteShift, kCGEventFlagMaskShift | flutter::kModifierFlagShiftLeft);
  CGEventSetIntegerValueField(cgEventDiscreteShift, kCGScrollWheelEventIsContinuous, 0);
  CGEventSetIntegerValueField(cgEventDiscreteShift, kCGScrollWheelEventDeltaAxis2,
                              0);  // scroll_delta_x
  CGEventSetIntegerValueField(cgEventDiscreteShift, kCGScrollWheelEventDeltaAxis1,
                              2);  // scroll_delta_y

  called = false;
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventDiscreteShift]];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindScroll);
  // pixelsPerLine is 40.0, direction is reversed and axes have been flipped back.
  EXPECT_FLOAT_EQ(last_event.scroll_delta_x, 0.0 * viewController.flutterView.layer.contentsScale);
  EXPECT_FLOAT_EQ(last_event.scroll_delta_y,
                  -80.0 * viewController.flutterView.layer.contentsScale);

  // Test for scale events.
  // Start gesture.
  called = false;
  [viewController magnifyWithEvent:flutter::testing::MockGestureEvent(NSEventTypeMagnify,
                                                                      NSEventPhaseBegan, 1, 0)];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomStart);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);

  // Update gesture.
  called = false;
  [viewController magnifyWithEvent:flutter::testing::MockGestureEvent(NSEventTypeMagnify,
                                                                      NSEventPhaseChanged, 1, 0)];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomUpdate);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.pan_x, 0);
  EXPECT_EQ(last_event.pan_y, 0);
  EXPECT_EQ(last_event.scale, 2);  // macOS uses logarithmic scaling values, the linear value for
                                   // flutter here should be 2^1 = 2.
  EXPECT_EQ(last_event.rotation, 0);

  // Make sure the scale values accumulate.
  called = false;
  [viewController magnifyWithEvent:flutter::testing::MockGestureEvent(NSEventTypeMagnify,
                                                                      NSEventPhaseChanged, 1, 0)];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomUpdate);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.pan_x, 0);
  EXPECT_EQ(last_event.pan_y, 0);
  EXPECT_EQ(last_event.scale, 4);  // macOS uses logarithmic scaling values, the linear value for
                                   // flutter here should be 2^(1+1) = 2.
  EXPECT_EQ(last_event.rotation, 0);

  // End gesture.
  called = false;
  [viewController magnifyWithEvent:flutter::testing::MockGestureEvent(NSEventTypeMagnify,
                                                                      NSEventPhaseEnded, 0, 0)];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomEnd);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);

  // Test for rotation events.
  // Start gesture.
  called = false;
  [viewController rotateWithEvent:flutter::testing::MockGestureEvent(NSEventTypeRotate,
                                                                     NSEventPhaseBegan, 1, 0)];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomStart);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);

  // Update gesture.
  called = false;
  [viewController rotateWithEvent:flutter::testing::MockGestureEvent(
                                      NSEventTypeRotate, NSEventPhaseChanged, 0, -180)];  // degrees
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomUpdate);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.pan_x, 0);
  EXPECT_EQ(last_event.pan_y, 0);
  EXPECT_EQ(last_event.scale, 1);
  EXPECT_EQ(last_event.rotation, M_PI);  // radians

  // Make sure the rotation values accumulate.
  called = false;
  [viewController rotateWithEvent:flutter::testing::MockGestureEvent(
                                      NSEventTypeRotate, NSEventPhaseChanged, 0, -360)];  // degrees
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomUpdate);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.pan_x, 0);
  EXPECT_EQ(last_event.pan_y, 0);
  EXPECT_EQ(last_event.scale, 1);
  EXPECT_EQ(last_event.rotation, 3 * M_PI);  // radians

  // End gesture.
  called = false;
  [viewController rotateWithEvent:flutter::testing::MockGestureEvent(NSEventTypeRotate,
                                                                     NSEventPhaseEnded, 0, 0)];
  EXPECT_TRUE(called);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);
  EXPECT_EQ(last_event.phase, kPanZoomEnd);
  EXPECT_EQ(last_event.device_kind, kFlutterPointerDeviceKindTrackpad);
  EXPECT_EQ(last_event.signal_kind, kFlutterPointerSignalKindNone);

  // Test that stray NSEventPhaseCancelled event does not crash
  called = false;
  [viewController rotateWithEvent:flutter::testing::MockGestureEvent(NSEventTypeRotate,
                                                                     NSEventPhaseCancelled, 0, 0)];
  EXPECT_FALSE(called);

  return true;
}

// Magic mouse can interleave mouse events with scroll events. This must not crash.
- (bool)mouseAndGestureEventsAreHandledSeparately:(id)engineMock {
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  [viewController loadView];

  // Test for pan events.
  // Start gesture.
  fml::CFRef<CGEventRef> cgEventStart(
      CGEventCreateScrollWheelEvent(NULL, kCGScrollEventUnitPixel, 1, 0));
  CGEventSetType(cgEventStart, kCGEventScrollWheel);
  CGEventSetIntegerValueField(cgEventStart, kCGScrollWheelEventScrollPhase, kCGScrollPhaseBegan);
  CGEventSetIntegerValueField(cgEventStart, kCGScrollWheelEventIsContinuous, 1);
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventStart]];

  fml::CFRef<CGEventRef> cgEventUpdate(CGEventCreateCopy(cgEventStart));
  CGEventSetIntegerValueField(cgEventUpdate, kCGScrollWheelEventScrollPhase, kCGScrollPhaseChanged);
  CGEventSetIntegerValueField(cgEventUpdate, kCGScrollWheelEventDeltaAxis2, 1);  // pan_x
  CGEventSetIntegerValueField(cgEventUpdate, kCGScrollWheelEventDeltaAxis1, 2);  // pan_y
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventUpdate]];

  NSEvent* mouseEvent = flutter::testing::CreateMouseEvent(0x00);
  [viewController mouseEntered:mouseEvent];
  [viewController mouseExited:mouseEvent];

  // End gesture.
  fml::CFRef<CGEventRef> cgEventEnd(CGEventCreateCopy(cgEventStart));
  CGEventSetIntegerValueField(cgEventEnd, kCGScrollWheelEventScrollPhase, kCGScrollPhaseEnded);
  [viewController scrollWheel:[NSEvent eventWithCGEvent:cgEventEnd]];

  return true;
}

- (bool)testViewWillAppearCalledMultipleTimes:(id)engineMock {
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  [viewController viewWillAppear];
  [viewController viewWillAppear];
  return true;
}

- (bool)testLookupKeyAssets {
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithProject:nil];
  NSString* key = [viewController lookupKeyForAsset:@"test.png"];
  EXPECT_TRUE(
      [key isEqualToString:@"Contents/Frameworks/App.framework/Resources/flutter_assets/test.png"]);
  return true;
}

- (bool)testLookupKeyAssetsWithPackage {
  FlutterViewController* viewController = [[FlutterViewController alloc] initWithProject:nil];

  NSString* packageKey = [viewController lookupKeyForAsset:@"test.png" fromPackage:@"test"];
  EXPECT_TRUE([packageKey
      isEqualToString:
          @"Contents/Frameworks/App.framework/Resources/flutter_assets/packages/test/test.png"]);
  return true;
}

static void SwizzledNoop(id self, SEL _cmd) {}

// Verify workaround an AppKit bug where mouseDown/mouseUp are not called on the view controller if
// the view is the content view of an NSPopover AND macOS's Reduced Transparency accessibility
// setting is enabled.
//
// See: https://github.com/flutter/flutter/issues/115015
// See: http://www.openradar.me/FB12050037
// See: https://developer.apple.com/documentation/appkit/nsresponder/1524634-mousedown
//
// TODO(cbracken): https://github.com/flutter/flutter/issues/154063
// Remove this test when we drop support for macOS 12 (Monterey).
- (bool)testMouseDownUpEventsSentToNextResponder:(id)engineMock {
  if (@available(macOS 13.3.1, *)) {
    // This workaround is disabled for macOS 13.3.1 onwards, since the underlying AppKit bug is
    // fixed.
    return true;
  }

  // The root cause of the above bug is NSResponder mouseDown/mouseUp methods that don't correctly
  // walk the responder chain calling the appropriate method on the next responder under certain
  // conditions. Simulate this by swizzling out the default implementations and replacing them with
  // no-ops.
  Method mouseDown = class_getInstanceMethod([NSResponder class], @selector(mouseDown:));
  Method mouseUp = class_getInstanceMethod([NSResponder class], @selector(mouseUp:));
  IMP noopImp = (IMP)SwizzledNoop;
  IMP origMouseDown = method_setImplementation(mouseDown, noopImp);
  IMP origMouseUp = method_setImplementation(mouseUp, noopImp);

  // Verify that mouseDown/mouseUp trigger mouseDown/mouseUp calls on FlutterViewController.
  MouseEventFlutterViewController* viewController =
      [[MouseEventFlutterViewController alloc] initWithEngine:engineMock nibName:@"" bundle:nil];
  FlutterView* view = (FlutterView*)[viewController view];

  EXPECT_FALSE(viewController.mouseDownCalled);
  EXPECT_FALSE(viewController.mouseUpCalled);

  NSEvent* mouseEvent = flutter::testing::CreateMouseEvent(0x00);
  [view mouseDown:mouseEvent];
  EXPECT_TRUE(viewController.mouseDownCalled);
  EXPECT_FALSE(viewController.mouseUpCalled);

  viewController.mouseDownCalled = NO;
  [view mouseUp:mouseEvent];
  EXPECT_FALSE(viewController.mouseDownCalled);
  EXPECT_TRUE(viewController.mouseUpCalled);

  // Restore the original NSResponder mouseDown/mouseUp implementations.
  method_setImplementation(mouseDown, origMouseDown);
  method_setImplementation(mouseUp, origMouseUp);

  return true;
}

- (bool)testModifierKeysAreSynthesizedOnMouseMove:(id)engineMock {
  id binaryMessengerMock = OCMProtocolMock(@protocol(FlutterBinaryMessenger));
  OCMStub(  // NOLINT(google-objc-avoid-throwing-exception)
      [engineMock binaryMessenger])
      .andReturn(binaryMessengerMock);

  // Need to return a real renderer to allow view controller to load.
  FlutterRenderer* renderer_ = [[FlutterRenderer alloc] initWithFlutterEngine:engineMock];
  OCMStub([engineMock renderer]).andReturn(renderer_);

  // Capture calls to sendKeyEvent
  __block NSMutableArray<KeyEventWrapper*>* events = [NSMutableArray array];
  OCMStub([[engineMock ignoringNonObjectArgs] sendKeyEvent:kDefaultFlutterKeyEvent
                                                  callback:nil
                                                  userData:nil])
      .andDo((^(NSInvocation* invocation) {
        FlutterKeyEvent* event;
        [invocation getArgument:&event atIndex:2];
        [events addObject:[[KeyEventWrapper alloc] initWithEvent:event]];
      }));

  __block NSMutableArray<NSDictionary*>* channelEvents = [NSMutableArray array];
  OCMStub([binaryMessengerMock sendOnChannel:@"flutter/keyevent"
                                     message:[OCMArg any]
                                 binaryReply:[OCMArg any]])
      .andDo((^(NSInvocation* invocation) {
        NSData* data;
        [invocation getArgument:&data atIndex:3];
        id event = [[FlutterJSONMessageCodec sharedInstance] decode:data];
        [channelEvents addObject:event];
      }));

  FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                nibName:@""
                                                                                 bundle:nil];
  [viewController loadView];
  [viewController viewWillAppear];

  // Zeroed modifier flag should not synthesize events.
  NSEvent* mouseEvent = flutter::testing::CreateMouseEvent(0x00);
  [viewController mouseMoved:mouseEvent];
  EXPECT_EQ([events count], 0u);

  // For each modifier key, check that key events are synthesized.
  for (NSNumber* keyCode in flutter::keyCodeToModifierFlag) {
    FlutterKeyEvent* event;
    NSDictionary* channelEvent;
    NSNumber* logicalKey;
    NSNumber* physicalKey;
    NSEventModifierFlags flag = [flutter::keyCodeToModifierFlag[keyCode] unsignedLongValue];

    // Cocoa event always contain combined flags.
    if (flag & (flutter::kModifierFlagShiftLeft | flutter::kModifierFlagShiftRight)) {
      flag |= NSEventModifierFlagShift;
    }
    if (flag & (flutter::kModifierFlagControlLeft | flutter::kModifierFlagControlRight)) {
      flag |= NSEventModifierFlagControl;
    }
    if (flag & (flutter::kModifierFlagAltLeft | flutter::kModifierFlagAltRight)) {
      flag |= NSEventModifierFlagOption;
    }
    if (flag & (flutter::kModifierFlagMetaLeft | flutter::kModifierFlagMetaRight)) {
      flag |= NSEventModifierFlagCommand;
    }

    // Should synthesize down event.
    NSEvent* mouseEvent = flutter::testing::CreateMouseEvent(flag);
    [viewController mouseMoved:mouseEvent];
    EXPECT_EQ([events count], 1u);
    event = events[0].data;
    logicalKey = [flutter::keyCodeToLogicalKey objectForKey:keyCode];
    physicalKey = [flutter::keyCodeToPhysicalKey objectForKey:keyCode];
    EXPECT_EQ(event->type, kFlutterKeyEventTypeDown);
    EXPECT_EQ(event->logical, logicalKey.unsignedLongLongValue);
    EXPECT_EQ(event->physical, physicalKey.unsignedLongLongValue);
    EXPECT_EQ(event->synthesized, true);

    channelEvent = channelEvents[0];
    EXPECT_TRUE([channelEvent[@"type"] isEqual:@"keydown"]);
    EXPECT_TRUE([channelEvent[@"keyCode"] isEqual:keyCode]);
    EXPECT_TRUE([channelEvent[@"modifiers"] isEqual:@(flag)]);

    // Should synthesize up event.
    mouseEvent = flutter::testing::CreateMouseEvent(0x00);
    [viewController mouseMoved:mouseEvent];
    EXPECT_EQ([events count], 2u);
    event = events[1].data;
    logicalKey = [flutter::keyCodeToLogicalKey objectForKey:keyCode];
    physicalKey = [flutter::keyCodeToPhysicalKey objectForKey:keyCode];
    EXPECT_EQ(event->type, kFlutterKeyEventTypeUp);
    EXPECT_EQ(event->logical, logicalKey.unsignedLongLongValue);
    EXPECT_EQ(event->physical, physicalKey.unsignedLongLongValue);
    EXPECT_EQ(event->synthesized, true);

    channelEvent = channelEvents[1];
    EXPECT_TRUE([channelEvent[@"type"] isEqual:@"keyup"]);
    EXPECT_TRUE([channelEvent[@"keyCode"] isEqual:keyCode]);
    EXPECT_TRUE([channelEvent[@"modifiers"] isEqual:@(0)]);

    [events removeAllObjects];
    [channelEvents removeAllObjects];
  };

  return true;
}

- (bool)testViewControllerIsReleased {
  __weak FlutterViewController* weakController;
  @autoreleasepool {
    id engineMock = flutter::testing::CreateMockFlutterEngine(@"");

    FlutterRenderer* renderer_ = [[FlutterRenderer alloc] initWithFlutterEngine:engineMock];
    OCMStub([engineMock renderer]).andReturn(renderer_);

    FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engineMock
                                                                                  nibName:@""
                                                                                   bundle:nil];
    [viewController loadView];
    weakController = viewController;

    [engineMock shutDownEngine];
  }

  EXPECT_EQ(weakController, nil);
  return true;
}

@end

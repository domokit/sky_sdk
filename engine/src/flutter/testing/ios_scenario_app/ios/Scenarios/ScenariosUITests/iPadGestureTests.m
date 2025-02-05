// Copyright 2020 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <XCTest/XCTest.h>

static const NSInteger kSecondsToWaitForFlutterView = 30;

@interface iPadGestureTests : XCTestCase

@end

@implementation iPadGestureTests

- (void)setUp {
  [super setUp];
  self.continueAfterFailure = NO;
}

static BOOL performBoolSelector(id target, SEL selector) {
  NSInvocation* invocation = [NSInvocation
      invocationWithMethodSignature:[[target class] instanceMethodSignatureForSelector:selector]];
  [invocation setSelector:selector];
  [invocation setTarget:target];
  [invocation invoke];
  BOOL returnValue;
  [invocation getReturnValue:&returnValue];
  return returnValue;
}

static int assertOneMessageAndGetSequenceNumber(NSMutableDictionary* messages, NSString* message) {
  NSMutableArray<NSNumber*>* matchingMessages = messages[message];
  XCTAssertNotNil(matchingMessages, @"Did not receive \"%@\" message", message);
  XCTAssertEqual(matchingMessages.count, 1, @"More than one \"%@\" message", message);
  return matchingMessages.firstObject.intValue;
}

// TODO(85810): Remove reflection in this test when Xcode version is upgraded to 13.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wundeclared-selector"
#pragma clang diagnostic ignored "-Warc-performSelector-leaks"
- (void)testPointerButtons {
  BOOL supportsPointerInteraction = NO;
  SEL supportsPointerInteractionSelector = @selector(supportsPointerInteraction);
  if ([XCUIDevice.sharedDevice respondsToSelector:supportsPointerInteractionSelector]) {
    supportsPointerInteraction =
        performBoolSelector(XCUIDevice.sharedDevice, supportsPointerInteractionSelector);
  }
  XCTSkipUnless(supportsPointerInteraction, "Device does not support pointer interaction.");
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--pointer-events" ];
  [app launch];

  NSPredicate* predicateToFindFlutterView =
      [NSPredicate predicateWithFormat:@"identifier BEGINSWITH 'flutter_view'"];
  XCUIElement* flutterView = [[app descendantsMatchingType:XCUIElementTypeAny]
      elementMatchingPredicate:predicateToFindFlutterView];
  if (![flutterView waitForExistenceWithTimeout:kSecondsToWaitForFlutterView]) {
    NSLog(@"%@", app.debugDescription);
    XCTFail(@"Failed due to not able to find any flutterView with %@ seconds",
            @(kSecondsToWaitForFlutterView));
  }

  XCTAssertNotNil(flutterView);

  [flutterView tap];
  // Initial add event should have buttons = 0
  XCTAssertTrue(
      [app.textFields[@"0,PointerChange.add,device=0,buttons=0"] waitForExistenceWithTimeout:1],
      @"PointerChange.add event did not occur for a normal tap");
  // Normal tap should have buttons = 0, the flutter framework will ensure it has buttons = 1
  XCTAssertTrue(
      [app.textFields[@"1,PointerChange.down,device=0,buttons=0"] waitForExistenceWithTimeout:1],
      @"PointerChange.down event did not occur for a normal tap");
  XCTAssertTrue(
      [app.textFields[@"2,PointerChange.up,device=0,buttons=0"] waitForExistenceWithTimeout:1],
      @"PointerChange.up event did not occur for a normal tap");
  XCTAssertTrue(
      [app.textFields[@"3,PointerChange.remove,device=0,buttons=0"] waitForExistenceWithTimeout:1],
      @"PointerChange.remove event did not occur for a normal tap");
  SEL rightClick = @selector(rightClick);
  XCTAssertTrue([flutterView respondsToSelector:rightClick],
                @"If supportsPointerInteraction is true, this should be true too.");
  [flutterView performSelector:rightClick];
  // On simulated right click, a hover also occurs, so the hover pointer is added
  XCTAssertTrue(
      [app.textFields[@"4,PointerChange.add,device=1,buttons=0"] waitForExistenceWithTimeout:1],
      @"PointerChange.add event did not occur for a right-click's hover pointer");

  // The hover pointer is removed after ~3.5 seconds, this ensures that all events are received
  XCTestExpectation* sleepExpectation = [self expectationWithDescription:@"never fires"];
  sleepExpectation.inverted = true;
  [self waitForExpectations:@[ sleepExpectation ] timeout:5.0];

  // The hover events are interspersed with the right-click events in a varying order.
  // Ensure the individual orderings are respected without hardcoding the absolute sequence.
  NSMutableDictionary<NSString*, NSMutableArray<NSNumber*>*>* messages =
      [[NSMutableDictionary alloc] init];
  for (XCUIElement* element in [app.textFields allElementsBoundByIndex]) {
    NSString* rawMessage = element.value;
    // Parse out the sequence number
    NSUInteger commaIndex = [rawMessage rangeOfString:@","].location;
    NSInteger messageSequenceNumber =
        [rawMessage substringWithRange:NSMakeRange(0, commaIndex)].integerValue;
    // Parse out the rest of the message
    NSString* message = [rawMessage
        substringWithRange:NSMakeRange(commaIndex + 1, rawMessage.length - (commaIndex + 1))];
    NSMutableArray<NSNumber*>* messageSequenceNumberList = messages[message];
    if (messageSequenceNumberList == nil) {
      messageSequenceNumberList = [[NSMutableArray alloc] init];
      messages[message] = messageSequenceNumberList;
    }
    [messageSequenceNumberList addObject:@(messageSequenceNumber)];
  }
  // The number of hover events is not consistent, there could be one or many.
  NSMutableArray<NSNumber*>* hoverSequenceNumbers =
      messages[@"PointerChange.hover,device=1,buttons=0"];
  int hoverRemovedSequenceNumber =
      assertOneMessageAndGetSequenceNumber(messages, @"PointerChange.remove,device=1,buttons=0");
  // Right click should have buttons = 2
  int rightClickAddedSequenceNumber;
  int rightClickDownSequenceNumber;
  int rightClickUpSequenceNumber;
  if (messages[@"PointerChange.add,device=2,buttons=0"] == nil) {
    // Sometimes the tap pointer has the same device as the right-click (the UITouch is reused)
    rightClickAddedSequenceNumber = 0;
    rightClickDownSequenceNumber =
        assertOneMessageAndGetSequenceNumber(messages, @"PointerChange.down,device=0,buttons=2");
    rightClickUpSequenceNumber =
        assertOneMessageAndGetSequenceNumber(messages, @"PointerChange.up,device=0,buttons=2");
  } else {
    rightClickAddedSequenceNumber =
        assertOneMessageAndGetSequenceNumber(messages, @"PointerChange.add,device=2,buttons=0");
    rightClickDownSequenceNumber =
        assertOneMessageAndGetSequenceNumber(messages, @"PointerChange.down,device=2,buttons=2");
    rightClickUpSequenceNumber =
        assertOneMessageAndGetSequenceNumber(messages, @"PointerChange.up,device=2,buttons=2");
  }
  XCTAssertGreaterThan(rightClickDownSequenceNumber, rightClickAddedSequenceNumber,
                       @"Right-click pointer was pressed before it was added");
  XCTAssertGreaterThan(rightClickUpSequenceNumber, rightClickDownSequenceNumber,
                       @"Right-click pointer was released before it was pressed");
  XCTAssertGreaterThan([[hoverSequenceNumbers firstObject] intValue], 4,
                       @"Hover occured before hover pointer was added");
  XCTAssertGreaterThan(hoverRemovedSequenceNumber, [[hoverSequenceNumbers lastObject] intValue],
                       @"Hover occured after hover pointer was removed");
}

- (void)testPointerHover {
  BOOL supportsPointerInteraction = NO;
  SEL supportsPointerInteractionSelector = @selector(supportsPointerInteraction);
  if ([XCUIDevice.sharedDevice respondsToSelector:supportsPointerInteractionSelector]) {
    supportsPointerInteraction =
        performBoolSelector(XCUIDevice.sharedDevice, supportsPointerInteractionSelector);
  }
  XCTSkipUnless(supportsPointerInteraction, "Device does not support pointer interaction.");
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--pointer-events" ];
  [app launch];

  NSPredicate* predicateToFindFlutterView =
      [NSPredicate predicateWithFormat:@"identifier BEGINSWITH 'flutter_view'"];
  XCUIElement* flutterView = [[app descendantsMatchingType:XCUIElementTypeAny]
      elementMatchingPredicate:predicateToFindFlutterView];
  if (![flutterView waitForExistenceWithTimeout:kSecondsToWaitForFlutterView]) {
    NSLog(@"%@", app.debugDescription);
    XCTFail(@"Failed due to not able to find any flutterView with %@ seconds",
            @(kSecondsToWaitForFlutterView));
  }

  XCTAssertNotNil(flutterView);

  SEL hover = @selector(hover);
  XCTAssertTrue([flutterView respondsToSelector:hover],
                @"If supportsPointerInteraction is true, this should be true too.");
  [flutterView performSelector:hover];
  XCTAssertTrue(
      [app.textFields[@"0,PointerChange.add,device=0,buttons=0"] waitForExistenceWithTimeout:1],
      @"PointerChange.add event did not occur for a hover");
  XCTAssertTrue(
      [app.textFields[@"1,PointerChange.hover,device=0,buttons=0"] waitForExistenceWithTimeout:1],
      @"PointerChange.hover event did not occur for a hover");
  // The number of hover events fired is not always the same
  NSInteger lastHoverSequenceNumber = -1;
  NSPredicate* predicateToFindHoverEvents =
      [NSPredicate predicateWithFormat:@"value ENDSWITH ',PointerChange.hover,device=0,buttons=0'"];
  for (XCUIElement* textField in
       [[app.textFields matchingPredicate:predicateToFindHoverEvents] allElementsBoundByIndex]) {
    NSInteger messageSequenceNumber =
        [[textField.value componentsSeparatedByString:@","] firstObject].integerValue;
    if (messageSequenceNumber > lastHoverSequenceNumber) {
      lastHoverSequenceNumber = messageSequenceNumber;
    }
  }
  XCTAssertNotEqual(lastHoverSequenceNumber, -1,
                    @"PointerChange.hover event did not occur for a hover");
  NSString* removeMessage = [NSString
      stringWithFormat:@"%ld,PointerChange.remove,device=0,buttons=0", lastHoverSequenceNumber + 1];
  XCTAssertTrue([app.textFields[removeMessage] waitForExistenceWithTimeout:1],
                @"PointerChange.remove event did not occur for a hover");
}

- (void)testPointerScroll {
  BOOL supportsPointerInteraction = NO;
  SEL supportsPointerInteractionSelector = @selector(supportsPointerInteraction);
  if ([XCUIDevice.sharedDevice respondsToSelector:supportsPointerInteractionSelector]) {
    supportsPointerInteraction =
        performBoolSelector(XCUIDevice.sharedDevice, supportsPointerInteractionSelector);
  }
  XCTSkipUnless(supportsPointerInteraction, "Device does not support pointer interaction.");
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--pointer-events" ];
  [app launch];

  NSPredicate* predicateToFindFlutterView =
      [NSPredicate predicateWithFormat:@"identifier BEGINSWITH 'flutter_view'"];
  XCUIElement* flutterView = [[app descendantsMatchingType:XCUIElementTypeAny]
      elementMatchingPredicate:predicateToFindFlutterView];
  if (![flutterView waitForExistenceWithTimeout:kSecondsToWaitForFlutterView]) {
    XCTFail(@"Failed due to not able to find any flutterView with %@ seconds",
            @(kSecondsToWaitForFlutterView));
  }

  XCTAssertNotNil(flutterView);

  SEL scroll = @selector(scrollByDeltaX:deltaY:);
  XCTAssertTrue([flutterView respondsToSelector:scroll],
                @"If supportsPointerInteraction is true, this should be true too.");
  // Need to use NSInvocation in order to send primitive arguments to selector.
  NSInvocation* invocation = [NSInvocation
      invocationWithMethodSignature:[XCUIElement instanceMethodSignatureForSelector:scroll]];
  [invocation setSelector:scroll];
  CGFloat deltaX = 0.0;
  CGFloat deltaY = 1000.0;
  [invocation setArgument:&deltaX atIndex:2];
  [invocation setArgument:&deltaY atIndex:3];
  [invocation invokeWithTarget:flutterView];

  // Hover immediately following the scroll to cause an inertia cancel event.
  SEL hover = @selector(hover:);
  XCTAssertTrue([flutterView respondsToSelector:hover],
                @"If supportsPointerInteraction is true, this should be true too.");
  [flutterView performSelector:hover];

  // The hover pointer is observed to be removed by the system after ~3.5 seconds of inactivity.
  // While this is not a documented behavior, it is the only way to test for the removal of the
  // hover pointer. Waiting for 5 seconds will ensure that all events are received before
  // processing.
  XCTestExpectation* sleepExpectation = [self expectationWithDescription:@"never fires"];
  sleepExpectation.inverted = true;
  [self waitForExpectations:@[ sleepExpectation ] timeout:5.0];

  // There are hover events interspersed with the scroll events in a varying order.
  // Ensure the individual orderings are respected without hardcoding the absolute sequence.
  NSMutableDictionary<NSString*, NSMutableArray<NSNumber*>*>* messages =
      [[NSMutableDictionary alloc] init];
  for (XCUIElement* element in [app.textFields allElementsBoundByIndex]) {
    NSString* rawMessage = element.value;
    // Parse out the sequence number
    NSUInteger commaIndex = [rawMessage rangeOfString:@","].location;
    NSInteger messageSequenceNumber =
        [rawMessage substringWithRange:NSMakeRange(0, commaIndex)].integerValue;
    // Parse out the rest of the message
    NSString* message = [rawMessage
        substringWithRange:NSMakeRange(commaIndex + 1, rawMessage.length - (commaIndex + 1))];
    NSMutableArray<NSNumber*>* messageSequenceNumberList = messages[message];
    if (messageSequenceNumberList == nil) {
      messageSequenceNumberList = [[NSMutableArray alloc] init];
      messages[message] = messageSequenceNumberList;
    }
    [messageSequenceNumberList addObject:@(messageSequenceNumber)];
  }
  // The number of hover events is not consistent, there could be one or many.
  int hoverAddedSequenceNumber = assertOneMessageAndGetSequenceNumber(
      messages, @"PointerChange.add,device=0,buttons=0,signalKind=PointerSignalKind.none");
  NSMutableArray<NSNumber*>* hoverSequenceNumbers =
      messages[@"PointerChange.hover,device=0,buttons=0,signalKind=PointerSignalKind.none"];
  int hoverRemovedSequenceNumber = assertOneMessageAndGetSequenceNumber(
      messages, @"PointerChange.remove,device=0,buttons=0,signalKind=PointerSignalKind.none");
  int panZoomAddedSequenceNumber = assertOneMessageAndGetSequenceNumber(
      messages, @"PointerChange.add,device=1,buttons=0,signalKind=PointerSignalKind.none");
  int panZoomStartSequenceNumber = assertOneMessageAndGetSequenceNumber(
      messages, @"PointerChange.panZoomStart,device=1,buttons=0,signalKind=PointerSignalKind.none");
  // The number of pan/zoom update events is not consistent, there could be one or many.
  NSMutableArray<NSNumber*>* panZoomUpdateSequenceNumbers =
      messages[@"PointerChange.panZoomUpdate,device=1,buttons=0,signalKind=PointerSignalKind.none"];
  int panZoomEndSequenceNumber = assertOneMessageAndGetSequenceNumber(
      messages, @"PointerChange.panZoomEnd,device=1,buttons=0,signalKind=PointerSignalKind.none");
  int inertiaCancelSequenceNumber = assertOneMessageAndGetSequenceNumber(
      messages,
      @"PointerChange.cancel,device=2,buttons=0,signalKind=PointerSignalKind.scrollInertiaCancel");

  XCTAssertGreaterThan(panZoomStartSequenceNumber, panZoomAddedSequenceNumber,
                       @"PanZoomStart occured before pointer was added");
  XCTAssertGreaterThan([[panZoomUpdateSequenceNumbers firstObject] intValue],
                       panZoomStartSequenceNumber, @"PanZoomUpdate occured before PanZoomStart");
  XCTAssertGreaterThan(panZoomEndSequenceNumber,
                       [[panZoomUpdateSequenceNumbers lastObject] intValue],
                       @"PanZoomUpdate occured after PanZoomEnd");
  XCTAssertGreaterThan(inertiaCancelSequenceNumber, panZoomEndSequenceNumber,
                       @"ScrollInertiaCancel occured before PanZoomEnd");

  XCTAssertGreaterThan([[hoverSequenceNumbers firstObject] intValue], hoverAddedSequenceNumber,
                       @"Hover occured before pointer was added");
  XCTAssertGreaterThan(hoverRemovedSequenceNumber, [[hoverSequenceNumbers lastObject] intValue],
                       @"Hover occured after pointer was removed");
}
#pragma clang diagnostic pop

@end

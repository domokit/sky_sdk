// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformViews_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterSemanticsScrollView.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTouchInterceptingView_Test.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/SemanticsObject.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/SemanticsObjectTestMocks.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/TextInputSemanticsObject.h"

FLUTTER_ASSERT_ARC

const float kFloatCompareEpsilon = 0.001;

@interface SemanticsObject (UIFocusSystem) <UIFocusItem, UIFocusItemContainer>
@end

@interface FlutterScrollableSemanticsObject (UIFocusItemScrollableContainer) <
    UIFocusItemScrollableContainer>
@end

@interface TextInputSemanticsObject (Test)
- (UIView<UITextInput>*)textInputSurrogate;
@end

@interface SemanticsObjectTest : XCTestCase
@end

@implementation SemanticsObjectTest

- (void)testCreate {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* object = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  XCTAssertNotNil(object);
}

- (void)testSetChildren {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* parent = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  SemanticsObject* child = [[SemanticsObject alloc] initWithBridge:bridge uid:1];
  parent.children = @[ child ];
  XCTAssertEqual(parent, child.parent);
  parent.children = @[];
  XCTAssertNil(child.parent);
}

- (void)testAccessibilityHitTestFocusAtLeaf {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* object0 = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  SemanticsObject* object1 = [[SemanticsObject alloc] initWithBridge:bridge uid:1];
  SemanticsObject* object2 = [[SemanticsObject alloc] initWithBridge:bridge uid:2];
  SemanticsObject* object3 = [[SemanticsObject alloc] initWithBridge:bridge uid:3];
  object0.children = @[ object1 ];
  object0.childrenInHitTestOrder = @[ object1 ];
  object1.children = @[ object2, object3 ];
  object1.childrenInHitTestOrder = @[ object2, object3 ];

  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  node0.label = "0";
  [object0 setSemanticsNode:&node0];

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  node1.label = "1";
  [object1 setSemanticsNode:&node1];

  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.rect = SkRect::MakeXYWH(0, 0, 100, 100);
  node2.label = "2";
  [object2 setSemanticsNode:&node2];

  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  node3.label = "3";
  [object3 setSemanticsNode:&node3];

  CGPoint point = CGPointMake(10, 10);
  id hitTestResult = [object0 _accessibilityHitTest:point withEvent:nil];

  // Focus to object2 because it's the first object in hit test order
  XCTAssertEqual(hitTestResult, object2);
}

- (void)testAccessibilityHitTestNoFocusableItem {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* object0 = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  SemanticsObject* object1 = [[SemanticsObject alloc] initWithBridge:bridge uid:1];
  SemanticsObject* object2 = [[SemanticsObject alloc] initWithBridge:bridge uid:2];
  SemanticsObject* object3 = [[SemanticsObject alloc] initWithBridge:bridge uid:3];
  object0.children = @[ object1 ];
  object0.childrenInHitTestOrder = @[ object1 ];
  object1.children = @[ object2, object3 ];
  object1.childrenInHitTestOrder = @[ object2, object3 ];

  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  [object0 setSemanticsNode:&node0];

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  [object1 setSemanticsNode:&node1];

  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.rect = SkRect::MakeXYWH(0, 0, 100, 100);
  [object2 setSemanticsNode:&node2];

  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  [object3 setSemanticsNode:&node3];

  CGPoint point = CGPointMake(10, 10);
  id hitTestResult = [object0 _accessibilityHitTest:point withEvent:nil];

  XCTAssertNil(hitTestResult);
}

- (void)testAccessibilityScrollToVisible {
  fml::WeakPtrFactory<flutter::testing::MockAccessibilityBridge> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::testing::MockAccessibilityBridge> bridge = factory.GetWeakPtr();
  SemanticsObject* object3 = [[SemanticsObject alloc] initWithBridge:bridge uid:3];

  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  [object3 setSemanticsNode:&node3];

  [object3 accessibilityScrollToVisible];

  XCTAssertTrue(bridge->observations.size() == 1);
  XCTAssertTrue(bridge->observations[0].id == 3);
  XCTAssertTrue(bridge->observations[0].action == flutter::SemanticsAction::kShowOnScreen);
}

- (void)testAccessibilityScrollToVisibleWithChild {
  fml::WeakPtrFactory<flutter::testing::MockAccessibilityBridge> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::testing::MockAccessibilityBridge> bridge = factory.GetWeakPtr();
  SemanticsObject* object3 = [[SemanticsObject alloc] initWithBridge:bridge uid:3];

  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  [object3 setSemanticsNode:&node3];

  [object3 accessibilityScrollToVisibleWithChild:object3];

  XCTAssertTrue(bridge->observations.size() == 1);
  XCTAssertTrue(bridge->observations[0].id == 3);
  XCTAssertTrue(bridge->observations[0].action == flutter::SemanticsAction::kShowOnScreen);
}

- (void)testAccessibilityHitTestOutOfRect {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* object0 = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  SemanticsObject* object1 = [[SemanticsObject alloc] initWithBridge:bridge uid:1];
  SemanticsObject* object2 = [[SemanticsObject alloc] initWithBridge:bridge uid:2];
  SemanticsObject* object3 = [[SemanticsObject alloc] initWithBridge:bridge uid:3];
  object0.children = @[ object1 ];
  object0.childrenInHitTestOrder = @[ object1 ];
  object1.children = @[ object2, object3 ];
  object1.childrenInHitTestOrder = @[ object2, object3 ];

  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  node0.label = "0";
  [object0 setSemanticsNode:&node0];

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  node1.label = "1";
  [object1 setSemanticsNode:&node1];

  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.rect = SkRect::MakeXYWH(0, 0, 100, 100);
  node2.label = "2";
  [object2 setSemanticsNode:&node2];

  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  node3.label = "3";
  [object3 setSemanticsNode:&node3];

  CGPoint point = CGPointMake(300, 300);
  id hitTestResult = [object0 _accessibilityHitTest:point withEvent:nil];

  XCTAssertNil(hitTestResult);
}

- (void)testReplaceChildAtIndex {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* parent = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  SemanticsObject* child1 = [[SemanticsObject alloc] initWithBridge:bridge uid:1];
  SemanticsObject* child2 = [[SemanticsObject alloc] initWithBridge:bridge uid:2];
  parent.children = @[ child1 ];
  [parent replaceChildAtIndex:0 withChild:child2];
  XCTAssertNil(child1.parent);
  XCTAssertEqual(parent, child2.parent);
  XCTAssertEqualObjects(parent.children, @[ child2 ]);
}

- (void)testPlainSemanticsObjectWithLabelHasStaticTextTrait {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  flutter::SemanticsNode node;
  node.label = "foo";
  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  XCTAssertEqual([object accessibilityTraits], UIAccessibilityTraitStaticText);
}

- (void)testNodeWithImplicitScrollIsAnAccessibilityElementWhenItisHidden {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling) |
               static_cast<int32_t>(flutter::SemanticsFlags::kIsHidden);
  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  XCTAssertEqual(object.isAccessibilityElement, YES);
}

- (void)testNodeWithImplicitScrollIsNotAnAccessibilityElementWhenItisNotHidden {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  XCTAssertEqual(object.isAccessibilityElement, NO);
}

- (void)testIntresetingSemanticsObjectWithLabelHasStaticTextTrait {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  flutter::SemanticsNode node;
  node.label = "foo";
  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
  SemanticsObject* child1 = [[SemanticsObject alloc] initWithBridge:bridge uid:1];
  object.children = @[ child1 ];
  [object setSemanticsNode:&node];
  XCTAssertEqual([object accessibilityTraits], UIAccessibilityTraitNone);
}

- (void)testIntresetingSemanticsObjectWithLabelHasStaticTextTrait1 {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  flutter::SemanticsNode node;
  node.label = "foo";
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsTextField);
  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  XCTAssertEqual([object accessibilityTraits], UIAccessibilityTraitNone);
}

- (void)testIntresetingSemanticsObjectWithLabelHasStaticTextTrait2 {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  flutter::SemanticsNode node;
  node.label = "foo";
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsButton);
  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  XCTAssertEqual([object accessibilityTraits], UIAccessibilityTraitButton);
}

- (void)testVerticalFlutterScrollableSemanticsObject {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  float transformScale = 0.5f;
  float screenScale = [[bridge->view() window] screen].scale;
  float effectivelyScale = transformScale / screenScale;
  float x = 10;
  float y = 10;
  float w = 100;
  float h = 200;
  float scrollExtentMax = 500.0;
  float scrollPosition = 150.0;

  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kVerticalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(x, y, w, h);
  node.scrollExtentMax = scrollExtentMax;
  node.scrollPosition = scrollPosition;
  node.transform = {
      transformScale, 0, 0, 0, 0, transformScale, 0, 0, 0, 0, transformScale, 0, 0, 0, 0, 1.0};
  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:0];
  [scrollable setSemanticsNode:&node];
  [scrollable accessibilityBridgeDidFinishUpdate];
  UIScrollView* scrollView = [scrollable nativeAccessibility];

  XCTAssertEqualWithAccuracy(scrollView.frame.origin.x, x * effectivelyScale, kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.origin.y, y * effectivelyScale, kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.size.width, w * effectivelyScale,
                             kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.size.height, h * effectivelyScale,
                             kFloatCompareEpsilon);

  XCTAssertEqualWithAccuracy(scrollView.contentSize.width, w * effectivelyScale,
                             kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.contentSize.height,
                             (h + scrollExtentMax) * effectivelyScale, kFloatCompareEpsilon);

  XCTAssertEqual(scrollView.contentOffset.x, 0);
  XCTAssertEqualWithAccuracy(scrollView.contentOffset.y, scrollPosition * effectivelyScale,
                             kFloatCompareEpsilon);
}

- (void)testVerticalFlutterScrollableSemanticsObjectNoWindowDoesNotCrash {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridgeNoWindow());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  float transformScale = 0.5f;
  float x = 10;
  float y = 10;
  float w = 100;
  float h = 200;
  float scrollExtentMax = 500.0;
  float scrollPosition = 150.0;

  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kVerticalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(x, y, w, h);
  node.scrollExtentMax = scrollExtentMax;
  node.scrollPosition = scrollPosition;
  node.transform = {
      transformScale, 0, 0, 0, 0, transformScale, 0, 0, 0, 0, transformScale, 0, 0, 0, 0, 1.0};
  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:0];
  [scrollable setSemanticsNode:&node];
  [scrollable accessibilityBridgeDidFinishUpdate];
  XCTAssertNoThrow([scrollable accessibilityBridgeDidFinishUpdate]);
}

- (void)testHorizontalFlutterScrollableSemanticsObject {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  float transformScale = 0.5f;
  float screenScale = [[bridge->view() window] screen].scale;
  float effectivelyScale = transformScale / screenScale;
  float x = 10;
  float y = 10;
  float w = 100;
  float h = 200;
  float scrollExtentMax = 500.0;
  float scrollPosition = 150.0;

  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kHorizontalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(x, y, w, h);
  node.scrollExtentMax = scrollExtentMax;
  node.scrollPosition = scrollPosition;
  node.transform = {
      transformScale, 0, 0, 0, 0, transformScale, 0, 0, 0, 0, transformScale, 0, 0, 0, 0, 1.0};
  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:0];
  [scrollable setSemanticsNode:&node];
  [scrollable accessibilityBridgeDidFinishUpdate];
  UIScrollView* scrollView = [scrollable nativeAccessibility];

  XCTAssertEqualWithAccuracy(scrollView.frame.origin.x, x * effectivelyScale, kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.origin.y, y * effectivelyScale, kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.size.width, w * effectivelyScale,
                             kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.size.height, h * effectivelyScale,
                             kFloatCompareEpsilon);

  XCTAssertEqualWithAccuracy(scrollView.contentSize.width, (w + scrollExtentMax) * effectivelyScale,
                             kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.contentSize.height, h * effectivelyScale,
                             kFloatCompareEpsilon);

  XCTAssertEqualWithAccuracy(scrollView.contentOffset.x, scrollPosition * effectivelyScale,
                             kFloatCompareEpsilon);
  XCTAssertEqual(scrollView.contentOffset.y, 0);
}

- (void)testCanHandleInfiniteScrollExtent {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  float transformScale = 0.5f;
  float screenScale = [[bridge->view() window] screen].scale;
  float effectivelyScale = transformScale / screenScale;
  float x = 10;
  float y = 10;
  float w = 100;
  float h = 200;
  float scrollExtentMax = INFINITY;
  float scrollPosition = 150.0;

  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kVerticalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(x, y, w, h);
  node.scrollExtentMax = scrollExtentMax;
  node.scrollPosition = scrollPosition;
  node.transform = {
      transformScale, 0, 0, 0, 0, transformScale, 0, 0, 0, 0, transformScale, 0, 0, 0, 0, 1.0};
  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:0];
  [scrollable setSemanticsNode:&node];
  [scrollable accessibilityBridgeDidFinishUpdate];
  UIScrollView* scrollView = [scrollable nativeAccessibility];
  XCTAssertEqualWithAccuracy(scrollView.frame.origin.x, x * effectivelyScale, kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.origin.y, y * effectivelyScale, kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.size.width, w * effectivelyScale,
                             kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.size.height, h * effectivelyScale,
                             kFloatCompareEpsilon);

  XCTAssertEqualWithAccuracy(scrollView.contentSize.width, w * effectivelyScale,
                             kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.contentSize.height,
                             (h + kScrollExtentMaxForInf + scrollPosition) * effectivelyScale,
                             kFloatCompareEpsilon);

  XCTAssertEqual(scrollView.contentOffset.x, 0);
  XCTAssertEqualWithAccuracy(scrollView.contentOffset.y, scrollPosition * effectivelyScale,
                             kFloatCompareEpsilon);
}

- (void)testCanHandleNaNScrollExtentAndScrollPoisition {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  float transformScale = 0.5f;
  float screenScale = [[bridge->view() window] screen].scale;
  float effectivelyScale = transformScale / screenScale;
  float x = 10;
  float y = 10;
  float w = 100;
  float h = 200;
  float scrollExtentMax = std::nan("");
  float scrollPosition = std::nan("");

  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kVerticalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(x, y, w, h);
  node.scrollExtentMax = scrollExtentMax;
  node.scrollPosition = scrollPosition;
  node.transform = {
      transformScale, 0, 0, 0, 0, transformScale, 0, 0, 0, 0, transformScale, 0, 0, 0, 0, 1.0};
  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:0];
  [scrollable setSemanticsNode:&node];
  [scrollable accessibilityBridgeDidFinishUpdate];
  UIScrollView* scrollView = [scrollable nativeAccessibility];

  XCTAssertEqualWithAccuracy(scrollView.frame.origin.x, x * effectivelyScale, kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.origin.y, y * effectivelyScale, kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.size.width, w * effectivelyScale,
                             kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.frame.size.height, h * effectivelyScale,
                             kFloatCompareEpsilon);

  // Content size equal to the scrollable size.
  XCTAssertEqualWithAccuracy(scrollView.contentSize.width, w * effectivelyScale,
                             kFloatCompareEpsilon);
  XCTAssertEqualWithAccuracy(scrollView.contentSize.height, h * effectivelyScale,
                             kFloatCompareEpsilon);

  XCTAssertEqual(scrollView.contentOffset.x, 0);
  XCTAssertEqual(scrollView.contentOffset.y, 0);
}

- (void)testFlutterScrollableSemanticsObjectIsNotHittestable {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kHorizontalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  node.scrollExtentMax = 100.0;
  node.scrollPosition = 0.0;

  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:0];
  [scrollable setSemanticsNode:&node];
  [scrollable accessibilityBridgeDidFinishUpdate];
  UIScrollView* scrollView = [scrollable nativeAccessibility];
  XCTAssertEqual([scrollView hitTest:CGPointMake(10, 10) withEvent:nil], nil);
}

- (void)testFlutterScrollableSemanticsObjectIsHiddenWhenVoiceOverIsRunning {
  flutter::testing::MockAccessibilityBridge* mock = new flutter::testing::MockAccessibilityBridge();
  mock->isVoiceOverRunningValue = false;
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(mock);
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kHorizontalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  node.scrollExtentMax = 100.0;
  node.scrollPosition = 0.0;

  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:0];
  [scrollable setSemanticsNode:&node];
  [scrollable accessibilityBridgeDidFinishUpdate];
  UIScrollView* scrollView = [scrollable nativeAccessibility];
  XCTAssertTrue(scrollView.isAccessibilityElement);
  mock->isVoiceOverRunningValue = true;
  XCTAssertFalse(scrollView.isAccessibilityElement);
}

- (void)testFlutterSemanticsObjectHasIdentifier {
  flutter::testing::MockAccessibilityBridge* mock = new flutter::testing::MockAccessibilityBridge();
  mock->isVoiceOverRunningValue = true;
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(mock);
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.identifier = "identifier";

  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  XCTAssertTrue([object.accessibilityIdentifier isEqualToString:@"identifier"]);
}

- (void)testFlutterScrollableSemanticsObjectWithLabelValueHintIsNotHiddenWhenVoiceOverIsRunning {
  flutter::testing::MockAccessibilityBridge* mock = new flutter::testing::MockAccessibilityBridge();
  mock->isVoiceOverRunningValue = true;
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(mock);
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kHorizontalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  node.label = "label";
  node.value = "value";
  node.hint = "hint";
  node.scrollExtentMax = 100.0;
  node.scrollPosition = 0.0;

  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:0];
  [scrollable setSemanticsNode:&node];
  [scrollable accessibilityBridgeDidFinishUpdate];
  UIScrollView* scrollView = [scrollable nativeAccessibility];
  XCTAssertTrue(scrollView.isAccessibilityElement);
  XCTAssertTrue(
      [scrollView.accessibilityLabel isEqualToString:NSLocalizedString(@"label", @"test")]);
  XCTAssertTrue(
      [scrollView.accessibilityValue isEqualToString:NSLocalizedString(@"value", @"test")]);
  XCTAssertTrue([scrollView.accessibilityHint isEqualToString:NSLocalizedString(@"hint", @"test")]);
}

- (void)testFlutterSemanticsObjectMergeTooltipToLabel {
  flutter::testing::MockAccessibilityBridge* mock = new flutter::testing::MockAccessibilityBridge();
  mock->isVoiceOverRunningValue = true;
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(mock);
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  node.label = "label";
  node.tooltip = "tooltip";
  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  XCTAssertTrue(object.isAccessibilityElement);
  XCTAssertTrue([object.accessibilityLabel isEqualToString:@"label\ntooltip"]);
}

- (void)testFlutterSemanticsObjectAttributedStringsDoNotCrashWhenEmpty {
  flutter::testing::MockAccessibilityBridge* mock = new flutter::testing::MockAccessibilityBridge();
  mock->isVoiceOverRunningValue = true;
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(mock);
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  XCTAssertTrue(object.accessibilityAttributedLabel == nil);
}

- (void)testFlutterScrollableSemanticsObjectReturnsParentContainerIfNoChildren {
  flutter::testing::MockAccessibilityBridge* mock = new flutter::testing::MockAccessibilityBridge();
  mock->isVoiceOverRunningValue = true;
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(mock);
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode parent;
  parent.id = 0;
  parent.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  parent.label = "label";
  parent.value = "value";
  parent.hint = "hint";

  flutter::SemanticsNode node;
  node.id = 1;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kHorizontalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  node.label = "label";
  node.value = "value";
  node.hint = "hint";
  node.scrollExtentMax = 100.0;
  node.scrollPosition = 0.0;
  parent.childrenInTraversalOrder.push_back(1);

  FlutterSemanticsObject* parentObject = [[FlutterSemanticsObject alloc] initWithBridge:bridge
                                                                                    uid:0];
  [parentObject setSemanticsNode:&parent];

  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:1];
  [scrollable setSemanticsNode:&node];
  UIScrollView* scrollView = [scrollable nativeAccessibility];

  parentObject.children = @[ scrollable ];
  [parentObject accessibilityBridgeDidFinishUpdate];
  [scrollable accessibilityBridgeDidFinishUpdate];
  XCTAssertTrue(scrollView.isAccessibilityElement);
  SemanticsObjectContainer* container =
      static_cast<SemanticsObjectContainer*>(scrollable.accessibilityContainer);
  XCTAssertEqual(container.semanticsObject, parentObject);
}

- (void)testFlutterScrollableSemanticsObjectNoScrollBarOrContentInsets {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kHorizontalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  node.scrollExtentMax = 100.0;
  node.scrollPosition = 0.0;

  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:0];
  [scrollable setSemanticsNode:&node];
  [scrollable accessibilityBridgeDidFinishUpdate];
  UIScrollView* scrollView = [scrollable nativeAccessibility];

  XCTAssertFalse(scrollView.showsHorizontalScrollIndicator);
  XCTAssertFalse(scrollView.showsVerticalScrollIndicator);
  XCTAssertEqual(scrollView.contentInsetAdjustmentBehavior,
                 UIScrollViewContentInsetAdjustmentNever);
  XCTAssertTrue(UIEdgeInsetsEqualToEdgeInsets(scrollView.contentInset, UIEdgeInsetsZero));
}

- (void)testSemanticsObjectBuildsAttributedString {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  flutter::SemanticsNode node;
  node.label = "label";
  std::shared_ptr<flutter::SpellOutStringAttribute> attribute =
      std::make_shared<flutter::SpellOutStringAttribute>();
  attribute->start = 1;
  attribute->end = 2;
  attribute->type = flutter::StringAttributeType::kSpellOut;
  node.labelAttributes.push_back(attribute);
  node.value = "value";
  attribute = std::make_shared<flutter::SpellOutStringAttribute>();
  attribute->start = 2;
  attribute->end = 3;
  attribute->type = flutter::StringAttributeType::kSpellOut;
  node.valueAttributes.push_back(attribute);
  node.hint = "hint";
  std::shared_ptr<flutter::LocaleStringAttribute> local_attribute =
      std::make_shared<flutter::LocaleStringAttribute>();
  local_attribute->start = 3;
  local_attribute->end = 4;
  local_attribute->type = flutter::StringAttributeType::kLocale;
  local_attribute->locale = "en-MX";
  node.hintAttributes.push_back(local_attribute);
  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  NSMutableAttributedString* expectedAttributedLabel =
      [[NSMutableAttributedString alloc] initWithString:NSLocalizedString(@"label", @"test")];
  NSDictionary* attributeDict = @{
    UIAccessibilitySpeechAttributeSpellOut : @YES,
  };
  [expectedAttributedLabel setAttributes:attributeDict range:NSMakeRange(1, 1)];
  XCTAssertTrue(
      [object.accessibilityAttributedLabel isEqualToAttributedString:expectedAttributedLabel]);

  NSMutableAttributedString* expectedAttributedValue =
      [[NSMutableAttributedString alloc] initWithString:NSLocalizedString(@"value", @"test")];
  attributeDict = @{
    UIAccessibilitySpeechAttributeSpellOut : @YES,
  };
  [expectedAttributedValue setAttributes:attributeDict range:NSMakeRange(2, 1)];
  XCTAssertTrue(
      [object.accessibilityAttributedValue isEqualToAttributedString:expectedAttributedValue]);

  NSMutableAttributedString* expectedAttributedHint =
      [[NSMutableAttributedString alloc] initWithString:NSLocalizedString(@"hint", @"test")];
  attributeDict = @{
    UIAccessibilitySpeechAttributeLanguage : @"en-MX",
  };
  [expectedAttributedHint setAttributes:attributeDict range:NSMakeRange(3, 1)];
  XCTAssertTrue(
      [object.accessibilityAttributedHint isEqualToAttributedString:expectedAttributedHint]);
}

- (void)testShouldTriggerAnnouncement {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* object = [[SemanticsObject alloc] initWithBridge:bridge uid:0];

  // Handle nil with no node set.
  XCTAssertFalse([object nodeShouldTriggerAnnouncement:nil]);

  // Handle initial setting of node with liveRegion
  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsLiveRegion);
  node.label = "foo";
  XCTAssertTrue([object nodeShouldTriggerAnnouncement:&node]);

  // Handle nil with node set.
  [object setSemanticsNode:&node];
  XCTAssertFalse([object nodeShouldTriggerAnnouncement:nil]);

  // Handle new node, still has live region, same label.
  XCTAssertFalse([object nodeShouldTriggerAnnouncement:&node]);

  // Handle update node with new label, still has live region.
  flutter::SemanticsNode updatedNode;
  updatedNode.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsLiveRegion);
  updatedNode.label = "bar";
  XCTAssertTrue([object nodeShouldTriggerAnnouncement:&updatedNode]);

  // Handle dropping the live region flag.
  updatedNode.flags = 0;
  XCTAssertFalse([object nodeShouldTriggerAnnouncement:&updatedNode]);

  // Handle adding the flag when the label has not changed.
  updatedNode.label = "foo";
  [object setSemanticsNode:&updatedNode];
  XCTAssertTrue([object nodeShouldTriggerAnnouncement:&node]);
}

- (void)testShouldDispatchShowOnScreenActionForHeader {
  fml::WeakPtrFactory<flutter::testing::MockAccessibilityBridge> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::testing::MockAccessibilityBridge> bridge = factory.GetWeakPtr();
  SemanticsObject* object = [[SemanticsObject alloc] initWithBridge:bridge uid:1];

  // Handle initial setting of node with header.
  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsHeader);
  node.label = "foo";

  [object setSemanticsNode:&node];

  // Simulate accessibility focus.
  [object accessibilityElementDidBecomeFocused];

  XCTAssertTrue(bridge->observations.size() == 1);
  XCTAssertTrue(bridge->observations[0].id == 1);
  XCTAssertTrue(bridge->observations[0].action == flutter::SemanticsAction::kShowOnScreen);
}

- (void)testShouldDispatchShowOnScreenActionForHidden {
  fml::WeakPtrFactory<flutter::testing::MockAccessibilityBridge> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::testing::MockAccessibilityBridge> bridge = factory.GetWeakPtr();
  SemanticsObject* object = [[SemanticsObject alloc] initWithBridge:bridge uid:1];

  // Handle initial setting of node with hidden.
  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsHidden);
  node.label = "foo";

  [object setSemanticsNode:&node];

  // Simulate accessibility focus.
  [object accessibilityElementDidBecomeFocused];

  XCTAssertTrue(bridge->observations.size() == 1);
  XCTAssertTrue(bridge->observations[0].id == 1);
  XCTAssertTrue(bridge->observations[0].action == flutter::SemanticsAction::kShowOnScreen);
}

- (void)testFlutterSwitchSemanticsObjectMatchesUISwitch {
  fml::WeakPtrFactory<flutter::testing::MockAccessibilityBridge> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::testing::MockAccessibilityBridge> bridge = factory.GetWeakPtr();
  FlutterSwitchSemanticsObject* object = [[FlutterSwitchSemanticsObject alloc] initWithBridge:bridge
                                                                                          uid:1];

  // Handle initial setting of node with header.
  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasToggledState) |
               static_cast<int32_t>(flutter::SemanticsFlags::kIsToggled) |
               static_cast<int32_t>(flutter::SemanticsFlags::kIsEnabled);
  node.label = "foo";
  [object setSemanticsNode:&node];
  // Create ab real UISwitch to compare the FlutterSwitchSemanticsObject with.
  UISwitch* nativeSwitch = [[UISwitch alloc] init];
  nativeSwitch.on = YES;

  XCTAssertEqual(object.accessibilityTraits, nativeSwitch.accessibilityTraits);
  XCTAssertEqualObjects(object.accessibilityValue, nativeSwitch.accessibilityValue);

  // Set the toggled to false;
  flutter::SemanticsNode update;
  update.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasToggledState) |
                 static_cast<int32_t>(flutter::SemanticsFlags::kIsEnabled);
  update.label = "foo";
  [object setSemanticsNode:&update];

  nativeSwitch.on = NO;

  XCTAssertEqual(object.accessibilityTraits, nativeSwitch.accessibilityTraits);
  XCTAssertEqualObjects(object.accessibilityValue, nativeSwitch.accessibilityValue);
}

- (void)testFlutterSemanticsObjectOfRadioButton {
  fml::WeakPtrFactory<flutter::testing::MockAccessibilityBridge> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::testing::MockAccessibilityBridge> bridge = factory.GetWeakPtr();
  FlutterSemanticsObject* object = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];

  // Handle initial setting of node with header.
  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsInMutuallyExclusiveGroup) |
               static_cast<int32_t>(flutter::SemanticsFlags::kHasCheckedState) |
               static_cast<int32_t>(flutter::SemanticsFlags::kHasEnabledState) |
               static_cast<int32_t>(flutter::SemanticsFlags::kIsEnabled);
  node.label = "foo";
  [object setSemanticsNode:&node];
  XCTAssertTrue((object.accessibilityTraits & UIAccessibilityTraitButton) > 0);
  XCTAssertNil(object.accessibilityValue);
}

- (void)testFlutterSwitchSemanticsObjectMatchesUISwitchDisabled {
  fml::WeakPtrFactory<flutter::testing::MockAccessibilityBridge> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::testing::MockAccessibilityBridge> bridge = factory.GetWeakPtr();
  FlutterSwitchSemanticsObject* object = [[FlutterSwitchSemanticsObject alloc] initWithBridge:bridge
                                                                                          uid:1];

  // Handle initial setting of node with header.
  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasToggledState) |
               static_cast<int32_t>(flutter::SemanticsFlags::kIsToggled);
  node.label = "foo";
  [object setSemanticsNode:&node];
  // Create ab real UISwitch to compare the FlutterSwitchSemanticsObject with.
  UISwitch* nativeSwitch = [[UISwitch alloc] init];
  nativeSwitch.on = YES;
  nativeSwitch.enabled = NO;

  XCTAssertEqual(object.accessibilityTraits, nativeSwitch.accessibilityTraits);
  XCTAssertEqualObjects(object.accessibilityValue, nativeSwitch.accessibilityValue);
}

- (void)testSemanticsObjectDeallocated {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* parent = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  SemanticsObject* child = [[SemanticsObject alloc] initWithBridge:bridge uid:1];
  parent.children = @[ child ];
  // Validate SemanticsObject deallocation does not crash.
  // https://github.com/flutter/flutter/issues/66032
  __weak SemanticsObject* weakObject = parent;
  parent = nil;
  XCTAssertNil(weakObject);
}

- (void)testFlutterSemanticsObjectReturnsNilContainerWhenBridgeIsNotAlive {
  FlutterSemanticsObject* parentObject;
  FlutterScrollableSemanticsObject* scrollable;
  FlutterSemanticsObject* object2;

  flutter::SemanticsNode parent;
  parent.id = 0;
  parent.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  parent.label = "label";
  parent.value = "value";
  parent.hint = "hint";

  flutter::SemanticsNode node;
  node.id = 1;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kHorizontalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  node.label = "label";
  node.value = "value";
  node.hint = "hint";
  node.scrollExtentMax = 100.0;
  node.scrollPosition = 0.0;
  parent.childrenInTraversalOrder.push_back(1);

  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  node2.label = "label";
  node2.value = "value";
  node2.hint = "hint";
  node2.scrollExtentMax = 100.0;
  node2.scrollPosition = 0.0;
  parent.childrenInTraversalOrder.push_back(2);

  {
    flutter::testing::MockAccessibilityBridge* mock =
        new flutter::testing::MockAccessibilityBridge();
    mock->isVoiceOverRunningValue = true;
    fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(mock);
    fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

    parentObject = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:0];
    [parentObject setSemanticsNode:&parent];

    scrollable = [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:1];
    [scrollable setSemanticsNode:&node];
    [scrollable accessibilityBridgeDidFinishUpdate];

    object2 = [[FlutterSemanticsObject alloc] initWithBridge:bridge uid:2];
    [object2 setSemanticsNode:&node2];

    parentObject.children = @[ scrollable, object2 ];
    [parentObject accessibilityBridgeDidFinishUpdate];
    [scrollable accessibilityBridgeDidFinishUpdate];
    [object2 accessibilityBridgeDidFinishUpdate];

    // Returns the correct container if the bridge is alive.
    SemanticsObjectContainer* container =
        static_cast<SemanticsObjectContainer*>(scrollable.accessibilityContainer);
    XCTAssertEqual(container.semanticsObject, parentObject);
    SemanticsObjectContainer* container2 =
        static_cast<SemanticsObjectContainer*>(object2.accessibilityContainer);
    XCTAssertEqual(container2.semanticsObject, parentObject);
  }
  // The bridge pointer went out of scope and was deallocated.

  XCTAssertNil(scrollable.accessibilityContainer);
  XCTAssertNil(object2.accessibilityContainer);
}

- (void)testAccessibilityHitTestSearchCanReturnPlatformView {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* object0 = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  SemanticsObject* object1 = [[SemanticsObject alloc] initWithBridge:bridge uid:1];
  SemanticsObject* object3 = [[SemanticsObject alloc] initWithBridge:bridge uid:3];
  FlutterTouchInterceptingView* platformView =
      [[FlutterTouchInterceptingView alloc] initWithFrame:CGRectMake(0, 0, 100, 100)];
  FlutterPlatformViewSemanticsContainer* platformViewSemanticsContainer =
      [[FlutterPlatformViewSemanticsContainer alloc] initWithBridge:bridge
                                                                uid:1
                                                       platformView:platformView];

  object0.children = @[ object1 ];
  object0.childrenInHitTestOrder = @[ object1 ];
  object1.children = @[ platformViewSemanticsContainer, object3 ];
  object1.childrenInHitTestOrder = @[ platformViewSemanticsContainer, object3 ];

  flutter::SemanticsNode node0;
  node0.id = 0;
  node0.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  node0.label = "0";
  [object0 setSemanticsNode:&node0];

  flutter::SemanticsNode node1;
  node1.id = 1;
  node1.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  node1.label = "1";
  [object1 setSemanticsNode:&node1];

  flutter::SemanticsNode node2;
  node2.id = 2;
  node2.rect = SkRect::MakeXYWH(0, 0, 100, 100);
  node2.label = "2";
  [platformViewSemanticsContainer setSemanticsNode:&node2];

  flutter::SemanticsNode node3;
  node3.id = 3;
  node3.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  node3.label = "3";
  [object3 setSemanticsNode:&node3];

  CGPoint point = CGPointMake(10, 10);
  id hitTestResult = [object0 _accessibilityHitTest:point withEvent:nil];

  XCTAssertEqual(hitTestResult, platformView);
}

- (void)testFlutterPlatformViewSemanticsContainer {
  fml::WeakPtrFactory<flutter::testing::MockAccessibilityBridge> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::testing::MockAccessibilityBridge> bridge = factory.GetWeakPtr();
  __weak FlutterTouchInterceptingView* weakPlatformView;
  __weak FlutterPlatformViewSemanticsContainer* weakContainer;
  @autoreleasepool {
    FlutterTouchInterceptingView* platformView = [[FlutterTouchInterceptingView alloc] init];
    weakPlatformView = platformView;

    @autoreleasepool {
      FlutterPlatformViewSemanticsContainer* container =
          [[FlutterPlatformViewSemanticsContainer alloc] initWithBridge:bridge
                                                                    uid:1
                                                           platformView:platformView];
      weakContainer = container;
      XCTAssertEqualObjects(platformView.accessibilityContainer, container);
      XCTAssertNotNil(weakPlatformView);
      XCTAssertNotNil(weakContainer);
    }
    // Check the variables are still lived.
    // `container` is `retain` in `platformView`, so it will not be nil here.
    XCTAssertNotNil(weakPlatformView);
    XCTAssertNotNil(weakContainer);
  }
  // Check if there's no more strong references to `platformView` after container and platformView
  // are released.
  XCTAssertNil(weakPlatformView);
  XCTAssertNil(weakContainer);
}

- (void)testTextInputSemanticsObject {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.label = "foo";
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsTextField) |
               static_cast<int32_t>(flutter::SemanticsFlags::kIsReadOnly);
  TextInputSemanticsObject* object = [[TextInputSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  [object accessibilityBridgeDidFinishUpdate];
  XCTAssertEqual([object accessibilityTraits], UIAccessibilityTraitNone);
}

- (void)testTextInputSemanticsObject_canPerformAction {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.label = "foo";
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsTextField) |
               static_cast<int32_t>(flutter::SemanticsFlags::kIsReadOnly);
  TextInputSemanticsObject* object = [[TextInputSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  [object accessibilityBridgeDidFinishUpdate];

  id textInputSurrogate = OCMClassMock([UIResponder class]);
  id partialSemanticsObject = OCMPartialMock(object);
  OCMStub([partialSemanticsObject textInputSurrogate]).andReturn(textInputSurrogate);

  OCMExpect([textInputSurrogate canPerformAction:[OCMArg anySelector] withSender:OCMOCK_ANY])
      .andReturn(YES);
  XCTAssertTrue([partialSemanticsObject canPerformAction:@selector(copy:) withSender:nil]);

  OCMExpect([textInputSurrogate canPerformAction:[OCMArg anySelector] withSender:OCMOCK_ANY])
      .andReturn(NO);
  XCTAssertFalse([partialSemanticsObject canPerformAction:@selector(copy:) withSender:nil]);
}

- (void)testTextInputSemanticsObject_editActions {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.label = "foo";
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsTextField) |
               static_cast<int32_t>(flutter::SemanticsFlags::kIsReadOnly);
  TextInputSemanticsObject* object = [[TextInputSemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  [object accessibilityBridgeDidFinishUpdate];

  id textInputSurrogate = OCMClassMock([UIResponder class]);
  id partialSemanticsObject = OCMPartialMock(object);
  OCMStub([partialSemanticsObject textInputSurrogate]).andReturn(textInputSurrogate);

  XCTestExpectation* copyExpectation =
      [self expectationWithDescription:@"Surrogate's copy method is called."];
  XCTestExpectation* cutExpectation =
      [self expectationWithDescription:@"Surrogate's cut method is called."];
  XCTestExpectation* pasteExpectation =
      [self expectationWithDescription:@"Surrogate's paste method is called."];
  XCTestExpectation* selectAllExpectation =
      [self expectationWithDescription:@"Surrogate's selectAll method is called."];
  XCTestExpectation* deleteExpectation =
      [self expectationWithDescription:@"Surrogate's delete method is called."];

  OCMStub([textInputSurrogate copy:OCMOCK_ANY]).andDo(^(NSInvocation* invocation) {
    [copyExpectation fulfill];
  });
  OCMStub([textInputSurrogate cut:OCMOCK_ANY]).andDo(^(NSInvocation* invocation) {
    [cutExpectation fulfill];
  });
  OCMStub([textInputSurrogate paste:OCMOCK_ANY]).andDo(^(NSInvocation* invocation) {
    [pasteExpectation fulfill];
  });
  OCMStub([textInputSurrogate selectAll:OCMOCK_ANY]).andDo(^(NSInvocation* invocation) {
    [selectAllExpectation fulfill];
  });
  OCMStub([textInputSurrogate delete:OCMOCK_ANY]).andDo(^(NSInvocation* invocation) {
    [deleteExpectation fulfill];
  });

  [partialSemanticsObject copy:nil];
  [partialSemanticsObject cut:nil];
  [partialSemanticsObject paste:nil];
  [partialSemanticsObject selectAll:nil];
  [partialSemanticsObject delete:nil];

  [self waitForExpectationsWithTimeout:1 handler:nil];
}

- (void)testSliderSemanticsObject {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();

  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsSlider);
  SemanticsObject* object = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  [object setSemanticsNode:&node];
  [object accessibilityBridgeDidFinishUpdate];
  XCTAssertEqual([object accessibilityActivate], YES);
}

- (void)testUIFocusItemConformance {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* parent = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  SemanticsObject* child = [[SemanticsObject alloc] initWithBridge:bridge uid:1];
  parent.children = @[ child ];

  // parentFocusEnvironment
  XCTAssertTrue([parent.parentFocusEnvironment isKindOfClass:[UIView class]]);
  XCTAssertEqual(child.parentFocusEnvironment, child.parent);

  // canBecomeFocused
  flutter::SemanticsNode childNode;
  childNode.flags = static_cast<int32_t>(flutter::SemanticsFlags::kIsHidden);
  childNode.actions = static_cast<int32_t>(flutter::SemanticsAction::kTap);
  [child setSemanticsNode:&childNode];
  XCTAssertFalse(child.canBecomeFocused);
  childNode.flags = 0;
  [child setSemanticsNode:&childNode];
  XCTAssertTrue(child.canBecomeFocused);
  childNode.actions = 0;
  [child setSemanticsNode:&childNode];
  XCTAssertFalse(child.canBecomeFocused);

  CGFloat scale = ((bridge->view().window.screen ?: UIScreen.mainScreen)).scale;

  childNode.rect = SkRect::MakeXYWH(0, 0, 100 * scale, 100 * scale);
  [child setSemanticsNode:&childNode];
  flutter::SemanticsNode parentNode;
  parentNode.rect = SkRect::MakeXYWH(0, 0, 200, 200);
  [parent setSemanticsNode:&parentNode];

  XCTAssertTrue(CGRectEqualToRect(child.frame, CGRectMake(0, 0, 100, 100)));
}

- (void)testUIFocusItemContainerConformance {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::AccessibilityBridgeIos> bridge = factory.GetWeakPtr();
  SemanticsObject* parent = [[SemanticsObject alloc] initWithBridge:bridge uid:0];
  SemanticsObject* child1 = [[SemanticsObject alloc] initWithBridge:bridge uid:1];
  SemanticsObject* child2 = [[SemanticsObject alloc] initWithBridge:bridge uid:2];
  parent.childrenInHitTestOrder = @[ child1, child2 ];

  // focusItemsInRect
  NSArray<id<UIFocusItem>>* itemsInRect = [parent focusItemsInRect:CGRectMake(0, 0, 100, 100)];
  XCTAssertEqual(itemsInRect.count, (unsigned long)2);
  XCTAssertTrue([itemsInRect containsObject:child1]);
  XCTAssertTrue([itemsInRect containsObject:child2]);
}

- (void)testUIFocusItemScrollableContainerConformance {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::testing::MockAccessibilityBridge> bridge = factory.GetWeakPtr();
  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:5];

  // setContentOffset
  CGPoint p = CGPointMake(123.0, 456.0);
  [scrollable.scrollView scrollViewWillEndDragging:scrollable.scrollView
                                      withVelocity:CGPointZero
                               targetContentOffset:&p];
  scrollable.scrollView.contentOffset = p;
  [scrollable.scrollView scrollViewDidEndDecelerating:scrollable.scrollView];
  XCTAssertEqual(bridge->observations.size(), (size_t)1);
  XCTAssertEqual(bridge->observations[0].id, 5);
  XCTAssertEqual(bridge->observations[0].action, flutter::SemanticsAction::kScrollToOffset);

  std::vector<uint8_t> args = bridge->observations[0].args;
  XCTAssertEqual(args.size(), 3 * sizeof(CGFloat));

  NSData* encoded = [NSData dataWithBytes:args.data() length:args.size()];
  FlutterStandardTypedData* decoded = [[FlutterStandardMessageCodec sharedInstance] decode:encoded];
  CGPoint point = CGPointZero;
  memcpy(&point, decoded.data.bytes, decoded.data.length);
  XCTAssertTrue(CGPointEqualToPoint(point, p));
}

- (void)testUIFocusItemScrollableContainerNoFeedbackLoops {
  fml::WeakPtrFactory<flutter::AccessibilityBridgeIos> factory(
      new flutter::testing::MockAccessibilityBridge());
  fml::WeakPtr<flutter::testing::MockAccessibilityBridge> bridge = factory.GetWeakPtr();
  FlutterScrollableSemanticsObject* scrollable =
      [[FlutterScrollableSemanticsObject alloc] initWithBridge:bridge uid:5];

  // setContentOffset
  const CGPoint p = CGPointMake(0.0, 456.0);
  scrollable.scrollView.contentOffset = p;
  bridge->observations.clear();

  const SkScalar scrollPosition = p.y + 0.0000000000000001;
  flutter::SemanticsNode node;
  node.flags = static_cast<int32_t>(flutter::SemanticsFlags::kHasImplicitScrolling);
  node.actions = flutter::kVerticalScrollSemanticsActions;
  node.rect = SkRect::MakeXYWH(0, 0, 100, 200);
  node.scrollExtentMax = 10000;
  node.scrollPosition = scrollPosition;
  node.transform = {1.0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 1.0, 0, 0, scrollPosition, 0, 1.0};
  [scrollable setSemanticsNode:&node];
  [scrollable accessibilityBridgeDidFinishUpdate];

  XCTAssertEqual(bridge->observations.size(), (size_t)0);
}
@end

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterAppDelegate.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterViewController.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterAppDelegate_Test.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEngine_Test.h"

FLUTTER_ASSERT_ARC

@interface FlutterAppDelegateTest : XCTestCase
@property(strong) FlutterAppDelegate* appDelegate;

@property(strong) id mockMainBundle;
@property(strong) id mockNavigationChannel;

// Retain callback until the tests are done.
// https://github.com/flutter/flutter/issues/74267
@property(strong) id mockEngineFirstFrameCallback;
@end

@implementation FlutterAppDelegateTest

- (void)setUp {
  [super setUp];

  id mockMainBundle = OCMClassMock([NSBundle class]);
  OCMStub([mockMainBundle mainBundle]).andReturn(mockMainBundle);
  self.mockMainBundle = mockMainBundle;

  FlutterAppDelegate* appDelegate = [[FlutterAppDelegate alloc] init];
  self.appDelegate = appDelegate;

  FlutterViewController* viewController = OCMClassMock([FlutterViewController class]);
  FlutterMethodChannel* navigationChannel = OCMClassMock([FlutterMethodChannel class]);
  self.mockNavigationChannel = navigationChannel;

  FlutterEngine* engine = OCMClassMock([FlutterEngine class]);
  OCMStub([engine navigationChannel]).andReturn(navigationChannel);
  OCMStub([viewController engine]).andReturn(engine);

  id mockEngineFirstFrameCallback = [OCMArg invokeBlockWithArgs:@NO, nil];
  self.mockEngineFirstFrameCallback = mockEngineFirstFrameCallback;
  OCMStub([engine waitForFirstFrame:3.0 callback:mockEngineFirstFrameCallback]);
  appDelegate.rootFlutterViewControllerGetter = ^{
    return viewController;
  };
}

- (void)tearDown {
  // Explicitly stop mocking the NSBundle class property.
  [self.mockMainBundle stopMocking];
  [super tearDown];
}

- (void)testLaunchUrl {
  OCMStub([self.mockMainBundle objectForInfoDictionaryKey:@"FlutterDeepLinkingEnabled"])
      .andReturn(@YES);

  OCMStub([self.mockNavigationChannel
              invokeMethod:@"pushRouteInformation"
                 arguments:@{@"location" : @"http://myApp/custom/route?query=test"}])
      .andReturn(@YES);

  BOOL result =
      [self.appDelegate application:[UIApplication sharedApplication]
                            openURL:[NSURL URLWithString:@"http://myApp/custom/route?query=test"]
                            options:@{}];

  XCTAssertTrue(result);
  OCMVerifyAll(self.mockNavigationChannel);
}

- (void)testLaunchUrlWithDeepLinkingNotSet {
  OCMStub([self.mockMainBundle objectForInfoDictionaryKey:@"FlutterDeepLinkingEnabled"])
      .andReturn(nil);

  OCMStub([self.mockNavigationChannel
              invokeMethod:@"pushRouteInformation"
                 arguments:@{@"location" : @"http://myApp/custom/route?query=test"}])
      .andReturn(@YES);

  BOOL result =
      [self.appDelegate application:[UIApplication sharedApplication]
                            openURL:[NSURL URLWithString:@"http://myApp/custom/route?query=test"]
                            options:@{}];

  XCTAssertTrue(result);
  OCMVerifyAll(self.mockNavigationChannel);
}

- (void)testLaunchUrlWithDeepLinkingDisabled {
  OCMStub([self.mockMainBundle objectForInfoDictionaryKey:@"FlutterDeepLinkingEnabled"])
      .andReturn(@NO);

  BOOL result =
      [self.appDelegate application:[UIApplication sharedApplication]
                            openURL:[NSURL URLWithString:@"http://myApp/custom/route?query=test"]
                            options:@{}];
  XCTAssertFalse(result);
  OCMReject([self.mockNavigationChannel invokeMethod:OCMOCK_ANY arguments:OCMOCK_ANY]);
}

- (void)testLaunchUrlWithQueryParameterAndFragment {
  OCMStub([self.mockMainBundle objectForInfoDictionaryKey:@"FlutterDeepLinkingEnabled"])
      .andReturn(@YES);
  OCMStub([self.mockNavigationChannel
              invokeMethod:@"pushRouteInformation"
                 arguments:@{@"location" : @"http://myApp/custom/route?query=test#fragment"}])
      .andReturn(@YES);
  BOOL result = [self.appDelegate
      application:[UIApplication sharedApplication]
          openURL:[NSURL URLWithString:@"http://myApp/custom/route?query=test#fragment"]
          options:@{}];
  XCTAssertTrue(result);
  OCMVerifyAll(self.mockNavigationChannel);
}

- (void)testLaunchUrlWithFragmentNoQueryParameter {
  OCMStub([self.mockMainBundle objectForInfoDictionaryKey:@"FlutterDeepLinkingEnabled"])
      .andReturn(@YES);
  OCMStub([self.mockNavigationChannel
              invokeMethod:@"pushRouteInformation"
                 arguments:@{@"location" : @"http://myApp/custom/route#fragment"}])
      .andReturn(@YES);
  BOOL result =
      [self.appDelegate application:[UIApplication sharedApplication]
                            openURL:[NSURL URLWithString:@"http://myApp/custom/route#fragment"]
                            options:@{}];
  XCTAssertTrue(result);
  OCMVerifyAll(self.mockNavigationChannel);
}

- (void)testReleasesWindowOnDealloc {
  __weak UIWindow* weakWindow;
  @autoreleasepool {
    id mockWindow = OCMClassMock([UIWindow class]);
    FlutterAppDelegate* appDelegate = [[FlutterAppDelegate alloc] init];
    appDelegate.window = mockWindow;
    weakWindow = mockWindow;
    XCTAssertNotNil(weakWindow);
    [mockWindow stopMocking];
    mockWindow = nil;
    appDelegate = nil;
  }
  // App delegate has released the window.
  XCTAssertNil(weakWindow);
}

#pragma mark - Deep linking

- (void)testUniversalLinkPushRouteInformation {
  OCMStub([self.mockMainBundle objectForInfoDictionaryKey:@"FlutterDeepLinkingEnabled"])
      .andReturn(@YES);
  OCMStub([self.mockNavigationChannel
              invokeMethod:@"pushRouteInformation"
                 arguments:@{@"location" : @"http://myApp/custom/route?query=test"}])
      .andReturn(@YES);
  NSUserActivity* userActivity = [[NSUserActivity alloc] initWithActivityType:@"com.example.test"];
  userActivity.webpageURL = [NSURL URLWithString:@"http://myApp/custom/route?query=test"];
  BOOL result = [self.appDelegate
               application:[UIApplication sharedApplication]
      continueUserActivity:userActivity
        restorationHandler:^(NSArray<id<UIUserActivityRestoring>>* __nullable restorableObjects){
        }];
  XCTAssertTrue(result);
  OCMVerifyAll(self.mockNavigationChannel);
}

@end

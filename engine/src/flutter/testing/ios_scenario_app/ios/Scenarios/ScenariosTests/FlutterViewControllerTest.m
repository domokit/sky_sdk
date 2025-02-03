// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Flutter/Flutter.h>
#import <XCTest/XCTest.h>

#import "AppDelegate.h"

FLUTTER_ASSERT_ARC

@interface FlutterViewControllerTest : XCTestCase
@property(nonatomic, strong) FlutterViewController* flutterViewController;
@end

@implementation FlutterViewControllerTest

- (void)setUp {
  [super setUp];
  self.continueAfterFailure = NO;
}

- (void)tearDown {
  if (self.flutterViewController) {
    XCTestExpectation* vcDismissed = [self expectationWithDescription:@"dismiss"];
    [self.flutterViewController dismissViewControllerAnimated:NO
                                                   completion:^{
                                                     [vcDismissed fulfill];
                                                   }];
    [self waitForExpectationsWithTimeout:10.0 handler:nil];
  }
  [super tearDown];
}

- (void)testFirstFrameCallback {
  XCTestExpectation* firstFrameRendered = [self expectationWithDescription:@"firstFrameRendered"];

  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test" project:nil];
  [engine runWithEntrypoint:nil];
  self.flutterViewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                     nibName:nil
                                                                      bundle:nil];

  XCTAssertFalse(self.flutterViewController.isDisplayingFlutterUI);

  XCTestExpectation* displayingFlutterUIExpectation =
      [self keyValueObservingExpectationForObject:self.flutterViewController
                                          keyPath:@"displayingFlutterUI"
                                    expectedValue:@YES];
  displayingFlutterUIExpectation.assertForOverFulfill = YES;

  [self.flutterViewController setFlutterViewDidRenderCallback:^{
    [firstFrameRendered fulfill];
  }];

  AppDelegate* appDelegate = (AppDelegate*)UIApplication.sharedApplication.delegate;
  UIViewController* rootVC = appDelegate.window.rootViewController;
  [rootVC presentViewController:self.flutterViewController animated:NO completion:nil];

  [self waitForExpectationsWithTimeout:30.0 handler:nil];
}

- (void)testDrawLayer {
  XCTestExpectation* firstFrameRendered = [self expectationWithDescription:@"firstFrameRendered"];
  XCTestExpectation* imageRendered = [self expectationWithDescription:@"imageRendered"];

  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test" project:nil];
  [engine runWithEntrypoint:nil];
  [engine.binaryMessenger
      setMessageHandlerOnChannel:@"waiting_for_status"
            binaryMessageHandler:^(NSData* _Nullable message, FlutterBinaryReply _Nonnull reply) {
              FlutterMethodChannel* channel = [FlutterMethodChannel
                  methodChannelWithName:@"driver"
                        binaryMessenger:engine.binaryMessenger
                                  codec:[FlutterJSONMethodCodec sharedInstance]];
              [channel invokeMethod:@"set_scenario" arguments:@{@"name" : @"solid_blue"}];
            }];

  self.flutterViewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                     nibName:nil
                                                                      bundle:nil];

  XCTAssertFalse(self.flutterViewController.isDisplayingFlutterUI);

  [self.flutterViewController setFlutterViewDidRenderCallback:^{
    [firstFrameRendered fulfill];
  }];

  AppDelegate* appDelegate = (AppDelegate*)UIApplication.sharedApplication.delegate;
  UIViewController* rootVC = appDelegate.window.rootViewController;
  [rootVC presentViewController:self.flutterViewController animated:NO completion:nil];

  CGColorSpaceRef color_space = CGColorSpaceCreateDeviceRGB();
  dispatch_after(dispatch_time(DISPATCH_TIME_NOW, NSEC_PER_SEC), dispatch_get_main_queue(), ^{
    size_t width = 300u;
    CGContextRef context =
        CGBitmapContextCreate(nil, width, width, 8, 4 * width, color_space,
                              kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    [appDelegate.window.layer renderInContext:context];
    uint32_t* image_data = (uint32_t*)CGBitmapContextGetData(context);
    if (image_data[20] == 0xFF0000FF) {
      [imageRendered fulfill];
      return;
    }
    CGContextRelease(context);
  });

  [self waitForExpectationsWithTimeout:30.0 handler:nil];

  CGColorSpaceRelease(color_space);
}

@end

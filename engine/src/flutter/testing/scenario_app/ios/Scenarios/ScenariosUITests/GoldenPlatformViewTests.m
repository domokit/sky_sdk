// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "GoldenPlatformViewTests.h"

#include <sys/sysctl.h>
#import "GoldenTestManager.h"

static const NSInteger kSecondsToWaitForPlatformView = 30;

@interface GoldenPlatformViewTests ()

@property(nonatomic, copy) NSString* goldenName;

@property(nonatomic, strong) GoldenTestManager* manager;

@end

@implementation GoldenPlatformViewTests

- (instancetype)initWithManager:(GoldenTestManager*)manager invocation:(NSInvocation*)invocation {
  self = [super initWithInvocation:invocation];
  _manager = manager;
  _rmseThreadhold = kDefaultRmseThreshold;
  return self;
}

- (void)setUp {
  [super setUp];
  self.continueAfterFailure = NO;

  self.application = [[XCUIApplication alloc] init];
  self.application.launchArguments = @[ self.manager.launchArg ];
  [self.application launch];
}

// Note: don't prefix with "test" or GoldenPlatformViewTests will run instead of the subclasses.
- (void)checkPlatformViewGolden {
  XCUIElement* element = self.application.textViews.firstMatch;
  BOOL exists = [element waitForExistenceWithTimeout:kSecondsToWaitForPlatformView];
  if (!exists) {
    XCTFail(@"It took longer than %@ second to find the platform view."
            @"There might be issues with the platform view's construction,"
            @"or with how the scenario is built.",
            @(kSecondsToWaitForPlatformView));
  }

  [self.manager checkGoldenForTest:self rmesThreshold:self.rmseThreadhold];
}
@end

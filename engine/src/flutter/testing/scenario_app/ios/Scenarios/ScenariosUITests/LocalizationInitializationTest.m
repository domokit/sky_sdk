// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Flutter/Flutter.h>
#import <XCTest/XCTest.h>

FLUTTER_ASSERT_ARC

@interface LocalizationInitializationTest : XCTestCase
@property(nonatomic, strong) XCUIApplication* application;
@end

@implementation LocalizationInitializationTest

- (void)setUp {
  [super setUp];
  self.continueAfterFailure = NO;

  self.application = [[XCUIApplication alloc] init];
  self.application.launchArguments = @[ @"--locale-initialization" ];
  [self.application launch];
}

- (void)testNoLocalePrepend {
  NSTimeInterval timeout = 10.0;

  // The locales received by dart:ui are exposed onBeginFrame via semantics label.
  // There should only be one locale. The list should consist of the default
  // locale provided by the iOS app.
  NSArray<NSString*>* preferredLocales = [NSLocale preferredLanguages];
  XCTAssertEqual(preferredLocales.count, 1);
  // Dart connects the locale parts with `_` while iOS connects them with `-`.
  // Converts to dart format before comparing.
  NSString* localeDart = [preferredLocales.firstObject stringByReplacingOccurrencesOfString:@"-"
                                                                                 withString:@"_"];
  NSString* expectedIdentifier = [NSString stringWithFormat:@"[%@]", localeDart];
  XCUIElement* textInputSemanticsObject =
      [self.application.textFields matchingIdentifier:expectedIdentifier].element;
  XCTAssertTrue([textInputSemanticsObject waitForExistenceWithTimeout:timeout]);

  [textInputSemanticsObject tap];
}

@end

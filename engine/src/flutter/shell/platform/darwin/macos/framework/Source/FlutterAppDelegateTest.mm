// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterAppDelegate.h"

#import "flutter/testing/testing.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

@interface AppDelegateNoopFlutterAppLifecycleDelegate : NSObject <FlutterAppLifecycleDelegate>
@property(nonatomic, copy, nullable) NSArray<NSURL*>* receivedURLs;
@end

@implementation AppDelegateNoopFlutterAppLifecycleDelegate
@end

@interface AppDelegateTestFlutterAppLifecycleDelegate : NSObject <FlutterAppLifecycleDelegate>
@property(nonatomic, copy, nullable) NSArray<NSURL*>* receivedURLs;
@end

@implementation AppDelegateTestFlutterAppLifecycleDelegate

- (BOOL)handleOpenURLs:(NSArray<NSURL*>*)urls {
  self.receivedURLs = [urls copy];
  return YES;
}

@end

namespace flutter::testing {

TEST(FlutterAppDelegateTest, DoesNotCallDelegatesWithoutHandler) {
  FlutterAppDelegate* appDelegate = [[FlutterAppDelegate alloc] init];
  AppDelegateNoopFlutterAppLifecycleDelegate* noopDelegate =
      [[AppDelegateNoopFlutterAppLifecycleDelegate alloc] init];
  [appDelegate addApplicationLifecycleDelegate:noopDelegate];

  [appDelegate application:NSApplication.sharedApplication openURLs:@[]];
  // No EXPECT, since the test is that the call doesn't throw due to calling without checking that
  // the method is implemented.
}

TEST(FlutterAppDelegateTest, ReceivesOpenURLs) {
  FlutterAppDelegate* appDelegate = [[FlutterAppDelegate alloc] init];
  AppDelegateTestFlutterAppLifecycleDelegate* delegate =
      [[AppDelegateTestFlutterAppLifecycleDelegate alloc] init];
  [appDelegate addApplicationLifecycleDelegate:delegate];

  NSURL* URL = [NSURL URLWithString:@"https://flutter.dev"];
  EXPECT_NE(URL, nil);
  NSArray<NSURL*>* URLs = @[ URL ];
  [appDelegate application:NSApplication.sharedApplication openURLs:URLs];

  EXPECT_EQ([delegate receivedURLs], URLs);
}

TEST(FlutterAppDelegateTest, OperURLsStopsAfterHandled) {
  FlutterAppDelegate* appDelegate = [[FlutterAppDelegate alloc] init];
  AppDelegateTestFlutterAppLifecycleDelegate* firstDelegate =
      [[AppDelegateTestFlutterAppLifecycleDelegate alloc] init];
  AppDelegateTestFlutterAppLifecycleDelegate* secondDelegate =
      [[AppDelegateTestFlutterAppLifecycleDelegate alloc] init];
  [appDelegate addApplicationLifecycleDelegate:firstDelegate];
  [appDelegate addApplicationLifecycleDelegate:secondDelegate];

  NSURL* URL = [NSURL URLWithString:@"https://flutter.dev"];
  EXPECT_NE(URL, nil);
  NSArray<NSURL*>* URLs = @[ URL ];
  [appDelegate application:NSApplication.sharedApplication openURLs:URLs];

  EXPECT_EQ([firstDelegate receivedURLs], URLs);
  EXPECT_EQ([secondDelegate receivedURLs], nil);
}

}  // namespace flutter::testing

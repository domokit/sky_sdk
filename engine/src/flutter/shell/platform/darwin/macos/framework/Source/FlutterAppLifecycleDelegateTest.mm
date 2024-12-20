// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterAppLifecycleDelegate.h"

#import "flutter/testing/testing.h"
#include "third_party/googletest/googletest/include/gtest/gtest.h"

@interface TestFlutterAppLifecycleDelegate : NSObject <FlutterAppLifecycleDelegate>
@property(nonatomic, readwrite, nullable) NSNotification* lastNotification;
@end

@implementation TestFlutterAppLifecycleDelegate

- (void)setNotification:(NSNotification*)notification {
  self.lastNotification = notification;
}

- (void)handleWillFinishLaunching:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleDidFinishLaunching:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleWillBecomeActive:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleDidBecomeActive:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleWillResignActive:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleDidResignActive:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleWillHide:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleDidHide:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleWillUnhide:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleDidUnhide:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleDidChangeScreenParameters:(NSNotification*)notification {
  [self setNotification:notification];
}

- (void)handleDidChangeOcclusionState:(NSNotification*)notification API_AVAILABLE(macos(10.9)) {
  [self setNotification:notification];
}

- (void)handleWillTerminate:(NSNotification*)notification {
  [self setNotification:notification];
}

@end

namespace flutter::testing {

TEST(FlutterAppLifecycleDelegateTest, RespondsToWillFinishLaunching) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* willFinishLaunching =
      [NSNotification notificationWithName:NSApplicationWillFinishLaunchingNotification object:nil];
  [registrar handleWillFinishLaunching:willFinishLaunching];
  EXPECT_EQ([delegate lastNotification], willFinishLaunching);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToDidFinishLaunching) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* didFinishLaunching =
      [NSNotification notificationWithName:NSApplicationDidFinishLaunchingNotification object:nil];
  [registrar handleDidFinishLaunching:didFinishLaunching];
  EXPECT_EQ([delegate lastNotification], didFinishLaunching);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToWillBecomeActive) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* willBecomeActive =
      [NSNotification notificationWithName:NSApplicationWillBecomeActiveNotification object:nil];
  [registrar handleWillBecomeActive:willBecomeActive];
  EXPECT_EQ([delegate lastNotification], willBecomeActive);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToDidBecomeActive) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* didBecomeActive =
      [NSNotification notificationWithName:NSApplicationDidBecomeActiveNotification object:nil];
  [registrar handleDidBecomeActive:didBecomeActive];
  EXPECT_EQ([delegate lastNotification], didBecomeActive);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToWillResignActive) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* willResignActive =
      [NSNotification notificationWithName:NSApplicationWillResignActiveNotification object:nil];
  [registrar handleWillResignActive:willResignActive];
  EXPECT_EQ([delegate lastNotification], willResignActive);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToDidResignActive) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* didResignActive =
      [NSNotification notificationWithName:NSApplicationDidResignActiveNotification object:nil];
  [registrar handleDidResignActive:didResignActive];
  EXPECT_EQ([delegate lastNotification], didResignActive);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToWillTerminate) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* applicationWillTerminate =
      [NSNotification notificationWithName:NSApplicationWillTerminateNotification object:nil];
  [registrar handleWillTerminate:applicationWillTerminate];
  EXPECT_EQ([delegate lastNotification], applicationWillTerminate);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToWillHide) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* willHide = [NSNotification notificationWithName:NSApplicationWillHideNotification
                                                           object:nil];
  [registrar handleWillHide:willHide];
  EXPECT_EQ([delegate lastNotification], willHide);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToWillUnhide) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* willUnhide =
      [NSNotification notificationWithName:NSApplicationWillUnhideNotification object:nil];
  [registrar handleWillUnhide:willUnhide];
  EXPECT_EQ([delegate lastNotification], willUnhide);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToDidHide) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* didHide = [NSNotification notificationWithName:NSApplicationDidHideNotification
                                                          object:nil];
  [registrar handleDidHide:didHide];
  EXPECT_EQ([delegate lastNotification], didHide);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToDidUnhide) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* didUnhide =
      [NSNotification notificationWithName:NSApplicationDidUnhideNotification object:nil];
  [registrar handleDidUnhide:didUnhide];
  EXPECT_EQ([delegate lastNotification], didUnhide);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToDidChangeScreenParameters) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* didChangeScreenParameters =
      [NSNotification notificationWithName:NSApplicationDidChangeScreenParametersNotification
                                    object:nil];
  [registrar handleDidChangeScreenParameters:didChangeScreenParameters];
  EXPECT_EQ([delegate lastNotification], didChangeScreenParameters);
}

TEST(FlutterAppLifecycleDelegateTest, RespondsToDidChangeOcclusionState) {
  FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
  TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
  [registrar addDelegate:delegate];

  NSNotification* didChangeOcclusionState =
      [NSNotification notificationWithName:NSApplicationDidChangeOcclusionStateNotification
                                    object:nil];
  if ([registrar respondsToSelector:@selector(handleDidChangeOcclusionState:)]) {
    [registrar handleDidChangeOcclusionState:didChangeOcclusionState];
    EXPECT_EQ([delegate lastNotification], didChangeOcclusionState);
  }
}

TEST(FlutterAppLifecycleDelegateTest, ReleasesDelegateOnDealloc) {
  __weak FlutterAppLifecycleRegistrar* weakRegistrar;
  __weak TestFlutterAppLifecycleDelegate* weakDelegate;
  @autoreleasepool {
    FlutterAppLifecycleRegistrar* registrar = [[FlutterAppLifecycleRegistrar alloc] init];
    weakRegistrar = registrar;
    TestFlutterAppLifecycleDelegate* delegate = [[TestFlutterAppLifecycleDelegate alloc] init];
    weakDelegate = delegate;
    [registrar addDelegate:delegate];
  }
  EXPECT_EQ(weakRegistrar, nil);
  EXPECT_EQ(weakDelegate, nil);
}

}  // namespace flutter::testing

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterRestorationPlugin.h"

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

#include "flutter/fml/logging.h"

FLUTTER_ASSERT_ARC

@interface FlutterRestorationPlugin ()
@property(nonatomic, copy) FlutterResult pendingRequest;
@end

@implementation FlutterRestorationPlugin {
  BOOL _waitForData;
  BOOL _restorationEnabled;
}

- (instancetype)initWithChannel:(FlutterMethodChannel*)channel
             restorationEnabled:(BOOL)restorationEnabled {
  FML_DCHECK(channel) << "channel must be set";
  self = [super init];
  if (self) {
    [channel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
      [self handleMethodCall:call result:result];
    }];
    _restorationEnabled = restorationEnabled;
    _waitForData = restorationEnabled;
  }
  return self;
}

- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result {
  if ([[call method] isEqualToString:@"put"]) {
    NSAssert(self.pendingRequest == nil, @"Cannot put data while a get request is pending.");
    FlutterStandardTypedData* data = [call arguments];
    self.restorationData = [data data];
    result(nil);
  } else if ([[call method] isEqualToString:@"get"]) {
    if (!_restorationEnabled || !_waitForData) {
      result([self dataForFramework]);
      return;
    }
    NSAssert(self.pendingRequest == nil, @"There can only be one pending request.");
    self.pendingRequest = result;
  } else {
    result(FlutterMethodNotImplemented);
  }
}

- (void)setRestorationData:(NSData*)data {
  if (data != _restorationData) {
    _restorationData = [data copy];
  }
  _waitForData = NO;
  if (self.pendingRequest != nil) {
    self.pendingRequest([self dataForFramework]);
    self.pendingRequest = nil;
  }
}

- (void)markRestorationComplete {
  _waitForData = NO;
  if (self.pendingRequest != nil) {
    NSAssert(_restorationEnabled, @"No request can be pending when restoration is disabled.");
    self.pendingRequest([self dataForFramework]);
    self.pendingRequest = nil;
  }
}

- (void)reset {
  self.pendingRequest = nil;
  self.restorationData = nil;
}

- (NSDictionary*)dataForFramework {
  if (!_restorationEnabled) {
    return @{@"enabled" : @NO};
  }
  if (self.restorationData == nil) {
    return @{@"enabled" : @YES};
  }
  return @{
    @"enabled" : @YES,
    @"data" : [FlutterStandardTypedData typedDataWithBytes:self.restorationData]
  };
}

@end

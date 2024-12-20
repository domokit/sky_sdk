// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "FlutterEngine+ScenariosTest.h"

@implementation FlutterEngine (ScenariosTest)

- (instancetype)initWithScenario:(NSString*)scenario
                  withCompletion:(nullable void (^)(void))engineRunCompletion {
  NSAssert([scenario length] != 0, @"You need to provide a scenario");
  self = [self initWithName:[NSString stringWithFormat:@"Test engine for %@", scenario]
                    project:nil];
  [self run];

  [self.binaryMessenger setMessageHandlerOnChannel:@"waiting_for_status"
                              binaryMessageHandler:^(NSData* message, FlutterBinaryReply reply) {
                                FlutterMethodChannel* channel = [FlutterMethodChannel
                                    methodChannelWithName:@"driver"
                                          binaryMessenger:self.binaryMessenger
                                                    codec:[FlutterJSONMethodCodec sharedInstance]];
                                [channel invokeMethod:@"set_scenario"
                                            arguments:@{@"name" : scenario}];

                                if (engineRunCompletion != nil) {
                                  engineRunCompletion();
                                }
                              }];
  return self;
}

@end
